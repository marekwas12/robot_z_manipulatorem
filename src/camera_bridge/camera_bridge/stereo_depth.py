import os

import cv2
import numpy as np
import yaml

import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import (
    Image,
    CompressedImage,
)

from std_msgs.msg import Float32

import message_filters


# ============================================================
# INPUT
# ============================================================

LEFT_TOPIC = (
    "/camera_left/left_camera/image_raw"
)

RIGHT_TOPIC = (
    "/camera_right/right_camera/image_raw"
)


# ============================================================
# OUTPUT
# ============================================================

LEFT_RECTIFIED_TOPIC = (
    "/stereo/left_rectified"
)

POINTS_TOPIC = (
    "/stereo/points"
)


# ============================================================
# FRAME
# ============================================================

OPTICAL_FRAME = (
    "stereo_camera_optical"
)


# ============================================================
# KALIBRACJA
# ============================================================

CALIBRATION_DIR = os.path.expanduser(
    "~/robot_ws/stereo_calibration_v3"
)

LEFT_YAML = os.path.join(
    CALIBRATION_DIR,
    "left.yaml",
)

RIGHT_YAML = os.path.join(
    CALIBRATION_DIR,
    "right.yaml",
)

STEREO_YAML = os.path.join(
    CALIBRATION_DIR,
    "stereo.yaml",
)


# ============================================================
# SYNCHRONIZACJA
# ============================================================

SYNC_QUEUE = 30
SYNC_SLOP_SEC = 0.10


# ============================================================
# SGBM
# ============================================================

MIN_DISPARITY = 0
NUM_DISPARITIES = 128
BLOCK_SIZE = 5


# ============================================================
# DISPLAY
# ============================================================

DISPLAY_FLIP_CODE = -1


class StereoDepth(Node):

    def __init__(self):

        super().__init__(
            "stereo_depth"
        )

        self.maps_ready = False

        self.image_size = None

        self.left_map_x = None
        self.left_map_y = None

        self.right_map_x = None
        self.right_map_y = None


        # ====================================================
        # CALIBRATION
        # ====================================================

        self.load_calibration()


        # ====================================================
        # SGBM
        # ====================================================

        self.stereo = (
            cv2.StereoSGBM_create(

                minDisparity=
                    MIN_DISPARITY,

                numDisparities=
                    NUM_DISPARITIES,

                blockSize=
                    BLOCK_SIZE,

                P1=
                    8
                    *
                    BLOCK_SIZE
                    *
                    BLOCK_SIZE,

                P2=
                    32
                    *
                    BLOCK_SIZE
                    *
                    BLOCK_SIZE,

                disp12MaxDiff=1,

                uniquenessRatio=10,

                speckleWindowSize=100,

                speckleRange=2,

                preFilterCap=31,

                mode=
                    cv2.STEREO_SGBM_MODE_SGBM_3WAY,
            )
        )


        # ====================================================
        # EXISTING OUTPUTS
        # ====================================================

        self.cameras_pub = (
            self.create_publisher(
                CompressedImage,
                "/stereo/rectified_check/compressed",
                5,
            )
        )

        self.rectified_debug_pub = (
            self.create_publisher(
                CompressedImage,
                "/stereo/rectified_debug/compressed",
                5,
            )
        )

        self.depth_pub = (
            self.create_publisher(
                CompressedImage,
                "/stereo/depth/compressed",
                5,
            )
        )

        self.preview_pub = (
            self.create_publisher(
                CompressedImage,
                "/stereo/preview/compressed",
                5,
            )
        )

        self.distance_pub = (
            self.create_publisher(
                Float32,
                "/stereo/center_distance",
                10,
            )
        )


        # ====================================================
        # NEW MACHINE-READABLE OUTPUTS
        # ====================================================

        self.left_rect_pub = (
            self.create_publisher(
                Image,
                LEFT_RECTIFIED_TOPIC,
                qos_profile_sensor_data,
            )
        )

        self.points_pub = (
            self.create_publisher(
                Image,
                POINTS_TOPIC,
                qos_profile_sensor_data,
            )
        )


        # ====================================================
        # INPUT
        # ====================================================

        self.left_sub = (
            message_filters.Subscriber(
                self,
                Image,
                LEFT_TOPIC,
                qos_profile=
                    qos_profile_sensor_data,
            )
        )

        self.right_sub = (
            message_filters.Subscriber(
                self,
                Image,
                RIGHT_TOPIC,
                qos_profile=
                    qos_profile_sensor_data,
            )
        )

        self.sync = (
            message_filters
            .ApproximateTimeSynchronizer(

                [
                    self.left_sub,
                    self.right_sub,
                ],

                queue_size=
                    SYNC_QUEUE,

                slop=
                    SYNC_SLOP_SEC,
            )
        )

        self.sync.registerCallback(
            self.stereo_callback
        )


        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "STEREO DEPTH + XYZ START"
        )

        self.get_logger().info(
            f"Kalibracja: {CALIBRATION_DIR}"
        )

        self.get_logger().info(
            f"Optical frame: {OPTICAL_FRAME}"
        )

        self.get_logger().info(
            "Publikuje:"
        )

        self.get_logger().info(
            LEFT_RECTIFIED_TOPIC
        )

        self.get_logger().info(
            POINTS_TOPIC
        )

        self.get_logger().info(
            "=========================================="
        )


    # ========================================================
    # YAML
    # ========================================================

    def read_yaml(
        self,
        path,
    ):

        if not os.path.exists(
            path
        ):

            raise RuntimeError(
                f"Brak pliku: {path}"
            )

        with open(
            path,
            "r",
        ) as file:

            return yaml.safe_load(
                file
            )


    def matrix_from_yaml(
        self,
        section,
    ):

        rows = int(
            section["rows"]
        )

        cols = int(
            section["cols"]
        )

        data = np.array(
            section["data"],
            dtype=np.float64,
        )

        return data.reshape(
            rows,
            cols,
        )


    # ========================================================
    # CALIBRATION
    # ========================================================

    def load_calibration(
        self,
    ):

        left = self.read_yaml(
            LEFT_YAML
        )

        right = self.read_yaml(
            RIGHT_YAML
        )

        stereo = self.read_yaml(
            STEREO_YAML
        )


        width = int(
            left[
                "image_width"
            ]
        )

        height = int(
            left[
                "image_height"
            ]
        )

        self.image_size = (
            width,
            height,
        )


        self.left_k = (
            self.matrix_from_yaml(
                left[
                    "camera_matrix"
                ]
            )
        )

        self.left_d = (
            self.matrix_from_yaml(
                left[
                    "distortion_coefficients"
                ]
            )
        )

        self.left_r = (
            self.matrix_from_yaml(
                left[
                    "rectification_matrix"
                ]
            )
        )

        self.left_p = (
            self.matrix_from_yaml(
                left[
                    "projection_matrix"
                ]
            )
        )


        self.right_k = (
            self.matrix_from_yaml(
                right[
                    "camera_matrix"
                ]
            )
        )

        self.right_d = (
            self.matrix_from_yaml(
                right[
                    "distortion_coefficients"
                ]
            )
        )

        self.right_r = (
            self.matrix_from_yaml(
                right[
                    "rectification_matrix"
                ]
            )
        )

        self.right_p = (
            self.matrix_from_yaml(
                right[
                    "projection_matrix"
                ]
            )
        )


        self.q_matrix = (
            self.matrix_from_yaml(
                stereo[
                    "Q"
                ]
            )
        )


        (
            self.left_map_x,
            self.left_map_y,
        ) = (
            cv2.initUndistortRectifyMap(

                self.left_k,

                self.left_d,

                self.left_r,

                self.left_p[
                    :,
                    :3
                ],

                self.image_size,

                cv2.CV_32FC1,
            )
        )


        (
            self.right_map_x,
            self.right_map_y,
        ) = (
            cv2.initUndistortRectifyMap(

                self.right_k,

                self.right_d,

                self.right_r,

                self.right_p[
                    :,
                    :3
                ],

                self.image_size,

                cv2.CV_32FC1,
            )
        )


        self.maps_ready = True


        self.get_logger().info(
            f"Kalibracja wczytana: "
            f"{width}x{height}"
        )


    # ========================================================
    # ROS IMAGE -> BGR
    # ========================================================

    def ros_image_to_bgr(
        self,
        msg,
    ):

        height = int(
            msg.height
        )

        width = int(
            msg.width
        )

        step = int(
            msg.step
        )


        if msg.encoding not in (
            "bgr8",
            "rgb8",
        ):

            raise RuntimeError(
                f"Nieobslugiwane encoding: "
                f"{msg.encoding}"
            )


        expected_row_bytes = (
            width
            *
            3
        )


        raw = np.frombuffer(
            msg.data,
            dtype=np.uint8,
        )


        expected_total = (
            step
            *
            height
        )


        if raw.size < expected_total:

            raise RuntimeError(
                "Za malo danych obrazu."
            )


        rows = (
            raw[
                :expected_total
            ]
            .reshape(
                height,
                step,
            )
        )


        pixels = rows[
            :,
            :expected_row_bytes
        ]


        image = pixels.reshape(
            height,
            width,
            3,
        )


        if msg.encoding == "rgb8":

            image = (
                cv2.cvtColor(
                    image,
                    cv2.COLOR_RGB2BGR,
                )
            )


        return np.ascontiguousarray(
            image
        )


    # ========================================================
    # RAW BGR IMAGE
    # ========================================================

    def publish_bgr_image(
        self,
        publisher,
        image,
        source_msg,
    ):

        image = np.ascontiguousarray(
            image,
            dtype=np.uint8,
        )

        msg = Image()

        msg.header = (
            source_msg.header
        )

        msg.header.frame_id = (
            OPTICAL_FRAME
        )

        msg.height = int(
            image.shape[0]
        )

        msg.width = int(
            image.shape[1]
        )

        msg.encoding = "bgr8"

        msg.is_bigendian = False

        msg.step = int(
            image.shape[1]
            *
            3
        )

        msg.data = (
            image.tobytes()
        )

        publisher.publish(
            msg
        )


    # ========================================================
    # XYZ IMAGE
    # ========================================================

    def publish_xyz_image(
        self,
        points_3d,
        source_msg,
    ):

        points = np.ascontiguousarray(
            points_3d,
            dtype=np.float32,
        )


        msg = Image()

        msg.header = (
            source_msg.header
        )

        msg.header.frame_id = (
            OPTICAL_FRAME
        )

        msg.height = int(
            points.shape[0]
        )

        msg.width = int(
            points.shape[1]
        )

        msg.encoding = "32FC3"

        msg.is_bigendian = False

        msg.step = int(
            points.shape[1]
            *
            3
            *
            4
        )

        msg.data = (
            points.tobytes()
        )

        self.points_pub.publish(
            msg
        )


    # ========================================================
    # JPEG
    # ========================================================

    def publish_compressed(
        self,
        publisher,
        image,
        source_msg,
    ):

        success, encoded = (
            cv2.imencode(

                ".jpg",

                image,

                [
                    int(
                        cv2.IMWRITE_JPEG_QUALITY
                    ),
                    80,
                ],
            )
        )


        if not success:
            return


        msg = CompressedImage()

        msg.header = (
            source_msg.header
        )

        msg.header.frame_id = (
            OPTICAL_FRAME
        )

        msg.format = "jpeg"

        msg.data = (
            encoded.tobytes()
        )

        publisher.publish(
            msg
        )


    # ========================================================
    # CENTER DISTANCE
    # ========================================================

    def center_distance(
        self,
        points_3d,
        disparity,
    ):

        height, width = (
            disparity.shape
        )

        cx = (
            width // 2
        )

        cy = (
            height // 2
        )

        radius = 10

        x1 = max(
            0,
            cx - radius,
        )

        x2 = min(
            width,
            cx + radius + 1,
        )

        y1 = max(
            0,
            cy - radius,
        )

        y2 = min(
            height,
            cy + radius + 1,
        )


        disparity_roi = disparity[
            y1:y2,
            x1:x2,
        ]


        points_roi = points_3d[
            y1:y2,
            x1:x2,
            :
        ]


        z = points_roi[
            :,
            :,
            2
        ]


        valid = (
            np.isfinite(
                z
            )
            &
            (
                z > 0.05
            )
            &
            (
                z < 10.0
            )
            &
            (
                disparity_roi > 1.0
            )
        )


        values = z[
            valid
        ]


        if values.size < 10:

            return None


        return float(
            np.median(
                values
            )
        )


    # ========================================================
    # CALLBACK
    # ========================================================

    def stereo_callback(
        self,
        left_msg,
        right_msg,
    ):

        if not self.maps_ready:
            return


        try:

            left = (
                self.ros_image_to_bgr(
                    left_msg
                )
            )

            right = (
                self.ros_image_to_bgr(
                    right_msg
                )
            )

        except Exception as exc:

            self.get_logger().error(
                f"Konwersja obrazu: {exc}"
            )

            return


        if (
            left.shape[1],
            left.shape[0],
        ) != self.image_size:

            self.get_logger().error(
                "Rozdzielczosc lewej kamery "
                "nie pasuje do kalibracji."
            )

            return


        if (
            right.shape[1],
            right.shape[0],
        ) != self.image_size:

            self.get_logger().error(
                "Rozdzielczosc prawej kamery "
                "nie pasuje do kalibracji."
            )

            return


        # ====================================================
        # RAW DISPLAY PAIR
        # ====================================================

        left_display = (
            cv2.flip(
                left,
                DISPLAY_FLIP_CODE,
            )
        )

        right_display = (
            cv2.flip(
                right,
                DISPLAY_FLIP_CODE,
            )
        )

        cameras = np.hstack(
            (
                left_display,
                right_display,
            )
        )

        self.publish_compressed(
            self.cameras_pub,
            cameras,
            right_msg,
        )


        # ====================================================
        # RECTIFICATION
        # ====================================================

        left_rect = (
            cv2.remap(

                left,

                self.left_map_x,

                self.left_map_y,

                cv2.INTER_LINEAR,
            )
        )


        right_rect = (
            cv2.remap(

                right,

                self.right_map_x,

                self.right_map_y,

                cv2.INTER_LINEAR,
            )
        )


        # Machine-readable rectified left.
        self.publish_bgr_image(
            self.left_rect_pub,
            left_rect,
            left_msg,
        )


        # ====================================================
        # RECTIFIED DEBUG
        # ====================================================

        left_debug = (
            cv2.flip(
                left_rect,
                DISPLAY_FLIP_CODE,
            )
        )

        right_debug = (
            cv2.flip(
                right_rect,
                DISPLAY_FLIP_CODE,
            )
        )

        debug = np.hstack(
            (
                left_debug,
                right_debug,
            )
        )


        debug_h, debug_w = (
            debug.shape[:2]
        )


        for y in range(
            20,
            debug_h,
            20,
        ):

            cv2.line(
                debug,
                (
                    0,
                    y,
                ),
                (
                    debug_w - 1,
                    y,
                ),
                (
                    0,
                    255,
                    0,
                ),
                1,
            )


        cv2.line(
            debug,
            (
                left_debug.shape[1],
                0,
            ),
            (
                left_debug.shape[1],
                debug_h - 1,
            ),
            (
                0,
                0,
                255,
            ),
            1,
        )


        self.publish_compressed(
            self.rectified_debug_pub,
            debug,
            right_msg,
        )


        # ====================================================
        # GRAY
        # ====================================================

        left_gray = (
            cv2.cvtColor(
                left_rect,
                cv2.COLOR_BGR2GRAY,
            )
        )

        right_gray = (
            cv2.cvtColor(
                right_rect,
                cv2.COLOR_BGR2GRAY,
            )
        )


        # ====================================================
        # DISPARITY
        # ====================================================

        disparity_raw = (
            self.stereo.compute(
                left_gray,
                right_gray,
            )
        )


        disparity = (
            disparity_raw.astype(
                np.float32
            )
            /
            16.0
        )


        # ====================================================
        # XYZ
        # ====================================================

        points_3d = (
            cv2.reprojectImageTo3D(

                disparity,

                self.q_matrix,

                handleMissingValues=True,
            )
        )


        points_3d = (
            points_3d.astype(
                np.float32
            )
        )


        # ====================================================
        # INVALID -> NaN
        # ====================================================

        invalid = (
            ~np.isfinite(
                points_3d[
                    :,
                    :,
                    2
                ]
            )
            |
            (
                disparity <= 1.0
            )
            |
            (
                points_3d[
                    :,
                    :,
                    2
                ]
                <= 0.05
            )
            |
            (
                points_3d[
                    :,
                    :,
                    2
                ]
                > 10.0
            )
        )


        points_3d[
            invalid
        ] = np.nan


        self.publish_xyz_image(
            points_3d,
            left_msg,
        )


        distance = (
            self.center_distance(
                points_3d,
                disparity,
            )
        )


        if distance is not None:

            distance_msg = (
                Float32()
            )

            distance_msg.data = (
                distance
            )

            self.distance_pub.publish(
                distance_msg
            )


        # ====================================================
        # PREVIEW
        # ====================================================

        preview = (
            cv2.flip(
                right_rect,
                DISPLAY_FLIP_CODE,
            )
        )


        height, width = (
            preview.shape[:2]
        )


        cx = (
            width // 2
        )

        cy = (
            height // 2
        )


        cv2.rectangle(
            preview,
            (
                cx - 10,
                cy - 10,
            ),
            (
                cx + 10,
                cy + 10,
            ),
            (
                0,
                255,
                0,
            ),
            1,
        )


        if distance is not None:

            text = (
                f"Distance: "
                f"{distance:.3f} m"
            )

        else:

            text = (
                "Distance: ---"
            )


        pass


        self.publish_compressed(
            self.preview_pub,
            preview,
            right_msg,
        )


        # ====================================================
        # COLOR DEPTH
        # ====================================================

        valid_disp = (
            disparity.copy()
        )

        valid_disp[
            valid_disp < 0
        ] = 0


        normalized = (
            np.clip(
                valid_disp
                /
                float(
                    NUM_DISPARITIES
                )
                *
                255.0,
                0,
                255,
            )
            .astype(
                np.uint8
            )
        )


        depth_color = (
            cv2.applyColorMap(
                normalized,
                cv2.COLORMAP_TURBO,
            )
        )


        depth_color = (
            cv2.flip(
                depth_color,
                DISPLAY_FLIP_CODE,
            )
        )


        depth_h, depth_w = (
            depth_color.shape[:2]
        )


        depth_cx = (
            depth_w // 2
        )

        depth_cy = (
            depth_h // 2
        )


        cv2.rectangle(
            depth_color,
            (
                depth_cx - 10,
                depth_cy - 10,
            ),
            (
                depth_cx + 10,
                depth_cy + 10,
            ),
            (
                255,
                255,
                255,
            ),
            1,
        )


        if distance is not None:

            depth_text = (
                f"Distance: "
                f"{distance:.3f} m"
            )

        else:

            depth_text = (
                "Distance: ---"
            )


        pass


        self.publish_compressed(
            self.depth_pub,
            depth_color,
            right_msg,
        )


def main(
    args=None,
):

    rclpy.init(
        args=args
    )

    node = (
        StereoDepth()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()