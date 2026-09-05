import json
import math

import cv2
import numpy as np

import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import (
    Image,
    CompressedImage,
)

from geometry_msgs.msg import (
    PointStamped,
)

from std_msgs.msg import (
    Bool,
    String,
)

import message_filters

import tf2_ros


# ============================================================
# INPUT
# ============================================================

IMAGE_TOPIC = (
    "/stereo/left_rectified"
)

POINTS_TOPIC = (
    "/stereo/points"
)


# ============================================================
# OUTPUT
# ============================================================

DETECTED_TOPIC = (
    "/cube_detector/detected"
)

TYPE_TOPIC = (
    "/cube_detector/type"
)

CAMERA_POINT_TOPIC = (
    "/cube_detector/position_camera"
)

BASE_POINT_TOPIC = (
    "/cube_detector/position_base"
)

DEBUG_TOPIC = (
    "/cube_detector/debug/compressed"
)

INFO_TOPIC = (
    "/cube_detector/info"
)


# ============================================================
# TF
# ============================================================

CAMERA_FRAME = (
    "stereo_camera_optical"
)

BASE_FRAME = (
    "base_link"
)


# ============================================================
# CUBE
# ============================================================

EXPECTED_SIZE_M = (
    0.070
)

SIZE_TOLERANCE_M = (
    0.035
)


# ============================================================
# IMAGE FILTERING
# ============================================================

MIN_CONTOUR_AREA = (
    150
)

MAX_CONTOUR_AREA = (
    30000
)

MIN_VALID_POINTS = (
    25
)


class CubeDetector(Node):

    def __init__(self):

        super().__init__(
            "cube_detector"
        )


        # ====================================================
        # TF
        # ====================================================

        self.tf_buffer = (
            tf2_ros.Buffer()
        )

        self.tf_listener = (
            tf2_ros.TransformListener(
                self.tf_buffer,
                self,
            )
        )


        # ====================================================
        # PUBLISHERS
        # ====================================================

        self.detected_pub = (
            self.create_publisher(
                Bool,
                DETECTED_TOPIC,
                10,
            )
        )

        self.type_pub = (
            self.create_publisher(
                String,
                TYPE_TOPIC,
                10,
            )
        )

        self.camera_point_pub = (
            self.create_publisher(
                PointStamped,
                CAMERA_POINT_TOPIC,
                10,
            )
        )

        self.base_point_pub = (
            self.create_publisher(
                PointStamped,
                BASE_POINT_TOPIC,
                10,
            )
        )

        self.debug_pub = (
            self.create_publisher(
                CompressedImage,
                DEBUG_TOPIC,
                5,
            )
        )

        self.info_pub = (
            self.create_publisher(
                String,
                INFO_TOPIC,
                10,
            )
        )


        # ====================================================
        # SUBSCRIBERS
        # ====================================================

        self.image_sub = (
            message_filters.Subscriber(
                self,
                Image,
                IMAGE_TOPIC,
                qos_profile=
                    qos_profile_sensor_data,
            )
        )

        self.points_sub = (
            message_filters.Subscriber(
                self,
                Image,
                POINTS_TOPIC,
                qos_profile=
                    qos_profile_sensor_data,
            )
        )


        self.sync = (
            message_filters
            .ApproximateTimeSynchronizer(

                [
                    self.image_sub,
                    self.points_sub,
                ],

                queue_size=10,

                slop=0.10,
            )
        )


        self.sync.registerCallback(
            self.callback
        )


        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "CUBE DETECTOR"
        )

        self.get_logger().info(
            "Szukam szescianu ~70 mm"
        )

        self.get_logger().info(
            f"Camera frame: {CAMERA_FRAME}"
        )

        self.get_logger().info(
            f"Base frame: {BASE_FRAME}"
        )

        self.get_logger().info(
            "=========================================="
        )


    # ========================================================
    # IMAGE DECODING
    # ========================================================

    def bgr_from_ros(
        self,
        msg,
    ):

        if msg.encoding != "bgr8":

            raise RuntimeError(
                f"Expected bgr8, "
                f"got {msg.encoding}"
            )


        raw = (
            np.frombuffer(
                msg.data,
                dtype=np.uint8,
            )
        )


        image = (
            raw.reshape(
                msg.height,
                msg.width,
                3,
            )
        )


        return (
            np.ascontiguousarray(
                image
            )
        )


    def xyz_from_ros(
        self,
        msg,
    ):

        if msg.encoding != "32FC3":

            raise RuntimeError(
                f"Expected 32FC3, "
                f"got {msg.encoding}"
            )


        raw = (
            np.frombuffer(
                msg.data,
                dtype=np.float32,
            )
        )


        xyz = (
            raw.reshape(
                msg.height,
                msg.width,
                3,
            )
        )


        return (
            np.ascontiguousarray(
                xyz
            )
        )


    # ========================================================
    # DEBUG JPEG
    # ========================================================

    def publish_debug(
        self,
        image,
        source_msg,
    ):

        ok, encoded = (
            cv2.imencode(
                ".jpg",
                image,
                [
                    int(
                        cv2.IMWRITE_JPEG_QUALITY
                    ),
                    85,
                ],
            )
        )


        if not ok:
            return


        msg = (
            CompressedImage()
        )

        msg.header = (
            source_msg.header
        )

        msg.header.frame_id = (
            CAMERA_FRAME
        )

        msg.format = "jpeg"

        msg.data = (
            encoded.tobytes()
        )

        self.debug_pub.publish(
            msg
        )


    # ========================================================
    # DETECTION STATE
    # ========================================================

    def publish_not_detected(
        self,
    ):

        detected = (
            Bool()
        )

        detected.data = False

        self.detected_pub.publish(
            detected
        )


    # ========================================================
    # CONTOUR -> 3D
    # ========================================================

    def contour_3d_data(
        self,
        contour,
        xyz,
    ):

        mask = (
            np.zeros(
                xyz.shape[:2],
                dtype=np.uint8,
            )
        )


        cv2.drawContours(
            mask,
            [
                contour
            ],
            -1,
            255,
            thickness=-1,
        )


        points = (
            xyz[
                mask > 0
            ]
        )


        valid = (
            np.all(
                np.isfinite(
                    points
                ),
                axis=1,
            )
        )


        points = (
            points[
                valid
            ]
        )


        if (
            points.shape[0]
            <
            MIN_VALID_POINTS
        ):

            return None


        # Remove extreme stereo outliers.
        med = (
            np.median(
                points,
                axis=0,
            )
        )


        distance = (
            np.linalg.norm(
                points
                -
                med,
                axis=1,
            )
        )


        keep = (
            distance
            <
            np.percentile(
                distance,
                80,
            )
        )


        points = (
            points[
                keep
            ]
        )


        if (
            points.shape[0]
            <
            MIN_VALID_POINTS
        ):

            return None


        center = (
            np.median(
                points,
                axis=0,
            )
        )


        p10 = (
            np.percentile(
                points,
                10,
                axis=0,
            )
        )


        p90 = (
            np.percentile(
                points,
                90,
                axis=0,
            )
        )


        extents = (
            p90
            -
            p10
        )


        return (
            center,
            extents,
            points.shape[0],
        )


    # ========================================================
    # SCORE
    # ========================================================

    def cube_score(
        self,
        contour,
        extents,
    ):

        area = (
            cv2.contourArea(
                contour
            )
        )


        rect = (
            cv2.minAreaRect(
                contour
            )
        )


        width_px, height_px = (
            rect[1]
        )


        if (
            width_px < 1.0
            or
            height_px < 1.0
        ):

            return None


        ratio = (
            min(
                width_px,
                height_px,
            )
            /
            max(
                width_px,
                height_px,
            )
        )


        # Main visible size in camera XY.
        size_x = abs(
            float(
                extents[0]
            )
        )

        size_y = abs(
            float(
                extents[1]
            )
        )


        visible_size = max(
            size_x,
            size_y,
        )


        size_error = abs(
            visible_size
            -
            EXPECTED_SIZE_M
        )


        if (
            size_error
            >
            SIZE_TOLERANCE_M
        ):

            return None


        if ratio < 0.45:

            return None


        size_score = (
            1.0
            -
            min(
                1.0,
                size_error
                /
                SIZE_TOLERANCE_M,
            )
        )


        shape_score = (
            ratio
        )


        score = (
            0.65
            *
            size_score
            +
            0.35
            *
            shape_score
        )


        return (
            float(
                score
            )
        )


    # ========================================================
    # TF
    # ========================================================

    def transform_to_base(
        self,
        point_camera,
    ):

        try:

            transform = (
                self.tf_buffer
                .lookup_transform(
                    BASE_FRAME,
                    CAMERA_FRAME,
                    rclpy.time.Time(),
                )
            )

        except Exception as exc:

            self.get_logger().warn(
                f"Brak TF camera->base: "
                f"{exc}"
            )

            return None


        t = (
            transform.transform.translation
        )

        q = (
            transform.transform.rotation
        )


        # Quaternion rotation matrix.
        x = q.x
        y = q.y
        z = q.z
        w = q.w


        rotation = np.array(
            [
                [
                    1
                    -
                    2
                    *
                    (
                        y*y
                        +
                        z*z
                    ),

                    2
                    *
                    (
                        x*y
                        -
                        z*w
                    ),

                    2
                    *
                    (
                        x*z
                        +
                        y*w
                    ),
                ],

                [
                    2
                    *
                    (
                        x*y
                        +
                        z*w
                    ),

                    1
                    -
                    2
                    *
                    (
                        x*x
                        +
                        z*z
                    ),

                    2
                    *
                    (
                        y*z
                        -
                        x*w
                    ),
                ],

                [
                    2
                    *
                    (
                        x*z
                        -
                        y*w
                    ),

                    2
                    *
                    (
                        y*z
                        +
                        x*w
                    ),

                    1
                    -
                    2
                    *
                    (
                        x*x
                        +
                        y*y
                    ),
                ],
            ],
            dtype=np.float64,
        )


        camera = np.array(
            point_camera,
            dtype=np.float64,
        )


        translation = np.array(
            [
                t.x,
                t.y,
                t.z,
            ],
            dtype=np.float64,
        )


        return (
            rotation
            @
            camera
            +
            translation
        )


    # ========================================================
    # CALLBACK
    # ========================================================

    def callback(
        self,
        image_msg,
        points_msg,
    ):

        try:

            image = (
                self.bgr_from_ros(
                    image_msg
                )
            )

            xyz = (
                self.xyz_from_ros(
                    points_msg
                )
            )

        except Exception as exc:

            self.get_logger().error(
                str(
                    exc
                )
            )

            return


        debug = (
            image.copy()
        )


        # ====================================================
        # EDGE DETECTION
        # ====================================================

        gray = (
            cv2.cvtColor(
                image,
                cv2.COLOR_BGR2GRAY,
            )
        )


        gray = (
            cv2.GaussianBlur(
                gray,
                (
                    5,
                    5,
                ),
                0,
            )
        )


        edges = (
            cv2.Canny(
                gray,
                40,
                120,
            )
        )


        kernel = (
            np.ones(
                (
                    3,
                    3,
                ),
                dtype=np.uint8,
            )
        )


        edges = (
            cv2.morphologyEx(
                edges,
                cv2.MORPH_CLOSE,
                kernel,
                iterations=2,
            )
        )


        contours, _ = (
            cv2.findContours(
                edges,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
        )


        best = None


        for contour in contours:

            area = (
                cv2.contourArea(
                    contour
                )
            )


            if (
                area
                <
                MIN_CONTOUR_AREA
                or
                area
                >
                MAX_CONTOUR_AREA
            ):

                continue


            perimeter = (
                cv2.arcLength(
                    contour,
                    True,
                )
            )


            approx = (
                cv2.approxPolyDP(
                    contour,
                    0.04
                    *
                    perimeter,
                    True,
                )
            )


            if (
                len(
                    approx
                )
                <
                4
                or
                len(
                    approx
                )
                >
                8
            ):

                continue


            data = (
                self.contour_3d_data(
                    contour,
                    xyz,
                )
            )


            if data is None:

                continue


            (
                center,
                extents,
                valid_count,
            ) = data


            # Range sanity.
            if (
                not np.isfinite(
                    center[2]
                )
                or
                center[2] < 0.10
                or
                center[2] > 2.5
            ):

                continue


            score = (
                self.cube_score(
                    contour,
                    extents,
                )
            )


            if score is None:

                continue


            if (
                best is None
                or
                score
                >
                best[
                    "score"
                ]
            ):

                best = {
                    "contour":
                        contour,

                    "center":
                        center,

                    "extents":
                        extents,

                    "score":
                        score,

                    "valid_count":
                        valid_count,
                }


        # ====================================================
        # NOTHING FOUND
        # ====================================================

        if best is None:

            self.publish_not_detected()

            cv2.putText(
                debug,
                "CUBE: ---",
                (
                    10,
                    25,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (
                    0,
                    0,
                    255,
                ),
                2,
                cv2.LINE_AA,
            )

            self.publish_debug(
                debug,
                image_msg,
            )

            return


        contour = (
            best[
                "contour"
            ]
        )

        center = (
            best[
                "center"
            ]
        )

        extents = (
            best[
                "extents"
            ]
        )

        score = (
            best[
                "score"
            ]
        )


        # ====================================================
        # CAMERA POINT
        # ====================================================

        camera_point = (
            PointStamped()
        )

        camera_point.header = (
            image_msg.header
        )

        camera_point.header.frame_id = (
            CAMERA_FRAME
        )

        camera_point.point.x = float(
            center[0]
        )

        camera_point.point.y = float(
            center[1]
        )

        camera_point.point.z = float(
            center[2]
        )


        self.camera_point_pub.publish(
            camera_point
        )


        # ====================================================
        # BASE POINT
        # ====================================================

        point_base = (
            self.transform_to_base(
                center
            )
        )


        if point_base is not None:

            base_msg = (
                PointStamped()
            )

            base_msg.header.stamp = (
                image_msg.header.stamp
            )

            base_msg.header.frame_id = (
                BASE_FRAME
            )

            base_msg.point.x = float(
                point_base[0]
            )

            base_msg.point.y = float(
                point_base[1]
            )

            base_msg.point.z = float(
                point_base[2]
            )


            self.base_point_pub.publish(
                base_msg
            )


        # ====================================================
        # DETECTED
        # ====================================================

        detected_msg = (
            Bool()
        )

        detected_msg.data = True

        self.detected_pub.publish(
            detected_msg
        )


        type_msg = (
            String()
        )

        type_msg.data = (
            "cube_70mm"
        )

        self.type_pub.publish(
            type_msg
        )


        # ====================================================
        # JSON
        # ====================================================

        info = {
            "detected":
                True,

            "type":
                "cube_70mm",

            "confidence":
                round(
                    score,
                    3,
                ),

            "camera": {
                "x":
                    round(
                        float(
                            center[0]
                        ),
                        4,
                    ),

                "y":
                    round(
                        float(
                            center[1]
                        ),
                        4,
                    ),

                "z":
                    round(
                        float(
                            center[2]
                        ),
                        4,
                    ),
            },

            "visible_extents": {
                "x":
                    round(
                        float(
                            abs(
                                extents[0]
                            )
                        ),
                        4,
                    ),

                "y":
                    round(
                        float(
                            abs(
                                extents[1]
                            )
                        ),
                        4,
                    ),

                "z":
                    round(
                        float(
                            abs(
                                extents[2]
                            )
                        ),
                        4,
                    ),
            },
        }


        if point_base is not None:

            info[
                "base"
            ] = {
                "x":
                    round(
                        float(
                            point_base[0]
                        ),
                        4,
                    ),

                "y":
                    round(
                        float(
                            point_base[1]
                        ),
                        4,
                    ),

                "z":
                    round(
                        float(
                            point_base[2]
                        ),
                        4,
                    ),
            }


        info_msg = (
            String()
        )

        info_msg.data = (
            json.dumps(
                info
            )
        )

        self.info_pub.publish(
            info_msg
        )


        # ====================================================
        # DEBUG IMAGE
        # ====================================================

        cv2.drawContours(
            debug,
            [
                contour
            ],
            -1,
            (
                0,
                255,
                0,
            ),
            2,
        )


        moments = (
            cv2.moments(
                contour
            )
        )


        if moments[
            "m00"
        ] != 0:

            px = int(
                moments[
                    "m10"
                ]
                /
                moments[
                    "m00"
                ]
            )

            py = int(
                moments[
                    "m01"
                ]
                /
                moments[
                    "m00"
                ]
            )

        else:

            px = 10
            py = 50


        cv2.circle(
            debug,
            (
                px,
                py,
            ),
            5,
            (
                0,
                0,
                255,
            ),
            -1,
        )


        label = (
            f"CUBE "
            f"{score:.2f} "
            f"Z={center[2]:.3f}m"
        )


        cv2.putText(
            debug,
            label,
            (
                10,
                25,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (
                0,
                255,
                0,
            ),
            2,
            cv2.LINE_AA,
        )


        if point_base is not None:

            base_text = (
                f"B "
                f"{point_base[0]:.3f} "
                f"{point_base[1]:.3f} "
                f"{point_base[2]:.3f}"
            )


            cv2.putText(
                debug,
                base_text,
                (
                    10,
                    50,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (
                    255,
                    255,
                    0,
                ),
                1,
                cv2.LINE_AA,
            )


        self.publish_debug(
            debug,
            image_msg,
        )


def main(
    args=None,
):

    rclpy.init(
        args=args
    )

    node = (
        CubeDetector()
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
