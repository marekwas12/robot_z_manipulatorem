# CLEANUP VERSION: wybiera NAJBLIZSZA kostke po Z; XYZ stable=3; ORT CPU optimized
import json
import math
import os
from collections import deque

import cv2
import numpy as np
import onnxruntime as ort
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String, Float32

import message_filters
import tf2_ros


# ============================================================
# MODEL
# ============================================================

MODEL_PATH = (
    "/home/marek/robot_ws/models/cube70mmv3.onnx"
)

MODEL_SIZE = 640

CLASS_NAME = "cube_70mm"
CLASS_ID = 0

# Wyższy próg = mniej fałszywych wykryć innych przedmiotów.
# Było 0.50. Podnosimy umiarkowanie do 0.65.
CONF_THRESHOLD = 0.50
NMS_THRESHOLD = 0.45

MASK_THRESHOLD = 0.50


# ============================================================
# ROS INPUT
# ============================================================

IMAGE_TOPIC = (
    "/stereo/left_rectified"
)

# XYZ z tego samego stereo_depth.
# AI daje MASKĘ kostki, a odległość/XYZ bierzemy z punktów stereo
# znajdujących się WEWNĄTRZ tej maski.
POINTS_TOPIC = (
    "/stereo/points"
)


# ============================================================
# ROS OUTPUT
# ============================================================

DETECTED_TOPIC = (
    "/ai_cube/detected"
)

CONFIDENCE_TOPIC = (
    "/ai_cube/confidence"
)

CAMERA_POINT_TOPIC = (
    "/ai_cube/position_camera"
)

# Szybki XYZ bez temporalnej mediany kilku klatek.
# Do jazdy i podgladu odleglosci uzywamy tego topicu.
FAST_CAMERA_POINT_TOPIC = (
    "/ai_cube/position_camera_fast"
)

BASE_POINT_TOPIC = (
    "/ai_cube/position_base"
)

INFO_TOPIC = (
    "/ai_cube/info"
)

DEBUG_TOPIC = (
    "/ai_cube/debug/compressed"
)

YAW_TOPIC = "/ai_cube/yaw_deg"
YAW_QUALITY_TOPIC = "/ai_cube/yaw_quality"

# ============================================================
# ROBUST YAW 0..45 DEG
#
# Kostka 70x70x70 mm ma symetrie co 90 deg, wiec do chwytania
# interesuje nas tylko bezwzgledny kat 0..45 deg.
# Zbieramy do 45 poprawnych estymacji, odrzucamy outliery MAD i publikujemy mediane.
# ============================================================
YAW_MEDIAN_WINDOW = 45
YAW_STALE_RESET_FRAMES = 30


# ============================================================
# TF
# ============================================================

CAMERA_FRAME = "stereo_camera_upright_optical"
BASE_FRAME = "base_link"


# ============================================================
# MONOCULAR 3D FROM KNOWN CUBE SIZE
# ============================================================

# Physical cube edge.
CUBE_SIZE_M = 0.070

# Camera must be in this fixed position during calibration and pick.
CALIBRATED_CAMERA_SERVO_DEG = 152.0

# Minimum mask quality.
MIN_MASK_PIXELS = 20
MIN_APPARENT_SIZE_PX = 12.0

# If width- and height-based depth estimates disagree too much, the
# silhouette is strongly non-square (rotation/partial occlusion).
# We still publish a point, but mark it lower quality in /ai_cube/info.
MAX_SIZE_AXIS_DISAGREEMENT = 0.45

# Temporal filtering is done directly in CAMERA coordinates.
HISTORY_SIZE = 10
MIN_STABLE_FRAMES = 3
MAX_CAMERA_SPREAD_M = 0.030

# ============================================================
# AI MASK + STEREO DEPTH
# ============================================================

# Minimalna liczba poprawnych punktów 3D wewnątrz maski AI.
MIN_STEREO_POINTS_IN_MASK = 20

# Odcinamy kilka pikseli od brzegu maski, bo właśnie na krawędziach
# disparity najczęściej miesza kostkę z tłem.
STEREO_MASK_ERODE_PX = 2

# Sensowny zakres roboczy dla kostki.
STEREO_Z_MIN_M = 0.08
STEREO_Z_MAX_M = 3.0

CALIBRATION_LEFT_YAML = os.path.expanduser(
    "~/robot_ws/stereo_calibration_v3/left.yaml"
)



class AICubeDetector(Node):

    def __init__(self):

        super().__init__(
            "ai_cube_detector"
        )

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.load_rectified_intrinsics()

        self.camera_history = deque(maxlen=HISTORY_SIZE)

        # Ostatnia wybrana najblizsza kostka. Gdy wybor przeskoczy
        # na inna kostke, czyscimy historie XYZ, zeby nie mieszac dwoch celow.
        self.last_nearest_raw = None

        # ====================================================
        # ONNX
        # ====================================================

        if not os.path.exists(
            MODEL_PATH
        ):

            raise RuntimeError(
                f"Brak modelu: {MODEL_PATH}"
            )

        self.get_logger().info(
            f"Laduje model: {MODEL_PATH}"
        )

        session_options = ort.SessionOptions()
        session_options.graph_optimization_level = (
            ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        )
        # Dobre ustawienie dla CPU: ONNX wykorzystuje kilka rdzeni wewnątrz
        # pojedynczej inferencji zamiast sztucznie kolejkować klatki.
        cpu_count = os.cpu_count() or 4
        session_options.intra_op_num_threads = max(1, min(4, cpu_count))
        session_options.inter_op_num_threads = 1
        session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL

        self.session = ort.InferenceSession(
            MODEL_PATH,
            sess_options=session_options,
            providers=[
                "CPUExecutionProvider"
            ],
        )

        self.input_name = (
            self.session
            .get_inputs()[0]
            .name
        )

        outputs = (
            self.session
            .get_outputs()
        )

        self.output_detection_name = (
            outputs[0].name
        )

        self.output_proto_name = (
            outputs[1].name
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

        self.confidence_pub = (
            self.create_publisher(
                Float32,
                CONFIDENCE_TOPIC,
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

        self.fast_camera_point_pub = (
            self.create_publisher(
                PointStamped,
                FAST_CAMERA_POINT_TOPIC,
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

        self.info_pub = (
            self.create_publisher(
                String,
                INFO_TOPIC,
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

        self.yaw_pub = self.create_publisher(Float32, YAW_TOPIC, 10)
        self.yaw_quality_pub = self.create_publisher(Float32, YAW_QUALITY_TOPIC, 10)
        self.yaw_raw_history = deque(maxlen=YAW_MEDIAN_WINDOW)
        self.yaw_stale_frames = 0

        # ====================================================
        # SUBSCRIBERS
        # ====================================================

        # Obraz i mapa XYZ muszą pochodzić z tej samej chwili.
        self.image_sub = message_filters.Subscriber(
            self,
            Image,
            IMAGE_TOPIC,
            qos_profile=qos_profile_sensor_data,
        )

        self.points_sub = message_filters.Subscriber(
            self,
            Image,
            POINTS_TOPIC,
            qos_profile=qos_profile_sensor_data,
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [
                self.image_sub,
                self.points_sub,
            ],
            queue_size=8,
            slop=0.08,
        )

        self.sync.registerCallback(
            self.callback
        )

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "AI CUBE DETECTOR"
        )

        self.get_logger().info(
            f"Model: cube70mmv3.onnx"
        )

        self.get_logger().info(
            f"Klasa: {CLASS_NAME}"
        )

        self.get_logger().info(
            f"Confidence >= {CONF_THRESHOLD:.2f}"
        )

        self.get_logger().info(
            f"Input: {IMAGE_TOPIC}"
        )

        self.get_logger().info(
            f"FAST XYZ: {FAST_CAMERA_POINT_TOPIC}"
        )

        self.get_logger().info(
            "XYZ: AI MASK + /stereo/points"
        )
        self.get_logger().info(
            f"Stereo XYZ input: {POINTS_TOPIC}"
        )

        self.get_logger().info(
            f"Intrinsics: fx={self.fx:.2f} fy={self.fy:.2f} "
            f"cx={self.cx:.2f} cy={self.cy:.2f}"
        )

        self.get_logger().info(
            "Stereo depth JEST uzywany do XYZ kostki przez maske AI."
        )

        self.get_logger().info(
            "OBRAZ: ROTATE_180 przed AI i obliczaniem XYZ"
        )

        self.get_logger().info(
            f"Output frame: {CAMERA_FRAME}"
        )
        self.get_logger().info(
            "YAW: 0..45 deg, do 45 poprawnych probek, MAD outlier reject + mediana"
        )

        self.get_logger().info(
            "=========================================="
        )


    # ========================================================
    # RECTIFIED INTRINSICS
    # ========================================================

    @staticmethod
    def _yaml_matrix(section):
        data = np.array(section["data"], dtype=np.float64)
        return data.reshape(int(section["rows"]), int(section["cols"]))

    def load_rectified_intrinsics(self):
        if not os.path.exists(CALIBRATION_LEFT_YAML):
            raise RuntimeError(
                f"Brak kalibracji: {CALIBRATION_LEFT_YAML}"
            )

        with open(CALIBRATION_LEFT_YAML, "r") as handle:
            data = yaml.safe_load(handle)

        p = self._yaml_matrix(data["projection_matrix"])
        self.fx = float(p[0, 0])
        self.fy = float(p[1, 1])

        self.image_width = int(data["image_width"])
        self.image_height = int(data["image_height"])

        # Kamera jest fizycznie obrocona o 180 stopni.
        # Obraz obracamy PRZED AI i geometrią.
        #
        # Transformacja pikseli:
        #   u' = (W - 1) - u
        #   v' = (H - 1) - v
        #
        # Dlatego principal point:
        #   cx' = (W - 1) - cx
        #   cy' = (H - 1) - cy
        raw_cx = float(p[0, 2])
        raw_cy = float(p[1, 2])

        self.cx = (self.image_width - 1) - raw_cx
        self.cy = (self.image_height - 1) - raw_cy

    # ========================================================
    # ROS IMAGE
    # ========================================================

    def ros_bgr(
        self,
        msg,
    ):

        if msg.encoding != "bgr8":

            raise RuntimeError(
                f"Expected bgr8, "
                f"got {msg.encoding}"
            )

        raw = np.frombuffer(
            msg.data,
            dtype=np.uint8,
        )

        expected = (
            msg.height
            *
            msg.step
        )

        if raw.size < expected:

            raise RuntimeError(
                "Za malo danych obrazu."
            )

        rows = raw[
            :expected
        ].reshape(
            msg.height,
            msg.step,
        )

        row_bytes = (
            msg.width
            *
            3
        )

        image = rows[
            :,
            :row_bytes
        ].reshape(
            msg.height,
            msg.width,
            3,
        )

        return np.ascontiguousarray(
            image
        )


    def ros_xyz(
        self,
        msg,
    ):

        if msg.encoding != "32FC3":

            raise RuntimeError(
                f"Expected 32FC3, "
                f"got {msg.encoding}"
            )

        raw = np.frombuffer(
            msg.data,
            dtype=np.float32,
        )

        expected = (
            msg.height
            *
            msg.width
            *
            3
        )

        if raw.size < expected:

            raise RuntimeError(
                "Za malo danych XYZ."
            )

        xyz = raw[
            :expected
        ].reshape(
            msg.height,
            msg.width,
            3,
        )

        return np.ascontiguousarray(
            xyz
        )


    # ========================================================
    # LETTERBOX
    # ========================================================

    def letterbox(
        self,
        image,
    ):

        original_h, original_w = (
            image.shape[:2]
        )

        scale = min(
            MODEL_SIZE / original_w,
            MODEL_SIZE / original_h,
        )

        new_w = int(
            round(
                original_w
                *
                scale
            )
        )

        new_h = int(
            round(
                original_h
                *
                scale
            )
        )

        resized = cv2.resize(
            image,
            (
                new_w,
                new_h,
            ),
            interpolation=
                cv2.INTER_LINEAR,
        )

        pad_w = (
            MODEL_SIZE
            -
            new_w
        )

        pad_h = (
            MODEL_SIZE
            -
            new_h
        )

        left = (
            pad_w // 2
        )

        right = (
            pad_w
            -
            left
        )

        top = (
            pad_h // 2
        )

        bottom = (
            pad_h
            -
            top
        )

        padded = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(
                114,
                114,
                114,
            ),
        )

        return (
            padded,
            scale,
            left,
            top,
        )


    # ========================================================
    # PREPROCESS
    # ========================================================

    def preprocess(
        self,
        image,
    ):

        padded, scale, pad_x, pad_y = (
            self.letterbox(
                image
            )
        )

        rgb = cv2.cvtColor(
            padded,
            cv2.COLOR_BGR2RGB,
        )

        tensor = (
            rgb
            .astype(
                np.float32
            )
            /
            255.0
        )

        tensor = np.transpose(
            tensor,
            (
                2,
                0,
                1,
            ),
        )

        tensor = np.expand_dims(
            tensor,
            axis=0,
        )

        tensor = np.ascontiguousarray(
            tensor,
            dtype=np.float32,
        )

        return (
            tensor,
            scale,
            pad_x,
            pad_y,
        )


    # ========================================================
    # NMS
    # ========================================================

    def nms(
        self,
        boxes,
        scores,
    ):

        if len(
            boxes
        ) == 0:

            return []

        boxes_xywh = []

        for box in boxes:

            x1, y1, x2, y2 = box

            boxes_xywh.append(
                [
                    float(x1),
                    float(y1),
                    float(
                        x2 - x1
                    ),
                    float(
                        y2 - y1
                    ),
                ]
            )

        indices = cv2.dnn.NMSBoxes(
            boxes_xywh,
            scores,
            CONF_THRESHOLD,
            NMS_THRESHOLD,
        )

        if len(
            indices
        ) == 0:

            return []

        return [
            int(i)
            for i in np.array(
                indices
            ).reshape(-1)
        ]


    # ========================================================
    # SIGMOID
    # ========================================================

    @staticmethod
    def sigmoid(
        value,
    ):

        value = np.clip(
            value,
            -50.0,
            50.0,
        )

        return (
            1.0
            /
            (
                1.0
                +
                np.exp(
                    -value
                )
            )
        )


    # ========================================================
    # MASK
    # ========================================================

    def build_mask(
        self,
        coefficients,
        prototypes,
        box_model,
        original_shape,
        scale,
        pad_x,
        pad_y,
    ):

        # prototypes:
        # [32, 160, 160]

        channels, proto_h, proto_w = (
            prototypes.shape
        )

        proto_flat = (
            prototypes.reshape(
                channels,
                -1,
            )
        )

        mask = (
            coefficients
            @
            proto_flat
        )

        mask = self.sigmoid(
            mask
        )

        mask = mask.reshape(
            proto_h,
            proto_w,
        )

        # Prototype -> 640 x 640
        mask_640 = cv2.resize(
            mask,
            (
                MODEL_SIZE,
                MODEL_SIZE,
            ),
            interpolation=
                cv2.INTER_LINEAR,
        )

        x1, y1, x2, y2 = (
            box_model
        )

        x1 = int(
            max(
                0,
                min(
                    MODEL_SIZE - 1,
                    round(x1),
                ),
            )
        )

        y1 = int(
            max(
                0,
                min(
                    MODEL_SIZE - 1,
                    round(y1),
                ),
            )
        )

        x2 = int(
            max(
                x1 + 1,
                min(
                    MODEL_SIZE,
                    round(x2),
                ),
            )
        )

        y2 = int(
            max(
                y1 + 1,
                min(
                    MODEL_SIZE,
                    round(y2),
                ),
            )
        )

        crop = np.zeros(
            (
                MODEL_SIZE,
                MODEL_SIZE,
            ),
            dtype=np.float32,
        )

        crop[
            y1:y2,
            x1:x2
        ] = (
            mask_640[
                y1:y2,
                x1:x2
            ]
        )

        original_h, original_w = (
            original_shape
        )

        scaled_w = int(
            round(
                original_w
                *
                scale
            )
        )

        scaled_h = int(
            round(
                original_h
                *
                scale
            )
        )

        x_start = int(
            pad_x
        )

        y_start = int(
            pad_y
        )

        x_end = (
            x_start
            +
            scaled_w
        )

        y_end = (
            y_start
            +
            scaled_h
        )

        crop = crop[
            y_start:y_end,
            x_start:x_end
        ]

        if crop.size == 0:

            return None

        mask_original = cv2.resize(
            crop,
            (
                original_w,
                original_h,
            ),
            interpolation=
                cv2.INTER_LINEAR,
        )

        return (
            mask_original
            >
            MASK_THRESHOLD
        )


    # ========================================================
    # POSTPROCESS
    # ========================================================

    def postprocess(
        self,
        output0,
        output1,
        image_shape,
        scale,
        pad_x,
        pad_y,
    ):

        predictions = (
            output0[0]
            .transpose(
                1,
                0,
            )
        )

        prototypes = (
            output1[0]
        )

        boxes_model = []
        boxes_original = []
        scores = []
        coefficients = []

        original_h, original_w = (
            image_shape
        )

        for prediction in predictions:

            confidence = float(
                prediction[4]
            )

            if (
                confidence
                <
                CONF_THRESHOLD
            ):

                continue

            cx = float(
                prediction[0]
            )

            cy = float(
                prediction[1]
            )

            width = float(
                prediction[2]
            )

            height = float(
                prediction[3]
            )

            x1_model = (
                cx
                -
                width / 2.0
            )

            y1_model = (
                cy
                -
                height / 2.0
            )

            x2_model = (
                cx
                +
                width / 2.0
            )

            y2_model = (
                cy
                +
                height / 2.0
            )

            x1 = (
                x1_model
                -
                pad_x
            ) / scale

            y1 = (
                y1_model
                -
                pad_y
            ) / scale

            x2 = (
                x2_model
                -
                pad_x
            ) / scale

            y2 = (
                y2_model
                -
                pad_y
            ) / scale

            x1 = float(
                np.clip(
                    x1,
                    0,
                    original_w - 1,
                )
            )

            y1 = float(
                np.clip(
                    y1,
                    0,
                    original_h - 1,
                )
            )

            x2 = float(
                np.clip(
                    x2,
                    0,
                    original_w - 1,
                )
            )

            y2 = float(
                np.clip(
                    y2,
                    0,
                    original_h - 1,
                )
            )

            if (
                x2 <= x1
                or
                y2 <= y1
            ):

                continue

            boxes_model.append(
                [
                    x1_model,
                    y1_model,
                    x2_model,
                    y2_model,
                ]
            )

            boxes_original.append(
                [
                    x1,
                    y1,
                    x2,
                    y2,
                ]
            )

            scores.append(
                confidence
            )

            coefficients.append(
                prediction[
                    5:37
                ].astype(
                    np.float32
                )
            )

        keep = self.nms(
            boxes_original,
            scores,
        )

        detections = []

        for index in keep:

            mask = self.build_mask(
                coefficients[index],
                prototypes,
                boxes_model[index],
                image_shape,
                scale,
                pad_x,
                pad_y,
            )

            if mask is None:

                continue

            if (
                np.count_nonzero(
                    mask
                )
                <
                80
            ):

                continue

            detections.append(
                {
                    "box":
                        boxes_original[index],

                    "score":
                        float(
                            scores[index]
                        ),

                    "mask":
                        mask,
                }
            )

        detections.sort(
            key=lambda item:
                item["score"],
            reverse=True,
        )

        return detections


    # ========================================================
    # MASK -> MONOCULAR XYZ
    # ========================================================

    def mask_to_monocular_xyz(self, mask):
        """
        Estimate cube center in the rectified left-camera optical frame.

        Assumption used for ranging:
        the cube has a known 70 mm edge and is presented approximately
        with the same orientation used during grasp calibration.

        u/v come from the segmentation-mask centroid.
        Z comes from the apparent 2D size:
            Zx = fx * L / width_px
            Zy = fy * L / height_px
        and the robust median of Zx/Zy is used.
        """
        mask_u8 = (mask.astype(np.uint8) * 255)

        # Light morphology removes isolated segmentation pixels.
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (3, 3),
        )
        mask_u8 = cv2.morphologyEx(
            mask_u8,
            cv2.MORPH_OPEN,
            kernel,
            iterations=1,
        )

        ys, xs = np.nonzero(mask_u8)
        if xs.size < MIN_MASK_PIXELS:
            return None

        # Robust image center from the mask median, less sensitive to a
        # small segmentation tail than a raw bounding-box center.
        u = float(np.median(xs))
        v = float(np.median(ys))

        contours, _ = cv2.findContours(
            mask_u8,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < MIN_MASK_PIXELS:
            return None

        # minAreaRect keeps the size measurement useful even if the
        # square silhouette is rotated in the image.
        rect = cv2.minAreaRect(contour)
        side_a, side_b = [float(x) for x in rect[1]]

        if side_a < MIN_APPARENT_SIZE_PX or side_b < MIN_APPARENT_SIZE_PX:
            return None

        width_px = max(side_a, side_b)
        height_px = min(side_a, side_b)

        z_from_a = float(self.fx * CUBE_SIZE_M / width_px)
        z_from_b = float(self.fy * CUBE_SIZE_M / height_px)

        z_candidates = np.array(
            [z_from_a, z_from_b],
            dtype=np.float64,
        )
        z = float(np.median(z_candidates))

        if not math.isfinite(z) or z <= 0.05 or z > 2.5:
            return None

        x = float((u - self.cx) * z / self.fx)
        y = float((v - self.cy) * z / self.fy)

        axis_disagreement = abs(z_from_a - z_from_b) / max(z, 1e-6)

        quality = {
            "u_px": u,
            "v_px": v,
            "width_px": width_px,
            "height_px": height_px,
            "z_from_width_m": z_from_a,
            "z_from_height_m": z_from_b,
            "axis_disagreement": axis_disagreement,
            "mask_pixels": int(xs.size),
            "orientation_warning": bool(
                axis_disagreement > MAX_SIZE_AXIS_DISAGREEMENT
            ),
        }

        return np.array([x, y, z], dtype=np.float64), quality

    # ========================================================
    # TEMPORAL STABILITY IN CAMERA FRAME
    # ========================================================

    def stable_camera_point(self, point):
        point = np.asarray(point, dtype=np.float64)

        if not np.all(np.isfinite(point)):
            self.camera_history.clear()
            return None, None

        self.camera_history.append(point)

        if len(self.camera_history) < MIN_STABLE_FRAMES:
            return None, None

        arr = np.asarray(self.camera_history, dtype=np.float64)
        median = np.median(arr, axis=0)
        distances = np.linalg.norm(arr - median, axis=1)

        cutoff = max(
            0.008,
            float(np.percentile(distances, 75)) * 1.8,
        )
        good = arr[distances <= cutoff]

        if good.shape[0] < MIN_STABLE_FRAMES:
            return None, None

        final = np.median(good, axis=0)
        spread = np.max(good, axis=0) - np.min(good, axis=0)

        if float(np.max(spread)) > MAX_CAMERA_SPREAD_M:
            return None, spread

        return final, spread

    # ========================================================
    # TF CAMERA -> BASE
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
                f"Brak TF "
                f"{CAMERA_FRAME}->{BASE_FRAME}: "
                f"{exc}"
            )

            return None

        t = (
            transform
            .transform
            .translation
        )

        q = (
            transform
            .transform
            .rotation
        )

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        rotation = np.array(
            [
                [
                    1 - 2 * (y*y + z*z),
                    2 * (x*y - z*w),
                    2 * (x*z + y*w),
                ],
                [
                    2 * (x*y + z*w),
                    1 - 2 * (x*x + z*z),
                    2 * (y*z - x*w),
                ],
                [
                    2 * (x*z - y*w),
                    2 * (y*z + x*w),
                    1 - 2 * (x*x + y*y),
                ],
            ],
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

        camera = np.array(
            point_camera,
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
    # DEBUG
    # ========================================================

    def publish_debug(
        self,
        image,
        source_msg,
    ):

        success, encoded = cv2.imencode(
            ".jpg",
            image,
            [
                int(
                    cv2.IMWRITE_JPEG_QUALITY
                ),
                85,
            ],
        )

        if not success:

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
    # NO DETECTION
    # ========================================================

    def publish_not_detected(
        self,
        debug,
        image_msg,
    ):

        detected = Bool()
        detected.data = False

        self.detected_pub.publish(
            detected
        )

        confidence = Float32()
        confidence.data = 0.0

        self.confidence_pub.publish(
            confidence
        )

        pass

        self.publish_debug(
            debug,
            image_msg,
        )


    # ========================================================
    # PROJECTIVE CUBE YAW FROM VANISHING POINTS - NO YAW CALIBRATION
    # ========================================================

    @staticmethod
    def _wrap_square_yaw(angle_deg):
        """Square symmetry: equivalent orientations repeat every 90 deg."""
        return ((float(angle_deg) + 45.0) % 90.0) - 45.0

    def estimate_geometric_yaw(self, image, mask):
        """
        Projective cube yaw from three orthogonal edge families.

        Unlike averaging 2-D line angles, this uses perspective geometry:
        parallel 3-D cube edges converge to vanishing points.  K^-1 * v gives
        the corresponding 3-D direction in the camera frame.  The most
        vertical-looking cube axis defines the table normal; the other two
        axes lie in the table plane.  Yaw is then measured in that plane
        relative to the camera-forward direction projected onto the plane.

        No yaw templates and no hand-entered angle calibration are used.
        J5_ZERO_DEG in cube_pick_node is only the mechanical wrist zero.
        """
        m = (mask.astype(np.uint8) * 255)
        if cv2.countNonZero(m) < MIN_MASK_PIXELS:
            return None

        # Edges only in a tight neighbourhood of the segmented cube.
        roi = cv2.dilate(m, np.ones((5, 5), np.uint8), iterations=1)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        edge = cv2.Canny(gray, 35, 110, apertureSize=3, L2gradient=True)
        edge = cv2.bitwise_and(edge, roi)

        ys, xs = np.nonzero(m)
        span = float(max(xs.max() - xs.min(), ys.max() - ys.min()))
        min_len = max(10, int(0.12 * span))

        raw = cv2.HoughLinesP(
            edge, 1, np.pi / 360.0,
            threshold=max(10, int(0.08 * span)),
            minLineLength=min_len,
            maxLineGap=max(5, int(0.08 * span)),
        )
        if raw is None or len(raw) < 6:
            return None

        h, w = m.shape
        segs = []
        for q in raw[:, 0, :]:
            x1, y1, x2, y2 = map(float, q)
            dx, dy = x2-x1, y2-y1
            length = math.hypot(dx, dy)
            if length < min_len:
                continue
            n = max(8, int(length / 3.0))
            xx = np.clip(np.rint(np.linspace(x1, x2, n)).astype(int), 0, w-1)
            yy = np.clip(np.rint(np.linspace(y1, y2, n)).astype(int), 0, h-1)
            if float(np.mean(roi[yy, xx] > 0)) < 0.75:
                continue
            a = math.atan2(dy, dx)
            # Undirected line embedding: angle and angle+pi are identical.
            feat = np.array([math.cos(2*a), math.sin(2*a)], dtype=np.float64)
            # homogeneous image line through endpoints
            l = np.cross(np.array([x1,y1,1.0]), np.array([x2,y2,1.0]))
            norm = math.hypot(l[0], l[1])
            if norm < 1e-9:
                continue
            l = l / norm
            segs.append((feat, l, length, (int(x1),int(y1),int(x2),int(y2))))

        if len(segs) < 6:
            return None

        X = np.stack([z[0] for z in segs])
        weights = np.asarray([z[2] for z in segs], dtype=np.float64)

        # Deterministic weighted k-means, k=3, on undirected line angles.
        # Seeds: longest line, then farthest from existing seeds.
        first = int(np.argmax(weights))
        centers = [X[first].copy()]
        while len(centers) < 3:
            d = np.min(np.stack([
                np.sum((X-c[None,:])**2, axis=1) for c in centers
            ]), axis=0)
            centers.append(X[int(np.argmax(d * weights))].copy())
        centers = np.stack(centers)

        labels = np.zeros(len(X), dtype=int)
        for _ in range(12):
            dist = np.stack([np.sum((X-c)**2, axis=1) for c in centers], axis=1)
            labels = np.argmin(dist, axis=1)
            newc = centers.copy()
            for k in range(3):
                idx = labels == k
                if np.sum(idx) < 2:
                    return None
                c = np.average(X[idx], axis=0, weights=weights[idx])
                nrm = np.linalg.norm(c)
                if nrm < 1e-9:
                    return None
                newc[k] = c/nrm
            if np.max(np.abs(newc-centers)) < 1e-5:
                centers = newc
                break
            centers = newc

        # Vanishing point of each family: least-squares intersection of lines.
        vanishing = []
        family_quality = []
        used = []
        for k in range(3):
            idxs = np.where(labels == k)[0]
            if len(idxs) < 2:
                return None
            L = np.stack([segs[i][1] for i in idxs])
            W = np.sqrt(np.asarray([segs[i][2] for i in idxs]))[:,None]
            _, sv, vt = np.linalg.svd(L * W, full_matrices=False)
            v = vt[-1]
            if abs(v[2]) > 1e-8:
                v = v / v[2]
            vanishing.append(v)
            # Smaller last singular value relative to first = better convergence.
            family_quality.append(float(max(0.0, 1.0 - sv[-1]/max(sv[0],1e-9))))
            used.extend([segs[i][3] for i in idxs])

        Kinv = np.array([
            [1.0/self.fx, 0.0, -self.cx/self.fx],
            [0.0, 1.0/self.fy, -self.cy/self.fy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        dirs = []
        for v in vanishing:
            d = Kinv @ v
            nrm = np.linalg.norm(d)
            if nrm < 1e-9:
                return None
            dirs.append(d/nrm)

        # Pick vertical cube axis. In an optical camera frame (+y down, +z
        # forward), the vertical object axis normally has the strongest image-y
        # component and the weakest forward component compared with table axes.
        vertical_idx = int(np.argmax([abs(d[1]) - 0.25*abs(d[2]) for d in dirs]))
        n = dirs[vertical_idx].copy()
        if n[1] > 0.0:   # choose "up" consistently
            n = -n

        horiz = [dirs[i].copy() for i in range(3) if i != vertical_idx]
        if len(horiz) != 2:
            return None

        # Build a camera-ground basis without external TF: camera forward
        # projected onto the plane is yaw=0; right completes the basis.
        forward = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        f = forward - np.dot(forward, n)*n
        fn = np.linalg.norm(f)
        if fn < 1e-5:
            # Camera almost normal to table: image-up is a better reference.
            f = np.array([0.0, -1.0, 0.0], dtype=np.float64)
            f = f - np.dot(f, n)*n
            fn = np.linalg.norm(f)
        if fn < 1e-5:
            return None
        f /= fn
        r = np.cross(f, n)
        r /= max(np.linalg.norm(r), 1e-9)

        # Both horizontal cube axes are equivalent modulo 90 degrees.
        candidates = []
        for d in horiz:
            d = d - np.dot(d,n)*n
            dn = np.linalg.norm(d)
            if dn < 1e-6:
                continue
            d /= dn
            a = math.degrees(math.atan2(np.dot(d,r), np.dot(d,f)))
            candidates.append(self._wrap_square_yaw(a))
        if not candidates:
            return None

        aa = np.deg2rad(np.asarray(candidates)*4.0)
        yaw = math.degrees(math.atan2(float(np.sum(np.sin(aa))), float(np.sum(np.cos(aa)))))/4.0
        yaw = self._wrap_square_yaw(yaw)

        # Orthogonality check in camera 3-D. Cube axes should be near 90 deg.
        orth_err = []
        for i in range(3):
            for j in range(i+1,3):
                orth_err.append(abs(float(np.dot(dirs[i], dirs[j]))))
        orth = max(0.0, 1.0 - float(np.mean(orth_err))/0.35)
        fam = float(np.mean(family_quality))
        quality = max(0.0, min(1.0, orth * fam))
        if quality < 0.12:
            return None

        return float(yaw), float(quality), used

    # ========================================================
    # AI MASK -> STEREO XYZ
    # ========================================================

    def mask_to_stereo_xyz(
        self,
        mask,
        xyz_upright,
    ):
        """
        AI wykrywa piksele kostki.
        Stereo depth daje XYZ dla każdego piksela.

        Zwracamy medianę XYZ WYŁĄCZNIE z wnętrza maski kostki.

        xyz_upright jest już obrócone o 180° razem z obrazem oraz ma:
            X' = -X
            Y' = -Y
            Z' =  Z

        dzięki czemu znak X zgadza się z tym, co robot/telefon widzi
        na prawidłowo obróconym obrazie.
        """

        if mask is None:
            return None

        mask_bool = (
            np.asarray(mask)
            >
            MASK_THRESHOLD
        )

        if mask_bool.shape != xyz_upright.shape[:2]:
            return None

        if STEREO_MASK_ERODE_PX > 0:

            kernel_size = (
                2 * STEREO_MASK_ERODE_PX + 1
            )

            kernel = np.ones(
                (
                    kernel_size,
                    kernel_size,
                ),
                dtype=np.uint8,
            )

            eroded = cv2.erode(
                mask_bool.astype(np.uint8),
                kernel,
                iterations=1,
            )

            if np.count_nonzero(eroded) >= MIN_STEREO_POINTS_IN_MASK:
                mask_bool = eroded > 0

        pts = xyz_upright[
            mask_bool
        ]

        if pts.size == 0:
            return None

        valid = (
            np.all(
                np.isfinite(pts),
                axis=1,
            )
            &
            (
                pts[:, 2]
                >
                STEREO_Z_MIN_M
            )
            &
            (
                pts[:, 2]
                <
                STEREO_Z_MAX_M
            )
        )

        pts = pts[
            valid
        ]

        if pts.shape[0] < MIN_STEREO_POINTS_IN_MASK:
            return None

        # Najpierw odpornie filtrujemy Z.
        z = pts[:, 2]

        z_med = float(
            np.median(z)
        )

        z_mad = float(
            np.median(
                np.abs(
                    z - z_med
                )
            )
        )

        z_limit = max(
            0.012,
            3.0
            *
            1.4826
            *
            z_mad,
        )

        keep = (
            np.abs(
                z - z_med
            )
            <=
            z_limit
        )

        pts = pts[
            keep
        ]

        if pts.shape[0] < MIN_STEREO_POINTS_IN_MASK:
            return None

        xyz = np.median(
            pts,
            axis=0,
        ).astype(np.float64)

        if not np.all(
            np.isfinite(xyz)
        ):
            return None

        return xyz


    # ========================================================
    # CALLBACK
    # ========================================================

    def callback(
        self,
        image_msg,
        points_msg,
    ):
        try:
            image = self.ros_bgr(
                image_msg
            )

            xyz_raw = self.ros_xyz(
                points_msg
            )

        except Exception as exc:
            self.get_logger().error(
                f"Input error: {exc}"
            )
            return

        if (
            xyz_raw.shape[0] != image.shape[0]
            or
            xyz_raw.shape[1] != image.shape[1]
        ):
            self.get_logger().error(
                "AI/STEREO rozny rozmiar obrazu."
            )
            return

        # KRYTYCZNE:
        # Surowy obraz z kamery jest do gory nogami.
        # Obracamy go o 180 stopni ZANIM AI policzy:
        # maske, centroid, rozmiar w pikselach oraz monocular XYZ.
        image = cv2.rotate(
            image,
            cv2.ROTATE_180,
        )

        # Ta sama rotacja pikseli dla XYZ.
        xyz_upright = np.ascontiguousarray(
            np.rot90(
                xyz_raw,
                2,
            )
        ).copy()

        # Obrót kamery o 180° wokół osi optycznej:
        # X/Y zmieniają znak, Z zostaje.
        xyz_upright[:, :, 0] *= -1.0
        xyz_upright[:, :, 1] *= -1.0

        debug = image.copy()

        tensor, scale, pad_x, pad_y = self.preprocess(image)

        try:
            output0, output1 = self.session.run(
                [
                    self.output_detection_name,
                    self.output_proto_name,
                ],
                {self.input_name: tensor},
            )
        except Exception as exc:
            self.get_logger().error(f"ONNX inference error: {exc}")
            return

        detections = self.postprocess(
            output0,
            output1,
            image.shape[:2],
            scale,
            pad_x,
            pad_y,
        )

        if not detections:
            self.camera_history.clear()
            self.last_nearest_raw = None
            self.publish_not_detected(debug, image_msg)
            return

        # ====================================================
        # WYBOR NAJBLIZSZEJ KOSTKI
        # ====================================================
        #
        # postprocess() zwraca wszystkie kostki widoczne w klatce.
        # W starej wersji wybierane bylo detections[0], czyli obiekt
        # o najwyzszym confidence. Dla misji zbierania chcemy NAJBLIZSZY.
        #
        # Dla kazdej maski liczymy monocular XYZ i wybieramy najmniejsze Z.
        # Jezeli dwie kostki sa widoczne jednoczesnie, publikowane topiki
        # /ai_cube/position_camera, /ai_cube/confidence i yaw dotycza
        # wybranej najblizszej kostki.
        # ====================================================

        valid_candidates = []

        for candidate in detections:

            candidate_camera = (
                self.mask_to_stereo_xyz(
                    candidate["mask"],
                    xyz_upright,
                )
            )

            if candidate_camera is None:
                continue

            # Metadane maski do /ai_cube/info i logów.
            # W starej wersji "quality" było słownikiem z geometrii
            # monokularnej. Po przejściu na stereo nie może być floatem,
            # bo dalszy kod korzysta z quality["..."].
            mask_bool = (
                np.asarray(candidate["mask"])
                >
                MASK_THRESHOLD
            )

            ys, xs = np.nonzero(
                mask_bool
            )

            if xs.size > 0:
                u_px = float(np.median(xs))
                v_px = float(np.median(ys))
                width_px = float(xs.max() - xs.min() + 1)
                height_px = float(ys.max() - ys.min() + 1)
                mask_pixels = int(xs.size)
            else:
                u_px = 0.0
                v_px = 0.0
                width_px = 0.0
                height_px = 0.0
                mask_pixels = 0

            candidate_quality = {
                "u_px": u_px,
                "v_px": v_px,
                "width_px": width_px,
                "height_px": height_px,

                # Te pola zostawiamy ze względu na kompatybilność
                # formatu /ai_cube/info. Odległość NIE pochodzi już
                # z szerokości/wysokości maski, tylko ze stereo.
                "z_from_width_m": float(candidate_camera[2]),
                "z_from_height_m": float(candidate_camera[2]),
                "axis_disagreement": 0.0,
                "orientation_warning": False,
                "mask_pixels": mask_pixels,
                "position_source": "stereo_points_in_ai_mask",
            }

            valid_candidates.append(
                (
                    float(candidate_camera[2]),
                    -float(candidate["score"]),
                    candidate,
                    candidate_camera,
                    candidate_quality,
                )
            )

        if not valid_candidates:
            # AI cos widzi, ale zadna maska nie daje sensownego XYZ.
            # Zachowujemy stare zachowanie informacyjne.
            detection = detections[0]
            score = float(detection["score"])

            self.camera_history.clear()

            detected_msg = Bool()
            detected_msg.data = True
            self.detected_pub.publish(detected_msg)

            confidence_msg = Float32()
            confidence_msg.data = score
            self.confidence_pub.publish(confidence_msg)

            self.get_logger().warn(
                "AI widzi kostke/kostki, ale brak poprawnych punktow stereo XYZ w masce."
            )

            # Rysujemy wszystkie boxy, zeby na telefonie bylo widac,
            # ze detektor cos znalazl mimo braku pozycji 3D.
            for candidate in detections:
                bx1, by1, bx2, by2 = [
                    int(round(value))
                    for value in candidate["box"]
                ]
                cv2.rectangle(
                    debug,
                    (bx1, by1),
                    (bx2, by2),
                    (0, 165, 255),
                    1,
                )

            self.publish_debug(debug, image_msg)
            return

        # Sortowanie:
        # 1) najmniejsze Z = najblizsza kostka,
        # 2) przy bardzo podobnym Z wyzszy confidence.
        valid_candidates.sort(
            key=lambda item: (item[0], item[1])
        )

        (
            selected_z,
            _negative_score,
            detection,
            raw_camera,
            quality,
        ) = valid_candidates[0]

        score = float(detection["score"])
        mask = detection["mask"]
        box = detection["box"]

        # Jezeli najblizszy cel nagle przeskoczyl o >12 cm w przestrzeni,
        # traktujemy go jako inna kostke i zerujemy filtr temporalny.
        if self.last_nearest_raw is not None:
            target_jump = float(
                np.linalg.norm(raw_camera - self.last_nearest_raw)
            )
            if target_jump > 0.12:
                self.camera_history.clear()

        self.last_nearest_raw = raw_camera.copy()

        # ====================================================
        # SZYBKI XYZ / Z DO JAZDY
        # ====================================================
        # Ten pomiar jest publikowany od razu z aktualnej klatki.
        # Nie czeka na MIN_STABLE_FRAMES ani mediane historii.
        fast_msg = PointStamped()
        fast_msg.header = image_msg.header
        fast_msg.header.frame_id = CAMERA_FRAME
        fast_msg.point.x = float(raw_camera[0])
        fast_msg.point.y = float(raw_camera[1])
        fast_msg.point.z = float(raw_camera[2])
        self.fast_camera_point_pub.publish(fast_msg)

        # Stary stabilny topic zostawiamy bez zmian dla precyzyjnych zadan.
        stable_camera, camera_spread = self.stable_camera_point(raw_camera)

        # Independent geometric yaw from visible cube edges.
        yaw_result = self.estimate_geometric_yaw(image, mask)
        yaw_value = None
        yaw_quality = 0.0
        yaw_lines = []

        if yaw_result is not None:
            yaw_signed, yaw_quality, yaw_lines = yaw_result

            # Do historii wpuszczamy tylko geometrycznie wiarygodne wyniki.
            # Sam estimator ma juz swoje testy jakosci; ten prog jest dodatkowym
            # zabezpieczeniem przed pojedynczymi przypadkowymi liniami.
            if not math.isfinite(float(yaw_signed)) or float(yaw_quality) < 0.20:
                yaw_result = None

        if yaw_result is not None:
            yaw_signed, yaw_quality, yaw_lines = yaw_result

            # Najpierw sprowadzamy do symetrii kwadratu [-45,+45],
            # potem bierzemy abs -> finalnie tylko 0..45 deg.
            yaw_folded = self._wrap_square_yaw(yaw_signed)
            yaw_abs = abs(float(yaw_folded))
            yaw_abs = max(0.0, min(45.0, yaw_abs))

            self.yaw_raw_history.append(yaw_abs)
            self.yaw_stale_frames = 0

            # Filtr statystyczny: najpierw odrzucamy outliery metoda MAD,
            # potem mediana ze wszystkich pozostalych poprawnych pomiarow.
            # Nie ma zadnego deadbandu: prawdziwe 1, 3 czy 5 deg pozostaje 1, 3, 5.
            hist = np.asarray(self.yaw_raw_history, dtype=np.float64)

            if hist.size >= 7:
                med0 = float(np.median(hist))
                abs_dev = np.abs(hist - med0)
                mad = float(np.median(abs_dev))

                if mad > 1e-6:
                    robust_sigma = 1.4826 * mad
                    keep = abs_dev <= max(2.5 * robust_sigma, 2.0)
                    filtered = hist[keep]
                    if filtered.size < max(5, hist.size // 2):
                        filtered = hist
                else:
                    filtered = hist
            else:
                filtered = hist

            yaw_value = float(np.median(filtered))
            yaw_value = max(0.0, min(45.0, yaw_value))

            ym = Float32()
            ym.data = float(yaw_value)
            self.yaw_pub.publish(ym)

            qm = Float32()
            qm.data = float(yaw_quality)
            self.yaw_quality_pub.publish(qm)
        else:
            # Nie kasujemy historii po jednej slabej klatce.
            # Estymator krawedzi potrafi czasem nie zwrocic wyniku mimo dobrej maski.
            self.yaw_stale_frames += 1
            if self.yaw_stale_frames >= YAW_STALE_RESET_FRAMES:
                self.yaw_raw_history.clear()
                self.yaw_stale_frames = 0

        # Detection itself.
        detected_msg = Bool()
        detected_msg.data = True
        self.detected_pub.publish(detected_msg)

        confidence_msg = Float32()
        confidence_msg.data = score
        self.confidence_pub.publish(confidence_msg)

        # /ai_cube/position_camera korzysta teraz z AI MASK + STEREO XYZ.
        # Publikujemy punkt stabilny, a do jazdy osobno szybki punkt bieżący.
        camera_for_publish = (
            stable_camera
            if stable_camera is not None
            else raw_camera
        )

        camera_msg = PointStamped()
        camera_msg.header = image_msg.header
        camera_msg.header.frame_id = CAMERA_FRAME
        camera_msg.point.x = float(camera_for_publish[0])
        camera_msg.point.y = float(camera_for_publish[1])
        camera_msg.point.z = float(camera_for_publish[2])
        self.camera_point_pub.publish(camera_msg)

        # Nie publikujemy /ai_cube/position_base z tego detektora.
        # Po obrocie obrazu pracujemy w wirtualnym, "upright" frame.
        # Chwytanie uzywa bezposredniej kalibracji:
        # CAMERA_UPRIGHT -> GRASP_TCP.
        base_published = False
        raw_base = None

        info = {
            "detected": True,
            "visible_detections": len(detections),
            "valid_xyz_candidates": len(valid_candidates),
            "selected": "nearest_stereo_z",
            "position_valid": bool(stable_camera is not None),
            "position_method": "stereo_points_inside_ai_mask_rot180",
            "class": CLASS_NAME,
            "confidence": round(score, 4),
            "camera_servo_required_deg": CALIBRATED_CAMERA_SERVO_DEG,
            "camera": {
                "x": round(float(camera_for_publish[0]), 5),
                "y": round(float(camera_for_publish[1]), 5),
                "z": round(float(camera_for_publish[2]), 5),
            },
            "image_measurement": {
                "u_px": round(float(quality["u_px"]), 2),
                "v_px": round(float(quality["v_px"]), 2),
                "width_px": round(float(quality["width_px"]), 2),
                "height_px": round(float(quality["height_px"]), 2),
                "z_from_width_m": round(
                    float(quality["z_from_width_m"]), 5
                ),
                "z_from_height_m": round(
                    float(quality["z_from_height_m"]), 5
                ),
                "axis_disagreement": round(
                    float(quality["axis_disagreement"]), 4
                ),
                "orientation_warning": bool(
                    quality["orientation_warning"]
                ),
                "mask_pixels": int(quality["mask_pixels"]),
                "position_source": quality.get(
                    "position_source",
                    "unknown",
                ),
            },
            "history_frames": len(self.camera_history),
            "yaw_method": "geometric_hough_edges_abs_0_45_median45_mad",
            "yaw_deg": None if yaw_value is None else round(float(yaw_value), 2),
            "yaw_quality": round(float(yaw_quality), 3),
            "yaw_history_frames": len(self.yaw_raw_history),
        }

        if camera_spread is not None:
            info["camera_spread_m"] = {
                "x": round(float(camera_spread[0]), 5),
                "y": round(float(camera_spread[1]), 5),
                "z": round(float(camera_spread[2]), 5),
            }

        info_msg = String()
        info_msg.data = json.dumps(info)
        self.info_pub.publish(info_msg)

        # Debug: cienkie pomaranczowe boxy = wszystkie widoczne kostki.
        # Zielona gruba maska/ramka ponizej = kostka wybrana jako najblizsza.
        for candidate in detections:
            bx1, by1, bx2, by2 = [
                int(round(value))
                for value in candidate["box"]
            ]
            cv2.rectangle(
                debug,
                (bx1, by1),
                (bx2, by2),
                (0, 165, 255),
                1,
            )

        pass

        # Debug overlay wybranej najblizszej kostki.
        overlay = debug.copy()
        overlay[mask] = (0, 255, 0)
        debug = cv2.addWeighted(
            overlay,
            0.30,
            debug,
            0.70,
            0,
        )

        x1, y1, x2, y2 = [
            int(round(value))
            for value in box
        ]

        cv2.rectangle(
            debug,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2,
        )

        status = (
            "AI+STEREO STABLE"
            if stable_camera is not None
            else
            "AI+STEREO"
        )

        pass

        pass

        pass

        for lx1, ly1, lx2, ly2 in yaw_lines:
            cv2.line(debug, (lx1, ly1), (lx2, ly2), (255, 0, 255), 2, cv2.LINE_AA)

        if yaw_value is not None:
            pass

        if stable_camera is not None:
            self.get_logger().info(
                "CUBE AI+STEREO VALID | "
                f"X={stable_camera[0]:+.4f} "
                f"Y={stable_camera[1]:+.4f} "
                f"Z={stable_camera[2]:+.4f} | "
                f"size={quality['width_px']:.1f}x"
                f"{quality['height_px']:.1f}px"
            )

        self.publish_debug(debug, image_msg)


def main(
    args=None,
):

    rclpy.init(
        args=args
    )

    node = AICubeDetector()

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