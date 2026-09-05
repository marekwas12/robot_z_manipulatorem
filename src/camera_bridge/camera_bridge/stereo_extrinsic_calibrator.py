import math
from collections import deque

import numpy as np

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

import tf2_ros


# ============================================================
# TOPICS
# ============================================================

CAMERA_POINT_TOPIC = (
    "/ai_cube/position_camera"
)

COMMAND_TOPIC = (
    "/stereo_calibration/command"
)


# ============================================================
# FRAMES
# ============================================================

BASE_FRAME = "base_link"
TOOL_FRAME = "tool0"


# ============================================================
# FILTERING
# ============================================================

CAMERA_BUFFER_SIZE = 30

MIN_CAMERA_SAMPLES = 10

MAX_CAMERA_SPREAD_M = 0.015


class StereoExtrinsicCalibrator(Node):

    def __init__(self):

        super().__init__(
            "stereo_extrinsic_calibrator"
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
        # CAMERA BUFFER
        # ====================================================

        self.camera_buffer = deque(
            maxlen=CAMERA_BUFFER_SIZE
        )

        # ====================================================
        # CALIBRATION PAIRS
        # ====================================================

        self.camera_samples = []
        self.base_samples = []

        # Temporary tool point waiting for matching camera point.
        self.pending_tool_point = None

        # ====================================================
        # SUBSCRIBERS
        # ====================================================

        self.create_subscription(
            PointStamped,
            CAMERA_POINT_TOPIC,
            self.camera_callback,
            10,
        )

        self.create_subscription(
            String,
            COMMAND_TOPIC,
            self.command_callback,
            10,
        )

        # ====================================================
        # START INFO
        # ====================================================

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "STEREO EXTRINSIC CALIBRATOR - AI"
        )

        self.get_logger().info(
            "Kamera podczas kalibracji:"
        )

        self.get_logger().info(
            "SERVO_1 = 155 deg"
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "Procedura jednej probki:"
        )

        self.get_logger().info(
            "1. Kostka w chwytaku"
        )

        self.get_logger().info(
            "2. capture_tool"
        )

        self.get_logger().info(
            "3. NIE RUSZAJ KOSTKI"
        )

        self.get_logger().info(
            "4. Odsun manipulator"
        )

        self.get_logger().info(
            "5. Poczekaj 2-3 sekundy"
        )

        self.get_logger().info(
            "6. capture_camera"
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "Komendy:"
        )

        self.get_logger().info(
            "capture_tool"
        )

        self.get_logger().info(
            "capture_camera"
        )

        self.get_logger().info(
            "list"
        )

        self.get_logger().info(
            "solve"
        )

        self.get_logger().info(
            "clear"
        )

        self.get_logger().info(
            "=========================================="
        )


    # ========================================================
    # AI CAMERA POINT
    # ========================================================

    def camera_callback(
        self,
        msg,
    ):

        point = np.array(
            [
                msg.point.x,
                msg.point.y,
                msg.point.z,
            ],
            dtype=np.float64,
        )

        if not np.all(
            np.isfinite(
                point
            )
        ):

            return

        if (
            point[2] < 0.10
            or
            point[2] > 2.00
        ):

            return

        self.camera_buffer.append(
            point
        )


    # ========================================================
    # TOOL POSITION
    # ========================================================

    def get_tool_position(
        self,
    ):

        try:

            transform = (
                self.tf_buffer
                .lookup_transform(
                    BASE_FRAME,
                    TOOL_FRAME,
                    rclpy.time.Time(),
                )
            )

        except Exception as exc:

            self.get_logger().error(
                f"Brak TF "
                f"{BASE_FRAME}->{TOOL_FRAME}: "
                f"{exc}"
            )

            return None

        t = (
            transform
            .transform
            .translation
        )

        return np.array(
            [
                t.x,
                t.y,
                t.z,
            ],
            dtype=np.float64,
        )


    # ========================================================
    # STABLE AI POINT
    # ========================================================

    def get_stable_camera_point(
        self,
    ):

        if (
            len(
                self.camera_buffer
            )
            <
            MIN_CAMERA_SAMPLES
        ):

            self.get_logger().error(
                "Za malo pomiarow AI. "
                "Poczekaj 2-3 sekundy."
            )

            return None

        points = np.array(
            self.camera_buffer,
            dtype=np.float64,
        )

        median = np.median(
            points,
            axis=0,
        )

        distances = np.linalg.norm(
            points
            -
            median,
            axis=1,
        )

        threshold = np.percentile(
            distances,
            70,
        )

        filtered = points[
            distances
            <=
            threshold
        ]

        if (
            filtered.shape[0]
            <
            6
        ):

            self.get_logger().error(
                "Za malo stabilnych pomiarow AI."
            )

            return None

        final_point = np.median(
            filtered,
            axis=0,
        )

        minimum = np.min(
            filtered,
            axis=0,
        )

        maximum = np.max(
            filtered,
            axis=0,
        )

        spread = (
            maximum
            -
            minimum
        )

        self.get_logger().info(
            "Stabilnosc AI:"
        )

        self.get_logger().info(
            f"X spread = "
            f"{spread[0] * 1000.0:.1f} mm"
        )

        self.get_logger().info(
            f"Y spread = "
            f"{spread[1] * 1000.0:.1f} mm"
        )

        self.get_logger().info(
            f"Z spread = "
            f"{spread[2] * 1000.0:.1f} mm"
        )

        if (
            np.max(
                spread
            )
            >
            MAX_CAMERA_SPREAD_M
        ):

            self.get_logger().error(
                "AI jest zbyt niestabilne. "
                "Nie zapisuje punktu."
            )

            return None

        return final_point


    # ========================================================
    # COMMANDS
    # ========================================================

    def command_callback(
        self,
        msg,
    ):

        command = (
            msg.data
            .strip()
            .lower()
        )

        if command == "capture_tool":

            self.capture_tool()

        elif command == "capture_camera":

            self.capture_camera()

        elif command == "list":

            self.list_samples()

        elif command == "solve":

            self.solve()

        elif command == "clear":

            self.clear()

        else:

            self.get_logger().warn(
                f"Nieznana komenda: {command}"
            )


    # ========================================================
    # CAPTURE TOOL
    # ========================================================

    def capture_tool(
        self,
    ):

        tool = (
            self.get_tool_position()
        )

        if tool is None:

            return

        self.pending_tool_point = (
            tool.copy()
        )

        self.camera_buffer.clear()

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "TOOL ZAPISANY"
        )

        self.get_logger().info(
            f"X = {tool[0]:+.4f}"
        )

        self.get_logger().info(
            f"Y = {tool[1]:+.4f}"
        )

        self.get_logger().info(
            f"Z = {tool[2]:+.4f}"
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "TERAZ:"
        )

        self.get_logger().info(
            "NIE RUSZAJ KOSTKI."
        )

        self.get_logger().info(
            "Odsun manipulator."
        )

        self.get_logger().info(
            "Poczekaj 2-3 sekundy."
        )

        self.get_logger().info(
            "Potem capture_camera."
        )

        self.get_logger().info(
            "=========================================="
        )


    # ========================================================
    # CAPTURE CAMERA
    # ========================================================

    def capture_camera(
        self,
    ):

        if (
            self.pending_tool_point
            is None
        ):

            self.get_logger().error(
                "Najpierw wykonaj capture_tool."
            )

            return

        camera = (
            self.get_stable_camera_point()
        )

        if camera is None:

            return

        self.base_samples.append(
            self.pending_tool_point.copy()
        )

        self.camera_samples.append(
            camera.copy()
        )

        number = len(
            self.camera_samples
        )

        tool = (
            self.pending_tool_point.copy()
        )

        self.pending_tool_point = None

        self.camera_buffer.clear()

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            f"PROBKA {number} GOTOWA"
        )

        self.get_logger().info(
            "CAMERA:"
        )

        self.get_logger().info(
            f"{camera[0]:+.5f} "
            f"{camera[1]:+.5f} "
            f"{camera[2]:+.5f}"
        )

        self.get_logger().info(
            "BASE / TOOL0:"
        )

        self.get_logger().info(
            f"{tool[0]:+.5f} "
            f"{tool[1]:+.5f} "
            f"{tool[2]:+.5f}"
        )

        self.get_logger().info(
            "=========================================="
        )


    # ========================================================
    # CLEAR
    # ========================================================

    def clear(
        self,
    ):

        self.camera_samples.clear()
        self.base_samples.clear()

        self.camera_buffer.clear()

        self.pending_tool_point = None

        self.get_logger().info(
            "Wyczyszczono wszystkie probki."
        )


    # ========================================================
    # LIST
    # ========================================================

    def list_samples(
        self,
    ):

        count = len(
            self.camera_samples
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            f"Liczba gotowych probek: {count}"
        )

        for index in range(
            count
        ):

            camera = (
                self.camera_samples[
                    index
                ]
            )

            base = (
                self.base_samples[
                    index
                ]
            )

            self.get_logger().info(
                f"{index + 1}: "
                f"C=["
                f"{camera[0]:+.4f}, "
                f"{camera[1]:+.4f}, "
                f"{camera[2]:+.4f}] "
                f"B=["
                f"{base[0]:+.4f}, "
                f"{base[1]:+.4f}, "
                f"{base[2]:+.4f}]"
            )


    # ========================================================
    # ROTATION -> RPY
    # ========================================================

    @staticmethod
    def rotation_to_rpy(
        rotation,
    ):

        sy = math.sqrt(
            rotation[0, 0] ** 2
            +
            rotation[1, 0] ** 2
        )

        singular = (
            sy < 1e-6
        )

        if not singular:

            roll = math.atan2(
                rotation[2, 1],
                rotation[2, 2],
            )

            pitch = math.atan2(
                -rotation[2, 0],
                sy,
            )

            yaw = math.atan2(
                rotation[1, 0],
                rotation[0, 0],
            )

        else:

            roll = math.atan2(
                -rotation[1, 2],
                rotation[1, 1],
            )

            pitch = math.atan2(
                -rotation[2, 0],
                sy,
            )

            yaw = 0.0

        return (
            roll,
            pitch,
            yaw,
        )


    # ========================================================
    # SOLVE
    # ========================================================

    def solve(
        self,
    ):

        count = len(
            self.camera_samples
        )

        if count < 5:

            self.get_logger().error(
                "Potrzeba minimum 5 gotowych probek. "
                "Najlepiej 6-8."
            )

            return

        camera = np.array(
            self.camera_samples,
            dtype=np.float64,
        )

        base = np.array(
            self.base_samples,
            dtype=np.float64,
        )

        camera_center = np.mean(
            camera,
            axis=0,
        )

        base_center = np.mean(
            base,
            axis=0,
        )

        camera_zero = (
            camera
            -
            camera_center
        )

        base_zero = (
            base
            -
            base_center
        )

        covariance = (
            camera_zero.T
            @
            base_zero
        )

        u, _, vt = np.linalg.svd(
            covariance
        )

        rotation = (
            vt.T
            @
            u.T
        )

        if (
            np.linalg.det(
                rotation
            )
            <
            0.0
        ):

            vt[
                -1,
                :
            ] *= -1.0

            rotation = (
                vt.T
                @
                u.T
            )

        translation = (
            base_center
            -
            rotation
            @
            camera_center
        )

        predicted = (
            (
                rotation
                @
                camera.T
            ).T
            +
            translation
        )

        errors = np.linalg.norm(
            predicted
            -
            base,
            axis=1,
        )

        rms = math.sqrt(
            float(
                np.mean(
                    errors ** 2
                )
            )
        )

        maximum = float(
            np.max(
                errors
            )
        )

        roll, pitch, yaw = (
            self.rotation_to_rpy(
                rotation
            )
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "WYNIK KALIBRACJI AI"
        )

        self.get_logger().info(
            "=========================================="
        )

        for index, error in enumerate(
            errors
        ):

            self.get_logger().info(
                f"Probka {index + 1}: "
                f"{error * 1000.0:.2f} mm"
            )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "TRANSLATION [m]:"
        )

        self.get_logger().info(
            f"x = {translation[0]:+.8f}"
        )

        self.get_logger().info(
            f"y = {translation[1]:+.8f}"
        )

        self.get_logger().info(
            f"z = {translation[2]:+.8f}"
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "RPY [rad]:"
        )

        self.get_logger().info(
            f"roll  = {roll:+.8f}"
        )

        self.get_logger().info(
            f"pitch = {pitch:+.8f}"
        )

        self.get_logger().info(
            f"yaw   = {yaw:+.8f}"
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "RPY [deg]:"
        )

        self.get_logger().info(
            f"roll  = "
            f"{math.degrees(roll):+.3f}"
        )

        self.get_logger().info(
            f"pitch = "
            f"{math.degrees(pitch):+.3f}"
        )

        self.get_logger().info(
            f"yaw   = "
            f"{math.degrees(yaw):+.3f}"
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            f"RMS = "
            f"{rms * 1000.0:.2f} mm"
        )

        self.get_logger().info(
            f"MAX = "
            f"{maximum * 1000.0:.2f} mm"
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "URDF:"
        )

        self.get_logger().info(
            '<origin '
            f'xyz="'
            f'{translation[0]:.8f} '
            f'{translation[1]:.8f} '
            f'{translation[2]:.8f}" '
            f'rpy="'
            f'{roll:.8f} '
            f'{pitch:.8f} '
            f'{yaw:.8f}"'
            '/>'
        )

        self.get_logger().info(
            "=========================================="
        )


def main(
    args=None,
):

    rclpy.init(
        args=args
    )

    node = (
        StereoExtrinsicCalibrator()
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