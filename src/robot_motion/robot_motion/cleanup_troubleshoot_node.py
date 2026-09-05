#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Float32, Int32, String


class CleanupTroubleshootNode(Node):

    def __init__(self):
        super().__init__("cleanup_troubleshoot_node")

        # ----------------------------------------------------
        # Latest values
        # ----------------------------------------------------
        self.ai_detected = None
        self.ai_x = None
        self.ai_y = None
        self.ai_z = None
        self.ai_time = None

        self.depth = None
        self.depth_time = None

        self.aruco_detected = None
        self.aruco_x = None
        self.aruco_y = None
        self.aruco_z = None
        self.aruco_id = None
        self.aruco_yaw = None
        self.aruco_quality = None
        self.aruco_time = None

        self.cleanup_status = "---"
        self.trunk_status = "---"
        self.pick_status = "---"
        self.place_status = "---"

        # ----------------------------------------------------
        # Subscriptions
        # ----------------------------------------------------
        self.create_subscription(
            Bool,
            "/ai_cube/detected",
            self.ai_detected_cb,
            10,
        )

        self.create_subscription(
            PointStamped,
            "/ai_cube/position_camera_fast",
            self.ai_position_cb,
            10,
        )

        self.create_subscription(
            Float32,
            "/stereo/center_distance",
            self.depth_cb,
            10,
        )

        self.create_subscription(
            Bool,
            "/cube_aruco/detected",
            self.aruco_detected_cb,
            10,
        )

        self.create_subscription(
            PointStamped,
            "/cube_aruco/position_camera",
            self.aruco_position_cb,
            10,
        )

        self.create_subscription(
            Int32,
            "/cube_aruco/id",
            self.aruco_id_cb,
            10,
        )

        self.create_subscription(
            Float32,
            "/cube_aruco/yaw_deg",
            self.aruco_yaw_cb,
            10,
        )

        self.create_subscription(
            Float32,
            "/cube_aruco/quality",
            self.aruco_quality_cb,
            10,
        )

        self.create_subscription(
            String,
            "/cleanup_mission/status",
            self.cleanup_status_cb,
            10,
        )

        self.create_subscription(
            String,
            "/trunk_loading/status",
            self.trunk_status_cb,
            10,
        )

        self.create_subscription(
            String,
            "/cube_pick/status",
            self.pick_status_cb,
            10,
        )

        self.create_subscription(
            String,
            "/cube_place/status",
            self.place_status_cb,
            10,
        )

        self.timer = self.create_timer(
            0.50,
            self.print_status,
        )

        self.get_logger().info(
            "CLEANUP TROUBLESHOOT STARTED"
        )

    # --------------------------------------------------------
    # Callbacks
    # --------------------------------------------------------

    def ai_detected_cb(self, msg):
        self.ai_detected = bool(msg.data)

    def ai_position_cb(self, msg):
        self.ai_x = float(msg.point.x)
        self.ai_y = float(msg.point.y)
        self.ai_z = float(msg.point.z)
        self.ai_time = time.monotonic()

    def depth_cb(self, msg):
        self.depth = float(msg.data)
        self.depth_time = time.monotonic()

    def aruco_detected_cb(self, msg):
        self.aruco_detected = bool(msg.data)

    def aruco_position_cb(self, msg):
        self.aruco_x = float(msg.point.x)
        self.aruco_y = float(msg.point.y)
        self.aruco_z = float(msg.point.z)
        self.aruco_time = time.monotonic()

    def aruco_id_cb(self, msg):
        self.aruco_id = int(msg.data)

    def aruco_yaw_cb(self, msg):
        self.aruco_yaw = float(msg.data)

    def aruco_quality_cb(self, msg):
        self.aruco_quality = float(msg.data)

    def cleanup_status_cb(self, msg):
        self.cleanup_status = msg.data.strip()

    def trunk_status_cb(self, msg):
        self.trunk_status = msg.data.strip()

    def pick_status_cb(self, msg):
        self.pick_status = msg.data.strip()

    def place_status_cb(self, msg):
        self.place_status = msg.data.strip()

    # --------------------------------------------------------
    # Helpers
    # --------------------------------------------------------

    @staticmethod
    def age_text(stamp):
        if stamp is None:
            return "---"

        return f"{time.monotonic() - stamp:.2f}s"

    @staticmethod
    def fmt(value, digits=3):
        if value is None:
            return "---"

        return f"{value:.{digits}f}"

    def ai_bearing(self):
        if self.ai_x is None or self.ai_z is None:
            return None

        if self.ai_z <= 0.0:
            return None

        return math.degrees(
            math.atan2(
                self.ai_x,
                self.ai_z,
            )
        )

    def aruco_bearing(self):
        if self.aruco_x is None or self.aruco_z is None:
            return None

        if self.aruco_z <= 0.0:
            return None

        return math.degrees(
            math.atan2(
                self.aruco_x,
                self.aruco_z,
            )
        )

    # --------------------------------------------------------
    # Console output
    # --------------------------------------------------------

    def print_status(self):

        ai_angle = self.ai_bearing()
        aruco_angle = self.aruco_bearing()

        print("\n" + "=" * 72)

        print(
            "CLEANUP:",
            self.cleanup_status
        )

        print(
            "AI:",
            f"det={self.ai_detected}",
            f"XYZ=({self.fmt(self.ai_x)},"
            f"{self.fmt(self.ai_y)},"
            f"{self.fmt(self.ai_z)})",
            f"angle={self.fmt(ai_angle, 1)} deg",
            f"age={self.age_text(self.ai_time)}",
        )

        print(
            "DEPTH:",
            f"center={self.fmt(self.depth)} m",
            f"age={self.age_text(self.depth_time)}",
        )

        print(
            "ARUCO:",
            f"det={self.aruco_detected}",
            f"ID={self.aruco_id}",
            f"XYZ=({self.fmt(self.aruco_x)},"
            f"{self.fmt(self.aruco_y)},"
            f"{self.fmt(self.aruco_z)})",
            f"angle={self.fmt(aruco_angle, 1)} deg",
            f"yaw={self.fmt(self.aruco_yaw, 1)} deg",
            f"Q={self.fmt(self.aruco_quality, 2)}",
            f"age={self.age_text(self.aruco_time)}",
        )

        print(
            "PICK:",
            self.pick_status,
            "| PLACE:",
            self.place_status,
            "| TRUNK:",
            self.trunk_status,
        )

        # Simple diagnosis.
        if self.ai_z is not None:

            if self.ai_z > 0.325:
                print(
                    "DECYZJA EXPECTED: JAZDA AI"
                )

            elif 0.285 <= self.ai_z <= 0.325:
                print(
                    "DECYZJA EXPECTED: STOP + HANDOFF DO ARUCO"
                )

            else:
                print(
                    "DECYZJA EXPECTED: bardzo blisko celu"
                )

        aruco_age = (
            None
            if self.aruco_time is None
            else time.monotonic() - self.aruco_time
        )

        if (
            self.aruco_detected is True
            and
            (
                aruco_age is None
                or
                aruco_age > 1.0
            )
        ):
            print(
                "UWAGA ARUCO: detected=True, ale stabilne XYZ jest STARE. "
                "To znaczy marker jest widziany, lecz stable_pose nie publikuje "
                "świeżego /cube_aruco/position_camera."
            )

        if (
            self.cleanup_status.startswith(
                "CZEKA_NA_ARUCO_PRZED_PICK"
            )
        ):
            print(
                "!!! STARA LOGIKA CLEANUP: nie powinien już czekać na ArUco. "
                "Po poprawce ArUco obsługuje cube_pick_node."
            )

        print("=" * 72)


def main(args=None):

    rclpy.init(
        args=args
    )

    node = CleanupTroubleshootNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()