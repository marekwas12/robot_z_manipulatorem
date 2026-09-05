#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

# ============================================================
# CUBE STACK PLACE NODE - ETAP 1 / KALIBRACJA
# ============================================================
#
# Na razie NIE rusza ramieniem automatycznie.
#
# Dlaczego:
# do prawdziwego ukladania wiezy potrzebujemy zmierzyc
# poprawne pozycje odkladania dla poziomu 0/1/2/3.
#
# Ten node trzyma licznik poziomu i interfejs ROS,
# ale bezpiecznie odmawia "start", dopoki nie wpiszesz
# skalibrowanych punktow.
# ============================================================

COMMAND_TOPIC = "/cube_stack_place/command"
STATUS_TOPIC = "/cube_stack_place/status"
LEVEL_TOPIC = "/cube_stack_place/level"

MAX_LEVELS = 4


class CubeStackPlaceNode(Node):

    def __init__(self):
        super().__init__(
            "cube_stack_place_node"
        )

        self.stack_level = 0

        self.create_subscription(
            String,
            COMMAND_TOPIC,
            self.command_callback,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            STATUS_TOPIC,
            10,
        )

        self.level_pub = self.create_publisher(
            Int32,
            LEVEL_TOPIC,
            10,
        )

        self.publish_level()

        self.get_logger().info(
            "CUBE STACK PLACE NODE - CALIBRATION MODE"
        )

    def publish_status(self, text):
        msg = String()
        msg.data = str(text)
        self.status_pub.publish(msg)
        self.get_logger().info(
            f"STACK PLACE STATUS -> {msg.data}"
        )

    def publish_level(self):
        msg = Int32()
        msg.data = int(
            self.stack_level
        )
        self.level_pub.publish(msg)

    def command_callback(self, msg):
        cmd = (
            msg.data
            .strip()
            .lower()
        )

        if cmd == "start":
            self.get_logger().error(
                "Brak skalibrowanych pozycji wiezy. "
                "Najpierw zmierz LEVEL_0..LEVEL_3."
            )
            self.publish_status(
                "error:not_calibrated"
            )
            return

        if cmd == "next":
            if self.stack_level < MAX_LEVELS:
                self.stack_level += 1
                self.publish_level()
                self.publish_status(
                    f"level:{self.stack_level}"
                )
            return

        if cmd == "reset":
            self.stack_level = 0
            self.publish_level()
            self.publish_status("idle")
            return

        self.get_logger().warn(
            f"Nieznana komenda: {cmd}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CubeStackPlaceNode()

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