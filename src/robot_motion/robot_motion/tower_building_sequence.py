#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

COMMAND_TOPIC = "/tower_building/command"
STATUS_TOPIC = "/tower_building/status"

PICK_COMMAND_TOPIC = "/cube_stack_pick/command"
PICK_STATUS_TOPIC = "/cube_stack_pick/status"

PLACE_COMMAND_TOPIC = "/cube_stack_place/command"
PLACE_STATUS_TOPIC = "/cube_stack_place/status"

IDLE = 0
WAIT_PICK = 1
WAIT_PLACE = 2


class TowerBuildingSequence(Node):

    def __init__(self):
        super().__init__(
            "tower_building_sequence"
        )

        self.state = IDLE

        self.pick_pub = self.create_publisher(
            String,
            PICK_COMMAND_TOPIC,
            10,
        )

        self.place_pub = self.create_publisher(
            String,
            PLACE_COMMAND_TOPIC,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            STATUS_TOPIC,
            10,
        )

        self.create_subscription(
            String,
            COMMAND_TOPIC,
            self.command_callback,
            10,
        )

        self.create_subscription(
            String,
            PICK_STATUS_TOPIC,
            self.pick_status_callback,
            10,
        )

        self.create_subscription(
            String,
            PLACE_STATUS_TOPIC,
            self.place_status_callback,
            10,
        )

    def send(self, pub, text):
        msg = String()
        msg.data = str(text)
        pub.publish(msg)

    def status(self, text):
        self.send(
            self.status_pub,
            text,
        )
        self.get_logger().info(
            f"TOWER STATUS -> {text}"
        )

    def command_callback(self, msg):
        cmd = (
            msg.data
            .strip()
            .lower()
        )

        if cmd == "start":
            if self.state != IDLE:
                self.get_logger().warn(
                    "Tower sequence juz trwa."
                )
                return

            self.status(
                "running:pick"
            )

            self.send(
                self.pick_pub,
                "full",
            )

            self.state = WAIT_PICK
            return

        if cmd == "reset":
            self.send(
                self.pick_pub,
                "reset",
            )
            self.send(
                self.place_pub,
                "reset",
            )
            self.state = IDLE
            self.status("idle")
            return

    def pick_status_callback(self, msg):
        if self.state != WAIT_PICK:
            return

        status = (
            msg.data
            .strip()
            .lower()
        )

        if status == "done":
            self.status(
                "running:place"
            )

            self.send(
                self.place_pub,
                "start",
            )

            self.state = WAIT_PLACE
            return

        if status == "error":
            self.status(
                "error:pick"
            )
            self.state = IDLE

    def place_status_callback(self, msg):
        if self.state != WAIT_PLACE:
            return

        status = (
            msg.data
            .strip()
            .lower()
        )

        if status.startswith(
            "done:"
        ):
            self.status(status)
            self.state = IDLE
            return

        if status.startswith(
            "error"
        ):
            self.status(
                f"error:place:{status}"
            )
            self.state = IDLE


def main(args=None):
    rclpy.init(args=args)
    node = TowerBuildingSequence()

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