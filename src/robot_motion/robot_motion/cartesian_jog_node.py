import rclpy

from rclpy.node import Node
from std_msgs.msg import String


OLD_TOPIC = "/manipulator/tcp_jog"
NEW_TOPIC = "/manipulator/cartesian_jog"

MAX_STEP_MM = 30.0


class CartesianJogCompatibilityNode(Node):
    """
    Adapter zgodnosci.

    NIE liczy IK.
    NIE publikuje absolutnych XYZ.
    NIE korzysta z MoveIt.

    Wszystko przekazuje do jednego:
        cartesian_controller

    Stary format:
        forward:10
        backward:10
        left:10
        right:10
        up:10
        down:10

    Nowy controller rozumie:
        forward
        back
        left
        right
        up
        down

    oraz:
        step:<metry>
    """

    def __init__(self):
        super().__init__(
            "cartesian_jog_node"
        )

        self.pub = self.create_publisher(
            String,
            NEW_TOPIC,
            10,
        )

        self.create_subscription(
            String,
            OLD_TOPIC,
            self.callback,
            10,
        )

        self.get_logger().info(
            "CARTESIAN JOG COMPATIBILITY ADAPTER"
        )

        self.get_logger().info(
            f"{OLD_TOPIC} -> {NEW_TOPIC}"
        )

        self.get_logger().info(
            "IK wykonuje TYLKO cartesian_controller."
        )

    def send(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)

    def callback(self, msg):
        raw = (
            msg.data
            .strip()
            .lower()
        )

        if ":" not in raw:
            self.get_logger().warn(
                f"Niepoprawny format: {raw}"
            )
            return

        command, value_text = (
            raw.split(":", 1)
        )

        mapping = {
            "forward": "forward",
            "backward": "back",
            "back": "back",
            "left": "left",
            "right": "right",
            "up": "up",
            "down": "down",
        }

        if command not in mapping:
            self.get_logger().warn(
                f"Nieznana komenda: {command}"
            )
            return

        try:
            mm = float(value_text)
        except ValueError:
            self.get_logger().warn(
                f"Niepoprawny krok: {value_text}"
            )
            return

        if (
            mm <= 0.0
            or
            mm > MAX_STEP_MM
        ):
            self.get_logger().warn(
                f"Krok musi byc 0..{MAX_STEP_MM:.0f} mm."
            )
            return

        step_m = mm / 1000.0

        # Najpierw ustawiamy krok, potem kierunek.
        self.send(
            f"step:{step_m:.6f}"
        )

        self.send(
            mapping[command]
        )

        self.get_logger().info(
            f"{raw} -> "
            f"step:{step_m:.6f} "
            f"+ {mapping[command]}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = (
        CartesianJogCompatibilityNode()
    )

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