from collections import deque
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped


CUBE_TOPIC = "/ai_cube/position_base"
IK_TARGET_TOPIC = "/manipulator/ik_target"

# Ile pomiarów pozycji kostki uśredniamy.
BUFFER_SIZE = 15

# Maksymalny rozrzut pomiarów, żeby uznać kostkę za stabilną.
MAX_SPREAD_M = 0.020

# Na pierwszy test zatrzymujemy tool0 10 cm nad środkiem kostki.
APPROACH_HEIGHT_M = 0.100

# Dodatkowe granice bezpieczeństwa.
MAX_XY_M = 0.65
MIN_TARGET_Z_M = -0.02
MAX_TARGET_Z_M = 0.70


class CubeApproachNode(Node):

    def __init__(self):

        super().__init__("cube_approach_node")

        self.samples = deque(maxlen=BUFFER_SIZE)

        self.target_sent = False

        self.create_subscription(
            PointStamped,
            CUBE_TOPIC,
            self.cube_callback,
            10,
        )

        self.ik_pub = self.create_publisher(
            Point,
            IK_TARGET_TOPIC,
            10,
        )

        self.get_logger().info(
            "=========================================="
        )
        self.get_logger().info(
            "CUBE APPROACH NODE"
        )
        self.get_logger().info(
            f"Cube input: {CUBE_TOPIC}"
        )
        self.get_logger().info(
            f"IK output: {IK_TARGET_TOPIC}"
        )
        self.get_logger().info(
            f"Approach height: {APPROACH_HEIGHT_M:.3f} m"
        )
        self.get_logger().info(
            "Czekam na stabilna pozycje kostki..."
        )
        self.get_logger().info(
            "=========================================="
        )

    def cube_callback(self, msg):

        # Po wysłaniu jednego celu nic więcej nie robimy.
        # Dzięki temu kamera nie będzie ciągle sterowała robotem.
        if self.target_sent:
            return

        if msg.header.frame_id != "base_link":
            self.get_logger().warn(
                f"Nieprawidlowy frame: {msg.header.frame_id}"
            )
            return

        x = float(msg.point.x)
        y = float(msg.point.y)
        z = float(msg.point.z)

        if not (
            math.isfinite(x)
            and math.isfinite(y)
            and math.isfinite(z)
        ):
            return

        self.samples.append(
            (x, y, z)
        )

        if len(self.samples) < BUFFER_SIZE:
            return

        xs = [p[0] for p in self.samples]
        ys = [p[1] for p in self.samples]
        zs = [p[2] for p in self.samples]

        spread_x = max(xs) - min(xs)
        spread_y = max(ys) - min(ys)
        spread_z = max(zs) - min(zs)

        max_spread = max(
            spread_x,
            spread_y,
            spread_z,
        )

        if max_spread > MAX_SPREAD_M:

            self.get_logger().info(
                f"Kostka niestabilna: "
                f"spread={max_spread * 1000.0:.1f} mm"
            )

            return

        cube_x = sum(xs) / len(xs)
        cube_y = sum(ys) / len(ys)
        cube_z = sum(zs) / len(zs)

        target_x = cube_x
        target_y = cube_y
        target_z = cube_z + APPROACH_HEIGHT_M

        self.get_logger().info("")
        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "STABILNA KOSTKA"
        )

        self.get_logger().info(
            f"cube X = {cube_x:+.4f}"
        )
        self.get_logger().info(
            f"cube Y = {cube_y:+.4f}"
        )
        self.get_logger().info(
            f"cube Z = {cube_z:+.4f}"
        )

        self.get_logger().info("")
        self.get_logger().info(
            "CEL TOOL0"
        )

        self.get_logger().info(
            f"target X = {target_x:+.4f}"
        )
        self.get_logger().info(
            f"target Y = {target_y:+.4f}"
        )
        self.get_logger().info(
            f"target Z = {target_z:+.4f}"
        )

        distance_xy = math.hypot(
            target_x,
            target_y,
        )

        if distance_xy > MAX_XY_M:
            self.get_logger().error(
                "STOP: cel XY poza bezpiecznym obszarem."
            )
            return

        if not (
            MIN_TARGET_Z_M
            <= target_z
            <= MAX_TARGET_Z_M
        ):
            self.get_logger().error(
                "STOP: target Z poza bezpiecznym obszarem."
            )
            return

        target = Point()

        target.x = target_x
        target.y = target_y
        target.z = target_z

        # Ustawiamy PRZED publish, żeby kolejny callback
        # nie wysłał drugiego celu.
        self.target_sent = True

        self.ik_pub.publish(
            target
        )

        self.get_logger().info("")
        self.get_logger().info(
            "CEL WYSLANY DO IK"
        )

        self.get_logger().info(
            "Robot powinien ustawic tool0 10 cm nad kostka."
        )

        self.get_logger().info(
            "=========================================="
        )


def main(args=None):

    rclpy.init(args=args)

    node = CubeApproachNode()

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
