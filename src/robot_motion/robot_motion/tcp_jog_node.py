import math

import rclpy

from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Point

from std_msgs.msg import String

from tf2_ros import (
    Buffer,
    TransformListener,
)


# ============================================================
# KONFIGURACJA
# ============================================================

BASE_FRAME = "base_link"
TCP_LINK = "tool0"

JOG_TOPIC = "/manipulator/tcp_jog"
IK_TARGET_TOPIC = "/manipulator/ik_target"


# ============================================================
# BEZPIECZEŃSTWO
# ============================================================

# Maksymalny pojedynczy ruch liniowy.
MAX_LINEAR_STEP_MM = 30.0

# Nie pozwalamy na sterowanie tangencjalne bardzo blisko
# osi J1, bo wtedy mały ruch liniowy wymaga ogromnego
# obrotu podstawy.
MIN_RADIUS_M = 0.020

# Maksymalny obrót wynikający z pojedynczego ruchu
# LEWO / PRAWO.
MAX_ANGULAR_STEP_RAD = math.radians(12.0)


# ============================================================
# GLOBALNE GRANICE
#
# To tylko dodatkowe zabezpieczenie.
# Nie oznacza, że każdy punkt wewnątrz jest osiągalny.
# ============================================================

MAX_XY_M = 0.70
MIN_Z_M = -0.02
MAX_Z_M = 0.75


class TcpJogNode(Node):

    def __init__(self):

        super().__init__(
            "tcp_jog_node"
        )


        # ====================================================
        # TF
        # ====================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )


        # ====================================================
        # PUBLIKACJA CELU DLA IK
        # ====================================================

        self.ik_pub = self.create_publisher(
            Point,
            IK_TARGET_TOPIC,
            10,
        )


        # ====================================================
        # KOMENDY JOG
        #
        # Format:
        #
        # forward:10
        # backward:10
        # left:10
        # right:10
        # up:10
        # down:10
        #
        # Wszystkie wartości są w mm.
        # ====================================================

        self.create_subscription(
            String,
            JOG_TOPIC,
            self.jog_callback,
            10,
        )


        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "TCP JOG NODE"
        )

        self.get_logger().info(
            f"Nasluchuje: {JOG_TOPIC}"
        )

        self.get_logger().info(
            f"Publikuje: {IK_TARGET_TOPIC}"
        )

        self.get_logger().info(
            "forward/backward = promien od podstawy"
        )

        self.get_logger().info(
            "left/right = ruch po luku wokol J1"
        )

        self.get_logger().info(
            "up/down = globalne Z"
        )

        self.get_logger().info(
            "Wszystkie odleglosci w mm."
        )

        self.get_logger().info(
            "=========================================="
        )


    # ========================================================
    # AKTUALNE TCP
    # ========================================================

    def get_tcp(self):

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    BASE_FRAME,
                    TCP_LINK,
                    Time(),
                )
            )

        except Exception as exc:

            self.get_logger().error(
                f"Brak TF {BASE_FRAME} -> {TCP_LINK}: "
                f"{exc}"
            )

            return None


        translation = (
            transform.transform.translation
        )


        return (
            float(translation.x),
            float(translation.y),
            float(translation.z),
        )


    # ========================================================
    # PARSOWANIE KOMENDY
    # ========================================================

    def parse_command(
        self,
        text,
    ):

        text = (
            text
            .strip()
            .lower()
        )


        if ":" not in text:

            return None


        command, value_text = (
            text.split(
                ":",
                1,
            )
        )


        command = (
            command.strip()
        )


        try:

            value_mm = float(
                value_text
                .strip()
                .replace(",", ".")
            )

        except ValueError:

            return None


        if value_mm <= 0.0:

            return None


        if value_mm > MAX_LINEAR_STEP_MM:

            self.get_logger().warn(
                f"Ograniczam krok "
                f"{value_mm:.1f} mm -> "
                f"{MAX_LINEAR_STEP_MM:.1f} mm"
            )

            value_mm = (
                MAX_LINEAR_STEP_MM
            )


        allowed = {
            "forward",
            "backward",
            "left",
            "right",
            "up",
            "down",
        }


        if command not in allowed:

            return None


        return (
            command,
            value_mm,
        )


    # ========================================================
    # KOMENDA
    # ========================================================

    def jog_callback(
        self,
        msg,
    ):

        parsed = (
            self.parse_command(
                msg.data
            )
        )


        if parsed is None:

            self.get_logger().warn(
                f"Niepoprawna komenda jog: "
                f"{msg.data}"
            )

            return


        command, value_mm = (
            parsed
        )


        tcp = (
            self.get_tcp()
        )


        if tcp is None:

            return


        x, y, z = tcp


        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            f"JOG: {command} "
            f"{value_mm:.1f} mm"
        )

        self.get_logger().info(
            f"START: "
            f"X={x * 1000:.1f} "
            f"Y={y * 1000:.1f} "
            f"Z={z * 1000:.1f} mm"
        )


        # ====================================================
        # UKŁAD CYLINDRYCZNY
        # ====================================================

        radius = math.hypot(
            x,
            y,
        )


        angle = math.atan2(
            y,
            x,
        )


        step_m = (
            value_mm
            /
            1000.0
        )


        # ====================================================
        # DALEJ
        # ====================================================

        if command == "forward":

            new_radius = (
                radius
                +
                step_m
            )


            new_x = (
                new_radius
                *
                math.cos(angle)
            )


            new_y = (
                new_radius
                *
                math.sin(angle)
            )


            new_z = z


        # ====================================================
        # BLIŻEJ
        # ====================================================

        elif command == "backward":

            new_radius = (
                radius
                -
                step_m
            )


            if new_radius < MIN_RADIUS_M:

                self.get_logger().error(
                    "STOP: za blisko osi podstawy."
                )

                self.get_logger().error(
                    f"Minimalny promien = "
                    f"{MIN_RADIUS_M * 1000:.0f} mm"
                )

                return


            new_x = (
                new_radius
                *
                math.cos(angle)
            )


            new_y = (
                new_radius
                *
                math.sin(angle)
            )


            new_z = z


        # ====================================================
        # LEWO / PRAWO
        #
        # value_mm oznacza długość łuku, NIE stopnie.
        #
        # theta = s / r
        # ====================================================

        elif command in (
            "left",
            "right",
        ):

            if radius < MIN_RADIUS_M:

                self.get_logger().error(
                    "STOP: TCP jest zbyt blisko osi J1 "
                    "dla ruchu LEWO/PRAWO."
                )

                self.get_logger().error(
                    f"Promien = "
                    f"{radius * 1000:.1f} mm"
                )

                return


            delta_angle = (
                step_m
                /
                radius
            )


            delta_angle = min(
                delta_angle,
                MAX_ANGULAR_STEP_RAD,
            )


            if command == "left":

                new_angle = (
                    angle
                    +
                    delta_angle
                )

            else:

                new_angle = (
                    angle
                    -
                    delta_angle
                )


            new_x = (
                radius
                *
                math.cos(
                    new_angle
                )
            )


            new_y = (
                radius
                *
                math.sin(
                    new_angle
                )
            )


            new_z = z


        # ====================================================
        # GÓRA
        # ====================================================

        elif command == "up":

            new_x = x

            new_y = y

            new_z = (
                z
                +
                step_m
            )


        # ====================================================
        # DÓŁ
        # ====================================================

        elif command == "down":

            new_x = x

            new_y = y

            new_z = (
                z
                -
                step_m
            )


        else:

            return


        # ====================================================
        # GLOBALNE BEZPIECZEŃSTWO
        # ====================================================

        if (
            abs(new_x) > MAX_XY_M
            or
            abs(new_y) > MAX_XY_M
            or
            new_z < MIN_Z_M
            or
            new_z > MAX_Z_M
        ):

            self.get_logger().error(
                "STOP: nowy cel poza globalnym "
                "zakresem bezpieczenstwa."
            )

            return


        # ====================================================
        # DEBUG
        # ====================================================

        dx = (
            new_x
            -
            x
        )

        dy = (
            new_y
            -
            y
        )

        dz = (
            new_z
            -
            z
        )


        distance = math.sqrt(
            dx * dx
            +
            dy * dy
            +
            dz * dz
        )


        self.get_logger().info(
            f"CEL:   "
            f"X={new_x * 1000:.1f} "
            f"Y={new_y * 1000:.1f} "
            f"Z={new_z * 1000:.1f} mm"
        )


        self.get_logger().info(
            f"Ruch kartezjanski: "
            f"{distance * 1000:.1f} mm"
        )


        self.get_logger().info(
            f"R = {radius * 1000:.1f} mm"
        )


        # ====================================================
        # PUBLIKACJA DO IK
        # ====================================================

        target = Point()

        target.x = (
            new_x
        )

        target.y = (
            new_y
        )

        target.z = (
            new_z
        )


        self.ik_pub.publish(
            target
        )


        self.get_logger().info(
            "Cel wyslany do IK."
        )


def main(
    args=None,
):

    rclpy.init(
        args=args
    )


    node = (
        TcpJogNode()
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
