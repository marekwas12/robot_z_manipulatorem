import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import String

import tf2_ros


# ============================================================
# TOPIKI
# ============================================================

IK_TOPIC = "/manipulator/ik_target"

AUTO_COMMAND_TOPIC = (
    "/cube_calibration_auto/command"
)

CALIBRATOR_COMMAND_TOPIC = (
    "/cube_tcp_calibration/command"
)


# ============================================================
# TF
# ============================================================

BASE_FRAME = "base_link"
TOOL_FRAME = "tool0"


# ============================================================
# PARAMETRY RUCHU
# ============================================================

# Pozycja bezpieczna podczas przejazdu XY.
# Jest kilka cm wyzej niz pozycje chwytu.
SAFE_Z = -0.020

# Jak blisko celu musi byc tool0.
TARGET_TOLERANCE = 0.008

# Ile kolejnych odczytow musi potwierdzic cel.
TARGET_STABLE_COUNT = 4

# Maksymalny czas jednego ruchu.
MOVE_TIMEOUT = 15.0

# Po capture dajemy chwile kalibratorowi na zapis.
CAPTURE_DELAY = 1.0


# ============================================================
# TWOJE RZECZYWISCIE SPRAWDZONE PUNKTY TOOL0
#
# To sa pozycje z poprzedniej kalibracji,
# ktore recznie ustawiles przy kostce lezacej na ziemi.
#
# X, Y, Z
# ============================================================

CALIBRATION_POINTS = [

    (
        0.2228388891,
        0.2052756102,
        -0.0774027657,
    ),

    (
        0.1953590718,
        0.2463345730,
        -0.0758831357,
    ),

    (
        0.2554263242,
        0.1705102766,
        -0.0764414952,
    ),

    (
        0.2480246799,
        0.2582346312,
        -0.0737699937,
    ),

    (
        0.2904577360,
        0.2113784540,
        -0.0773131708,
    ),

    (
        0.2539284925,
        0.2302956288,
        -0.0761119201,
    ),

    (
        0.2438419039,
        0.2206923982,
        -0.0748553826,
    ),

    (
        0.2760335140,
        0.1855816749,
        -0.0708392253,
    ),
]


# ============================================================
# STANY
# ============================================================

IDLE = 0

WAIT_LIFT = 1
WAIT_XY = 2
WAIT_DESCEND = 3

WAIT_USER = 4

AFTER_CAPTURE = 5

DONE = 6
ERROR = 7


class CubeCalibrationAuto(Node):

    def __init__(self):

        super().__init__(
            "cube_calibration_auto"
        )

        # ====================================================
        # PUBLIKATORY
        # ====================================================

        self.ik_pub = (
            self.create_publisher(
                Point,
                IK_TOPIC,
                10,
            )
        )

        self.calibrator_pub = (
            self.create_publisher(
                String,
                CALIBRATOR_COMMAND_TOPIC,
                10,
            )
        )

        # ====================================================
        # KOMENDY
        # ====================================================

        self.create_subscription(
            String,
            AUTO_COMMAND_TOPIC,
            self.command_callback,
            10,
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
        # STAN
        # ====================================================

        self.state = IDLE

        self.point_index = 0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        self.target_stable_counter = 0

        self.state_start_time = (
            self.get_clock().now()
        )

        # ====================================================
        # TIMER
        # ====================================================

        self.timer = (
            self.create_timer(
                0.10,
                self.update,
            )
        )

        # ====================================================
        # INFO
        # ====================================================

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "AUTO CUBE TCP CALIBRATION"
        )

        self.get_logger().info(
            f"Liczba punktow: "
            f"{len(CALIBRATION_POINTS)}"
        )

        self.get_logger().info(
            "Punkty pochodza z recznych "
            "sprawdzonych ustawien TOOL0."
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "Komendy:"
        )

        self.get_logger().info(
            "start   -> nowa kalibracja"
        )

        self.get_logger().info(
            "capture -> zapisz kostke + TOOL0 "
            "i jedz dalej"
        )

        self.get_logger().info(
            "skip    -> pomin punkt"
        )

        self.get_logger().info(
            "status  -> pokaz stan"
        )

        self.get_logger().info(
            "stop    -> zatrzymaj"
        )

        self.get_logger().info(
            "=========================================="
        )

    # ========================================================
    # STAN / CZAS
    # ========================================================

    def set_state(
        self,
        state,
    ):

        self.state = state

        self.state_start_time = (
            self.get_clock().now()
        )

        self.target_stable_counter = 0

    def state_time(self):

        return (
            (
                self.get_clock().now()
                -
                self.state_start_time
            ).nanoseconds
            /
            1e9
        )

    # ========================================================
    # TOOL0
    # ========================================================

    def get_tool_position(self):

        try:

            transform = (
                self.tf_buffer
                .lookup_transform(
                    BASE_FRAME,
                    TOOL_FRAME,
                    rclpy.time.Time(),
                )
            )

        except Exception:

            return None

        t = (
            transform
            .transform
            .translation
        )

        return (
            float(t.x),
            float(t.y),
            float(t.z),
        )

    # ========================================================
    # AKTUALNY PUNKT
    # ========================================================

    def current_point(self):

        if (
            self.point_index
            >=
            len(CALIBRATION_POINTS)
        ):

            return None

        return (
            CALIBRATION_POINTS[
                self.point_index
            ]
        )

    # ========================================================
    # CEL IK
    # ========================================================

    def send_target(
        self,
        x,
        y,
        z,
    ):

        self.target_x = float(x)
        self.target_y = float(y)
        self.target_z = float(z)

        self.target_stable_counter = 0

        msg = Point()

        msg.x = self.target_x
        msg.y = self.target_y
        msg.z = self.target_z

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "NOWY CEL TCP"
        )

        self.get_logger().info(
            f"X = {self.target_x:+.5f}"
        )

        self.get_logger().info(
            f"Y = {self.target_y:+.5f}"
        )

        self.get_logger().info(
            f"Z = {self.target_z:+.5f}"
        )

        self.ik_pub.publish(
            msg
        )

    # ========================================================
    # CZY CEL OSIAGNIETY
    # ========================================================

    def target_reached(self):

        tcp = (
            self.get_tool_position()
        )

        if tcp is None:

            self.target_stable_counter = 0

            return False

        x, y, z = tcp

        error = math.sqrt(
            (
                x
                -
                self.target_x
            ) ** 2
            +
            (
                y
                -
                self.target_y
            ) ** 2
            +
            (
                z
                -
                self.target_z
            ) ** 2
        )

        if (
            error
            <=
            TARGET_TOLERANCE
        ):

            self.target_stable_counter += 1

        else:

            self.target_stable_counter = 0

        return (
            self.target_stable_counter
            >=
            TARGET_STABLE_COUNT
        )

    # ========================================================
    # TIMEOUT
    # ========================================================

    def movement_timeout(self):

        if (
            self.state_time()
            <
            MOVE_TIMEOUT
        ):

            return False

        tcp = (
            self.get_tool_position()
        )

        self.get_logger().error(
            "=========================================="
        )

        self.get_logger().error(
            "TIMEOUT RUCHU"
        )

        if tcp is not None:

            self.get_logger().error(
                f"TCP="
                f"({tcp[0]:+.4f}, "
                f"{tcp[1]:+.4f}, "
                f"{tcp[2]:+.4f})"
            )

        self.get_logger().error(
            f"CEL="
            f"({self.target_x:+.4f}, "
            f"{self.target_y:+.4f}, "
            f"{self.target_z:+.4f})"
        )

        self.get_logger().error(
            "=========================================="
        )

        self.set_state(
            ERROR
        )

        return True

    # ========================================================
    # START RUCHU DO AKTUALNEGO PUNKTU
    # ========================================================

    def begin_current_point(self):

        point = (
            self.current_point()
        )

        if point is None:

            self.finish_calibration()

            return

        tcp = (
            self.get_tool_position()
        )

        if tcp is None:

            self.get_logger().error(
                "Brak TF base_link -> tool0."
            )

            self.set_state(
                ERROR
            )

            return

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            f"PRZEJSCIE DO PUNKTU "
            f"{self.point_index + 1}/"
            f"{len(CALIBRATION_POINTS)}"
        )

        self.get_logger().info(
            "=========================================="
        )

        # ----------------------------------------------------
        # Najpierw tylko pionowo do SAFE_Z.
        # Aktualne X/Y zostaja.
        # ----------------------------------------------------

        self.send_target(
            tcp[0],
            tcp[1],
            SAFE_Z,
        )

        self.set_state(
            WAIT_LIFT
        )

    # ========================================================
    # GOTOWY PUNKT
    # ========================================================

    def announce_ready(self):

        point = (
            self.current_point()
        )

        if point is None:

            return

        x, y, z = point

        tcp = (
            self.get_tool_position()
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            f"PUNKT "
            f"{self.point_index + 1}/"
            f"{len(CALIBRATION_POINTS)} GOTOWY"
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "ZADANY:"
        )

        self.get_logger().info(
            f"X={x:+.5f} "
            f"Y={y:+.5f} "
            f"Z={z:+.5f}"
        )

        if tcp is not None:

            self.get_logger().info(
                "RZECZYWISTY TOOL0:"
            )

            self.get_logger().info(
                f"X={tcp[0]:+.5f} "
                f"Y={tcp[1]:+.5f} "
                f"Z={tcp[2]:+.5f}"
            )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "TERAZ:"
        )

        self.get_logger().info(
            "1. WLOZ KOSTKE DOKLADNIE "
            "POD CHWYTAK."
        )

        self.get_logger().info(
            "2. KOSTKA PROSTO WZGLEDEM ROBOTA."
        )

        self.get_logger().info(
            "3. NIE RUSZAJ RAMIENIEM."
        )

        self.get_logger().info(
            "4. POCZEKAJ AZ AI/STEREO "
            "USTABILIZUJE WYNIK."
        )

        self.get_logger().info(
            "5. WYSLIJ CAPTURE."
        )

        self.get_logger().info(
            "=========================================="
        )

    # ========================================================
    # FINISH
    # ========================================================

    def finish_calibration(self):

        self.set_state(
            DONE
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "=========================================="
        )

        self.get_logger().info(
            "WSZYSTKIE PUNKTY ZAKONCZONE"
        )

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "Teraz wykonaj:"
        )

        self.get_logger().info(
            "solve na /cube_tcp_calibration/command"
        )

        self.get_logger().info(
            "=========================================="
        )

    # ========================================================
    # KOMENDY
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

        # ----------------------------------------------------
        # START
        # ----------------------------------------------------

        if command == "start":

            if self.state not in (
                IDLE,
                DONE,
                ERROR,
            ):

                self.get_logger().warn(
                    "Kalibracja juz trwa."
                )

                return

            self.point_index = 0

            # Czyścimy stare probki.
            clear_msg = String()
            clear_msg.data = "clear"

            self.calibrator_pub.publish(
                clear_msg
            )

            self.get_logger().info(
                "NOWA KALIBRACJA."
            )

            self.begin_current_point()

            return

        # ----------------------------------------------------
        # CAPTURE
        # ----------------------------------------------------

        if command == "capture":

            if (
                self.state
                !=
                WAIT_USER
            ):

                self.get_logger().warn(
                    "Robot nie czeka teraz "
                    "na probke."
                )

                return

            capture_msg = String()
            capture_msg.data = "capture"

            self.calibrator_pub.publish(
                capture_msg
            )

            self.get_logger().info(
                f"CAPTURE punktu "
                f"{self.point_index + 1}."
            )

            self.set_state(
                AFTER_CAPTURE
            )

            return

        # ----------------------------------------------------
        # SKIP
        # ----------------------------------------------------

        if command == "skip":

            if (
                self.state
                !=
                WAIT_USER
            ):

                self.get_logger().warn(
                    "Robot nie czeka teraz "
                    "na punkt."
                )

                return

            self.get_logger().warn(
                f"POMIJAM punkt "
                f"{self.point_index + 1}."
            )

            self.point_index += 1

            self.begin_current_point()

            return

        # ----------------------------------------------------
        # STATUS
        # ----------------------------------------------------

        if command == "status":

            tcp = (
                self.get_tool_position()
            )

            self.get_logger().info(
                ""
            )

            self.get_logger().info(
                f"STATE = {self.state}"
            )

            self.get_logger().info(
                f"POINT = "
                f"{self.point_index + 1}/"
                f"{len(CALIBRATION_POINTS)}"
            )

            if tcp is not None:

                self.get_logger().info(
                    f"TCP="
                    f"({tcp[0]:+.4f}, "
                    f"{tcp[1]:+.4f}, "
                    f"{tcp[2]:+.4f})"
                )

            return

        # ----------------------------------------------------
        # STOP
        # ----------------------------------------------------

        if command == "stop":

            self.set_state(
                IDLE
            )

            self.get_logger().warn(
                "AUTO KALIBRACJA ZATRZYMANA."
            )

            return

        self.get_logger().warn(
            f"Nieznana komenda: {command}"
        )

    # ========================================================
    # UPDATE
    # ========================================================

    def update(self):

        # ====================================================
        # CZEKAMY NA PODNIESIENIE
        # ====================================================

        if self.state == WAIT_LIFT:

            if self.movement_timeout():
                return

            if not self.target_reached():
                return

            point = (
                self.current_point()
            )

            if point is None:

                self.finish_calibration()

                return

            x, y, z = point

            self.get_logger().info(
                "SAFE Z OSIAGNIETE."
            )

            # ------------------------------------------------
            # Przejazd XY na SAFE_Z.
            # ------------------------------------------------

            self.send_target(
                x,
                y,
                SAFE_Z,
            )

            self.set_state(
                WAIT_XY
            )

            return

        # ====================================================
        # CZEKAMY NA XY
        # ====================================================

        if self.state == WAIT_XY:

            if self.movement_timeout():
                return

            if not self.target_reached():
                return

            point = (
                self.current_point()
            )

            if point is None:

                self.finish_calibration()

                return

            x, y, z = point

            self.get_logger().info(
                "XY OSIAGNIETE."
            )

            # ------------------------------------------------
            # Teraz pionowo do PRAWDZIWEGO Z tego punktu.
            # ------------------------------------------------

            self.send_target(
                x,
                y,
                z,
            )

            self.set_state(
                WAIT_DESCEND
            )

            return

        # ====================================================
        # CZEKAMY NA ZJAZD
        # ====================================================

        if self.state == WAIT_DESCEND:

            if self.movement_timeout():
                return

            if not self.target_reached():
                return

            self.set_state(
                WAIT_USER
            )

            self.announce_ready()

            return

        # ====================================================
        # PO CAPTURE
        # ====================================================

        if self.state == AFTER_CAPTURE:

            if (
                self.state_time()
                <
                CAPTURE_DELAY
            ):

                return

            self.point_index += 1

            if (
                self.point_index
                >=
                len(CALIBRATION_POINTS)
            ):

                self.finish_calibration()

                return

            self.begin_current_point()

            return


def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        CubeCalibrationAuto()
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