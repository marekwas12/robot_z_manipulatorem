import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState

import serial


# =====================================================
# SERIAL
# =====================================================

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200


# =====================================================
# KROKOWCE
#
# 200 pelnych krokow / obrot
# mikrokrok 1/8
# przelozenie 1:10.4
#
# 200 * 8 * 10.4 = 16640 krokow / obrot jointa
# =====================================================

STEPS_PER_REV = 23755.0

STEPS_PER_RAD = (
    STEPS_PER_REV /
    (2.0 * math.pi)
)


# =====================================================
# OGRANICZENIE CZESTOTLIWOSCI SERIAL
#
# Controller state moze przychodzic bardzo czesto.
# Arduino nie musi dostawac 100 komend/s.
#
# Na poczatek 20 Hz.
# =====================================================

SERIAL_COMMAND_PERIOD = 0.05


# =====================================================
# MINIMALNA ZMIANA
#
# Jezeli kolejna referencja jest praktycznie taka sama,
# nie wysylamy jej ponownie.
# =====================================================

MIN_STEP_CHANGE = 2

MIN_SERVO_CHANGE_DEG = 0.2


class SerialJointBridge(Node):

    def __init__(self):

        super().__init__(
            "serial_joint_bridge"
        )

        # =================================================
        # STAN
        # =================================================

        self.homed = False
        self.homing_started = False
        self.arduino_ready = False

        self.last_command_time = 0.0

        self.last_sent_command = None

        self.pending_controller_command = None


        self.get_logger().info(
            f"STEPS_PER_RAD = "
            f"{STEPS_PER_RAD:.3f}"
        )


        # =================================================
        # SERIAL
        # =================================================

        self.serial = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUD,
            timeout=0.01,
            write_timeout=0.1,
        )

        self.get_logger().info(
            f"Serial otwarty: "
            f"{SERIAL_PORT} @ {SERIAL_BAUD}"
        )


        # Arduino Mega zwykle resetuje sie
        # po otwarciu USB.
        time.sleep(2.0)


        # =================================================
        # STARY TOPIC TESTOWY
        #
        # Zostawiamy go, bo przydaje sie
        # do recznych testow.
        # =================================================

        self.manual_subscription = (
            self.create_subscription(
                JointState,
                "/robot_joint_targets",
                self.manual_target_callback,
                10,
            )
        )


        # =================================================
        # JOINT TRAJECTORY CONTROLLER
        #
        # To tutaj pojawiaja sie pozycje interpolowane
        # przez arm_controller.
        # =================================================

        self.controller_subscription = (
            self.create_subscription(
                JointTrajectoryControllerState,
                "/arm_controller/controller_state",
                self.controller_state_callback,
                10,
            )
        )


        # =================================================
        # SPRZETOWY STAN JOINTOW Z ARDUINO
        # =================================================

        self.hardware_joint_state_pub = (
            self.create_publisher(
                JointState,
                "/hardware_joint_states",
                10,
            )
        )


        # =================================================
        # SERIAL RX
        # =================================================

        self.serial_timer = (
            self.create_timer(
                0.02,
                self.read_serial,
            )
        )


        # =================================================
        # HOMING
        # =================================================

        self.homing_timer = (
            self.create_timer(
                0.2,
                self.update_homing,
            )
        )


        # =================================================
        # WYSYLANIE REFERENCJI KONTROLERA
        #
        # Maksymalnie 20 Hz.
        # =================================================

        self.command_timer = (
            self.create_timer(
                SERIAL_COMMAND_PERIOD,
                self.send_pending_controller_command,
            )
        )


        self.get_logger().info(
            "Czekam na start Arduino..."
        )


    # =====================================================
    # SERIAL TX
    # =====================================================

    def send_serial(
        self,
        command: str
    ):

        try:

            self.serial.write(
                command.encode("ascii")
            )

            self.get_logger().info(
                f"TX: {command.strip()}"
            )

        except serial.SerialException as exc:

            self.get_logger().error(
                f"Blad Serial TX: {exc}"
            )


    # =====================================================
    # HOMING
    # =====================================================

    def update_homing(self):

        if self.homed:
            return

        if not self.arduino_ready:
            return

        if self.homing_started:
            return


        self.get_logger().info(
            "Arduino gotowe."
        )

        self.get_logger().info(
            "Rozpoczynam automatyczny homing..."
        )


        self.send_serial(
            "H\n"
        )

        self.homing_started = True


    # =====================================================
    # KONWERSJA RAD -> ARDUINO
    # =====================================================

    def positions_to_arduino(
        self,
        positions
    ):

        # ---------------------------------------------
        # J1-J3
        # rad -> kroki
        # ---------------------------------------------

        j1_steps = round(
            positions["joint_1"] *
            STEPS_PER_RAD
        )

        j2_steps = round(
            positions["joint_2"] *
            STEPS_PER_RAD
        )

        j3_steps = round(
            positions["joint_3"] *
            STEPS_PER_RAD
        )


        # ---------------------------------------------
        # J4-J5
        # rad -> stopnie
        # ---------------------------------------------

        j4_deg = math.degrees(
            positions["joint_4"]
        )

        j5_deg = math.degrees(
            positions["joint_5"]
        )


        # ---------------------------------------------
        # HARD LIMITS
        # ---------------------------------------------

        j1_steps = max(
            -15557,
            min(0, j1_steps)
        )

        j2_steps = max(
            0,
            min(10100, j2_steps)
        )

        j3_steps = max(
            0,
            min(11835, j3_steps)
        )

        j4_deg = max(
            5.0,
            min(180.0, j4_deg)
        )

        j5_deg = max(
            0.0,
            min(180.0, j5_deg)
        )


        return (
            j1_steps,
            j2_steps,
            j3_steps,
            j4_deg,
            j5_deg,
        )


    # =====================================================
    # CZY NOWA KOMENDA ROZNI SIE OD POPRZEDNIEJ
    # =====================================================

    def command_changed(
        self,
        command
    ):

        if self.last_sent_command is None:
            return True


        old = self.last_sent_command

        if abs(command[0] - old[0]) >= MIN_STEP_CHANGE:
            return True

        if abs(command[1] - old[1]) >= MIN_STEP_CHANGE:
            return True

        if abs(command[2] - old[2]) >= MIN_STEP_CHANGE:
            return True

        if (
            abs(command[3] - old[3]) >=
            MIN_SERVO_CHANGE_DEG
        ):
            return True

        if (
            abs(command[4] - old[4]) >=
            MIN_SERVO_CHANGE_DEG
        ):
            return True


        return False


    # =====================================================
    # TWORZENIE @J
    # =====================================================

    def make_joint_command(
        self,
        command
    ):

        return (
            "@J:"
            f"{command[0]},"
            f"{command[1]},"
            f"{command[2]},"
            f"{command[3]:.1f},"
            f"{command[4]:.1f}\n"
        )


    # =====================================================
    # STARY RECZNY TOPIC
    # =====================================================

    def manual_target_callback(
        self,
        msg: JointState
    ):

        if not self.homed:

            self.get_logger().warn(
                "Ignoruje /robot_joint_targets - "
                "robot nie jest po homingu"
            )

            return


        if len(msg.name) != len(msg.position):

            self.get_logger().error(
                "Niepoprawny JointState"
            )

            return


        positions = dict(
            zip(
                msg.name,
                msg.position
            )
        )


        required = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
        ]


        for joint in required:

            if joint not in positions:

                self.get_logger().error(
                    f"Brak {joint}"
                )

                return


        command = (
            self.positions_to_arduino(
                positions
            )
        )


        serial_command = (
            self.make_joint_command(
                command
            )
        )


        self.send_serial(
            serial_command
        )

        self.last_sent_command = command


    # =====================================================
    # ARM_CONTROLLER
    #
    # Uzywamy reference.positions.
    #
    # Nie wysylamy od razu.
    # Zapisujemy tylko najnowsza pozycje.
    # Timer wysle ja maksymalnie 20 razy/s.
    # =====================================================

    def controller_state_callback(
        self,
        msg: JointTrajectoryControllerState
    ):

        if not self.homed:
            return


        if not msg.reference.positions:
            return


        if (
            len(msg.joint_names) !=
            len(msg.reference.positions)
        ):
            return


        positions = dict(
            zip(
                msg.joint_names,
                msg.reference.positions
            )
        )


        required = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
        ]


        for joint in required:

            if joint not in positions:
                return


        self.pending_controller_command = (
            self.positions_to_arduino(
                positions
            )
        )


    # =====================================================
    # TIMER TX DLA ARM_CONTROLLER
    # =====================================================

    def send_pending_controller_command(self):

        if not self.homed:
            return


        if self.pending_controller_command is None:
            return


        command = (
            self.pending_controller_command
        )


        if not self.command_changed(
            command
        ):
            return


        serial_command = (
            self.make_joint_command(
                command
            )
        )


        self.send_serial(
            serial_command
        )


        self.last_sent_command = command


    # =====================================================
    # PARSOWANIE RAPORTU POZYCJI @P Z ARDUINO
    #
    # Format:
    # @P:J1_steps,J2_steps,J3_steps,J4_deg,J5_deg
    # =====================================================

    def handle_position_report(
        self,
        text: str
    ):

        if not text.startswith("@P:"):
            return

        payload = text[3:]
        parts = payload.split(",")

        if len(parts) != 5:

            self.get_logger().error(
                f"Niepoprawny raport @P: {text}"
            )

            return


        try:

            j1_steps = int(parts[0])
            j2_steps = int(parts[1])
            j3_steps = int(parts[2])

            j4_deg = float(parts[3])
            j5_deg = float(parts[4])

        except ValueError:

            self.get_logger().error(
                f"Niepoprawne liczby w @P: {text}"
            )

            return


        # =============================================
        # KROKI -> RAD
        # =============================================

        j1_rad = (
            j1_steps /
            STEPS_PER_RAD
        )

        j2_rad = (
            j2_steps /
            STEPS_PER_RAD
        )

        j3_rad = (
            j3_steps /
            STEPS_PER_RAD
        )


        # =============================================
        # STOPNIE -> RAD
        # =============================================

        j4_rad = math.radians(
            j4_deg
        )

        j5_rad = math.radians(
            j5_deg
        )


        # =============================================
        # PUBLIKACJA PRAWDZIWEGO STANU Z ARDUINO
        # =============================================

        msg = JointState()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.name = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
        ]

        msg.position = [
            j1_rad,
            j2_rad,
            j3_rad,
            j4_rad,
            j5_rad,
        ]

        self.hardware_joint_state_pub.publish(
            msg
        )


    # =====================================================
    # SERIAL RX
    # =====================================================

    def read_serial(self):

        try:

            while (
                self.serial.in_waiting > 0
            ):

                raw_line = (
                    self.serial.readline()
                )

                if not raw_line:
                    return


                text = raw_line.decode(
                    "utf-8",
                    errors="replace"
                ).strip()


                if not text:
                    continue


                # -------------------------------------
                # Raport pozycji z Arduino
                # -------------------------------------

                if text.startswith("@P:"):

                    self.handle_position_report(
                        text
                    )

                    continue


                self.get_logger().info(
                    f"ARDUINO: {text}"
                )


                # -------------------------------------
                # Arduino wystartowalo
                # -------------------------------------

                if (
                    "Manipulator gotowy"
                    in text
                ):

                    self.arduino_ready = True


                # -------------------------------------
                # Homing zakonczony
                # -------------------------------------

                if (
                    "HOMING GOTOWY"
                    in text
                ):

                    self.homed = True

                    self.pending_controller_command = None
                    self.last_sent_command = None


                    self.get_logger().info(
                        "=============================="
                    )

                    self.get_logger().info(
                        "HOMING GOTOWY"
                    )

                    self.get_logger().info(
                        "Sterowanie z arm_controller "
                        "odblokowane"
                    )

                    self.get_logger().info(
                        "=============================="
                    )


                # -------------------------------------
                # Blad homingu
                # -------------------------------------

                if (
                    "HOMING ERROR"
                    in text
                ):

                    self.homed = False

                    self.homing_started = False

                    self.pending_controller_command = None

                    self.get_logger().error(
                        "BLAD HOMINGU"
                    )


                if (
                    "HOMING PRZERWANY"
                    in text
                ):

                    self.homed = False

                    self.homing_started = False

                    self.pending_controller_command = None

                    self.get_logger().error(
                        "HOMING PRZERWANY"
                    )


                # -------------------------------------
                # Arduino twierdzi, ze nie jest homed
                # -------------------------------------

                if (
                    "\"error\":\"not_homed\""
                    in text
                ):

                    self.homed = False

                    self.homing_started = False

                    self.pending_controller_command = None

                    self.last_sent_command = None

                    self.get_logger().error(
                        "Arduino utracilo stan homingu"
                    )


        except serial.SerialException as exc:

            self.get_logger().error(
                f"Blad Serial RX: {exc}"
            )


    # =====================================================
    # SHUTDOWN
    # =====================================================

    def destroy_node(self):

        try:

            if self.serial.is_open:
                self.serial.close()

        except serial.SerialException:
            pass


        super().destroy_node()


def main(args=None):

    rclpy.init(
        args=args
    )


    node = SerialJointBridge()


    try:

        rclpy.spin(
            node
        )


    except KeyboardInterrupt:

        pass


    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()