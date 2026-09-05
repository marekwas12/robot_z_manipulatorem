import json
import math
import threading
import time

import rclpy

from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory

from sensor_msgs.msg import JointState, Range
from std_msgs.msg import String

from trajectory_msgs.msg import JointTrajectoryPoint

import serial


# ============================================================
# SERIAL
# ============================================================

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200


# ============================================================
# MECHANIKA
# ============================================================

STEPS_PER_REV = 16640.0

STEPS_PER_RAD = (
    STEPS_PER_REV
    /
    (
        2.0
        *
        math.pi
    )
)


# ============================================================
# CZESTOTLIWOSC WYSYLANIA
#
# 0.05 s = 20 Hz
# ============================================================

COMMAND_PERIOD = 0.05

# ============================================================
# J5 / SERWO 4 - ABSOLUTNY ZAKRES MECHANICZNY
# ============================================================
# Fizyczne Serwo 4 moze poruszac sie TYLKO 52..112 stopni.
# W calym ROS traktujemy joint_5 jako dodatni fizyczny kat serwa.
J5_SERVO_MIN_DEG = 52.0
J5_SERVO_CENTER_DEG = 82.0
J5_SERVO_MAX_DEG = 112.0


# ============================================================
# JOINTY
# ============================================================

REQUIRED_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
]


# ============================================================
# KOMENDY MANUALNE
# ============================================================

ALLOWED_MANUAL_COMMANDS = {

    "V",

    # HOMING
    "H",

    # tryb platformy
    "A",
    "M",

    # manipulator
    "w",
    "s",
    "a",
    "d",
    "f",
    "g",
    "x",
    "z",

    # dodatkowe
    "t",
    "y",
    "u",
    "i",

    # serwa
    "o",
    "p",
    "h",
    "j",
    "k",
    "l",

    # jazda
    "0",
    "1",
    "2",
    "3",
    "4",

    # wentylatory / buzzer
    "5",
    "6",
    "7",
    "8",
}


# ============================================================
# HC-SR04
# ============================================================
#
# Arduino wysyla:
# {
#   "ultrasonic": true,
#   "frontLeft": cm,
#   "frontCenter": cm,
#   "frontRight": cm,
#   "right": cm,
#   "left": cm,
#   "rear": cm
# }
#
# W ROS publikujemy:
#   /robot/ultrasonic           std_msgs/String  (surowy JSON)
#   /ultrasonic/front_left      sensor_msgs/Range
#   /ultrasonic/front_center    sensor_msgs/Range
#   /ultrasonic/front_right     sensor_msgs/Range
#   /ultrasonic/right           sensor_msgs/Range
#   /ultrasonic/left            sensor_msgs/Range
#   /ultrasonic/rear            sensor_msgs/Range
# ============================================================

ULTRASONIC_MIN_RANGE_M = 0.02
ULTRASONIC_MAX_RANGE_M = 4.00

# Przyblizone pole widzenia HC-SR04.
ULTRASONIC_FOV_RAD = math.radians(30.0)

ULTRASONIC_TOPICS = {
    "frontLeft": (
        "/ultrasonic/front_left",
        "ultrasonic_front_left",
    ),
    "frontCenter": (
        "/ultrasonic/front_center",
        "ultrasonic_front_center",
    ),
    "frontRight": (
        "/ultrasonic/front_right",
        "ultrasonic_front_right",
    ),
    "right": (
        "/ultrasonic/right",
        "ultrasonic_right",
    ),
    "left": (
        "/ultrasonic/left",
        "ultrasonic_left",
    ),
    "rear": (
        "/ultrasonic/rear",
        "ultrasonic_rear",
    ),
}


class SerialTrajectoryBridge(Node):

    def __init__(self):

        super().__init__(
            "serial_trajectory_bridge"
        )

        # ====================================================
        # STAN
        # ====================================================

        self.homed = False
        self.homing_started = False
        self.arduino_ready = False
        self.have_position = False

        self.positions = {
            "joint_1": 0.0,
            "joint_2": 0.0,
            "joint_3": 0.0,
            "joint_4": 0.0,
            "joint_5": 0.0,
        }

        self.serial_lock = (
            threading.Lock()
        )

        # ====================================================
        # SERIAL
        # ====================================================

        self.serial = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUD,
            timeout=0.01,
            write_timeout=0.1,
        )

        self.get_logger().info(
            f"Serial otwarty: "
            f"{SERIAL_PORT} "
            f"@ {SERIAL_BAUD}"
        )

        self.get_logger().info(
            f"STEPS_PER_RAD = "
            f"{STEPS_PER_RAD:.3f}"
        )

        # Arduino resetuje sie po otwarciu portu.
        time.sleep(
            2.0
        )

        # ====================================================
        # JOINT STATES
        # ====================================================

        self.joint_state_pub = (
            self.create_publisher(
                JointState,
                "/joint_states",
                10,
            )
        )

        # ====================================================
        # STATUS ROBOTA
        # ====================================================

        self.status_pub = (
            self.create_publisher(
                String,
                "/robot/status",
                10,
            )
        )


        # ====================================================
        # HC-SR04
        # ====================================================

        self.ultrasonic_raw_pub = (
            self.create_publisher(
                String,
                "/robot/ultrasonic",
                10,
            )
        )

        self.ultrasonic_publishers = {}

        for key, (
            topic,
            _frame_id,
        ) in ULTRASONIC_TOPICS.items():

            self.ultrasonic_publishers[key] = (
                self.create_publisher(
                    Range,
                    topic,
                    10,
                )
            )

        self.get_logger().info(
            "HC-SR04 ROS: /robot/ultrasonic + 6x sensor_msgs/Range"
        )

        # ====================================================
        # KOMENDY TELEFONU
        # ====================================================

        self.command_sub = (
            self.create_subscription(
                String,
                "/manipulator/command",
                self.command_callback,
                10,
            )
        )

        self.get_logger().info(
            "Nasluchuje: /manipulator/command"
        )

        self.get_logger().info(
            "Komendy A/M wlaczaja/wylaczaja tryb autonomiczny Arduino."
        )

        # ====================================================
        # SERIAL RX
        # ====================================================

        self.serial_timer = (
            self.create_timer(
                0.02,
                self.read_serial,
            )
        )

        # ====================================================
        # HOMING
        # ====================================================

        self.homing_timer = (
            self.create_timer(
                0.2,
                self.update_homing,
            )
        )

        # ====================================================
        # FOLLOW JOINT TRAJECTORY
        # ====================================================

        self.action_server = (
            ActionServer(
                self,
                FollowJointTrajectory,
                "/arm_controller/"
                "follow_joint_trajectory",

                execute_callback=
                    self.execute_callback,

                goal_callback=
                    self.goal_callback,

                cancel_callback=
                    self.cancel_callback,
            )
        )

        self.get_logger().info(
            "Action server:"
        )

        self.get_logger().info(
            "/arm_controller/"
            "follow_joint_trajectory"
        )

        self.get_logger().info(
            "COMMAND_PERIOD = "
            f"{COMMAND_PERIOD:.3f} s "
            f"({1.0 / COMMAND_PERIOD:.1f} Hz)"
        )

        self.get_logger().info(
            "Jednopunktowe trajectory: "
            "QUINTIC SMOOTHSTEP"
        )

        self.get_logger().info(
            "Czekam na Arduino i homing..."
        )

    # ========================================================
    # SERIAL TX
    # ========================================================

    def send_serial(
        self,
        text,
    ):

        try:

            with self.serial_lock:

                self.serial.write(
                    text.encode(
                        "ascii"
                    )
                )

                self.serial.flush()

        except (
            serial.SerialException,
            OSError,
        ) as exc:

            self.get_logger().error(
                f"Serial TX error: {exc}"
            )

    # ========================================================
    # KOMENDY MANUALNE
    # ========================================================













    def command_callback(
        self,
        msg,
    ):

        command = (
            msg.data
            .strip()
        )




                # ====================================================
        # KOMENDA AUTONOMICZNA DO ARDUINO
        #
        # Przyklady:
        # @S5:54
        # @S5:72
        # ====================================================

        if command.startswith("@"):

            # Twarda ochrona bezposrednich komend do J5 / Serwo 4.
            # Nawet jezeli inny node wysle @S4 poza zakresem,
            # bridge nie przepusci wartosci <52 ani >112 stopni.
            if command.startswith("@S4:"):
                try:
                    requested = float(command.split(":", 1)[1])
                except ValueError:
                    self.get_logger().error(
                        f"Niepoprawna komenda J5: {command}"
                    )
                    return

                safe = max(
                    J5_SERVO_MIN_DEG,
                    min(J5_SERVO_MAX_DEG, requested),
                )

                if abs(safe - requested) > 1e-9:
                    self.get_logger().warn(
                        f"J5 HARD LIMIT: @S4 {requested:.1f} -> {safe:.1f} deg"
                    )

                command = f"@S4:{safe:.1f}"

            self.get_logger().info(
                f"ROS -> ARDUINO AUTO: {command}"
            )

            self.send_serial(
                command + "\n"
            )

            return



        if len(command) != 1:

            self.get_logger().warn(
                "Odrzucono komende "
                "manualna o zlej dlugosci: "
                f"{command}"
            )

            return

        if (
            command
            not in
            ALLOWED_MANUAL_COMMANDS
        ):

            self.get_logger().warn(
                "Nieznana komenda telefonu: "
                f"{command}"
            )

            return

        # ====================================================
        # HOMING
        # ====================================================

        if command == "H":

            self.get_logger().warn(
                "HOMING z aplikacji"
            )

            self.homed = False
            self.have_position = False
            self.homing_started = True

            self.send_serial(
                "H\n"
            )

            return

        # ====================================================
        # ZWYKLA KOMENDA 1-ZNAKOWA
        # ====================================================

        self.get_logger().info(
            f"TELEFON -> ARDUINO: "
            f"{command}"
        )

        self.send_serial(
            command
        )

    # ========================================================
    # HOMING
    # ========================================================

    def update_homing(
        self,
    ):

        if self.homed:
            return

        if not self.arduino_ready:
            return

        if self.homing_started:
            return

        self.get_logger().info(
            "Rozpoczynam homing..."
        )

        self.send_serial(
            "H\n"
        )

        self.homing_started = True

    # ========================================================
    # @P Z ARDUINO
    # ========================================================

    def handle_position_report(
        self,
        text,
    ):

        if not text.startswith(
            "@P:"
        ):

            return

        parts = (
            text[3:]
            .split(",")
        )

        if len(parts) != 5:
            return

        try:

            j1_steps = int(
                parts[0]
            )

            j2_steps = int(
                parts[1]
            )

            j3_steps = int(
                parts[2]
            )

            j4_deg = float(
                parts[3]
            )

            j5_deg = float(
                parts[4]
            )

        except ValueError:
            return

        self.positions[
            "joint_1"
        ] = (
            j1_steps
            /
            STEPS_PER_RAD
        )

        self.positions[
            "joint_2"
        ] = (
            j2_steps
            /
            STEPS_PER_RAD
        )

        self.positions[
            "joint_3"
        ] = (
            j3_steps
            /
            STEPS_PER_RAD
        )

        self.positions[
            "joint_4"
        ] = math.radians(
            j4_deg
        )

        # Arduino moze raportowac J5 ze znakiem zaleznym od firmware.
        # Dla reszty ROS interesuje nas fizyczny kat Serwo 4.
        j5_servo_deg = abs(float(j5_deg))
        j5_servo_deg = max(
            J5_SERVO_MIN_DEG,
            min(J5_SERVO_MAX_DEG, j5_servo_deg),
        )

        self.positions[
            "joint_5"
        ] = math.radians(
            j5_servo_deg
        )

        self.have_position = True

        # ====================================================
        # /joint_states
        # ====================================================

        msg = JointState()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.name = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "gripper_joint",
        ]

        msg.position = [
            self.positions[
                "joint_1"
            ],

            self.positions[
                "joint_2"
            ],

            self.positions[
                "joint_3"
            ],

            self.positions[
                "joint_4"
            ],

            self.positions[
                "joint_5"
            ],

            0.4,
        ]

        self.joint_state_pub.publish(
            msg
        )

    # ========================================================
    # STATUS JSON
    # ========================================================

    def handle_robot_status(
        self,
        text,
    ):

        if not (
            text.startswith("{")
            and
            text.endswith("}")
        ):

            return False

        if (
            "\"upperTemp\""
            not in text
        ):

            return False

        msg = String()

        msg.data = text

        self.status_pub.publish(
            msg
        )

        return True

    # ========================================================
    # SERIAL RX
    # ========================================================

    # ========================================================
    # HC-SR04 JSON
    # ========================================================

    def publish_ultrasonic_range(
        self,
        key,
        distance_cm,
    ):

        if key not in ULTRASONIC_TOPICS:
            return

        try:
            distance_cm = float(
                distance_cm
            )
        except (
            TypeError,
            ValueError,
        ):
            return

        # Arduino uzywa -1 przy braku poprawnego echa.
        if distance_cm < 0.0:
            distance_m = float("inf")
        else:
            distance_m = (
                distance_cm
                /
                100.0
            )

        topic, frame_id = (
            ULTRASONIC_TOPICS[key]
        )

        msg = Range()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            frame_id
        )

        msg.radiation_type = (
            Range.ULTRASOUND
        )

        msg.field_of_view = (
            ULTRASONIC_FOV_RAD
        )

        msg.min_range = (
            ULTRASONIC_MIN_RANGE_M
        )

        msg.max_range = (
            ULTRASONIC_MAX_RANGE_M
        )

        msg.range = float(
            distance_m
        )

        self.ultrasonic_publishers[
            key
        ].publish(
            msg
        )


    def handle_ultrasonic_status(
        self,
        text,
    ):

        if not (
            text.startswith("{")
            and
            text.endswith("}")
        ):
            return False

        if (
            '"ultrasonic"'
            not in text
        ):
            return False

        try:

            data = json.loads(
                text
            )

        except json.JSONDecodeError:

            self.get_logger().warn(
                "Niepoprawny JSON HC-SR04: "
                f"{text}"
            )

            return True

        if not data.get(
            "ultrasonic",
            False,
        ):
            return False

        # Surowy JSON przydaje sie do debugowania
        # oraz przyszlego autonomous-cleanup node.
        raw_msg = String()
        raw_msg.data = text

        self.ultrasonic_raw_pub.publish(
            raw_msg
        )

        for key in ULTRASONIC_TOPICS:

            if key not in data:
                continue

            self.publish_ultrasonic_range(
                key,
                data[key],
            )

        return True


    def read_serial(
        self,
    ):

        try:

            while True:

                with self.serial_lock:

                    if (
                        self.serial
                        .in_waiting
                        <= 0
                    ):

                        break

                    raw = (
                        self.serial
                        .readline()
                    )

                if not raw:
                    break

                text = raw.decode(
                    "utf-8",
                    errors="replace",
                ).strip()

                if not text:
                    continue

                # ============================================
                # POZYCJE
                # ============================================

                if text.startswith(
                    "@P:"
                ):

                    self.handle_position_report(
                        text
                    )

                    continue

                # ============================================
                # HC-SR04
                # ============================================

                if self.handle_ultrasonic_status(
                    text
                ):

                    continue

                # ============================================
                # STATUS TELEFONU
                # ============================================

                if self.handle_robot_status(
                    text
                ):

                    continue

                # ============================================
                # LOG
                # ============================================

                self.get_logger().info(
                    f"ARDUINO: {text}"
                )

                # ============================================
                # ARDUINO READY
                # ============================================

                if (
                    "Manipulator gotowy"
                    in text
                ):

                    self.arduino_ready = True

                # ============================================
                # HOMING GOTOWY
                # ============================================

                if (
                    "HOMING GOTOWY"
                    in text
                ):

                    self.homed = True
                    self.have_position = False

                    self.get_logger().info(
                        "=============================="
                    )

                    self.get_logger().info(
                        "HOMING GOTOWY"
                    )

                    self.get_logger().info(
                        "Czekam na @P..."
                    )

                    self.get_logger().info(
                        "=============================="
                    )

                # ============================================
                # HOMING ERROR
                # ============================================

                if (
                    "HOMING ERROR"
                    in text

                    or

                    "HOMING PRZERWANY"
                    in text

                    or

                    "\"error\":\"not_homed\""
                    in text
                ):

                    self.homed = False
                    self.have_position = False
                    self.homing_started = False

                    self.get_logger().error(
                        "Robot utracil homing"
                    )

        except (
            serial.SerialException,
            OSError,
        ) as exc:

            self.get_logger().error(
                f"Serial RX error: {exc}"
            )

    # ========================================================
    # RAD -> @J
    # ========================================================

    def make_command(
        self,
        positions,
    ):

        j1 = round(
            positions[
                "joint_1"
            ]
            *
            STEPS_PER_RAD
        )

        j2 = round(
            positions[
                "joint_2"
            ]
            *
            STEPS_PER_RAD
        )

        j3 = round(
            positions[
                "joint_3"
            ]
            *
            STEPS_PER_RAD
        )

        j4 = math.degrees(
            positions[
                "joint_4"
            ]
        )

        # joint_5 w ROS = dodatni fizyczny kat Serwo 4.
        # abs() dodatkowo zabezpiecza przed starym nodem wysylajacym znak minus.
        j5 = abs(
            math.degrees(
                positions[
                    "joint_5"
                ]
            )
        )

        # ====================================================
        # LIMITY
        # ====================================================

        j1 = max(
            -15557,
            min(
                0,
                j1,
            ),
        )

        j2 = max(
            0,
            min(
                10100,
                j2,
            ),
        )

        j3 = max(
            0,
            min(
                11835,
                j3,
            ),
        )

        j4 = max(
            5.0,
            min(
                180.0,
                j4,
            ),
        )

        # OSTATECZNY HARD LIMIT bezposrednio przed @J.
        j5 = max(
            J5_SERVO_MIN_DEG,
            min(
                J5_SERVO_MAX_DEG,
                j5,
            ),
        )

        return (
            f"@J:"
            f"{j1},"
            f"{j2},"
            f"{j3},"
            f"{j4:.1f},"
            f"{j5:.1f}\n"
        )

    # ========================================================
    # ACTION GOAL
    # ========================================================

    def goal_callback(
        self,
        goal_request,
    ):

        if not self.homed:

            self.get_logger().error(
                "Odrzucam trajectory - "
                "robot nie jest po homingu"
            )

            return GoalResponse.REJECT

        if not self.have_position:

            self.get_logger().error(
                "Odrzucam trajectory - "
                "brak aktualnego @P"
            )

            return GoalResponse.REJECT

        names = (
            goal_request
            .trajectory
            .joint_names
        )

        if set(names) != set(
            REQUIRED_JOINTS
        ):

            self.get_logger().error(
                f"Niepoprawne joint_names: "
                f"{names}"
            )

            return GoalResponse.REJECT

        if not (
            goal_request
            .trajectory
            .points
        ):

            return GoalResponse.REJECT

        # Dodatkowa walidacja wszystkich punktow trajectory dla J5.
        try:
            j5_index = list(names).index("joint_5")
        except ValueError:
            self.get_logger().error("Brak joint_5 w trajectory.")
            return GoalResponse.REJECT

        for point in goal_request.trajectory.points:
            if len(point.positions) <= j5_index:
                self.get_logger().error("Za malo pozycji w trajectory.")
                return GoalResponse.REJECT

            requested_j5_deg = abs(
                math.degrees(float(point.positions[j5_index]))
            )

            if not (
                J5_SERVO_MIN_DEG - 1e-6
                <= requested_j5_deg
                <= J5_SERVO_MAX_DEG + 1e-6
            ):
                self.get_logger().error(
                    f"Odrzucam trajectory: J5={requested_j5_deg:.2f} deg, "
                    f"dozwolone tylko {J5_SERVO_MIN_DEG:.0f}.."
                    f"{J5_SERVO_MAX_DEG:.0f} deg."
                )
                return GoalResponse.REJECT

        self.get_logger().info(
            "Trajectory zaakceptowana"
        )

        return GoalResponse.ACCEPT

    # ========================================================
    # CANCEL
    # ========================================================

    def cancel_callback(
        self,
        goal_handle,
    ):

        self.get_logger().warn(
            "Zadano anulowanie trajectory"
        )

        return CancelResponse.ACCEPT

    # ========================================================
    # CZAS PUNKTU
    # ========================================================

    @staticmethod
    def point_time(
        point,
    ):

        return (
            float(
                point.time_from_start.sec
            )

            +

            float(
                point.time_from_start.nanosec
            )
            /
            1e9
        )

    # ========================================================
    # QUINTIC SMOOTHSTEP
    #
    # 0 -> 1
    #
    # predkosc = 0 na poczatku
    # predkosc = 0 na koncu
    #
    # przyspieszenie = 0 na poczatku
    # przyspieszenie = 0 na koncu
    # ========================================================

    @staticmethod
    def smoothstep_quintic(
        alpha,
    ):

        alpha = max(
            0.0,
            min(
                1.0,
                alpha,
            ),
        )

        return (
            10.0
            *
            alpha**3

            -

            15.0
            *
            alpha**4

            +

            6.0
            *
            alpha**5
        )

    # ========================================================
    # INTERPOLACJA
    # ========================================================

    def interpolate(
        self,
        names,
        start_positions,
        points,
        t,
    ):

        # ====================================================
        # SPECJALNY PRZYPADEK:
        #
        # JEDEN PUNKT KONCOWY
        #
        # READY / PICK
        #
        # Tutaj uzywamy quintic.
        # ====================================================

        if len(points) == 1:

            final_point = (
                points[0]
            )

            duration = (
                self.point_time(
                    final_point
                )
            )

            if duration <= 0.0:

                return dict(
                    zip(
                        names,
                        final_point.positions,
                    )
                )

            alpha = (
                t
                /
                duration
            )

            smooth_alpha = (
                self.smoothstep_quintic(
                    alpha
                )
            )

            result = {}

            for i, name in enumerate(
                names
            ):

                start = (
                    start_positions[
                        name
                    ]
                )

                target = (
                    final_point
                    .positions[
                        i
                    ]
                )

                result[
                    name
                ] = (
                    start

                    +

                    smooth_alpha
                    *
                    (
                        target
                        -
                        start
                    )
                )

            return result

        # ====================================================
        # WIELE PUNKTOW
        #
        # Dla trajectory z MoveIt zachowujemy
        # interpolacje liniowa miedzy punktami.
        #
        # Nie robimy quintic na kazdym segmencie,
        # bo wtedy robot zwalnialby przy kazdym waypoint.
        # ====================================================

        first_t = (
            self.point_time(
                points[0]
            )
        )

        if t <= first_t:

            if first_t <= 0.0:

                return dict(
                    zip(
                        names,
                        points[
                            0
                        ].positions,
                    )
                )

            alpha = max(
                0.0,
                min(
                    1.0,
                    t
                    /
                    first_t,
                ),
            )

            result = {}

            for i, name in enumerate(
                names
            ):

                start = (
                    start_positions[
                        name
                    ]
                )

                target = (
                    points[
                        0
                    ]
                    .positions[
                        i
                    ]
                )

                result[
                    name
                ] = (
                    start

                    +

                    alpha
                    *
                    (
                        target
                        -
                        start
                    )
                )

            return result

        # ====================================================
        # KOLEJNE SEGMENTY
        # ====================================================

        for index in range(
            1,
            len(
                points
            ),
        ):

            previous = (
                points[
                    index - 1
                ]
            )

            current = (
                points[
                    index
                ]
            )

            t0 = (
                self.point_time(
                    previous
                )
            )

            t1 = (
                self.point_time(
                    current
                )
            )

            if t <= t1:

                if t1 <= t0:

                    alpha = 1.0

                else:

                    alpha = (
                        (
                            t
                            -
                            t0
                        )
                        /
                        (
                            t1
                            -
                            t0
                        )
                    )

                alpha = max(
                    0.0,
                    min(
                        1.0,
                        alpha,
                    ),
                )

                result = {}

                for i, name in enumerate(
                    names
                ):

                    p0 = (
                        previous
                        .positions[
                            i
                        ]
                    )

                    p1 = (
                        current
                        .positions[
                            i
                        ]
                    )

                    result[
                        name
                    ] = (
                        p0

                        +

                        alpha
                        *
                        (
                            p1
                            -
                            p0
                        )
                    )

                return result

        # ====================================================
        # PO TRAJEKTORII
        # ====================================================

        return dict(
            zip(
                names,
                points[
                    -1
                ].positions,
            )
        )

    # ========================================================
    # ACTION EXECUTE
    # ========================================================

    def execute_callback(
        self,
        goal_handle,
    ):

        trajectory = (
            goal_handle
            .request
            .trajectory
        )

        names = list(
            trajectory.joint_names
        )

        points = list(
            trajectory.points
        )

        start_positions = {
            name:
                self.positions[
                    name
                ]

            for name
            in REQUIRED_JOINTS
        }

        duration = (
            self.point_time(
                points[
                    -1
                ]
            )
        )

        self.get_logger().info(
            f"Wykonuje trajectory: "
            f"{duration:.3f} s"
        )

        if len(points) == 1:

            self.get_logger().info(
                "Profil ruchu: "
                "QUINTIC SMOOTHSTEP"
            )

        else:

            self.get_logger().info(
                f"Trajectory ma "
                f"{len(points)} punktow"
            )

        start_time = (
            time.monotonic()
        )

        # ====================================================
        # REALIZACJA
        # ====================================================

        while True:

            # =================================================
            # CANCEL
            # =================================================

            if (
                goal_handle
                .is_cancel_requested
            ):

                self.send_serial(
                    "x"
                )

                goal_handle.canceled()

                result = (
                    FollowJointTrajectory
                    .Result()
                )

                result.error_code = 0

                result.error_string = (
                    "Trajectory canceled"
                )

                return result

            # =================================================
            # CZAS
            # =================================================

            elapsed = (
                time.monotonic()
                -
                start_time
            )

            if elapsed >= duration:
                break

            # =================================================
            # CEL
            # =================================================

            target = (
                self.interpolate(
                    names,
                    start_positions,
                    points,
                    elapsed,
                )
            )

            # =================================================
            # SERIAL
            # =================================================

            self.send_serial(
                self.make_command(
                    target
                )
            )

            # =================================================
            # FEEDBACK
            # =================================================

            feedback = (
                FollowJointTrajectory
                .Feedback()
            )

            feedback.joint_names = list(
                REQUIRED_JOINTS
            )

            actual = (
                JointTrajectoryPoint()
            )

            actual.positions = [
                self.positions[
                    joint
                ]

                for joint
                in REQUIRED_JOINTS
            ]

            feedback.actual = (
                actual
            )

            goal_handle.publish_feedback(
                feedback
            )

            # =================================================
            # 20 Hz
            # =================================================

            time.sleep(
                COMMAND_PERIOD
            )

        # ====================================================
        # OSTATNI PUNKT
        # ====================================================

        final_target = dict(
            zip(
                names,
                points[
                    -1
                ].positions,
            )
        )

        self.send_serial(
            self.make_command(
                final_target
            )
        )

        # ====================================================
        # CZEKAMY NA FIZYCZNE DOJSCIE
        # ====================================================

        deadline = (
            time.monotonic()
            +
            5.0
        )

        while (
            time.monotonic()
            <
            deadline
        ):

            all_ok = True

            for joint in REQUIRED_JOINTS:

                error = abs(
                    self.positions[
                        joint
                    ]

                    -

                    final_target[
                        joint
                    ]
                )

                if joint in (
                    "joint_1",
                    "joint_2",
                    "joint_3",
                ):

                    tolerance = (
                        15.0
                        /
                        STEPS_PER_RAD
                    )

                else:

                    tolerance = (
                        math.radians(
                            1.0
                        )
                    )

                if error > tolerance:

                    all_ok = False

                    break

            if all_ok:
                break

            time.sleep(
                0.05
            )

        # ====================================================
        # DONE / REALNE DOJŚCIE DO CELU
        # ====================================================

        if not all_ok:
            self.get_logger().error(
                "Trajectory TIMEOUT: robot fizycznie nie osiagnal celu. "
                "NIE oznaczam goal jako SUCCESS."
            )

            goal_handle.abort()

            result = (
                FollowJointTrajectory
                .Result()
            )

            # FollowJointTrajectory: GOAL_TOLERANCE_VIOLATED = -5.
            result.error_code = -5
            result.error_string = (
                "Physical target not reached within tolerance"
            )

            return result

        goal_handle.succeed()

        result = (
            FollowJointTrajectory
            .Result()
        )

        result.error_code = 0

        result.error_string = (
            "Trajectory completed"
        )

        self.get_logger().info(
            "Trajectory zakonczona - fizyczny cel osiagniety"
        )

        return result

    # ========================================================
    # SHUTDOWN
    # ========================================================

    def destroy_node(
        self,
    ):

        try:

            if self.serial.is_open:

                self.serial.close()

        except (
            serial.SerialException,
            OSError,
        ):

            pass

        super().destroy_node()


def main(
    args=None,
):

    rclpy.init(
        args=args
    )

    node = (
        SerialTrajectoryBridge()
    )

    executor = (
        MultiThreadedExecutor(
            num_threads=4
        )
    )

    executor.add_node(
        node
    )

    try:

        executor.spin()

    except KeyboardInterrupt:

        pass

    finally:

        executor.shutdown()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()