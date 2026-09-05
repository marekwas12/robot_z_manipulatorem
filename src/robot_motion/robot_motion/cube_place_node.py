import math

import numpy as np
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String


# ============================================================
# TOPIKI / ACTION
# ============================================================

COMMAND_TOPIC = "/cube_place/command"
STATUS_TOPIC = "/cube_place/status"
ARDUINO_COMMAND_TOPIC = "/manipulator/command"
ACTION_NAME = "/arm_controller/follow_joint_trajectory"


# ============================================================
# JOINTY
# ============================================================

JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
]


# ============================================================
# CHWYTAK
# ============================================================

GRIPPER_SERVO = 5
GRIPPER_OPEN_ANGLE_DEG = 54.0

OPEN_SETTLE_S = 0.70


# ============================================================
# PUNKTY ZMIERZONE PRZEZ UZYTKOWNIKA
#
# P1 = pozycja po podniesieniu kostki
# P2 = obrot / transport w strone bagaznika
# P3 = pozycja odkladania
# P4 = otwarcie chwytaka w P3
# P5 = READY
#
# Uzywamy BEZPOSREDNIO zmierzonych joint_states.
# Nie przeliczamy tych punktow drugi raz przez IK.
# ============================================================

P1 = np.array(
    [
        -3.911887006152675,
        +1.6115766160482257,
        +3.942849818363536,
        +2.548180707911721,
        +1.4311699866353502,
    ],
    dtype=float,
)

P2 = np.array(
    [
        -0.8820625527386726,
        +1.6115766160482257,
        +3.942849818363536,
        +2.548180707911721,
        +1.4311699866353502,
    ],
    dtype=float,
)

# Nowy fizycznie zapisany punkt odłożenia do bagażnika.
# Pochodzi bezpośrednio z /joint_states.
P3 = np.array(
    [
        -0.7835101870431275,
        +1.947258811846462,
        +3.6570101983193686,
        +2.426007660272118,
        +1.4311699866353502,
    ],
    dtype=float,
)

READY = np.array(
    [
        -3.9035799101936637,
        +1.9755784571612738,
        +3.6245370050250507,
        +2.530727415391778,
        +1.4311699866353502,
    ],
    dtype=float,
)


# ============================================================
# CZASY RUCHU
#
# P1 -> P2 to duzy obrot J1, dlatego dajemy mu wiecej czasu.
# ============================================================

TIME_TO_P1_S = 2.5
TIME_P1_TO_P2_S = 5.0
TIME_P2_TO_P3_S = 2.5
TIME_P3_TO_READY_S = 5.0

JOINT_REACHED_TOL_RAD = math.radians(2.0)
MOVE_TIMEOUT_MARGIN_S = 3.0


IDLE = 0
MOVE_P1 = 1
MOVE_P2 = 2
WAIT_PICK_CHECK = 3
MOVE_P3 = 4
OPEN_GRIPPER = 5
MOVE_READY = 6
ABORT_TO_READY = 7
DONE = 8
ERROR = 9


class CubePlaceNode(Node):

    def __init__(self):
        super().__init__("cube_place_node")

        self.current_q = None

        self.state = IDLE
        self.state_started = self.get_clock().now()

        self.goal_active = False
        self.goal_done = False
        self.goal_ok = False
        self.goal_label = ""
        self.goal_target = None
        self.goal_expected_time = 0.0

        self.open_command_sent = False
        self.skip_pick_check = False

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            30,
        )

        self.create_subscription(
            String,
            COMMAND_TOPIC,
            self.command_callback,
            10,
        )

        self.arduino_pub = self.create_publisher(
            String,
            ARDUINO_COMMAND_TOPIC,
            10,
        )

        # Status potrzebny misji autonomicznego zbierania.
        # Nie zmienia ruchow P1/P2/P3/READY.
        self.status_pub = self.create_publisher(
            String,
            STATUS_TOPIC,
            10,
        )

        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            ACTION_NAME,
        )

        self.timer = self.create_timer(
            0.05,
            self.update,
        )

        self.get_logger().info("==========================================")
        self.get_logger().info("CUBE PLACE NODE")
        self.get_logger().info("start = P1 -> P2 -> CHECK -> P3 -> OPEN -> READY")
        self.get_logger().info("reset = przerwanie logiki / powrot do IDLE")
        self.get_logger().info(f"COMMAND: {COMMAND_TOPIC}")
        self.get_logger().info(f"STATUS : {STATUS_TOPIC}")
        self.get_logger().info("==========================================")

    def publish_status(self, text):
        msg = String()
        msg.data = str(text)
        self.status_pub.publish(msg)
        self.get_logger().info(f"PLACE STATUS -> {msg.data}")

    def joint_state_callback(self, msg):
        values = {}

        for name, pos in zip(
            msg.name,
            msg.position,
        ):
            if name in JOINT_NAMES:
                values[name] = float(pos)

        if len(values) == len(JOINT_NAMES):
            self.current_q = np.array(
                [
                    values[name]
                    for name in JOINT_NAMES
                ],
                dtype=float,
            )

    def state_time(self):
        return (
            self.get_clock().now()
            -
            self.state_started
        ).nanoseconds / 1e9

    def set_state(self, new_state):
        self.state = new_state
        self.state_started = self.get_clock().now()

    def fail(self, text):
        self.get_logger().error(text)
        self.publish_status("error:start")

        self.goal_active = False
        self.goal_done = False
        self.goal_ok = False

        self.set_state(ERROR)

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()

        if cmd in ("start", "place"):
            if self.state not in (
                IDLE,
                DONE,
                ERROR,
            ):
                self.get_logger().warn(
                    "Sekwencja PLACE juz trwa."
                )
                return

            if self.current_q is None:
                self.get_logger().error(
                    "Brak /joint_states."
                )
                self.publish_status("error:start")
                return

            if not self.trajectory_client.server_is_ready():
                self.get_logger().error(
                    f"Brak action servera {ACTION_NAME}."
                )
                self.publish_status("error:start")
                return

            self.skip_pick_check = (cmd == "place")

            if self.skip_pick_check:
                self.get_logger().info(
                    "START PLACE-ONLY: P1 -> P2 -> P3 -> OPEN -> READY"
                )
                self.publish_status("running:place")
            else:
                self.get_logger().info(
                    "START PLACE: P1 -> P2 -> CHECK -> P3 -> OPEN -> READY"
                )
                self.publish_status("running:start")

            self.open_command_sent = False
            self.set_state(MOVE_P1)

            self.start_joint_move(
                P1,
                TIME_TO_P1_S,
                "P1",
            )
            return

        if cmd == "continue":
            if self.state != WAIT_PICK_CHECK:
                self.get_logger().warn(
                    "continue ignorowane - nie czekam na PICK CHECK."
                )
                return

            self.get_logger().info(
                "PICK CHECK OK -> kontynuuje P2 -> P3."
            )

            self.publish_status(
                "running:continue"
            )

            self.set_state(
                MOVE_P3
            )

            self.start_joint_move(
                P3,
                TIME_P2_TO_P3_S,
                "P3",
            )
            return

        if cmd == "abort":
            if self.state != WAIT_PICK_CHECK:
                self.get_logger().warn(
                    "abort ignorowane - nie czekam na PICK CHECK."
                )
                return

            self.get_logger().warn(
                "PICK CHECK FAIL -> wracam do READY BEZ otwierania chwytaka."
            )

            self.publish_status(
                "running:abort_to_ready"
            )

            self.set_state(
                ABORT_TO_READY
            )

            self.start_joint_move(
                READY,
                TIME_P3_TO_READY_S,
                "READY_ABORT",
            )
            return

        if cmd == "reset":
            self.goal_active = False
            self.goal_done = False
            self.goal_ok = False
            self.open_command_sent = False
            self.skip_pick_check = False
            self.set_state(IDLE)

            self.get_logger().warn(
                "RESET -> IDLE"
            )
            self.publish_status("idle")
            return

        if cmd == "status":
            self.get_logger().info(
                f"STATE={self.state}"
            )

            if self.current_q is not None:
                self.get_logger().info(
                    "Q DEG = "
                    f"{[round(math.degrees(v), 2) for v in self.current_q]}"
                )
            return

        self.get_logger().warn(
            f"Nieznana komenda: {cmd}"
        )

    def start_joint_move(
        self,
        target_q,
        duration_s,
        label,
    ):
        if self.current_q is None:
            self.fail(
                f"{label}: brak /joint_states."
            )
            return

        if self.goal_active:
            self.fail(
                f"{label}: poprzedni goal nadal aktywny."
            )
            return

        target_q = np.asarray(
            target_q,
            dtype=float,
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = list(
            JOINT_NAMES
        )

        point = JointTrajectoryPoint()
        point.positions = [
            float(v)
            for v in target_q
        ]

        sec = int(duration_s)
        nsec = int(
            (duration_s - sec)
            * 1e9
        )

        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nsec

        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.goal_active = True
        self.goal_done = False
        self.goal_ok = False
        self.goal_label = label
        self.goal_target = target_q.copy()
        self.goal_expected_time = float(duration_s)

        self.get_logger().info(
            f"{label} -> Q deg = "
            f"{[round(math.degrees(v), 2) for v in target_q]} "
            f"| t={duration_s:.1f}s"
        )

        future = (
            self.trajectory_client
            .send_goal_async(goal)
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(
        self,
        future,
    ):
        try:
            handle = future.result()
        except Exception as exc:
            self.fail(
                f"{self.goal_label}: goal exception: {exc}"
            )
            return

        if not handle.accepted:
            self.fail(
                f"{self.goal_label}: trajectory odrzucona."
            )
            return

        result_future = (
            handle.get_result_async()
        )

        result_future.add_done_callback(
            self.goal_result_callback
        )

    def goal_result_callback(
        self,
        future,
    ):
        try:
            wrapped = future.result()

            self.get_logger().info(
                f"{self.goal_label}: trajectory zakonczona, "
                f"status={wrapped.status}"
            )

            self.goal_ok = True
            self.goal_done = True

        except Exception as exc:
            self.fail(
                f"{self.goal_label}: result exception: {exc}"
            )

    def target_reached(self):
        if (
            self.current_q is None
            or
            self.goal_target is None
        ):
            return False

        err = np.abs(
            self.current_q
            -
            self.goal_target
        )

        return bool(
            np.max(err)
            <=
            JOINT_REACHED_TOL_RAD
        )

    def finish_goal_if_ready(self):
        if not self.goal_active:
            return False

        elapsed = (
            self.get_clock().now()
            -
            self.state_started
        ).nanoseconds / 1e9

        if (
            self.goal_done
            and
            self.goal_ok
            and
            self.target_reached()
        ):
            self.goal_active = False
            self.goal_done = False
            self.goal_ok = False
            return True

        if elapsed > (
            self.goal_expected_time
            +
            MOVE_TIMEOUT_MARGIN_S
        ):
            if self.current_q is None:
                self.fail(
                    f"{self.goal_label}: TIMEOUT bez joint_states."
                )
            else:
                err_deg = [
                    round(
                        math.degrees(abs(a - b)),
                        2,
                    )
                    for a, b in zip(
                        self.current_q,
                        self.goal_target,
                    )
                ]

                self.fail(
                    f"{self.goal_label}: TIMEOUT, joint error deg={err_deg}"
                )

        return False

    def open_gripper(self):
        msg = String()
        msg.data = (
            f"@S{GRIPPER_SERVO}:"
            f"{GRIPPER_OPEN_ANGLE_DEG:.0f}"
        )

        self.arduino_pub.publish(msg)

        self.get_logger().info(
            f"ARDUINO -> {msg.data}"
        )

    def update(self):
        if self.state in (
            IDLE,
            DONE,
            ERROR,
        ):
            return

        if self.state == MOVE_P1:
            if self.finish_goal_if_ready():
                self.get_logger().info(
                    "P1 OSIAGNIETE -> P2"
                )

                self.set_state(MOVE_P2)

                self.start_joint_move(
                    P2,
                    TIME_P1_TO_P2_S,
                    "P2",
                )
            return

        if self.state == MOVE_P2:
            if self.finish_goal_if_ready():
                if self.skip_pick_check:
                    self.get_logger().info(
                        "P2 OSIAGNIETE -> PLACE-ONLY, jade bezposrednio do nowego P3."
                    )
                    self.set_state(MOVE_P3)
                    self.start_joint_move(
                        P3,
                        TIME_P2_TO_P3_S,
                        "P3_TRUNK",
                    )
                else:
                    self.get_logger().info(
                        "P2 OSIAGNIETE -> CHECK CZY KOSTKA ZOSTALA PODNIESIONA"
                    )
                    self.set_state(
                        WAIT_PICK_CHECK
                    )
                    self.publish_status(
                        "check:pickup"
                    )

            return

        if self.state == WAIT_PICK_CHECK:
            # Czekamy na:
            #   /cube_place/command = continue
            # albo:
            #   /cube_place/command = abort
            return

        if self.state == MOVE_P3:
            if self.finish_goal_if_ready():
                self.get_logger().info(
                    "P3 OSIAGNIETE -> OTWIERAM CHWYTAK"
                )

                self.open_command_sent = False
                self.set_state(OPEN_GRIPPER)
            return

        if self.state == OPEN_GRIPPER:
            if not self.open_command_sent:
                self.open_gripper()
                self.open_command_sent = True

            if self.state_time() >= OPEN_SETTLE_S:
                self.get_logger().info(
                    "KOSTKA ODPUSZCZONA -> READY"
                )

                self.set_state(MOVE_READY)

                self.start_joint_move(
                    READY,
                    TIME_P3_TO_READY_S,
                    "READY",
                )
            return

        if self.state == ABORT_TO_READY:
            if self.finish_goal_if_ready():
                self.get_logger().warn(
                    "ABORT: ramie w READY -> mozna ponowic cube_pick."
                )

                self.publish_status(
                    "aborted:pickup_check"
                )

                self.set_state(
                    DONE
                )

            return

        if self.state == MOVE_READY:
            if self.finish_goal_if_ready():
                self.get_logger().info(
                    "PLACE GOTOWE. ROBOT W READY."
                )

                if self.skip_pick_check:
                    self.publish_status("done:place")
                else:
                    self.publish_status("done:start")
                self.set_state(DONE)
            return


def main(args=None):
    rclpy.init(args=args)

    node = CubePlaceNode()

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