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
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import serial


# =====================================================
# SERIAL
# =====================================================

SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200


# =====================================================
# MECHANIKA
# =====================================================

STEPS_PER_REV = 200.0 * 8.0 * 10.4

STEPS_PER_RAD = (
    STEPS_PER_REV /
    (2.0 * math.pi)
)


# =====================================================
# WYSYLANIE TRAJEKTORII
#
# 20 Hz sprawdzilismy juz fizycznie.
# =====================================================

COMMAND_PERIOD = 0.05


REQUIRED_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
]


class SerialTrajectoryBridge(Node):

    def __init__(self):

        super().__init__(
            "serial_trajectory_bridge"
        )

        # =============================================
        # STAN
        # =============================================

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


        self.serial_lock = threading.Lock()


        # =============================================
        # SERIAL
        # =============================================

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


        self.get_logger().info(
            f"STEPS_PER_RAD = "
            f"{STEPS_PER_RAD:.3f}"
        )


        # Arduino Mega resetuje sie po USB.
        time.sleep(2.0)


        # =============================================
        # /joint_states
        # =============================================

        self.joint_state_pub = (
            self.create_publisher(
                JointState,
                "/joint_states",
                10,
            )
        )


        # =============================================
        # SERIAL RX
        # =============================================

        self.serial_timer = (
            self.create_timer(
                0.02,
                self.read_serial,
            )
        )


        # =============================================
        # HOMING
        # =============================================

        self.homing_timer = (
            self.create_timer(
                0.2,
                self.update_homing,
            )
        )


        # =============================================
        # FOLLOW JOINT TRAJECTORY
        #
        # Ta nazwa jest celowa.
        #
        # MoveIt juz oczekuje:
        #
        # /arm_controller/follow_joint_trajectory
        # =============================================

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )


        self.get_logger().info(
            "Action server:"
        )

        self.get_logger().info(
            "/arm_controller/follow_joint_trajectory"
        )

        self.get_logger().info(
            "Czekam na Arduino i homing..."
        )


    # =================================================
    # SERIAL TX
    # =================================================

    def send_serial(self, text):

        try:

            with self.serial_lock:

                self.serial.write(
                    text.encode("ascii")
                )

        except serial.SerialException as exc:

            self.get_logger().error(
                f"Serial TX error: {exc}"
            )


    # =================================================
    # HOMING
    # =================================================

    def update_homing(self):

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


    # =================================================
    # @P Z ARDUINO
    # =================================================

    def handle_position_report(self, text):

        if not text.startswith("@P:"):
            return


        parts = (
            text[3:]
            .split(",")
        )


        if len(parts) != 5:
            return


        try:

            j1_steps = int(parts[0])
            j2_steps = int(parts[1])
            j3_steps = int(parts[2])

            j4_deg = float(parts[3])
            j5_deg = float(parts[4])

        except ValueError:
            return


        self.positions["joint_1"] = (
            j1_steps /
            STEPS_PER_RAD
        )

        self.positions["joint_2"] = (
            j2_steps /
            STEPS_PER_RAD
        )

        self.positions["joint_3"] = (
            j3_steps /
            STEPS_PER_RAD
        )

        self.positions["joint_4"] = (
            math.radians(j4_deg)
        )

        self.positions["joint_5"] = (
            math.radians(j5_deg)
        )


        self.have_position = True


        # =============================================
        # PUBLIKACJA /joint_states
        # =============================================

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
            self.positions["joint_1"],
            self.positions["joint_2"],
            self.positions["joint_3"],
            self.positions["joint_4"],
            self.positions["joint_5"],
            0.4,
        ]


        self.joint_state_pub.publish(
            msg
        )


    # =================================================
    # SERIAL RX
    # =================================================

    def read_serial(self):

        try:

            while True:

                with self.serial_lock:

                    if (
                        self.serial.in_waiting <= 0
                    ):
                        break

                    raw = (
                        self.serial.readline()
                    )


                if not raw:
                    break


                text = raw.decode(
                    "utf-8",
                    errors="replace",
                ).strip()


                if not text:
                    continue


                if text.startswith("@P:"):

                    self.handle_position_report(
                        text
                    )

                    continue


                self.get_logger().info(
                    f"ARDUINO: {text}"
                )


                if (
                    "Manipulator gotowy"
                    in text
                ):

                    self.arduino_ready = True


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


                if (
                    "HOMING ERROR" in text or
                    "HOMING PRZERWANY" in text or
                    "\"error\":\"not_homed\"" in text
                ):

                    self.homed = False

                    self.have_position = False

                    self.homing_started = False


                    self.get_logger().error(
                        "Robot utracil homing"
                    )


        except serial.SerialException as exc:

            self.get_logger().error(
                f"Serial RX error: {exc}"
            )


    # =================================================
    # RAD -> @J
    # =================================================

    def make_command(self, positions):

        j1 = round(
            positions["joint_1"] *
            STEPS_PER_RAD
        )

        j2 = round(
            positions["joint_2"] *
            STEPS_PER_RAD
        )

        j3 = round(
            positions["joint_3"] *
            STEPS_PER_RAD
        )


        j4 = math.degrees(
            positions["joint_4"]
        )

        j5 = math.degrees(
            positions["joint_5"]
        )


        # =============================================
        # TWARDE LIMITY
        # =============================================

        j1 = max(
            -15557,
            min(0, j1)
        )

        j2 = max(
            0,
            min(10100, j2)
        )

        j3 = max(
            0,
            min(11835, j3)
        )

        j4 = max(
            5.0,
            min(180.0, j4)
        )

        j5 = max(
            0.0,
            min(180.0, j5)
        )


        return (
            f"@J:"
            f"{j1},"
            f"{j2},"
            f"{j3},"
            f"{j4:.1f},"
            f"{j5:.1f}\n"
        )


    # =================================================
    # ACTION - GOAL
    # =================================================

    def goal_callback(self, goal_request):

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
                f"Niepoprawne joint_names: {names}"
            )

            return GoalResponse.REJECT


        if not (
            goal_request
            .trajectory
            .points
        ):

            return GoalResponse.REJECT


        self.get_logger().info(
            "Trajectory zaakceptowana"
        )


        return GoalResponse.ACCEPT


    # =================================================
    # ACTION - CANCEL
    # =================================================

    def cancel_callback(
        self,
        goal_handle
    ):

        self.get_logger().warn(
            "Zadano anulowanie trajectory"
        )


        return CancelResponse.ACCEPT


    # =================================================
    # CZAS TRAJECTORY POINT
    # =================================================

    @staticmethod
    def point_time(point):

        return (
            float(
                point.time_from_start.sec
            )
            +
            float(
                point.time_from_start.nanosec
            ) / 1e9
        )


    # =================================================
    # INTERPOLACJA
    # =================================================

    def interpolate(
        self,
        names,
        start_positions,
        points,
        t,
    ):

        # =============================================
        # pierwszy punkt
        # =============================================

        first_t = self.point_time(
            points[0]
        )


        if t <= first_t:

            if first_t <= 0.0:

                return dict(
                    zip(
                        names,
                        points[0].positions
                    )
                )


            alpha = max(
                0.0,
                min(
                    1.0,
                    t / first_t
                )
            )


            result = {}


            for i, name in enumerate(names):

                start = (
                    start_positions[name]
                )

                target = (
                    points[0]
                    .positions[i]
                )


                result[name] = (
                    start +
                    alpha *
                    (target - start)
                )


            return result


        # =============================================
        # kolejne segmenty
        # =============================================

        for index in range(
            1,
            len(points)
        ):

            previous = (
                points[index - 1]
            )

            current = (
                points[index]
            )


            t0 = self.point_time(
                previous
            )

            t1 = self.point_time(
                current
            )


            if t <= t1:

                if t1 <= t0:

                    alpha = 1.0

                else:

                    alpha = (
                        (t - t0) /
                        (t1 - t0)
                    )


                alpha = max(
                    0.0,
                    min(1.0, alpha)
                )


                result = {}


                for i, name in enumerate(names):

                    p0 = (
                        previous
                        .positions[i]
                    )

                    p1 = (
                        current
                        .positions[i]
                    )


                    result[name] = (
                        p0 +
                        alpha *
                        (p1 - p0)
                    )


                return result


        # =============================================
        # koniec
        # =============================================

        return dict(
            zip(
                names,
                points[-1].positions
            )
        )


    # =================================================
    # ACTION EXECUTE
    # =================================================

    def execute_callback(
        self,
        goal_handle
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
            name: self.positions[name]
            for name in REQUIRED_JOINTS
        }


        duration = self.point_time(
            points[-1]
        )


        self.get_logger().info(
            f"Wykonuje trajectory: "
            f"{duration:.3f} s"
        )


        start_time = time.monotonic()


        # =============================================
        # REALIZACJA W CZASIE
        # =============================================

        while True:

            if goal_handle.is_cancel_requested:

                self.send_serial(
                    "x\n"
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


            elapsed = (
                time.monotonic() -
                start_time
            )


            if elapsed >= duration:
                break


            target = self.interpolate(
                names,
                start_positions,
                points,
                elapsed,
            )


            self.send_serial(
                self.make_command(
                    target
                )
            )


            # =========================================
            # FEEDBACK
            # =========================================

            feedback = (
                FollowJointTrajectory
                .Feedback()
            )


            feedback.joint_names = (
                list(REQUIRED_JOINTS)
            )


            actual = JointTrajectoryPoint()

            actual.positions = [
                self.positions[joint]
                for joint
                in REQUIRED_JOINTS
            ]


            feedback.actual = actual


            goal_handle.publish_feedback(
                feedback
            )


            time.sleep(
                COMMAND_PERIOD
            )


        # =============================================
        # OSTATNI PUNKT
        # =============================================

        final_target = dict(
            zip(
                names,
                points[-1].positions
            )
        )


        self.send_serial(
            self.make_command(
                final_target
            )
        )


        # =============================================
        # CZEKAMY CHWILE NA FIZYCZNE DOJSCIE
        # =============================================

        deadline = (
            time.monotonic() +
            5.0
        )


        while (
            time.monotonic() <
            deadline
        ):

            all_ok = True


            for joint in REQUIRED_JOINTS:

                error = abs(
                    self.positions[joint] -
                    final_target[joint]
                )


                if joint in (
                    "joint_1",
                    "joint_2",
                    "joint_3",
                ):

                    tolerance = (
                        15.0 /
                        STEPS_PER_RAD
                    )

                else:

                    tolerance = (
                        math.radians(1.0)
                    )


                if error > tolerance:

                    all_ok = False

                    break


            if all_ok:
                break


            time.sleep(0.05)


        # =============================================
        # SUCCEEDED
        # =============================================

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
            "Trajectory zakonczona"
        )


        return result


    # =================================================
    # SHUTDOWN
    # =================================================

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


    node = SerialTrajectoryBridge()


    executor = MultiThreadedExecutor(
        num_threads=4
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

        rclpy.shutdown()


if __name__ == "__main__":

    main()