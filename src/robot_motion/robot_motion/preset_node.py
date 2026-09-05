import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from sensor_msgs.msg import JointState

from control_msgs.action import FollowJointTrajectory

from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)


# ============================================================
# JOINTY
# ============================================================

JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
]


# ============================================================
# PRESETY
# ============================================================

PRESETS = {

    "pick": {
        "joint_1": -3.90357991,
        "joint_2":  3.35946512,
        "joint_3":  2.64845323,
        "joint_4":  2.26892803,
        "joint_5":  1.43116999,
    },

    "ready": {
        "joint_1": -3.90357991,
        "joint_2":  1.97557846,
        "joint_3":  3.62453701,
        "joint_4":  2.53072742,
        "joint_5":  1.43116999,
    },

    "trunk": {
        "joint_1": -0.7835101870431275,
        "joint_2":  1.947258811846462,
        "joint_3":  3.6570101983193686,
        "joint_4":  2.426007660272118,
        "joint_5":  1.4311699866353502,
    },

    "pion": {
        "joint_1": -3.9035799101936637,
        "joint_2":  1.5285056564581108,
        "joint_3":  2.1681520453019942,
        "joint_4":  1.6580627893946132,
        "joint_5":  1.4311699866353502,
    },
}


# ============================================================
# TRAJEKTORIA
# ============================================================

TRAJECTORY_POINTS = 50

MIN_DURATION = 4.0

MAX_DURATION = 8.0


class PresetNode(Node):

    def __init__(self):

        super().__init__(
            "manipulator_preset_node"
        )

        self.current_positions = {}

        self.busy = False


        # ====================================================
        # JOINT STATES
        # ====================================================

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )


        # ====================================================
        # PRESET COMMAND
        # ====================================================

        self.create_subscription(
            String,
            "/manipulator/preset",
            self.preset_callback,
            10,
        )


        # ====================================================
        # TRAJECTORY CLIENT
        # ====================================================

        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )


        self.get_logger().info(
            "======================================"
        )

        self.get_logger().info(
            "MANIPULATOR PRESET NODE - SMOOTH"
        )

        self.get_logger().info(
            "Nasluchuje: /manipulator/preset"
        )

        self.get_logger().info(
            "Presety: ready, pick, trunk, pion"
        )

        self.get_logger().info(
            f"Punkty trajektorii: {TRAJECTORY_POINTS}"
        )

        self.get_logger().info(
            "Profil: quintic smoothstep"
        )

        self.get_logger().info(
            "======================================"
        )


    # ========================================================
    # JOINT STATES
    # ========================================================

    def joint_state_callback(
        self,
        msg,
    ):

        for name, position in zip(
            msg.name,
            msg.position,
        ):

            if name in JOINTS:

                self.current_positions[name] = (
                    float(position)
                )


    # ========================================================
    # QUINTIC SMOOTHSTEP
    #
    # s(t) = 10t^3 - 15t^4 + 6t^5
    #
    # daje:
    # - zerowa predkosc na poczatku
    # - zerowa predkosc na koncu
    # - zerowe przyspieszenie na poczatku
    # - zerowe przyspieszenie na koncu
    # ========================================================

    @staticmethod
    def smoothstep_quintic(
        t,
    ):

        return (
            10.0 * t**3
            -
            15.0 * t**4
            +
            6.0 * t**5
        )


    # ========================================================
    # PRESET CALLBACK
    # ========================================================

    def preset_callback(
        self,
        msg,
    ):

        preset_name = (
            msg.data
            .strip()
            .lower()
        )


        # ====================================================
        # BUSY
        # ====================================================

        if self.busy:

            self.get_logger().warn(
                "Manipulator wykonuje juz ruch."
            )

            return


        # ====================================================
        # PRESET ISTNIEJE?
        # ====================================================

        if preset_name not in PRESETS:

            self.get_logger().warn(
                f"Nieznany preset: {preset_name}"
            )

            return


        # ====================================================
        # MAMY JOINT STATES?
        # ====================================================

        if len(
            self.current_positions
        ) != len(
            JOINTS
        ):

            self.get_logger().error(
                "Brak aktualnych /joint_states."
            )

            return


        # ====================================================
        # CONTROLLER GOTOWY?
        # ====================================================

        if not (
            self.trajectory_client
            .server_is_ready()
        ):

            self.get_logger().error(
                "Brak kontrolera trajectory."
            )

            return


        target = (
            PRESETS[
                preset_name
            ]
        )


        # ====================================================
        # START
        # ====================================================

        start = {

            joint:
                self.current_positions[
                    joint
                ]

            for joint
            in JOINTS
        }


        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "======================================"
        )

        self.get_logger().info(
            f"PRESET: {preset_name.upper()}"
        )


        # ====================================================
        # NAJWIEKSZA ZMIANA JOINTA
        # ====================================================

        max_delta = 0.0


        for joint in JOINTS:

            current = (
                start[
                    joint
                ]
            )

            goal = (
                target[
                    joint
                ]
            )

            delta = abs(
                goal
                -
                current
            )

            max_delta = max(
                max_delta,
                delta,
            )


            self.get_logger().info(
                f"{joint}: "
                f"{current:.4f} -> "
                f"{goal:.4f} "
                f"delta={goal-current:+.4f}"
            )


        # ====================================================
        # CZAS TRAJEKTORII
        #
        # Wiekszy ruch = wolniejszy przejazd.
        # ====================================================

        duration = (
            3.0
            +
            max_delta
            *
            2.0
        )


        duration = max(
            MIN_DURATION,
            min(
                MAX_DURATION,
                duration,
            ),
        )


        self.get_logger().info(
            f"Czas ruchu: {duration:.2f} s"
        )


        # ====================================================
        # TRAJEKTORIA
        # ====================================================

        trajectory = (
            JointTrajectory()
        )

        trajectory.joint_names = list(
            JOINTS
        )


        # ====================================================
        # PUNKTY POSREDNIE
        # ====================================================

        for i in range(
            1,
            TRAJECTORY_POINTS + 1,
        ):

            t = (
                i
                /
                TRAJECTORY_POINTS
            )


            s = (
                self.smoothstep_quintic(
                    t
                )
            )


            point = (
                JointTrajectoryPoint()
            )


            # ================================================
            # POZYCJE
            # ================================================

            point.positions = [

                start[joint]
                +
                s
                *
                (
                    target[joint]
                    -
                    start[joint]
                )

                for joint
                in JOINTS
            ]


            # ================================================
            # CZAS
            # ================================================

            point_time = (
                duration
                *
                t
            )


            seconds = int(
                point_time
            )


            nanoseconds = int(
                (
                    point_time
                    -
                    seconds
                )
                *
                1e9
            )


            point.time_from_start.sec = (
                seconds
            )

            point.time_from_start.nanosec = (
                nanoseconds
            )


            trajectory.points.append(
                point
            )


        # ====================================================
        # GOAL
        # ====================================================

        goal = (
            FollowJointTrajectory.Goal()
        )

        goal.trajectory = (
            trajectory
        )


        self.busy = True


        self.get_logger().info(
            f"Wysylam "
            f"{len(trajectory.points)} "
            f"punktow trajektorii..."
        )


        future = (
            self.trajectory_client
            .send_goal_async(
                goal
            )
        )


        future.add_done_callback(
            self.goal_response_callback
        )


    # ========================================================
    # GOAL RESPONSE
    # ========================================================

    def goal_response_callback(
        self,
        future,
    ):

        try:

            goal_handle = (
                future.result()
            )

        except Exception as exc:

            self.get_logger().error(
                f"Blad trajectory: {exc}"
            )

            self.busy = False

            return


        if not goal_handle.accepted:

            self.get_logger().error(
                "Trajectory odrzucona."
            )

            self.busy = False

            return


        self.get_logger().info(
            "Trajectory zaakceptowana."
        )


        result_future = (
            goal_handle
            .get_result_async()
        )


        result_future.add_done_callback(
            self.result_callback
        )


    # ========================================================
    # RESULT
    # ========================================================

    def result_callback(
        self,
        future,
    ):

        try:

            result = (
                future.result()
            )


            self.get_logger().info(
                f"Preset zakonczony. "
                f"status={result.status}"
            )


        except Exception as exc:

            self.get_logger().error(
                f"Blad wyniku: {exc}"
            )


        self.busy = False


def main(
    args=None,
):

    rclpy.init(
        args=args
    )


    node = (
        PresetNode()
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