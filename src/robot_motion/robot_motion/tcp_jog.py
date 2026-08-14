import os
import time
import threading

import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState as RobotStateMsg

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy

from tf2_ros import Buffer, TransformListener


# =====================================================
# KONFIGURACJA
# =====================================================

BASE_FRAME = "base_link"
TCP_LINK = "tool0"

PLANNING_GROUP = "manipulator"


# Jeden klik = 2 mm
JOG_STEP = 0.002


# Maksymalna zmiana pojedynczego jointa
# dla jednej komendy jog.
MAX_JOINT_DELTA = 0.35


IK_TIMEOUT_NS = 500_000_000


REQUIRED_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
]


class TCPJog(Node):

    def __init__(self):

        super().__init__(
            "tcp_jog"
        )


        # =================================================
        # CALLBACK GROUP
        # =================================================

        self.callback_group = (
            ReentrantCallbackGroup()
        )


        # =================================================
        # BLOKADA
        #
        # Nie pozwalamy rozpocząć drugiego ruchu,
        # jeśli pierwszy jeszcze trwa.
        # =================================================

        self.motion_lock = threading.Lock()


        # =================================================
        # JOINT STATE
        # =================================================

        self.current_joint_state = None

        self.joint_state_sub = (
            self.create_subscription(
                JointState,
                "/joint_states",
                self.joint_state_callback,
                10,
                callback_group=self.callback_group,
            )
        )


        # =================================================
        # TF
        # =================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )


        # =================================================
        # IK
        # =================================================

        self.ik_client = self.create_client(
            GetPositionIK,
            "/compute_ik",
            callback_group=self.callback_group,
        )


        # =================================================
        # KOMENDY JOG
        #
        # Topic:
        #
        # /tcp_jog
        #
        # Komendy:
        #
        # x+
        # x-
        # y+
        # y-
        # z+
        # z-
        # =================================================

        self.command_sub = (
            self.create_subscription(
                String,
                "/tcp_jog",
                self.command_callback,
                10,
                callback_group=self.callback_group,
            )
        )


        # =================================================
        # MOVEITPY
        #
        # Uruchamiamy TYLKO RAZ.
        # =================================================

        self.get_logger().info(
            "Uruchamiam MoveItPy..."
        )

        self.moveit = MoveItPy(
            node_name="tcp_jog_moveit"
        )

        self.manipulator = (
            self.moveit.get_planning_component(
                PLANNING_GROUP
            )
        )

        self.robot_model = (
            self.moveit.get_robot_model()
        )


        # =================================================
        # READY
        # =================================================

        self.get_logger().info(
            "================================="
        )

        self.get_logger().info(
            "TCP JOG GOTOWY"
        )

        self.get_logger().info(
            f"Krok: {JOG_STEP * 1000:.1f} mm"
        )

        self.get_logger().info(
            "Komendy: x+ x- y+ y- z+ z-"
        )

        self.get_logger().info(
            "Topic: /tcp_jog"
        )

        self.get_logger().info(
            "================================="
        )


    # =====================================================
    # JOINT STATE CALLBACK
    # =====================================================

    def joint_state_callback(
        self,
        msg,
    ):

        self.current_joint_state = msg


    # =====================================================
    # AKTUALNY TCP
    # =====================================================

    def get_current_tcp(self):

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    BASE_FRAME,
                    TCP_LINK,
                    rclpy.time.Time(),
                )
            )

        except Exception as exc:

            self.get_logger().error(
                f"Brak TF "
                f"{BASE_FRAME} -> {TCP_LINK}: "
                f"{exc}"
            )

            return None


        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )


    # =====================================================
    # JOINT MAP
    # =====================================================

    def get_current_joint_map(self):

        if self.current_joint_state is None:

            return None


        result = {}


        for name, position in zip(
            self.current_joint_state.name,
            self.current_joint_state.position,
        ):

            if name in REQUIRED_JOINTS:

                result[name] = position


        if len(result) != len(
            REQUIRED_JOINTS
        ):

            return None


        return result


    # =====================================================
    # COMPUTE IK
    # =====================================================

    def compute_ik(
        self,
        x,
        y,
        z,
    ):

        if self.current_joint_state is None:

            self.get_logger().error(
                "Brak /joint_states"
            )

            return None


        if not self.ik_client.service_is_ready():

            self.get_logger().error(
                "/compute_ik nie jest gotowe"
            )

            return None


        request = GetPositionIK.Request()


        request.ik_request.group_name = (
            PLANNING_GROUP
        )

        request.ik_request.ik_link_name = (
            TCP_LINK
        )

        request.ik_request.avoid_collisions = True


        request.ik_request.timeout.sec = 0

        request.ik_request.timeout.nanosec = (
            IK_TIMEOUT_NS
        )


        # =================================================
        # CEL TCP
        # =================================================

        pose = PoseStamped()

        pose.header.frame_id = BASE_FRAME

        pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z


        # position_only_ik
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0


        request.ik_request.pose_stamped = pose


        # =================================================
        # SEED = AKTUALNY FIZYCZNY STAN
        # =================================================

        seed_state = RobotStateMsg()

        seed_state.joint_state.name = list(
            self.current_joint_state.name
        )

        seed_state.joint_state.position = list(
            self.current_joint_state.position
        )

        seed_state.joint_state.header = (
            self.current_joint_state.header
        )

        request.ik_request.robot_state = (
            seed_state
        )


        # =================================================
        # CALL
        # =================================================

        future = self.ik_client.call_async(
            request
        )


        deadline = (
            time.monotonic() + 2.0
        )


        # MultiThreadedExecutor w drugim watku
        # obsłuży odpowiedź service.
        while (
            rclpy.ok()
            and
            not future.done()
            and
            time.monotonic() < deadline
        ):

            time.sleep(0.01)


        if not future.done():

            self.get_logger().error(
                "Timeout /compute_ik"
            )

            return None


        response = future.result()


        if response is None:

            self.get_logger().error(
                "Brak odpowiedzi /compute_ik"
            )

            return None


        if response.error_code.val != 1:

            self.get_logger().error(
                f"IK ERROR: "
                f"{response.error_code.val}"
            )

            return None


        solution = {}


        for name, position in zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ):

            if name in REQUIRED_JOINTS:

                solution[name] = position


        if len(solution) != len(
            REQUIRED_JOINTS
        ):

            self.get_logger().error(
                "IK nie zwrocilo J1-J5"
            )

            return None


        return solution


    # =====================================================
    # WALIDACJA IK
    # =====================================================

    def validate_solution(
        self,
        current,
        target,
    ):

        self.get_logger().info(
            "Zmiany jointow:"
        )


        for joint in REQUIRED_JOINTS:

            delta = (
                target[joint]
                -
                current[joint]
            )


            self.get_logger().info(
                f"  {joint}: "
                f"{delta:+.6f} rad"
            )


            if abs(delta) > MAX_JOINT_DELTA:

                self.get_logger().error(
                    "STOP: zbyt duza zmiana "
                    f"{joint}: "
                    f"{delta:+.6f} rad"
                )

                return False


        return True


    # =====================================================
    # WYKONANIE RUCHU
    # =====================================================

    def execute_move(
        self,
        dx,
        dy,
        dz,
    ):

        current_tcp = (
            self.get_current_tcp()
        )


        if current_tcp is None:

            return


        current_x = current_tcp[0]
        current_y = current_tcp[1]
        current_z = current_tcp[2]


        target_x = current_x + dx
        target_y = current_y + dy
        target_z = current_z + dz


        self.get_logger().info(
            "---------------------------------"
        )

        self.get_logger().info(
            "JOG TCP"
        )

        self.get_logger().info(
            f"START: "
            f"X={current_x:.4f} "
            f"Y={current_y:.4f} "
            f"Z={current_z:.4f}"
        )

        self.get_logger().info(
            f"OFFSET: "
            f"DX={dx:+.4f} "
            f"DY={dy:+.4f} "
            f"DZ={dz:+.4f}"
        )

        self.get_logger().info(
            f"CEL: "
            f"X={target_x:.4f} "
            f"Y={target_y:.4f} "
            f"Z={target_z:.4f}"
        )


        # =================================================
        # AKTUALNE JOINTY
        # =================================================

        current_joint_map = (
            self.get_current_joint_map()
        )


        if current_joint_map is None:

            self.get_logger().error(
                "Brak pelnego /joint_states"
            )

            return


        # =================================================
        # IK
        # =================================================

        solution = self.compute_ik(
            target_x,
            target_y,
            target_z,
        )


        if solution is None:

            self.get_logger().error(
                "STOP: IK nieudane"
            )

            return


        # =================================================
        # WALIDACJA
        # =================================================

        if not self.validate_solution(
            current_joint_map,
            solution,
        ):

            self.get_logger().error(
                "Robot NIE zostal poruszony."
            )

            return


        # =================================================
        # MOVEIT GOAL
        # =================================================

        self.manipulator.set_start_state_to_current_state()


        goal_state = RobotState(
            self.robot_model
        )

        goal_state.joint_positions = (
            solution
        )

        goal_state.update()


        ok = self.manipulator.set_goal_state(
            robot_state=goal_state
        )


        if not ok:

            self.get_logger().error(
                "MoveIt odrzucil goal state"
            )

            return


        # =================================================
        # PLAN
        # =================================================

        self.get_logger().info(
            "Planuje..."
        )


        plan_result = (
            self.manipulator.plan()
        )


        if not plan_result:

            self.get_logger().error(
                "PLANOWANIE NIEUDANE"
            )

            return


        self.get_logger().info(
            "PLAN OK"
        )


        # =================================================
        # EXECUTE
        # =================================================

        self.get_logger().info(
            "Wykonuje..."
        )


        status = self.moveit.execute(
            plan_result.trajectory,
            controllers=[]
        )


        self.get_logger().info(
            f"Wynik: {status}"
        )

        self.get_logger().info(
            "---------------------------------"
        )


    # =====================================================
    # KOMENDA
    # =====================================================

    def command_callback(
        self,
        msg,
    ):

        command = (
            msg.data
            .strip()
            .lower()
        )


        commands = {

            "x+": (
                JOG_STEP,
                0.0,
                0.0,
            ),

            "x-": (
                -JOG_STEP,
                0.0,
                0.0,
            ),

            "y+": (
                0.0,
                JOG_STEP,
                0.0,
            ),

            "y-": (
                0.0,
                -JOG_STEP,
                0.0,
            ),

            "z+": (
                0.0,
                0.0,
                JOG_STEP,
            ),

            "z-": (
                0.0,
                0.0,
                -JOG_STEP,
            ),
        }


        if command not in commands:

            self.get_logger().warn(
                f"Nieznana komenda: "
                f"'{command}'"
            )

            self.get_logger().info(
                "Dozwolone: "
                "x+ x- y+ y- z+ z-"
            )

            return


        # =================================================
        # TYLKO JEDEN RUCH NARAZ
        # =================================================

        if not self.motion_lock.acquire(
            blocking=False
        ):

            self.get_logger().warn(
                "Robot jest w ruchu. "
                "Komenda pominieta."
            )

            return


        try:

            dx, dy, dz = (
                commands[command]
            )


            self.execute_move(
                dx,
                dy,
                dz,
            )

        finally:

            self.motion_lock.release()


def main(args=None):

    rclpy.init(args=args)


    node = TCPJog()


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

        # MoveItPy na Twoim Jazzy ma problem
        # z destruktorem przy shutdown.
        #
        # Nie dopuszczamy do teardown MoveItCpp.
        os._exit(0)


if __name__ == "__main__":

    main()
