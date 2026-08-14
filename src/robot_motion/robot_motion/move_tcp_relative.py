import os
import time

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

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

IK_TIMEOUT_NS = 500_000_000


REQUIRED_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
]


class TCPHelper(Node):

    def __init__(self):

        super().__init__(
            "move_tcp_relative_helper"
        )


        # =================================================
        # PARAMETRY RUCHU
        # =================================================

        self.declare_parameter(
            "dx",
            0.0
        )

        self.declare_parameter(
            "dy",
            0.0
        )

        self.declare_parameter(
            "dz",
            0.0
        )


        # =================================================
        # TF
        # =================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )


        # =================================================
        # AKTUALNY JOINT STATE
        #
        # Uzyjemy go jako seed dla IK.
        # =================================================

        self.current_joint_state = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )


        # =================================================
        # IK
        # =================================================

        self.ik_client = self.create_client(
            GetPositionIK,
            "/compute_ik"
        )


    # =====================================================
    # CALLBACK /joint_states
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

        self.get_logger().info(
            "Czekam na TF base_link -> tool0..."
        )

        deadline = (
            time.monotonic() + 5.0
        )

        while (
            rclpy.ok()
            and
            time.monotonic() < deadline
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            try:

                transform = (
                    self.tf_buffer.lookup_transform(
                        BASE_FRAME,
                        TCP_LINK,
                        rclpy.time.Time(),
                    )
                )

                x = (
                    transform
                    .transform
                    .translation
                    .x
                )

                y = (
                    transform
                    .transform
                    .translation
                    .y
                )

                z = (
                    transform
                    .transform
                    .translation
                    .z
                )

                return x, y, z

            except Exception:

                pass

        return None


    # =====================================================
    # CZEKAJ NA AKTUALNY JOINT STATE
    # =====================================================

    def wait_for_joint_state(
        self,
        timeout_sec=2.0,
    ):

        deadline = (
            time.monotonic()
            +
            timeout_sec
        )

        while (
            rclpy.ok()
            and
            self.current_joint_state is None
            and
            time.monotonic() < deadline
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

        return (
            self.current_joint_state
            is not None
        )


    # =====================================================
    # IK
    # =====================================================

    def compute_ik(
        self,
        x,
        y,
        z,
    ):

        self.get_logger().info(
            "Czekam na /compute_ik..."
        )

        while not self.ik_client.wait_for_service(
            timeout_sec=1.0
        ):

            self.get_logger().info(
                "Nadal czekam na /compute_ik..."
            )


        # =================================================
        # MUSIMY MIEC AKTUALNY JOINT STATE
        # =================================================

        if not self.wait_for_joint_state(
            timeout_sec=2.0
        ):

            self.get_logger().error(
                "Brak /joint_states - "
                "nie moge seedowac IK"
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


        # =================================================
        # position_only_ik
        #
        # Orientacja nie jest wymuszana.
        # Quaternion musi jednak byc poprawny.
        # =================================================

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0


        request.ik_request.pose_stamped = pose


        # =================================================
        # SEED IK
        #
        # To jest najwazniejsza zmiana.
        #
        # Zamiast pozwolic KDL zaczynac z losowego/
        # domyslnego stanu, podajemy AKTUALNY fizyczny
        # stan robota.
        # =================================================

        seed_state = RobotStateMsg()

        seed_state.joint_state.name = list(
            self.current_joint_state.name
        )

        seed_state.joint_state.position = list(
            self.current_joint_state.position
        )

        seed_state.joint_state.velocity = []

        seed_state.joint_state.effort = []

        seed_state.joint_state.header = (
            self.current_joint_state.header
        )


        request.ik_request.robot_state = (
            seed_state
        )


        # =================================================
        # DEBUG SEED
        # =================================================

        self.get_logger().info(
            "Seed IK z aktualnego /joint_states:"
        )

        for name, position in zip(
            self.current_joint_state.name,
            self.current_joint_state.position,
        ):

            if name in REQUIRED_JOINTS:

                self.get_logger().info(
                    f"  {name}: "
                    f"{position:.6f}"
                )


        # =================================================
        # WYSLANIE IK
        # =================================================

        future = self.ik_client.call_async(
            request
        )


        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=2.0
        )


        if not future.done():

            self.get_logger().error(
                "Timeout compute_ik"
            )

            return None


        response = future.result()


        if response is None:

            self.get_logger().error(
                "Brak odpowiedzi compute_ik"
            )

            return None


        if response.error_code.val != 1:

            self.get_logger().error(
                f"IK ERROR: "
                f"{response.error_code.val}"
            )

            return None


        return response.solution


def main(args=None):

    rclpy.init(args=args)


    helper = TCPHelper()


    # =====================================================
    # 1. PARAMETRY DX DY DZ
    # =====================================================

    dx = (
        helper.get_parameter("dx")
        .get_parameter_value()
        .double_value
    )

    dy = (
        helper.get_parameter("dy")
        .get_parameter_value()
        .double_value
    )

    dz = (
        helper.get_parameter("dz")
        .get_parameter_value()
        .double_value
    )


    # =====================================================
    # LIMIT BEZPIECZENSTWA OFFSETU
    #
    # Na tym etapie nie pozwalamy przypadkiem wpisac
    # np. dx:=0.5.
    #
    # Maksymalnie 20 mm na jeden ruch.
    # =====================================================

    MAX_OFFSET = 0.020


    if (
        abs(dx) > MAX_OFFSET
        or
        abs(dy) > MAX_OFFSET
        or
        abs(dz) > MAX_OFFSET
    ):

        print(
            "STOP: offset za duzy."
        )

        print(
            "Maksymalnie +/- 0.020 m "
            "na jedna os."
        )

        helper.destroy_node()

        rclpy.shutdown()

        return


    # =====================================================
    # 2. ODCZYT AKTUALNEGO TCP
    # =====================================================

    current = helper.get_current_tcp()


    if current is None:

        print(
            "BLAD: nie moge odczytac "
            "base_link -> tool0"
        )

        helper.destroy_node()

        rclpy.shutdown()

        return


    current_x, current_y, current_z = current


    print()
    print("=" * 60)

    print(
        "AKTUALNY TCP:"
    )

    print(
        f"X = {current_x:.6f}"
    )

    print(
        f"Y = {current_y:.6f}"
    )

    print(
        f"Z = {current_z:.6f}"
    )


    # =====================================================
    # 3. NOWY CEL
    # =====================================================

    target_x = (
        current_x + dx
    )

    target_y = (
        current_y + dy
    )

    target_z = (
        current_z + dz
    )


    print()
    print(
        "OFFSET:"
    )

    print(
        f"DX = {dx:+.6f}"
    )

    print(
        f"DY = {dy:+.6f}"
    )

    print(
        f"DZ = {dz:+.6f}"
    )


    print()
    print(
        "CEL TCP:"
    )

    print(
        f"X = {target_x:.6f}"
    )

    print(
        f"Y = {target_y:.6f}"
    )

    print(
        f"Z = {target_z:.6f}"
    )

    print("=" * 60)


    # =====================================================
    # 4. IK
    # =====================================================

    solution = helper.compute_ik(
        target_x,
        target_y,
        target_z,
    )


    if solution is None:

        print(
            "STOP: brak rozwiazania IK"
        )

        helper.destroy_node()

        rclpy.shutdown()

        return


    solution_map = {}


    for name, position in zip(
        solution.joint_state.name,
        solution.joint_state.position,
    ):

        if name in REQUIRED_JOINTS:

            solution_map[name] = position


    if len(solution_map) != len(
        REQUIRED_JOINTS
    ):

        print(
            "STOP: IK nie zwrocilo "
            "wszystkich J1-J5"
        )

        helper.destroy_node()

        rclpy.shutdown()

        return


    print()
    print(
        "IK OK:"
    )


    for joint in REQUIRED_JOINTS:

        print(
            f"{joint}: "
            f"{solution_map[joint]:.6f}"
        )


    # =====================================================
    # 5. SPRAWDZENIE ROZNICY JOINTOW
    #
    # Dodatkowe zabezpieczenie:
    # dla malego ruchu TCP nie chcemy nagle
    # przestawic ktoregos jointa o ogromny kat.
    # =====================================================

    current_joint_map = {}


    for name, position in zip(
        helper.current_joint_state.name,
        helper.current_joint_state.position,
    ):

        if name in REQUIRED_JOINTS:

            current_joint_map[name] = position


    MAX_JOINT_DELTA = 0.35


    print()
    print(
        "ZMIANA JOINTOW:"
    )


    for joint in REQUIRED_JOINTS:

        delta = (
            solution_map[joint]
            -
            current_joint_map[joint]
        )

        print(
            f"{joint}: "
            f"{delta:+.6f} rad"
        )


        if abs(delta) > MAX_JOINT_DELTA:

            print()
            print(
                "STOP: IK znalazlo zbyt odlegla "
                "konfiguracje."
            )

            print(
                f"{joint} zmienilby sie o "
                f"{delta:+.6f} rad."
            )

            print(
                "Robot NIE zostanie poruszony."
            )

            helper.destroy_node()

            rclpy.shutdown()

            return


    # Helper nie jest juz potrzebny.
    helper.destroy_node()


    # =====================================================
    # 6. MOVEIT
    # =====================================================

    print()
    print(
        "Uruchamiam MoveItPy..."
    )


    moveit = MoveItPy(
        node_name="move_tcp_relative"
    )


    manipulator = (
        moveit.get_planning_component(
            PLANNING_GROUP
        )
    )


    # =====================================================
    # 7. START = AKTUALNY STAN
    # =====================================================

    manipulator.set_start_state_to_current_state()


    # =====================================================
    # 8. GOAL STATE Z IK
    # =====================================================

    robot_model = (
        moveit.get_robot_model()
    )


    goal_state = RobotState(
        robot_model
    )


    goal_state.joint_positions = (
        solution_map
    )


    goal_state.update()


    ok = manipulator.set_goal_state(
        robot_state=goal_state
    )


    if not ok:

        print(
            "STOP: MoveIt odrzucil "
            "goal state"
        )

        os._exit(1)


    # =====================================================
    # 9. PLAN
    # =====================================================

    print()
    print(
        "Planuje..."
    )


    plan_result = (
        manipulator.plan()
    )


    if not plan_result:

        print(
            "PLANOWANIE NIEUDANE"
        )

        os._exit(1)


    print(
        "PLAN OK"
    )


    # =====================================================
    # 10. EXECUTE
    # =====================================================

    print()
    print(
        "Wykonuje..."
    )


    status = moveit.execute(
        plan_result.trajectory,
        controllers=[]
    )


    print()
    print(
        f"Wynik: {status}"
    )


    # =====================================================
    # WORKAROUND MOVEITPY JAZZY
    # =====================================================

    print()
    print(
        "Ruch zakonczony."
    )

    print(
        "Koncze proces bez teardown MoveItPy."
    )


    os._exit(0)


if __name__ == "__main__":

    main()