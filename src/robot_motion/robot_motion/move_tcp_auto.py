import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetPositionIK

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy


# =====================================================
# CEL TCP
#
# To jest ABSOLUTNA pozycja w base_link.
#
# Aktualnie sprawdzony punkt:
# X = 0.017
# Y = -0.024
# Z = 0.331
#
# Na pierwszy test robimy +5 mm w Z:
# Z = 0.336
# =====================================================

TARGET_X = 0.017
TARGET_Y = -0.024
TARGET_Z = 0.336


# =====================================================
# PARAMETRY
# =====================================================

PLANNING_GROUP = "manipulator"
TCP_LINK = "tool0"
BASE_FRAME = "base_link"

IK_TIMEOUT_NS = 500_000_000


REQUIRED_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
]


class IKClient(Node):

    def __init__(self):

        super().__init__(
            "move_tcp_auto_ik"
        )

        self.client = self.create_client(
            GetPositionIK,
            "/compute_ik"
        )


    def wait_for_service(self):

        self.get_logger().info(
            "Czekam na /compute_ik..."
        )

        while not self.client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                "Nadal czekam na /compute_ik..."
            )


    def compute_ik(
        self,
        x,
        y,
        z,
    ):

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


        pose = PoseStamped()

        pose.header.frame_id = BASE_FRAME

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z


        # position_only_ik:
        # orientacja jest ignorowana przez KDL,
        # ale quaternion musi byc poprawny.
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0


        request.ik_request.pose_stamped = pose


        future = self.client.call_async(
            request
        )


        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=2.0
        )


        if not future.done():

            self.get_logger().error(
                "Timeout /compute_ik"
            )

            return None


        response = future.result()


        if response is None:

            self.get_logger().error(
                "Brak odpowiedzi z /compute_ik"
            )

            return None


        # MoveItErrorCodes.SUCCESS == 1
        if response.error_code.val != 1:

            self.get_logger().error(
                f"IK nieudane. "
                f"error_code="
                f"{response.error_code.val}"
            )

            return None


        return response.solution


def main(args=None):

    rclpy.init(args=args)


    # =================================================
    # 1. IK
    # =================================================

    ik_node = IKClient()

    ik_node.wait_for_service()


    print()
    print("=" * 60)

    print(
        "CEL TCP:"
    )

    print(
        f"X = {TARGET_X:.4f} m"
    )

    print(
        f"Y = {TARGET_Y:.4f} m"
    )

    print(
        f"Z = {TARGET_Z:.4f} m"
    )

    print("=" * 60)


    ik_solution = ik_node.compute_ik(
        TARGET_X,
        TARGET_Y,
        TARGET_Z,
    )


    if ik_solution is None:

        print(
            "STOP: brak rozwiazania IK"
        )

        ik_node.destroy_node()

        rclpy.shutdown()

        return


    print()
    print("IK OK")


    solution_map = {}


    for name, position in zip(
        ik_solution.joint_state.name,
        ik_solution.joint_state.position,
    ):

        if name in REQUIRED_JOINTS:

            solution_map[name] = position


    if len(solution_map) != len(
        REQUIRED_JOINTS
    ):

        print(
            "BLAD: IK nie zwrocilo wszystkich "
            "J1-J5"
        )

        print(
            solution_map
        )

        ik_node.destroy_node()

        rclpy.shutdown()

        return


    print()
    print("Rozwiazanie IK:")


    for joint in REQUIRED_JOINTS:

        print(
            f"{joint}: "
            f"{solution_map[joint]:.6f}"
        )


    # IK node nie jest juz potrzebny.
    ik_node.destroy_node()


    # =================================================
    # 2. MOVEITPY
    # =================================================

    print()
    print(
        "Uruchamiam MoveItPy..."
    )


    moveit = MoveItPy(
        node_name="move_tcp_auto"
    )


    manipulator = (
        moveit.get_planning_component(
            PLANNING_GROUP
        )
    )


    # =================================================
    # 3. START = PRAWDZIWY AKTUALNY STAN
    # =================================================

    manipulator.set_start_state_to_current_state()


    # =================================================
    # 4. ROBOT STATE Z ROZWIAZANIA IK
    # =================================================

    robot_model = moveit.get_robot_model()

    goal_state = RobotState(
        robot_model
    )


    # Ustawiamy tylko J1-J5.
    goal_state.joint_positions = (
        solution_map
    )


    # Wazne po recznej zmianie jointow.
    goal_state.update()


    # =================================================
    # 5. CEL JOINT-SPACE
    # =================================================

    goal_ok = manipulator.set_goal_state(
        robot_state=goal_state
    )


    if not goal_ok:

        print(
            "STOP: MoveIt odrzucil "
            "RobotState z IK"
        )

        return


    # =================================================
    # 6. PLANOWANIE
    # =================================================

    print()
    print(
        "Planuje joint-space..."
    )


    plan_result = (
        manipulator.plan()
    )


    if not plan_result:

        print(
            "PLANOWANIE NIEUDANE"
        )

        return


    print(
        "PLAN OK"
    )


    # =================================================
    # 7. EXECUTE
    #
    # Trajektoria pojdzie do:
    #
    # /arm_controller/follow_joint_trajectory
    #
    # czyli do serial_trajectory_bridge.
    # =================================================

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
        f"Wynik wykonania: {status}"
    )


if __name__ == "__main__":

    main()
