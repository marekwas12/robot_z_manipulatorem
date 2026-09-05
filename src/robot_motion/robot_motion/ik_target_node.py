import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


BASE_FRAME = "base_link"
TCP_LINK = "tool0"
PLANNING_GROUP = "manipulator"
IK_TOPIC = "/manipulator/ik_target"
IK_PRECISION_TOPIC = "/manipulator/ik_target_precision"

REQUIRED_JOINTS = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
]

JOINT_LIMITS = {
    "joint_1": (-5.87425, 0.0),
    "joint_2": (0.0, 3.81371),
    "joint_3": (0.0, 4.46884),
    "joint_4": (0.08727, math.pi),
    "joint_5": (0.0, 3.14159),
}

# Maksymalna zmiana jednego jointa dla pojedynczego punktu TCP.
# Rozwiazania przekraczajace te limity sa wyrzucane PRZED scoringiem.
MAX_JOINT_DELTA = {
    "joint_1": math.radians(30.0),
    "joint_2": math.radians(70.0),
    "joint_3": math.radians(70.0),
    "joint_4": math.radians(55.0),
    "joint_5": math.radians(25.0),
}

PRECISION_MAX_JOINT_DELTA = {
    "joint_1": math.radians(8.0),
    "joint_2": math.radians(15.0),
    "joint_3": math.radians(15.0),
    "joint_4": math.radians(0.75),
    "joint_5": math.radians(0.25),
}

# Mocno trzymamy nadgarstek, ale NIE wymuszamy pelnego quaternionu.
# Dla 5-DOF jest to wazne: pozycja XYZ ma byc osiagalna, a sposrod
# rozwiazan wybieramy takie, ktore najmniej rusza J4/J5.
SCORE_WEIGHTS = {
    "joint_1": 7.0,
    "joint_2": 2.0,
    "joint_3": 2.0,
    "joint_4": 14.0,
    "joint_5": 18.0,
}

PRECISION_SCORE_WEIGHTS = {
    "joint_1": 10.0,
    "joint_2": 2.0,
    "joint_3": 2.0,
    "joint_4": 120.0,
    "joint_5": 220.0,
}

WORKSPACE_X_MAX = 0.70
WORKSPACE_Y_MAX = 0.70
WORKSPACE_Z_MIN = -0.35
WORKSPACE_Z_MAX = 0.80

IK_TIMEOUT_NS = 250_000_000


class IKTargetNode(Node):
    def __init__(self):
        super().__init__("ik_target_node")

        self.current_joint_state = None
        self.busy = False

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            20,
        )

        self.create_subscription(
            Point,
            IK_TOPIC,
            self.target_callback,
            10,
        )

        self.create_subscription(
            Point,
            IK_PRECISION_TOPIC,
            self.precision_target_callback,
            10,
        )

        self.ik_client = self.create_client(
            GetPositionIK,
            "/compute_ik",
        )

        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

        self.get_logger().info("==========================================")
        self.get_logger().info("IK TARGET NODE V4 - POSITION + PRECISION WRIST LOCK")
        self.get_logger().info(f"Normalny cel XYZ: {IK_TOPIC}")
        self.get_logger().info(f"Precyzyjny cel XYZ: {IK_PRECISION_TOPIC}")
        self.get_logger().info("J4/J5 sa mocno premiowane za pozostanie blisko aktualnego ustawienia.")
        self.get_logger().info("Pelna orientacja NIE jest wymuszana.")
        self.get_logger().info("==========================================")

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def get_current_joint_map(self):
        if self.current_joint_state is None:
            return None

        result = {}
        for name, position in zip(
            self.current_joint_state.name,
            self.current_joint_state.position,
        ):
            if name in REQUIRED_JOINTS:
                result[name] = float(position)

        if len(result) != len(REQUIRED_JOINTS):
            return None

        return result

    @staticmethod
    def clamp_joint(name, value):
        lower, upper = JOINT_LIMITS[name]
        return max(lower, min(upper, value))

    def generate_seeds(self, current, precision=False):
        seeds = [current.copy()]

        if precision:
            variations = [
                (0.0, +0.04, -0.04, 0.0),
                (0.0, -0.04, +0.04, 0.0),
                (0.0, +0.08, -0.08, 0.0),
                (0.0, -0.08, +0.08, 0.0),
                (+math.radians(2.0), 0.0, 0.0, 0.0),
                (-math.radians(2.0), 0.0, 0.0, 0.0),
                (+math.radians(4.0), +0.05, -0.05, 0.0),
                (-math.radians(4.0), -0.05, +0.05, 0.0),
            ]
        else:
            variations = [
                (0.0, +0.12, -0.12, 0.0),
                (0.0, -0.12, +0.12, 0.0),
                (0.0, +0.25, -0.25, 0.0),
                (0.0, -0.25, +0.25, 0.0),
                (0.0, +0.40, -0.40, 0.0),
                (0.0, -0.40, +0.40, 0.0),
                (0.0, 0.0, 0.0, +0.10),
                (0.0, 0.0, 0.0, -0.10),
                (+math.radians(5.0), 0.0, 0.0, 0.0),
                (-math.radians(5.0), 0.0, 0.0, 0.0),
                (+math.radians(10.0), +0.15, -0.15, 0.0),
                (-math.radians(10.0), -0.15, +0.15, 0.0),
            ]

        for dj1, dj2, dj3, dj4 in variations:
            seed = current.copy()
            seed["joint_1"] += dj1
            seed["joint_2"] += dj2
            seed["joint_3"] += dj3
            seed["joint_4"] += dj4
            # J5 specjalnie zostawiamy na aktualnej pozycji.
            seed["joint_5"] = current["joint_5"]

            for joint in REQUIRED_JOINTS:
                seed[joint] = self.clamp_joint(joint, seed[joint])

            seeds.append(seed)

        return seeds

    def target_callback(self, msg):
        self.handle_target(msg, precision=False)

    def precision_target_callback(self, msg):
        self.handle_target(msg, precision=True)

    def handle_target(self, msg, precision):
        if self.busy:
            self.get_logger().warn("Robot wykonuje poprzedni ruch.")
            return

        current = self.get_current_joint_map()
        if current is None:
            self.get_logger().error("Brak aktualnych /joint_states.")
            return

        x = float(msg.x)
        y = float(msg.y)
        z = float(msg.z)

        if not all(math.isfinite(v) for v in (x, y, z)):
            self.get_logger().error("Cel zawiera NaN/Inf.")
            return

        if (
            abs(x) > WORKSPACE_X_MAX
            or abs(y) > WORKSPACE_Y_MAX
            or z < WORKSPACE_Z_MIN
            or z > WORKSPACE_Z_MAX
        ):
            self.get_logger().error(
                f"Cel poza workspace: XYZ=({x:+.4f},{y:+.4f},{z:+.4f}), "
                f"Zmin={WORKSPACE_Z_MIN:+.3f}"
            )
            return

        mode = "PRECISION_WRIST_LOCK" if precision else "NORMAL_POSITION_ONLY"

        self.get_logger().info("")
        self.get_logger().info("==========================================")
        self.get_logger().info(f"NOWY CEL TCP - {mode}")
        self.get_logger().info(f"XYZ = ({x:+.4f}, {y:+.4f}, {z:+.4f}) m")
        if precision:
            self.get_logger().info(
                "J4 max 0.75 deg / waypoint, J5 max 0.25 deg / waypoint."
            )
        self.get_logger().info("==========================================")

        self.busy = True
        self.solve_multi_seed(x, y, z, current, precision)

    def make_ik_request(self, x, y, z, seed_map):
        request = GetPositionIK.Request()
        request.ik_request.group_name = PLANNING_GROUP
        request.ik_request.ik_link_name = TCP_LINK
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = IK_TIMEOUT_NS

        pose = request.ik_request.pose_stamped
        pose.header.frame_id = BASE_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # KDL ma pracowac w trybie position_only_ik=true.
        # Quaternion musi byc poprawny formalnie, ale w tym trybie jest ignorowany.
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        state = RobotState()
        state.joint_state.name = list(REQUIRED_JOINTS)
        state.joint_state.position = [seed_map[j] for j in REQUIRED_JOINTS]
        request.ik_request.robot_state = state

        return request

    def solve_multi_seed(self, x, y, z, current, precision):
        if not self.ik_client.service_is_ready():
            self.get_logger().error("/compute_ik nie jest dostepne.")
            self.busy = False
            return

        seeds = self.generate_seeds(current, precision=precision)
        solutions = []
        self.try_seed(0, seeds, x, y, z, current, solutions, precision)

    def try_seed(self, index, seeds, x, y, z, current, solutions, precision):
        if index >= len(seeds):
            self.finish_seed_search(current, solutions, precision)
            return

        request = self.make_ik_request(x, y, z, seeds[index])
        future = self.ik_client.call_async(request)
        future.add_done_callback(
            lambda fut: self.seed_result_callback(
                fut,
                index,
                seeds,
                x,
                y,
                z,
                current,
                solutions,
                precision,
            )
        )

    def seed_result_callback(
        self,
        future,
        index,
        seeds,
        x,
        y,
        z,
        current,
        solutions,
        precision,
    ):
        response = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Seed {index + 1}: IK exception: {exc}")

        if response is not None and response.error_code.val == 1:
            solution = {}
            for name, position in zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            ):
                if name in REQUIRED_JOINTS:
                    solution[name] = float(position)

            if len(solution) == len(REQUIRED_JOINTS):
                safe, reason = self.solution_is_safe(current, solution, precision)
                score = self.solution_score(current, solution, precision)

                if safe:
                    self.get_logger().info(
                        f"Seed {index + 1}: OK SAFE score={score:.4f}"
                    )
                    solutions.append((score, solution, index))
                else:
                    self.get_logger().warn(
                        f"Seed {index + 1}: ODRZUCONY: {reason}"
                    )

        self.try_seed(
            index + 1,
            seeds,
            x,
            y,
            z,
            current,
            solutions,
            precision,
        )

    @staticmethod
    def solution_is_safe(current, solution, precision=False):
        for joint in REQUIRED_JOINTS:
            value = solution[joint]
            lower, upper = JOINT_LIMITS[joint]

            if value < lower or value > upper:
                return False, f"{joint} poza limitem"

            delta = value - current[joint]
            limits = PRECISION_MAX_JOINT_DELTA if precision else MAX_JOINT_DELTA
            if abs(delta) > limits[joint]:
                return (
                    False,
                    f"{joint} delta={math.degrees(delta):+.2f} deg > "
                    f"{math.degrees(limits[joint]):.2f} deg",
                )

        return True, "OK"

    @staticmethod
    def solution_score(current, solution, precision=False):
        score = 0.0
        weights = PRECISION_SCORE_WEIGHTS if precision else SCORE_WEIGHTS
        for joint in REQUIRED_JOINTS:
            delta = abs(solution[joint] - current[joint])
            score += weights[joint] * delta
        return score

    def finish_seed_search(self, current, solutions, precision):
        if not solutions:
            mode = "PRECISION" if precision else "NORMAL"
            self.get_logger().error(
                f"IK ERROR [{mode}]: brak bezpiecznego rozwiazania. "
                "W precision nie robie fallbacku do ruchu nadgarstka."
            )
            self.busy = False
            return

        solutions.sort(key=lambda item: item[0])
        score, solution, seed_index = solutions[0]

        self.get_logger().info("")
        self.get_logger().info("==========================================")
        self.get_logger().info(f"WYBRANO SAFE SEED {seed_index + 1} ({'PRECISION' if precision else 'NORMAL'})")
        self.get_logger().info(f"Score = {score:.4f}")
        self.get_logger().info(f"Bezpiecznych rozwiazan: {len(solutions)}")

        for joint in REQUIRED_JOINTS:
            delta = solution[joint] - current[joint]
            self.get_logger().info(
                f"{joint}: {current[joint]:+.4f} -> {solution[joint]:+.4f} "
                f"delta={math.degrees(delta):+.1f} deg"
            )

        self.get_logger().info("==========================================")
        self.send_trajectory(current, solution)

    def send_trajectory(self, current, target):
        if not self.trajectory_client.server_is_ready():
            self.get_logger().error("Brak arm_controller.")
            self.busy = False
            return

        max_delta = max(
            abs(target[j] - current[j])
            for j in REQUIRED_JOINTS
        )

        duration = max(1.2, min(4.5, 1.2 + max_delta * 2.2))

        trajectory = JointTrajectory()
        trajectory.joint_names = list(REQUIRED_JOINTS)

        point = JointTrajectoryPoint()
        point.positions = [target[j] for j in REQUIRED_JOINTS]
        seconds = int(duration)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int((duration - seconds) * 1e9)
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(f"Wysylam trajectory: {duration:.2f} s")
        future = self.trajectory_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Trajectory exception: {exc}")
            self.busy = False
            return

        if not goal_handle.accepted:
            self.get_logger().error("Controller odrzucil trajectory.")
            self.busy = False
            return

        self.get_logger().info("Trajectory zaakceptowana.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.trajectory_result_callback)

    def trajectory_result_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(
                f"Trajectory zakonczona. status={result.status}"
            )
        except Exception as exc:
            self.get_logger().error(f"Blad trajectory result: {exc}")
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = IKTargetNode()

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