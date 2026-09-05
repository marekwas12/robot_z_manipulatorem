# FAST PICK COMPANION: 3 mm IK waypoints, 0.06 s/segment
import math

import numpy as np
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_motion.arm_kinematics import (
    ArmKinematics,
    JOINT_NAMES,
    TASK_FRAME_YAW_DEG,
    radial_and_left_units,
    task_delta_to_base,
)


JOG_TOPIC = "/manipulator/cartesian_jog"
ABSOLUTE_TARGET_TOPIC = "/manipulator/cartesian_target"

ACTION_NAME = "/arm_controller/follow_joint_trajectory"

DEFAULT_STEP_M = 0.005

# Jeden waypoint modelowy co 1 mm.
MAX_CARTESIAN_SEGMENT_M = 0.003

# Czas jednego 1 mm segmentu.
SECONDS_PER_SEGMENT = 0.06

MAX_TOTAL_JOG_M = 0.050
MAX_TOTAL_ABSOLUTE_M = 0.160


class CartesianController(Node):

    def __init__(self):
        super().__init__("cartesian_controller")

        self.kin = ArmKinematics()

        self.current_q = None
        self.step_m = DEFAULT_STEP_M
        self.busy = False

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            20,
        )

        self.create_subscription(
            String,
            JOG_TOPIC,
            self.jog_callback,
            10,
        )

        self.create_subscription(
            Point,
            ABSOLUTE_TARGET_TOPIC,
            self.absolute_target_callback,
            10,
        )

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            ACTION_NAME,
        )

        self.get_logger().info("==========================================")
        self.get_logger().info("CARTESIAN CONTROLLER FAST - WLASNA IK")
        self.get_logger().info("BEZ MoveIt / BEZ compute_ik")
        self.get_logger().info("J4: ZAMROZONY podczas ruchu XYZ")
        self.get_logger().info("J5: ZAMROZONY podczas ruchu XYZ")
        self.get_logger().info(
            f"JOG: {JOG_TOPIC} | step={self.step_m*1000:.1f} mm"
        )
        self.get_logger().info(
            "x+/x- = fizycznie PRAWO/LEWO robota"
        )
        self.get_logger().info(
            "y+/y- = fizycznie PRZOD/TYL robota"
        )
        self.get_logger().info(
            "z+/z- = GORA/DOL"
        )
        self.get_logger().info(
            f"TASK FRAME yaw = {TASK_FRAME_YAW_DEG:.1f} deg wzgledem base_link"
        )
        self.get_logger().info(
            "forward/back/left/right/up/down pozostaja jako stare jogi"
        )
        self.get_logger().info("==========================================")

    def joint_state_callback(self, msg):
        values = {}
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                values[name] = float(pos)

        if len(values) != len(JOINT_NAMES):
            return

        self.current_q = np.array(
            [values[name] for name in JOINT_NAMES],
            dtype=float,
        )

    def jog_callback(self, msg):
        command = msg.data.strip().lower()

        if command.startswith("step:"):
            try:
                value = float(command.split(":", 1)[1])
            except ValueError:
                self.get_logger().error("Niepoprawny step.")
                return

            if not (0.001 <= value <= MAX_TOTAL_JOG_M):
                self.get_logger().error(
                    f"Step musi byc 0.001 ... {MAX_TOTAL_JOG_M:.3f} m"
                )
                return

            self.step_m = value
            self.get_logger().info(
                f"STEP = {self.step_m*1000:.1f} mm"
            )
            return

        if command == "status":
            self.print_status()
            return

        if command not in (
            "x+",
            "x-",
            "y+",
            "y-",
            "z+",
            "z-",
            "forward",
            "back",
            "left",
            "right",
            "up",
            "down",
        ):
            self.get_logger().warn(f"Nieznana komenda: {command}")
            return

        if self.current_q is None:
            self.get_logger().error("Brak /joint_states.")
            return

        if self.busy:
            self.get_logger().warn("Controller jest BUSY.")
            return

        q0 = self.current_q.copy()
        p0 = self.kin.tcp(q0)

        # x/y/z z telefonu sa teraz w FIZYCZNYM ukladzie robota:
        #
        #   +X = PRAWO
        #   +Y = PRZOD
        #   +Z = GORA
        #
        # task_delta_to_base() przelicza ten wektor na matematyczny base_link.
        # Sama FK/IK pozostaje bez zmian w base_link.
        if command == "x+":
            delta = task_delta_to_base(
                +self.step_m,
                0.0,
                0.0,
            )
        elif command == "x-":
            delta = task_delta_to_base(
                -self.step_m,
                0.0,
                0.0,
            )
        elif command == "y+":
            delta = task_delta_to_base(
                0.0,
                +self.step_m,
                0.0,
            )
        elif command == "y-":
            delta = task_delta_to_base(
                0.0,
                -self.step_m,
                0.0,
            )
        elif command == "z+":
            delta = task_delta_to_base(
                0.0,
                0.0,
                +self.step_m,
            )
        elif command == "z-":
            delta = task_delta_to_base(
                0.0,
                0.0,
                -self.step_m,
            )
        else:
            # Stare komendy zostawione dla kompatybilnosci.
            try:
                forward, left = radial_and_left_units(p0)
            except ValueError as exc:
                self.get_logger().error(str(exc))
                return

            if command == "forward":
                delta = forward * self.step_m
            elif command == "back":
                delta = -forward * self.step_m
            elif command == "left":
                delta = left * self.step_m
            elif command == "right":
                delta = -left * self.step_m
            elif command == "up":
                delta = np.array([0.0, 0.0, +self.step_m])
            else:
                delta = np.array([0.0, 0.0, -self.step_m])

        target = p0 + delta

        self.get_logger().info("")
        self.get_logger().info(
            f"JOG {command}: {self.step_m*1000:.1f} mm"
        )
        self.get_logger().info(
            f"MODEL TCP start = "
            f"({p0[0]:+.4f},{p0[1]:+.4f},{p0[2]:+.4f})"
        )
        self.get_logger().info(
            f"MODEL TCP cel   = "
            f"({target[0]:+.4f},{target[1]:+.4f},{target[2]:+.4f})"
        )
        self.get_logger().info(
            f"DELTA base_link = "
            f"({delta[0]*1000:+.2f},"
            f"{delta[1]*1000:+.2f},"
            f"{delta[2]*1000:+.2f}) mm"
        )
        self.get_logger().info(
            f"LOCK: J4={math.degrees(q0[3]):.2f} deg, "
            f"J5={math.degrees(q0[4]):.2f} deg"
        )

        self.plan_and_send(q0, target)

    def absolute_target_callback(self, msg):
        if self.current_q is None:
            self.get_logger().error("Brak /joint_states.")
            return

        if self.busy:
            self.get_logger().warn("Controller jest BUSY.")
            return

        q0 = self.current_q.copy()
        p0 = self.kin.tcp(q0)

        target = np.array(
            [float(msg.x), float(msg.y), float(msg.z)],
            dtype=float,
        )

        distance = float(np.linalg.norm(target - p0))
        if distance > MAX_TOTAL_ABSOLUTE_M:
            self.get_logger().error(
                f"Cel absolutny jest {distance*1000:.1f} mm od TCP. "
                f"Limit jednego polecenia = "
                f"{MAX_TOTAL_ABSOLUTE_M*1000:.0f} mm."
            )
            return

        self.plan_and_send(q0, target)

    def print_status(self):
        if self.current_q is None:
            self.get_logger().info("Brak joint_states.")
            return

        p = self.kin.tcp(self.current_q)

        self.get_logger().info(
            f"Q deg = {[round(math.degrees(v), 2) for v in self.current_q]}"
        )
        self.get_logger().info(
            f"MODEL TCP = ({p[0]:+.5f}, {p[1]:+.5f}, {p[2]:+.5f})"
        )

    def plan_and_send(self, q_start, target_xyz):
        p_start = self.kin.tcp(q_start)

        distance = float(np.linalg.norm(target_xyz - p_start))
        if distance < 1e-6:
            return

        segments = max(
            1,
            int(math.ceil(distance / MAX_CARTESIAN_SEGMENT_M)),
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)

        q_prev = q_start.copy()

        for i in range(1, segments + 1):
            alpha = i / segments
            waypoint_xyz = p_start + alpha * (target_xyz - p_start)

            ok, q_next, error = self.kin.solve_local_position(
                q_prev,
                waypoint_xyz,
                lock_j4=True,
                lock_j5=True,
            )

            if not ok:
                self.get_logger().error(
                    f"IK lokalna FAIL waypoint {i}/{segments}: "
                    f"error={error*1000:.2f} mm"
                )
                return

            # Dodatkowa kontrola: J4/J5 maja zostac DOKLADNIE takie same.
            q_next[3] = q_start[3]
            q_next[4] = q_start[4]

            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in q_next]

            t = i * SECONDS_PER_SEGMENT
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int(
                (t - int(t)) * 1e9
            )

            trajectory.points.append(point)
            q_prev = q_next

        final_model = self.kin.tcp(q_prev)
        model_error = float(np.linalg.norm(target_xyz - final_model))

        self.get_logger().info(
            f"Plan: {segments} waypointow, "
            f"model error={model_error*1000:.2f} mm"
        )

        if model_error > 0.001:
            self.get_logger().error(
                "Modelowy blad koncowy > 1 mm. Nie wysylam."
            )
            return

        if not self.action_client.server_is_ready():
            self.get_logger().error(
                f"Brak action servera {ACTION_NAME}"
            )
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.busy = True
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Goal exception: {exc}")
            self.busy = False
            return

        if not goal_handle.accepted:
            self.get_logger().error("Trajectory odrzucona.")
            self.busy = False
            return

        self.get_logger().info("Trajectory zaakceptowana.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            wrapped = future.result()
            self.get_logger().info(
                f"Trajectory zakonczona, status={wrapped.status}"
            )
        except Exception as exc:
            self.get_logger().error(f"Result exception: {exc}")

        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = CartesianController()

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