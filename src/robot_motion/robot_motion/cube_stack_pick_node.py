#!/usr/bin/env python3
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_motion.arm_kinematics import (
    ArmKinematics,
    JOINT_NAMES,
    apply_radial_tangential_offset,
)

CAMERA_CUBE_TOPIC = "/cube_aruco/position_camera"
CUBE_YAW_TOPIC = "/cube_aruco/yaw_deg"
CUBE_YAW_QUALITY_TOPIC = "/cube_aruco/quality"

COMMAND_TOPIC = "/cube_stack_pick/command"
STATUS_TOPIC = "/cube_stack_pick/status"

CARTESIAN_TARGET_TOPIC = "/manipulator/cartesian_target"
ARDUINO_COMMAND_TOPIC = "/manipulator/command"
ACTION_NAME = "/arm_controller/follow_joint_trajectory"

CAMERA_SERVO = 1
CAMERA_PICK_ANGLE = 148.0

GRIPPER_SERVO = 5
GRIPPER_OPEN_ANGLE = 54.0
GRIPPER_CLOSE_ANGLE = 67.0

# ============================================================
# GEOMETRIA CHWYTU
# ============================================================
#
# Używamy DOKŁADNIE tej samej geometrii chwytu co działający cube_pick_node.
# Nie dodajemy żadnych osobnych przesunięć XYZ dla stack-pick.
#
GRASP_RADIAL_OFFSET_M = -0.050
GRASP_RIGHT_OFFSET_M = 0.000

PREGRASP_HEIGHT_M = 0.080
LIFT_HEIGHT_M = 0.080

CAMERA_SETTLE_TIME = 0.50
GRIPPER_OPEN_SETTLE_TIME = 0.30
GRIPPER_CLOSE_SETTLE_TIME = 0.60

CUBE_SAMPLE_COUNT = 4
YAW_SAMPLE_COUNT = 4
DETECTION_TIMEOUT = 35.0
MAX_CAMERA_SPREAD_M = 0.012
MAX_YAW_SPREAD_DEG = 5.0
MIN_YAW_QUALITY = 0.40

MAX_COMMAND_SEGMENT_M = 0.150
TARGET_TOLERANCE_M = 0.003
MOVE_SEGMENT_TIMEOUT = 12.0
INTER_SEGMENT_DELAY_S = 0.08

ARUCO_TO_TCP_MATRIX = np.array(
    [
        [0.88453485, -1.17355526, 1.26237240],
        [-0.99125178, -1.27969740, 1.39649531],
        [-0.07973974, 0.33350932, 0.09273243],
    ],
    dtype=float,
)

ARUCO_TO_TCP_OFFSET = np.array(
    [-0.10332456, -0.08068988, -0.30763703],
    dtype=float,
)

CAL_CAMERA_MIN = np.array(
    [-0.022512, -0.013296, +0.293775],
    dtype=float,
)

CAL_CAMERA_MAX = np.array(
    [+0.069997, +0.023496, +0.344621],
    dtype=float,
)

CAL_MARGIN_M = 0.015

WRIST_J4_CAL_DEG = 130.0
WRIST_J5_HOME_DEG = 82.0

J5_SERVO_MIN_DEG = 52.0
J5_SERVO_CENTER_DEG = 82.0
J5_SERVO_MAX_DEG = 112.0
J5_YAW_AT_MIN_SERVO_DEG = 29.639503002
J5_YAW_AT_MAX_SERVO_DEG = -24.063885212

IDLE = 0
SET_CAMERA = 1
OPEN_GRIPPER = 2
SET_WRIST_HOME = 3
COLLECT_CUBE = 4
MOVE_PREGRASP = 5
ROTATE_J5 = 6
MOVE_TO_GRASP = 7
CLOSE_GRIPPER = 8
MOVE_LIFT = 9
DONE = 10
ERROR = 11


def wrap_square_delta_deg(angle_deg):
    return ((float(angle_deg) + 45.0) % 90.0) - 45.0


def yaw_to_j5_deg(yaw_deg):
    delta = wrap_square_delta_deg(yaw_deg)

    if delta >= 0.0:
        span = max(1e-6, float(J5_YAW_AT_MIN_SERVO_DEG))
        alpha = max(0.0, min(1.0, delta / span))
        servo = (
            J5_SERVO_CENTER_DEG
            + alpha * (J5_SERVO_MIN_DEG - J5_SERVO_CENTER_DEG)
        )
    else:
        span = max(1e-6, abs(float(J5_YAW_AT_MAX_SERVO_DEG)))
        alpha = max(0.0, min(1.0, abs(delta) / span))
        servo = (
            J5_SERVO_CENTER_DEG
            + alpha * (J5_SERVO_MAX_DEG - J5_SERVO_CENTER_DEG)
        )

    return max(
        J5_SERVO_MIN_DEG,
        min(J5_SERVO_MAX_DEG, float(servo)),
    )


class CubeStackPickNode(Node):

    def __init__(self):
        super().__init__("cube_stack_pick_node")

        self.kin = ArmKinematics()
        self.current_q = None

        self.latest_yaw_deg = None
        self.latest_yaw_quality = None

        self.samples = deque(maxlen=CUBE_SAMPLE_COUNT)
        self.yaw_samples = deque(maxlen=YAW_SAMPLE_COUNT)

        self.state = IDLE
        self.state_started = self.get_clock().now()
        self.state_command_sent = False

        self.grasp_xyz = None
        self.pregrasp_xyz = None
        self.lift_xyz = None
        self.target_j5_deg = None

        self.stage_target_xyz = None
        self.segment_target_xyz = None
        self.segment_started = None
        self.target_stable_counter = 0
        self.next_segment_not_before = None

        self.wrist_motion_started = False
        self.wrist_motion_done = False

        self.create_subscription(
            PointStamped,
            CAMERA_CUBE_TOPIC,
            self.cube_callback,
            20,
        )

        self.create_subscription(
            Float32,
            CUBE_YAW_TOPIC,
            self.yaw_callback,
            20,
        )

        self.create_subscription(
            Float32,
            CUBE_YAW_QUALITY_TOPIC,
            self.yaw_quality_callback,
            20,
        )

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

        self.cartesian_pub = self.create_publisher(
            Point,
            CARTESIAN_TARGET_TOPIC,
            10,
        )

        self.arduino_pub = self.create_publisher(
            String,
            ARDUINO_COMMAND_TOPIC,
            10,
        )

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

        self.get_logger().info("CUBE STACK PICK NODE - SAME GRASP GEOMETRY AS cube_pick_node")

    def now(self):
        return self.get_clock().now()

    def state_time(self):
        return (self.now() - self.state_started).nanoseconds / 1e9

    def set_state(self, state):
        self.state = state
        self.state_started = self.now()
        self.state_command_sent = False
        self.target_stable_counter = 0

        if state == DONE:
            self.publish_status("done")
        elif state == ERROR:
            self.publish_status("error")

    def publish_status(self, text):
        msg = String()
        msg.data = str(text)
        self.status_pub.publish(msg)
        self.get_logger().info(f"STACK PICK STATUS -> {msg.data}")

    def fail(self, text):
        self.get_logger().error(text)
        self.set_state(ERROR)

    def send_arduino(self, text):
        msg = String()
        msg.data = text
        self.arduino_pub.publish(msg)
        self.get_logger().info(f"ARDUINO -> {text}")

    def joint_state_callback(self, msg):
        values = {}
        for name, pos in zip(msg.name, msg.position):
            if name in JOINT_NAMES:
                values[name] = float(pos)

        if len(values) == len(JOINT_NAMES):
            self.current_q = np.array(
                [values[name] for name in JOINT_NAMES],
                dtype=float,
            )

    def current_tcp(self):
        if self.current_q is None:
            return None
        return self.kin.tcp(self.current_q)

    def cube_callback(self, msg):
        if self.state != COLLECT_CUBE:
            return

        p = np.array(
            [float(msg.point.x), float(msg.point.y), float(msg.point.z)],
            dtype=float,
        )

        if not np.all(np.isfinite(p)):
            return

        if p[2] < 0.10 or p[2] > 2.0:
            return

        self.samples.append(p)

    def yaw_callback(self, msg):
        value = float(msg.data)

        if not np.isfinite(value):
            return

        value = wrap_square_delta_deg(value)
        self.latest_yaw_deg = value

        if (
            self.state == COLLECT_CUBE
            and self.latest_yaw_quality is not None
            and self.latest_yaw_quality >= MIN_YAW_QUALITY
        ):
            self.yaw_samples.append(value)

    def yaw_quality_callback(self, msg):
        value = float(msg.data)
        if np.isfinite(value):
            self.latest_yaw_quality = value

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()

        if cmd == "full":
            if self.state not in (IDLE, DONE, ERROR):
                self.get_logger().warn("STACK PICK juz trwa.")
                return

            if self.current_q is None:
                self.fail("Brak /joint_states.")
                return

            self.samples.clear()
            self.yaw_samples.clear()
            self.latest_yaw_deg = None
            self.latest_yaw_quality = None

            self.grasp_xyz = None
            self.pregrasp_xyz = None
            self.lift_xyz = None
            self.target_j5_deg = None

            self.wrist_motion_started = False
            self.wrist_motion_done = False

            self.publish_status("running")
            self.set_state(SET_CAMERA)
            return

        if cmd == "reset":
            self.set_state(IDLE)
            self.publish_status("idle")
            return

        self.get_logger().warn(f"Nieznana komenda: {cmd}")

    def stable_camera_position(self):
        if len(self.samples) < CUBE_SAMPLE_COUNT:
            return None

        data = np.asarray(self.samples, dtype=float)
        median = np.median(data, axis=0)

        spread = (
            np.max(data, axis=0)
            - np.min(data, axis=0)
        )

        if float(np.max(spread)) > MAX_CAMERA_SPREAD_M:
            return None

        return median

    def stable_yaw(self):
        if len(self.yaw_samples) < YAW_SAMPLE_COUNT:
            return None

        yaw = np.asarray(self.yaw_samples, dtype=float)
        center = float(np.median(yaw))

        if float(np.max(np.abs(yaw - center))) > MAX_YAW_SPREAD_DEG:
            return None

        return center

    def camera_to_grasp(self, camera):
        camera = np.asarray(camera, dtype=float)

        grasp = ARUCO_TO_TCP_MATRIX @ camera + ARUCO_TO_TCP_OFFSET
        grasp = grasp.copy()

        self.get_logger().info(
            "GRASP BEFORE ROBOT OFFSET = "
            f"({grasp[0]:+.4f},{grasp[1]:+.4f},{grasp[2]:+.4f})"
        )

        try:
            grasp = apply_radial_tangential_offset(
                grasp,
                forward_m=GRASP_RADIAL_OFFSET_M,
                right_m=GRASP_RIGHT_OFFSET_M,
                up_m=0.0,
            )
        except ValueError as exc:
            raise ValueError(
                f"Nie moge policzyc korekcji chwytu: {exc}"
            )

        self.get_logger().info(
            f"RADIAL OFFSET = {GRASP_RADIAL_OFFSET_M*1000:+.1f} mm"
        )
        self.get_logger().info(
            f"RIGHT OFFSET = {GRASP_RIGHT_OFFSET_M*1000:+.1f} mm"
        )
        self.get_logger().info(
            "GRASP AFTER ROBOT OFFSET = "
            f"({grasp[0]:+.4f},{grasp[1]:+.4f},{grasp[2]:+.4f})"
        )

        return grasp

    def start_wrist_target(self, target_j4_deg, target_j5_deg):
        if self.current_q is None:
            self.fail("Brak joint_states do wrist.")
            return

        if not self.trajectory_client.server_is_ready():
            self.fail("Brak FollowJointTrajectory.")
            return

        q = self.current_q.copy()
        q[3] = math.radians(float(target_j4_deg))
        q[4] = math.radians(float(target_j5_deg))

        trajectory = JointTrajectory()
        trajectory.joint_names = list(JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.wrist_motion_started = True
        self.wrist_motion_done = False

        future = self.trajectory_client.send_goal_async(goal)
        future.add_done_callback(self.wrist_goal_response)

    def wrist_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self.fail(f"Wrist goal exception: {exc}")
            return

        if not handle.accepted:
            self.fail("Wrist trajectory odrzucona.")
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(self.wrist_result)

    def wrist_result(self, future):
        try:
            future.result()
            self.wrist_motion_done = True
        except Exception as exc:
            self.fail(f"Wrist result exception: {exc}")

    def start_stage_move(self, target):
        self.stage_target_xyz = np.asarray(target, dtype=float)
        self.segment_target_xyz = None
        self.next_segment_not_before = None
        self.target_stable_counter = 0
        self.send_next_segment()

    def send_next_segment(self):
        current = self.current_tcp()

        if current is None:
            self.fail("Brak TCP.")
            return

        delta = self.stage_target_xyz - current
        distance = float(np.linalg.norm(delta))

        if distance <= TARGET_TOLERANCE_M:
            self.segment_target_xyz = None
            self.on_stage_reached()
            return

        step = min(MAX_COMMAND_SEGMENT_M, distance)
        target = current + delta * (step / distance)

        self.segment_target_xyz = target
        self.segment_started = self.now()

        msg = Point()
        msg.x, msg.y, msg.z = [float(v) for v in target]
        self.cartesian_pub.publish(msg)

        self.get_logger().info(
            f"SEGMENT -> ({target[0]:+.4f},{target[1]:+.4f},{target[2]:+.4f})"
        )

    def update_stage_move(self):
        if self.segment_target_xyz is None:
            if (
                self.next_segment_not_before is not None
                and self.now() >= self.next_segment_not_before
            ):
                self.next_segment_not_before = None
                self.send_next_segment()
            return

        current = self.current_tcp()
        if current is None:
            return

        error = float(
            np.linalg.norm(
                self.segment_target_xyz
                - current
            )
        )

        if error <= TARGET_TOLERANCE_M:
            self.target_stable_counter += 1
        else:
            self.target_stable_counter = 0

        if self.target_stable_counter >= 2:
            self.segment_target_xyz = None
            self.target_stable_counter = 0
            self.next_segment_not_before = (
                self.now()
                + rclpy.duration.Duration(
                    seconds=INTER_SEGMENT_DELAY_S
                )
            )
            return

        elapsed = (
            self.now()
            - self.segment_started
        ).nanoseconds / 1e9

        if elapsed > MOVE_SEGMENT_TIMEOUT:
            current_txt = (
                f"TCP=({current[0]:+.4f},{current[1]:+.4f},{current[2]:+.4f})"
            )

            target = self.segment_target_xyz

            self.fail(
                "TIMEOUT ruchu | "
                f"error={error*1000:.1f} mm | "
                f"{current_txt} | "
                f"TARGET=({target[0]:+.4f},{target[1]:+.4f},{target[2]:+.4f})"
            )

    def on_stage_reached(self):
        if self.state == MOVE_PREGRASP:
            self.set_state(ROTATE_J5)
            return

        if self.state == MOVE_TO_GRASP:
            self.set_state(CLOSE_GRIPPER)
            return

        if self.state == MOVE_LIFT:
            self.get_logger().info(
                "STACK PICK: kostka podniesiona."
            )
            self.set_state(DONE)

    def update(self):
        if self.state in (IDLE, DONE, ERROR):
            return

        if self.state == SET_CAMERA:
            if not self.state_command_sent:
                self.send_arduino(
                    f"@S{CAMERA_SERVO}:{CAMERA_PICK_ANGLE:.0f}"
                )
                self.state_command_sent = True

            if self.state_time() >= CAMERA_SETTLE_TIME:
                self.set_state(OPEN_GRIPPER)
            return

        if self.state == OPEN_GRIPPER:
            if not self.state_command_sent:
                self.send_arduino(
                    f"@S{GRIPPER_SERVO}:{GRIPPER_OPEN_ANGLE:.0f}"
                )
                self.state_command_sent = True

            if self.state_time() >= GRIPPER_OPEN_SETTLE_TIME:
                self.set_state(SET_WRIST_HOME)
            return

        if self.state == SET_WRIST_HOME:
            if not self.wrist_motion_started:
                self.start_wrist_target(
                    WRIST_J4_CAL_DEG,
                    WRIST_J5_HOME_DEG,
                )
                return

            if self.wrist_motion_done:
                self.wrist_motion_started = False
                self.wrist_motion_done = False
                self.samples.clear()
                self.yaw_samples.clear()
                self.set_state(COLLECT_CUBE)
            return

        if self.state == COLLECT_CUBE:
            if self.state_time() > DETECTION_TIMEOUT:
                self.fail("TIMEOUT swiezego ArUco.")
                return

            camera = self.stable_camera_position()
            yaw = self.stable_yaw()

            if camera is None or yaw is None:
                return

            try:
                grasp = self.camera_to_grasp(camera)
            except ValueError as exc:
                self.fail(str(exc))
                return

            self.grasp_xyz = grasp
            self.pregrasp_xyz = grasp + np.array(
                [0.0, 0.0, PREGRASP_HEIGHT_M],
                dtype=float,
            )
            self.lift_xyz = grasp + np.array(
                [0.0, 0.0, LIFT_HEIGHT_M],
                dtype=float,
            )

            self.target_j5_deg = yaw_to_j5_deg(yaw)

            self.get_logger().info(
                "STACK PICK LOCK | "
                f"CAM=({camera[0]:+.4f},{camera[1]:+.4f},{camera[2]:+.4f}) | "
                f"GRASP=({grasp[0]:+.4f},{grasp[1]:+.4f},{grasp[2]:+.4f}) | "
                f"yaw={yaw:+.1f}"
            )

            self.set_state(MOVE_PREGRASP)
            self.start_stage_move(self.pregrasp_xyz)
            return

        if self.state == MOVE_PREGRASP:
            self.update_stage_move()
            return

        if self.state == ROTATE_J5:
            if not self.wrist_motion_started:
                self.start_wrist_target(
                    WRIST_J4_CAL_DEG,
                    self.target_j5_deg,
                )
                return

            if self.wrist_motion_done:
                self.wrist_motion_started = False
                self.wrist_motion_done = False
                self.set_state(MOVE_TO_GRASP)
                self.start_stage_move(self.grasp_xyz)
            return

        if self.state == MOVE_TO_GRASP:
            self.update_stage_move()
            return

        if self.state == CLOSE_GRIPPER:
            if not self.state_command_sent:
                self.send_arduino(
                    f"@S{GRIPPER_SERVO}:{GRIPPER_CLOSE_ANGLE:.0f}"
                )
                self.state_command_sent = True

            if self.state_time() >= GRIPPER_CLOSE_SETTLE_TIME:
                self.set_state(MOVE_LIFT)
                self.start_stage_move(self.lift_xyz)
            return

        if self.state == MOVE_LIFT:
            self.update_stage_move()
            return


def main(args=None):
    rclpy.init(args=args)
    node = CubeStackPickNode()

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