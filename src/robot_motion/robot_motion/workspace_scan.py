import csv
import math
import time

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState

from sensor_msgs.msg import JointState


BASE_FRAME = "base_link"
TCP_LINK = "tool0"
GROUP = "manipulator"

STEP_M = 0.10

X_MIN = -0.55
X_MAX = 0.55

Y_MIN = -0.55
Y_MAX = 0.55

Z_MIN = 0.00
Z_MAX = 0.70

IK_TIMEOUT_NS = 30_000_000


class WorkspaceScanner(Node):

    def __init__(self):

        super().__init__(
            "workspace_scanner"
        )

        self.current_joint_state = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10,
        )

        self.ik_client = self.create_client(
            GetPositionIK,
            "/compute_ik",
        )

    def joint_callback(
        self,
        msg,
    ):

        self.current_joint_state = msg

    def wait_for_system(
        self,
    ):

        self.get_logger().info(
            "Czekam na /compute_ik..."
        )

        while not self.ik_client.wait_for_service(
            timeout_sec=1.0
        ):

            self.get_logger().info(
                "Nadal czekam na MoveIt..."
            )

        self.get_logger().info(
            "Czekam na /joint_states..."
        )

        while (
            rclpy.ok()
            and
            self.current_joint_state is None
        ):

            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        self.get_logger().info(
            "System gotowy."
        )

    def check_point(
        self,
        x,
        y,
        z,
    ):

        request = (
            GetPositionIK.Request()
        )

        request.ik_request.group_name = (
            GROUP
        )

        request.ik_request.ik_link_name = (
            TCP_LINK
        )

        request.ik_request.avoid_collisions = (
            True
        )

        request.ik_request.timeout.sec = 0

        request.ik_request.timeout.nanosec = (
            IK_TIMEOUT_NS
        )

        pose = PoseStamped()

        pose.header.frame_id = (
            BASE_FRAME
        )

        pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # position_only_ik
        pose.pose.orientation.w = 1.0

        request.ik_request.pose_stamped = (
            pose
        )

        if self.current_joint_state is not None:

            seed = RobotState()

            seed.joint_state.name = list(
                self.current_joint_state.name
            )

            seed.joint_state.position = list(
                self.current_joint_state.position
            )

            request.ik_request.robot_state = (
                seed
            )

        future = (
            self.ik_client.call_async(
                request
            )
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=0.15,
        )

        if not future.done():

            return False

        response = future.result()

        if response is None:

            return False

        return (
            response.error_code.val == 1
        )


def values(
    minimum,
    maximum,
    step,
):

    value = minimum

    while value <= maximum + 1e-9:

        yield round(
            value,
            6,
        )

        value += step


def main(
    args=None,
):

    rclpy.init(
        args=args
    )

    node = WorkspaceScanner()

    node.wait_for_system()

    output_file = (
        "workspace_points.csv"
    )

    reachable = []

    checked = 0

    start_time = time.monotonic()

    print()
    print(
        "======================================="
    )
    print(
        "WORKSPACE SCAN"
    )
    print(
        f"STEP = {STEP_M * 1000:.0f} mm"
    )
    print(
        "Fizyczny robot NIE BEDZIE RUSZANY."
    )
    print(
        "======================================="
    )
    print()

    with open(
        output_file,
        "w",
        newline="",
    ) as csv_file:

        writer = csv.writer(
            csv_file
        )

        writer.writerow(
            [
                "x_mm",
                "y_mm",
                "z_mm",
                "reachable",
            ]
        )

        for z in values(
            Z_MIN,
            Z_MAX,
            STEP_M,
        ):

            layer_reachable = 0

            for y in values(
                Y_MIN,
                Y_MAX,
                STEP_M,
            ):

                for x in values(
                    X_MIN,
                    X_MAX,
                    STEP_M,
                ):

                    ok = node.check_point(
                        x,
                        y,
                        z,
                    )

                    checked += 1

                    writer.writerow(
                        [
                            round(
                                x * 1000,
                                1,
                            ),
                            round(
                                y * 1000,
                                1,
                            ),
                            round(
                                z * 1000,
                                1,
                            ),
                            1 if ok else 0,
                        ]
                    )

                    if ok:

                        reachable.append(
                            (
                                x,
                                y,
                                z,
                            )
                        )

                        layer_reachable += 1

            print(
                f"Z = {z * 1000:6.1f} mm | "
                f"osiagalne = {layer_reachable}"
            )

    elapsed = (
        time.monotonic()
        -
        start_time
    )

    print()
    print(
        "======================================="
    )
    print(
        "WYNIK"
    )
    print(
        "======================================="
    )

    print(
        f"Sprawdzone punkty: {checked}"
    )

    print(
        f"Osiagalne punkty: {len(reachable)}"
    )

    print(
        f"Czas: {elapsed:.1f} s"
    )

    if reachable:

        xs = [
            p[0]
            for p in reachable
        ]

        ys = [
            p[1]
            for p in reachable
        ]

        zs = [
            p[2]
            for p in reachable
        ]

        print()
        print(
            "GLOBALNE GRANICE OSIAGALNYCH PUNKTOW:"
        )

        print(
            f"X: "
            f"{min(xs) * 1000:.0f} .. "
            f"{max(xs) * 1000:.0f} mm"
        )

        print(
            f"Y: "
            f"{min(ys) * 1000:.0f} .. "
            f"{max(ys) * 1000:.0f} mm"
        )

        print(
            f"Z: "
            f"{min(zs) * 1000:.0f} .. "
            f"{max(zs) * 1000:.0f} mm"
        )

        max_radius = max(
            math.sqrt(
                x * x
                +
                y * y
            )
            for x, y, _ in reachable
        )

        print(
            f"Maksymalny promien XY: "
            f"{max_radius * 1000:.0f} mm"
        )

    print()
    print(
        f"CSV zapisany: {output_file}"
    )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()
