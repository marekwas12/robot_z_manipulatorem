import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK


CURRENT_X = 0.017
CURRENT_Y = -0.024
CURRENT_Z = 0.326

STEP = 0.005


TESTS = [
    ("X +5 mm", CURRENT_X + STEP, CURRENT_Y, CURRENT_Z),
    ("X -5 mm", CURRENT_X - STEP, CURRENT_Y, CURRENT_Z),

    ("Y +5 mm", CURRENT_X, CURRENT_Y + STEP, CURRENT_Z),
    ("Y -5 mm", CURRENT_X, CURRENT_Y - STEP, CURRENT_Z),

    ("Z +5 mm", CURRENT_X, CURRENT_Y, CURRENT_Z + STEP),
    ("Z -5 mm", CURRENT_X, CURRENT_Y, CURRENT_Z - STEP),
]


class IKTester(Node):

    def __init__(self):

        super().__init__("test_tcp_ik")

        self.client = self.create_client(
            GetPositionIK,
            "/compute_ik"
        )

        self.get_logger().info(
            "Czekam na /compute_ik..."
        )

        while not self.client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                "Nadal czekam na /compute_ik..."
            )


    def test_pose(self, name, x, y, z):

        request = GetPositionIK.Request()

        request.ik_request.group_name = "manipulator"

        request.ik_request.ik_link_name = "tool0"

        request.ik_request.avoid_collisions = True

        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = 300_000_000


        pose = PoseStamped()

        pose.header.frame_id = "base_link"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # Przy position_only_ik orientacja nie jest istotna.
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
            timeout_sec=1.0
        )


        print()
        print("=" * 50)
        print(name)
        print(
            f"X={x:.4f} "
            f"Y={y:.4f} "
            f"Z={z:.4f}"
        )


        if not future.done():

            print(">>> TIMEOUT")

            return


        response = future.result()

        if response is None:

            print(">>> BLAD USLUGI")

            return


        code = response.error_code.val


        # MoveItErrorCodes.SUCCESS = 1
        if code == 1:

            print(">>> IK OK")

            names = (
                response.solution
                .joint_state
                .name
            )

            positions = (
                response.solution
                .joint_state
                .position
            )


            for joint_name, position in zip(
                names,
                positions
            ):

                if joint_name.startswith(
                    "joint_"
                ):

                    print(
                        f"    {joint_name}: "
                        f"{position:.6f}"
                    )

        else:

            print(
                f">>> BRAK IK "
                f"(error_code={code})"
            )


def main(args=None):

    rclpy.init(args=args)

    node = IKTester()


    for name, x, y, z in TESTS:

        node.test_pose(
            name,
            x,
            y,
            z
        )


    print()
    print("=" * 50)
    print("KONIEC")
    print("Robot NIE byl poruszany.")


    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()
