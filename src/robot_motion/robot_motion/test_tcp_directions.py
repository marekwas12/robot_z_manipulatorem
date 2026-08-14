import rclpy

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy


# Aktualne TCP z TF
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


def main(args=None):

    rclpy.init(args=args)

    moveit = MoveItPy(
        node_name="test_tcp_directions"
    )

    manipulator = moveit.get_planning_component(
        "manipulator"
    )

    for name, x, y, z in TESTS:

        print()
        print("=" * 50)
        print(name)
        print(
            f"cel: X={x:.4f} "
            f"Y={y:.4f} "
            f"Z={z:.4f}"
        )

        manipulator.set_start_state_to_current_state()

        pose = PoseStamped()

        pose.header.frame_id = "base_link"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # Quaternion jest nieistotny przy position_only_ik,
        # ale Pose musi zawierac poprawny quaternion.
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        ok = manipulator.set_goal_state(
            pose_stamped_msg=pose,
            pose_link="tool0",
        )

        if not ok:
            print("GOAL ODRZUCONY")
            continue

        plan = manipulator.plan()

        if plan:
            print(">>> OSIAGALNY")
        else:
            print(">>> NIEOSIAGALNY")

    print()
    print("KONIEC TESTU - robot nie byl poruszany")


if __name__ == "__main__":
    main()
