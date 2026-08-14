import rclpy

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy


# =====================================================
# CEL TCP
#
# Aktualna pozycja:
# X = 0.017
# Y = -0.024
# Z = 0.326
#
# Pierwszy test:
# +5 mm w Z
# =====================================================

TARGET_X = 0.017
TARGET_Y = -0.024
TARGET_Z = 0.331


# =====================================================
# AKTUALNA ORIENTACJA TOOL0
#
# Nie zmieniamy jej.
# =====================================================

QX = -0.524
QY = 0.768
QZ = 0.207
QW = 0.304


def main(args=None):

    rclpy.init(args=args)

    print("Uruchamiam MoveItPy TCP")

    moveit = MoveItPy(
        node_name="robot_motion_tcp"
    )

    manipulator = moveit.get_planning_component(
        "manipulator"
    )

    print(
        f"Planning group: "
        f"{manipulator.planning_group_name}"
    )


    # =================================================
    # START = AKTUALNY STAN FIZYCZNEGO ROBOTA
    # =================================================

    manipulator.set_start_state_to_current_state()


    # =================================================
    # CEL TCP
    # =================================================

    target_pose = PoseStamped()

    target_pose.header.frame_id = "base_link"

    target_pose.pose.position.x = TARGET_X
    target_pose.pose.position.y = TARGET_Y
    target_pose.pose.position.z = TARGET_Z

    target_pose.pose.orientation.x = QX
    target_pose.pose.orientation.y = QY
    target_pose.pose.orientation.z = QZ
    target_pose.pose.orientation.w = QW


    print(
        "Cel TCP:"
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


    # =================================================
    # CEL POSE DLA TOOL0
    # =================================================

    goal_ok = manipulator.set_goal_state(
        pose_stamped_msg=target_pose,
        pose_link="tool0",
    )


    if not goal_ok:

        print(
            "BLAD: MoveIt nie zaakceptowal celu TCP"
        )

        return


    # =================================================
    # PLAN
    # =================================================

    print(
        "Planuje..."
    )

    plan_result = manipulator.plan()


    if not plan_result:

        print(
            "PLANOWANIE NIEUDANE"
        )

        return


    print(
        "PLAN OK"
    )


    # =================================================
    # EXECUTE
    #
    # MoveIt wysle FollowJointTrajectory do:
    #
    # /arm_controller/follow_joint_trajectory
    #
    # czyli do serial_trajectory_bridge.
    # =================================================

    print(
        "Wykonuje trajektorie..."
    )

    status = moveit.execute(
        plan_result.trajectory,
        controllers=[]
    )


    print(
        f"Wynik wykonania: {status}"
    )


if __name__ == "__main__":
    main()