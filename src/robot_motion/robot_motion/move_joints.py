import rclpy

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState


def main():

    rclpy.init()

    # =====================================================
    # START MOVEIT
    # =====================================================

    moveit = MoveItPy(
        node_name="robot_motion"
    )

    manipulator = moveit.get_planning_component(
        "manipulator"
    )

    print("MoveItPy uruchomiony")
    print("Planning group: manipulator")


    # =====================================================
    # AKTUALNY STAN JAKO START
    # =====================================================

    manipulator.set_start_state_to_current_state()


    # =====================================================
    # TWORZYMY STAN DOCELOWY
    # =====================================================

    robot_model = moveit.get_robot_model()

    goal_state = RobotState(
        robot_model
    )


    # =====================================================
    # CEL JOINTÓW
    # =====================================================

    target = {
        "joint_1": -0.378151,
        "joint_2": 0.410086,
        "joint_3": 0.437975,
        "joint_4": 3.141592,
        "joint_5": 1.578253,
    }


    goal_state.joint_positions = target

    goal_state.update()


    # =====================================================
    # USTAWIAMY CEL PLANOWANIA
    # =====================================================

    success = manipulator.set_goal_state(
        robot_state=goal_state
    )

    if not success:

        print(
            "BLAD: nie udalo sie ustawic stanu docelowego"
        )

        rclpy.shutdown()
        return


    print("Cel ustawiony:")

    for joint, value in target.items():
        print(
            f"  {joint} = {value:.4f} rad"
        )


    # =====================================================
    # PLANOWANIE
    # =====================================================

    print("Planowanie...")

    plan_result = manipulator.plan()


    if not plan_result:

        print(
            "BLAD: MoveIt nie znalazl trajektorii"
        )

        rclpy.shutdown()
        return


    print("PLAN OK")


    # =====================================================
    # WYKONANIE
    # =====================================================

    print("Wykonywanie trajektorii...")

    moveit.execute(
        plan_result.trajectory,
        controllers=[]
    )

    print("RUCH ZAKONCZONY")


    rclpy.shutdown()


if __name__ == "__main__":
    main()