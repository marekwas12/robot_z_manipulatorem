from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    robot_description_package = get_package_share_directory(
        "robot_description"
    )

    robot_xacro = os.path.join(
        robot_description_package,
        "urdf",
        "robot.urdf.xacro"
    )


    # =====================================================
    # MOVEIT CONFIG
    # =====================================================

    moveit_config = (
        MoveItConfigsBuilder(
            "marek_robot",
            package_name="robot_moveit_config"
        )

        .robot_description(
            file_path=robot_xacro
        )

        .robot_description_semantic(
            file_path="config/marek_robot.srdf"
        )

        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )

        .joint_limits(
            file_path="config/joint_limits.yaml"
        )

        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )

        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
            load_all=False
        )

        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )

        .to_moveit_configs()
    )


    # =====================================================
    # MOVE GROUP
    # =====================================================

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",

        parameters=[
            moveit_config.to_dict(),

            {
                "allow_trajectory_execution": True,
                "moveit_manage_controllers": True
            }
        ]
    )


    return LaunchDescription([
        move_group
    ])