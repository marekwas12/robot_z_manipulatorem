from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    full_path = os.path.join(package_path, relative_path)

    with open(full_path, "r") as f:
        return yaml.safe_load(f)


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
    # PODSTAWOWA KONFIGURACJA MOVEIT
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

        .to_moveit_configs()
    )


    # =====================================================
    # KONFIGURACJA OMPL
    # =====================================================

    ompl_yaml = load_yaml(
        "robot_moveit_config",
        "config/ompl_planning.yaml"
    )


    # =====================================================
    # PARAMETRY SPECYFICZNE DLA MOVEITPY
    # =====================================================

    moveit_py_parameters = {

        # MoveItPy oczekuje pipeline_names
        "planning_pipelines": {
            "pipeline_names": [
                "ompl"
            ]
        },

        # Domyślne parametry planowania
        "plan_request_params": {
            "planning_attempts": 1,
            "planning_pipeline": "ompl",
            "planner_id": "RRTConnectkConfigDefault",
            "max_velocity_scaling_factor": 0.5,
            "max_acceleration_scaling_factor": 0.5,
            "planning_time": 5.0,
        },

        # Opcje monitora sceny
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic":
                "/moveit_cpp/attached_collision_object",
            "publish_planning_scene_topic":
                "/moveit_cpp/publish_planning_scene",
            "monitored_planning_scene_topic":
                "/moveit_cpp/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },

        # Konfiguracja właściwego pipeline OMPL
        "ompl": ompl_yaml,
    }


    # =====================================================
    # MOVEIT PY NODE
    # =====================================================

    move_joints = Node(
        package="robot_motion",
        executable="move_tcp_auto",
        name="robot_motion",
        output="screen",

        parameters=[
            moveit_config.to_dict(),
            moveit_py_parameters,
        ]
    )


    return LaunchDescription([
        move_joints
    ])