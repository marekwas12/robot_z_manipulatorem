import os

from glob import glob
from setuptools import find_packages, setup


package_name = 'robot_motion'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='marek',
    maintainer_email='marek@todo.todo',

    description='Robot motion control',

    license='TODO',

    entry_points={
        'console_scripts': [
            'move_joints = robot_motion.move_joints:main',
            'serial_joint_bridge = robot_motion.serial_joint_bridge:main',
            'serial_trajectory_bridge = robot_motion.serial_trajectory_bridge:main',
            'move_tcp = robot_motion.move_tcp:main',
            'test_tcp_directions = robot_motion.test_tcp_directions:main',
            'test_tcp_ik = robot_motion.test_tcp_ik:main',
            'move_tcp_auto = robot_motion.move_tcp_auto:main',
            'move_tcp_relative = robot_motion.move_tcp_relative:main',
            'tcp_jog = robot_motion.tcp_jog:main',
            'tcp_keyboard = robot_motion.tcp_keyboard:main',
            'ik_target_node = robot_motion.ik_target_node:main',
            'workspace_scan = robot_motion.workspace_scan:main',
            'tcp_jog_node = robot_motion.tcp_jog_node:main',
            'manipulator_preset_node = robot_motion.preset_node:main',
            'cube_approach_node = robot_motion.cube_approach_node:main',
            'cube_pick_node = robot_motion.cube_pick_node:main',
            'cube_calibration_auto = robot_motion.cube_calibration_auto:main',
            'cartesian_jog_node = robot_motion.cartesian_jog_node:main',
            'cartesian_controller = robot_motion.cartesian_controller:main',
            'cube_place_node = robot_motion.cube_place_node:main',
            'trunk_loading_node = robot_motion.trunk_loading_node:main',
            'autonomous_drive_node = robot_motion.autonomous_drive_node:main',
            'cleanup_mission_node = robot_motion.cleanup_mission_node:main',
            'cleanup_troubleshoot_node = robot_motion.cleanup_troubleshoot_node:main',
            'cube_stack_pick_node = robot_motion.cube_stack_pick_node:main',
            'cube_stack_place_node = robot_motion.cube_stack_place_node:main',
            'tower_building_sequence = robot_motion.tower_building_sequence:main',
        ],
    },
)