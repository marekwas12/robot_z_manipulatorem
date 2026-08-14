from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lewa kamera
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_left',
            parameters=[{
                'camera': "/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a",
                'role': 'video',
                'format': 'RGB888',
                'width': 320,
                'height': 240,
                'FrameDurationLimits': [66666, 66666]
            }]
        ),
        # Prawa kamera
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_right',
            parameters=[{
                'camera': "/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a",
                'role': 'video',
                'format': 'RGB888',
                'width': 320,
                'height': 240,
                'FrameDurationLimits': [66666, 66666]
            }]
        ),
        # Stereo Processor
        Node(
            package='camera_bridge',
            executable='stereo_processor',
            name='stereo_processor'
        ),
        # Foxglove Bridge
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge'
        )
    ])