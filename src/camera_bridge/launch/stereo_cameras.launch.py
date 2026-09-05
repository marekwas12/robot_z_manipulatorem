from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    left_camera = Node(
        package="camera_ros",
        executable="camera_node",
        name="left_camera",
        namespace="camera_left",
        output="screen",
        parameters=[
            {
                "camera":
                    "/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a",

                "role": "video",
                "format": "RGB888",

                "width": 480,
                "height": 270,

                "FrameDurationLimits": [
                    200000,
                    200000,
                ],

                "frame_id": "camera_left_optical_frame",
            }
        ],
    )


    right_camera = Node(
        package="camera_ros",
        executable="camera_node",
        name="right_camera",
        namespace="camera_right",
        output="screen",
        parameters=[
            {
                "camera":
                    "/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a",

                "role": "video",
                "format": "RGB888",

                "width": 480,
                "height": 270,

                "FrameDurationLimits": [
                    200000,
                    200000,
                ],

                "frame_id": "camera_right_optical_frame",
            }
        ],
    )


    return LaunchDescription([
        left_camera,
        right_camera,
    ])
