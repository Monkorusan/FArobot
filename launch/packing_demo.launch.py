import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("FArobot")
    default_config = os.path.join(share_dir, "config", "boxes.yaml")

    rviz_config = os.path.join(share_dir, "config", "packing_demo.rviz")

    return LaunchDescription(
        [
            Node(
                package="FArobot",
                executable="packing_demo_node",
                name="packing_demo_node",
                output="screen",
                parameters=[
                    {"config": default_config},
                    {"frame_id": "world"},
                    {"publish_rate_hz": 1.0},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
