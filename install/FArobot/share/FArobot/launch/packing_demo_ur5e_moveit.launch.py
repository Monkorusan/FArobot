import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("FArobot")
    default_config = os.path.join(share_dir, "config", "boxes.yaml")

    ur_moveit_launch = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "launch",
        "ur_moveit.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_moveit_launch),
                launch_arguments={"ur_type": "ur5e"}.items(),
            ),
            Node(
                package="FArobot",
                executable="packing_demo_node",
                name="packing_demo_node",
                output="screen",
                parameters=[
                    {"config": default_config},
                    {"frame_id": "base_link"},
                    {"publish_rate_hz": 1.0},
                ],
            ),
        ]
    )
