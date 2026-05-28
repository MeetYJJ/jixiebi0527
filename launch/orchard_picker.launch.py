import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_default = os.path.join(
        get_package_share_directory("orchard_picker"),
        "config",
        "picker_params.yaml",
    )
    config = LaunchConfiguration("config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=config_default),
            Node(
                package="orchard_picker",
                executable="rm_json_bridge.py",
                name="rm_json_bridge",
                output="screen",
                emulate_tty=True,
                parameters=[config],
            ),
            Node(
                package="orchard_picker",
                executable="pick_task_manager.py",
                name="pick_task_manager",
                output="screen",
                emulate_tty=True,
                parameters=[config],
            ),
        ]
    )
