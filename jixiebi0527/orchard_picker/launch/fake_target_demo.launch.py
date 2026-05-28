import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("orchard_picker")
    config_default = os.path.join(package_share, "config", "picker_params.yaml")
    config = LaunchConfiguration("config")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=config_default),
            DeclareLaunchArgument("x", default_value="0.200269"),
            DeclareLaunchArgument("y", default_value="-0.219514"),
            DeclareLaunchArgument("z", default_value="0.597973"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_share, "launch", "orchard_picker.launch.py")
                ),
                launch_arguments={"config": config}.items(),
            ),
            Node(
                package="orchard_picker",
                executable="fake_apple_target.py",
                name="fake_apple_target",
                output="screen",
                emulate_tty=True,
                parameters=[
                    config,
                    {
                        "x": ParameterValue(x, value_type=float),
                        "y": ParameterValue(y, value_type=float),
                        "z": ParameterValue(z, value_type=float),
                    },
                ],
            ),
        ]
    )
