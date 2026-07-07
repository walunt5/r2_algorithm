import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("r2_odin_relative_rotate")
    default_config = os.path.join(share, "config", "relative_rotate.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "relative_rotate_config",
                default_value=default_config,
                description="Relative rotate controller YAML file",
            ),
            Node(
                package="r2_odin_relative_rotate",
                executable="odin_relative_rotate_server",
                name="odin_relative_rotate_server",
                output="screen",
                parameters=[LaunchConfiguration("relative_rotate_config")],
            ),
        ]
    )
