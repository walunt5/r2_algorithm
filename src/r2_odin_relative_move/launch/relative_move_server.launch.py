import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("r2_odin_relative_move")
    default_config = os.path.join(share, "config", "relative_move.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Relative move controller YAML file",
            ),
            Node(
                package="r2_odin_relative_move",
                executable="odin_relative_move_server",
                name="odin_relative_move_server",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
