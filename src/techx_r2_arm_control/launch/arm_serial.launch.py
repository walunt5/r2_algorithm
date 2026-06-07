import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_name = "techx_r2_arm_control"

    default_params_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "arm_serial.yaml",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to arm serial node parameter yaml file.",
    )

    arm_serial_node = Node(
        package=package_name,
        executable="arm_serial_node",
        name="arm_serial_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        arm_serial_node,
    ])