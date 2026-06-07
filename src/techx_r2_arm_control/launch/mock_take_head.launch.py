import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_name = "techx_r2_arm_control"

    default_params_file = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "mock_arm_serial.yaml",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to mock arm serial parameter yaml file.",
    )

    result_mode_arg = DeclareLaunchArgument(
        "result_mode",
        default_value="success",
        description="Mock result mode: success, error, no_ack, timeout.",
    )

    socat_process = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "rm -f /tmp/r2_arm_upper /tmp/r2_arm_lower; "
            "socat -d -d "
            "pty,raw,echo=0,link=/tmp/r2_arm_upper "
            "pty,raw,echo=0,link=/tmp/r2_arm_lower",
        ],
        output="screen",
    )

    mock_arm_stm32 = Node(
        package=package_name,
        executable="mock_arm_stm32",
        name="mock_arm_stm32",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"result_mode": LaunchConfiguration("result_mode")},
        ],
    )

    arm_serial_node = Node(
        package=package_name,
        executable="arm_serial_node",
        name="arm_serial_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    delayed_nodes = TimerAction(
        period=0.8,
        actions=[
            mock_arm_stm32,
            arm_serial_node,
        ],
    )

    return LaunchDescription([
        params_file_arg,
        result_mode_arg,
        socat_process,
        delayed_nodes,
    ])