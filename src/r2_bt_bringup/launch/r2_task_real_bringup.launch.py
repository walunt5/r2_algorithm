import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def include_launch(
    package_name: str,
    *relative_path: str,
    launch_arguments=None,
):
    package_share = get_package_share_directory(
        package_name
    )

    launch_file = os.path.join(
        package_share,
        *relative_path,
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file
        ),
        launch_arguments=(
            launch_arguments.items()
            if launch_arguments
            else None
        ),
    )


def generate_launch_description():
    # 普通情况下使用 relative_move.yaml。
    # UI 启动三区配置时传入：
    # relative_move_config_name:=relative_move_zone3.yaml
    relative_move_config_name_arg = (
        DeclareLaunchArgument(
            "relative_move_config_name",
            default_value="relative_move.yaml",
            description=(
                "YAML filename used by the Odin "
                "relative move server"
            ),
        )
    )

    relative_move_config_path = (
        PathJoinSubstitution(
            [
                FindPackageShare(
                    "r2_odin_relative_move"
                ),
                "config",
                LaunchConfiguration(
                    "relative_move_config_name"
                ),
            ]
        )
    )

    odin_driver_launch = include_launch(
        "odin_ros_driver",
        "launch",
        "odin1_ros2.launch.py",
    )

    weapon_visual_servo_launch = include_launch(
        "gmk_visual_servo",
        "launch",
        "weapon_visual_servo.launch.py",
    )

    light_signal_launch = include_launch(
        "light_signal_detector",
        "launch",
        "light_signal.launch.py",
    )

    arm_serial_launch = include_launch(
        "techx_r2_arm_control",
        "launch",
        "arm_serial.launch.py",
    )

    chassis_serial_launch = include_launch(
        "techx_r2_chassis_control",
        "launch",
        "chassis_serial.launch.py",
    )

    relative_move_launch = include_launch(
        "r2_odin_relative_move",
        "launch",
        "relative_move_server.launch.py",
        launch_arguments={
            "config": relative_move_config_path,
        },
    )

    return LaunchDescription(
        [
            relative_move_config_name_arg,
            odin_driver_launch,
            weapon_visual_servo_launch,
            light_signal_launch,
            arm_serial_launch,
            chassis_serial_launch,
            relative_move_launch,
        ]
    )