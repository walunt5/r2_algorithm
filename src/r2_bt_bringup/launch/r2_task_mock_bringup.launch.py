import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_share = get_package_share_directory("r2_nav_bringup")
    arm_share = get_package_share_directory("techx_r2_arm_control")
    chassis_share = get_package_share_directory("techx_r2_chassis_control")
    vision_servo_share = get_package_share_directory("r2_vision_servo")

    arm_result_mode_arg = DeclareLaunchArgument(
        "arm_result_mode",
        default_value="success",
        description="Mock arm result mode: success, error, no_ack, timeout.",
    )

    chassis_result_mode_arg = DeclareLaunchArgument(
        "chassis_result_mode",
        default_value="success",
        description="Mock chassis result mode: success, error, no_ack, timeout.",
    )

    chassis_drop_first_ack_arg = DeclareLaunchArgument(
        "chassis_drop_first_ack",
        default_value="false",
        description="Mock chassis drops first ACK to test resend.",
    )

    launch_vision_servo_arg = DeclareLaunchArgument(
        "launch_vision_servo",
        default_value="false",
        description="Launch vision servo action server.",
    )

    vision_servo_enable_cmd_vel_arg = DeclareLaunchArgument(
        "vision_servo_enable_cmd_vel",
        default_value="false",
        description="Allow vision servo to publish non-zero /cmd_vel.",
    )

    vision_servo_config_file_arg = DeclareLaunchArgument(
        "vision_servo_config_file",
        default_value=os.path.join(vision_servo_share, "config", "vision_servo.yaml"),
        description="Path to r2_vision_servo parameter YAML.",
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav_share,
                "launch",
                "r2_odin_web_nav.launch.py",
            )
        )
    )

    arm_mock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                arm_share,
                "launch",
                "mock_take_head.launch.py",
            )
        ),
        launch_arguments={
            "result_mode": LaunchConfiguration("arm_result_mode"),
        }.items(),
    )

    chassis_mock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                chassis_share,
                "launch",
                "mock_chassis.launch.py",
            )
        ),
        launch_arguments={
            "result_mode": LaunchConfiguration("chassis_result_mode"),
            "drop_first_ack": LaunchConfiguration("chassis_drop_first_ack"),
        }.items(),
    )

    vision_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                vision_servo_share,
                "launch",
                "vision_servo.launch.py",
            )
        ),
        launch_arguments={
            "config_file": LaunchConfiguration("vision_servo_config_file"),
            "enable_cmd_vel": LaunchConfiguration("vision_servo_enable_cmd_vel"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_vision_servo")),
    )

    return LaunchDescription([
        arm_result_mode_arg,
        chassis_result_mode_arg,
        chassis_drop_first_ack_arg,
        launch_vision_servo_arg,
        vision_servo_enable_cmd_vel_arg,
        vision_servo_config_file_arg,
        nav_launch,
        arm_mock_launch,
        chassis_mock_launch,
        vision_servo_launch,
    ])
