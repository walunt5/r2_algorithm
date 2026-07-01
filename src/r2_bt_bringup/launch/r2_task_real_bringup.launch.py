import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_share = get_package_share_directory("r2_nav_bringup")
    arm_share = get_package_share_directory("techx_r2_arm_control")
    chassis_share = get_package_share_directory("techx_r2_chassis_control")
    odin_pose_pid_share = get_package_share_directory("r2_odin_pose_pid")
    vision_servo_share = get_package_share_directory("r2_vision_servo")

    launch_odin_pose_pid_arg = DeclareLaunchArgument(
        "launch_odin_pose_pid",
        default_value="false",
        description="Launch real odin pose pid align action server",
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
        ),
        launch_arguments={
            # 实机总启动里，旧的 cmd_vel_to_serial_node 不启动
            # 底盘通信交给 techx_r2_chassis_control/chassis_serial_node
            "launch_serial": "false",
        }.items(),
    )

    arm_serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                arm_share,
                "launch",
                "arm_serial.launch.py",
            )
        )
    )

    chassis_serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                chassis_share,
                "launch",
                "chassis_serial.launch.py",
            )
        )
    )

    odin_pose_pid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                odin_pose_pid_share,
                "launch",
                "odin_pose_pid_server.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("launch_odin_pose_pid")),
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
        launch_odin_pose_pid_arg,
        launch_vision_servo_arg,
        vision_servo_enable_cmd_vel_arg,
        vision_servo_config_file_arg,
        nav_launch,
        arm_serial_launch,
        chassis_serial_launch,
        odin_pose_pid_launch,
        vision_servo_launch,
    ])
