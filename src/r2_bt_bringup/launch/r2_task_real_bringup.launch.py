import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 各功能包安装后的 share 目录
    nav_share = get_package_share_directory("r2_nav_bringup")
    arm_share = get_package_share_directory("techx_r2_arm_control")
    chassis_share = get_package_share_directory("techx_r2_chassis_control")
    odin_pose_pid_share = get_package_share_directory("r2_odin_pose_pid")

    visual_servo_share = get_package_share_directory("gmk_visual_servo")
    light_signal_share = get_package_share_directory("light_signal_detector")

    # =========================
    # 启动参数
    # =========================

    launch_odin_pose_pid_arg = DeclareLaunchArgument(
        "launch_odin_pose_pid",
        default_value="false",
        description="Launch real odin pose pid align action server",
    )

    launch_weapon_visual_servo_arg = DeclareLaunchArgument(
        "launch_weapon_visual_servo",
        default_value="true",
        description="Launch weapon visual servo and UDP bridge",
    )

    launch_light_signal_arg = DeclareLaunchArgument(
        "launch_light_signal",
        default_value="true",
        description="Launch camera, light detector and light wait action server",
    )

    # =========================
    # 导航系统
    # =========================

    nav_launch = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        nav_share,
                        "launch",
                        "r2_odin_web_nav.launch.py",
                    )
                ),
                launch_arguments={
                    # 旧串口节点不启动
                    # 底盘通信交给 chassis_serial_node
                    "launch_serial": "false",
                }.items(),
            )
        ],
    )

    # =========================
    # 机械臂串口
    # =========================

    arm_serial_launch = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        arm_share,
                        "launch",
                        "arm_serial.launch.py",
                    )
                )
            )
        ],
    )

    # =========================
    # 底盘串口
    # =========================

    chassis_serial_launch = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        chassis_share,
                        "launch",
                        "chassis_serial.launch.py",
                    )
                )
            )
        ],
    )

    # =========================
    # Odin PID 精调
    # =========================

    odin_pose_pid_launch = GroupAction(
        scoped=True,
        condition=IfCondition(
            LaunchConfiguration("launch_odin_pose_pid")
        ),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        odin_pose_pid_share,
                        "launch",
                        "odin_pose_pid_server.launch.py",
                    )
                )
            )
        ],
    )

    # =========================
    # 武器头视觉伺服
    # =========================

    weapon_visual_servo_launch = GroupAction(
        scoped=True,
        condition=IfCondition(
            LaunchConfiguration("launch_weapon_visual_servo")
        ),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        visual_servo_share,
                        "launch",
                        "weapon_visual_servo.launch.py",
                    )
                )
            )
        ],
    )

    # =========================
    # 灯光信号检测
    # =========================

    light_signal_launch = GroupAction(
        scoped=True,
        condition=IfCondition(
            LaunchConfiguration("launch_light_signal")
        ),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        light_signal_share,
                        "launch",
                        "light_signal.launch.py",
                    )
                )
            )
        ],
    )

    return LaunchDescription(
        [
            # 参数必须先声明
            launch_odin_pose_pid_arg,
            launch_weapon_visual_servo_arg,
            launch_light_signal_arg,

            # 子系统
            nav_launch,
            arm_serial_launch,
            chassis_serial_launch,
            odin_pose_pid_launch,
            weapon_visual_servo_launch,
            light_signal_launch,
        ]
    )