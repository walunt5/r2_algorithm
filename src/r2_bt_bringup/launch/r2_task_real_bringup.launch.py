import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav_share = get_package_share_directory("r2_nav_bringup")
    arm_share = get_package_share_directory("techx_r2_arm_control")
    chassis_share = get_package_share_directory("techx_r2_chassis_control")

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

    return LaunchDescription([
        nav_launch,
        arm_serial_launch,
        chassis_serial_launch,
    ])