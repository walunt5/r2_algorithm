import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Launch navigation, real serial drivers, and the goal recorder GUI."""
    r2_nav_share = get_package_share_directory("r2_nav_bringup")
    arm_share = get_package_share_directory("techx_r2_arm_control")
    chassis_share = get_package_share_directory("techx_r2_chassis_control")

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                r2_nav_share,
                "launch",
                "r2_odin_web_nav.launch.py",
            )
        ),
        launch_arguments={
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

    goal_recorder_gui = Node(
        package="r2_nav_bringup",
        executable="r2_goal_recorder_gui",
        name="r2_goal_recorder_gui_node",
        output="screen",
    )

    return LaunchDescription([
        nav_launch,
        arm_serial_launch,
        chassis_serial_launch,
        goal_recorder_gui,
    ])
