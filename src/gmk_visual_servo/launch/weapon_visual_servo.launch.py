import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bridge_share = get_package_share_directory("gmk_weapon_bridge")
    servo_share = get_package_share_directory("gmk_visual_servo")

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "weapon_bridge.launch.py")
        )
    )
    servo_node = Node(
        package="gmk_visual_servo",
        executable="visual_servo_action_server",
        name="weapon_visual_servo",
        output="screen",
        parameters=[os.path.join(servo_share, "config", "visual_servo.yaml")],
    )
    return LaunchDescription([bridge_launch, servo_node])
