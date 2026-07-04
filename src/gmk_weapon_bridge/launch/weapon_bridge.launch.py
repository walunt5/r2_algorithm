from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("gmk_weapon_bridge")
    params = os.path.join(package_share, "config", "weapon_bridge.yaml")

    return LaunchDescription([
        Node(
            package="gmk_weapon_bridge",
            executable="bridge_node",
            name="weapon_bridge",
            output="screen",
            parameters=[params],
        ),
    ])
