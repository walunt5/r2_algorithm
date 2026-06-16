#!/usr/bin/env python3
# ==============================================================================
#  vision_bridge.launch.py
#  用法：ros2 launch techx_vision_bridge vision_bridge.launch.py
#  参数自动从 config/vision_bridge.yaml 加载
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """加载 YAML 配置文件并启动 vision_bridge_node"""

    # 获取 share 目录下的配置文件路径
    pkg_share = get_package_share_directory("techx_vision_bridge")
    config_path = os.path.join(pkg_share, "config", "vision_bridge.yaml")

    node = Node(
        package="techx_vision_bridge",
        executable="vision_bridge_node",
        name="vision_bridge_node",
        output="screen",
        parameters=[config_path],
    )

    return LaunchDescription([node])