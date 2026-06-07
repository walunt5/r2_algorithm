import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    jie_octomap_share = get_package_share_directory("jie_octomap")
    serial_comm_share = get_package_share_directory("serial_communication_pkg")

    web_root = os.path.join(jie_octomap_share, "web")
    web_server_script = os.path.abspath(
        os.path.join(
            jie_octomap_share,
            "..",
            "..",
            "lib",
            "jie_octomap",
            "no_cache_http_server.py",
        )
    )

    web_http_port_arg = DeclareLaunchArgument(
        "web_http_port",
        default_value="8080",
        description="HTTP port for the web viewer",
    )

    controller_node = Node(
        package="octo_planner",
        executable="d1_controller",
        name="d1_controller",
        output="screen",
        parameters=[
            {
                "path_topic": "/planned_path",
                "start_navigation_topic": "/start_navigation",
                "stop_navigation_topic": "/stop_navigation",
                "require_start_command": True,

                "cmd_vel_topic": "/cmd_vel",
                "manual_cmd_vel_topic": "/web_cmd_vel",
                "tracking_point_marker_topic": "/tracking_point_marker",

                "enable_tracking_debug_view": False,
                "map_frame": "map",
                "base_frame": "base_link",
                "base_frame_candidates": "base_link,base_footprint,chassis_base_link",
            }
        ],
    )

    cmd_vel_to_serial_node = Node(
        package="serial_communication_pkg",
        executable="cmd_vel_to_serial_node",
        name="cmd_vel_to_serial_node",
        output="screen",
        parameters=[
            os.path.join(serial_comm_share, "config", "serial_params.yaml")
        ],
    )

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
    )

    web_http_server = ExecuteProcess(
        cmd=[
            web_server_script,
            "--port",
            LaunchConfiguration("web_http_port"),
            "--directory",
            web_root,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            web_http_port_arg,
            controller_node,
            cmd_vel_to_serial_node,
            rosbridge_node,
            web_http_server,
        ]
    )