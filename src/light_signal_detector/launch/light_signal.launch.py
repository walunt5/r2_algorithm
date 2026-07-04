from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("light_signal_detector")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "light_signal.yaml"]
                ),
            ),
            DeclareLaunchArgument("color_topic", default_value="/camera/color/image_raw"),
            Node(
                package="light_signal_detector",
                executable="light_signal_detector_node.py",
                name="light_signal_detector",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {"color_topic": LaunchConfiguration("color_topic")},
                ],
            ),
            Node(
                package="light_signal_detector",
                executable="light_signal_wait_action_server.py",
                name="light_signal_wait_action_server",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                ],
            ),
        ]
    )
