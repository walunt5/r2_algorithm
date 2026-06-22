from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("enable_cmd_vel", default_value="false"),
        DeclareLaunchArgument("action_name", default_value="/r2_chassis/vision_servo"),
        DeclareLaunchArgument("request_topic", default_value="/techx/vision/request"),
        DeclareLaunchArgument("selected_topic", default_value="/techx/vision/selected"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("global_frame", default_value="map"),
        DeclareLaunchArgument("robot_frame", default_value="chassis_base_link"),
        DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("tf_timeout_sec", default_value="0.10"),
        DeclareLaunchArgument("max_vx", default_value="0.06"),
        DeclareLaunchArgument("max_vy", default_value="0.06"),
        DeclareLaunchArgument("max_wz", default_value="0.20"),
    ]

    node = Node(
        package="r2_vision_servo",
        executable="vision_servo_action_server",
        name="vision_servo_action_server",
        output="screen",
        parameters=[
            {
                "enable_cmd_vel": ParameterValue(
                    LaunchConfiguration("enable_cmd_vel"),
                    value_type=bool,
                ),
                "action_name": LaunchConfiguration("action_name"),
                "request_topic": LaunchConfiguration("request_topic"),
                "selected_topic": LaunchConfiguration("selected_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "global_frame": LaunchConfiguration("global_frame"),
                "robot_frame": LaunchConfiguration("robot_frame"),
                "control_rate_hz": ParameterValue(
                    LaunchConfiguration("control_rate_hz"),
                    value_type=float,
                ),
                "tf_timeout_sec": ParameterValue(
                    LaunchConfiguration("tf_timeout_sec"),
                    value_type=float,
                ),
                "max_vx": ParameterValue(LaunchConfiguration("max_vx"), value_type=float),
                "max_vy": ParameterValue(LaunchConfiguration("max_vy"), value_type=float),
                "max_wz": ParameterValue(LaunchConfiguration("max_wz"), value_type=float),
            }
        ],
    )

    return LaunchDescription(args + [node])
