import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


STRING_OVERRIDES = [
    "action_name",
    "request_topic",
    "selected_topic",
    "cmd_vel_topic",
    "global_frame",
    "robot_frame",
]

FLOAT_OVERRIDES = [
    "control_rate_hz",
    "tf_timeout_sec",
    "request_republish_sec",
    "kp_x",
    "ki_x",
    "kd_x",
    "kp_y",
    "ki_y",
    "kd_y",
    "kp_yaw",
    "ki_yaw",
    "kd_yaw",
    "i_limit",
    "max_vx",
    "max_vy",
    "max_wz",
]


def _optional_launch_arg(context, name):
    value = LaunchConfiguration(name).perform(context).strip()
    return value if value else None


def _parse_bool(value):
    normalized = value.lower()
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"invalid bool value: {value}")


def _launch_setup(context, *args, **kwargs):
    overrides = {}

    for name in STRING_OVERRIDES:
        value = _optional_launch_arg(context, name)
        if value is not None:
            overrides[name] = value

    for name in FLOAT_OVERRIDES:
        value = _optional_launch_arg(context, name)
        if value is not None:
            overrides[name] = float(value)

    enable_cmd_vel = _optional_launch_arg(context, "enable_cmd_vel")
    if enable_cmd_vel is not None:
        overrides["enable_cmd_vel"] = _parse_bool(enable_cmd_vel)

    parameters = [LaunchConfiguration("config_file")]
    if overrides:
        parameters.append(overrides)

    node = Node(
        package="r2_vision_servo",
        executable="vision_servo_action_server",
        name="vision_servo_action_server",
        output="screen",
        parameters=parameters,
    )

    return [node]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("r2_vision_servo"),
        "config",
        "vision_servo.yaml",
    )

    args = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to r2_vision_servo parameter YAML.",
        ),
        DeclareLaunchArgument("enable_cmd_vel", default_value=""),
        DeclareLaunchArgument("action_name", default_value=""),
        DeclareLaunchArgument("request_topic", default_value=""),
        DeclareLaunchArgument("selected_topic", default_value=""),
        DeclareLaunchArgument("cmd_vel_topic", default_value=""),
        DeclareLaunchArgument("global_frame", default_value=""),
        DeclareLaunchArgument("robot_frame", default_value=""),
        DeclareLaunchArgument("control_rate_hz", default_value=""),
        DeclareLaunchArgument("tf_timeout_sec", default_value=""),
        DeclareLaunchArgument("request_republish_sec", default_value=""),
        DeclareLaunchArgument("kp_x", default_value=""),
        DeclareLaunchArgument("ki_x", default_value=""),
        DeclareLaunchArgument("kd_x", default_value=""),
        DeclareLaunchArgument("kp_y", default_value=""),
        DeclareLaunchArgument("ki_y", default_value=""),
        DeclareLaunchArgument("kd_y", default_value=""),
        DeclareLaunchArgument("kp_yaw", default_value=""),
        DeclareLaunchArgument("ki_yaw", default_value=""),
        DeclareLaunchArgument("kd_yaw", default_value=""),
        DeclareLaunchArgument("i_limit", default_value=""),
        DeclareLaunchArgument("max_vx", default_value=""),
        DeclareLaunchArgument("max_vy", default_value=""),
        DeclareLaunchArgument("max_wz", default_value=""),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
