from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('arm_action_name', default_value='/r2_arm/execute_action'),
        DeclareLaunchArgument('lift_action_name', default_value='/r2_chassis/lift_control'),
        DeclareLaunchArgument('nav_action_name', default_value='/r2_navigate_to_pose'),
        DeclareLaunchArgument('goals_file', default_value=''),
        DeclareLaunchArgument('publish_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('default_vx', default_value='0.50'),
        DeclareLaunchArgument('default_vy', default_value='0.50'),
        DeclareLaunchArgument('default_wz', default_value='1.00'),
        DeclareLaunchArgument('max_vx', default_value='2.50'),
        DeclareLaunchArgument('max_vy', default_value='2.50'),
        DeclareLaunchArgument('max_wz', default_value='1.20'),
        DeclareLaunchArgument('speed_step', default_value='0.01'),
        DeclareLaunchArgument('arm_pose_timeout_ms', default_value='8000'),
        DeclareLaunchArgument('gripper_timeout_ms', default_value='3000'),
        DeclareLaunchArgument('lift_timeout_sec', default_value='15.0'),
        DeclareLaunchArgument('nav_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('server_check_period_ms', default_value='1000'),
    ]

    node = Node(
        package='r2_operator_console',
        executable='operator_console',
        name='r2_operator_console',
        output='screen',
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'arm_action_name': LaunchConfiguration('arm_action_name'),
            'lift_action_name': LaunchConfiguration('lift_action_name'),
            'nav_action_name': LaunchConfiguration('nav_action_name'),
            'goals_file': LaunchConfiguration('goals_file'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            'default_vx': LaunchConfiguration('default_vx'),
            'default_vy': LaunchConfiguration('default_vy'),
            'default_wz': LaunchConfiguration('default_wz'),
            'max_vx': LaunchConfiguration('max_vx'),
            'max_vy': LaunchConfiguration('max_vy'),
            'max_wz': LaunchConfiguration('max_wz'),
            'speed_step': LaunchConfiguration('speed_step'),
            'arm_pose_timeout_ms': LaunchConfiguration('arm_pose_timeout_ms'),
            'gripper_timeout_ms': LaunchConfiguration('gripper_timeout_ms'),
            'lift_timeout_sec': LaunchConfiguration('lift_timeout_sec'),
            'nav_timeout_sec': LaunchConfiguration('nav_timeout_sec'),
            'server_check_period_ms': LaunchConfiguration('server_check_period_ms'),
        }],
    )

    return LaunchDescription(args + [node])
