import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('r2_odin_pose_pid'),
        'config',
        'odin_pose_pid.yaml'
    )

    config_yaml_arg = DeclareLaunchArgument(
        'config_yaml',
        default_value=default_config,
        description='Path to odin_pose_pid.yaml'
    )

    server_node = Node(
        package='r2_odin_pose_pid',
        executable='odin_pose_pid_server',
        name='odin_pose_pid_server',
        output='screen',
        parameters=[
            {
                'config_yaml': LaunchConfiguration('config_yaml')
            }
        ]
    )

    return LaunchDescription([
        config_yaml_arg,
        server_node
    ])