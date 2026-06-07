from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的share目录路径
    pkg_share = get_package_share_directory('serial_communication_pkg')
    
    return LaunchDescription([
        Node(
            package='serial_communication_pkg',
            executable='cmd_vel_to_serial_node',
            name='cmd_vel_to_serial_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'serial_params.yaml')]
        ),
    ])