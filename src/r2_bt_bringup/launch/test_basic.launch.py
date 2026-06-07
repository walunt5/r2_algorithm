import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    executor_share_dir = get_package_share_directory('r2_bt_executor')

    xml_file_path = os.path.join(
        executor_share_dir,
        'config',
        'test_basic.xml'
    )

    bt_executor_node = Node(
        package='r2_bt_executor',
        executable='r2_bt_executor_node',
        name='r2_bt_executor_node',
        output='screen',
        parameters=[
            {
                'xml_file_path': xml_file_path
            }
        ]
    )

    return LaunchDescription([
        bt_executor_node
    ])