from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xml_file_name_arg = DeclareLaunchArgument(
        "xml_file_name",
        default_value="test_meilin_kfs_logic.xml",
        description="Behavior tree XML file name under r2_bt_executor/config.",
    )

    xml_file_path = PathJoinSubstitution([
        FindPackageShare("r2_bt_executor"),
        "config",
        LaunchConfiguration("xml_file_name"),
    ])

    bt_executor_node = Node(
        package="r2_bt_executor",
        executable="r2_bt_executor_node",
        name="r2_bt_executor_node",
        output="screen",
        parameters=[
            {
                "xml_file_path": xml_file_path,
            }
        ],
    )

    return LaunchDescription([
        xml_file_name_arg,
        bt_executor_node,
    ])