from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    light_signal_share = FindPackageShare("light_signal_detector")
    orbbec_camera_share = FindPackageShare("orbbec_camera")

    # 启动 Gemini 335 相机
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    orbbec_camera_share,
                    "launch",
                    "gemini_330_series.launch.py",
                ]
            )
        ),
        launch_arguments={
            # 相机命名空间，最终彩色图像通常为：
            # /camera/color/image_raw
            "camera_name": "camera",

            # 灯光识别需要彩色图像
            "enable_color": "true",

            # 仅做灯光识别时可以关闭深度，降低 USB 和 CPU 负载
            "enable_depth": "false",

            # 不需要点云
            "enable_point_cloud": "false",
            "enable_colored_point_cloud": "false",

            # 可根据实际分辨率设置
            # 设置为 0 表示使用驱动默认配置
            "color_width": "640",
            "color_height": "480",
            "color_fps": "30",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [
                        light_signal_share,
                        "config",
                        "light_signal.yaml",
                    ]
                ),
            ),

            DeclareLaunchArgument(
                "color_topic",
                default_value="/camera/color/image_raw",
            ),

            # 先加入相机子 launch
            camera_launch,

            # 橙色灯光检测节点
            Node(
                package="light_signal_detector",
                executable="light_signal_detector_node.py",
                name="light_signal_detector",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "color_topic": LaunchConfiguration("color_topic"),
                    },
                ],
            ),

            # 等待灯光信号的 Action Server
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