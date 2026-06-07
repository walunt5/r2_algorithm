import os
import shutil
import yaml

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def require_file(path: str, name: str) -> str:
    path = os.path.abspath(os.path.expanduser(str(path).strip()))
    if not path or not os.path.isfile(path):
        raise RuntimeError(f"{name} 文件不存在: {path}")
    return path


def require_dir(path: str, name: str) -> str:
    path = os.path.abspath(os.path.expanduser(str(path).strip()))
    if not path or not os.path.isdir(path):
        raise RuntimeError(f"{name} 目录不存在: {path}")
    return path


def update_relocalization_map_path(config_path: str, bin_file_path: str) -> None:
    with open(config_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    updated = False
    for idx, line in enumerate(lines):
        stripped = line.lstrip()
        if stripped.startswith("relocalization_map_abs_path:"):
            comment = ""
            if "#" in line:
                comment = "  #" + line.split("#", 1)[1].rstrip("\n")
            lines[idx] = f'  relocalization_map_abs_path: "{bin_file_path}"{comment}\n'
            updated = True
            break

    if not updated:
        raise RuntimeError(
            f"配置文件中未找到 relocalization_map_abs_path 项: {config_path}"
        )

    with open(config_path, "w", encoding="utf-8") as f:
        f.writelines(lines)


def sync_config_to_driver(src_config_path: str, driver_package_dir: str) -> str:
    driver_config_candidates = []

    colcon_prefix_path = os.environ.get("COLCON_PREFIX_PATH", "")
    if colcon_prefix_path:
        first_prefix = colcon_prefix_path.split(":")[0]
        install_pos = first_prefix.find("/install")
        if install_pos != -1:
            workspace_root = first_prefix[:install_pos]
            driver_config_candidates.append(
                os.path.join(
                    workspace_root,
                    "src",
                    "odin_ros_driver",
                    "config",
                    "control_command.yaml",
                )
            )

    driver_config_candidates.append(
        os.path.join(driver_package_dir, "config", "control_command.yaml")
    )

    for driver_config_path in driver_config_candidates:
        driver_config_dir = os.path.dirname(driver_config_path)
        if os.path.isdir(driver_config_dir):
            shutil.copyfile(src_config_path, driver_config_path)
            return driver_config_path

    raise RuntimeError("未找到 odin_ros_driver/config/control_command.yaml 可同步的目标路径")


def generate_launch_description():
    r2_nav_share = get_package_share_directory("r2_nav_bringup")
    jie_octomap_share = get_package_share_directory("jie_octomap")
    odin_ros_driver_share = get_package_share_directory("odin_ros_driver")
    serial_comm_share = get_package_share_directory("serial_communication_pkg")

    config_path = os.path.join(r2_nav_share, "config", "r2_nav_params.yaml")
    config = load_yaml(config_path)

    maps = config.get("maps", {})
    ui = config.get("ui", {})
    frames = config.get("frames", {})
    static_tf = config.get("static_tf", {}).get("odin_to_chassis", {})
    planner = config.get("planner", {})
    d1_controller_params = dict(
        config.get("d1_controller", {}).get("ros__parameters", {})
    )

    action_server_params = dict(
        config.get("action_server", {}).get("ros__parameters", {})
    )

    relocalization_bin_file = require_file(
        maps.get("relocalization_bin_file", ""),
        "Odin1 重定位 .bin 地图",
    )
    relocalization_pcd_file = require_file(
        maps.get("relocalization_pcd_file", ""),
        "PCD 可视化地图",
    )
    map_package_dir = require_dir(
        maps.get("map_package_dir", ""),
        "OctoMap 地图包",
    )

    goals_file = os.path.join(
        r2_nav_share,
        "config",
        "r2_nav_goals.yaml",
    )

    show_rviz_default = "true" if bool(ui.get("show_rviz", False)) else "false"
    show_map_gui_default = "true" if bool(ui.get("show_map_gui", False)) else "false"
    launch_web_default = "true" if bool(ui.get("launch_web", True)) else "false"
    launch_rosbridge_default = "true" if bool(ui.get("launch_rosbridge", True)) else "false"
    web_http_port_default = str(ui.get("web_http_port", "8080"))

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value=show_rviz_default,
        description="Launch RViz",
    )
    launch_map_gui_arg = DeclareLaunchArgument(
        "launch_map_gui",
        default_value=show_map_gui_default,
        description="Launch map GUI",
    )
    launch_planner_arg = DeclareLaunchArgument(
        "launch_planner",
        default_value="true",
        description="Launch jie_path_node",
    )
    launch_controller_arg = DeclareLaunchArgument(
        "launch_controller",
        default_value="true",
        description="Launch d1_controller",
    )
    launch_serial_arg = DeclareLaunchArgument(
        "launch_serial",
        default_value="true",
        description="Launch serial communication node",
    )
    launch_web_arg = DeclareLaunchArgument(
        "launch_web",
        default_value=launch_web_default,
        description="Launch web viewer",
    )
    launch_rosbridge_arg = DeclareLaunchArgument(
        "launch_rosbridge",
        default_value=launch_rosbridge_default,
        description="Launch rosbridge websocket",
    )
    web_http_port_arg = DeclareLaunchArgument(
        "web_http_port",
        default_value=web_http_port_default,
        description="HTTP port for web viewer",
    )

    try:
        odin_costmap_share = get_package_share_directory("odin_costmap")
        loc_control_config_path = os.path.join(
            odin_costmap_share,
            "config",
            "loc_control_command.yaml",
        )
    except PackageNotFoundError:
        loc_control_config_path = os.path.join(
            jie_octomap_share,
            "config",
            "loc_control_command.yaml",
        )

    update_relocalization_map_path(
        loc_control_config_path,
        relocalization_bin_file,
    )
    driver_control_config_path = sync_config_to_driver(
        loc_control_config_path,
        odin_ros_driver_share,
    )

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

    odin1_loc_rviz_config_file = os.path.join(
        jie_octomap_share,
        "rviz",
        "odin1_loc.rviz",
    )

    host_sdk_node = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        parameters=[{"config_file": driver_control_config_path}],
    )

    with open(driver_control_config_path, "r", encoding="utf-8") as f:
        pcd2depth_params = yaml.safe_load(f) or {}
    pcd2depth_params["calib_file_path"] = os.path.join(
        odin_ros_driver_share,
        "config",
        "calib.yaml",
    )

    pcd2depth_node = Node(
        package="odin_ros_driver",
        executable="pcd2depth_ros2_node",
        name="pcd2depth_ros2_node",
        output="screen",
        parameters=[pcd2depth_params],
    )

    with open(driver_control_config_path, "r", encoding="utf-8") as f:
        reprojection_params = yaml.safe_load(f) or {}
    reprojection_params["calib_file_path"] = os.path.join(
        odin_ros_driver_share,
        "config",
        "calib.yaml",
    )

    cloud_reprojection_node = Node(
        package="odin_ros_driver",
        executable="cloud_reprojection_ros2_node",
        name="cloud_reprojection_ros2_node",
        output="screen",
        parameters=[reprojection_params],
    )

    with open(driver_control_config_path, "r", encoding="utf-8") as f:
        overlay_params = yaml.safe_load(f) or {}
    overlay_params["calib_file_path"] = os.path.join(
        odin_ros_driver_share,
        "config",
        "calib.yaml",
    )

    image_overlay_node = Node(
        package="odin_ros_driver",
        executable="image_overlay_node",
        name="image_overlay_node",
        output="screen",
        parameters=[overlay_params],
    )

    static_odin_to_chassis_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odin1_to_chassis_base_link",
        output="screen",
        arguments=[
            str(static_tf.get("x", 0.0)),
            str(static_tf.get("y", 0.0)),
            str(static_tf.get("z", 0.0)),
            str(static_tf.get("yaw", 0.0)),
            str(static_tf.get("pitch", 0.0)),
            str(static_tf.get("roll", 0.0)),
            "odin1_base_link",
            frames.get("base_frame", "chassis_base_link"),
        ],
    )

    pcd_publisher_node = Node(
        package="jie_octomap",
        executable="pcd_file_publisher.py",
        name="pcd_file_publisher",
        output="screen",
        parameters=[
            {
                "pcd_path": relocalization_pcd_file,
                "topic": "/pcd_points",
                "frame_id": frames.get("map_frame", "map"),
                "publish_hz": 1.0,
            }
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", odin1_loc_rviz_config_file],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    map_package_manager_node = Node(
        package="jie_octomap",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
        parameters=[
            {
                "autoload_package_path": map_package_dir,
            }
        ],
    )

    occupied_marker_node = Node(
        package="jie_octomap",
        executable="octomap_to_occupied_markers_node",
        name="octomap_to_occupied_markers",
        output="screen",
        parameters=[
            {
                "octomap_topic": "/octomap",
                "marker_topic": "/octomap_occupied_markers",
                "frame_id": frames.get("map_frame", "map"),
            }
        ],
    )

    planner_node = Node(
        package="octo_planner",
        executable="jie_path_node",
        name="jie_path_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_planner")),
        parameters=[
            {
                "octomap_topic": "/octomap",
                "start_topic": "/start_point",
                "goal_topic": "/goal_point",
                "goal_pose_topic": "/goal_pose",
                "path_topic": "/planned_path",
                "path_marker_topic": "/planned_path_marker",
                "preblocked_marker_topic": "/preblocked_cells_markers",
                "edited_occupied_marker_topic": "/edited_occupied_markers",
                "traversable_marker_topic": "/traversable_cells_markers",
                "risk_cost_topic": "/risk_cost_cells",
                "frame_id": frames.get("map_frame", "map"),
                "map_id": "r2_loaded_map",
                "source_world_file": "",
                "robot_radius": float(planner.get("robot_radius", 0.25)),
                "max_iterations": int(planner.get("max_iterations", 500000)),
                "snap_search_radius_cells": int(planner.get("snap_search_radius_cells", 12)),
                "require_ground_support": bool(planner.get("require_ground_support", True)),
                "strict_direct_ground_support": bool(planner.get("strict_direct_ground_support", False)),
                "ground_support_xy_radius_cells": int(planner.get("ground_support_xy_radius_cells", 1)),
                "ground_support_depth_cells": int(planner.get("ground_support_depth_cells", 1)),
                "enable_preblocked_costmap": bool(planner.get("enable_preblocked_costmap", True)),
                "preblocked_costmap_radius_cells": int(planner.get("preblocked_costmap_radius_cells", 3)),
                "preblocked_costmap_weight": float(planner.get("preblocked_costmap_weight", 2.5)),
            }
        ],
    )

    controller_node = Node(
        package="octo_planner",
        executable="d1_controller",
        name="d1_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_controller")),
        parameters=[
            d1_controller_params,
            {
                "path_topic": "/planned_path",
                "start_navigation_topic": "/start_navigation",
                "stop_navigation_topic": "/stop_navigation",
                "require_start_command": True,
                "cmd_vel_topic": "/cmd_vel",
                "manual_cmd_vel_topic": "/web_cmd_vel",
                "tracking_point_marker_topic": "/tracking_point_marker",
                "enable_tracking_debug_view": ParameterValue(
                    LaunchConfiguration("launch_map_gui"),
                    value_type=bool,
                ),
                "map_frame": frames.get("map_frame", "map"),
                "base_frame": frames.get("base_frame", "chassis_base_link"),
                "base_frame_candidates": frames.get(
                    "base_frame_candidates",
                    "chassis_base_link,odin1_base_link,base_link,base_footprint",
                ),
            }
        ],
    )

    cmd_vel_to_serial_node = Node(
        package="serial_communication_pkg",
        executable="cmd_vel_to_serial_node",
        name="cmd_vel_to_serial_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_serial")),
        parameters=[
            os.path.join(serial_comm_share, "config", "serial_params.yaml")
        ],
    )

    map_viewer_gui_node = Node(
        package="jie_octomap",
        executable="map_viewer_gui",
        name="map_viewer_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_map_gui")),
        additional_env={"MAP_VIEWER_DEFAULT_PACKAGE": map_package_dir},
        parameters=[
            {
                "tf_parent_frame": frames.get("map_frame", "map"),
                "tf_child_frame": frames.get("base_frame", "chassis_base_link"),
            }
        ],
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
        condition=IfCondition(LaunchConfiguration("launch_web")),
    )

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_rosbridge")),
    )

    r2_nav_action_server_node = Node(
        package="r2_nav_action_server",
        executable="r2_nav_action_server_node",
        name="r2_nav_action_server_node",
        output="screen",
        parameters=[
            action_server_params,
            {
                "goals_file": goals_file,
                "map_frame": frames.get("map_frame", "map"),
                "base_frame": frames.get("base_frame", "chassis_base_link"),
                "goal_position_tolerance": float(
                    d1_controller_params.get("goal_position_tolerance", 0.10)
                ),
                "goal_yaw_tolerance": float(
                    d1_controller_params.get("goal_yaw_tolerance", 0.20)
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            launch_rviz_arg,
            launch_map_gui_arg,
            launch_planner_arg,
            launch_controller_arg,
            launch_serial_arg,
            launch_web_arg,
            launch_rosbridge_arg,
            web_http_port_arg,
            host_sdk_node,
            pcd2depth_node,
            cloud_reprojection_node,
            image_overlay_node,
            static_odin_to_chassis_node,
            pcd_publisher_node,
            rviz_node,
            map_package_manager_node,
            occupied_marker_node,
            planner_node,
            controller_node,
            cmd_vel_to_serial_node,
            map_viewer_gui_node,
            web_http_server,
            rosbridge_node,
            r2_nav_action_server_node,
        ]
    )