#!/usr/bin/env python3
import math
import threading
import time
import yaml

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from r2_nav_interfaces.action import NavigateToPose
from r2_nav_interfaces.msg import NavigationStart


DEFAULT_CONTROL_MODE = "x_then_y"
VALID_CONTROL_MODES = {"fixed_map", "x_then_y"}


def resolve_control_mode(control_mode: str) -> str:
    """Normalize and validate the per-goal translation control mode."""
    mode = str(control_mode).strip()
    if not mode:
        return DEFAULT_CONTROL_MODE
    if mode not in VALID_CONTROL_MODES:
        raise ValueError(
            f"unsupported control_mode={mode!r}; expected one of "
            f"{sorted(VALID_CONTROL_MODES)}"
        )
    return mode


def yaw_to_quaternion(yaw: float):
    """只考虑平面 yaw，转换成四元数。"""
    z = math.sin(yaw * 0.5)
    w = math.cos(yaw * 0.5)
    return 0.0, 0.0, z, w


def quaternion_to_yaw(q) -> float:
    """从四元数中提取 yaw。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """把角度归一化到 -pi 到 pi。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class R2NavActionServer(Node):
    def __init__(self):
        super().__init__("r2_nav_action_server_node")

        self.callback_group = ReentrantCallbackGroup()

        # 参数
        self.declare_parameter("action_name", "/r2_navigate_to_pose")
        self.declare_parameter("goals_file", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "chassis_base_link")

        self.declare_parameter("start_point_topic", "/start_point")
        self.declare_parameter("goal_point_topic", "/goal_point")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter(
            "navigation_start_request_topic", "/navigation_start_request"
        )
        self.declare_parameter("stop_navigation_topic", "/stop_navigation")
        self.declare_parameter("planned_path_topic", "/planned_path")

        self.declare_parameter("path_wait_timeout_sec", 5.0)
        self.declare_parameter("path_source_mode", "planner")
        self.declare_parameter("goal_position_tolerance", 0.10)
        self.declare_parameter("goal_yaw_tolerance", 0.20)
        self.declare_parameter("feedback_rate_hz", 10.0)
        self.declare_parameter("tf_wait_timeout_sec", 5.0)

        self.action_name = self.get_parameter("action_name").value
        self.goals_file = self.get_parameter("goals_file").value
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.path_wait_timeout_sec = float(
            self.get_parameter("path_wait_timeout_sec").value
        )
        self.path_source_mode = str(
            self.get_parameter("path_source_mode").value
        ).strip()
        if self.path_source_mode not in ("planner", "direct_goal"):
            self.get_logger().warn(
                f"未知 path_source_mode={self.path_source_mode}，回退到 planner"
            )
            self.path_source_mode = "planner"
        self.goal_position_tolerance = float(
            self.get_parameter("goal_position_tolerance").value
        )
        self.goal_yaw_tolerance = float(
            self.get_parameter("goal_yaw_tolerance").value
        )
        self.feedback_rate_hz = float(
            self.get_parameter("feedback_rate_hz").value
        )
        self.tf_wait_timeout_sec = float(
            self.get_parameter("tf_wait_timeout_sec").value
        )

        # 目标点表
        self.goals = self.load_goals(self.goals_file)

        # Publisher
        planning_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.start_point_pub = self.create_publisher(
            PointStamped,
            self.get_parameter("start_point_topic").value,
            planning_qos,
        )
        self.goal_point_pub = self.create_publisher(
            PointStamped,
            self.get_parameter("goal_point_topic").value,
            planning_qos,
        )
        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("goal_pose_topic").value,
            planning_qos,
        )
        self.navigation_start_pub = self.create_publisher(
            NavigationStart,
            self.get_parameter("navigation_start_request_topic").value,
            10,
        )
        self.stop_nav_pub = self.create_publisher(
            Bool,
            self.get_parameter("stop_navigation_topic").value,
            10,
        )
        self.planned_path_pub = self.create_publisher(
            Path,
            self.get_parameter("planned_path_topic").value,
            planning_qos,
        )

        # Path 订阅
        self.path_event = threading.Event()
        self.last_path = None
        self.path_sub = self.create_subscription(
            Path,
            self.get_parameter("planned_path_topic").value,
            self.path_callback,
            10,
        )

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 防止多个导航任务同时执行
        self.active_lock = threading.Lock()
        self.active_goal = False

        self.action_server = ActionServer(
            self,
            NavigateToPose,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(f"R2 Nav Action Server 已启动: {self.action_name}")
        self.get_logger().info(
            f"map_frame={self.map_frame}, base_frame={self.base_frame}, "
            f"tf_wait_timeout_sec={self.tf_wait_timeout_sec:.1f}, "
            f"path_wait_timeout_sec={self.path_wait_timeout_sec:.1f}, "
            f"path_source_mode={self.path_source_mode}"
        )

    def load_goals(self, goals_file: str):
        if not goals_file:
            self.get_logger().warn("未设置 goals_file，后续只能使用 target_pose 导航")
            return {}

        try:
            with open(goals_file, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            goals = data.get("goals", {})
            self.get_logger().info(f"已加载预设目标点数量: {len(goals)}")
            return goals
        except Exception as e:
            self.get_logger().error(f"读取 goals_file 失败: {goals_file}, error={e}")
            return {}

    def goal_callback(self, goal_request):
        with self.active_lock:
            if self.active_goal:
                self.get_logger().warn("已有导航任务正在执行，拒绝新的导航目标")
                return GoalResponse.REJECT

        if goal_request.goal_name:
            if goal_request.goal_name not in self.goals:
                self.get_logger().error(f"未知 goal_name: {goal_request.goal_name}")
                return GoalResponse.REJECT

        try:
            control_mode = resolve_control_mode(goal_request.control_mode)
        except ValueError as e:
            self.get_logger().error(str(e))
            return GoalResponse.REJECT

        self.get_logger().info(
            f"接受导航目标: goal_name={goal_request.goal_name}, "
            f"control_mode={control_mode}, timeout={goal_request.timeout_sec}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("收到导航取消请求")
        return CancelResponse.ACCEPT

    def path_callback(self, msg: Path):
        self.last_path = msg
        if len(msg.poses) > 0:
            self.path_event.set()

    def lookup_current_pose(self) -> PoseStamped:
        tf = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            Time(),
        )

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def wait_for_current_pose(self, timeout_sec: float) -> PoseStamped:
        """
        等待 map_frame -> base_frame 的 TF 出现。

        作用：
        - Odin 刚启动时,map -> odin1_base_link 可能还没发布；
        - 静态 TF odin1_base_link -> chassis_base_link 也可能刚启动；
        - 所以 Action Server 不应该第一次查不到 TF 就立刻失败。
        """
        start_time = time.time()
        last_error = None

        while rclpy.ok() and (time.time() - start_time) < timeout_sec:
            try:
                return self.lookup_current_pose()
            except TransformException as e:
                last_error = e
                time.sleep(0.1)

        raise RuntimeError(
            f"等待 TF 超时: {self.map_frame} -> {self.base_frame}, "
            f"timeout={timeout_sec:.1f}s, last_error={last_error}"
        )

    def resolve_target_pose(self, goal) -> PoseStamped:
        if goal.goal_name:
            item = self.goals[goal.goal_name]

            pose = PoseStamped()
            pose.header.frame_id = item.get("frame_id", self.map_frame)
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = float(item.get("x", 0.0))
            pose.pose.position.y = float(item.get("y", 0.0))
            pose.pose.position.z = float(item.get("z", 0.0))

            yaw = float(item.get("yaw", 0.0))
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            return pose

        target = goal.target_pose
        if not target.header.frame_id:
            target.header.frame_id = self.map_frame
        target.header.stamp = self.get_clock().now().to_msg()
        return target

    def publish_planning_topics(self, start_pose: PoseStamped, target_pose: PoseStamped):
        now = self.get_clock().now().to_msg()

        start_point = PointStamped()
        start_point.header.stamp = now
        start_point.header.frame_id = self.map_frame
        start_point.point.x = start_pose.pose.position.x
        start_point.point.y = start_pose.pose.position.y
        start_point.point.z = start_pose.pose.position.z

        goal_point = PointStamped()
        goal_point.header.stamp = now
        goal_point.header.frame_id = target_pose.header.frame_id or self.map_frame
        goal_point.point.x = target_pose.pose.position.x
        goal_point.point.y = target_pose.pose.position.y
        goal_point.point.z = target_pose.pose.position.z

        target_pose.header.stamp = now

        for _ in range(3):
            self.start_point_pub.publish(start_point)
            time.sleep(0.05)

            self.goal_pose_pub.publish(target_pose)
            time.sleep(0.05)

            self.goal_point_pub.publish(goal_point)
            time.sleep(0.15)

    def transform_pose_to_map(self, pose: PoseStamped) -> PoseStamped:
        if not pose.header.frame_id:
            pose.header.frame_id = self.map_frame

        if pose.header.frame_id == self.map_frame:
            pose.header.stamp = self.get_clock().now().to_msg()
            return pose

        return self.tf_buffer.transform(
            pose,
            self.map_frame,
            timeout=Duration(seconds=0.2),
        )

    def publish_direct_path(self, start_pose: PoseStamped, target_pose: PoseStamped):
        now = self.get_clock().now().to_msg()

        path = Path()
        path.header.stamp = now
        path.header.frame_id = self.map_frame

        start = PoseStamped()
        start.header = path.header
        start.pose = start_pose.pose

        target = PoseStamped()
        target.header = path.header
        target.pose = target_pose.pose

        path.poses = [start, target]
        self.last_path = path
        self.path_event.set()

        for _ in range(3):
            self.planned_path_pub.publish(path)
            time.sleep(0.05)

        self.get_logger().info(
            "direct_goal 模式：已发布两点 /planned_path，"
            f"start=({start.pose.position.x:.3f}, {start.pose.position.y:.3f}), "
            f"goal=({target.pose.position.x:.3f}, {target.pose.position.y:.3f})"
        )

    def publish_start_navigation(self, control_mode: str):
        msg = NavigationStart()
        msg.control_mode = control_mode
        self.navigation_start_pub.publish(msg)

    def publish_stop_navigation(self):
        msg = Bool()
        msg.data = True
        self.stop_nav_pub.publish(msg)

    def compute_error(self, current_pose: PoseStamped, target_pose: PoseStamped):
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)

        current_yaw = quaternion_to_yaw(current_pose.pose.orientation)
        target_yaw = quaternion_to_yaw(target_pose.pose.orientation)
        yaw_error = normalize_angle(target_yaw - current_yaw)

        return distance, yaw_error

    def make_result(self, success: bool, message: str):
        result = NavigateToPose.Result()
        result.success = success
        result.message = message
        return result

    def execute_callback(self, goal_handle):
        with self.active_lock:
            self.active_goal = True

        goal = goal_handle.request
        timeout_sec = float(goal.timeout_sec)
        if timeout_sec <= 0.0:
            timeout_sec = 60.0

        start_time = time.time()

        try:
            feedback = NavigateToPose.Feedback()
            control_mode = resolve_control_mode(goal.control_mode)

            # 1. 解析目标点
            feedback.state = "RESOLVE_GOAL"
            goal_handle.publish_feedback(feedback)
            target_pose = self.resolve_target_pose(goal)

            self.get_logger().info(
                f"导航目标解析完成: frame={target_pose.header.frame_id}, "
                f"x={target_pose.pose.position.x:.3f}, "
                f"y={target_pose.pose.position.y:.3f}"
            )

            # 2. 等待并查询当前机器人位置
            feedback.state = "WAIT_CURRENT_POSE_TF"
            goal_handle.publish_feedback(feedback)

            try:
                start_pose = self.wait_for_current_pose(self.tf_wait_timeout_sec)
            except Exception as e:
                goal_handle.abort()
                return self.make_result(False, f"等待当前 TF 失败: {e}")

            try:
                target_pose = self.transform_pose_to_map(target_pose)
            except Exception as e:
                goal_handle.abort()
                return self.make_result(False, f"目标位姿转换到 map 失败: {e}")

            # 3. 发布路径。planner 模式走规划器；direct_goal 模式直接发布两点路径。
            feedback.state = "PUBLISH_PATH"
            goal_handle.publish_feedback(feedback)

            self.path_event.clear()
            self.last_path = None
            if self.path_source_mode == "direct_goal":
                self.publish_direct_path(start_pose, target_pose)
            else:
                self.publish_planning_topics(start_pose, target_pose)

                # 4. 等待 /planned_path
                feedback.state = "WAIT_PATH"
                goal_handle.publish_feedback(feedback)

                got_path = self.path_event.wait(timeout=self.path_wait_timeout_sec)
                if not got_path:
                    goal_handle.abort()
                    return self.make_result(False, "等待 /planned_path 超时，规划失败或没有收到路径")

                self.get_logger().info("收到 /planned_path，准备启动导航")

            # 5. 启动导航
            feedback.state = "START_NAVIGATION"
            goal_handle.publish_feedback(feedback)
            self.publish_start_navigation(control_mode)

            # 6. 监控是否到达
            feedback.state = "TRACKING"
            rate_sleep = 1.0 / max(self.feedback_rate_hz, 1.0)

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.publish_stop_navigation()
                    goal_handle.canceled()
                    return self.make_result(False, "导航被取消")

                if time.time() - start_time > timeout_sec:
                    self.publish_stop_navigation()
                    goal_handle.abort()
                    return self.make_result(False, f"导航超时: {timeout_sec:.1f}s")

                try:
                    current_pose = self.lookup_current_pose()
                except TransformException as e:
                    self.get_logger().warn(f"导航中查询 TF 失败: {e}")
                    time.sleep(rate_sleep)
                    continue

                distance, yaw_error = self.compute_error(current_pose, target_pose)

                feedback.state = "TRACKING"
                feedback.distance_to_goal = float(distance)
                feedback.yaw_error = float(yaw_error)
                goal_handle.publish_feedback(feedback)

                if (
                    distance <= self.goal_position_tolerance
                    and abs(yaw_error) <= self.goal_yaw_tolerance
                ):
                    self.publish_stop_navigation()
                    goal_handle.succeed()
                    return self.make_result(
                        True,
                        f"导航成功，到达目标，distance={distance:.3f}, yaw_error={yaw_error:.3f}",
                    )

                time.sleep(rate_sleep)

            goal_handle.abort()
            return self.make_result(False, "rclpy 已关闭，导航中断")

        finally:
            with self.active_lock:
                self.active_goal = False


def main(args=None):
    rclpy.init(args=args)
    node = R2NavActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
