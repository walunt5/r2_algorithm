import math
import os
import time

import yaml

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import Twist
from r2_odin_interfaces.action import OdinPosePidAlign

from ament_index_python.packages import get_package_share_directory

from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min(value, max_value), min_value)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_yaml_file(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if data is None:
        return {}
    return data


class Pid1D:
    def __init__(self, kp: float, ki: float, kd: float, i_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = abs(i_limit)
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 1e-6:
            return self.kp * error

        self.integral += error * dt
        self.integral = clamp(self.integral, -self.i_limit, self.i_limit)

        if self.prev_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.prev_error) / dt

        self.prev_error = error

        return (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )


class OdinPosePidServer(Node):
    def __init__(self):
        super().__init__('odin_pose_pid_server')

        default_config = os.path.join(
            get_package_share_directory('r2_odin_pose_pid'),
            'config',
            'odin_pose_pid.yaml'
        )

        self.declare_parameter('config_yaml', default_config)
        config_yaml = self.get_parameter('config_yaml').get_parameter_value().string_value

        self.config = load_yaml_file(config_yaml)
        self.get_logger().info(f'Loading odin pose pid config: {config_yaml}')

        self.global_frame = self.config.get('frames', {}).get('global_frame', 'map')
        self.robot_frame = self.config.get('frames', {}).get('robot_frame', 'chassis_base_link')

        self.cmd_vel_topic = self.config.get('topics', {}).get('cmd_vel', '/cmd_vel')

        self.rate_hz = float(self.config.get('control', {}).get('rate_hz', 30.0))
        self.default_timeout_sec = float(
            self.config.get('control', {}).get('default_timeout_sec', 8.0)
        )
        self.tf_timeout_sec = float(
            self.config.get('control', {}).get('tf_timeout_sec', 0.1)
        )

        strategy = self.config.get('strategy', {})
        self.yaw_enter_xy_threshold = float(
            strategy.get('yaw_enter_xy_threshold', 0.08)
        )
        self.yaw_exit_xy_threshold = float(
            strategy.get('yaw_exit_xy_threshold', 0.12)
        )

        tolerance = self.config.get('tolerance', {})
        self.tol_x = float(tolerance.get('x', 0.02))
        self.tol_y = float(tolerance.get('y', 0.02))
        self.tol_yaw = float(tolerance.get('yaw', 0.03))
        self.stable_time_sec = float(tolerance.get('stable_time_ms', 300)) / 1000.0

        limits = self.config.get('limits', {})
        self.max_vx = float(limits.get('max_vx', 0.20))
        self.max_vy = float(limits.get('max_vy', 0.20))
        self.max_wz = float(limits.get('max_wz', 0.45))

        pid_cfg = self.config.get('pid', {})
        self.pid_x = self._make_pid(pid_cfg.get('x', {}), 0.8, 0.0, 0.03, 0.2)
        self.pid_y = self._make_pid(pid_cfg.get('y', {}), 0.8, 0.0, 0.03, 0.2)
        self.pid_yaw = self._make_pid(pid_cfg.get('yaw', {}), 1.2, 0.0, 0.05, 0.2)

        self.nav_goals_yaml = self._resolve_package_file(
            self.config.get('files', {}).get('nav_goals_package', 'r2_nav_bringup'),
            self.config.get('files', {}).get('nav_goals_relative_path', 'config/r2_nav_goals.yaml')
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=True
        )

        self.action_server = ActionServer(
            self,
            OdinPosePidAlign,
            '/r2_odin_pose_pid/align',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info(
            f'OdinPosePid REAL action server started: /r2_odin_pose_pid/align, '
            f'global_frame={self.global_frame}, robot_frame={self.robot_frame}, '
            f'cmd_vel={self.cmd_vel_topic}, nav_goals_yaml={self.nav_goals_yaml}'
        )

    def _make_pid(self, cfg: dict, kp: float, ki: float, kd: float, i_limit: float) -> Pid1D:
        return Pid1D(
            float(cfg.get('kp', kp)),
            float(cfg.get('ki', ki)),
            float(cfg.get('kd', kd)),
            float(cfg.get('i_limit', i_limit))
        )

    def _resolve_package_file(self, package: str, relative_path: str) -> str:
        return os.path.join(
            get_package_share_directory(package),
            relative_path
        )

    def goal_callback(self, goal_request):
        if goal_request.use_goal_name and not goal_request.goal_name:
            self.get_logger().error(
                'Reject goal: use_goal_name=true but goal_name is empty'
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accept align goal request: goal_name={goal_request.goal_name}, '
            f'use_goal_name={goal_request.use_goal_name}, '
            f'timeout_sec={goal_request.timeout_sec}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('Cancel request received')
        return CancelResponse.ACCEPT

    def _resolve_target_pose(self, goal) -> tuple:
        if goal.use_goal_name:
            goals_data = load_yaml_file(self.nav_goals_yaml)
            goals = goals_data.get('goals', {})

            if goal.goal_name not in goals:
                raise RuntimeError(
                    f'goal_name not found in {self.nav_goals_yaml}: {goal.goal_name}'
                )

            target = goals[goal.goal_name]
            frame_id = target.get('frame_id', self.global_frame)
            target_x = float(target.get('x', 0.0))
            target_y = float(target.get('y', 0.0))
            target_yaw = float(target.get('yaw', 0.0))

            return frame_id, target_x, target_y, target_yaw

        target_pose = goal.target_pose
        frame_id = target_pose.header.frame_id or self.global_frame
        target_x = float(target_pose.pose.position.x)
        target_y = float(target_pose.pose.position.y)
        target_yaw = yaw_from_quaternion(target_pose.pose.orientation)

        return frame_id, target_x, target_y, target_yaw

    def _lookup_current_pose(self, frame_id: str) -> tuple:
        transform = self.tf_buffer.lookup_transform(
            frame_id,
            self.robot_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=self.tf_timeout_sec)
        )

        current_x = transform.transform.translation.x
        current_y = transform.transform.translation.y
        current_yaw = yaw_from_quaternion(transform.transform.rotation)

        return current_x, current_y, current_yaw

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _make_result(self, success: bool, message: str,
                     error_x: float = 0.0, error_y: float = 0.0, error_yaw: float = 0.0):
        result = OdinPosePidAlign.Result()
        result.success = success
        result.message = message
        result.final_error_x = float(error_x)
        result.final_error_y = float(error_y)
        result.final_error_yaw = float(error_yaw)
        return result

    def execute_callback(self, goal_handle):
        goal = goal_handle.request

        try:
            target_frame, target_x, target_y, target_yaw = self._resolve_target_pose(goal)
        except Exception as e:
            self.get_logger().error(f'Failed to resolve target pose: {e}')
            goal_handle.abort()
            self._publish_zero()
            return self._make_result(False, str(e))

        timeout_sec = float(goal.timeout_sec)
        if timeout_sec <= 0.0:
            timeout_sec = self.default_timeout_sec

        self.get_logger().info(
            f'Start odin pose pid align: goal_name={goal.goal_name}, '
            f'target_frame={target_frame}, '
            f'target=({target_x:.3f}, {target_y:.3f}, {target_yaw:.3f}), '
            f'timeout_sec={timeout_sec:.2f}'
        )

        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_yaw.reset()

        stage = 'ALIGN_YAW'
        stable_start_time = None

        start_time = time.monotonic()
        last_time = start_time

        rate = self.create_rate(self.rate_hz)

        last_error_forward = 0.0
        last_error_left = 0.0
        last_error_yaw = 0.0

        while rclpy.ok():
            now = time.monotonic()
            dt = now - last_time
            last_time = now

            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Odin pose pid align canceled')
                goal_handle.canceled()
                self._publish_zero()
                return self._make_result(
                    False,
                    'canceled',
                    last_error_forward,
                    last_error_left,
                    last_error_yaw
                )

            if now - start_time > timeout_sec:
                self.get_logger().error(
                    f'Odin pose pid align timeout. '
                    f'last_error=({last_error_forward:.3f}, '
                    f'{last_error_left:.3f}, {last_error_yaw:.3f})'
                )
                goal_handle.abort()
                self._publish_zero()
                return self._make_result(
                    False,
                    'timeout',
                    last_error_forward,
                    last_error_left,
                    last_error_yaw
                )

            try:
                current_x, current_y, current_yaw = self._lookup_current_pose(target_frame)
            except TransformException as e:
                self.get_logger().warn(
                    f'TF lookup failed: {target_frame} -> {self.robot_frame}: {e}'
                )

                feedback = OdinPosePidAlign.Feedback()
                feedback.state = 'WAIT_TF'
                feedback.error_x = last_error_forward
                feedback.error_y = last_error_left
                feedback.error_yaw = last_error_yaw
                feedback.cmd_vx = 0.0
                feedback.cmd_vy = 0.0
                feedback.cmd_wz = 0.0
                goal_handle.publish_feedback(feedback)

                self._publish_zero()
                rate.sleep()
                continue

            dx = target_x - current_x
            dy = target_y - current_y

            c = math.cos(current_yaw)
            s = math.sin(current_yaw)

            error_forward = c * dx + s * dy
            error_left = -s * dx + c * dy
            error_yaw = normalize_angle(target_yaw - current_yaw)

            last_error_forward = error_forward
            last_error_left = error_left
            last_error_yaw = error_yaw

            position_ok = (
                abs(error_forward) < self.tol_x and
                abs(error_left) < self.tol_y
            )
            yaw_ok = abs(error_yaw) < self.tol_yaw

            if position_ok and yaw_ok:
                if stable_start_time is None:
                    stable_start_time = now

                if now - stable_start_time >= self.stable_time_sec:
                    self._publish_zero()
                    goal_handle.succeed()

                    self.get_logger().info(
                        f'Odin pose pid align SUCCESS. '
                        f'final_error=({error_forward:.3f}, '
                        f'{error_left:.3f}, {error_yaw:.3f})'
                    )

                    return self._make_result(
                        True,
                        'odin pose pid align success',
                        error_forward,
                        error_left,
                        error_yaw
                    )
            else:
                stable_start_time = None

            cmd = Twist()

            if stage == 'ALIGN_YAW':
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = clamp(
                    self.pid_yaw.update(error_yaw, dt),
                    -self.max_wz,
                    self.max_wz
                )

                if abs(error_yaw) < self.yaw_enter_xy_threshold:
                    stage = 'ALIGN_XY'
                    self.pid_x.reset()
                    self.pid_y.reset()
                    self.get_logger().info('Switch stage: ALIGN_YAW -> ALIGN_XY')

            elif stage == 'ALIGN_XY':
                if abs(error_yaw) > self.yaw_exit_xy_threshold:
                    stage = 'ALIGN_YAW'
                    self.pid_yaw.reset()
                    self._publish_zero()
                    self.get_logger().warn(
                        'Switch stage: ALIGN_XY -> ALIGN_YAW, yaw drift too large'
                    )
                    rate.sleep()
                    continue

                if position_ok and not yaw_ok:
                    stage = 'ALIGN_YAW'
                    self.pid_yaw.reset()
                    self._publish_zero()
                    self.get_logger().info(
                        'Switch stage: ALIGN_XY -> ALIGN_YAW, final yaw correction'
                    )
                    rate.sleep()
                    continue

                cmd.linear.x = clamp(
                    self.pid_x.update(error_forward, dt),
                    -self.max_vx,
                    self.max_vx
                )
                cmd.linear.y = clamp(
                    self.pid_y.update(error_left, dt),
                    -self.max_vy,
                    self.max_vy
                )
                cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            feedback = OdinPosePidAlign.Feedback()
            feedback.state = stage
            feedback.error_x = float(error_forward)
            feedback.error_y = float(error_left)
            feedback.error_yaw = float(error_yaw)
            feedback.cmd_vx = float(cmd.linear.x)
            feedback.cmd_vy = float(cmd.linear.y)
            feedback.cmd_wz = float(cmd.angular.z)
            goal_handle.publish_feedback(feedback)

            rate.sleep()

        self._publish_zero()
        goal_handle.abort()
        return self._make_result(
            False,
            'rclpy shutdown',
            last_error_forward,
            last_error_left,
            last_error_yaw
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdinPosePidServer()
    try:
        rclpy.spin(node)
    finally:
        node._publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()