#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from r2_vision_servo_interfaces.action import VisionServo
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from techx_vision_bridge.msg import VisionRequest, VisionSelection
from tf2_ros import Buffer, TransformException, TransformListener


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


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min(value, max_value), min_value)


class Pid:
    def __init__(self, kp: float, ki: float, kd: float, out_limit: float, i_limit: float):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_limit = abs(float(out_limit))
        self.i_limit = abs(float(i_limit))
        self.integral = 0.0
        self.last_error = None
        self.last_time = None

    def reset(self) -> None:
        self.integral = 0.0
        self.last_error = None
        self.last_time = None

    def update(self, error: float) -> float:
        now = time.monotonic()
        if self.last_time is None:
            dt = 0.0
        else:
            dt = max(1.0e-3, now - self.last_time)
        self.last_time = now

        if dt > 0.0:
            self.integral = clamp(
                self.integral + error * dt,
                -self.i_limit,
                self.i_limit,
            )

        if self.last_error is None or dt <= 0.0:
            derivative = 0.0
        else:
            derivative = (error - self.last_error) / dt
        self.last_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(output, -self.out_limit, self.out_limit)


class VisionServoActionServer(Node):
    def __init__(self):
        super().__init__("vision_servo_action_server")

        self.declare_parameter("action_name", "/r2_chassis/vision_servo")
        self.declare_parameter("request_topic", "/techx/vision/request")
        self.declare_parameter("selected_topic", "/techx/vision/selected")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "chassis_base_link")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("tf_timeout_sec", 0.10)
        self.declare_parameter("enable_cmd_vel", False)
        self.declare_parameter("request_republish_sec", 0.5)

        self.declare_parameter("kp_x", 0.8)
        self.declare_parameter("ki_x", 0.0)
        self.declare_parameter("kd_x", 0.03)
        self.declare_parameter("kp_y", 0.8)
        self.declare_parameter("ki_y", 0.0)
        self.declare_parameter("kd_y", 0.03)
        self.declare_parameter("kp_yaw", 1.2)
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("kd_yaw", 0.05)
        self.declare_parameter("i_limit", 0.2)
        self.declare_parameter("max_vx", 0.06)
        self.declare_parameter("max_vy", 0.06)
        self.declare_parameter("max_wz", 0.20)

        self.action_name = self.param_str("action_name")
        self.request_topic = self.param_str("request_topic")
        self.selected_topic = self.param_str("selected_topic")
        self.cmd_vel_topic = self.param_str("cmd_vel_topic")
        self.global_frame = self.param_str("global_frame")
        self.robot_frame = self.param_str("robot_frame")

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.request_pub = self.create_publisher(VisionRequest, self.request_topic, qos)
        self.selected_sub = self.create_subscription(
            VisionSelection,
            self.selected_topic,
            self.on_selection,
            qos,
            callback_group=self.callback_group,
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.latest_selection = None
        self.latest_selection_time = None
        self.selection_lock = threading.Lock()
        self.goal_lock = threading.Lock()
        self.goal_active = False
        self.request_seq = 0

        self.action_server = ActionServer(
            self,
            VisionServo,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        mode = "LIVE /cmd_vel" if self.param_bool("enable_cmd_vel") else "DRY-RUN"
        self.get_logger().info(
            f"vision_servo_action_server ready action={self.action_name} "
            f"request={self.request_topic} selected={self.selected_topic} "
            f"cmd_vel={self.cmd_vel_topic} mode={mode}"
        )

    def param_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def param_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def on_selection(self, msg: VisionSelection) -> None:
        with self.selection_lock:
            self.latest_selection = msg
            self.latest_selection_time = self.get_clock().now()

    def goal_callback(self, goal_request):
        ok, message = self.validate_goal(goal_request)
        if not ok:
            self.get_logger().error(f"Reject vision servo goal: {message}")
            return GoalResponse.REJECT

        with self.goal_lock:
            if self.goal_active:
                self.get_logger().warn("Reject vision servo goal because another goal is active")
                return GoalResponse.REJECT
            self.goal_active = True

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("vision servo cancel requested")
        self.publish_stop()
        return CancelResponse.ACCEPT

    def validate_goal(self, goal) -> tuple[bool, str]:
        valid_strategies = {
            VisionServo.Goal.ALIGN_STRATEGY_YAW_THEN_Y_THEN_X,
            VisionServo.Goal.ALIGN_STRATEGY_YAW_GATE_XY_PARALLEL,
        }
        valid_yaw_modes = {
            VisionServo.Goal.YAW_MODE_NONE,
            VisionServo.Goal.YAW_MODE_HOLD_CURRENT_ODIN_YAW,
            VisionServo.Goal.YAW_MODE_USE_GOAL_YAW,
        }
        if goal.align_strategy not in valid_strategies:
            return False, f"unsupported align_strategy={goal.align_strategy}"
        if goal.yaw_mode not in valid_yaw_modes:
            return False, f"unsupported yaw_mode={goal.yaw_mode}"
        if goal.timeout_ms == 0:
            return False, "timeout_ms must be > 0"
        if goal.stable_required_frames == 0:
            return False, "stable_required_frames must be > 0"
        if goal.max_frame_age_sec <= 0.0:
            return False, "max_frame_age_sec must be > 0"
        return True, "ok"

    def next_request_seq(self) -> int:
        self.request_seq = (self.request_seq + 1) & 0xFFFFFFFF
        if self.request_seq == 0:
            self.request_seq = 1
        return self.request_seq

    def publish_request(self, goal, request_seq: int) -> None:
        msg = VisionRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.request_seq = request_seq
        msg.target_type = int(goal.target_type)
        msg.zone_id = int(goal.zone_id)
        msg.use_class_id = bool(goal.use_class_id)
        msg.class_id = int(goal.class_id)
        msg.use_color = bool(goal.use_color)
        msg.color = int(goal.color)
        msg.require_control_xyz = False
        msg.min_confidence = float(goal.min_confidence)
        msg.max_frame_age_sec = float(goal.max_frame_age_sec)
        self.request_pub.publish(msg)

    def get_current_selection(self, request_seq: int):
        with self.selection_lock:
            selection = self.latest_selection
            selection_time = self.latest_selection_time
        if selection is None or selection_time is None:
            return None
        if selection.request_seq != request_seq:
            return None
        return selection

    def lookup_current_yaw(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=self.param_float("tf_timeout_sec")),
            )
        except TransformException as exc:
            return None, str(exc)
        return yaw_from_quaternion(transform.transform.rotation), ""

    def publish_stop(self) -> None:
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def publish_cmd(self, cmd: Twist) -> None:
        if self.param_bool("enable_cmd_vel"):
            self.cmd_pub.publish(cmd)

    def make_pid(self, axis: str, out_limit_name: str) -> Pid:
        return Pid(
            self.param_float(f"kp_{axis}"),
            self.param_float(f"ki_{axis}"),
            self.param_float(f"kd_{axis}"),
            self.param_float(out_limit_name),
            self.param_float("i_limit"),
        )

    def make_result(
        self,
        success: bool,
        error_code: int,
        message: str,
        final_robot_x: float = 0.0,
        final_robot_y: float = 0.0,
        final_robot_z: float = 0.0,
        final_error_x: float = 0.0,
        final_error_y: float = 0.0,
        final_error_z: float = 0.0,
        final_yaw_rad: float = 0.0,
        final_yaw_error_rad: float = 0.0,
    ):
        result = VisionServo.Result()
        result.success = bool(success)
        result.error_code = int(error_code)
        result.message = message
        result.final_robot_x = float(final_robot_x)
        result.final_robot_y = float(final_robot_y)
        result.final_robot_z = float(final_robot_z)
        result.final_error_x = float(final_error_x)
        result.final_error_y = float(final_error_y)
        result.final_error_z = float(final_error_z)
        result.final_yaw_rad = float(final_yaw_rad)
        result.final_yaw_error_rad = float(final_yaw_error_rad)
        return result

    def publish_feedback(
        self,
        goal_handle,
        phase: int,
        message: str,
        robot_x: float,
        robot_y: float,
        robot_z: float,
        error_x: float,
        error_y: float,
        error_z: float,
        current_yaw: float,
        target_yaw: float,
        yaw_error: float,
        cmd: Twist,
    ) -> None:
        feedback = VisionServo.Feedback()
        feedback.phase = int(phase)
        feedback.message = message
        feedback.current_robot_x = float(robot_x)
        feedback.current_robot_y = float(robot_y)
        feedback.current_robot_z = float(robot_z)
        feedback.error_x = float(error_x)
        feedback.error_y = float(error_y)
        feedback.error_z = float(error_z)
        feedback.current_yaw_rad = float(current_yaw)
        feedback.target_yaw_rad = float(target_yaw)
        feedback.yaw_error_rad = float(yaw_error)
        feedback.cmd_vx = float(cmd.linear.x)
        feedback.cmd_vy = float(cmd.linear.y)
        feedback.cmd_wz = float(cmd.angular.z)
        goal_handle.publish_feedback(feedback)

    def build_cmd(
        self,
        goal,
        pid_x: Pid,
        pid_y: Pid,
        pid_yaw: Pid,
        error_x: float,
        error_y: float,
        yaw_error: float,
    ):
        cmd = Twist()
        yaw_enabled = goal.yaw_mode != VisionServo.Goal.YAW_MODE_NONE
        yaw_ok = (not yaw_enabled) or abs(yaw_error) <= goal.yaw_tolerance_rad

        if goal.align_strategy == VisionServo.Goal.ALIGN_STRATEGY_YAW_THEN_Y_THEN_X:
            if yaw_enabled and abs(yaw_error) > goal.yaw_tolerance_rad:
                cmd.angular.z = pid_yaw.update(yaw_error)
                return cmd, VisionServo.Feedback.PHASE_ALIGN_YAW, "align yaw"

            if abs(error_y) > abs(goal.tolerance_y):
                if yaw_enabled and abs(yaw_error) > goal.yaw_gate_rad:
                    cmd.angular.z = pid_yaw.update(yaw_error)
                    return cmd, VisionServo.Feedback.PHASE_ALIGN_YAW, "yaw drift too large"
                cmd.linear.y = pid_y.update(error_y)
                if yaw_enabled:
                    cmd.angular.z = pid_yaw.update(yaw_error)
                return cmd, VisionServo.Feedback.PHASE_ALIGN_Y, "align y"

            cmd.linear.x = pid_x.update(error_x)
            cmd.linear.y = pid_y.update(error_y)
            if yaw_enabled:
                cmd.angular.z = pid_yaw.update(yaw_error)
            return cmd, VisionServo.Feedback.PHASE_ALIGN_X, "align x"

        if yaw_enabled and abs(yaw_error) > goal.yaw_gate_rad:
            cmd.angular.z = pid_yaw.update(yaw_error)
            return cmd, VisionServo.Feedback.PHASE_ALIGN_YAW, "yaw gate"

        cmd.linear.x = pid_x.update(error_x)
        cmd.linear.y = pid_y.update(error_y)
        return cmd, VisionServo.Feedback.PHASE_ALIGN_XY, "align xy"

    def success_in_tolerance(
        self,
        goal,
        error_x: float,
        error_y: float,
        error_z: float,
        yaw_error: float,
    ) -> bool:
        x_ok = abs(error_x) <= abs(goal.tolerance_x)
        y_ok = abs(error_y) <= abs(goal.tolerance_y)
        z_ok = abs(error_z) <= abs(goal.tolerance_z)
        if goal.align_strategy == VisionServo.Goal.ALIGN_STRATEGY_YAW_THEN_Y_THEN_X:
            yaw_ok = (
                goal.yaw_mode == VisionServo.Goal.YAW_MODE_NONE
                or abs(yaw_error) <= abs(goal.yaw_tolerance_rad)
            )
            return x_ok and y_ok and z_ok and yaw_ok
        return x_ok and y_ok and z_ok

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        request_seq = self.next_request_seq()
        self.publish_request(goal, request_seq)

        target_yaw = float(goal.target_yaw_rad)
        current_yaw = 0.0
        if goal.yaw_mode == VisionServo.Goal.YAW_MODE_HOLD_CURRENT_ODIN_YAW:
            yaw, err = self.lookup_current_yaw()
            if yaw is None:
                self.publish_stop()
                goal_handle.abort()
                with self.goal_lock:
                    self.goal_active = False
                return self.make_result(
                    False,
                    VisionServo.Result.ERROR_NO_ODIN_POSE,
                    f"failed to lock current yaw: {err}",
                )
            current_yaw = yaw
            target_yaw = yaw

        pid_x = self.make_pid("x", "max_vx")
        pid_y = self.make_pid("y", "max_vy")
        pid_yaw = self.make_pid("yaw", "max_wz")

        stable_count = 0
        start_time = time.monotonic()
        last_request_time = 0.0
        last_log_time = 0.0
        timeout_sec = max(float(goal.timeout_ms) / 1000.0, 0.1)
        sleep_dt = 1.0 / max(self.param_float("control_rate_hz"), 1.0)

        final_robot_x = 0.0
        final_robot_y = 0.0
        final_robot_z = 0.0
        final_error_x = 0.0
        final_error_y = 0.0
        final_error_z = 0.0
        final_yaw_error = 0.0
        last_error_code = VisionServo.Result.ERROR_NO_VISION_FRAME
        last_message = "waiting for matching vision selection"

        try:
            dry_run = "false" if self.param_bool("enable_cmd_vel") else "true"
            self.get_logger().info(
                f"vision servo start request_seq={request_seq} "
                f"target_type={goal.target_type} zone_id={goal.zone_id} "
                f"class_id={goal.class_id} strategy={goal.align_strategy} "
                f"yaw_mode={goal.yaw_mode} dry_run={dry_run}"
            )

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.publish_stop()
                    goal_handle.canceled()
                    return self.make_result(
                        False,
                        VisionServo.Result.ERROR_CANCELED,
                        "vision servo canceled",
                        final_robot_x,
                        final_robot_y,
                        final_robot_z,
                        final_error_x,
                        final_error_y,
                        final_error_z,
                        current_yaw,
                        final_yaw_error,
                    )

                now = time.monotonic()
                if now - start_time > timeout_sec:
                    self.publish_stop()
                    goal_handle.abort()
                    return self.make_result(
                        False,
                        VisionServo.Result.ERROR_TIMEOUT,
                        f"vision servo timeout after {timeout_sec:.2f}s: {last_message}",
                        final_robot_x,
                        final_robot_y,
                        final_robot_z,
                        final_error_x,
                        final_error_y,
                        final_error_z,
                        current_yaw,
                        final_yaw_error,
                    )

                if now - last_request_time > self.param_float("request_republish_sec"):
                    self.publish_request(goal, request_seq)
                    last_request_time = now

                selection = self.get_current_selection(request_seq)
                cmd = Twist()
                if selection is None:
                    last_error_code = VisionServo.Result.ERROR_NO_VISION_FRAME
                    last_message = "no VisionSelection for current request"
                    self.publish_stop()
                    self.publish_feedback(
                        goal_handle,
                        VisionServo.Feedback.PHASE_WAIT_TARGET,
                        last_message,
                        final_robot_x,
                        final_robot_y,
                        final_robot_z,
                        final_error_x,
                        final_error_y,
                        final_error_z,
                        current_yaw,
                        target_yaw,
                        final_yaw_error,
                        cmd,
                    )
                    time.sleep(sleep_dt)
                    continue

                if selection.status != VisionSelection.STATUS_OK or not selection.has_match:
                    if selection.status == VisionSelection.STATUS_NO_MATCH:
                        last_error_code = VisionServo.Result.ERROR_NO_MATCH
                    elif selection.status == VisionSelection.STATUS_FRAME_STALE:
                        last_error_code = VisionServo.Result.ERROR_FRAME_STALE
                    else:
                        last_error_code = VisionServo.Result.ERROR_NO_VISION_FRAME
                    last_message = f"selection status={selection.status} has_match={selection.has_match}"
                    self.publish_stop()
                    self.publish_feedback(
                        goal_handle,
                        VisionServo.Feedback.PHASE_WAIT_TARGET,
                        last_message,
                        final_robot_x,
                        final_robot_y,
                        final_robot_z,
                        final_error_x,
                        final_error_y,
                        final_error_z,
                        current_yaw,
                        target_yaw,
                        final_yaw_error,
                        cmd,
                    )
                    time.sleep(sleep_dt)
                    continue

                target = selection.target
                if not target.valid_robot_xyz:
                    last_error_code = VisionServo.Result.ERROR_NO_MATCH
                    last_message = "selected target has no valid robot xyz"
                    self.publish_stop()
                    time.sleep(sleep_dt)
                    continue

                if goal.yaw_mode != VisionServo.Goal.YAW_MODE_NONE:
                    yaw, err = self.lookup_current_yaw()
                    if yaw is None:
                        last_error_code = VisionServo.Result.ERROR_NO_ODIN_POSE
                        last_message = f"TF yaw unavailable: {err}"
                        self.publish_stop()
                        time.sleep(sleep_dt)
                        continue
                    current_yaw = yaw
                    final_yaw_error = normalize_angle(target_yaw - current_yaw)
                else:
                    current_yaw = 0.0
                    final_yaw_error = 0.0

                final_robot_x = float(target.robot_x)
                final_robot_y = float(target.robot_y)
                final_robot_z = float(target.robot_z)
                final_error_x = final_robot_x - float(goal.desired_robot_x)
                final_error_y = final_robot_y - float(goal.desired_robot_y)
                final_error_z = final_robot_z - float(goal.desired_robot_z)

                in_tolerance = self.success_in_tolerance(
                    goal,
                    final_error_x,
                    final_error_y,
                    final_error_z,
                    final_yaw_error,
                )
                if in_tolerance:
                    stable_count += 1
                    self.publish_stop()
                    cmd = Twist()
                    phase = VisionServo.Feedback.PHASE_DONE
                    message = f"in tolerance stable_count={stable_count}"
                else:
                    stable_count = 0
                    cmd, phase, message = self.build_cmd(
                        goal,
                        pid_x,
                        pid_y,
                        pid_yaw,
                        final_error_x,
                        final_error_y,
                        final_yaw_error,
                    )
                    self.publish_cmd(cmd)

                self.publish_feedback(
                    goal_handle,
                    phase,
                    message,
                    final_robot_x,
                    final_robot_y,
                    final_robot_z,
                    final_error_x,
                    final_error_y,
                    final_error_z,
                    current_yaw,
                    target_yaw,
                    final_yaw_error,
                    cmd,
                )

                if now - last_log_time > 1.0:
                    last_log_time = now
                    self.get_logger().info(
                        f"vision servo phase={phase} "
                        f"err=({final_error_x:.3f}, {final_error_y:.3f}, {final_error_z:.3f}) "
                        f"yaw_err={final_yaw_error:.3f} "
                        f"cmd=({cmd.linear.x:.3f}, {cmd.linear.y:.3f}, {cmd.angular.z:.3f}) "
                        f"stable={stable_count}/{goal.stable_required_frames}"
                    )

                if stable_count >= int(goal.stable_required_frames):
                    self.publish_stop()
                    goal_handle.succeed()
                    return self.make_result(
                        True,
                        VisionServo.Result.ERROR_SUCCESS,
                        "vision servo succeeded",
                        final_robot_x,
                        final_robot_y,
                        final_robot_z,
                        final_error_x,
                        final_error_y,
                        final_error_z,
                        current_yaw,
                        final_yaw_error,
                    )

                last_error_code = VisionServo.Result.ERROR_SUCCESS
                last_message = "servo running"
                time.sleep(sleep_dt)

            self.publish_stop()
            goal_handle.abort()
            return self.make_result(
                False,
                last_error_code,
                "rclpy shutdown",
                final_robot_x,
                final_robot_y,
                final_robot_z,
                final_error_x,
                final_error_y,
                final_error_z,
                current_yaw,
                final_yaw_error,
            )
        finally:
            self.publish_stop()
            with self.goal_lock:
                self.goal_active = False


def main(args=None):
    rclpy.init(args=args)
    node = VisionServoActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_stop()
        except BaseException:
            pass
        try:
            executor.shutdown()
        except BaseException:
            pass
        try:
            node.destroy_node()
        except BaseException:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException:
            pass


if __name__ == "__main__":
    main()
