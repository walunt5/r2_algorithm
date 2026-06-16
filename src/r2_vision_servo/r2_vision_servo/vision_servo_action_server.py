#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PointStamped
from techx_vision_bridge.msg import Target3D
from r2_vision_servo_msgs.action import VisionServo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # 必须导入，用于注册 PointStamped 的 TF2 转换


class Pid:
    def __init__(self, kp, ki, kd, out_limit):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_limit = abs(float(out_limit))

        self.integral = 0.0
        self.last_error = None
        self.last_time = None

    def reset(self):
        self.integral = 0.0
        self.last_error = None
        self.last_time = None

    def update(self, error):
        now = time.monotonic()

        if self.last_time is None:
            dt = 0.0
        else:
            dt = max(1e-3, now - self.last_time)

        self.last_time = now

        if dt > 0.0:
            self.integral += error * dt

        if self.last_error is None or dt <= 0.0:
            derivative = 0.0
        else:
            derivative = (error - self.last_error) / dt

        self.last_error = error

        out = self.kp * error + self.ki * self.integral + self.kd * derivative

        if out > self.out_limit:
            out = self.out_limit
        elif out < -self.out_limit:
            out = -self.out_limit

        return out


class VisionServoActionServer(Node):
    def __init__(self):
        super().__init__("vision_servo_action_server")

        # 话题与坐标系
        self.declare_parameter("target_topic", "/techx/vision/targets")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("source_frame", "camera_optical_frame")
        self.declare_parameter("target_frame", "chassis_base_link")

        # 控制频率与目标超时
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("target_timeout_sec", 0.3)

        # PID 参数：第一版建议只用 P，ki/kd 先为 0
        self.declare_parameter("kp_x", 0.35)
        self.declare_parameter("ki_x", 0.0)
        self.declare_parameter("kd_x", 0.0)

        self.declare_parameter("kp_y", 0.45)
        self.declare_parameter("ki_y", 0.0)
        self.declare_parameter("kd_y", 0.0)

        # 限速，防止一上来冲太快
        self.declare_parameter("max_vx", 0.20)
        self.declare_parameter("max_vy", 0.20)

        self.target_topic = self.get_parameter("target_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.source_frame = self.get_parameter("source_frame").value
        self.target_frame = self.get_parameter("target_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.latest_msg = None
        self.latest_msg_time = None
        self.latest_lock = threading.Lock()

        vision_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            Target3D,
            self.target_topic,
            self.on_target,
            vision_qos,
        )

        self.action_server = ActionServer(
            self,
            VisionServo,
            "vision_servo",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            f"vision_servo_action_server 已启动：订阅 {self.target_topic}，发布 {self.cmd_vel_topic}"
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"收到视觉伺服请求：desired=({goal_request.desired_x:.3f}, "
            f"{goal_request.desired_y:.3f}, {goal_request.desired_z:.3f}), "
            f"tol=({goal_request.tolerance_x:.3f}, {goal_request.tolerance_y:.3f}, "
            f"{goal_request.tolerance_z:.3f}), timeout={goal_request.timeout_sec:.1f}s"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("收到取消视觉伺服请求")
        self.publish_stop()
        return CancelResponse.ACCEPT

    def on_target(self, msg: Target3D):
        with self.latest_lock:
            self.latest_msg = msg
            self.latest_msg_time = self.get_clock().now()

    def publish_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def get_latest_target_in_chassis(self):
        with self.latest_lock:
            msg = self.latest_msg
            msg_time = self.latest_msg_time

        if msg is None or msg_time is None:
            return None, "尚未收到视觉目标"

        age = (self.get_clock().now() - msg_time).nanoseconds / 1e9
        target_timeout = float(self.get_parameter("target_timeout_sec").value)

        if age > target_timeout:
            return None, f"视觉目标超时：{age:.3f}s > {target_timeout:.3f}s"

        p_camera = PointStamped()

        # 对静态 TF 来说，用当前时间最稳，避免上游时间戳导致外推错误
        p_camera.header.stamp = self.get_clock().now().to_msg()

        # 不使用 msg.header.frame_id，避免 camera_link_0 / camera_link_1 影响 TF
        # 统一认为 x/y/z 来自真实相机坐标系 camera_optical_frame
        p_camera.header.frame_id = self.source_frame

        p_camera.point.x = float(msg.x)
        p_camera.point.y = float(msg.y)
        p_camera.point.z = float(msg.z)

        try:
            p_chassis = self.tf_buffer.transform(
                p_camera,
                self.target_frame,
                timeout=Duration(seconds=0.05),
            )
        except Exception as e:
            return None, f"TF 转换失败：{e}"

        return p_chassis, ""

    def execute_callback(self, goal_handle):
        goal = goal_handle.request

        pid_x = Pid(
            self.get_parameter("kp_x").value,
            self.get_parameter("ki_x").value,
            self.get_parameter("kd_x").value,
            self.get_parameter("max_vx").value,
        )

        pid_y = Pid(
            self.get_parameter("kp_y").value,
            self.get_parameter("ki_y").value,
            self.get_parameter("kd_y").value,
            self.get_parameter("max_vy").value,
        )

        desired_x = float(goal.desired_x)
        desired_y = float(goal.desired_y)
        desired_z = float(goal.desired_z)

        tol_x = abs(float(goal.tolerance_x))
        tol_y = abs(float(goal.tolerance_y))
        tol_z = abs(float(goal.tolerance_z))

        use_z_tolerance = bool(goal.use_z_tolerance)
        stable_frames_required = max(1, int(goal.stable_frames))
        timeout_sec = max(0.5, float(goal.timeout_sec))

        stable_count = 0
        start_time = time.monotonic()

        rate_hz = float(self.get_parameter("control_rate_hz").value)
        sleep_dt = 1.0 / max(1.0, rate_hz)

        final_target_x = 0.0
        final_target_y = 0.0
        final_target_z = 0.0
        final_error_x = 0.0
        final_error_y = 0.0
        final_error_z = 0.0

        self.get_logger().info("视觉伺服开始")

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.publish_stop()
                goal_handle.canceled()
                result = VisionServo.Result()
                result.success = False
                result.message = "视觉伺服被取消"
                return result

            elapsed = time.monotonic() - start_time
            if elapsed > timeout_sec:
                self.publish_stop()
                goal_handle.abort()
                result = VisionServo.Result()
                result.success = False
                result.message = f"视觉伺服超时：{timeout_sec:.1f}s"
                result.final_target_x = final_target_x
                result.final_target_y = final_target_y
                result.final_target_z = final_target_z
                result.final_error_x = final_error_x
                result.final_error_y = final_error_y
                result.final_error_z = final_error_z
                return result

            p_chassis, err_msg = self.get_latest_target_in_chassis()
            if p_chassis is None:
                self.publish_stop()
                self.get_logger().warn(err_msg)
                time.sleep(sleep_dt)
                continue

            target_x = float(p_chassis.point.x)
            target_y = float(p_chassis.point.y)
            target_z = float(p_chassis.point.z)

            error_x = target_x - desired_x
            error_y = target_y - desired_y
            error_z = target_z - desired_z

            final_target_x = target_x
            final_target_y = target_y
            final_target_z = target_z
            final_error_x = error_x
            final_error_y = error_y
            final_error_z = error_z

            x_ok = abs(error_x) <= tol_x
            y_ok = abs(error_y) <= tol_y
            z_ok = abs(error_z) <= tol_z if use_z_tolerance else True
            in_tolerance = x_ok and y_ok and z_ok

            if in_tolerance:
                stable_count += 1
                # 只要进入容差，就不要再继续动
                self.publish_stop()
            else:
                stable_count = 0

            feedback = VisionServo.Feedback()
            feedback.target_x = target_x
            feedback.target_y = target_y
            feedback.target_z = target_z
            feedback.error_x = error_x
            feedback.error_y = error_y
            feedback.error_z = error_z
            feedback.stable_count = stable_count
            goal_handle.publish_feedback(feedback)

            if stable_count >= stable_frames_required:
                self.publish_stop()
                goal_handle.succeed()

                result = VisionServo.Result()
                result.success = True
                result.message = "视觉伺服成功"
                result.final_target_x = target_x
                result.final_target_y = target_y
                result.final_target_z = target_z
                result.final_error_x = error_x
                result.final_error_y = error_y
                result.final_error_z = error_z

                self.get_logger().info(
                    f"视觉伺服成功：error_x={error_x:.3f}, "
                    f"error_y={error_y:.3f}, error_z={error_z:.3f}"
                )
                return result

            # 没进入容差时，才允许发布速度
            cmd = Twist()
            cmd.linear.x = pid_x.update(error_x)
            cmd.linear.y = pid_y.update(error_y)
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                f"target=({target_x:.3f}, {target_y:.3f}, {target_z:.3f}), "
                f"error=({error_x:.3f}, {error_y:.3f}, {error_z:.3f}), "
                f"cmd=({cmd.linear.x:.3f}, {cmd.linear.y:.3f})",
            )

            self.get_logger().info(
                f"target=({target_x:.3f}, {target_y:.3f}, {target_z:.3f}), "
                f"error=({error_x:.3f}, {error_y:.3f}, {error_z:.3f}), "
                f"cmd=({cmd.linear.x:.3f}, {cmd.linear.y:.3f})",
            )

            time.sleep(sleep_dt)

        self.publish_stop()
        goal_handle.abort()
        result = VisionServo.Result()
        result.success = False
        result.message = "rclpy 已退出"
        return result


def main():
    rclpy.init()
    node = VisionServoActionServer()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()