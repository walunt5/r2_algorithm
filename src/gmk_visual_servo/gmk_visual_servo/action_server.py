"""ROS 2 action server for staged weapon visual servo control."""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

from gmk_visual_servo.controller import (
    ControlStep,
    PolylineServoController,
    ResultCode,
    ServoConfig,
    TargetObservation,
)
from gmk_visual_servo_interfaces.action import VisualServo


class VisualServoActionServer(Node):
    def __init__(self) -> None:
        super().__init__("weapon_visual_servo")
        self._declare_parameters()
        self._config = self._load_config()
        self._control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        if not math.isfinite(self._control_rate_hz) or self._control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be finite and > 0")

        target_topic = str(self.get_parameter("target_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        action_name = str(self.get_parameter("action_name").value)

        self._sample_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._latest_observation: Optional[TargetObservation] = None
        self._goal_reserved = False
        self._stop_event = threading.Event()

        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._target_group = MutuallyExclusiveCallbackGroup()
        self._action_group = ReentrantCallbackGroup()
        self._target_sub = self.create_subscription(
            Float32MultiArray,
            target_topic,
            self._target_callback,
            target_qos,
            callback_group=self._target_group,
        )
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._action_server = ActionServer(
            self,
            VisualServo,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_group,
        )
        self.get_logger().info(
            f"visual servo ready: action={action_name}, target={target_topic}, "
            f"cmd={cmd_vel_topic}, strategy=Y-then-X"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "target_topic": "/weapon_target",
            "cmd_vel_topic": "/cmd_vel",
            "action_name": "/weapon_visual_servo",
            "u_ref_px": 320.0,
            "u_tolerance_px": 8.0,
            "u_reentry_px": 16.0,
            "y_settle_time_sec": 0.3,
            "y_reentry_time_sec": 0.1,
            "depth_tolerance_m": 0.02,
            "final_settle_time_sec": 0.3,
            "vy_gain": 0.0015,
            "vx_gain": 0.8,
            "min_vy_mps": 0.05,
            "min_vx_mps": 0.05,
            "max_vy_mps": 0.15,
            "max_vx_mps": 0.20,
            "min_confidence": 0.6,
            "vision_timeout_sec": 0.1,
            "target_reacquire_sec": 0.3,
            "control_rate_hz": 50.0,
            "min_valid_depth_m": 0.05,
            "max_valid_depth_m": 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_config(self) -> ServoConfig:
        names = (
            "u_ref_px",
            "u_tolerance_px",
            "u_reentry_px",
            "y_settle_time_sec",
            "y_reentry_time_sec",
            "depth_tolerance_m",
            "final_settle_time_sec",
            "vy_gain",
            "vx_gain",
            "min_vy_mps",
            "min_vx_mps",
            "max_vy_mps",
            "max_vx_mps",
            "min_confidence",
            "vision_timeout_sec",
            "target_reacquire_sec",
            "min_valid_depth_m",
            "max_valid_depth_m",
        )
        values = {name: float(self.get_parameter(name).value) for name in names}
        config = ServoConfig(**values)
        config.validate()
        return config

    def _target_callback(self, msg: Float32MultiArray) -> None:
        now = time.monotonic()
        data = list(msg.data)
        if len(data) < 5:
            observation = TargetObservation(False, 0.0, 0.0, 0.0, 0.0, now)
            self.get_logger().warning(
                f"ignored malformed /weapon_target: expected 5 values, got {len(data)}",
                throttle_duration_sec=2.0,
            )
        else:
            observation = TargetObservation(
                valid=data[0] >= 0.5,
                u=float(data[1]),
                v=float(data[2]),
                depth_m=float(data[3]),
                confidence=float(data[4]),
                received_at=now,
            )
        with self._sample_lock:
            self._latest_observation = observation

    def _goal_callback(self, goal_request: VisualServo.Goal) -> GoalResponse:
        target_distance = float(goal_request.target_distance_m)
        timeout = float(goal_request.timeout_sec)
        if not math.isfinite(target_distance) or not (
            self._config.min_valid_depth_m
            <= target_distance
            <= self._config.max_valid_depth_m
        ):
            self.get_logger().warning("rejected goal: target_distance_m is invalid")
            return GoalResponse.REJECT
        if not math.isfinite(timeout) or timeout <= 0.0:
            self.get_logger().warning("rejected goal: timeout_sec must be > 0")
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_reserved:
                self.get_logger().warning("rejected goal: another visual-servo goal is active")
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        controller = PolylineServoController(self._config)
        controller.start(
            time.monotonic(),
            float(goal_handle.request.target_distance_m),
            float(goal_handle.request.timeout_sec),
        )
        period = 1.0 / self._control_rate_hz
        last_step: Optional[ControlStep] = None
        self.get_logger().info(
            "accepted visual-servo goal: "
            f"distance={goal_handle.request.target_distance_m:.3f} m, "
            f"timeout={goal_handle.request.timeout_sec:.2f} s"
        )

        try:
            while rclpy.ok() and not self._stop_event.is_set():
                if goal_handle.is_cancel_requested:
                    self._publish_zero()
                    goal_handle.canceled()
                    return self._make_result(
                        ResultCode.CANCELED,
                        "visual-servo goal canceled",
                        last_step,
                    )

                with self._sample_lock:
                    observation = self._latest_observation
                step = controller.step(time.monotonic(), observation)
                last_step = step
                self._publish_command(step.vx_mps, step.vy_mps)
                goal_handle.publish_feedback(self._make_feedback(step))

                if step.terminal_code is not None:
                    self._publish_zero()
                    if step.terminal_code == ResultCode.SUCCESS:
                        goal_handle.succeed()
                    else:
                        goal_handle.abort()
                    self.get_logger().info(
                        f"visual-servo finished: code={int(step.terminal_code)}, {step.message}"
                    )
                    return self._make_result(step.terminal_code, step.message, step)

                self._stop_event.wait(period)

            self._publish_zero()
            goal_handle.abort()
            return self._make_result(
                ResultCode.INTERNAL_ERROR,
                "visual-servo server stopped",
                last_step,
            )
        except Exception as exc:  # Ensure every unexpected failure stops the chassis.
            self._publish_zero()
            self.get_logger().error(f"visual-servo execution failed: {exc!r}")
            try:
                goal_handle.abort()
            except Exception:
                pass
            return self._make_result(ResultCode.INTERNAL_ERROR, str(exc), last_step)
        finally:
            self._publish_zero()
            with self._goal_lock:
                self._goal_reserved = False

    @staticmethod
    def _make_feedback(step: ControlStep) -> VisualServo.Feedback:
        feedback = VisualServo.Feedback()
        feedback.state = int(step.phase)
        feedback.u = float(step.u)
        feedback.v = float(step.v)
        feedback.depth_m = float(step.depth_m)
        feedback.confidence = float(step.confidence)
        feedback.error_u_px = float(step.error_u_px)
        feedback.error_depth_m = float(step.error_depth_m)
        feedback.command_vx_mps = float(step.vx_mps)
        feedback.command_vy_mps = float(step.vy_mps)
        feedback.elapsed_sec = float(step.elapsed_sec)
        return feedback

    @staticmethod
    def _make_result(
        code: ResultCode, message: str, step: Optional[ControlStep]
    ) -> VisualServo.Result:
        result = VisualServo.Result()
        result.code = int(code)
        result.message = message
        result.final_u_error_px = float(step.error_u_px) if step is not None else 0.0
        result.final_depth_error_m = float(step.error_depth_m) if step is not None else 0.0
        return result

    def _publish_command(self, vx: float, vy: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = 0.0
        self._cmd_pub.publish(msg)

    def _publish_zero(self) -> None:
        self._publish_command(0.0, 0.0)

    def request_stop(self) -> None:
        self._stop_event.set()
        self._publish_zero()

    def destroy_node(self):
        self.request_stop()
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualServoActionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.request_stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
