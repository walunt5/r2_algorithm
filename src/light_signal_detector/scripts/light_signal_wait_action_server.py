#!/usr/bin/env python3
"""Wait for a new positive light-signal sample through a ROS 2 action."""

import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from r2_light_interfaces.action import WaitForLightSignal


class LightSignalWaitActionServer(Node):
    """Convert new samples on a Bool topic into bounded action results."""

    def __init__(self) -> None:
        super().__init__("light_signal_wait_action_server")
        self.declare_parameter("signal_topic", "/light_signal/on")
        self.declare_parameter("action_name", "/r2_light_signal/wait")
        self.declare_parameter("feedback_rate_hz", 10.0)

        feedback_rate_hz = float(self.get_parameter("feedback_rate_hz").value)
        if feedback_rate_hz <= 0.0:
            raise ValueError("feedback_rate_hz must be greater than zero")
        self._feedback_period_sec = 1.0 / feedback_rate_hz

        self._lock = threading.Lock()
        self._signal_sequence = 0
        self._last_true_sequence = 0
        self._last_signal = False
        self._goal_reserved = False
        self._callback_group = ReentrantCallbackGroup()

        signal_topic = str(self.get_parameter("signal_topic").value)
        action_name = str(self.get_parameter("action_name").value)
        self._signal_sub = self.create_subscription(
            Bool,
            signal_topic,
            self._signal_callback,
            10,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            WaitForLightSignal,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f"Light signal wait action ready: action={action_name}, "
            f"signal_topic={signal_topic}"
        )

    def _signal_callback(self, msg: Bool) -> None:
        with self._lock:
            self._signal_sequence += 1
            self._last_signal = bool(msg.data)
            if self._last_signal:
                self._last_true_sequence = self._signal_sequence

    def _goal_callback(self, goal_request) -> GoalResponse:
        if float(goal_request.timeout_sec) <= 0.0:
            self.get_logger().error(
                f"Reject light wait goal: invalid timeout_sec={goal_request.timeout_sec}"
            )
            return GoalResponse.REJECT
        with self._lock:
            if self._goal_reserved:
                self.get_logger().warn(
                    "Reject light wait goal: another goal is already active"
                )
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _make_result(self, success: bool, last_signal: bool, message: str):
        result = WaitForLightSignal.Result()
        result.success = success
        result.last_signal = last_signal
        result.message = message
        return result

    def _execute_callback(self, goal_handle):
        timeout_sec = float(goal_handle.request.timeout_sec)
        start_time = time.monotonic()
        with self._lock:
            start_sequence = self._signal_sequence

        received_new_signal = False
        try:
            while True:
                elapsed_sec = time.monotonic() - start_time
                with self._lock:
                    current_sequence = self._signal_sequence
                    last_true_sequence = self._last_true_sequence
                    last_signal = self._last_signal

                received_new_signal = current_sequence > start_sequence
                if last_true_sequence > start_sequence:
                    goal_handle.succeed()
                    return self._make_result(
                        True,
                        last_signal,
                        f"New true light signal received after {elapsed_sec:.3f}s",
                    )

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._make_result(
                        False,
                        last_signal,
                        "Light signal wait was canceled",
                    )

                if elapsed_sec >= timeout_sec:
                    goal_handle.abort()
                    if received_new_signal:
                        message = (
                            f"Timed out after {timeout_sec:.3f}s; "
                            "new signals were received but none was true"
                        )
                    else:
                        message = (
                            f"Timed out after {timeout_sec:.3f}s; "
                            "no new light signal was received"
                        )
                    return self._make_result(False, last_signal, message)

                feedback = WaitForLightSignal.Feedback()
                feedback.signal_received = received_new_signal
                feedback.detected = last_signal if received_new_signal else False
                feedback.elapsed_sec = float(elapsed_sec)
                goal_handle.publish_feedback(feedback)
                time.sleep(
                    min(self._feedback_period_sec, timeout_sec - elapsed_sec)
                )
        finally:
            with self._lock:
                self._goal_reserved = False

    def destroy_node(self) -> None:
        self._action_server.destroy()
        super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = LightSignalWaitActionServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
