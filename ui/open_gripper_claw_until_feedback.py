#!/usr/bin/env python3
import argparse
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from techx_r2_arm_interfaces.action import ExecuteAction


class ExecuteActionFirstFeedbackClient(Node):
    def __init__(self):
        super().__init__("ui_open_gripper_claw_until_feedback")
        self.client = ActionClient(self, ExecuteAction, "/r2_arm/execute_action")
        self.feedback_event = threading.Event()
        self.feedback_state = None
        self.feedback_message = ""

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.feedback_state = int(feedback.state)
        self.feedback_message = feedback.message
        self.feedback_event.set()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-id", type=int, default=4)
    parser.add_argument("--action-id", type=int, default=1025)
    parser.add_argument("--timeout-ms", type=int, default=3000)
    parser.add_argument("--param", type=int, default=0)
    parser.add_argument("--flags", type=int, default=0)
    parser.add_argument("--server-timeout", type=float, default=1.0)
    parser.add_argument("--goal-response-timeout", type=float, default=1.0)
    parser.add_argument("--feedback-timeout", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = ExecuteActionFirstFeedbackClient()

    try:
        if not node.client.wait_for_server(timeout_sec=args.server_timeout):
            print("action_server_timeout", file=sys.stderr)
            return 2

        goal = ExecuteAction.Goal()
        goal.target_id = args.target_id
        goal.action_id = args.action_id
        goal.timeout_ms = args.timeout_ms
        goal.param = args.param
        goal.flags = args.flags

        send_goal_future = node.client.send_goal_async(
            goal,
            feedback_callback=node.feedback_callback,
        )
        rclpy.spin_until_future_complete(
            node,
            send_goal_future,
            timeout_sec=args.goal_response_timeout,
        )

        if not send_goal_future.done():
            print("goal_response_timeout", file=sys.stderr)
            return 3

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("goal_rejected", file=sys.stderr)
            return 4

        deadline = time.monotonic() + args.feedback_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if node.feedback_event.is_set():
                print(
                    f"first_feedback state={node.feedback_state} "
                    f"message={node.feedback_message}"
                )
                return 0

            remaining = max(deadline - time.monotonic(), 0.0)
            rclpy.spin_once(node, timeout_sec=min(0.05, remaining))

        print("first_feedback_timeout_after_goal_accepted", file=sys.stderr)
        return 5

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
