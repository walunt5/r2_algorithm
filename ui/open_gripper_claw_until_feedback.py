#!/usr/bin/env python3
import argparse
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from techx_r2_arm_interfaces.action import ExecuteAction


DEFAULT_ACTION_NAME = "/r2_arm/execute_action"


class ExecuteActionFirstFeedbackClient(Node):
    def __init__(self, action_name: str):
        super().__init__(
            "ui_open_gripper_claw_until_feedback"
        )

        self.client = ActionClient(
            self,
            ExecuteAction,
            action_name,
        )

        self.feedback_event = threading.Event()
        self.feedback_state = None
        self.feedback_message = ""

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.feedback_state = int(
            getattr(
                feedback,
                "state",
                0,
            )
        )

        self.feedback_message = str(
            getattr(
                feedback,
                "message",
                "",
            )
        )

        self.feedback_event.set()


def non_negative_float(value: str) -> float:
    parsed = float(value)

    if parsed < 0.0:
        raise argparse.ArgumentTypeError(
            "timeout must be non-negative"
        )

    return parsed


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Send a gripper ExecuteAction goal and "
            "return after the first feedback."
        )
    )

    parser.add_argument(
        "--action-name",
        default=DEFAULT_ACTION_NAME,
    )
    parser.add_argument(
        "--target-id",
        type=int,
        default=4,
    )
    parser.add_argument(
        "--action-id",
        type=int,
        default=1025,
    )
    parser.add_argument(
        "--timeout-ms",
        type=int,
        default=3000,
    )
    parser.add_argument(
        "--param",
        type=int,
        default=0,
    )
    parser.add_argument(
        "--flags",
        type=int,
        default=0,
    )
    parser.add_argument(
        "--server-timeout",
        type=non_negative_float,
        default=1.0,
    )
    parser.add_argument(
        "--goal-response-timeout",
        type=non_negative_float,
        default=1.0,
    )
    parser.add_argument(
        "--feedback-timeout",
        type=non_negative_float,
        default=1.0,
    )

    return parser.parse_args()


def spin_until_future_or_timeout(
    node: Node,
    future,
    timeout_sec: float,
) -> bool:
    deadline = (
        time.monotonic()
        + timeout_sec
    )

    while rclpy.ok():
        if future.done():
            return True

        remaining = (
            deadline - time.monotonic()
        )

        if remaining <= 0.0:
            return future.done()

        rclpy.spin_once(
            node,
            timeout_sec=min(
                0.05,
                remaining,
            ),
        )

    return future.done()


def main() -> int:
    args = parse_args()

    if args.timeout_ms < 0:
        print(
            "timeout_ms_must_be_non_negative",
            file=sys.stderr,
        )
        return 1

    rclpy.init()

    node = ExecuteActionFirstFeedbackClient(
        args.action_name
    )

    try:
        if not node.client.wait_for_server(
            timeout_sec=args.server_timeout
        ):
            print(
                "action_server_timeout "
                f"action_name={args.action_name}",
                file=sys.stderr,
            )
            return 2

        goal = ExecuteAction.Goal()
        goal.target_id = args.target_id
        goal.action_id = args.action_id
        goal.timeout_ms = args.timeout_ms
        goal.param = args.param
        goal.flags = args.flags

        send_goal_future = (
            node.client.send_goal_async(
                goal,
                feedback_callback=(
                    node.feedback_callback
                ),
            )
        )

        if not spin_until_future_or_timeout(
            node,
            send_goal_future,
            args.goal_response_timeout,
        ):
            print(
                "goal_response_timeout",
                file=sys.stderr,
            )
            return 3

        try:
            goal_handle = (
                send_goal_future.result()
            )
        except Exception as error:
            print(
                "goal_response_exception "
                f"error={error}",
                file=sys.stderr,
            )
            return 6

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            print(
                "goal_rejected",
                file=sys.stderr,
            )
            return 4

        deadline = (
            time.monotonic()
            + args.feedback_timeout
        )

        while rclpy.ok():
            if node.feedback_event.is_set():
                print(
                    "first_feedback "
                    f"state={node.feedback_state} "
                    f"message={node.feedback_message}"
                )
                return 0

            remaining = (
                deadline - time.monotonic()
            )

            if remaining <= 0.0:
                break

            rclpy.spin_once(
                node,
                timeout_sec=min(
                    0.05,
                    remaining,
                ),
            )

        # 目标已经被服务器接受，不主动取消。
        # 即使脚本退出，夹爪动作仍可由服务器继续执行。
        print(
            "goal_accepted_but_feedback_timeout "
            "action_continues",
            file=sys.stderr,
        )
        return 5

    except KeyboardInterrupt:
        print(
            "interrupted",
            file=sys.stderr,
        )
        return 130

    except Exception as error:
        print(
            "unexpected_exception "
            f"error={error}",
            file=sys.stderr,
        )
        return 10

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())