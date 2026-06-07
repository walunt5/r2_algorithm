import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from techx_r2_arm_interfaces.action import ExecuteAction


class TestExecuteActionClient(Node):
    def __init__(self):
        super().__init__("test_execute_action_client")
        self.client = ActionClient(
            self,
            ExecuteAction,
            "/r2_arm/execute_action",
        )

    def send_goal(
        self,
        target_id: int,
        action_id: int,
        timeout_ms: int,
        param: int,
        flags: int,
    ):
        self.get_logger().info(
            "waiting for /r2_arm/execute_action action server..."
        )
        self.client.wait_for_server()

        goal = ExecuteAction.Goal()
        goal.target_id = target_id
        goal.action_id = action_id
        goal.timeout_ms = timeout_ms
        goal.param = param
        goal.flags = flags

        self.get_logger().info(
            f"send ExecuteAction goal: "
            f"target_id=0x{target_id:02X}, "
            f"action_id=0x{action_id:04X}, "
            f"timeout_ms={timeout_ms}, param={param}, flags=0x{flags:02X}"
        )

        return self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"feedback: state={feedback.state}, message={feedback.message}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = TestExecuteActionClient()

    target_id = 1
    action_id = 0x0101
    timeout_ms = 3000
    param = 0
    flags = 0

    extra_args = [arg for arg in sys.argv[1:] if not arg.startswith("--")]

    if len(extra_args) >= 1:
        target_id = int(extra_args[0], 0)

    if len(extra_args) >= 2:
        action_id = int(extra_args[1], 0)

    if len(extra_args) >= 3:
        timeout_ms = int(extra_args[2], 0)

    if len(extra_args) >= 4:
        param = int(extra_args[3], 0)

    if len(extra_args) >= 5:
        flags = int(extra_args[4], 0)

    send_goal_future = node.send_goal(
        target_id,
        action_id,
        timeout_ms,
        param,
        flags,
    )
    rclpy.spin_until_future_complete(node, send_goal_future)

    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        node.get_logger().error("goal rejected")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("goal accepted, waiting result...")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result().result

    node.get_logger().info(
        f"result: success={result.success}, "
        f"final_state={result.final_state}, "
        f"error_code=0x{result.error_code:04X}, "
        f"message={result.message}"
    )

    node.destroy_node()
    rclpy.shutdown()