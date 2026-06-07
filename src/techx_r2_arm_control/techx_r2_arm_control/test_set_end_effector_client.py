import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from techx_r2_arm_interfaces.action import SetEndEffector


class TestSetEndEffectorClient(Node):
    def __init__(self):
        super().__init__("test_set_end_effector_client")
        self.client = ActionClient(
            self,
            SetEndEffector,
            "/r2_arm/set_end_effector",
        )

    def send_goal(self, io_id: int, state: int, hold_ms: int):
        self.get_logger().info(
            "waiting for /r2_arm/set_end_effector action server..."
        )
        self.client.wait_for_server()

        goal = SetEndEffector.Goal()
        goal.io_id = io_id
        goal.state = state
        goal.hold_ms = hold_ms

        self.get_logger().info(
            f"send SetEndEffector goal: "
            f"io_id=0x{io_id:02X}, state={state}, hold_ms={hold_ms}"
        )

        return self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"feedback: state={feedback.state}, "
            f"feedback_ok={feedback.feedback_ok}, "
            f"message={feedback.message}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = TestSetEndEffectorClient()

    io_id = 1
    state = 1
    hold_ms = 0

    extra_args = [arg for arg in sys.argv[1:] if not arg.startswith("--")]

    if len(extra_args) >= 1:
        io_id = int(extra_args[0], 0)

    if len(extra_args) >= 2:
        state = int(extra_args[1], 0)

    if len(extra_args) >= 3:
        hold_ms = int(extra_args[2], 0)

    send_goal_future = node.send_goal(io_id, state, hold_ms)
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
        f"feedback_ok={result.feedback_ok}, "
        f"message={result.message}"
    )

    node.destroy_node()
    rclpy.shutdown()