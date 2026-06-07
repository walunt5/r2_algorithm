import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from techx_r2_arm_interfaces.action import GetHead


class TestGetHeadClient(Node):
    def __init__(self):
        super().__init__("test_get_head_client")
        self.client = ActionClient(self, GetHead, "/r2_arm/get_head")

    def send_goal(self):
        self.get_logger().info("waiting for /r2_arm/get_head action server...")
        self.client.wait_for_server()

        goal = GetHead.Goal()

        self.get_logger().info("send GetHead goal")
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

    node = TestGetHeadClient()

    send_goal_future = node.send_goal()
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