import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from r2_odin_interfaces.action import OdinPosePidAlign


class OdinPosePidMockServer(Node):
    def __init__(self):
        super().__init__('odin_pose_pid_mock_server')

        self._server = ActionServer(
            self,
            OdinPosePidAlign,
            '/r2_odin_pose_pid/align',
            self.execute_callback
        )

        self.get_logger().info(
            'OdinPosePid MOCK action server started: /r2_odin_pose_pid/align'
        )

    def execute_callback(self, goal_handle):
        goal = goal_handle.request

        self.get_logger().info(
            f'MOCK received align goal: '
            f'goal_name={goal.goal_name}, '
            f'use_goal_name={goal.use_goal_name}, '
            f'timeout_sec={goal.timeout_sec}'
        )

        feedback = OdinPosePidAlign.Feedback()
        feedback.state = 'MOCK_ALIGNING'
        feedback.error_x = 0.10
        feedback.error_y = 0.05
        feedback.error_yaw = 0.03
        feedback.cmd_vx = 0.0
        feedback.cmd_vy = 0.0
        feedback.cmd_wz = 0.0
        goal_handle.publish_feedback(feedback)

        time.sleep(1.0)

        goal_handle.succeed()

        result = OdinPosePidAlign.Result()
        result.success = True
        result.message = 'MOCK odin pose pid align success'
        result.final_error_x = 0.0
        result.final_error_y = 0.0
        result.final_error_yaw = 0.0

        self.get_logger().info('MOCK align finished with SUCCESS')

        return result


def main(args=None):
    rclpy.init(args=args)
    node = OdinPosePidMockServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()