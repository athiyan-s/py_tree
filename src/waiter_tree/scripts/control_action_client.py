#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from prototype.action import Control  # Import your custom action

class ControlActionClient(Node):
    def __init__(self):
        super().__init__('control_action_client')
        self._action_client = ActionClient(self, Control, 'control_action')

    def send_goal(self, command):
        goal_msg = Control.Goal()
        goal_msg.command = command

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback}')

def main(args=None):
    rclpy.init(args=args)
    node = ControlActionClient()
    node.send_goal('Control 1')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
