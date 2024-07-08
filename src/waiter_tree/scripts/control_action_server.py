#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from waiter_tree_interfaces.action import Control  # Import your custom action

class ControlActionServer(Node):
    def __init__(self):
        super().__init__('control_action_server')
        self._action_server = ActionServer(
            self,
            Control,
            'control_action',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal: {goal_handle.request.command}')
        feedback_msg = Control.Feedback()
        feedback_msg.feedback = f'Executing {goal_handle.request.command}'
        goal_handle.publish_feedback(feedback_msg)
        
        # Simulate a long-running task
        for i in range(5):
            self.get_logger().info(f'Executing {goal_handle.request.command}: Step {i + 1}')
            feedback_msg.feedback = f'Executing {goal_handle.request.command}: Step {i + 1}'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Control.Result()
        result.success = True
        self.get_logger().info(f'Goal succeeded: {goal_handle.request.command}')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ControlActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
