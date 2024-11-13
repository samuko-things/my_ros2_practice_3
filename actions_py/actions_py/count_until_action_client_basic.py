#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from my_practice3_interface.action import CountUntil


class CountUntilActionClientNode(Node):
    def __init__(self):
        super().__init__("count_until_action_client")
        self.count_until_action_client_ = ActionClient(
            self,
            CountUntil,
            "count_until"
        )
        
        self.get_logger().info("count_until_action_client node has started")

    def send_goal(self, target_number, period_btw_count):
        #wait for the action server (with time out if necessary)
        self.count_until_action_client_.wait_for_server()

        #create goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period_btw_count = period_btw_count

        #send goal
        self.get_logger().info("Sending goal")
        self.count_until_action_client_.send_goal_async(goal).add_done_callback(self.goal_reponse_callback)

    def goal_reponse_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: %d" % (result.reached_number))


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilActionClientNode()
    node.send_goal(6, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
