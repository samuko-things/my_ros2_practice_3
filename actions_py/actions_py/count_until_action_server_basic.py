#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
import time

from my_practice3_interface.action import CountUntil


class CountUntilActionServerNode(Node):
    def __init__(self):
        super().__init__("count_until_action_server")
        self.count_until_action_server_ = ActionServer(
            node=self,
            action_type=CountUntil,
            action_name="count_until",
            execute_callback=self.execute_callback)
        
        self.get_logger().info("count_until_action_server node has started")

    def execute_callback(self, goal_handle: ServerGoalHandle):
        #Get Request from Goal
        target_number = goal_handle.request.target_number
        period_btw_count = goal_handle.request.period_btw_count

        #Execute the action
        self.get_logger().info("Executing the goal")
        counter = 0
        for i in range(target_number):
            counter += 1
            self.get_logger().info("counter = %d" % (counter,))
            time.sleep(period_btw_count)

        #Once done, set goal final state
        goal_handle.succeed()

        # Send the result
        result = CountUntil.Result()
        result.reached_number = counter
        return result

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilActionServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
