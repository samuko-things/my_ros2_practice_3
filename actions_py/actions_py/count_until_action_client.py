#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

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
        self.count_until_action_client_.\
          send_goal_async(goal, feedback_callback=self.goal_feedback_callback).\
          add_done_callback(self.goal_reponse_callback)
        
        #send a cancel request 2 seconds later
        # self.timer_ = self.create_timer(3.0, self.cancel_goal)
    
    def cancel_goal(self):
        self.get_logger().info("send a cancel request")
        self.goal_handle_.cancel_goal_async()
        # self.timer_.cancel()

    def goal_reponse_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal Got rejected")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Result: %d" % (result.reached_number))

    def goal_feedback_callback(self, feedback_msg):
        current_number = feedback_msg.feedback.current_number
        self.get_logger().info("Got Feedback: %d" % (current_number,))

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilActionClientNode()
    node.send_goal(5, 2.0)
    # node.send_goal(10, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
