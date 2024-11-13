#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import time
import threading

from my_practice3_interface.action import CountUntil


class CountUntilActionServerNode(Node):
    def __init__(self):
        super().__init__("count_until_action_server")

        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ = []

        self.count_until_action_server_ = ActionServer(
            node=self,
            action_type=CountUntil,
            action_name="count_until",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        
        self.get_logger().info("count_until_action_server node has started")
    
    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Received a Goal")
        
        # validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().info("rejecting the goal")
            return GoalResponse.REJECT #this will abort the goal
        
        self.get_logger().info("accepting the goal")
        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a Cancel Request")
        return CancelResponse.ACCEPT # or REJECT
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
          self.goal_handle_ = goal_handle

        #Get Request from Goal
        target_number = goal_handle.request.target_number
        period_btw_count = goal_handle.request.period_btw_count

        #Execute the action
        self.get_logger().info("Executing the goal")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0
        for i in range(target_number):
            if not goal_handle.is_active:
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result
            
            # run this part if goal cancel command is received
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling the Goal")
                goal_handle.canceled() # .succeed() .abort()
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result
            #---------------------------------------------

            counter += 1
            self.get_logger().info("counter = %d" % (counter,))
            #send feed back
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            #-------------
            time.sleep(period_btw_count)

        #Once done, set goal final state
        goal_handle.succeed()
        # goal_handle.abort()

        # Send the result
        result.reached_number = counter
        self.process_next_goal_in_queue()
        return result
    
    def process_next_goal_in_queue(self):
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_ = None



def main(args=None):
    rclpy.init(args=args)
    node = CountUntilActionServerNode()
    rclpy.spin(node, executor=MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
