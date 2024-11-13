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

        self.count_until_action_server_ = ActionServer(
            node=self,
            action_type=CountUntil,
            action_name="count_until",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        
        self.get_logger().info("count_until_action_server node has started")
    
    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Received a Goal")

        # # policy: refuse new goal if current goal is still active
        # with self.goal_lock_:
        #   if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #       self.get_logger().info("A Goal is already active, rejecting new goal")
        #       return GoalResponse.REJECT #this will abort the goal
        # # -------------------------------------------------------
        
        # validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().info("rejecting the goal")
            return GoalResponse.REJECT #this will abort the goal
        
        # # policy: preempt existing goal when receiving new goal
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("Abort current goal and accep new goal")
        #         self.goal_handle_.abort()
        # # -----------------------------------------------------
        
        self.get_logger().info("accepting the goal")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a Cancel Request")
        return CancelResponse.ACCEPT # or REJECT
    
    def execute_callback(self, goal_handle: ServerGoalHandle):
        # # policy: refuse new goal if current goal is still active
        # # policy: preempt existing goal when receiving new goal
        # with self.goal_lock_:
        #   self.goal_handle_ = goal_handle
        # #------------------------------------------------------

        #Get Request from Goal
        target_number = goal_handle.request.target_number
        period_btw_count = goal_handle.request.period_btw_count

        #Execute the action
        self.get_logger().info("Executing the goal")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0
        for i in range(target_number):
            # # policy: preempt existing goal when receiving new goal
            # if not goal_handle.is_active:
            #     result.reached_number = counter
            #     return result
            # # ------------------------------------------------
            
            # run this part if goal cancel command is received
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling the Goal")
                goal_handle.canceled() # .succeed() .abort()
                result.reached_number = counter
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
        return result

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilActionServerNode()
    rclpy.spin(node, executor=MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
