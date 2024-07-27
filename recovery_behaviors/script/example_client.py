#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from dddmr_sys_core.action import RecoveryBehaviors
import time

class ExampleGetPlan(Node):

    def __init__(self):
        super().__init__('example_recovery_behaviors_client')
        self.get_logger().info("Starting example recovery behaviors client")
        self._action_client = ActionClient(self, RecoveryBehaviors, 'recovery_behaviors')

    def send_goal(self):

        self.get_logger().info("Sending Goal")

        goal_msg = RecoveryBehaviors.Goal()

        goal_msg.behavior_name = "rotate_inplace"

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.get_logger().info("Goal sent. Waiting response")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
        #time.sleep(5)
        #future = goal_handle.cancel_goal_async()
        time.sleep(5)
        self.send_goal()
        #time.sleep(5)
        #self.send_goal()        
    def get_result_callback(self, future):
        
        if(future.result().status == 4):
            self.get_logger().info('Goal status: Succeed')
        elif(future.result().status == 6): 
            self.get_logger().info('Goal status: ABORTED')
        else:
            self.get_logger().info('Goal status: ' + str(future.result().status))
        
        result = future.result().result
        #TODO check result().status also
        #self.get_logger().info(result)
        
        

def main(args=None):
    rclpy.init(args=args)
        
    action_client = ExampleGetPlan()
    
    
    #future = action_client.send_goal(False) #Get status without opening
    future = action_client.send_goal()  #Send an 'open' command, and then get status
        
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()