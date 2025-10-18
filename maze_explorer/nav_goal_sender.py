#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        self.get_logger().info('Nav2 Goal Sender initialized')
    
    def send_goal(self, x, y, z=0.0, orientation_z=0.0, orientation_w=1.0, frame_id='map'):
        """Send a navigation goal with replanning enabled"""
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        
        # Set orientation
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = orientation_z
        goal_msg.pose.pose.orientation.w = orientation_w
        
        # KEY: Set the behavior tree to use
        # This ensures continuous replanning like RViz does
        goal_msg.behavior_tree = ''  # Use default behavior tree
        
        self.get_logger().info(f'Sending goal: x={x}, y={y}, z={z}')
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return send_goal_future
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted! Navigation with replanning active.')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation completed!')
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance remaining: {feedback.distance_remaining:.2f}m',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    
    goal_sender = Nav2GoalSender()
    
    # Send goal
    goal_sender.send_goal(
        x=-5.0,
        y=0.0,
        z=0.0,
        orientation_z=0.0,
        orientation_w=1.0
    )
    
    try:
        rclpy.spin(goal_sender)
    except KeyboardInterrupt:
        pass
    
    goal_sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()