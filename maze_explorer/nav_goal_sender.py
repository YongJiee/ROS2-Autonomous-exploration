#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import math
import numpy as np

class OpportunisticNavigator(Node):
    def __init__(self):
        super().__init__('opportunistic_navigator')
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for 2D Nav Goal
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        
        # Control parameters
        self.rotation_speed = 0.5        
        self.angle_tolerance = 0.05
        self.goal_distance = 2.5
        
        # Initial goal (you can change this)
        self.initial_goal_x = 5.0
        self.initial_goal_y = 0.0
        
        # INF detection parameters
        self.inf_detection_time = 5.0  # Must see INF for 2 seconds
        self.inf_first_seen = None
        self.inf_direction_detected = None
        
        # Exit detection parameters (270° to 90° = entire front)
        self.exit_sector_start = 270  # Right side
        self.exit_sector_end = 90     # Left side (wraps through 0°)
        self.exit_detection_time = 2.0  # Must see exit for 2 seconds
        self.exit_first_seen = None
        
        # State
        self.target_angle = None
        self.is_rotating = False
        self.is_navigating = False
        self.initial_goal_sent = False
        self.inf_triggered = False
        self.maze_complete = False
        self.current_goal_handle = None
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Opportunistic Navigator Started!')
        self.get_logger().info(f'Initial goal: ({self.initial_goal_x}, {self.initial_goal_y})')
        self.get_logger().info(f'Will switch to INF if detected for {self.inf_detection_time}s')
        self.get_logger().info(f'Exit detection: INF from {self.exit_sector_start}° to {self.exit_sector_end}° (front hemisphere)')
        self.get_logger().info('='*60)
    
    def odom_callback(self, msg):
        """Get current robot pose from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if not self.odom_received:
            self.get_logger().info(f'✓ Odometry received!')
            self.odom_received = True
            
            # Send initial goal after getting odometry
            self.send_initial_goal()
    
    def send_initial_goal(self):
        """Send the initial navigation goal"""
        if self.initial_goal_sent:
            return
        
        self.get_logger().info('━'*60)
        self.get_logger().info('🎯 SENDING INITIAL GOAL')
        self.get_logger().info(f'   Goal: ({self.initial_goal_x}, {self.initial_goal_y})')
        self.get_logger().info('━'*60)
        
        self.send_nav_goal(self.initial_goal_x, self.initial_goal_y, 0.0)
        self.initial_goal_sent = True
    
    def is_in_front_hemisphere(self, angle_deg):
        """Check if angle is in front hemisphere (270° to 90° wrapping through 0°)"""
        angle_deg = angle_deg % 360
        
        # From 270° to 360° OR from 0° to 90°
        return angle_deg >= self.exit_sector_start or angle_deg <= self.exit_sector_end
    
    def scan_callback(self, msg):
        """Monitor laser scan for INF detection and exit detection"""
        
        if not self.odom_received or self.maze_complete:
            return
        
        ranges = np.array(msg.ranges)
        inf_indices = np.where(np.isinf(ranges))[0]
        
        if len(inf_indices) == 0:
            # No INF - reset all timers
            if self.inf_first_seen is not None or self.exit_first_seen is not None:
                self.get_logger().info('INF lost, resetting timers')
            self.inf_first_seen = None
            self.inf_direction_detected = None
            self.exit_first_seen = None
            return
        
        # Check for EXIT condition (INF in entire front hemisphere)
        exit_inf_indices = []
        for idx in inf_indices:
            angle = msg.angle_min + (idx * msg.angle_increment)
            angle_deg = math.degrees(angle) % 360
            if self.is_in_front_hemisphere(angle_deg):
                exit_inf_indices.append(idx)
        
        current_time = self.get_clock().now()
        
        # Check if ALL front hemisphere has INF (robot is out of maze)
        if len(exit_inf_indices) > 0:
            # Count how many beams are in front hemisphere
            total_front_beams = 0
            for i in range(len(ranges)):
                angle = msg.angle_min + (i * msg.angle_increment)
                angle_deg = math.degrees(angle) % 360
                if self.is_in_front_hemisphere(angle_deg):
                    total_front_beams += 1
            
            # Calculate percentage of front that is INF
            front_inf_percentage = (len(exit_inf_indices) / total_front_beams) * 100
            
            self.get_logger().info(
                f'🚪 Front INF: {len(exit_inf_indices)}/{total_front_beams} beams ({front_inf_percentage:.1f}%)',
                throttle_duration_sec=0.5
            )
            
            # If majority of front is INF, trigger exit detection
            if front_inf_percentage > 95:  # 80% of front is clear
                if self.exit_first_seen is None:
                    self.exit_first_seen = current_time
                    self.get_logger().info(f'🚪 EXIT DETECTED! Front is {front_inf_percentage:.1f}% clear!')
                    self.get_logger().info(f'   Starting exit timer...')
                    return
                
                # Check how long we've seen exit
                exit_elapsed = (current_time - self.exit_first_seen).nanoseconds / 1e9
                
                if exit_elapsed < self.exit_detection_time:
                    self.get_logger().info(
                        f'🚪 EXIT stable for {exit_elapsed:.1f}s / {self.exit_detection_time}s',
                        throttle_duration_sec=0.5
                    )
                    return
                
                # EXIT DETECTED FOR LONG ENOUGH!
                self.trigger_maze_complete()
                return
            else:
                # Not enough front clearance
                if self.exit_first_seen is not None:
                    self.get_logger().info('Exit condition lost (not enough front clearance)')
                self.exit_first_seen = None
        else:
            # No front INF
            if self.exit_first_seen is not None:
                self.get_logger().info('Exit INF lost')
            self.exit_first_seen = None
        
        # Don't process general INF if already triggered or rotating
        if self.inf_triggered or self.is_rotating:
            return
        
        # General INF detection (for opportunistic navigation)
        inf_idx = inf_indices[len(inf_indices)//2]
        laser_angle = msg.angle_min + (inf_idx * msg.angle_increment)
        laser_angle_deg = math.degrees(laser_angle) % 360
        
        if self.inf_first_seen is None:
            # First time seeing INF
            self.inf_first_seen = current_time
            self.inf_direction_detected = laser_angle
            self.get_logger().info(f'⏱️ INF detected at {laser_angle_deg:.1f}°, starting timer...')
            return
        
        # Check how long we've been seeing INF
        elapsed = (current_time - self.inf_first_seen).nanoseconds / 1e9
        
        if elapsed < self.inf_detection_time:
            # Still counting
            self.get_logger().info(
                f'⏱️ INF stable for {elapsed:.1f}s / {self.inf_detection_time}s',
                throttle_duration_sec=0.5
            )
            return
        
        # INF detected for long enough! Trigger!
        self.get_logger().info('━'*60)
        self.get_logger().info('🚨 INF DETECTED FOR REQUIRED TIME!')
        self.get_logger().info(f'   Found {len(inf_indices)} INF beams')
        self.get_logger().info(f'   Direction: {laser_angle_deg:.1f}°')
        self.get_logger().info('   Canceling current goal...')
        
        # Cancel current navigation
        if self.is_navigating and self.current_goal_handle is not None:
            self.get_logger().info('   Sending cancel request to Nav2...')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        
        # Mark as triggered
        self.inf_triggered = True
        
        # Start rotation to face INF
        angle_diff = self.normalize_angle(self.inf_direction_detected)
        
        if abs(angle_diff) < self.angle_tolerance:
            self.get_logger().info('   Already facing INF, sending goal...')
            self.send_nav_goal_to_inf(0.0)
            return
        
        target_global_angle = self.current_yaw + self.inf_direction_detected
        target_global_angle = self.normalize_angle(target_global_angle)
        
        self.get_logger().info(f'   Rotating {math.degrees(angle_diff):.1f}° to face INF')
        self.get_logger().info('━'*60)
        
        self.target_angle = target_global_angle
        self.is_rotating = True
    
    def trigger_maze_complete(self):
        """Exit detected - maze complete!"""
        self.get_logger().info('━'*60)
        self.get_logger().info('━'*60)
        self.get_logger().info('🎉🎉🎉 ROBOT OUT OF MAZE! 🎉🎉🎉')
        self.get_logger().info('   Front hemisphere is clear (270° to 90°)')
        self.get_logger().info('   Canceling navigation...')
        
        # Cancel current navigation
        if self.is_navigating and self.current_goal_handle is not None:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        
        # Stop robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Mark maze complete
        self.maze_complete = True
        
        self.get_logger().info('')
        self.get_logger().info('🏁 MAZE COMPLETE - MISSION SUCCESS! 🏁')
        self.get_logger().info(f'   Final position: ({self.current_x:.2f}, {self.current_y:.2f})')
        self.get_logger().info('')
        self.get_logger().info('━'*60)
        self.get_logger().info('━'*60)
    
    def cancel_done_callback(self, future):
        """Handle goal cancellation"""
        try:
            cancel_response = future.result()
            if cancel_response.goals_canceling:
                self.get_logger().info('✅ Goal cancellation accepted')
            else:
                self.get_logger().warn('⚠️ Goal cancellation not accepted')
        except Exception as e:
            self.get_logger().error(f'❌ Cancel error: {e}')
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def control_loop(self):
        """Control loop to rotate robot to target angle"""
        
        if self.maze_complete:
            return
        
        if not self.is_rotating or self.target_angle is None:
            return
        
        angle_diff = self.normalize_angle(self.target_angle - self.current_yaw)
        
        if abs(angle_diff) < self.angle_tolerance:
            self.get_logger().info('✅ ROTATION COMPLETE!')
            
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.is_rotating = False
            self.target_angle = None
            
            # Send goal toward INF
            self.send_nav_goal_to_inf(0.0)
            return
        
        twist = Twist()
        
        if angle_diff > 0:
            twist.angular.z = self.rotation_speed
            direction = "⬅️"
        else:
            twist.angular.z = -self.rotation_speed
            direction = "➡️"
        
        if abs(angle_diff) < 0.2:
            twist.angular.z *= 0.5
        
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(
            f'{direction} {math.degrees(angle_diff):+.1f}°',
            throttle_duration_sec=0.5
        )
    
    def send_nav_goal_to_inf(self, laser_angle):
        """Send goal toward INF direction"""
        global_angle = self.current_yaw + laser_angle
        goal_x = self.current_x + self.goal_distance * math.cos(global_angle)
        goal_y = self.current_y + self.goal_distance * math.sin(global_angle)
        
        self.get_logger().info('━'*60)
        self.get_logger().info('🎯 SENDING GOAL TOWARD INF')
        self.get_logger().info(f'   Goal: ({goal_x:.2f}, {goal_y:.2f})')
        self.get_logger().info('━'*60)
        
        self.send_nav_goal(goal_x, goal_y, global_angle)
    
    def send_nav_goal(self, goal_x, goal_y, goal_yaw):
        """Send navigation goal"""
        
        self.is_navigating = True
        
        # Create action goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = 0.0
        
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
        
        # Publish to /goal_pose for visualization
        goal_pose = PoseStamped()
        goal_pose.header = goal_msg.pose.header
        goal_pose.pose = goal_msg.pose.pose
        self.goal_pub.publish(goal_pose)
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Nav2 action server not available!')
            self.is_navigating = False
            return
        
        # Send goal
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self.send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Handle goal acceptance"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ GOAL REJECTED!')
            self.is_navigating = False
            return
        
        self.get_logger().info('✅ GOAL ACCEPTED - Navigating...')
        
        # Store goal handle for cancellation
        self.current_goal_handle = goal_handle
        
        # Wait for result
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.nav_result_callback)
    
    def nav_result_callback(self, future):
        """Handle navigation completion"""
        result = future.result()
        status = result.status
        
        self.get_logger().info('━'*60)
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('🎉 NAVIGATION SUCCEEDED!')
            self.get_logger().info(f'   Reached: ({self.current_x:.2f}, {self.current_y:.2f})')
            
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('⚠️ NAVIGATION ABORTED')
            
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('⚠️ NAVIGATION CANCELED')
        
        self.get_logger().info('━'*60)
        
        self.is_navigating = False
        self.current_goal_handle = None
    
    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        
        self.get_logger().info(
            f'🚗 Navigating... ({current_pose.position.x:.2f}, {current_pose.position.y:.2f})',
            throttle_duration_sec=1.0
        )
    
    def stop_robot(self):
        """Emergency stop"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = OpportunisticNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()