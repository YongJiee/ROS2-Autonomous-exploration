#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class TurtleBotExplorer(Node):
    def __init__(self):
        super().__init__('turtlebot_explorer')
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        self.linear_speed = 0.15
        self.angular_speed = 0.5
        self.safe_distance = 0.35
        self.wall_follow_distance = 0.4
        
        self.scan_data = None
        self.timer = self.create_timer(0.1, self.explore)
        
        self.get_logger().info('TurtleBot Explorer Started!')
        
    def scan_callback(self, msg):
        self.scan_data = np.array(msg.ranges)
        self.scan_data[np.isinf(self.scan_data)] = 10.0
        
    def get_sector_distance(self, start_deg, end_deg):
        if self.scan_data is None:
            return 10.0
        
        num_readings = len(self.scan_data)
        start_idx = int((start_deg / 360.0) * num_readings) % num_readings
        end_idx = int((end_deg / 360.0) * num_readings) % num_readings
        
        if start_idx < end_idx:
            sector = self.scan_data[start_idx:end_idx]
        else:
            sector = np.concatenate([self.scan_data[start_idx:], 
                                    self.scan_data[:end_idx]])
        
        return np.min(sector) if len(sector) > 0 else 10.0
        
    def explore(self):
        if self.scan_data is None:
            return
        
        twist = Twist()
        
        front = self.get_sector_distance(-15, 15)
        front_right = self.get_sector_distance(-60, -30)
        right = self.get_sector_distance(-100, -80)
        front_left = self.get_sector_distance(30, 60)
        
        if front < self.safe_distance:
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            self.get_logger().info('Obstacle ahead - turning left')
            
        elif right > self.wall_follow_distance * 1.5:
            twist.linear.x = self.linear_speed * 0.5
            twist.angular.z = -self.angular_speed * 0.8
            self.get_logger().info('Lost wall - turning right')
            
        elif right < self.wall_follow_distance * 0.7:
            twist.linear.x = self.linear_speed * 0.7
            twist.angular.z = self.angular_speed * 0.3
            self.get_logger().info('Too close to wall - veering left')
            
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
            self.get_logger().info(f'Following wall - F:{front:.2f} R:{right:.2f}')
        
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    explorer = TurtleBotExplorer()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        explorer.get_logger().info('Exploration stopped by user')
    finally:
        twist = Twist()
        explorer.cmd_vel_pub.publish(twist)
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
