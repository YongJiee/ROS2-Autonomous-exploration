#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class TightMazeExplorer(Node):
    def __init__(self):
        super().__init__('tight_maze_explorer')
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        
        self.scan_data = None
        self.timer = self.create_timer(0.2, self.navigate)
        
        self.get_logger().info('🏃 Tight Maze Explorer - For 0.5m corridors!')
        
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), 10.0, ranges)
        ranges = np.where(np.isnan(ranges), 0.0, ranges)
        self.scan_data = ranges
        
    def navigate(self):
        if self.scan_data is None:
            return
        
        n = len(self.scan_data)
        
        # Get distances
        front = np.min(self.scan_data[int(n*0.45):int(n*0.55)])  # Narrow front
        right = np.min(self.scan_data[int(n*0.0):int(n*0.20)])
        left = np.min(self.scan_data[int(n*0.80):int(n*1.0)])
        
        twist = Twist()
        
        # CRITICAL: In 0.5m maze, front is normally 0.25-0.35m
        # Only stop if REALLY about to hit (< 0.22m)
        COLLISION_THRESHOLD = 0.05  # Was 0.28 - TOO HIGH!
        
        self.get_logger().info(f'F:{front:.2f} R:{right:.2f} L:{left:.2f}')
        
        if front < COLLISION_THRESHOLD:
            # Actually about to collide - turn
            twist.linear.x = 0.0
            twist.angular.z = 0.3 if left > right else -0.3
            self.get_logger().warn(f'🚨 COLLISION RISK! Turning')
            
        elif right > 0.55:
            # Lost wall
            twist.linear.x = 0.08
            twist.angular.z = -0.25
            self.get_logger().info('↪️  Finding wall')
            
        elif right < 0.25:
            # Too close to wall
            twist.linear.x = 0.08
            twist.angular.z = 0.2
            self.get_logger().info('↖️  Away from wall')
            
        else:
            # GO FORWARD! Even if front is 0.25-0.35m (normal in 0.5m maze)
            twist.linear.x = 0.10
            twist.angular.z = 0.0
            self.get_logger().info('✅ FORWARD!')
        
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TightMazeExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()