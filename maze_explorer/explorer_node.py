#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np

class CostmapExplorer(Node):
    def __init__(self):
        super().__init__('costmap_explorer')
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, 'local_costmap/costmap', self.costmap_callback, 10)
        
        self.linear_speed = 0.12
        self.angular_speed = 0.5
        
        self.scan_data = None
        self.costmap = None
        self.timer = self.create_timer(0.1, self.explore)
        
        self.get_logger().info('🗺️  Costmap Explorer Started!')
        
    def scan_callback(self, msg):
        self.scan_data = np.array(msg.ranges)
        self.scan_data = np.where(np.isinf(self.scan_data), msg.range_max, self.scan_data)
        
    def costmap_callback(self, msg):
        """Receive and process costmap"""
        self.costmap = msg
        
    def find_exit_from_costmap(self):
        """Analyze costmap to find exits (large clear areas)"""
        if self.costmap is None:
            return None, 0.0
        
        width = self.costmap.info.width
        height = self.costmap.info.height
        data = np.array(self.costmap.data).reshape((height, width))
        
        # Find clear areas (value 0 = free space)
        clear_mask = (data == 0)
        
        # Look for large continuous clear areas
        # This is simplified - you'd want proper connected component analysis
        
        # For now, just find the direction with most clear cells
        center_x = width // 2
        center_y = height // 2
        
        # Check sectors around robot
        sectors = {
            'front': clear_mask[center_y-10:center_y, center_x-5:center_x+5],
            'left': clear_mask[center_y-5:center_y+5, center_x+5:center_x+15],
            'right': clear_mask[center_y-5:center_y+5, center_x-15:center_x-5],
        }
        
        # Count clear cells in each direction
        clear_counts = {k: np.sum(v) for k, v in sectors.items()}
        
        # Find best direction (most clear space)
        best_direction = max(clear_counts, key=clear_counts.get)
        clearance = clear_counts[best_direction]
        
        self.get_logger().info(f'🗺️  Clearance: {clear_counts}')
        
        return best_direction, clearance
        
    def explore(self):
        """Explore using costmap analysis"""
        if self.scan_data is None:
            return
        
        twist = Twist()
        
        # Try to use costmap if available
        if self.costmap is not None:
            direction, clearance = self.find_exit_from_costmap()
            
            # If we found a large clear area (potential exit)
            if clearance > 50:  # Threshold for "large" area
                self.get_logger().info(f'🎯 Large opening detected: {direction}')
                
                if direction == 'front':
                    twist.linear.x = 0.15
                    twist.angular.z = 0.0
                elif direction == 'left':
                    twist.linear.x = 0.05
                    twist.angular.z = 0.5
                elif direction == 'right':
                    twist.linear.x = 0.05
                    twist.angular.z = -0.5
            else:
                # Use simple wall following
                self.wall_follow(twist)
        else:
            # Fallback to LiDAR-only wall following
            self.wall_follow(twist)
        
        self.cmd_vel_pub.publish(twist)
    
    def wall_follow(self, twist):
        """Simple wall following logic"""
        front = np.min(self.scan_data[len(self.scan_data)//3:2*len(self.scan_data)//3])
        right = np.min(self.scan_data[:len(self.scan_data)//4])
        
        if front < 0.3:
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        elif right > 0.5:
            twist.linear.x = 0.1
            twist.angular.z = -0.4
        else:
            twist.linear.x = 0.12
            twist.angular.z = 0.0

def main(args=None):
    rclpy.init(args=args)
    explorer = CostmapExplorer()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()