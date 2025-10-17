#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.ndimage import binary_dilation, label
import math

class MazeExplorer(Node):
    def __init__(self):
        super().__init__('maze_explorer')
        
        # Parameters for grid maze
        self.min_frontier_size = 3
        self.grid_size = 0.5  # Match maze grid
        
        # Subscribe to map and laser scan
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.map_data = None
        self.scan_data = None
        self.exploring = False
        self.failed_goals = []  # Track failed navigation attempts
        
        self.get_logger().info('Maze Explorer initialized for grid maze')
        
        # Timer for exploration
        self.timer = self.create_timer(5.0, self.explore_callback)
    
    def map_callback(self, msg):
        self.map_data = msg
    
    def scan_callback(self, msg):
        self.scan_data = msg
    
    def find_frontiers(self):
        """Find frontier cells for grid maze"""
        if self.map_data is None:
            return []
        
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin
        data = np.array(self.map_data.data).reshape((height, width))
        
        # Find free, unknown spaces
        free_space = (data == 0)
        unknown = (data == -1)
        
        # Frontiers: unknown cells adjacent to free space
        dilated_free = binary_dilation(free_space, iterations=1)
        frontiers = unknown & dilated_free
        
        # Cluster frontiers
        labeled_frontiers, num_features = label(frontiers)
        
        frontier_points = []
        for i in range(1, num_features + 1):
            frontier_cluster = (labeled_frontiers == i)
            cluster_size = np.sum(frontier_cluster)
            
            if cluster_size < self.min_frontier_size:
                continue
            
            # Get centroid
            coords = np.argwhere(frontier_cluster)
            centroid_y = int(np.mean(coords[:, 0]))
            centroid_x = int(np.mean(coords[:, 1]))
            
            # Convert to world coordinates
            world_x = origin.position.x + centroid_x * resolution
            world_y = origin.position.y + centroid_y * resolution
            
            # Check if this goal has failed before
            failed = False
            for fx, fy in self.failed_goals:
                if math.sqrt((world_x - fx)**2 + (world_y - fy)**2) < 0.3:
                    failed = True
                    break
            
            if not failed:
                # Snap to grid for structured maze
                grid_x = round(world_x / self.grid_size) * self.grid_size
                grid_y = round(world_y / self.grid_size) * self.grid_size
                
                frontier_points.append((grid_x, grid_y, cluster_size))
        
        # Sort by size
        frontier_points.sort(key=lambda x: x[2], reverse=True)
        
        return frontier_points
    
    def explore_callback(self):
        """Main exploration loop"""
        if self.exploring:
            return
        
        frontiers = self.find_frontiers()
        
        if not frontiers:
            self.get_logger().info('✅ Exploration complete!')
            return
        
        # Try frontiers in order
        for target in frontiers[:5]:
            self.get_logger().info(f'Attempting frontier: ({target[0]:.2f}, {target[1]:.2f})')
            if self.send_navigation_goal(target[0], target[1]):
                break
    
    def send_navigation_goal(self, x, y):
        """Send navigation goal"""
        self.exploring = True
        self.current_goal = (x, y)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        """Handle goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Goal rejected')
            self.failed_goals.append(self.current_goal)
            self.exploring = False
            return
        
        self.get_logger().info('✓ Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle goal completion"""
        result = future.result()
        self.exploring = False
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('✓ Reached frontier!')
        else:
            self.get_logger().warn(f'⚠ Goal failed: {result.status}')
            self.failed_goals.append(self.current_goal)
            # Limit failed goals list
            if len(self.failed_goals) > 20:
                self.failed_goals.pop(0)


def main(args=None):
    rclpy.init(args=args)
    explorer = MazeExplorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()