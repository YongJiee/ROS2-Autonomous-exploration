#!/usr/bin/env python3
"""
Minimal Frontier + Open-Space Explorer for TurtleBot3 Burger
With comprehensive recovery system - NON-BLOCKING VERSION
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from sensor_msgs.msg import LaserScan

import numpy as np
from scipy.ndimage import binary_dilation
import cv2
import math
import time
from tf_transformations import euler_from_quaternion
from enum import Enum


class State(Enum):
    IDLE = 0
    NAVIGATING = 1
    PAUSING = 2
    RECOVERY_STOPPING = 3
    RECOVERY_FINDING_SPACE = 4
    RECOVERY_ROTATING = 5
    RECOVERY_MOVING = 6
    RECOVERY_CLEARING = 7
    ROTATING = 8
    ALIGNING = 9
    CRAWLING = 10
    SCANNING_FOR_PATH = 11


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Parameters
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('open_space_distance_threshold', float('inf'))
        self.declare_parameter('forward_speed', 0.12)
        self.declare_parameter('rotation_speed', 0.20)
        self.declare_parameter('rotation_sign', 1.0)
        self.declare_parameter('crawl_cycle_sec', 0.15)
        self.declare_parameter('goal_reuse_threshold', 0.20)
        self.declare_parameter('nav_timeout', 30.0)

        # Pause timings
        self.declare_parameter('pause_after_goal', 2.0)
        self.declare_parameter('pause_after_crawl', 1.5)
        self.declare_parameter('pause_after_align', 1.0)
        self.declare_parameter('pause_after_recovery', 3.0)

        # Load parameters
        self.pause_after_goal = float(self.get_parameter('pause_after_goal').value)
        self.pause_after_crawl = float(self.get_parameter('pause_after_crawl').value)
        self.pause_after_align = float(self.get_parameter('pause_after_align').value)
        self.pause_after_recovery = float(self.get_parameter('pause_after_recovery').value)

        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.open_space_distance_threshold = float(self.get_parameter('open_space_distance_threshold').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.rotation_speed = float(self.get_parameter('rotation_speed').value)
        self.rotation_sign = float(self.get_parameter('rotation_sign').value)
        self.crawl_cycle_sec = float(self.get_parameter('crawl_cycle_sec').value)
        self.goal_reuse_threshold = float(self.get_parameter('goal_reuse_threshold').value)
        self.nav_timeout = float(self.get_parameter('nav_timeout').value)

        # State machine
        self.state = State.IDLE
        self.pause_start_time = None
        self.pause_duration = 0.0
        self.state_start_time = None
        
        # State-specific data
        self.rotation_target_yaw = None
        self.rotation_start_yaw = None
        self.scan_step_count = 0
        self.scan_rotation_direction = 1  # 1 = left/CCW, -1 = right/CW
        self.scan_max_rotations = 18  # Default 90°, increased to 36 (180°) for back
        self.scan_tried_both_directions = False  # Track if we've tried both directions
        self.crawl_start_time = None
        self.recovery_target_angle = None
        self.recovery_max_range = None

        # Regular state
        self.navigation_active = False
        self.map_data = None
        self.map_info = None
        self.current_pose = None
        self.scan_data = None

        # Goal tracking
        self.attempted_goals = []
        self.blacklist = []
        self.goal_start_time = None
        self.consecutive_failures = 0

        # ROS interfaces
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Main control loop - runs frequently
        self.create_timer(0.05, self.control_loop)  # 20Hz control loop
        self.create_timer(1.5, self.exploration_decision_loop)
        self.create_timer(2.0, self.check_navigation_timeout)
        self.create_timer(5.0, self.debug_front_rays)

        self.last_open_space_time = None
        self.last_open_space_angle = None
        self.open_space_cooldown = 5.0
        self.open_space_failed_time = None  # Track when open space scanning failed
        self.open_space_retry_cooldown = 10.0  # Don't retry for 10 seconds after failure
        
        self.get_logger().info("🚀 Frontier Explorer ready (non-blocking)!")

    # Callbacks
    def amcl_callback(self, msg):
        self.current_pose = msg.pose.pose

    def odom_callback(self, msg):
        if self.current_pose is None:
            self.current_pose = msg.pose.pose

    def map_callback(self, msg):
        self.map_info = msg.info
        try:
            self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        except Exception as e:
            self.get_logger().warn(f"Map reshape failed: {e}")

    def scan_callback(self, msg):
        self.scan_data = msg

    def check_navigation_timeout(self):
        """Check if navigation has been active too long without completing."""
        if not self.navigation_active or self.goal_start_time is None:
            return
        
        elapsed = time.time() - self.goal_start_time
        if elapsed > self.nav_timeout:
            self.get_logger().warn(f"⏰ Navigation timeout after {elapsed:.1f}s - triggering recovery")
            self.navigation_active = False
            self.goal_start_time = None
            self.start_recovery()

    # Utility
    def get_robot_yaw(self):
        if self.current_pose is None:
            return 0.0
        q = self.current_pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def map_to_world(self, mx, my):
        wx = mx * self.map_info.resolution + self.map_info.origin.position.x
        wy = my * self.map_info.resolution + self.map_info.origin.position.y
        return wx, wy

    def is_point_free(self, xw, yw):
        if self.map_info is None or self.map_data is None:
            return True
        res = self.map_info.resolution
        ox, oy = self.map_info.origin.position.x, self.map_info.origin.position.y
        mx = int((xw - ox) / res)
        my = int((yw - oy) / res)
        if not (0 <= mx < self.map_info.width and 0 <= my < self.map_info.height):
            return False
        clearance = 2
        y0 = max(0, my - clearance)
        y1 = min(self.map_info.height, my + clearance + 1)
        x0 = max(0, mx - clearance)
        x1 = min(self.map_info.width, mx + clearance + 1)
        return not np.any(self.map_data[y0:y1, x0:x1] > 50)

    def _ranges_numpy(self):
        if self.scan_data is None:
            return None
        rm, rM = self.scan_data.range_min, self.scan_data.range_max
        arr = np.array(self.scan_data.ranges, dtype=np.float32)
        arr = np.nan_to_num(arr, nan=0.0, posinf=np.inf, neginf=0.0)
        return np.clip(arr, rm, rM)

    # State transitions
    def transition_to_pause(self, duration, next_state_after_pause):
        """Transition to PAUSING state for a specific duration."""
        self.state = State.PAUSING
        self.pause_start_time = time.time()
        self.pause_duration = duration
        self.next_state_after_pause = next_state_after_pause
        self.get_logger().info(f"⏸️ Pausing for {duration:.1f}s")

    def transition_to_idle(self):
        """Transition back to IDLE state."""
        self.state = State.IDLE
        self.cmd_vel_pub.publish(Twist())

    # Main control loop - handles all state transitions
    def control_loop(self):
        """Main non-blocking control loop - runs at 20Hz."""
        
        if self.state == State.PAUSING:
            elapsed = time.time() - self.pause_start_time
            if elapsed >= self.pause_duration:
                self.state = self.next_state_after_pause
                self.get_logger().info(f"✅ Pause complete, transitioning to {self.state.name}")
        
        elif self.state == State.ROTATING:
            self.handle_rotation()
        
        elif self.state == State.SCANNING_FOR_PATH:
            self.handle_path_scanning()
        
        elif self.state == State.CRAWLING:
            self.handle_crawling()
        
        elif self.state == State.RECOVERY_STOPPING:
            self.handle_recovery_stopping()
        
        elif self.state == State.RECOVERY_ROTATING:
            self.handle_recovery_rotating()
        
        elif self.state == State.RECOVERY_MOVING:
            self.handle_recovery_moving()

    # Recovery - now broken into non-blocking steps
    def start_recovery(self):
        """Begin recovery sequence."""
        if self.state != State.IDLE and self.state != State.NAVIGATING:
            self.get_logger().warn("⚠️ Already in recovery - skipping")
            return
        
        self.consecutive_failures += 1
        self.get_logger().warn(f"🛟 Starting recovery (failure #{self.consecutive_failures})...")
        
        self.cmd_vel_pub.publish(Twist())
        self.state = State.RECOVERY_STOPPING
        self.state_start_time = time.time()

    def handle_recovery_stopping(self):
        """Wait 0.5s after stopping."""
        if time.time() - self.state_start_time >= 0.5:
            # Find open space
            ranges = self._ranges_numpy()
            if ranges is None or len(ranges) == 0:
                self.get_logger().warn("⚠️ No LiDAR - aborting recovery")
                self.clear_costmaps_async()
                self.transition_to_idle()
                return
            
            max_idx = int(np.argmax(ranges))
            max_range = ranges[max_idx]
            
            if max_range < 0.5:
                self.get_logger().warn("🚫 No clear space - only clearing costmaps")
                self.clear_costmaps_async()
                self.transition_to_idle()
                return
            
            angle_min = self.scan_data.angle_min
            angle_increment = self.scan_data.angle_increment
            target_angle = angle_min + (max_idx * angle_increment)
            
            self.get_logger().info(f"🎯 Open space at {math.degrees(target_angle):.1f}° ({max_range:.2f}m)")
            
            # Store for rotation
            self.recovery_target_angle = target_angle
            self.recovery_max_range = max_range
            
            # Start rotation
            self.start_rotation_relative(target_angle, State.RECOVERY_MOVING)

    def handle_recovery_rotating(self):
        """Recovery rotation is handled by handle_rotation()."""
        pass

    def handle_recovery_moving(self):
        """Move forward if we have enough space."""
        if not hasattr(self, 'recovery_move_start'):
            # Just started moving
            if self.recovery_max_range > 0.8:
                self.get_logger().info("⬆️ Moving forward 0.5m")
                self.recovery_move_start = time.time()
                twist = Twist()
                twist.linear.x = 0.15
                self.cmd_vel_pub.publish(twist)
            else:
                # Skip moving, go straight to clearing
                self.finish_recovery()
                return
        
        # Check if we've moved long enough
        if time.time() - self.recovery_move_start >= 3.0:
            self.cmd_vel_pub.publish(Twist())
            del self.recovery_move_start
            self.finish_recovery()

    def finish_recovery(self):
        """Complete recovery sequence."""
        self.clear_costmaps_async()
        
        if self.consecutive_failures >= 3 and self.current_pose:
            px, py = self.current_pose.position.x, self.current_pose.position.y
            self.blacklist.append((px, py))
            self.get_logger().warn(f"🚫 Blacklisted ({px:.2f}, {py:.2f}) after {self.consecutive_failures} failures")
        
        self.get_logger().info(f"✅ Recovery complete - pausing {self.pause_after_recovery:.1f}s")
        self.transition_to_pause(self.pause_after_recovery, State.IDLE)

    def clear_costmaps_async(self):
        """Clear costmaps asynchronously."""
        req = ClearEntireCostmap.Request()
        if self.clear_global_costmap_client.service_is_ready():
            self.get_logger().info("🧹 Clearing global costmap")
            self.clear_global_costmap_client.call_async(req)
        if self.clear_local_costmap_client.service_is_ready():
            self.get_logger().info("🧽 Clearing local costmap")
            self.clear_local_costmap_client.call_async(req)

    # Rotation - non-blocking version
    def start_rotation_relative(self, rel_angle, next_state):
        """Start a relative rotation (non-blocking)."""
        if self.current_pose is None:
            self.transition_to_idle()
            return
        
        rel_angle = np.clip(rel_angle, -math.pi, math.pi)
        self.rotation_start_yaw = self.get_robot_yaw()
        self.rotation_target_yaw = math.atan2(
            math.sin(self.rotation_start_yaw + rel_angle),
            math.cos(self.rotation_start_yaw + rel_angle)
        )
        self.rotation_timeout = time.time() + abs(rel_angle) / 0.25 + 3.0
        self.rotation_next_state = next_state
        self.state = State.ROTATING
        
    def handle_rotation(self):
        """Handle rotation control loop."""
        if time.time() > self.rotation_timeout:
            self.cmd_vel_pub.publish(Twist())
            self.state = self.rotation_next_state
            return
        
        yaw = self.get_robot_yaw()
        err = math.atan2(
            math.sin(self.rotation_target_yaw - yaw),
            math.cos(self.rotation_target_yaw - yaw)
        )
        
        if abs(err) < 0.05:
            self.cmd_vel_pub.publish(Twist())
            self.state = self.rotation_next_state
            return
        
        k = 0.8
        omega = np.clip(k * err, -0.6, 0.6)
        omega = np.clip(abs(omega), 0.08, 0.6) * np.sign(omega) * self.rotation_sign
        
        twist = Twist()
        twist.angular.z = omega
        self.cmd_vel_pub.publish(twist)

    # Open-Space Controller
    def open_space_controller(self):
        """Detect open space and navigate toward it."""
        if self.scan_data is None or self.current_pose is None:
            return False
        if self.state != State.IDLE:
            return False
        
        # Check if we recently failed - don't retry immediately
        if self.open_space_failed_time is not None:
            time_since_failure = time.time() - self.open_space_failed_time
            if time_since_failure < self.open_space_retry_cooldown:
                return False  # Skip open space detection during cooldown
        
        ranges = self._ranges_numpy()
        if ranges is None or len(ranges) == 0:
            return False
        
        max_range = np.max(ranges)
        sensor_max = self.scan_data.range_max
        
        # Check if at sensor's max range (indicates open space) OR exceeds threshold
        if not (max_range >= sensor_max or max_range >= self.open_space_distance_threshold):
            return False
        
        # Count how many rays are at max range (open space)
        rays_at_max = np.sum(ranges >= sensor_max * 0.95)  # Within 95% of max
        min_rays_needed = 10  # Need at least 10 rays showing open space
        
        if rays_at_max < min_rays_needed:
            # Not enough rays - navigate closer with Nav2 instead
            self.get_logger().info(f"🌌 Open space detected but only {rays_at_max} rays - navigating closer")
            
            # Find direction of max range
            max_idx = int(np.argmax(ranges))
            angle_min = self.scan_data.angle_min
            angle_increment = self.scan_data.angle_increment
            open_angle = angle_min + (max_idx * angle_increment)
            
            # Send nav2 goal towards open space
            step_dist = np.clip(max_range * 0.7, 0.8, 1.5)
            yaw = self.get_robot_yaw()
            goal_heading = yaw + open_angle
            
            goal_x = self.current_pose.position.x + step_dist * math.cos(goal_heading)
            goal_y = self.current_pose.position.y + step_dist * math.sin(goal_heading)
            
            if self.is_point_free(goal_x, goal_y):
                self.send_goal(goal_x, goal_y)
                return True
            else:
                return False
        
        # Enough rays showing open space - proceed with scanning/crawling
        self.get_logger().info(f"🌌 Open space detected: max={max_range:.2f}m ({rays_at_max} rays at max)")
        
        # Smart direction: Check left, right, AND back to determine rotation direction
        n_rays = len(ranges)
        if n_rays >= 360:
            # Check left side (rays 10-90, ~10-90 degrees)
            left_rays = ranges[10:90]
            # Check right side (rays 270-350, ~-90 to -10 degrees)
            right_rays = ranges[270:350]
            # Check back side (rays 135-225, ~135-225 degrees / -135 to -180 and 180 to 135)
            back_rays = ranges[135:225]
        else:
            # Fallback for different ray counts
            quarter = n_rays // 4
            left_rays = ranges[quarter//9:quarter]
            right_rays = ranges[-quarter:-quarter//9]
            back_rays = ranges[n_rays//3:2*n_rays//3]
        
        # Count open rays on each side
        left_open = np.sum(left_rays >= sensor_max * 0.95)
        right_open = np.sum(right_rays >= sensor_max * 0.95)
        back_open = np.sum(back_rays >= sensor_max * 0.95)
        
        self.get_logger().info(f"📊 Open rays - LEFT: {left_open}, RIGHT: {right_open}, BACK: {back_open}")
        
        # Determine rotation direction based on which area has most open space
        if back_open > left_open and back_open > right_open:
            # Back has most open space - choose shorter rotation (prefer right for 180°)
            self.scan_rotation_direction = -1  # Rotate right to reach back
            self.scan_max_rotations = 36  # Need up to 180° to reach back
            self.get_logger().info(f"🔍 Smart scan: BACK has most open space ({back_open} rays) - rotating RIGHT up to 180°")
        elif left_open > right_open:
            self.scan_rotation_direction = 1  # Rotate left (counterclockwise)
            self.scan_max_rotations = 18  # Standard 90° scan
            self.get_logger().info(f"🔍 Smart scan: LEFT has more open space ({left_open} vs {right_open} rays)")
        else:
            self.scan_rotation_direction = -1  # Rotate right (clockwise)
            self.scan_max_rotations = 18  # Standard 90° scan
            self.get_logger().info(f"🔍 Smart scan: RIGHT has more open space ({right_open} vs {left_open} rays)")
        
        # Start path scanning
        self.scan_step_count = 0
        self.state = State.SCANNING_FOR_PATH
        self.state_start_time = time.time()
        return True

    def handle_path_scanning(self):
        """Non-blocking path scanning."""
        step_angle = math.radians(5) * self.scan_rotation_direction  # Use smart direction
        min_clearance = 0.8
        
        ranges = self._ranges_numpy()
        if ranges is None or len(ranges) == 0:
            self.get_logger().warn("⚠️ No scan data")
            self.open_space_failed_time = time.time()  # Mark as failed
            self.transition_to_idle()
            return
        
        n_rays = len(ranges)
        
        # Check current direction
        if n_rays >= 360:
            front_rays = np.concatenate([ranges[350:360], ranges[0:10]])
        else:
            front_width = min(10, n_rays // 36)
            front_rays = np.concatenate([ranges[-front_width:], ranges[:front_width]])
        
        front_rays = front_rays[np.isfinite(front_rays) | np.isinf(front_rays)]
        
        if len(front_rays) > 0:
            front_min = np.min(front_rays)
            front_avg = np.mean(front_rays)
            
            self.get_logger().info(
                f"  Step {self.scan_step_count+1}/{self.scan_max_rotations}: rays[350:360,0:10] → "
                f"min={front_min:.2f}m, avg={front_avg:.2f}m ({len(front_rays)} rays)"
            )
            
            # Found clear path!
            if front_min >= min_clearance:
                self.get_logger().info(f"✅ Clear! Min distance: {front_min:.2f}m")
                self.open_space_failed_time = None  # Clear failure flag on success
                self.scan_tried_both_directions = False  # Reset for next scan
                self.transition_to_pause(self.pause_after_align, State.CRAWLING)
                return
        
        # Need to rotate more
        self.scan_step_count += 1
        if self.scan_step_count >= self.scan_max_rotations:
            # Check if we already tried both directions
            if not hasattr(self, 'scan_tried_both_directions') or not self.scan_tried_both_directions:
                # First direction failed, try the opposite direction
                self.scan_rotation_direction *= -1  # Flip direction
                self.scan_step_count = 0  # Reset counter
                self.scan_max_rotations = 18  # Standard 90° for opposite direction
                self.scan_tried_both_directions = True
                direction_name = "LEFT" if self.scan_rotation_direction > 0 else "RIGHT"
                self.get_logger().warn(f"⚠️ No clear path in first direction - trying {direction_name} (up to 90°)")
                self.start_rotation_relative(step_angle, State.SCANNING_FOR_PATH)
                return
            else:
                # Both directions failed
                total_degrees = self.scan_step_count * 5  # Each step is 5°
                self.get_logger().warn(f"⚠️ No clear path found after checking both directions ({total_degrees}° total) - blocking retry for 10s")
                self.open_space_failed_time = time.time()  # Mark as failed, block retry
                self.scan_tried_both_directions = False  # Reset for next time
                self.transition_to_idle()
                return
        
        # Rotate 5° and check again next cycle
        direction_symbol = "↺" if self.scan_rotation_direction > 0 else "↻"
        self.get_logger().info(f"  ⚠️ Blocked - rotating 5° {direction_symbol}")
        self.start_rotation_relative(step_angle, State.SCANNING_FOR_PATH)

    def handle_crawling(self):
        """Non-blocking crawl forward."""
        if not hasattr(self, 'crawl_start_time') or self.crawl_start_time is None:
            # Just started crawling
            self.crawl_start_time = time.time()
            self.crawl_last_log = time.time()
            self.get_logger().info("🚶 Starting crawl")
        
        # Check timeout
        if time.time() - self.crawl_start_time > 1.0:
            self.cmd_vel_pub.publish(Twist())
            self.crawl_start_time = None
            self.open_space_failed_time = None  # Clear failure flag after successful crawl
            self.get_logger().info("✅ Crawl complete")
            self.transition_to_pause(self.pause_after_crawl, State.IDLE)
            return
        
        # Check front distance
        ranges = self._ranges_numpy()
        if ranges is None or len(ranges) == 0:
            self.cmd_vel_pub.publish(Twist())
            self.crawl_start_time = None
            self.transition_to_idle()
            return
        
        n_rays = len(ranges)
        if n_rays >= 360:
            front_rays = np.concatenate([ranges[350:360], ranges[0:10]])
        else:
            front_width = min(10, n_rays // 36)
            front_rays = np.concatenate([ranges[-front_width:], ranges[:front_width]])
        
        front_rays = front_rays[front_rays > 0]
        
        if len(front_rays) == 0:
            self.cmd_vel_pub.publish(Twist())
            self.crawl_start_time = None
            self.get_logger().warn("⚠️ No valid front rays")
            self.transition_to_idle()
            return
        
        front_min = np.min(front_rays)
        stop_distance = 0.6
        slowdown_distance = 1.0
        
        # STOP if too close
        if front_min < stop_distance:
            self.cmd_vel_pub.publish(Twist())
            self.crawl_start_time = None
            self.get_logger().warn(f"🛑 STOP at {front_min:.2f}m")
            self.transition_to_idle()
            return
        
        # Adjust speed based on distance
        if front_min < slowdown_distance:
            speed_factor = (front_min - stop_distance) / (slowdown_distance - stop_distance)
            speed_factor = max(0.3, min(1.0, speed_factor))
            speed = self.forward_speed * speed_factor
        else:
            speed = self.forward_speed
        
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)
        
        # Log periodically
        if time.time() - self.crawl_last_log > 0.5:
            self.get_logger().info(f"  📍 front={front_min:.2f}m, speed={speed:.2f}m/s")
            self.crawl_last_log = time.time()

    def debug_front_rays(self):
        """Debug: Print what rays 350-360 and 0-10 are seeing."""
        ranges = self._ranges_numpy()
        if ranges is None or len(ranges) == 0:
            return
        
        if len(ranges) >= 360:
            front_rays = np.concatenate([ranges[350:360], ranges[0:10]])
            front_rays_valid = front_rays[front_rays > 0]
            if len(front_rays_valid) > 0:
                self.get_logger().info(
                    f"[DEBUG] Front rays: min={np.min(front_rays_valid):.2f}m, "
                    f"avg={np.mean(front_rays_valid):.2f}m"
                )

    # Goal Handling
    def send_goal(self, x, y):
        """Send navigation goal to Nav2."""
        # Check attempted goals
        for ax, ay in self.attempted_goals:
            if math.hypot(x - ax, y - ay) < self.goal_reuse_threshold:
                return False
        
        # Check if free and not blacklisted
        if not self.is_point_free(x, y):
            return False
        for bx, by in self.blacklist:
            if math.hypot(x - bx, y - by) < self.goal_reuse_threshold:
                return False

        self.attempted_goals.append((x, y))

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 server unavailable")
            return False
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f"📍 Goal: ({x:.2f}, {y:.2f})")
        self.navigation_active = True
        self.state = State.NAVIGATING
        self.goal_start_time = time.time()
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, fut):
        goal_handle = fut.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn("❌ Goal rejected")
            self.navigation_active = False
            self.goal_start_time = None
            self.transition_to_idle()
            
            # Try alternate, then recovery if that fails
            if not self.try_lidar_alternate_goal():
                self.start_recovery()
            return
        
        self.get_logger().info("✅ Goal accepted")
        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, fut):
        self.navigation_active = False
        self.goal_start_time = None
        result = fut.result()
        
        # Check status
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f"🏁 Goal succeeded - pausing {self.pause_after_goal:.1f}s")
            self.transition_to_pause(self.pause_after_goal, State.IDLE)
            self.consecutive_failures = 0
            return
        
        # Goal failed
        self.get_logger().warn(f"❌ Goal failed (status {result.status})")
        
        # Add to blacklist if repeated failures near this location
        if self.current_pose:
            px, py = self.current_pose.position.x, self.current_pose.position.y
            nearby = sum(1 for ax, ay in self.attempted_goals 
                    if math.hypot(px - ax, py - ay) < self.goal_reuse_threshold)
            if nearby >= 2:
                self.blacklist.append((px, py))
                self.get_logger().warn(f"🚫 Blacklisted ({px:.2f}, {py:.2f})")
        
        # Try alternate immediately, then recovery if that fails
        if not self.try_lidar_alternate_goal():
            self.start_recovery()
        else:
            self.transition_to_idle()

    def try_lidar_alternate_goal(self):
        """Select a new goal toward the most open direction."""
        ranges = self._ranges_numpy()
        if ranges is None or len(ranges) == 0:
            return False
        
        max_idx = int(np.argmax(ranges))
        max_range = ranges[max_idx]
        
        if max_range < 0.8:
            self.get_logger().warn(f"🚫 Max range too short: {max_range:.2f}m")
            return False
        
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        open_angle = angle_min + (max_idx * angle_increment)
        
        step_dist = np.clip(max_range * 0.6, 0.6, 1.2)
        yaw = self.get_robot_yaw()
        goal_heading = yaw + open_angle
        
        goal_x = self.current_pose.position.x + step_dist * math.cos(goal_heading)
        goal_y = self.current_pose.position.y + step_dist * math.sin(goal_heading)
        
        if not self.is_point_free(goal_x, goal_y):
            return False
        
        self.get_logger().info(f"🎯 Alternate: ({goal_x:.2f}, {goal_y:.2f})")
        return self.send_goal(goal_x, goal_y)

    # Frontier Detection
    def detect_frontiers(self):
        """Detect unknown-edge frontiers in the occupancy map."""
        if self.map_data is None:
            return []

        free = (self.map_data == 0).astype(np.uint8)
        unknown = (self.map_data == -1).astype(np.uint8)
        frontiers = (binary_dilation(free, np.ones((3, 3))) & unknown).astype(np.uint8)

        num_labels, labels = cv2.connectedComponents(frontiers)
        results = []
        for i in range(1, num_labels):
            pts = np.argwhere(labels == i)
            if len(pts) >= self.min_frontier_size:
                centroid = pts.mean(axis=0)
                wx, wy = self.map_to_world(int(centroid[1]), int(centroid[0]))
                results.append({'centroid': (wx, wy), 'size': len(pts)})

        return results

    def select_best_frontier(self, frontiers):
        """Select the best frontier from candidates."""
        if not self.current_pose:
            return None
        
        rx, ry = self.current_pose.position.x, self.current_pose.position.y
        frontiers.sort(key=lambda f: math.hypot(f['centroid'][0] - rx, f['centroid'][1] - ry))
        
        for f in frontiers:
            gx, gy = f['centroid']
            
            if not self.is_point_free(gx, gy):
                continue
            if any(math.hypot(gx - bx, gy - by) < self.goal_reuse_threshold for bx, by in self.blacklist):
                continue
            if any(math.hypot(gx - ax, gy - ay) < self.goal_reuse_threshold for ax, ay in self.attempted_goals):
                continue
            
            return f
        
        return None

    # Main Loop
    def exploration_decision_loop(self):
        """Main exploration decision loop."""
        # Only make decisions when idle
        if self.state != State.IDLE:
            return
        if self.map_data is None or self.current_pose is None:
            return
        
        # Try open-space controller first
        if self.open_space_controller():
            return
        
        # Normal frontier exploration
        frontiers = self.detect_frontiers()
        
        if not frontiers:
            self.get_logger().warn("⚠️ No frontiers")
            
            # Try alternate goal
            if not self.try_lidar_alternate_goal():
                # No alternate either - trigger recovery
                self.start_recovery()
            return
        
        chosen = self.select_best_frontier(frontiers)
        
        if chosen:
            gx, gy = chosen['centroid']
            if not self.send_goal(gx, gy):
                # Couldn't send goal - try alternate
                if not self.try_lidar_alternate_goal():
                    self.start_recovery()
        else:
            self.get_logger().warn("⚠️ No valid frontier")
            
            # Try alternate goal
            if not self.try_lidar_alternate_goal():
                # No alternate either - trigger recovery
                self.start_recovery()


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()