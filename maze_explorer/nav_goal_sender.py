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
import time

class Exploration_Node(Node):
    def __init__(self):
        super().__init__('Exploration_Node')
        
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)    #To Subscribe to Lidar
        self.create_subscription(Odometry, '/odom', self.odom_pos_callback, 10) #To Subscrive to odom(Current Pos)
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)             #To publish to robot movement
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)    #To Publish goal/Set Goal
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose') #To tell Nav2 to sent goal/go there
        
        #Robot inital value
        self.x = self.y = self.yaw = 0.0
        self.odom_ready = False
        
        #Navigation Value
        self.initial_goal = (5.0, 0.0)      #Set Custom goal
        self.goal_distance = 3.0            #Set goal 2.5m from robot pos
        self.timeout = 30.0                 #Set timeout for recovery behaviour
        self.inf_time = 2.0                 #Check how long inf val see
        self.exit_time = 5.0                #Check how long the robot is out of maze
        self.max_fails = 3                  #Set max number of time before recovery trigger
        self.recovery_cooldown = 10.0       #Each interval of recovery trigger
        
        #Tracking flag
        self.nav_start = None
        self.inf_start = None
        self.exit_start = None
        self.last_goal = None
        self.goal_handle = None
        self.last_recovery = None
        self.fails = 0
        
        #Track robot status
        self.done = False
        self.rotating = False
        self.navigating = False
        self.inf_triggered = False
        self.in_timeout = False
        self.in_recovery = False
        
        #Check for rotation and time out
        self.create_timer(0.1, self.rotation_loop)
        self.create_timer(1.0, self.stuck_timeout)
        
        self.get_logger().info('Exploration Start')
    
    def stop_movement(self): #Stop robot movement                  
        self.cmd_vel.publish(Twist())
    
    def angle_range(self, angle): #Check angle from -270 to 190
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def extract_sensor_data(self, msg, angle_min, angle_max): #retrive value from Lidar and divide into 4 parts
        ranges = []
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            angle = (math.degrees(msg.angle_min + i*msg.angle_increment) % 360)
            if angle_min > angle_max:
                if angle >= angle_min or angle <= angle_max:
                    ranges.append(r)
            else:
                if angle_min <= angle <= angle_max:
                    ranges.append(r)
        return ranges

    def odom_pos_callback(self, msg): #Get robot position reference from odom, X,y and yaw
        if self.done:
            self.stop_movement()
            return
        
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        
        if not self.odom_ready:
            self.odom_ready = True
            self.send_goal(*self.initial_goal, 0.0)
    
    def lidar_callback(self, msg): #logic for exit detection and exit maze

        if self.done:       #Robot out of maze, stop robot
            self.stop_movement()
            return
        
        if not self.odom_ready or self.in_recovery: #dont do anything unless know its pos
            return
        
        inf_idx = np.where(np.isinf(msg.ranges))[0] #check lidar val got inf
        if len(inf_idx) == 0:                       #no inf detected
            self.inf_start = self.exit_start = None
            return
        
       #check whether robot out of maze from 270 to 90
        front_inf = sum(1 for i in inf_idx if 
                       (a := math.degrees(msg.angle_min + i*msg.angle_increment)%360) >= 270 or a <= 90)
        total_front = sum(1 for i in range(len(msg.ranges)) if 
                         (a := math.degrees(msg.angle_min + i*msg.angle_increment)%360) >= 270 or a <= 90)
        front_pct = (front_inf/total_front)*100 if total_front > 0 else 0
        
        if front_pct > 80:
            if self.exit_start is None:
                self.exit_start = self.get_clock().now()
            elif (self.get_clock().now() - self.exit_start).nanoseconds/1e9 >= self.exit_time:
                self.maze_complete()
                return
        else:
            self.exit_start = None
        
        #Check exit is found
        if self.rotating or self.inf_triggered:
            return
        
        angle = msg.angle_min + inf_idx[len(inf_idx)//2]*msg.angle_increment
        
        if self.inf_start is None:
            self.inf_start = self.get_clock().now()
        elif (self.get_clock().now() - self.inf_start).nanoseconds/1e9 >= self.inf_time:
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            
            self.inf_triggered = True
            diff = self.angle_range(angle)
            
            if abs(diff) < 0.05:
                self.send_goal_location(angle)
            else:
                self.rotating = True
                self.target_yaw = self.angle_range(self.yaw + angle)
    
    #Robot out of mze
    def maze_complete(self):
        self.get_logger().info('Robot out of maze')
        
        self.done = True    #Set flag to done
        
        #cancel navigation
        if self.goal_handle:
            try:
                self.goal_handle.cancel_goal_async()
            except:
                pass
        
        #reset Flag
        self.navigating = False
        self.rotating = False
        self.inf_triggered = False
        self.in_recovery = False
        
       #Stop robot multiple times
        for _ in range(10):
            self.stop_movement()
            time.sleep(0.1)
    
    #Timeout if robot in same position too long
    def stuck_timeout(self):
        if self.done:
            self.stop_movement()
            return
        
        if not self.navigating or not self.nav_start:
            return
        
        if (self.get_clock().now() - self.nav_start).nanoseconds/1e9 > self.timeout:
            self.in_timeout = True
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.navigating = False
            self.send_goal_location(0.0)
    
    #Recovery rotate in place
    def rotation_loop(self):
        if self.done:
            self.stop_movement()
            return
        
        if not self.rotating or not hasattr(self, 'target_yaw'):
            return
        
        diff = self.angle_range(self.target_yaw - self.yaw)
        
        if abs(diff) < 0.05:
            self.stop_movement()
            self.rotating = False
            self.send_goal_location(0.0)
        else:
            twist = Twist()
            twist.angular.z = (0.5 if diff > 0 else -0.5) * (0.5 if abs(diff) < 0.2 else 1.0)
            self.cmd_vel.publish(twist)
    
   #Send goal in rviz
    def send_goal_location(self, angle):
        #Dont sent goal if complete
        if self.done:
            return
        
        g_angle = self.yaw + angle
        gx = self.x + self.goal_distance * math.cos(g_angle)
        gy = self.y + self.goal_distance * math.sin(g_angle)
        self.send_goal(gx, gy, g_angle)
    
    #Send goal in nav2
    def send_goal(self, x, y, yaw):
        #Dont sent goal if complete
        if self.done:
            return
        
        self.last_goal = (x, y, yaw)
        self.navigating = True
        self.nav_start = self.get_clock().now()
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.nav_start.to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw/2)
        goal.pose.pose.orientation.w = math.cos(yaw/2)
        
        self.goal_pub.publish(goal.pose)
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.reset()
            return
        
        self.nav_client.send_goal_async(goal).add_done_callback(self.goal_feedback)
    
    #Check goal feedback
    def goal_feedback(self, future):
        #Stop if complete
        if self.done:
            return
        
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.fails += 1
            self.check_recovery()
        else:
            self.goal_handle.get_result_async().add_done_callback(self.goal_outcome)
    
    #check goal complete, success/fail
    def goal_outcome(self, future):
        #Stop if complete
        if self.done:
            return
        
        status = future.result().status
        
        if status == GoalStatus.STATUS_CANCELED and self.in_timeout:
            self.in_timeout = False
            return
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.fails = 0
            self.reset()
        elif status == GoalStatus.STATUS_ABORTED:
            self.fails += 1
            self.check_recovery()
        elif status == GoalStatus.STATUS_CANCELED and not (self.rotating or self.inf_triggered):
            self.fails += 1
            self.check_recovery()
        
        self.navigating = False
        self.nav_start = self.goal_handle = None
    
    #Recovery Behaviour
    
    def check_recovery(self):
        #Dont recover if complete
        if self.done:
            return
        
        if self.fails < self.max_fails:
            self.reset()
            return
        
        if self.last_recovery:
            elapsed = (self.get_clock().now() - self.last_recovery).nanoseconds/1e9
            if elapsed < self.recovery_cooldown:
                self.fails = 0
                self.reset()
                return
        
        self.recovery()
    
    #Execute recovery
    def recovery(self):
        #Stop if complete
        if self.done:
            return
        
        self.last_recovery = self.get_clock().now()
        self.fails = 0
        self.in_recovery = True
        self.stop_movement()
        time.sleep(1.0)
        
        msg = self.lidar_status()
        if not msg:
            self.recovery_spin()
            return
        
        #check sensor which side is empty
        directions = {
            'forward': self.extract_sensor_data(msg, 350, 10),
            'backward': self.extract_sensor_data(msg, 170, 190),
            'left': self.extract_sensor_data(msg, 80, 100),
            'right': self.extract_sensor_data(msg, 260, 280)
        }
        
        clearances = {k: min(v) if v else 0.0 for k, v in directions.items()}
        safe = {k: v for k, v in clearances.items() if v >= 0.4}
        
        if not safe:
            self.recovery_spin()
            return
        
        #Choose safest method
        best = max(safe, key=safe.get)
        
        #Recovery movement (Linear_vel, Angular_vel, duration)
        moves = {
            'forward': (0.2, 0.0, 1.5),
            'backward': (-0.1, 0.0, 1.0),
            'left': (0.15, 0.3, 2.0),
            'right': (0.15, -0.3, 2.0)
        }
        
        self.control_movement(*moves[best])
        self.in_recovery = False
        self.reset()
        
        if self.last_goal:
            self.send_goal(*self.last_goal)
        else:
            self.send_goal(*self.initial_goal, 0.0)
    
    #Reovery complete then read sensor data
    def lidar_status(self):
        msg = None
        def cb(m):
            nonlocal msg
            msg = msg or m
        
        sub = self.create_subscription(LaserScan, '/scan', cb, 10)
        start = self.get_clock().now()
        
        while not msg and (self.get_clock().now() - start).nanoseconds/1e9 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.destroy_subscription(sub)
        return msg
    
    #Recovery using cmd_vel
    def control_movement(self, linear, angular, duration):
        twist = Twist()
        twist.linear.x, twist.angular.z = linear, angular
        
        for _ in range(int(duration * 10)):
            # FIXED: Check if done during movement
            if self.done:
                self.stop_movement()
                return
            self.cmd_vel.publish(twist)
            time.sleep(0.1)
        
        self.stop_movement()
        time.sleep(0.5)
    
    #Recovery rotate in place
    def recovery_spin(self):
        if self.done:
            return
        
        self.control_movement(0.0, 0.5, 6.0)
        self.in_recovery = False
        self.reset()
    
    #Reset flag to start navigation
    def reset(self):
        """Clear all navigation state flags"""
        self.navigating = self.inf_triggered = False
        self.inf_start = self.nav_start = self.goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    node = Exploration_Node()
    try:                        #Run Main Program
        rclpy.spin(node)
    except KeyboardInterrupt:   #When exit program
        node.stop_movement()     
    finally:                    #After exit progam
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()