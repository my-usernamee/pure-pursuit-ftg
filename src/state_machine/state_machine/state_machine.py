#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
import numpy as np
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.ftg_start_time = None
        self.declare_parameter("max_ftg_allowance", 8.0)
        self.declare_parameter("required_safe_samples", 10)
        self.declare_parameter("min_ftg_hold", 1.2)
        self.declare_parameter("safety_dist", 0.5)
        self.declare_parameter("return_dist", 1.0)
        self.declare_parameter("timeout_return_clearance", 0.85)
        self.declare_parameter("front_window", 30)
        self.declare_parameter("opponent_flag_topic", "/opponent_detected")
        self.declare_parameter("opponent_pose_topic", "/opponent_detection")
        self.declare_parameter("ignore_opponent", True)
        self.declare_parameter("opponent_hold_time", 0.6)
        self.declare_parameter("opponent_ignore_dist", 3.5)
        self.declare_parameter("emergency_dist", 0.25)

        self.max_ftg_allowance = float(self.get_parameter("max_ftg_allowance").value)
        self.safe_count = 0          
        self.required_safe_samples = int(self.get_parameter("required_safe_samples").value)
        self.min_ftg_hold = float(self.get_parameter("min_ftg_hold").value)

        # Definition of States
        self.GB_TRACK = "GB_TRACK"    
        self.FTGONLY = "FTGONLY"  
        self.current_state = self.GB_TRACK

        self.safety_dist = float(self.get_parameter("safety_dist").value)
        self.return_dist = float(self.get_parameter("return_dist").value)
        self.timeout_return_clearance = float(self.get_parameter("timeout_return_clearance").value)
        self.front_window = int(self.get_parameter("front_window").value)
        self.opponent_flag_topic = self.get_parameter("opponent_flag_topic").value
        self.opponent_pose_topic = self.get_parameter("opponent_pose_topic").value
        self.ignore_opponent = bool(self.get_parameter("ignore_opponent").value)
        self.opponent_hold_time = float(self.get_parameter("opponent_hold_time").value)
        self.opponent_ignore_dist = float(self.get_parameter("opponent_ignore_dist").value)
        self.emergency_dist = float(self.get_parameter("emergency_dist").value)

        self.opponent_detected = False
        self.opponent_distance = None
        self.last_opponent_time = 0.0
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.state_pub = self.create_publisher(String, '/state', 10)
        self.opponent_flag_sub = self.create_subscription(
            Bool, self.opponent_flag_topic, self.opponent_flag_callback, 10
        )
        self.opponent_pose_sub = self.create_subscription(
            PoseStamped, self.opponent_pose_topic, self.opponent_pose_callback, 10
        )

        self.get_logger().info("--- State Machine Started: Defaulting to GB_TRACK ---")

    def scan_callback(self, msg):
            num_points = len(msg.ranges)
            mid = num_points // 2
            window = self.front_window  # around 10 degree by default
            front_view = msg.ranges[mid - window : mid + window]
            
            # filter out unreasonably small value
            valid_ranges = [r for r in front_view if r > 0.1]
            if not valid_ranges:
                return

            min_dist = min(valid_ranges)
            now = time.time()
            opponent_recent = (
                self.ignore_opponent and
                self.opponent_detected and
                (now - self.last_opponent_time) <= self.opponent_hold_time
            )
            opponent_close = (
                opponent_recent and
                self.opponent_distance is not None and
                self.opponent_distance <= self.opponent_ignore_dist
            )
            if opponent_close and min_dist > self.emergency_dist:
                min_dist = max(min_dist, self.return_dist + 0.05)
            new_state = self.current_state
            if self.current_state == self.GB_TRACK:
                if min_dist < self.safety_dist:
                    new_state = self.FTGONLY
                    self.ftg_start_time = now
                    self.safe_count = 0
            else:
                duration = now - self.ftg_start_time
                if min_dist > self.return_dist:
                    self.safe_count += 1
                else:
                    self.safe_count = 0

                should_return_by_clearance = (
                    duration > self.min_ftg_hold and
                    self.safe_count >= self.required_safe_samples
                )
                should_return_by_timeout = (
                    duration > self.max_ftg_allowance and
                    min_dist > self.timeout_return_clearance
                )

                if should_return_by_clearance or should_return_by_timeout:
                    new_state = "GB_TRACK"
                    self.ftg_start_time = None 

            # print out log at state transition
            if new_state != self.current_state:
                if new_state == self.FTGONLY:
                    self.get_logger().warn(f"[DETECTED] Obstacle at {min_dist:.2f}m! Switching to FTGONLY")
                else:
                    self.get_logger().info(f"[CLEAR] Path is clear. Returning to GB_TRACK")
                
                self.current_state = new_state

            # publish state to controller
            state_msg = String()
            state_msg.data = self.current_state
            self.state_pub.publish(state_msg)

    def opponent_flag_callback(self, msg: Bool):
        self.opponent_detected = bool(msg.data)
        if self.opponent_detected:
            self.last_opponent_time = time.time()

    def opponent_pose_callback(self, msg: PoseStamped):
        dx = float(msg.pose.position.x)
        dy = float(msg.pose.position.y)
        self.opponent_distance = math.sqrt(dx * dx + dy * dy)
        self.last_opponent_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StateMachine())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
