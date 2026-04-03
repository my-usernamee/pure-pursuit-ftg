#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from pure_pursuit.pure_pursuit_logic import PurePursuitLogic
from pure_pursuit.ftg_logic import FTGLogic

class ControllerManager(Node):
    def __init__(self):
        super().__init__('controller_manager_node')

        self.declare_parameter("waypoints_path", "/sim_ws/src/pure_pursuit/racelines/arc.csv")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("state_topic", "/state")
        self.declare_parameter("opp_odom_topic", "/ego_racecar/opp_odom")
        self.declare_parameter("min_lookahead", 2.0)
        self.declare_parameter("max_lookahead", 4.0)
        self.declare_parameter("lookahead_ratio", 8.0)
        self.declare_parameter("K_p", 0.5)
        self.declare_parameter("steering_limit", 25.0) # Degrees
        self.declare_parameter("velocity_percentage", 0.6)
        self.declare_parameter("wheelbase", 0.33)
        self.declare_parameter("visualize_lookahead", False)
        self.declare_parameter("overtake_enable", True)
        self.declare_parameter("use_lidar_opponent", False)
        self.declare_parameter("lidar_opp_topic", "/opponent_detection")
        self.declare_parameter("lidar_opp_flag_topic", "/opponent_detected")
        self.declare_parameter("overtake_dist", 3.0)
        self.declare_parameter("overtake_clear_dist", 4.0)
        self.declare_parameter("overtake_lat_offset", 0.7)
        self.declare_parameter("overtake_speed_delta", 0.2)

        self.path = self.get_parameter("waypoints_path").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.drive_topic = self.get_parameter("drive_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.opp_odom_topic = self.get_parameter("opp_odom_topic").value
        self.min_la = self.get_parameter("min_lookahead").value
        self.max_la = self.get_parameter("max_lookahead").value
        self.la_ratio = self.get_parameter("lookahead_ratio").value
        self.kp = self.get_parameter("K_p").value
        self.steer_limit = np.radians(self.get_parameter("steering_limit").value)
        self.vel_percent = self.get_parameter("velocity_percentage").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.visualize_lookahead = bool(self.get_parameter("visualize_lookahead").value)
        self.overtake_enable = bool(self.get_parameter("overtake_enable").value)
        self.use_lidar_opponent = bool(self.get_parameter("use_lidar_opponent").value)
        self.lidar_opp_topic = self.get_parameter("lidar_opp_topic").value
        self.lidar_opp_flag_topic = self.get_parameter("lidar_opp_flag_topic").value
        self.overtake_dist = float(self.get_parameter("overtake_dist").value)
        self.overtake_clear_dist = float(self.get_parameter("overtake_clear_dist").value)
        self.overtake_lat_offset = float(self.get_parameter("overtake_lat_offset").value)
        self.overtake_speed_delta = float(self.get_parameter("overtake_speed_delta").value)

        # 2. Initialize Logic & Data
        self.waypoints = np.loadtxt(self.path, delimiter=',', skiprows=1) # Assume x, y, v
        self.pure_pursuit_logic = PurePursuitLogic(self.wheelbase, self.waypoints)
        self.ftg_logic = FTGLogic()
        self.curr_velocity = 0.0
        self.current_state = "GB_TRACK" 
        self.latest_scan = None
        self.last_track_steer = 0.0
        self.last_lookahead_point = None
        self.opp_pose = None
        self.opp_speed = 0.0
        self.lidar_opp_pose = None
        self.lidar_opp_detected = False
        self.overtake_active = False
        self.overtake_side = 1
        self.overtake_blend = 0.0

        # 3. Pubs & Subs
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.state_sub = self.create_subscription(String, self.state_topic, self.state_callback, 10)
        if self.overtake_enable:
            self.opp_sub = self.create_subscription(Odometry, self.opp_odom_topic, self.opp_odom_callback, 10)
            if self.use_lidar_opponent:
                self.lidar_opp_sub = self.create_subscription(
                    PoseStamped, self.lidar_opp_topic, self.lidar_opp_callback, 10
                )
                self.lidar_opp_flag_sub = self.create_subscription(
                    Bool, self.lidar_opp_flag_topic, self.lidar_opp_flag_callback, 10
                )
            else:
                self.lidar_opp_sub = None
                self.lidar_opp_flag_sub = None
        else:
            self.opp_sub = None

        self.get_logger().info("Pure Pursuit Node Started")
        self.viz_pub = self.create_publisher(Marker, '/waypoint_markers', 10)
        self.path_viz_pub = self.create_publisher(Marker, '/full_track_path', 10)
        # Trigger the path visualization once at the start
        # (Wait a tiny bit for RViz to connect)
        self.create_timer(1.0, self.publish_static_path)
        if not self.visualize_lookahead:
            self._cleared_lookahead = False
            self.create_timer(1.0, self.clear_lookahead_marker)
        else:
            self.create_timer(0.1, self.publish_lookahead_timer)

    def state_callback(self, msg):
        if msg.data != self.current_state:
            self.get_logger().info(f"--- STATE SWITCH: {self.current_state} -> {msg.data} ---")
            self.ftg_logic.prev_steering = 0.0
        self.current_state = msg.data
    
    def scan_callback(self, msg):
        self.latest_scan = msg

    def opp_odom_callback(self, msg):
        self.opp_pose = msg.pose.pose
        self.opp_speed = msg.twist.twist.linear.x

    def lidar_opp_callback(self, msg: PoseStamped):
        self.lidar_opp_pose = msg.pose

    def lidar_opp_flag_callback(self, msg: Bool):
        self.lidar_opp_detected = bool(msg.data)

    def get_yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.get_logger().info(f"DEBUG: Active Logic: {self.current_state}", throttle_duration_sec=1.0)
        self.curr_velocity = msg.twist.twist.linear.x
        
        if self.current_state == "FTGONLY":
            # Keep raceline hint fresh even during FTG so local avoidance
            # can rejoin the computed line contextually.
            self.update_track_hint(msg)
            self.execute_ftg_logic()
        else:
            self.execute_pure_pursuit_logic(msg)

    def update_track_hint(self, msg):
        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y
        car_yaw = self.get_yaw_from_quat(msg.pose.pose.orientation)

        la_ratio = self.get_parameter("lookahead_ratio").value
        min_la = self.get_parameter("min_lookahead").value
        max_la = self.get_parameter("max_lookahead").value
        lookahead_dist = np.clip(max_la * self.curr_velocity / la_ratio, min_la, max_la)

        target_pt_car, actual_la, target_idx = self.pure_pursuit_logic.find_target_waypoint(
            car_x, car_y, car_yaw, lookahead_dist
        )
        if target_idx == -1:
            return None

        steer = self.pure_pursuit_logic.calculate_steering(target_pt_car, actual_la, self.kp)
        self.last_track_steer = float(np.clip(steer, -self.steer_limit, self.steer_limit))
        return target_idx
    
    def execute_ftg_logic(self):
        if self.latest_scan is None:
            return

        # Provide a moderate raceline-side hint so FTG can favor the inner route
        # when that side is still viable.
        ftg_hint = float(np.clip(self.last_track_steer, -0.30, 0.30))
        speed, steer = self.ftg_logic.process_lidar(
            self.latest_scan, preferred_steer=ftg_hint
        )
        # self.get_logger().warn(f"FTG Active: Steer={steer:.2f}, Speed={speed:.2f}", throttle_duration_sec=1.0)
        self.publish_drive(steer, speed)

    def execute_pure_pursuit_logic(self, msg):
        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y
        car_yaw = self.get_yaw_from_quat(msg.pose.pose.orientation)

        # Dynamic Lookahead
        la_ratio = self.get_parameter("lookahead_ratio").value
        min_la = self.get_parameter("min_lookahead").value
        max_la = self.get_parameter("max_lookahead").value
        lookahead_dist = np.clip(max_la * self.curr_velocity / la_ratio, min_la, max_la)

        target_pt_car, actual_la, target_idx = self.pure_pursuit_logic.find_target_waypoint(
            car_x, car_y, car_yaw, lookahead_dist
        )

        if target_idx == -1:
            self.publish_drive(0.0, 0.0) 
            return

        if self.overtake_enable:
            opp_car = None
            opp_dist = None
            opp_ahead = False
            slower = False
            if self.use_lidar_opponent and self.lidar_opp_detected and self.lidar_opp_pose is not None:
                opp_car = np.array([self.lidar_opp_pose.position.x, self.lidar_opp_pose.position.y])
                opp_dist = float(np.linalg.norm(opp_car[:2]))
                opp_ahead = opp_car[0] > 0.25
                slower = True
            elif self.opp_pose is not None:
                opp_x = self.opp_pose.position.x
                opp_y = self.opp_pose.position.y
                opp_car = self.pure_pursuit_logic.transform_point_to_car_frame(
                    car_x, car_y, car_yaw, np.array([opp_x, opp_y])
                )
                opp_dist = float(np.linalg.norm(opp_car[:2]))
                opp_ahead = opp_car[0] > 0.25
                slower = self.opp_speed < (self.curr_velocity - self.overtake_speed_delta)

            if opp_car is not None and opp_dist is not None:
                if opp_ahead and opp_dist < self.overtake_dist and (slower or self.curr_velocity > 0.6):
                    if not self.overtake_active:
                        self.overtake_side = self.pick_overtake_side()
                    self.overtake_active = True
                elif (not opp_ahead) or opp_dist > self.overtake_clear_dist:
                    self.overtake_active = False

                if self.overtake_active:
                    self.overtake_blend = min(1.0, self.overtake_blend + 0.08)
                else:
                    self.overtake_blend = max(0.0, self.overtake_blend - 0.08)

                if self.overtake_blend > 0.0:
                    side = 1 if self.overtake_side >= 0 else -1
                    target_pt_car[1] += side * self.overtake_lat_offset * self.overtake_blend
                    target_pt_car[1] = float(np.clip(target_pt_car[1], -1.8, 1.8))

        self.last_lookahead_point = self.waypoints[target_idx]
        steer = self.pure_pursuit_logic.calculate_steering(target_pt_car, actual_la, self.kp)
        self.last_track_steer = float(np.clip(steer, -self.steer_limit, self.steer_limit))
        target_vel = self.waypoints[target_idx, 2] * self.vel_percent
        
        self.publish_drive(steer, target_vel)
   
    def publish_drive(self, steer, vel):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = float(vel)
        drive_msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(drive_msg)
    
    def visualize_lookahead_point(self, point):
        """
        Publishes a marker to visualize the current lookahead point in RViz.
        :param point: A list or array [x, y] in the 'map' frame.
        """
        if not self.visualize_lookahead:
            return
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lookahead_point"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = Duration(seconds=0).to_msg()
        
        # Set the scale of the sphere (diameter in meters)
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        
        # Set the color (RGBA) - Bright Red for visibility
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        # Set the position of the marker
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = 0.0 # Waypoints are on the 2D plane
        
        # Publish the marker
        self.viz_pub.publish(marker)

    def clear_lookahead_marker(self):
        if self._cleared_lookahead:
            return
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lookahead_point"
        marker.id = 1
        marker.action = Marker.DELETE
        self.viz_pub.publish(marker)
        self._cleared_lookahead = True

    def publish_lookahead_timer(self):
        if self.last_lookahead_point is None:
            return
        self.visualize_lookahead_point(self.last_lookahead_point)

    def pick_overtake_side(self):
        if self.latest_scan is None:
            return 1
        ranges = np.array(self.latest_scan.ranges, dtype=float)
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = 10.0

        center = len(ranges) // 2
        fov = int(70 / (self.latest_scan.angle_increment * 180 / np.pi))
        start = max(0, center - fov)
        end = min(len(ranges), center + fov)
        front = ranges[start:end]
        if len(front) < 10:
            return 1

        mid = len(front) // 2
        right = front[:mid]
        left = front[mid:]
        left_open = float(np.percentile(left, 75))
        right_open = float(np.percentile(right, 75))
        if abs(left_open - right_open) < 0.2:
            return 1
        return 1 if left_open > right_open else -1
    def publish_static_path(self):
        """
        Publishes all waypoints from the CSV as a single continuous green line.
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "static_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP # This connects all points in order
        marker.action = Marker.ADD
        
        # Line width
        marker.scale.x = 0.1 
        
        # Color: Green (so it contrasts with your red lookahead dot)
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        # Add all waypoints from your loaded CSV to the marker
        for wp in self.waypoints:
            p = Point()
            p.x = float(wp[0])
            p.y = float(wp[1])
            p.z = 0.0
            marker.points.append(p)
        
        # If it's a loop, connect the last point to the first
        if len(self.waypoints) > 0:
            p_start = Point()
            p_start.x = float(self.waypoints[0][0])
            p_start.y = float(self.waypoints[0][1])
            marker.points.append(p_start)

        self.path_viz_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControllerManager())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
