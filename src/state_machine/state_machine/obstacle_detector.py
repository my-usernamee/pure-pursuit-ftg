#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('detect_dist', 0.9)
        self.declare_parameter('front_window', 30)
        self.declare_parameter('status_topic', '/obstacle_detected')
        self.declare_parameter('distance_topic', '/obstacle_distance')
        self.declare_parameter('marker_topic', '/obstacle_marker')

        self.scan_topic = self.get_parameter('scan_topic').value
        self.detect_dist = float(self.get_parameter('detect_dist').value)
        self.front_window = int(self.get_parameter('front_window').value)
        self.status_topic = self.get_parameter('status_topic').value
        self.distance_topic = self.get_parameter('distance_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value

        self.detect_pub = self.create_publisher(Bool, self.status_topic, 10)
        self.distance_pub = self.create_publisher(Float32, self.distance_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        self.get_logger().info("Obstacle detector started")

    def on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=float)
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = float(msg.range_max) if msg.range_max > 0.0 else 10.0

        mid = len(ranges) // 2
        window = max(1, self.front_window)
        start = max(0, mid - window)
        end = min(len(ranges), mid + window)
        front = ranges[start:end]
        valid = front[(front > 0.05)]

        detected = False
        min_dist = float('inf')
        min_idx = None
        if valid.size > 0:
            min_dist = float(np.min(valid))
            detected = min_dist < self.detect_dist
            if detected:
                local_idx = int(np.argmin(front))
                min_idx = start + local_idx

        detect_msg = Bool()
        detect_msg.data = bool(detected)
        self.detect_pub.publish(detect_msg)

        dist_msg = Float32()
        dist_msg.data = 0.0 if min_dist == float('inf') else min_dist
        self.distance_pub.publish(dist_msg)

        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = msg.header.frame_id
        marker.ns = 'obstacle'
        marker.id = 0
        if detected and min_idx is not None:
            angle = msg.angle_min + min_idx * msg.angle_increment
            x = min_dist * np.cos(angle)
            y = min_dist * np.sin(angle)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.05
            marker.scale.x = 0.18
            marker.scale.y = 0.18
            marker.scale.z = 0.18
            marker.color.a = 0.95
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
        else:
            marker.action = Marker.DELETE
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
