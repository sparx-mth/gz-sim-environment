#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Float32
import math
from nav_msgs.msg import Odometry

class ColorFinder(Node):
    def __init__(self):
        super().__init__('color_finder_node_0')

        self.declare_parameter('camera_topic', '/robot_0/rgb_camera')
        self.declare_parameter('color', 'blue')

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.color = self.get_parameter('color').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)

        self.drone_name = self.camera_topic.split('/')[1]

        self.trigger_pub = self.create_publisher(Empty, f'/{self.drone_name}/color_detected_trigger', 10)
        self.azimuth_pub = self.create_publisher(Float32, f'/{self.drone_name}/detected_azimuth', 10)
        self.global_azimuth_pub = self.create_publisher(Float32, f'/{self.drone_name}/global_azimuth', 10)

        self.pose_sub = self.create_subscription(
            Odometry,
            f'/{self.drone_name}/ground_truth_pose',
            self.pose_callback,
            10
        )

        self.current_yaw_deg = None
        self.initial_yaw_deg = None 
        self.hfov_deg = 73.0

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_callback(self, msg):
        orientation = msg.pose.pose.orientation  
        yaw = self.quaternion_to_yaw(orientation)
        self.current_yaw_deg = math.degrees(yaw) % 360

        if self.initial_yaw_deg is None:
            self.initial_yaw_deg = self.current_yaw_deg
            self.get_logger().info(f"[{self.drone_name}] Initial yaw set: {self.initial_yaw_deg:.2f}°")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        if self.color == 'red':
            mask = cv2.inRange(hsv, (0, 150, 120), (10, 255, 255)) + \
                   cv2.inRange(hsv, (170, 150, 120), (180, 255, 255))
        elif self.color == 'blue':
            mask = cv2.inRange(hsv, (100, 150, 0), (140, 255, 255))
        else:
            self.get_logger().error(f"Unsupported color: {self.color}")
            return

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            cx = int(x + w / 2)
            cy = int(y + h / 2)

            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(cv_image, (cx, cy), 4, (255, 0, 0), -1)
            cv2.imshow("Detected Object", cv_image)
            cv2.waitKey(1)

            frame_width = cv_image.shape[1]
            relative_azimuth = ((cx / frame_width) - 0.5) * self.hfov_deg

            self.azimuth_pub.publish(Float32(data=relative_azimuth))
            self.trigger_pub.publish(Empty())
            self.get_logger().info(f"[{self.drone_name}] Relative azimuth: {relative_azimuth:.2f}°")

            if self.current_yaw_deg is not None and self.initial_yaw_deg is not None:
                delta_yaw_deg = (self.current_yaw_deg - self.initial_yaw_deg + 360) % 360
                global_azimuth = (delta_yaw_deg + relative_azimuth) % 360
                self.global_azimuth_pub.publish(Float32(data=global_azimuth))
                self.get_logger().info(f"[{self.drone_name}] Global azimuth: {global_azimuth:.2f}°")
            else:
                self.get_logger().warn("Yaw not yet available or initial yaw not set. Skipping global azimuth calculation.")
        else:
            cv2.imshow("Detected Object", cv_image)
            cv2.waitKey(1)
            self.get_logger().info("No target color detected.")

def main(args=None):
    rclpy.init(args=args)
    node = ColorFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()