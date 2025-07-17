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
from message_filters import Subscriber, ApproximateTimeSynchronizer
from scipy.spatial.transform import Rotation as R
import numpy as np
import math

class ColorFinder(Node):
    def __init__(self):
        super().__init__('color_finder_node_1')

        # Get parameters from ROS2 parameter server
        self.declare_parameter('camera_topic', '/robot_1/rgb_camera')
        self.declare_parameter('color', 'red')

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.color = self.get_parameter('color').get_parameter_value().string_value

        # Extract the drone name from the topic (e.g., robot_0)
        self.drone_name = self.camera_topic.split('/')[1]

        # Publishers
        self.bridge = CvBridge()
        self.trigger_pub = self.create_publisher(Empty, f'/{self.drone_name}/color_detected_trigger', 10)
        self.azimuth_pub = self.create_publisher(Float32, f'/{self.drone_name}/detected_azimuth', 10)
        self.global_azimuth_pub = self.create_publisher(Float32, f'/{self.drone_name}/global_azimuth', 10)

        # Message filter subscribers (used for synchronization)
        self.image_sub = Subscriber(self, Image, self.camera_topic)
        self.pose_sub = Subscriber(self, PoseStamped, f'/{self.drone_name}/robot_pose_slam')

        # ApproximateTimeSynchronizer: synchronizes topics by closest timestamp within 0.5s
        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.pose_sub],
            queue_size=10,
            slop=0.01  # max time difference allowed between messages
        )
        self.ts.registerCallback(self.synced_callback)

        # One-time pose subscriber for initial yaw
        self.initial_yaw_rad = None

        self.pose_once_sub = self.create_subscription(
            PoseStamped,
            f'/{self.drone_name}/robot_pose_slam',
            self.pose_once_callback,
            10
        )

        self.hfov_deg = 73.0  # horizontal field of view of the camera in degrees

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle in radians"""
        quat = [q.x, q.y, q.z, q.w]
        r = R.from_quat(quat)
        euler = r.as_euler('ZYX', degrees=False)  # Changed to ZYX for proper yaw extraction
        return euler[0]  # yaw is the first component in ZYX
    
    def normalize_angle(self, angle_rad):
        """Normalize angle to [-pi, pi] range"""
        while angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2 * math.pi
        return angle_rad
    
    def pose_once_callback(self, pose_msg):
        if self.initial_yaw_rad is None:
            orientation = pose_msg.pose.orientation
            self.initial_yaw_rad = self.quaternion_to_yaw(orientation)
            self.get_logger().info(f"[{self.drone_name}] Initial yaw set: {math.degrees(self.initial_yaw_rad):.2f}°")
            self.destroy_subscription(self.pose_once_sub)

    def synced_callback(self, image_msg, pose_msg):
        self.get_logger().info(f"Pose timestamp: {pose_msg.header.stamp}")
        self.get_logger().info(f"Image timestamp: {image_msg.header.stamp}")

        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Create a binary mask for the selected color
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
            # Take the largest contour
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            cx = int(x + w / 2)
            cy = int(y + h / 2)

            # Draw bounding box and center point
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(cv_image, (cx, cy), 4, (255, 0, 0), -1)
            cv2.startWindowThread()
            cv2.imshow("Detected Object", cv_image)
            cv2.waitKey(1)

            # Calculate relative azimuth angle (angle from camera center to detected object)
            frame_width = cv_image.shape[1]
            relative_azimuth_deg = ((cx / frame_width) - 0.5) * self.hfov_deg
            self.get_logger().info(f"[{self.drone_name}] relative_azimuth_deg: {relative_azimuth_deg:.2f}°")

            self.azimuth_pub.publish(Float32(data=relative_azimuth_deg))
            self.trigger_pub.publish(Empty())

            # Get current yaw from the pose
            orientation = pose_msg.pose.orientation
            current_yaw_rad = self.quaternion_to_yaw(orientation)
            
            # Skip if initial yaw is not set yet
            if self.initial_yaw_rad is None:
                self.get_logger().warn(f"[{self.drone_name}] Initial yaw not set yet, skipping global azimuth calculation")
                return

            # Calculate the change in yaw since initial position
            delta_yaw_rad = self.normalize_angle(current_yaw_rad - self.initial_yaw_rad)
            delta_yaw_deg = math.degrees(delta_yaw_rad)
            
            self.get_logger().info(f"[{self.drone_name}] Initial yaw: {math.degrees(self.initial_yaw_rad):.2f}°")
            self.get_logger().info(f"[{self.drone_name}] Current yaw: {math.degrees(current_yaw_rad):.2f}°")
            self.get_logger().info(f"[{self.drone_name}] Delta yaw: {delta_yaw_deg:.2f}°")

            # Global azimuth = robot's rotation + relative azimuth from camera
            global_azimuth_deg = delta_yaw_deg + relative_azimuth_deg
            
            # Normalize to [-180, 180] range for easier interpretation
            global_azimuth_normalized = self.normalize_angle(math.radians(global_azimuth_deg))
            global_azimuth_deg = math.degrees(global_azimuth_normalized)

            self.global_azimuth_pub.publish(Float32(data=global_azimuth_deg))
            self.get_logger().info(f"[{self.drone_name}] Global azimuth: {global_azimuth_deg:.2f}°")

        else:
            # No color detected – still show frame
            cv2.startWindowThread()
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