#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import tf_transformations as tf
import math

class DroneColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector_azimuth')

        self.declare_parameter('camera_topic', '/robot_1/rgb_camera')
        self.declare_parameter('color', 'blue')

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.color = self.get_parameter('color').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.FOV = 90  # degrees
        self.current_yaw = None

        self.image_sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)

        drone_name = camera_topic.split('/')[1]
        pub_topic = '/' + drone_name + '/detected_azimuth'
        abs_pub_topic = '/' + drone_name + '/detected_absolute_azimuth'
        pose_topic = '/' + drone_name + '/ground_truth_to_tf/pose'

        self.azimuth_pub = self.create_publisher(Float32, pub_topic, 10)
        self.absolute_azimuth_pub = self.create_publisher(Float32, abs_pub_topic, 10)

        self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)

    def pose_callback(self, msg):
        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        euler = tf.euler_from_quaternion(quat)
        self.current_yaw = math.degrees(euler[2])

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_w = cv_image.shape[1]

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        if self.color == 'red':
            lower1 = (0, 150, 120)
            upper1 = (10, 255, 255)
            lower2 = (170, 150, 120)
            upper2 = (180, 255, 255)
            mask1 = cv2.inRange(hsv, lower1, upper1)
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = mask1 + mask2
        elif self.color == 'blue':
            lower = (100, 150, 0)
            upper = (140, 255, 255)
            mask = cv2.inRange(hsv, lower, upper)
        else:
            self.get_logger().error(f"Unsupported color: {self.color}")
            return

        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            area = cv2.contourArea(c)

            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            if M["m00"] != 0:
                cx = float(x + w / 2.0)
                center_x = float(img_w) / 2.0

                x_norm = (cx - center_x) / center_x
                azimuth = x_norm * (self.FOV / 2)

                self.get_logger().info(f"Azimuth: {azimuth:.2f} deg")
                self.azimuth_pub.publish(Float32(data=azimuth))

                if self.current_yaw is not None:
                    absolute_azimuth = abs((self.current_yaw + azimuth + 180) % 360 - 180)
                    self.get_logger().info(f"Absolute Azimuth: {absolute_azimuth:.2f} deg")
                    self.absolute_azimuth_pub.publish(Float32(data=absolute_azimuth))
                else:
                    self.get_logger().warn("Yaw not available yet!")

                cv2.circle(cv_image, (int(cx), int(cv_image.shape[0] / 2)), 5, (0, 255, 0), -1)
                cv2.imshow("Detected Blob", cv_image)
                cv2.waitKey(1)
            else:
                self.get_logger().warn("Contour found but zero area!")
        else:
            self.get_logger().warn("No contours detected.")

def main(args=None):
    rclpy.init(args=args)
    node = DroneColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()