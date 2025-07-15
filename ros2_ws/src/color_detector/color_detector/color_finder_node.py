#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

class ColorFinder(Node):
    def __init__(self):
        super().__init__('color_finder_node')

        self.declare_parameter('camera_topic', '/robot_0/rgb_camera')
        self.declare_parameter('color', 'red')

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.color = self.get_parameter('color').get_parameter_value().string_value

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)

        drone_name = self.camera_topic.split('/')[1]
        self.trigger_pub = self.create_publisher(Empty, f'/{drone_name}/color_detected_trigger', 10)

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
            self.get_logger().info("Target color detected. Sending trigger.")
            self.trigger_pub.publish(Empty())  
            self.get_logger().info("No target color detected.")

def main(args=None):
    rclpy.init(args=args)
    node = ColorFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()