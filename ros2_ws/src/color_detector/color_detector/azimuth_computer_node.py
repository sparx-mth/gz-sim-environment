#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty
from scipy.spatial.transform import Rotation as R
import math
from nav_msgs.msg import Odometry

class AzimuthComputer(Node):
    def __init__(self):
        super().__init__('azimuth_computer_node')

        self.declare_parameter('camera_topic', '/robot_1/rgb_camera')
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        drone_name = self.camera_topic.split('/')[1]

        self.bridge = CvBridge()
        self.FOV = 73  # degrees
        self.current_yaw = None
        self.process_next_image = False

        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.pose_sub = self.create_subscription(Odometry, f'/{drone_name}/ground_truth_pose', self.pose_callback, 10)
        self.trigger_sub = self.create_subscription(Empty, f'/{drone_name}/color_detected_trigger', self.trigger_callback, 10)

        self.azimuth_pub = self.create_publisher(Float32, f'/{drone_name}/detected_azimuth', 10)
        self.absolute_azimuth_pub = self.create_publisher(Float32, f'/{drone_name}/detected_absolute_azimuth', 10)

    def trigger_callback(self, msg):
        #self.get_logger().info("Trigger received – will process next image.")

        self.process_next_image = True

    def pose_callback(self, msg):
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        r = R.from_quat(quat)
        euler_xyz = r.as_euler('xyz', degrees=True)
        euler_zyx = r.as_euler('zyx', degrees=True)
        euler_yxz = r.as_euler('yxz', degrees=True)

        #self.get_logger().info(f"Euler xyz: {euler_xyz}")
        #self.get_logger().info(f"Euler zyx: {euler_zyx}")
        #self.get_logger().info(f"Euler yxz: {euler_yxz}")


        euler = r.as_euler('zxy', degrees=True)
        self.current_yaw = euler[0]
        self.get_logger().info(f"Yaw: {self.current_yaw:.2f} deg")
        #self.get_logger().info(f"Received yaw from pose: {self.current_yaw:.2f} deg")

    def image_callback(self, msg):
        if not self.process_next_image:
            return

        self.process_next_image = False

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_w = cv_image.shape[1]

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 150, 120), (10, 255, 255)) + \
               cv2.inRange(hsv, (170, 150, 120), (180, 255, 255))

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            cx = float(x + w / 2.0)
            center_x = float(img_w) / 2.0
            x_norm = (cx - center_x) / center_x
            azimuth = x_norm * (self.FOV / 2)

            self.get_logger().info(f"Azimuth: {azimuth:.2f} deg")
            self.azimuth_pub.publish(Float32(data=azimuth))

            if self.current_yaw is not None:
                absolute_azimuth = (self.current_yaw + azimuth + 180) % 360 - 180
                self.get_logger().info(f"Absolute Azimuth: {absolute_azimuth:.2f} deg")
                self.absolute_azimuth_pub.publish(Float32(data=absolute_azimuth))
            else:
                self.get_logger().warn("Yaw not available!")

def main(args=None):
    rclpy.init(args=args)
    node = AzimuthComputer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
