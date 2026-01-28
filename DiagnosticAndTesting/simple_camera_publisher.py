#!/usr/bin/env python3
"""
Simple Camera Publisher for ROS2
Alternative to usb_cam - publishes camera images to ROS2 topics
Works with IFWATER stereo camera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class SimpleCameraPublisher(Node):
    def __init__(self):
        super().__init__('simple_camera_publisher')
        
        # Parameters
        self.declare_parameter('left_device', '/dev/video32')
        self.declare_parameter('right_device', '/dev/video33')
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_rate', 30.0)
        
        # Get parameters
        left_device = self.get_parameter('left_device').value
        right_device = self.get_parameter('right_device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Convert device paths to numbers
        self.left_device_num = int(left_device.split('video')[-1])
        self.right_device_num = int(right_device.split('video')[-1])
        
        # Initialize cameras
        self.left_cap = cv2.VideoCapture(self.left_device_num)
        self.right_cap = cv2.VideoCapture(self.right_device_num)
        
        # Set camera properties
        for cap in [self.left_cap, self.right_cap]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Check if cameras opened
        if not self.left_cap.isOpened():
            self.get_logger().error(f'Failed to open left camera: {left_device}')
        else:
            self.get_logger().info(f'✓ Left camera opened: {left_device}')
        
        if not self.right_cap.isOpened():
            self.get_logger().error(f'Failed to open right camera: {right_device}')
        else:
            self.get_logger().info(f'✓ Right camera opened: {right_device}')
        
        # Publishers
        self.left_image_pub = self.create_publisher(
            Image, '/camera_rear/left/image_raw', 10)
        self.right_image_pub = self.create_publisher(
            Image, '/camera_rear/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(
            CameraInfo, '/camera_rear/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(
            CameraInfo, '/camera_rear/right/camera_info', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_images)
        
        # Get actual camera properties
        self.actual_width = int(self.left_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_height = int(self.left_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.get_logger().info('='*60)
        self.get_logger().info('Simple Camera Publisher Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Resolution: {self.actual_width}x{self.actual_height}')
        self.get_logger().info(f'Target FPS: {self.fps}')
        self.get_logger().info(f'Publishing to:')
        self.get_logger().info(f'  /camera_rear/left/image_raw')
        self.get_logger().info(f'  /camera_rear/right/image_raw')
        self.get_logger().info('='*60)
        
    def create_camera_info(self, width, height, frame_id):
        """Create a basic CameraInfo message"""
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width = width
        msg.height = height
        
        # Basic camera matrix (no calibration)
        msg.k = [width, 0.0, width/2.0,
                0.0, width, height/2.0,
                0.0, 0.0, 1.0]
        
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        msg.r = [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
        
        msg.p = [width, 0.0, width/2.0, 0.0,
                0.0, width, height/2.0, 0.0,
                0.0, 0.0, 1.0, 0.0]
        
        msg.distortion_model = "plumb_bob"
        
        return msg
        
    def publish_images(self):
        """Capture and publish images from both cameras"""
        timestamp = self.get_clock().now().to_msg()
        
        # Left camera
        if self.left_cap.isOpened():
            ret_left, frame_left = self.left_cap.read()
            if ret_left:
                # Convert to ROS Image message
                img_msg = self.bridge.cv2_to_imgmsg(frame_left, encoding='bgr8')
                img_msg.header.stamp = timestamp
                img_msg.header.frame_id = 'camera_rear_left_optical_frame'
                self.left_image_pub.publish(img_msg)
                
                # Publish camera info
                info_msg = self.create_camera_info(
                    self.actual_width, self.actual_height,
                    'camera_rear_left_optical_frame')
                info_msg.header.stamp = timestamp
                self.left_info_pub.publish(info_msg)
        
        # Right camera
        if self.right_cap.isOpened():
            ret_right, frame_right = self.right_cap.read()
            if ret_right:
                # Convert to ROS Image message
                img_msg = self.bridge.cv2_to_imgmsg(frame_right, encoding='bgr8')
                img_msg.header.stamp = timestamp
                img_msg.header.frame_id = 'camera_rear_right_optical_frame'
                self.right_image_pub.publish(img_msg)
                
                # Publish camera info
                info_msg = self.create_camera_info(
                    self.actual_width, self.actual_height,
                    'camera_rear_right_optical_frame')
                info_msg.header.stamp = timestamp
                self.right_info_pub.publish(info_msg)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down cameras...')
        if self.left_cap.isOpened():
            self.left_cap.release()
        if self.right_cap.isOpened():
            self.right_cap.release()
        self.get_logger().info('✓ Cameras released')


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleCameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()