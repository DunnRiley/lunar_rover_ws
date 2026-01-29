#!/usr/bin/env python3
"""
Stereo Image Splitter for Side-by-Side Stereo Camera
Takes a single 1600x600 image and splits it into left (800x600) and right (800x600)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class StereoImageSplitter(Node):
    def __init__(self):
        super().__init__('stereo_image_splitter')
        
        # Parameters
        self.declare_parameter('input_topic', '/camera_rear/left/image_raw')
        self.declare_parameter('stereo_width', 1600)
        self.declare_parameter('stereo_height', 600)
        
        input_topic = self.get_parameter('input_topic').value
        self.stereo_width = self.get_parameter('stereo_width').value
        self.stereo_height = self.get_parameter('stereo_height').value
        
        # Calculate split dimensions
        self.single_width = self.stereo_width // 2  # 800
        self.single_height = self.stereo_height      # 600
        
        self.bridge = CvBridge()
        
        # Subscribe to the combined stereo image
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        # Publishers for split images
        self.left_image_pub = self.create_publisher(
            Image, '/camera_rear/left/image_raw', 10)
        self.right_image_pub = self.create_publisher(
            Image, '/camera_rear/right/image_raw', 10)
        
        self.left_info_pub = self.create_publisher(
            CameraInfo, '/camera_rear/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(
            CameraInfo, '/camera_rear/right/camera_info', 10)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Stereo Image Splitter Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Input: {input_topic} ({self.stereo_width}x{self.stereo_height})')
        self.get_logger().info(f'Output Left:  /camera_rear/left/image_raw ({self.single_width}x{self.single_height})')
        self.get_logger().info(f'Output Right: /camera_rear/right/image_raw ({self.single_width}x{self.single_height})')
        self.get_logger().info('='*60)
    
    def create_camera_info(self, width, height, frame_id):
        """Create a basic CameraInfo message"""
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width = width
        msg.height = height
        
        # Basic camera matrix (no calibration)
        focal_length = width  # Rough estimate
        
        msg.k = [focal_length, 0.0, width/2.0,
                0.0, focal_length, height/2.0,
                0.0, 0.0, 1.0]
        
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        msg.r = [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
        
        msg.p = [focal_length, 0.0, width/2.0, 0.0,
                0.0, focal_length, height/2.0, 0.0,
                0.0, 0.0, 1.0, 0.0]
        
        msg.distortion_model = "plumb_bob"
        
        return msg
    
    def image_callback(self, msg):
        """Split the stereo image into left and right"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Verify dimensions
            if cv_image.shape[1] != self.stereo_width or cv_image.shape[0] != self.stereo_height:
                self.get_logger().warn(
                    f'Unexpected image size: {cv_image.shape[1]}x{cv_image.shape[0]}',
                    throttle_duration_sec=5.0
                )
                return
            
            # Split image: left half and right half
            left_image = cv_image[:, :self.single_width]  # Left 800 pixels
            right_image = cv_image[:, self.single_width:]  # Right 800 pixels
            
            timestamp = self.get_clock().now().to_msg()
            
            # Publish left image
            left_msg = self.bridge.cv2_to_imgmsg(left_image, encoding='bgr8')
            left_msg.header.stamp = timestamp
            left_msg.header.frame_id = 'camera_rear_left_optical_frame'
            self.left_image_pub.publish(left_msg)
            
            # Publish left camera info
            left_info = self.create_camera_info(
                self.single_width, self.single_height,
                'camera_rear_left_optical_frame'
            )
            left_info.header.stamp = timestamp
            self.left_info_pub.publish(left_info)
            
            # Publish right image
            right_msg = self.bridge.cv2_to_imgmsg(right_image, encoding='bgr8')
            right_msg.header.stamp = timestamp
            right_msg.header.frame_id = 'camera_rear_right_optical_frame'
            self.right_image_pub.publish(right_msg)
            
            # Publish right camera info
            right_info = self.create_camera_info(
                self.single_width, self.single_height,
                'camera_rear_right_optical_frame'
            )
            right_info.header.stamp = timestamp
            self.right_info_pub.publish(right_info)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = StereoImageSplitter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()