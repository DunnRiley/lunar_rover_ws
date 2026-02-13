#!/usr/bin/env python3
"""
Stereo Image Combiner - Combines left and right stereo images into one side-by-side view
Handles unequal split (adjustable crop parameters)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class StereoCombiner(Node):
    def __init__(self):
        super().__init__('stereo_combiner')
        
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('left_topic', '/camera_rear/left/image_raw')
        self.declare_parameter('right_topic', '/camera_rear/right/image_raw')
        self.declare_parameter('output_topic', '/camera_rear/stereo_combined')
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('left_crop_start', 0)  # Start pixel for left image
        self.declare_parameter('left_crop_width', 800)  # Width of left image
        self.declare_parameter('right_crop_start', 800)  # Start pixel for right image  
        self.declare_parameter('right_crop_width', 800)  # Width of right image
        
        left_topic = self.get_parameter('left_topic').value
        right_topic = self.get_parameter('right_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        
        self.left_crop_start = self.get_parameter('left_crop_start').value
        self.left_crop_width = self.get_parameter('left_crop_width').value
        self.right_crop_start = self.get_parameter('right_crop_start').value
        self.right_crop_width = self.get_parameter('right_crop_width').value
        
        self.get_logger().info(f'Combining stereo: {left_topic} + {right_topic}')
        self.get_logger().info(f'Output: {output_topic}')
        self.get_logger().info(f'Left crop: {self.left_crop_start}:{self.left_crop_start + self.left_crop_width}')
        self.get_logger().info(f'Right crop: {self.right_crop_start}:{self.right_crop_start + self.right_crop_width}')
        
        # Store latest images
        self.left_image = None
        self.right_image = None
        
        # Subscribers
        self.left_sub = self.create_subscription(
            Image,
            left_topic,
            self.left_callback,
            10)
        
        self.right_sub = self.create_subscription(
            Image,
            right_topic,
            self.right_callback,
            10)
        
        # Publisher
        if self.publish_compressed:
            self.publisher = self.create_publisher(
                CompressedImage,
                output_topic + '/compressed',
                10)
            self.get_logger().info('Publishing compressed stereo image')
        else:
            self.publisher = self.create_publisher(
                Image,
                output_topic,
                10)
            self.get_logger().info('Publishing raw stereo image')
        
        # Timer to combine images at fixed rate
        self.create_timer(0.1, self.combine_and_publish)  # 10 Hz
    
    def left_callback(self, msg):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Left image error: {str(e)}')
    
    def right_callback(self, msg):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Right image error: {str(e)}')
    
    def combine_and_publish(self):
        if self.left_image is None or self.right_image is None:
            return
        
        try:
            # Crop images to get proper left and right views
            height = self.left_image.shape[0]
            
            # Crop left image
            left_cropped = self.left_image[:, 
                self.left_crop_start:self.left_crop_start + self.left_crop_width]
            
            # Crop right image  
            right_cropped = self.right_image[:, 
                self.right_crop_start:self.right_crop_start + self.right_crop_width]
            
            # Make sure both are same height
            if left_cropped.shape[0] != right_cropped.shape[0]:
                min_height = min(left_cropped.shape[0], right_cropped.shape[0])
                left_cropped = left_cropped[:min_height, :]
                right_cropped = right_cropped[:min_height, :]
            
            # Combine side by side
            combined = np.hstack([left_cropped, right_cropped])
            
            # Add labels
            cv2.putText(combined, 'LEFT', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(combined, 'RIGHT', 
                       (self.left_crop_width + 10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Publish
            if self.publish_compressed:
                # Compress as JPEG
                success, encoded = cv2.imencode('.jpg', combined, 
                    [cv2.IMWRITE_JPEG_QUALITY, 60])
                
                if success:
                    msg = CompressedImage()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'camera_rear_link'
                    msg.format = 'jpeg'
                    msg.data = encoded.tobytes()
                    self.publisher.publish(msg)
            else:
                # Publish raw
                msg = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_rear_link'
                self.publisher.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Combine error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    combiner = StereoCombiner()
    
    try:
        rclpy.spin(combiner)
    except KeyboardInterrupt:
        pass
    finally:
        combiner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()