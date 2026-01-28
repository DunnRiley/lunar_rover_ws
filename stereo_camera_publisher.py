#!/usr/bin/env python3
"""
Single-Device Stereo Camera Publisher
For IFWATER cameras that present as ONE device with side-by-side stereo (1600x600)
Automatically splits into left (800x600) and right (800x600) images
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class StereoSingleDevicePublisher(Node):
    def __init__(self):
        super().__init__('stereo_single_device_publisher')
        
        # Parameters
        self.declare_parameter('device', '/dev/video32')
        self.declare_parameter('width', 1600)
        self.declare_parameter('height', 600)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_rate', 30.0)
        
        device = self.get_parameter('device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        publish_rate = self.get_parameter('publish_rate').value
        
        self.bridge = CvBridge()
        
        # Calculate split dimensions
        self.full_width = width
        self.full_height = height
        self.single_width = width // 2  # 800
        self.single_height = height      # 600
        
        # Open camera
        device_num = int(device.split('video')[-1])
        
        self.get_logger().info(f'Opening stereo camera: {device}')
        self.cap = cv2.VideoCapture(device_num, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera: {device}')
            raise RuntimeError(f'Cannot open {device}')
        
        # Set properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        # Get actual resolution
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.get_logger().info(f'Camera opened: {actual_width}x{actual_height}')
        
        # Verify it's a stereo image
        if actual_width < actual_height * 2:
            self.get_logger().warn(
                f'Expected side-by-side stereo (width ~2x height), got {actual_width}x{actual_height}'
            )
        
        # Publishers
        self.left_image_pub = self.create_publisher(
            Image, '/camera_rear/left/image_raw', 10)
        self.right_image_pub = self.create_publisher(
            Image, '/camera_rear/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(
            CameraInfo, '/camera_rear/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(
            CameraInfo, '/camera_rear/right/camera_info', 10)
        
        # Timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_images)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Stereo Camera Publisher Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Input device: {device} ({actual_width}x{actual_height})')
        self.get_logger().info(f'Left output:  /camera_rear/left/image_raw ({self.single_width}x{self.single_height})')
        self.get_logger().info(f'Right output: /camera_rear/right/image_raw ({self.single_width}x{self.single_height})')
        self.get_logger().info('='*60)
    
    def create_camera_info(self, width, height, frame_id):
        """Create a basic CameraInfo message"""
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width = width
        msg.height = height
        
        # Basic camera matrix
        focal_length = width
        
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
    
    def publish_images(self):
        """Capture and split stereo image"""
        if not self.cap.isOpened():
            return
        
        ret, frame = self.cap.read()
        
        if not ret or frame is None:
            self.get_logger().warn('Failed to capture frame', throttle_duration_sec=5.0)
            return
        
        try:
            # Split stereo image
            left_image = frame[:, :self.single_width]   # Left half
            right_image = frame[:, self.single_width:]  # Right half
            
            timestamp = self.get_clock().now().to_msg()
            
            # Publish left image
            left_msg = self.bridge.cv2_to_imgmsg(left_image, encoding='bgr8')
            left_msg.header.stamp = timestamp
            left_msg.header.frame_id = 'camera_rear_left_optical_frame'
            self.left_image_pub.publish(left_msg)
            
            # Publish left info
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
            
            # Publish right info
            right_info = self.create_camera_info(
                self.single_width, self.single_height,
                'camera_rear_right_optical_frame'
            )
            right_info.header.stamp = timestamp
            self.right_info_pub.publish(right_info)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}', throttle_duration_sec=5.0)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down camera...')
        if self.cap.isOpened():
            self.cap.release()
        self.get_logger().info('✓ Camera released')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = StereoSingleDevicePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    except Exception as e:
        print(f'\nError: {e}')
        print('\nMake sure your stereo camera is plugged in and accessible')
    finally:
        if 'node' in locals():
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()