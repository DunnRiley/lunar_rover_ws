#!/usr/bin/env python3
"""
Improved Camera Publisher for ROS2
Handles device detection issues and provides better error messages
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
import subprocess


class ImprovedCameraPublisher(Node):
    def __init__(self):
        super().__init__('improved_camera_publisher')
        
        # Parameters
        self.declare_parameter('left_device', '')
        self.declare_parameter('right_device', '')
        self.declare_parameter('auto_detect', True)
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_rate', 30.0)
        
        # Get parameters
        left_device = self.get_parameter('left_device').value
        right_device = self.get_parameter('right_device').value
        auto_detect = self.get_parameter('auto_detect').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        publish_rate = self.get_parameter('publish_rate').value
        
        self.bridge = CvBridge()
        
        # Auto-detect devices if not specified
        if auto_detect and (not left_device or not right_device):
            self.get_logger().info('Auto-detecting camera devices...')
            devices = self.find_working_devices()
            
            if len(devices) >= 2:
                left_device = devices[0]
                right_device = devices[1]
                self.get_logger().info(f'Auto-detected: Left={left_device}, Right={right_device}')
            elif len(devices) == 1:
                self.get_logger().warn(f'Only found 1 camera: {devices[0]}')
                left_device = devices[0]
                right_device = ''
            else:
                self.get_logger().error('No working cameras found!')
                raise RuntimeError('No cameras detected')
        
        # Initialize cameras
        self.left_cap = None
        self.right_cap = None
        self.actual_width = width
        self.actual_height = height
        
        if left_device:
            self.left_cap = self.open_camera(left_device, width, height, 'Left')
        
        if right_device:
            self.right_cap = self.open_camera(right_device, width, height, 'Right')
        
        if not self.left_cap and not self.right_cap:
            self.get_logger().error('Failed to open any cameras!')
            raise RuntimeError('No cameras could be opened')
        
        # Publishers
        if self.left_cap:
            self.left_image_pub = self.create_publisher(
                Image, '/camera_rear/left/image_raw', 10)
            self.left_info_pub = self.create_publisher(
                CameraInfo, '/camera_rear/left/camera_info', 10)
        
        if self.right_cap:
            self.right_image_pub = self.create_publisher(
                Image, '/camera_rear/right/image_raw', 10)
            self.right_info_pub = self.create_publisher(
                CameraInfo, '/camera_rear/right/camera_info', 10)
        
        # Timer
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_images)
        
        self.get_logger().info('='*60)
        self.get_logger().info('Camera Publisher Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Resolution: {self.actual_width}x{self.actual_height}')
        self.get_logger().info(f'Target FPS: {self.fps}')
        
        if self.left_cap:
            self.get_logger().info('Publishing: /camera_rear/left/image_raw')
        if self.right_cap:
            self.get_logger().info('Publishing: /camera_rear/right/image_raw')
        
        self.get_logger().info('='*60)
    
    def find_working_devices(self):
        """Find working V4L2 capture devices"""
        working_devices = []
        
        # Check /dev/video0 through /dev/video33
        for i in range(34):
            device_path = f'/dev/video{i}'
            
            if not os.path.exists(device_path):
                continue
            
            # Try to open with OpenCV
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
            
            if cap.isOpened():
                # Try to read a frame
                ret, frame = cap.read()
                
                if ret and frame is not None:
                    self.get_logger().info(f'✓ Found working device: {device_path}')
                    working_devices.append(device_path)
                
                cap.release()
        
        return working_devices
    
    def open_camera(self, device_path, width, height, name):
        """Try to open a camera device"""
        self.get_logger().info(f'Opening {name} camera: {device_path}')
        
        # Extract device number
        try:
            device_num = int(device_path.split('video')[-1])
        except:
            self.get_logger().error(f'Invalid device path: {device_path}')
            return None
        
        # Try V4L2 backend first
        cap = cv2.VideoCapture(device_num, cv2.CAP_V4L2)
        
        if not cap.isOpened():
            self.get_logger().warn(f'V4L2 failed, trying default backend...')
            cap = cv2.VideoCapture(device_num)
        
        if not cap.isOpened():
            self.get_logger().error(f'Failed to open {name} camera: {device_path}')
            return None
        
        # Set properties
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Try to read a test frame
        ret, frame = cap.read()
        
        if not ret:
            self.get_logger().error(f'{name} camera opened but cannot read frames')
            cap.release()
            return None
        
        # Get actual resolution
        self.actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.get_logger().info(f'✓ {name} camera opened: {self.actual_width}x{self.actual_height}')
        
        return cap
    
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
        if self.left_cap and self.left_cap.isOpened():
            ret_left, frame_left = self.left_cap.read()
            if ret_left:
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(frame_left, encoding='bgr8')
                    img_msg.header.stamp = timestamp
                    img_msg.header.frame_id = 'camera_rear_left_optical_frame'
                    self.left_image_pub.publish(img_msg)
                    
                    info_msg = self.create_camera_info(
                        self.actual_width, self.actual_height,
                        'camera_rear_left_optical_frame')
                    info_msg.header.stamp = timestamp
                    self.left_info_pub.publish(info_msg)
                except Exception as e:
                    self.get_logger().error(f'Left camera error: {e}', throttle_duration_sec=5.0)
        
        # Right camera
        if self.right_cap and self.right_cap.isOpened():
            ret_right, frame_right = self.right_cap.read()
            if ret_right:
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(frame_right, encoding='bgr8')
                    img_msg.header.stamp = timestamp
                    img_msg.header.frame_id = 'camera_rear_right_optical_frame'
                    self.right_image_pub.publish(img_msg)
                    
                    info_msg = self.create_camera_info(
                        self.actual_width, self.actual_height,
                        'camera_rear_right_optical_frame')
                    info_msg.header.stamp = timestamp
                    self.right_info_pub.publish(info_msg)
                except Exception as e:
                    self.get_logger().error(f'Right camera error: {e}', throttle_duration_sec=5.0)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down cameras...')
        if self.left_cap and self.left_cap.isOpened():
            self.left_cap.release()
        if self.right_cap and self.right_cap.isOpened():
            self.right_cap.release()
        self.get_logger().info('✓ Cameras released')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImprovedCameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    except Exception as e:
        print(f'\nError: {e}')
        print('\nTroubleshooting:')
        print('1. Run: chmod +x find_camera_devices.sh && ./find_camera_devices.sh')
        print('2. Use the devices it recommends')
        print('3. Make sure no other programs are using the cameras')
    finally:
        if 'node' in locals():
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()