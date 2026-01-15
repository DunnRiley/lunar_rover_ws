#!/usr/bin/env python3
"""
FIXED Camera Info Publisher
Subscribes to depth image and publishes matching camera_info with SAME timestamp
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        self.msg_count = 0
        
        # Subscribe to depth image to get timestamps
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        # Publisher for camera info
        self.pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        
        self.get_logger().info('Camera Info Publisher started (timestamp-synchronized)')
        self.get_logger().info('  Subscribing to: /camera/depth/image_raw')
        self.get_logger().info('  Publishing to: /camera/depth/camera_info')
        
    def depth_callback(self, depth_msg):
        """Publish camera_info with SAME timestamp as depth image"""
        
        # Log first message
        if self.msg_count == 0:
            self.get_logger().info(f'✅ Receiving depth images!')
            self.get_logger().info(f'   Frame: {depth_msg.header.frame_id}')
            self.get_logger().info(f'   Size: {depth_msg.width}x{depth_msg.height}')
        self.msg_count += 1
        
        msg = CameraInfo()
        
        # CRITICAL: Use the EXACT same timestamp as depth image
        msg.header.stamp = depth_msg.header.stamp
        
        # Use the remapped frame name that matches the URDF
        msg.header.frame_id = 'camera_rear_depth_optical_frame'
        
        # D435 camera parameters (640x480)
        msg.height = 480
        msg.width = 640
        
        # Camera intrinsics
        fx = 320.0
        fy = 320.0
        cx = 320.0
        cy = 240.0
        
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        msg.binning_x = 0
        msg.binning_y = 0
        
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CameraInfoPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()