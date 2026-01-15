#!/usr/bin/env python3
"""
Camera Rotation Controller - OOP Style
Subscribes to a single rotation command and publishes to both cameras
Similar to the motor controller pattern
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class CameraRotationController(Node):
    """
    Camera rotation controller that synchronizes both cameras
    Input: /camera_rotation_command (Float64)
    Output: Commands to both front and rear camera joints
    """
    
    def __init__(self):
        super().__init__('camera_rotation_controller')
        
        # Subscriber for rotation commands
        self.rotation_sub = self.create_subscription(
            Float64,
            '/camera_rotation_command',
            self.rotation_callback,
            10
        )
        
        # Publishers to Gazebo joint controllers
        self.front_camera_pub = self.create_publisher(
            Float64,
            '/camera_rotation',  # Matches Gazebo plugin topic
            10
        )
        
        self.rear_camera_pub = self.create_publisher(
            Float64,
            '/camera_rear_rotation',  # Matches Gazebo plugin topic
            10
        )
        
        # State
        self.current_angle = 0.0
        
        self.get_logger().info('Camera Rotation Controller initialized')
        self.get_logger().info('  Subscribing to: /camera_rotation_command')
        self.get_logger().info('  Publishing to: /camera_rotation, /camera_rear_rotation')
    
    def rotation_callback(self, msg):
        """Handle rotation command and send to both cameras"""
        self.current_angle = msg.data
        
        # Create message for both cameras
        rotation_msg = Float64()
        rotation_msg.data = self.current_angle
        
        # Publish to both cameras
        self.front_camera_pub.publish(rotation_msg)
        self.rear_camera_pub.publish(rotation_msg)
        
        degrees = self.current_angle * 57.2958
        self.get_logger().info(
            f'Camera rotation: {degrees:.1f}° ({self.current_angle:.2f} rad)',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    
    controller = CameraRotationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
