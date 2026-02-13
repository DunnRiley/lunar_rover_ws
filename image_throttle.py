#!/usr/bin/env python3
"""
Image Throttle Node - Reduces image publishing rate for network streaming
This republishes images at a lower rate (e.g., every 3rd frame)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import time


class ImageThrottle(Node):
    def __init__(self):
        super().__init__('image_throttle')
        
        # Parameters
        self.declare_parameter('decimation', 3)  # Publish every Nth frame
        self.declare_parameter('input_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('output_topic', '/camera/camera/color/image_raw/throttled')
        self.declare_parameter('use_compressed', True)  # Subscribe to compressed topics
        
        decimation = self.get_parameter('decimation').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        use_compressed = self.get_parameter('use_compressed').value
        
        self.frame_count = 0
        self.decimation = decimation
        
        # Choose message type based on compressed flag
        if use_compressed:
            msg_type = CompressedImage
            input_topic = input_topic + '/compressed'
            self.get_logger().info(f'Throttling compressed images: {input_topic}')
        else:
            msg_type = Image
            self.get_logger().info(f'Throttling raw images: {input_topic}')
        
        self.get_logger().info(f'Publishing every {decimation} frame(s)')
        self.get_logger().info(f'Output: {output_topic}')
        
        # Subscriber and publisher
        self.subscription = self.create_subscription(
            msg_type,
            input_topic,
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            msg_type,
            output_topic,
            10)
        
        # Stats
        self.last_pub_time = time.time()
    
    def image_callback(self, msg):
        self.frame_count += 1
        
        # Only publish every Nth frame
        if self.frame_count % self.decimation == 0:
            self.publisher.publish(msg)
            
            # Log rate
            now = time.time()
            dt = now - self.last_pub_time
            if dt > 0:
                rate = 1.0 / dt
                self.get_logger().info(f'Publishing at {rate:.1f} Hz', throttle_duration_sec=5.0)
            self.last_pub_time = now


def main(args=None):
    rclpy.init(args=args)
    throttle = ImageThrottle()
    
    try:
        rclpy.spin(throttle)
    except KeyboardInterrupt:
        pass
    finally:
        throttle.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()