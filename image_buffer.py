#!/usr/bin/env python3
"""
Image Buffer Node - Adds delay buffer to smooth out bursty network transmission
This allows accumulating images in a buffer and playing them back smoothly with delay
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from collections import deque
import time


class ImageBuffer(Node):
    def __init__(self):
        super().__init__('image_buffer')
        
        # Parameters
        self.declare_parameter('input_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('output_topic', '/camera/camera/color/buffered/compressed')
        self.declare_parameter('buffer_delay_sec', 5.0)  # 5 second delay buffer
        self.declare_parameter('target_fps', 6.0)  # Target playback FPS
        self.declare_parameter('use_compressed', True)
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.buffer_delay = self.get_parameter('buffer_delay_sec').value
        target_fps = self.get_parameter('target_fps').value
        use_compressed = self.get_parameter('use_compressed').value
        
        self.get_logger().info(f'Buffering: {input_topic} -> {output_topic}')
        self.get_logger().info(f'Buffer delay: {self.buffer_delay} seconds')
        self.get_logger().info(f'Target playback: {target_fps} FPS')
        
        # Message buffer with timestamps
        self.buffer = deque()
        self.max_buffer_size = 200  # Prevent runaway memory
        
        # Choose message type
        if use_compressed:
            msg_type = CompressedImage
        else:
            msg_type = Image
        
        # Subscriber
        self.subscription = self.create_subscription(
            msg_type,
            input_topic,
            self.image_callback,
            10)
        
        # Publisher
        self.publisher = self.create_publisher(
            msg_type,
            output_topic,
            10)
        
        # Timer for publishing at target FPS
        publish_period = 1.0 / target_fps
        self.create_timer(publish_period, self.publish_callback)
        
        # Stats
        self.last_receive_time = time.time()
        self.receive_count = 0
        self.publish_count = 0
        
        # Timer for stats
        self.create_timer(5.0, self.print_stats)
    
    def image_callback(self, msg):
        """Store incoming messages with timestamp"""
        current_time = time.time()
        
        # Add to buffer with receive timestamp
        self.buffer.append((current_time, msg))
        self.receive_count += 1
        
        # Limit buffer size
        if len(self.buffer) > self.max_buffer_size:
            self.buffer.popleft()
            self.get_logger().warn(f'Buffer overflow - dropping old frames', 
                                  throttle_duration_sec=5.0)
    
    def publish_callback(self):
        """Publish frames from buffer with delay"""
        if not self.buffer:
            return
        
        current_time = time.time()
        
        # Find frames that are old enough to publish (past the delay)
        while self.buffer:
            receive_time, msg = self.buffer[0]
            age = current_time - receive_time
            
            # If frame is older than buffer delay, publish it
            if age >= self.buffer_delay:
                self.buffer.popleft()
                self.publisher.publish(msg)
                self.publish_count += 1
                break  # Only publish one frame per timer tick
            else:
                # Frame not old enough yet
                break
    
    def print_stats(self):
        """Print buffer statistics"""
        buffer_size = len(self.buffer)
        
        if buffer_size > 0:
            current_time = time.time()
            oldest_age = current_time - self.buffer[0][0]
            newest_age = current_time - self.buffer[-1][0]
            
            self.get_logger().info(
                f'Buffer: {buffer_size} frames | '
                f'Oldest: {oldest_age:.1f}s | '
                f'Newest: {newest_age:.1f}s | '
                f'Received: {self.receive_count} | '
                f'Published: {self.publish_count}'
            )
        else:
            self.get_logger().warn('Buffer empty - not receiving images')


def main(args=None):
    rclpy.init(args=args)
    buffer_node = ImageBuffer()
    
    try:
        rclpy.spin(buffer_node)
    except KeyboardInterrupt:
        pass
    finally:
        buffer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()