#!/usr/bin/env python3
"""
Optimized Image Pipeline for Low Bandwidth Streaming
- Receives high-rate images
- Decimates to target rate
- Re-compresses with low quality
- Buffers for smooth playback
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import time
from collections import deque


class OptimizedImagePipeline(Node):
    def __init__(self):
        super().__init__('optimized_image_pipeline')
        
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('input_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('output_topic', '/camera/camera/color/optimized/compressed')
        self.declare_parameter('jpeg_quality', 25)  # Very low for bandwidth
        self.declare_parameter('decimation', 5)  # Only keep every 5th frame
        self.declare_parameter('buffer_delay_sec', 5.0)
        self.declare_parameter('target_fps', 6.0)
        self.declare_parameter('resize_factor', 1.0)  # Optional downscale (0.5 = half size)
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.decimation = self.get_parameter('decimation').value
        self.buffer_delay = self.get_parameter('buffer_delay_sec').value
        target_fps = self.get_parameter('target_fps').value
        self.resize_factor = self.get_parameter('resize_factor').value
        
        self.get_logger().info(f'Optimized pipeline: {input_topic} -> {output_topic}')
        self.get_logger().info(f'JPEG quality: {self.jpeg_quality}%')
        self.get_logger().info(f'Decimation: 1/{self.decimation} frames')
        self.get_logger().info(f'Buffer delay: {self.buffer_delay}s')
        self.get_logger().info(f'Target output: {target_fps} FPS')
        if self.resize_factor != 1.0:
            self.get_logger().info(f'Resize factor: {self.resize_factor}x')
        
        # Frame counter for decimation
        self.frame_count = 0
        
        # Buffer for smooth playback
        self.buffer = deque(maxlen=200)
        
        # Stats
        self.received = 0
        self.processed = 0
        self.published = 0
        
        # Subscriber
        self.subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self.image_callback,
            10)
        
        # Publisher
        self.publisher = self.create_publisher(
            CompressedImage,
            output_topic,
            10)
        
        # Timer for smooth publishing
        publish_period = 1.0 / target_fps
        self.create_timer(publish_period, self.publish_callback)
        
        # Stats timer
        self.create_timer(5.0, self.print_stats)
    
    def image_callback(self, msg):
        """Decimate and recompress incoming images"""
        self.received += 1
        self.frame_count += 1
        
        # Only process every Nth frame
        if self.frame_count % self.decimation != 0:
            return
        
        try:
            # Decode compressed image
            np_arr = cv2.imdecode(
                cv2.UMat(msg.data),
                cv2.IMREAD_COLOR
            )
            
            if np_arr is None:
                self.get_logger().error('Failed to decode image', throttle_duration_sec=5.0)
                return
            
            # Optional resize
            if self.resize_factor != 1.0:
                new_width = int(np_arr.shape[1] * self.resize_factor)
                new_height = int(np_arr.shape[0] * self.resize_factor)
                np_arr = cv2.resize(np_arr, (new_width, new_height))
            
            # Re-compress with low quality
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            success, encoded = cv2.imencode('.jpg', np_arr, encode_params)
            
            if not success:
                self.get_logger().error('Failed to encode image', throttle_duration_sec=5.0)
                return
            
            # Create new compressed message
            new_msg = CompressedImage()
            new_msg.header.stamp = self.get_clock().now().to_msg()
            new_msg.header.frame_id = msg.header.frame_id
            new_msg.format = 'jpeg'
            new_msg.data = encoded.tobytes()
            
            # Add to buffer with timestamp
            current_time = time.time()
            self.buffer.append((current_time, new_msg))
            self.processed += 1
            
        except Exception as e:
            self.get_logger().error(f'Processing error: {str(e)}', throttle_duration_sec=5.0)
    
    def publish_callback(self):
        """Publish from buffer with delay for smooth playback"""
        if not self.buffer:
            return
        
        current_time = time.time()
        
        # Find frame old enough to publish
        while self.buffer:
            receive_time, msg = self.buffer[0]
            age = current_time - receive_time
            
            # Publish if older than buffer delay
            if age >= self.buffer_delay:
                self.buffer.popleft()
                self.publisher.publish(msg)
                self.published += 1
                break  # Only one per timer tick for consistent rate
            else:
                break  # Not old enough yet
    
    def print_stats(self):
        """Print statistics"""
        buffer_size = len(self.buffer)
        
        self.get_logger().info(
            f'Recv: {self.received} | '
            f'Processed: {self.processed} | '
            f'Published: {self.published} | '
            f'Buffer: {buffer_size} frames'
        )
        
        if buffer_size > 0:
            current_time = time.time()
            oldest_age = current_time - self.buffer[0][0]
            newest_age = current_time - self.buffer[-1][0]
            self.get_logger().info(
                f'Buffer timing: oldest={oldest_age:.1f}s, newest={newest_age:.1f}s'
            )
        else:
            self.get_logger().warn('Buffer empty - may need to reduce target FPS')


def main(args=None):
    rclpy.init(args=args)
    pipeline = OptimizedImagePipeline()
    
    try:
        rclpy.spin(pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()