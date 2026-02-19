#!/usr/bin/env python3
"""
Optimized Image Pipeline for Low Bandwidth Streaming
- Receives compressed images from camera
- Decimates to target rate
- Re-compresses with lower quality JPEG
- Buffers for smooth playback with configurable delay

FIXES vs old version:
- Correct numpy decode of CompressedImage (was broken with cv2.UMat)
- Handles both compressed and raw input
- Better error recovery
- Cleaner stats logging
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import deque


class OptimizedImagePipeline(Node):
    def __init__(self):
        super().__init__('optimized_image_pipeline')

        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('input_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('output_topic', '/camera/camera/color/optimized/compressed')
        self.declare_parameter('jpeg_quality', 25)
        self.declare_parameter('decimation', 5)       # Keep every Nth frame
        self.declare_parameter('buffer_delay_sec', 1.0)  # Delay before publishing
        self.declare_parameter('target_fps', 6.0)
        self.declare_parameter('resize_factor', 1.0)
        self.declare_parameter('input_is_compressed', True)  # True = CompressedImage, False = Image

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.decimation = self.get_parameter('decimation').value
        self.buffer_delay = self.get_parameter('buffer_delay_sec').value
        target_fps = self.get_parameter('target_fps').value
        self.resize_factor = self.get_parameter('resize_factor').value
        input_is_compressed = self.get_parameter('input_is_compressed').value

        self.get_logger().info(f'Pipeline: {input_topic} → {output_topic}')
        self.get_logger().info(f'JPEG quality: {self.jpeg_quality}%  Decimation: 1/{self.decimation}')
        self.get_logger().info(f'Buffer delay: {self.buffer_delay}s  Target FPS: {target_fps}')

        self.frame_count = 0
        self.buffer = deque(maxlen=300)

        # Stats
        self.received = 0
        self.processed = 0
        self.published = 0
        self.decode_errors = 0

        # Subscriber - support both raw and compressed input
        if input_is_compressed:
            self.subscription = self.create_subscription(
                CompressedImage, input_topic, self.compressed_callback, 10)
        else:
            self.subscription = self.create_subscription(
                Image, input_topic, self.raw_callback, 10)

        # Publisher always outputs CompressedImage
        self.publisher = self.create_publisher(CompressedImage, output_topic, 10)

        publish_period = 1.0 / target_fps
        self.create_timer(publish_period, self.publish_callback)
        self.create_timer(5.0, self.print_stats)

    def compressed_callback(self, msg: CompressedImage):
        """Handle incoming CompressedImage messages."""
        self.received += 1
        self.frame_count += 1

        if self.frame_count % self.decimation != 0:
            return

        try:
            # FIXED: correct way to decode a ROS CompressedImage
            np_arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                self.decode_errors += 1
                return

            self._process_and_buffer(frame, msg.header.frame_id)

        except Exception as e:
            self.decode_errors += 1
            self.get_logger().error(f'Decode error: {e}', throttle_duration_sec=5.0)

    def raw_callback(self, msg: Image):
        """Handle incoming raw Image messages (color or depth)."""
        self.received += 1
        self.frame_count += 1

        if self.frame_count % self.decimation != 0:
            return

        try:
            # Depth images are 16UC1 — use passthrough then convert to a
            # displayable 8-bit colormap so they can be JPEG-encoded normally.
            if msg.encoding in ('16UC1', '32FC1'):
                raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                # Normalize to 0-255 for JPEG encoding (clip at 4m = 4000mm)
                clipped = np.clip(raw, 0, 4000).astype(np.float32)
                normalized = (clipped / 4000.0 * 255).astype(np.uint8)
                frame = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            self._process_and_buffer(frame, msg.header.frame_id)
        except Exception as e:
            self.decode_errors += 1
            self.get_logger().error(f'CV bridge error: {e}', throttle_duration_sec=5.0)

    def _process_and_buffer(self, frame: np.ndarray, frame_id: str):
        """Resize, re-compress, and add to delay buffer."""
        try:
            if self.resize_factor != 1.0:
                new_w = int(frame.shape[1] * self.resize_factor)
                new_h = int(frame.shape[0] * self.resize_factor)
                frame = cv2.resize(frame, (new_w, new_h))

            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            success, encoded = cv2.imencode('.jpg', frame, encode_params)

            if not success:
                return

            new_msg = CompressedImage()
            new_msg.header.stamp = self.get_clock().now().to_msg()
            new_msg.header.frame_id = frame_id
            new_msg.format = 'jpeg'
            new_msg.data = encoded.tobytes()

            self.buffer.append((time.time(), new_msg))
            self.processed += 1

        except Exception as e:
            self.get_logger().error(f'Processing error: {e}', throttle_duration_sec=5.0)

    def publish_callback(self):
        """Publish the oldest buffered frame once it has aged past buffer_delay."""
        if not self.buffer:
            return

        current_time = time.time()
        if current_time - self.buffer[0][0] >= self.buffer_delay:
            _, msg = self.buffer.popleft()
            self.publisher.publish(msg)
            self.published += 1

    def print_stats(self):
        buf = len(self.buffer)
        self.get_logger().info(
            f'recv={self.received} proc={self.processed} pub={self.published} '
            f'buf={buf} errs={self.decode_errors}'
        )
        if buf > 0:
            now = time.time()
            self.get_logger().info(
                f'Buffer: oldest={now - self.buffer[0][0]:.1f}s '
                f'newest={now - self.buffer[-1][0]:.1f}s'
            )
        elif self.received == 0:
            self.get_logger().warn('No frames received yet — check input topic name')


def main(args=None):
    rclpy.init(args=args)
    node = OptimizedImagePipeline()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Already shut down — harmless


if __name__ == '__main__':
    main()
