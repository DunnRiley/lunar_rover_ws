#!/usr/bin/env python3
"""
optimized_image_pipeline.py  —  Camera streaming pipeline for miniPC → laptop

CRITICAL FIX: RealSense D435 publishes /camera/camera/color/image_raw/compressed
with RELIABLE QoS. Previous version used BEST_EFFORT input QoS → zero frames received.
This version tries RELIABLE first, auto-retries with BEST_EFFORT if no frames.

Also fixes: output QoS is BEST_EFFORT + VOLATILE which is what RViz Image needs.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import deque


class OptimizedImagePipeline(Node):
    def __init__(self):
        super().__init__('optimized_image_pipeline')
        self.bridge = CvBridge()

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('input_topic',
                               '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('output_topic',
                               '/camera/color/stream/compressed')
        self.declare_parameter('jpeg_quality',        30)
        self.declare_parameter('decimation',           5)
        self.declare_parameter('buffer_delay_sec',   0.0)
        self.declare_parameter('target_fps',          6.0)
        self.declare_parameter('resize_factor',       1.0)
        self.declare_parameter('input_is_compressed', True)
        # Try RELIABLE first (RealSense compressed = RELIABLE)
        # Set to false if you know input is BEST_EFFORT
        self.declare_parameter('input_reliable',      True)

        self.input_topic         = self.get_parameter('input_topic').value
        self.output_topic        = self.get_parameter('output_topic').value
        self.jpeg_quality        = self.get_parameter('jpeg_quality').value
        self.decimation          = self.get_parameter('decimation').value
        self.buffer_delay        = self.get_parameter('buffer_delay_sec').value
        target_fps               = self.get_parameter('target_fps').value
        self.resize_factor       = self.get_parameter('resize_factor').value
        self.input_is_compressed = self.get_parameter('input_is_compressed').value
        input_reliable           = self.get_parameter('input_reliable').value

        self.get_logger().info('=' * 64)
        self.get_logger().info(f'  IN  : {self.input_topic}')
        self.get_logger().info(f'  OUT : {self.output_topic}')
        self.get_logger().info(
            f'  JPEG={self.jpeg_quality}%  decim=1/{self.decimation}  '
            f'delay={self.buffer_delay}s  fps={target_fps}  '
            f'compressed_in={self.input_is_compressed}  '
            f'reliable_in={input_reliable}')
        self.get_logger().info('=' * 64)

        self.frame_count     = 0
        self.buffer          = deque(maxlen=300)
        self.received        = 0
        self.processed       = 0
        self.published       = 0
        self.decode_errors   = 0
        self._last_recv      = time.time()
        self._sub            = None
        self._sub_start      = time.time()
        self._using_reliable = input_reliable

        # ── Output QoS: BEST_EFFORT + VOLATILE ───────────────────────────
        # RViz Image display requires BEST_EFFORT + VOLATILE
        self._out_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            CompressedImage, self.output_topic, self._out_qos)
        self.hb_pub = self.create_publisher(
            String, '/rover/pipeline_hb', 10)

        # ── Subscribe ─────────────────────────────────────────────────────
        self._subscribe(reliable=input_reliable)

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(1.0 / target_fps, self._publish_cb)
        self.create_timer(5.0, self._stats_cb)
        self.create_timer(1.0, self._heartbeat_cb)

    # ── Subscription management ───────────────────────────────────────────

    def _subscribe(self, reliable: bool):
        if self._sub is not None:
            self.destroy_subscription(self._sub)

        rel   = ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT
        label = 'RELIABLE' if reliable else 'BEST_EFFORT'

        qos = QoSProfile(
            reliability=rel,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.get_logger().info(
            f'Subscribing {self.input_topic}  QoS={label}')

        if self.input_is_compressed:
            self._sub = self.create_subscription(
                CompressedImage, self.input_topic, self._compressed_cb, qos)
        else:
            self._sub = self.create_subscription(
                Image, self.input_topic, self._raw_cb, qos)

        self._using_reliable = reliable
        self._sub_start      = time.time()

    # ── Input callbacks ───────────────────────────────────────────────────

    def _compressed_cb(self, msg: CompressedImage):
        self.received   += 1
        self._last_recv  = time.time()
        self.frame_count += 1
        if self.frame_count % self.decimation != 0:
            return
        try:
            arr   = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.decode_errors += 1
                return
            self._process(frame, msg.header.frame_id)
        except Exception as e:
            self.decode_errors += 1
            self.get_logger().error(f'Decode: {e}', throttle_duration_sec=5.0)

    def _raw_cb(self, msg: Image):
        self.received   += 1
        self._last_recv  = time.time()
        self.frame_count += 1
        if self.frame_count % self.decimation != 0:
            return
        try:
            if msg.encoding in ('16UC1', '32FC1'):
                raw   = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                clip  = np.clip(raw, 0, 4000).astype(np.float32)
                norm  = (clip / 4000.0 * 255).astype(np.uint8)
                frame = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._process(frame, msg.header.frame_id)
        except Exception as e:
            self.decode_errors += 1
            self.get_logger().error(f'Bridge: {e}', throttle_duration_sec=5.0)

    # ── Processing ────────────────────────────────────────────────────────

    def _process(self, frame: np.ndarray, frame_id: str):
        try:
            if self.resize_factor != 1.0:
                w = int(frame.shape[1] * self.resize_factor)
                h = int(frame.shape[0] * self.resize_factor)
                frame = cv2.resize(frame, (w, h))
            ok, enc = cv2.imencode(
                '.jpg', frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not ok:
                return
            out                 = CompressedImage()
            out.header.stamp    = self.get_clock().now().to_msg()
            out.header.frame_id = frame_id
            out.format          = 'jpeg'
            out.data            = enc.tobytes()
            self.buffer.append((time.time(), out))
            self.processed += 1
        except Exception as e:
            self.get_logger().error(f'Process: {e}', throttle_duration_sec=5.0)

    # ── Publish ───────────────────────────────────────────────────────────

    def _publish_cb(self):
        if self.buffer and time.time() - self.buffer[0][0] >= self.buffer_delay:
            _, msg = self.buffer.popleft()
            self.publisher.publish(msg)
            self.published += 1

    def _heartbeat_cb(self):
        msg      = String()
        msg.data = (f'{self.output_topic}  recv={self.received} '
                    f'pub={self.published} buf={len(self.buffer)}')
        self.hb_pub.publish(msg)

    # ── Stats + QoS watchdog ──────────────────────────────────────────────

    def _stats_cb(self):
        idle = time.time() - self._last_recv
        self.get_logger().info(
            f'recv={self.received}/5s  proc={self.processed}  '
            f'pub={self.published}  buf={len(self.buffer)}  '
            f'errs={self.decode_errors}  idle={idle:.1f}s')

        # If nothing is arriving, flip QoS and retry
        if self.received == 0:
            elapsed = time.time() - self._sub_start
            if elapsed > 5.0:
                alt = not self._using_reliable
                self.get_logger().warn(
                    f'No frames from {self.input_topic} after {elapsed:.0f}s '
                    f'(QoS={"RELIABLE" if self._using_reliable else "BEST_EFFORT"})\n'
                    f'  → retrying with '
                    f'{"BEST_EFFORT" if self._using_reliable else "RELIABLE"}\n'
                    f'  → debug: ros2 topic info -v {self.input_topic}')
                self._subscribe(reliable=alt)
        else:
            self.received     = 0
            self.processed    = 0
            self.published    = 0
            self.decode_errors = 0


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
            pass


if __name__ == '__main__':
    main()