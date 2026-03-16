#!/usr/bin/env python3
"""
Stereo Camera Publisher
Captures a side-by-side stereo USB camera and publishes left/right as
separate ROS2 Image topics.

USB stereo cameras often appear as two v4l2 nodes, e.g. /dev/video6 (capture)
and /dev/video7 (metadata). This script auto-detects the working one.

Topics published:
  /camera_rear/left/image_raw   sensor_msgs/Image
  /camera_rear/right/image_raw  sensor_msgs/Image
"""

import glob
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


def find_working_device(device_str: str) -> str | None:
    """
    Find the actual v4l2 capture device that OpenCV can read frames from.

    USB stereo cameras often register two nodes:
      /dev/video6  — capture device  (works with OpenCV)
      /dev/video7  — metadata node   (opens but returns no frames)

    Strategy:
      1. Resolve any symlink  (/dev/video_stereo -> /dev/video6 or video7)
      2. Try that path directly (pass full string, NOT integer index)
      3. Try the sibling device  (6 <-> 7,  4 <-> 5)
      4. Scan /dev/video4 through /dev/video9 as a last resort

    A device is "working" only if it opens AND returns a real frame.
    """
    real = os.path.realpath(device_str)
    candidates = [real]

    # Add sibling (XOR last bit: 6<->7, 4<->5, 8<->9)
    basename = os.path.basename(real)
    try:
        idx = int(basename.replace('video', ''))
        sibling = f'/dev/video{idx ^ 1}'
        if sibling not in candidates:
            candidates.append(sibling)
        # Always try the lower of the pair first
        lower = f'/dev/video{min(idx, idx ^ 1)}'
        if lower not in candidates:
            candidates.insert(0, lower)
    except ValueError:
        pass

    # Broad fallback
    for d in sorted(glob.glob('/dev/video*')):
        if d not in candidates:
            candidates.append(d)

    print(f'[stereo] Trying devices in order: {candidates}')

    for path in candidates:
        if not os.path.exists(path):
            continue
        # Use full path string — avoids the int() parsing bug entirely
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            print(f'[stereo]   {path}  -> could not open')
            continue
        ret, frame = cap.read()
        cap.release()
        if ret and frame is not None:
            print(f'[stereo]   {path}  -> OK ({frame.shape[1]}x{frame.shape[0]})')
            return path
        print(f'[stereo]   {path}  -> opened but no frames')

    return None


class StereoCameraPublisher(Node):
    def __init__(self):
        super().__init__('stereo_camera_publisher')

        # Parameters
        self.declare_parameter('device',       '/dev/video_stereo')
        self.declare_parameter('width',        1600)
        self.declare_parameter('height',       600)
        self.declare_parameter('fps',          15)
        self.declare_parameter('publish_rate', 15.0)

        device_param = self.get_parameter('device').value
        width        = self.get_parameter('width').value
        height       = self.get_parameter('height').value
        fps          = self.get_parameter('fps').value
        publish_rate = self.get_parameter('publish_rate').value

        self.bridge = CvBridge()

        # Find a working capture device
        self.get_logger().info(f'Searching for stereo camera from: {device_param}')
        working = find_working_device(device_param)

        if working is None:
            self.get_logger().error('No working stereo capture device found.')
            self.get_logger().error('Tried symlink, sibling, and /dev/video0-9.')
            raise RuntimeError('Cannot find stereo camera')

        self.get_logger().info(f'Using capture device: {working}')

        # Open capture using full path string — never an integer
        self.cap = cv2.VideoCapture(working, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f'VideoCapture failed for {working}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)

        actual_w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(
            f'Opened: {actual_w}x{actual_h} @ {actual_fps:.1f} fps')
        self.get_logger().info(
            f'Split:  left 0->{actual_w//2}  right {actual_w//2}->{actual_w}')

        self.half_width = actual_w // 2

        # Publishers
        self.left_pub  = self.create_publisher(
            Image, '/camera_rear/left/image_raw',  10)
        self.right_pub = self.create_publisher(
            Image, '/camera_rear/right/image_raw', 10)

        self.create_timer(1.0 / publish_rate, self._capture_and_publish)

        self._frame_count = 0
        self.get_logger().info(f'Stereo publisher ready at {publish_rate} Hz')

    def _capture_and_publish(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn(
                'Failed to read frame', throttle_duration_sec=2.0)
            return

        self._frame_count += 1
        left  = frame[:, :self.half_width]
        right = frame[:, self.half_width:]
        stamp = self.get_clock().now().to_msg()

        left_msg = self.bridge.cv2_to_imgmsg(left, encoding='bgr8')
        left_msg.header.stamp    = stamp
        left_msg.header.frame_id = 'camera_rear_link'

        right_msg = self.bridge.cv2_to_imgmsg(right, encoding='bgr8')
        right_msg.header.stamp    = stamp
        right_msg.header.frame_id = 'camera_rear_link'

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        if self._frame_count % 30 == 0:
            self.get_logger().info(
                f'Frames published: {self._frame_count}',
                throttle_duration_sec=10.0)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StereoCameraPublisher()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f'ERROR: {e}')
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()  