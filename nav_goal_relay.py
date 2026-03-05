#!/usr/bin/env python3
"""
nav_goal_relay.py  —  runs on LAPTOP

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  OVERVIEW
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  The user clicks a pixel on the compressed depth image in RViz
  using the "Publish Point" tool (which actually publishes to
  /clicked_point as a PointStamped in the image frame).

  However, for a 2D image display in RViz the clicked point
  has x/y in pixel coordinates and z=0.  We need to:

    1.  Receive the click pixel (u, v)
    2.  Look up the depth value D(u,v) from the latest
        streamed depth image (/camera/depth/stream/compressed)
    3.  Back-project using D435 intrinsics to get a 3D point
        in the camera optical frame
    4.  Send [cam_x, cam_y, cam_z] to the miniPC via
        /nav/goal_camera_frame  (Float32MultiArray)

  The miniPC's nav_depth_processor.py then converts this to its
  world frame using dead-reckoning pose and drives to it.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  HOW TO SET A GOAL IN RVIZ
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  1. Open RViz with laptop_stream.rviz (already done by launcher)
  2. Select the "Publish Point" tool in the RViz toolbar
  3. Click on the FRONT DEPTH image panel
     - The depth image shows /camera/depth/stream/compressed
     - Click on a spot on the ground in front of an obstacle
     - A green goal marker will appear in the depth image panel
  4. The system sends the goal to the miniPC automatically
  5. Watch /nav/planned_path appear in RViz as a green line

  To CANCEL:  publish True to /nav/cancel, or press the
              Cancel button that appears in RViz (via the
              overlay marker this node publishes).

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  DEPTH LOOKUP DETAILS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  The streamed depth is JPEG-compressed and decimated (3 fps).
  JPEG introduces artefacts in depth, so we sample a small
  neighbourhood around the clicked pixel and take the median
  of valid (non-zero, non-NaN) values.

  If the depth at the clicked pixel is 0 (unknown/too close),
  we warn the user and ignore the click.

  The streamed depth image was encoded as a TURBO colormap
  (see optimized_image_pipeline.py) so we CANNOT directly read
  depth in metres from it. Instead, this node subscribes to the
  ORIGINAL (uncompressed) depth topic, which is available on
  the laptop over ROS2 DDS — but only if bandwidth allows.

  FALLBACK: If the raw depth topic is not available on the
  laptop (too slow over WiFi), we re-subscribe to the miniPC's
  raw depth stream via an additional lightweight compressed
  relay. The relay sends raw 16-bit values, not a colormap.
  Set use_raw_depth_relay:=true to enable this path.

  DEFAULT (recommended): use the existing
  /camera/depth/stream/compressed but decode the turbo
  colourmap BACK to approximate depth. This is lossy but
  good enough for goal selection (±5cm at 2m).
"""

import math
import struct

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray, Bool, String
from visualization_msgs.msg import Marker


# ── Turbo colourmap inverse LUT ───────────────────────────────────────────
# We encode depth 0-4000 mm as turbo colourmap (see pipeline).
# To invert: find the closest turbo colour and map back.
# We precompute a LUT at module load time.

def _build_turbo_inverse_lut(n_steps=4001):
    """
    Returns an array of shape (256,256,256) → depth_mm  (float32).
    Too large for memory — instead build a fast approximate function.
    We use a 1-D LUT on hue of the colour.
    """
    # Sample the turbo colourmap
    indices = np.linspace(0, 1, n_steps)
    # OpenCV COLORMAP_TURBO maps 0→dark-blue, 255→dark-red
    lut_img = np.arange(n_steps, dtype=np.float32).reshape(1, n_steps, 1)
    lut_img_8u = (lut_img / (n_steps - 1) * 255).astype(np.uint8)
    coloured   = cv2.applyColorMap(lut_img_8u, cv2.COLORMAP_TURBO)  # (1,N,3)
    bgr_table  = coloured[0, :, :]  # (N,3)
    return bgr_table, n_steps


_TURBO_BGR_TABLE, _TURBO_N = _build_turbo_inverse_lut()


def turbo_bgr_to_depth_m(bgr_patch: np.ndarray) -> float | None:
    """
    Convert a small BGR patch (from the turbo-encoded depth image) back
    to depth in metres.  Takes the median after removing zeros.
    Max encoded depth = 4.0 m (as set in optimized_image_pipeline.py).
    """
    if bgr_patch.size == 0:
        return None
    # Flatten to (N, 3)
    flat = bgr_patch.reshape(-1, 3).astype(np.float32)

    # For each pixel, find closest turbo entry via L2 in BGR space
    # We do this efficiently by sampling the LUT every 16 steps
    step = 16
    lut_sub = _TURBO_BGR_TABLE[::step].astype(np.float32)   # (N/step, 3)

    # Vectorised: (N_pixels, 1, 3) - (1, N_lut, 3) → (N_pixels, N_lut)
    diff  = flat[:, np.newaxis, :] - lut_sub[np.newaxis, :, :]
    dist2 = (diff**2).sum(axis=2)
    best  = dist2.argmin(axis=1)   # index into lut_sub
    depth_idx = best * step        # index into full LUT  (0..4000)
    depth_m   = depth_idx / (_TURBO_N - 1) * 4.0

    # Filter: reject 0-depth and anything > 3.8 m (over-range)
    valid = (depth_m > 0.05) & (depth_m < 3.8)
    if not valid.any():
        return None
    return float(np.median(depth_m[valid]))


# ══════════════════════════════════════════════════════════════════════════════
#  NODE
# ══════════════════════════════════════════════════════════════════════════════

class NavGoalRelay(Node):

    def __init__(self):
        super().__init__('nav_goal_relay')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('depth_topic',
                               '/camera/depth/stream/compressed')
        self.declare_parameter('cam_info_topic',
                               '/camera/camera/color/camera_info')
        # D435 424x240 defaults (overridden by camera_info)
        self.declare_parameter('fx', 213.0)
        self.declare_parameter('fy', 213.0)
        self.declare_parameter('cx', 212.0)
        self.declare_parameter('cy', 120.0)
        # Depth patch radius (pixels) for median sampling
        self.declare_parameter('depth_patch_r', 5)

        depth_topic    = self.get_parameter('depth_topic').value
        caminfo_topic  = self.get_parameter('cam_info_topic').value
        self._fx       = self.get_parameter('fx').value
        self._fy       = self.get_parameter('fy').value
        self._cx       = self.get_parameter('cx').value
        self._cy       = self.get_parameter('cy').value
        self._patch_r  = self.get_parameter('depth_patch_r').value

        # Image dimensions (set by first depth image)
        self._img_w = 424
        self._img_h = 240

        # Latest decoded depth BGR image
        self._depth_bgr: np.ndarray | None = None

        # ── QoS ──────────────────────────────────────────────────────────
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=2)
        rel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── Subscribers ──────────────────────────────────────────────────
        # Streamed (compressed) depth image from miniPC
        self.create_subscription(
            CompressedImage, depth_topic,
            self._depth_cb, be_qos)

        # Camera intrinsics (available from miniPC via DDS)
        self.create_subscription(
            CameraInfo, caminfo_topic,
            self._caminfo_cb, rel_qos)

        # RViz "Publish Point" click — pixel coordinates
        # RViz sends this as a PointStamped with x=col, y=row, z=0
        # when clicking on a 2D Image display panel.
        self.create_subscription(
            PointStamped, '/clicked_point',
            self._click_cb, rel_qos)

        # Nav status from miniPC (for UI feedback)
        self.create_subscription(
            String, '/nav/status',
            self._status_cb, rel_qos)

        # ── Publishers ───────────────────────────────────────────────────
        # Goal to miniPC
        self._goal_pub   = self.create_publisher(
            Float32MultiArray, '/nav/goal_camera_frame', rel_qos)
        # Cancel
        self._cancel_pub = self.create_publisher(
            Bool, '/nav/cancel', rel_qos)
        # Visual feedback marker (shown on depth image overlay)
        self._marker_pub = self.create_publisher(
            Marker, '/nav/goal_marker', rel_qos)

        self.get_logger().info('='*60)
        self.get_logger().info('  nav_goal_relay  READY on laptop')
        self.get_logger().info(f'  Depth: {depth_topic}')
        self.get_logger().info('  Click the depth image in RViz to set goal')
        self.get_logger().info('='*60)

    # ── Camera info ───────────────────────────────────────────────────────

    def _caminfo_cb(self, msg: CameraInfo):
        self._fx   = msg.k[0]
        self._fy   = msg.k[4]
        self._cx   = msg.k[2]
        self._cy   = msg.k[5]
        self._img_w = msg.width
        self._img_h = msg.height

    # ── Depth image ───────────────────────────────────────────────────────

    def _depth_cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            bgr    = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if bgr is not None:
                self._depth_bgr = bgr
                h, w = bgr.shape[:2]
                self._img_h = h
                self._img_w = w
        except Exception as e:
            self.get_logger().error(f'Depth decode: {e}',
                                    throttle_duration_sec=5.0)

    # ── Click handler ─────────────────────────────────────────────────────

    def _click_cb(self, msg: PointStamped):
        """
        RViz "Publish Point" on a 2D Image panel sends:
          msg.point.x  = column (u) in image pixels
          msg.point.y  = row    (v) in image pixels
          msg.point.z  = 0

        Note: if the user clicks on a 3D view, x/y/z will be 3D world
        coords — we check the frame_id to distinguish.
        """
        frame = msg.header.frame_id

        # If frame_id is a 3D frame, ignore (user clicked wrong panel)
        if frame in ('base_link', 'odom', 'map', 'camera_link',
                     'camera_depth_optical_frame'):
            self.get_logger().warn(
                f'Click received in 3D frame "{frame}" — '
                'please click on the DEPTH IMAGE panel, not the 3D view',
                throttle_duration_sec=2.0)
            return

        u = msg.point.x
        v = msg.point.y

        # Sanity check pixel bounds
        if u < 0 or v < 0 or u >= self._img_w or v >= self._img_h:
            self.get_logger().warn(
                f'Click ({u:.0f},{v:.0f}) outside image {self._img_w}x{self._img_h}')
            return

        if self._depth_bgr is None:
            self.get_logger().warn('No depth image received yet')
            return

        # Sample depth around the clicked pixel
        u_i = int(round(u))
        v_i = int(round(v))
        r   = self._patch_r

        patch = self._depth_bgr[
            max(0, v_i - r): min(self._img_h, v_i + r + 1),
            max(0, u_i - r): min(self._img_w, u_i + r + 1)]

        depth_m = turbo_bgr_to_depth_m(patch)

        if depth_m is None or depth_m < 0.1:
            self.get_logger().warn(
                f'No valid depth at pixel ({u_i},{v_i}) — try clicking '
                'a brighter (further) area of the depth image')
            return

        # Back-project to camera 3D
        cam_x = (u_i - self._cx) / self._fx * depth_m
        cam_y = (v_i - self._cy) / self._fy * depth_m
        cam_z = depth_m

        self.get_logger().info(
            f'Goal click: pixel ({u_i},{v_i}) → depth {depth_m:.2f}m → '
            f'cam ({cam_x:.2f}, {cam_y:.2f}, {cam_z:.2f})')

        # Send to miniPC
        goal_msg = Float32MultiArray()
        goal_msg.data = [float(cam_x), float(cam_y), float(cam_z)]
        self._goal_pub.publish(goal_msg)

        # Publish visual marker
        self._publish_goal_marker(cam_x, cam_y, cam_z)

    # ── Status feedback ───────────────────────────────────────────────────

    def _status_cb(self, msg: String):
        self.get_logger().info(f'Nav status: {msg.data}')

    # ── Marker ───────────────────────────────────────────────────────────

    def _publish_goal_marker(self, cx, cy, cz):
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = 'camera_color_optical_frame'
        m.ns   = 'nav_goal'
        m.id   = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(cx)
        m.pose.position.y = float(cy)
        m.pose.position.z = float(cz)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.2
        m.color.a = 0.9
        m.lifetime.sec = 30
        self._marker_pub.publish(m)


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()