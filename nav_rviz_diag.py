#!/usr/bin/env python3
"""
nav_rviz_diag.py  —  runs on MINI PC
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Standalone diagnostic.  Does NOT depend on any other nav node.
Run this ALONE to verify RViz can receive and display markers.

What it does every second:
  1. Broadcasts TF:  odom → base_link  (identity, rover at origin)
  2. Publishes a bright RED cube on /nav/viz_markers  at (1, 0, 0)
  3. Publishes a bright GREEN sphere on /nav/viz_markers at (0, 1, 0)
  4. Publishes a bright BLUE cylinder on /nav/viz_markers at (0, 0, 0)
  5. Publishes a WHITE text marker saying "DIAG OK" at (0, 0, 0.5)
  6. Prints a status line to terminal every second

If you can see these shapes in RViz → the pipeline works,
the problem is in nav_depth_processor.
If you see nothing → the problem is RViz config / TF / topics.

Usage:
  python3 nav_rviz_diag.py

In RViz make sure:
  • Fixed Frame = odom
  • MarkerArray display subscribed to /nav/viz_markers
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import sys
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

try:
    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    HAS_TF = True
    print('[DIAG] tf2_ros available', flush=True)
except ImportError:
    HAS_TF = False
    print('[DIAG] *** tf2_ros NOT available — TF will not be broadcast ***', flush=True)
    print('[DIAG]     Install: sudo apt install ros-humble-tf2-ros', flush=True)


def make_marker(ns, mid, mtype, x, y, z, r, g, b, a,
                sx=0.3, sy=0.3, sz=0.3, text='', frame='odom'):
    m = Marker()
    m.header.frame_id = frame
    m.ns     = ns
    m.id     = mid
    m.type   = mtype
    m.action = Marker.ADD
    m.pose.position.x    = float(x)
    m.pose.position.y    = float(y)
    m.pose.position.z    = float(z)
    m.pose.orientation.w = 1.0
    m.scale.x = float(sx)
    m.scale.y = float(sy)
    m.scale.z = float(sz)
    m.color   = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
    m.lifetime.sec = 3   # stays visible 3s even if we pause
    if text:
        m.text = text
    return m


class DiagNode(Node):

    def __init__(self):
        super().__init__('nav_rviz_diag')

        rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self._viz_pub = self.create_publisher(MarkerArray, '/nav/viz_markers', rel)

        if HAS_TF:
            self._tf_br = TransformBroadcaster(self)
        else:
            self._tf_br = None

        self._count = 0
        self.create_timer(1.0, self._tick)

        print('[DIAG] node started — publishing to /nav/viz_markers', flush=True)
        print('[DIAG] In RViz: Fixed Frame = odom', flush=True)
        print('[DIAG]          Add MarkerArray → /nav/viz_markers', flush=True)
        print('[DIAG] You should see:', flush=True)
        print('[DIAG]   RED cube    at (1, 0, 0)', flush=True)
        print('[DIAG]   GREEN sphere at (0, 1, 0)', flush=True)
        print('[DIAG]   BLUE cylinder at origin', flush=True)
        print('[DIAG]   WHITE text "DIAG OK" above origin', flush=True)
        print(flush=True)

    def _tick(self):
        self._count += 1
        stamp = self.get_clock().now().to_msg()

        # ── TF: odom → base_link (rover at origin, not moving) ────────────
        if self._tf_br is not None:
            tf = TransformStamped()
            tf.header.stamp    = stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id  = 'base_link'
            tf.transform.translation.x = 0.0
            tf.transform.translation.y = 0.0
            tf.transform.translation.z = 0.0
            tf.transform.rotation.w    = 1.0
            self._tf_br.sendTransform(tf)
            tf_status = 'TF broadcast OK'
        else:
            tf_status = 'NO TF (tf2_ros missing)'

        # ── Markers ────────────────────────────────────────────────────────
        ma = MarkerArray()

        # Red cube 1m ahead
        ma.markers.append(make_marker(
            'diag', 0, Marker.CUBE,
            1.0, 0.0, 0.15,
            1.0, 0.0, 0.0, 1.0,
            0.3, 0.3, 0.3))

        # Green sphere 1m left
        ma.markers.append(make_marker(
            'diag', 1, Marker.SPHERE,
            0.0, 1.0, 0.15,
            0.0, 1.0, 0.0, 1.0,
            0.3, 0.3, 0.3))

        # Blue cylinder at origin
        ma.markers.append(make_marker(
            'diag', 2, Marker.CYLINDER,
            0.0, 0.0, 0.05,
            0.0, 0.4, 1.0, 0.8,
            0.4, 0.4, 0.10))

        # White text above origin
        ma.markers.append(make_marker(
            'diag', 3, Marker.TEXT_VIEW_FACING,
            0.0, 0.0, 0.50,
            1.0, 1.0, 1.0, 1.0,
            0.1, 0.1, 0.20,
            text=f'DIAG OK  #{self._count}'))

        # Pulsing yellow arrow pointing right (+y)
        arr = make_marker(
            'diag', 4, Marker.ARROW,
            0.0, 0.0, 0.10,
            1.0, 1.0, 0.0, 1.0,
            0.8, 0.05, 0.05)
        # Arrow direction: along +y axis → rotate 90° around Z
        arr.pose.orientation.z = math.sin(math.pi / 4)
        arr.pose.orientation.w = math.cos(math.pi / 4)
        ma.markers.append(arr)

        # Set all timestamps
        for mk in ma.markers:
            mk.header.stamp = stamp

        self._viz_pub.publish(ma)

        print(f'[DIAG #{self._count:04d}]  {tf_status}  '
              f'published {len(ma.markers)} markers to /nav/viz_markers',
              flush=True)

        # Every 5 ticks also print active ROS2 topics for diagnostics
        if self._count % 5 == 0:
            print('[DIAG] checking for expected topics...', flush=True)
            # We can't call ros2 topic list from inside a node, but we can
            # print a reminder of what to check manually
            print('[DIAG] Run on miniPC: ros2 topic list | grep nav', flush=True)
            print('[DIAG] Run on miniPC: ros2 topic echo /nav/viz_markers --once', flush=True)
            print('[DIAG] Run on laptop: ros2 topic echo /nav/viz_markers --once', flush=True)
            print(flush=True)


def main():
    rclpy.init()
    node = DiagNode()
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