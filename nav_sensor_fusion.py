#!/usr/bin/env python3
"""
nav_sensor_fusion.py  —  MINI PC

Fuses IMU gyro and camera depth into a unified state.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  WHAT EACH SENSOR ACTUALLY DOES IN THIS SYSTEM
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  WHEEL ODOMETRY — handled entirely by the Arduino firmware.
    The firmware uses the Back Left AS5048A SPI magnetic encoder
    to measure distance internally for the 0xDC drive-by-distance
    command. The Pi never receives wheel counts — it only receives
    a "DIST_DONE" or "DIST_TIMEOUT" text line when the move finishes.
    The mission sequencer blocks on /nav/arduino_done, not on any
    distance counter in this node.

  ACTUATOR ENCODER — the telemetry packet contains leftActuatorCount
    (a quadrature encoder for the left actuator, uint16 centred at
    32000). This is published as /nav/encoder_raw and is tracked here
    for diagnostic display and for the actuator_position action to
    verify the actuator has settled. It has NOTHING to do with wheel
    odometry.

  IMU GYRO — gz_scale (deg/s × 1000) is integrated here to give a
    continuous heading estimate. Used by the arc_turn mission action.
    Heading is reset to 0 before each turn via /nav/heading_reset.

  CAMERA DEPTH — published externally on /nav/depth_dist.
    The drive_forward action uses this as the primary stop criterion
    when use_camera=true. This node just passes it through to the
    fused state JSON so the sequencer has one place to read everything.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  PUBLISHED TOPICS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  /nav/fused_state   String (JSON)   full state at FUSION_HZ
  /nav/heading_deg   Float32         IMU-integrated heading (degrees)
  /nav/actuator_enc  Float32         latest actuator encoder count
  /nav/imu_ready     Bool            True once first valid IMU packet

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SUBSCRIBED TOPICS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  /imu/gyro_deg_s    Float32MultiArray   [gx, gy, gz] deg/s
  /imu/accel_ms2     Float32MultiArray   [ax, ay, az] m/s²
  /nav/encoder_raw   Int32               actuator encoder count
  /nav/depth_dist    Float32             camera forward distance (m)
  /nav/heading_reset Bool                zeroes heading accumulator
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32, String
from geometry_msgs.msg import Twist


# ── Tuning ────────────────────────────────────────────────────────────────────

# Which index in [gx, gy, gz] is the yaw (rotation about vertical).
# Default gz = index 2.  Change to 0 or 1 if your IMU is mounted differently.
# Verify: spin rover clockwise → heading_deg should increase.
GYRO_YAW_INDEX = 2

# Sign correction: +1.0 if CW gives positive gz, -1.0 to flip.
GYRO_YAW_SIGN = 1.0

# Zero-rate bias (deg/s). Measure by leaving rover stationary for 10 s
# and averaging the gz output. Set that value here.
GYRO_BIAS_DEG_S = 0.0

# Anything below this is treated as zero during straight driving.
# Increase if heading drifts noticeably while going straight.
GYRO_DEADZONE_DEG_S = 0.8

# Camera distance: readings older than this are marked invalid.
CAMERA_STALE_S = 0.8

FUSION_HZ = 20.0


class NavSensorFusion(Node):

    def __init__(self):
        super().__init__('nav_sensor_fusion')

        self._lock = threading.Lock()

        # ── Heading state (IMU gyro integral) ─────────────────────────────
        self._heading_deg  = 0.0
        self._last_gyro_t  = time.monotonic()
        self._gyro_ready   = False
        self._gyro_z_raw   = 0.0   # latest bias-corrected gz, deg/s

        # ── Actuator encoder (for diagnostics / settle check) ─────────────
        self._actuator_enc = 0     # latest leftActuatorCount

        # ── Camera distance ───────────────────────────────────────────────
        self._camera_dist_m = None
        self._camera_t      = 0.0

        # ── IMU accel ─────────────────────────────────────────────────────
        self._ax = self._ay = self._az = 0.0

        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        be  = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(Float32MultiArray, '/imu/gyro_deg_s',
                                 self._gyro_cb, rel)
        self.create_subscription(Float32MultiArray, '/imu/accel_ms2',
                                 self._accel_cb, rel)
        self.create_subscription(Int32, '/nav/encoder_raw',
                                 self._encoder_cb, rel)
        self.create_subscription(Float32, '/nav/depth_dist',
                                 self._depth_cb, be)
        self.create_subscription(Bool, '/nav/heading_reset',
                                 self._heading_reset_cb, rel)

        # ── Publishers ────────────────────────────────────────────────────
        self._state_pub   = self.create_publisher(String,  '/nav/fused_state',  rel)
        self._heading_pub = self.create_publisher(Float32, '/nav/heading_deg',  rel)
        self._enc_pub     = self.create_publisher(Float32, '/nav/actuator_enc', rel)
        self._imu_rdy_pub = self.create_publisher(Bool,    '/nav/imu_ready',    rel)

        self.create_timer(1.0 / FUSION_HZ, self._publish_state)
        self.create_timer(5.0,             self._diagnostics)

        self.get_logger().info('nav_sensor_fusion ready')
        self.get_logger().info(
            f'  GYRO_YAW_INDEX={GYRO_YAW_INDEX}  '
            f'GYRO_YAW_SIGN={GYRO_YAW_SIGN}  '
            f'DEADZONE={GYRO_DEADZONE_DEG_S}°/s')
        self.get_logger().info(
            '  NOTE: wheel odometry is handled by the Arduino firmware.')
        self.get_logger().info(
            '  This node tracks heading (IMU) and actuator encoder only.')

    # ── Gyro callback ─────────────────────────────────────────────────────

    def _gyro_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return
        raw_gz = float(msg.data[GYRO_YAW_INDEX])
        now    = time.monotonic()

        with self._lock:
            dt = now - self._last_gyro_t
            self._last_gyro_t = now
            was_ready = self._gyro_ready
            self._gyro_ready = True

            gz = (raw_gz - GYRO_BIAS_DEG_S) * GYRO_YAW_SIGN
            if abs(gz) < GYRO_DEADZONE_DEG_S:
                gz = 0.0
            self._gyro_z_raw = gz

            if 0 < dt < 0.5:
                self._heading_deg += gz * dt

        if not was_ready:
            m = Bool(); m.data = True
            self._imu_rdy_pub.publish(m)
            self.get_logger().info('[fusion] IMU ready')

    # ── Accel callback ────────────────────────────────────────────────────

    def _accel_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            with self._lock:
                self._ax = float(msg.data[0])
                self._ay = float(msg.data[1])
                self._az = float(msg.data[2])

    # ── Actuator encoder callback ──────────────────────────────────────────
    # This is leftActuatorCount from the telemetry packet.
    # It is NOT a wheel odometer — it is the actuator quadrature encoder.

    def _encoder_cb(self, msg: Int32):
        with self._lock:
            self._actuator_enc = msg.data

    # ── Camera depth callback ─────────────────────────────────────────────

    def _depth_cb(self, msg: Float32):
        d = float(msg.data)
        if 0.05 < d < 10.0:
            with self._lock:
                self._camera_dist_m = d
                self._camera_t      = time.monotonic()

    # ── Heading reset ─────────────────────────────────────────────────────

    def _heading_reset_cb(self, msg: Bool):
        if msg.data:
            with self._lock:
                self._heading_deg = 0.0
            self.get_logger().info('[fusion] Heading reset to 0°')

    # ── Publish ───────────────────────────────────────────────────────────

    def _publish_state(self):
        now = time.monotonic()
        with self._lock:
            heading    = self._heading_deg
            gz         = self._gyro_z_raw
            act_enc    = self._actuator_enc
            cam_d      = self._camera_dist_m
            cam_age    = now - self._camera_t if self._camera_t > 0 else 999.0
            imu_ok     = self._gyro_ready
            ax, ay, az = self._ax, self._ay, self._az

        camera_valid = (cam_d is not None) and (cam_age < CAMERA_STALE_S)

        state = {
            'heading_deg'  : round(heading, 2),
            'gyro_z_deg_s' : round(gz, 2),
            'actuator_enc' : act_enc,
            'camera_dist_m': round(cam_d, 3) if camera_valid else None,
            'camera_valid' : camera_valid,
            'imu_ready'    : imu_ok,
            'accel'        : [round(ax, 3), round(ay, 3), round(az, 3)],
            'ts'           : round(now, 3),
        }

        m = String(); m.data = json.dumps(state)
        self._state_pub.publish(m)

        h = Float32(); h.data = float(heading)
        self._heading_pub.publish(h)

        e = Float32(); e.data = float(act_enc)
        self._enc_pub.publish(e)

    # ── Diagnostics ───────────────────────────────────────────────────────

    def _diagnostics(self):
        with self._lock:
            h   = self._heading_deg
            gz  = self._gyro_z_raw
            enc = self._actuator_enc
            imu = self._gyro_ready
        self.get_logger().info(
            f'[fusion] heading={h:+.1f}°  gz={gz:+.2f}°/s  '
            f'act_enc={enc}  imu_ok={imu}')
        if not imu:
            self.get_logger().warn(
                '[fusion] No IMU — is nav_arduino_bridge running?\n'
                '  Check: ros2 topic hz /imu/gyro_deg_s')


def main(args=None):
    rclpy.init(args=args)
    node = NavSensorFusion()
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