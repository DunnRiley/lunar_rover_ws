#!/usr/bin/env python3
"""
nav_cmd_mux.py  —  runs on MINI PC

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  PRIORITY MUX  —  Joystick vs Autonomous Navigation
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  INPUTS:
    /joy                sensor_msgs/Joy       Controller state
    /nav/cmd_vel        geometry_msgs/Twist   Nav planner output

  OUTPUT:
    /cmd_vel            geometry_msgs/Twist   → Arduino motor controller

  LOGIC:
    • If any joystick axis or button has been non-zero in the last
      JOY_TIMEOUT seconds → MANUAL mode, pass /joy through
      (joy_to_arduino.py already handles the serial protocol,
       so in MANUAL mode we just suppress /nav/cmd_vel)

    • If no joystick input for JOY_TIMEOUT seconds → AUTO mode,
      pass /nav/cmd_vel through to /cmd_vel

    • Transition MANUAL → AUTO: zero /cmd_vel first, then resume nav
    • Transition AUTO → MANUAL: immediately switch (joystick takes over)

  Additionally:
    • Publishes /nav/joystick_active (Bool) so nav_depth_processor
      can pause dead reckoning while joystick is active
    • Publishes /nav/mux_status (String) for logging

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  NOTE ON joy_to_arduino.py COEXISTENCE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  joy_to_arduino.py reads /joy and writes DIRECTLY to the
  Arduino serial port — it does NOT go through /cmd_vel.

  The Arduino motor controller (arduino_motor_controller.py)
  listens on /cmd_vel.

  So the two paths are:
    joy_to_arduino.py  →  serial (direct)
    nav_depth_processor.py → /nav/cmd_vel → mux → /cmd_vel
                             → arduino_motor_controller.py → serial

  This means BOTH could send to the Arduino simultaneously.

  SOLUTION implemented here:
    When joystick is active, this mux sends a ZERO /cmd_vel
    to suppress the motor controller. The joystick takes over
    via joy_to_arduino.py directly.

    When joystick is idle, the mux passes nav cmd_vel through
    and joy_to_arduino.py watchdog will have stopped sending
    (JOY_TIMEOUT in joy_to_arduino.py = 0.5s).

  This works because joy_to_arduino.py already has a watchdog
  that stops the motors after 0.5s of no /joy messages.
  Set JOY_TIMEOUT here to be slightly longer than that.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


# ── Tuning ────────────────────────────────────────────────────────────────

JOY_TIMEOUT    = 0.7    # seconds of joystick silence before AUTO resumes
JOY_DEADZONE   = 0.08   # axis value below this is treated as zero
RESUME_DELAY   = 0.5    # extra delay after joystick goes idle before AUTO resumes
STATUS_HZ      = 2.0    # how often to log mux status


class NavCmdMux(Node):

    def __init__(self):
        super().__init__('nav_cmd_mux')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('joy_timeout',   JOY_TIMEOUT)
        self.declare_parameter('joy_deadzone',  JOY_DEADZONE)
        self.declare_parameter('resume_delay',  RESUME_DELAY)

        self._joy_timeout  = self.get_parameter('joy_timeout').value
        self._joy_deadzone = self.get_parameter('joy_deadzone').value
        self._resume_delay = self.get_parameter('resume_delay').value

        # ── State ────────────────────────────────────────────────────────
        self._mode          = 'MANUAL'   # 'MANUAL' or 'AUTO'
        self._last_joy_time = 0.0        # monotonic time of last joystick input
        self._last_nav_vel  = Twist()    # most recent nav cmd_vel

        # ── QoS ──────────────────────────────────────────────────────────
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        rel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            Joy, '/joy',
            self._joy_cb, be_qos)

        self.create_subscription(
            Twist, '/nav/cmd_vel',
            self._nav_vel_cb, rel_qos)

        # ── Publishers ───────────────────────────────────────────────────
        # This is what arduino_motor_controller.py listens to
        self._cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',            rel_qos)
        self._jactive_pub = self.create_publisher(Bool,  '/nav/joystick_active', rel_qos)
        self._status_pub  = self.create_publisher(String, '/nav/mux_status',     rel_qos)

        # ── Timers ───────────────────────────────────────────────────────
        self.create_timer(1.0 / STATUS_HZ, self._status_cb)
        # Watchdog: check if joystick has gone idle → switch to AUTO
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info('='*60)
        self.get_logger().info('  nav_cmd_mux  READY')
        self.get_logger().info('  Starting in MANUAL mode')
        self.get_logger().info(f'  Joy timeout: {self._joy_timeout}s + {self._resume_delay}s resume delay')
        self.get_logger().info('  Touch any stick to override nav; release to resume')
        self.get_logger().info('='*60)

    # ── Joystick activity detector ────────────────────────────────────────

    def _joy_cb(self, msg: Joy):
        """Detect any meaningful joystick input."""
        active = False

        for ax in msg.axes:
            if abs(ax) > self._joy_deadzone:
                active = True
                break

        if not active:
            for btn in msg.buttons:
                if btn != 0:
                    active = True
                    break

        if active:
            now = time.monotonic()
            was_manual = (self._mode == 'MANUAL')
            self._last_joy_time = now

            if self._mode == 'AUTO':
                # Joystick just took over — immediately stop nav cmd_vel
                self._mode = 'MANUAL'
                zero = Twist()
                self._cmd_pub.publish(zero)
                self.get_logger().info('⬛ MANUAL override — joystick active')
                self._publish_joy_active(True)

        # Always tell the nav node if joystick is active
        # (it pauses dead reckoning)
        self._publish_joy_active(active)

    # ── Nav cmd_vel pass-through ──────────────────────────────────────────

    def _nav_vel_cb(self, msg: Twist):
        """Store latest nav command and forward it if in AUTO mode."""
        self._last_nav_vel = msg

        if self._mode == 'AUTO':
            self._cmd_pub.publish(msg)

    # ── Watchdog: joystick idle → resume AUTO ─────────────────────────────

    def _watchdog(self):
        if self._mode == 'MANUAL':
            now = time.monotonic()
            idle_time = now - self._last_joy_time

            if idle_time > (self._joy_timeout + self._resume_delay):
                self._mode = 'AUTO'
                self.get_logger().info('✓ AUTO mode — joystick idle, nav resumed')
                self._publish_joy_active(False)

    # ── Status publisher ──────────────────────────────────────────────────

    def _status_cb(self):
        now = time.monotonic()
        idle = now - self._last_joy_time

        m = String()
        m.data = f'{self._mode}  joy_idle={idle:.1f}s'
        self._status_pub.publish(m)

        self.get_logger().info(
            f'[mux] mode={self._mode}  joy_idle={idle:.1f}s',
            throttle_duration_sec=5.0)

    # ── Helper ───────────────────────────────────────────────────────────

    def _publish_joy_active(self, active: bool):
        m = Bool()
        m.data = active
        self._jactive_pub.publish(m)


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = NavCmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero cmd_vel on shutdown
        zero = Twist()
        node._cmd_pub.publish(zero)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()