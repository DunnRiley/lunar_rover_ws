#!/usr/bin/env python3
"""
joy_to_arduino.py  —  runs on the MINI PC
Subscribes to /joy (published by laptop's joy_node over DDS),
writes directly to Arduino serial. No cmd_vel middleman.

Architecture:
  Laptop:  joy_node → /joy  (raw USB controller, sent over DDS)
  MiniPC:  joy_to_arduino.py → serial → Arduino

Why this is better than laptop→cmd_vel→miniPC→serial:
  - One fewer network hop for motor commands
  - Serial writes happen locally, no laptop queue buildup
  - Joy arrives at ~50Hz but we only write serial when values CHANGE
  - Rate-limited to MAX_SERIAL_HZ to never overflow Arduino buffer

Protocol: [0xAA][Device][Speed][Direction][0x55]
  0x05  LEFT  side (FL+BL)
  0x06  RIGHT side (FR+BR)
  0x08  Both actuators
  0xFF  STOP ALL
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import serial
import time
import threading
import glob

# ── Serial protocol ───────────────────────────────────────────────────────
START      = 0xAA
END        = 0x55
DEV_LEFT   = 0x05   # FL + BL together
DEV_RIGHT  = 0x06   # FR + BR together
DEV_ACT    = 0x08   # Both actuators
DEV_KILL   = 0xFF

# ── Controller mapping ────────────────────────────────────────────────────
AXIS_FWD   = 1    # Left  stick Y  (+1 = forward)
AXIS_TURN  = 3    # Right stick X  (+1 = left)
BTN_LB     = 4    # Left  bumper → actuator extend
BTN_RB     = 5    # Right bumper → actuator retract
BTN_X      = 2    # X → speed up
BTN_B      = 1    # B → speed down
BTN_START  = 7    # Emergency stop toggle

# ── Tuning ────────────────────────────────────────────────────────────────
DEADZONE       = 0.10
ANGULAR_SCALE  = 1.2
MAX_MOTOR      = 200    # 0-255, cap to leave headroom
MAX_SERIAL_HZ  = 20     # max serial writes per second — Arduino can handle ~200/s
                        # but 20Hz gives plenty of margin and feels responsive
MIN_SERIAL_GAP = 1.0 / MAX_SERIAL_HZ

SPEED_START    = 0.50
SPEED_STEP     = 0.05
JOY_TIMEOUT    = 0.5    # stop motors if no /joy message for this long


class JoyToArduino(Node):

    def __init__(self, port: str):
        super().__init__('joy_to_arduino')

        # ── Serial ───────────────────────────────────────────────────────
        self._ser  = None
        self._lock = threading.Lock()
        self._connect(port)

        # ── State ────────────────────────────────────────────────────────
        self._speed      = SPEED_START
        self._emergency  = False
        self._last_joy   = self.get_clock().now()
        self._prev_btns  = {}

        # Change-detection: only write serial when output actually changes
        self._last_left  = (0, 0)   # (speed_byte, direction_byte)
        self._last_right = (0, 0)
        self._last_act   = (0, 0)

        # Rate limiter: don't write serial faster than MAX_SERIAL_HZ
        self._last_write = 0.0

        # ── Subscriptions ────────────────────────────────────────────────
        self.create_subscription(Joy,  '/joy',             self._joy_cb,   10)
        self.create_subscription(Bool, '/emergency_stop',  self._estop_cb, 10)

        # ── Watchdog ─────────────────────────────────────────────────────
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Joy → Arduino  (direct serial, miniPC)')
        self.get_logger().info(f'Serial: {port}  |  Max rate: {MAX_SERIAL_HZ}Hz')
        self.get_logger().info('Left stick=drive  RStick=turn')
        self.get_logger().info('LB=extend  RB=retract  X=spd+  B=spd-  Start=estop')
        self.get_logger().info('=' * 50)

    # ── Serial helpers ────────────────────────────────────────────────────

    def _connect(self, port: str):
        try:
            self._ser = serial.Serial(port, 115200, timeout=1.0)
            time.sleep(2.0)
            self._ser.reset_input_buffer()
            self.get_logger().info(f'Connected to Arduino on {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial connect failed: {e}')
            self._ser = None

    def _send(self, device: int, speed: int, direction: int):
        """Write one 5-byte packet. Called with _lock held."""
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(bytes([START, device, speed & 0xFF, direction & 0xFF, END]))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')

    def _stop_all(self):
        with self._lock:
            self._send(DEV_KILL, 0, 0)
        self._last_left  = (0, 0)
        self._last_right = (0, 0)
        self._last_act   = (0, 0)

    # ── Drive output — only writes serial when value changes ─────────────

    def _set_drive(self, left_f: float, right_f: float):
        """
        left_f / right_f in [-1.0, 1.0].
        Converts to (speed_byte, dir_byte), skips write if unchanged.
        Rate-limited to MAX_SERIAL_HZ.
        """
        def to_sd(v):
            spd = min(int(abs(v) * MAX_MOTOR), 255)
            d   = 0 if v >= 0 else 1
            return (spd, d)

        l = to_sd(left_f)
        r = to_sd(right_f)

        now = time.monotonic()
        changed = (l != self._last_left or r != self._last_right)
        rate_ok = (now - self._last_write) >= MIN_SERIAL_GAP

        if changed and rate_ok:
            with self._lock:
                self._send(DEV_LEFT,  l[0], l[1])
                self._send(DEV_RIGHT, r[0], r[1])
            self._last_left  = l
            self._last_right = r
            self._last_write = now

    def _set_actuator(self, val: int):
        """val: +1=extend, -1=retract, 0=stop. Only writes on change."""
        if val > 0:
            sd = (MAX_MOTOR, 0)
        elif val < 0:
            sd = (MAX_MOTOR, 1)
        else:
            sd = (0, 0)

        if sd != self._last_act:
            with self._lock:
                self._send(DEV_ACT, sd[0], sd[1])
            self._last_act = sd

    # ── Button edge detection ─────────────────────────────────────────────

    def _rising(self, idx: int, current: int) -> bool:
        prev = self._prev_btns.get(idx, 0)
        self._prev_btns[idx] = current
        return current == 1 and prev == 0

    def _dz(self, v: float) -> float:
        return v if abs(v) >= DEADZONE else 0.0

    # ── /joy callback ─────────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy):
        self._last_joy = self.get_clock().now()

        def ax(i):  return msg.axes[i]    if i < len(msg.axes)    else 0.0
        def btn(i): return msg.buttons[i] if i < len(msg.buttons) else 0

        # Emergency stop toggle
        if self._rising(BTN_START, btn(BTN_START)):
            self._emergency = not self._emergency
            if self._emergency:
                self._stop_all()
                self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            else:
                self.get_logger().info('Emergency stop cleared')

        if self._emergency:
            # Consume edge detection so buttons don't fire on resume
            for b in (BTN_LB, BTN_RB, BTN_X, BTN_B):
                self._prev_btns[b] = btn(b)
            return

        # Speed adjust — rising edge only, one step per click
        if self._rising(BTN_X, btn(BTN_X)):
            self._speed = round(min(1.0, self._speed + SPEED_STEP), 2)
            self.get_logger().info(f'Speed: {self._speed:.2f}')

        if self._rising(BTN_B, btn(BTN_B)):
            self._speed = round(max(0.05, self._speed - SPEED_STEP), 2)
            self.get_logger().info(f'Speed: {self._speed:.2f}')

        # Actuators — hold LB or RB, stop when neither held
        lb = btn(BTN_LB)
        rb = btn(BTN_RB)
        if lb:
            self._set_actuator(1)
        elif rb:
            self._set_actuator(-1)
        else:
            self._set_actuator(0)

        # Drive — differential mix, change-detected, rate-limited
        fwd  = self._dz(ax(AXIS_FWD))
        turn = self._dz(ax(AXIS_TURN))

        left  = (fwd - turn * ANGULAR_SCALE) * self._speed
        right = (fwd + turn * ANGULAR_SCALE) * self._speed

        # Normalise if either side exceeds 1.0
        mx = max(abs(left), abs(right), 1.0)
        self._set_drive(left / mx, right / mx)

    # ── Watchdog ──────────────────────────────────────────────────────────

    def _watchdog(self):
        if self._emergency:
            return
        elapsed = (self.get_clock().now() - self._last_joy).nanoseconds / 1e9
        if elapsed > JOY_TIMEOUT:
            if self._last_left != (0, 0) or self._last_right != (0, 0):
                self._stop_all()
                self.get_logger().warn('Controller timeout — motors stopped',
                                       throttle_duration_sec=2.0)

    # ── Emergency stop subscriber ─────────────────────────────────────────

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._emergency = True
            self._stop_all()
            self.get_logger().error('EMERGENCY STOP (from /emergency_stop topic)')
        else:
            self._emergency = False
            self.get_logger().info('Emergency stop cleared (from topic)')

    # ── Cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop_all()
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Auto-detect Arduino port
    ports = []
    for pattern in ['/dev/ttyACM*', '/dev/ttyUSB*']:
        ports.extend(glob.glob(pattern))

    if not ports:
        print('ERROR: No Arduino found at /dev/ttyACM* or /dev/ttyUSB*')
        rclpy.shutdown()
        return

    port = ports[0]
    print(f'Using Arduino port: {port}')

    node = JoyToArduino(port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()