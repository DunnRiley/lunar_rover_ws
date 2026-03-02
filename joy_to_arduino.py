#!/usr/bin/env python3
"""
joy_to_arduino.py  —  VERSION 4  (arc only, no pivot)

HOW TO CONFIRM THIS IS RUNNING:
  Terminal will show:  === JOY_TO_ARDUINO VERSION 4 LOADED ===

HOW TO DEPLOY:
  cp joy_to_arduino.py ~/lunar_rover_ws/joy_to_arduino.py
  pkill -f joy_to_arduino
  cd ~/lunar_rover_ws && python3 joy_to_arduino.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  CONTROLS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Right stick Y  ── DRIVE forward / backward
                    (right stick X is completely ignored)

  Left  stick X  ── TURN left / right
                    (left  stick Y is completely ignored)

  Arc ratio slider (GUI, 0-100):
    0   = inner wheel STOPS, outer drives around it
    50  = inner at 50% of outer speed
    100 = inner = outer speed (drives straight)

  Both wheels ALWAYS go the same direction.
  NO counter-rotation. NO pivot mode.

  X  (btn 2)  ── Speed UP   (+5%)
  B  (btn 1)  ── Speed DOWN (-5%)
  LB (btn 4)  ── Actuator EXTEND  (hold)
  RB (btn 5)  ── Actuator RETRACT (hold)
  Start (btn 7) ─ Emergency stop toggle

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  IF AXES ARE WRONG
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ros2 topic echo /joy
  Push Right stick UP    -> note axis index -> set AXIS_DRIVE
  Push Left  stick RIGHT -> note axis index -> set AXIS_TURN
  Standard Xbox USB: axis 0=Left X, 1=Left Y, 3=Right X, 4=Right Y

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  IF ROVER SPINS INSTEAD OF GOING STRAIGHT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Toggle RIGHT_FLIP (True <-> False)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SERIAL PROTOCOL  [0xAA][Device][Speed][Direction][0x55]
    0x05  LEFT  side  (FL + BL)
    0x06  RIGHT side  (FR + BR)
    0x08  Both actuators
    0xFF  STOP ALL
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  PERMANENT SERIAL PERMISSION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  sudo usermod -aG dialout $USER   then log out and back in
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Int8 as RosInt8
import serial
import time
import threading
import glob

VERSION = 4

# ── Serial protocol bytes ─────────────────────────────────────────
START     = 0xAA
END       = 0x55
DEV_LEFT  = 0x05
DEV_RIGHT = 0x06
DEV_ACT   = 0x08
DEV_KILL  = 0xFF

# ═════════════════════════════════════════════════════════════════
#  AXIS / BUTTON MAPPING
# ═════════════════════════════════════════════════════════════════

AXIS_DRIVE = 4    # Right stick Y   — right stick X (axis 3) IGNORED
AXIS_TURN  = 0    # Left  stick X   — left  stick Y (axis 1) IGNORED

BTN_B     = 1     # Speed DOWN
BTN_X     = 2     # Speed UP
BTN_LB    = 4     # Actuator EXTEND  (hold)
BTN_RB    = 5     # Actuator RETRACT (hold)
BTN_START = 7     # Emergency stop

# ═════════════════════════════════════════════════════════════════
#  MOTOR DIRECTION
# ═════════════════════════════════════════════════════════════════

RIGHT_FLIP = True   # Right motors physically mirrored on this rover.
                    # Toggle if rover spins fwd instead of going straight.

# ═════════════════════════════════════════════════════════════════
#  TUNING
# ═════════════════════════════════════════════════════════════════

DEADZONE    = 0.10
MAX_MOTOR   = 200
MAX_HZ      = 20
MIN_GAP     = 1.0 / MAX_HZ
SPEED_START = 0.50
SPEED_STEP  = 0.05
JOY_TIMEOUT = 0.5
ARC_DEFAULT = 50


# ═════════════════════════════════════════════════════════════════
#  ARC DRIVE MATH
#
#  Both wheels ALWAYS go the same direction (both fwd or both back).
#
#  Outer wheel = full drive speed.
#  Inner wheel = (ratio/100) * outer, scaled by turn magnitude.
#
#  ratio=0   inner stops completely at full turn input
#  ratio=50  inner runs at 50% of outer at full turn input
#  ratio=100 inner equals outer (straight line)
#
#  At partial turn inputs the inner reduction blends proportionally,
#  so gentle stick = gentle curve, full stick = maximum arc.
# ═════════════════════════════════════════════════════════════════

def compute_arc(fwd: float, turn: float, ratio: int, speed: float):
    """
    Returns (left_f, right_f) each in [-1.0, 1.0].
    Both values always have the same sign (or one is zero).
    Positive = forward, negative = backward.
    """
    if abs(fwd) < 0.001 and abs(turn) < 0.001:
        return 0.0, 0.0

    inner_fraction = ratio / 100.0
    blend          = abs(turn)   # 0.0 = straight,  1.0 = full turn

    if abs(fwd) < 0.001:
        # No forward input — use turn as the drive signal.
        # Both sides go "forward" relative to the turn direction.
        # Outer side at full turn speed, inner at ratio%.
        outer = abs(turn) * speed
        inner = outer * inner_fraction
        if turn > 0:        # right: left=outer, right=inner
            return outer, inner
        else:               # left:  left=inner, right=outer
            return inner, outer

    # Has forward/backward input.
    # outer stays at fwd*speed regardless of turn.
    # inner scales from fwd*speed (at blend=0) down to fwd*speed*inner_fraction (at blend=1).
    outer        = fwd * speed
    actual_inner = fwd * speed * (1.0 - blend * (1.0 - inner_fraction))

    if turn > 0:            # right: left=outer, right=inner
        return outer, actual_inner
    elif turn < 0:          # left:  left=inner, right=outer
        return actual_inner, outer
    else:
        return outer, outer # straight


# ═════════════════════════════════════════════════════════════════
#  ROS NODE
# ═════════════════════════════════════════════════════════════════

class JoyToArduino(Node):

    def __init__(self, port: str):
        super().__init__('joy_to_arduino')

        self._ser   = None
        self._lock  = threading.Lock()
        self._port  = port
        self._connect(port)

        self._speed     = SPEED_START
        self._ratio     = ARC_DEFAULT
        self._estop     = False
        self._last_joy  = self.get_clock().now()
        self._prev_btns = {}
        self._joy_count = 0

        self._last_left  = (0, 0)
        self._last_right = (0, 0)
        self._last_act   = (0, 0)
        self._last_write = 0.0

        self.create_subscription(Joy,     '/joy',            self._joy_cb,   10)
        self.create_subscription(Bool,    '/emergency_stop', self._estop_cb, 10)
        self.create_subscription(RosInt8, '/arc_ratio',      self._ratio_cb, 10)
        self.create_timer(0.1, self._watchdog)
        self.create_timer(5.0, self._diagnostics)

        print('')
        print('=' * 62)
        print(f'  === JOY_TO_ARDUINO VERSION {VERSION} LOADED ===')
        print(f'  MODE   : ARC ONLY (no pivot)')
        print(f'  DRIVE  : axis {AXIS_DRIVE} (Right stick Y  —  X ignored)')
        print(f'  TURN   : axis {AXIS_TURN} (Left  stick X  —  Y ignored)')
        print(f'  RATIO  : {self._ratio} (0=inner stops, 100=straight)')
        print(f'           Change via GUI /arc_ratio slider')
        print(f'  SPEED  : {self._speed:.2f}  (X btn=up, B btn=down)')
        print(f'  RIGHT_FLIP={RIGHT_FLIP}  MAX_MOTOR={MAX_MOTOR}')
        print(f'  Serial : {port}  open={self._ser is not None}')
        print('  Move sticks to see live output below.')
        print('=' * 62)
        print('')

    # ── Serial ────────────────────────────────────────────────────

    def _connect(self, port: str):
        try:
            self._ser = serial.Serial(port, 115200, timeout=1.0)
            time.sleep(2.0)
            self._ser.reset_input_buffer()
            self.get_logger().info(f'Arduino connected: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial failed: {e}')
            self.get_logger().error('Fix: sudo usermod -aG dialout $USER (log out/in)')
            self._ser = None

    def _send(self, device: int, speed: int, direction: int):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(bytes([START, device,
                                       speed & 0xFF, direction & 0xFF, END]))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write: {e}')
                self._ser = None

    def _stop_all(self):
        with self._lock:
            self._send(DEV_KILL, 0, 0)
        self._last_left = self._last_right = self._last_act = (0, 0)

    # ── Motor output ──────────────────────────────────────────────

    def _to_sd(self, v: float, flip: bool = False):
        spd = min(int(abs(v) * MAX_MOTOR), 255)
        d   = 0 if v >= 0 else 1
        if flip:
            d = 1 - d
        return (spd, d)

    def _set_drive(self, lf: float, rf: float):
        l = self._to_sd(lf, flip=False)
        r = self._to_sd(rf, flip=RIGHT_FLIP)

        now     = time.monotonic()
        changed = (l != self._last_left or r != self._last_right)
        rate_ok = (now - self._last_write) >= MIN_GAP

        if changed and rate_ok:
            with self._lock:
                self._send(DEV_LEFT,  l[0], l[1])
                self._send(DEV_RIGHT, r[0], r[1])
            self._last_left  = l
            self._last_right = r
            self._last_write = now
            print(f'[DRIVE] ratio={self._ratio:3d} speed={self._speed:.2f} | '
                  f'LEFT spd={l[0]:3d} dir={l[1]} | RIGHT spd={r[0]:3d} dir={r[1]}')

    def _set_actuator(self, val: int):
        sd = (MAX_MOTOR, 0) if val > 0 else \
             (MAX_MOTOR, 1) if val < 0 else (0, 0)
        if sd != self._last_act:
            with self._lock:
                self._send(DEV_ACT, sd[0], sd[1])
            self._last_act = sd
            print(f'[ACT]  val={val}  spd={sd[0]} dir={sd[1]}')

    def _stop_drive(self):
        if self._last_left != (0, 0) or self._last_right != (0, 0):
            with self._lock:
                self._send(DEV_LEFT,  0, 0)
                self._send(DEV_RIGHT, 0, 0)
            self._last_left = self._last_right = (0, 0)
            print('[STOP]')

    # ── Helpers ───────────────────────────────────────────────────

    def _rising(self, idx: int, cur: int) -> bool:
        prev = self._prev_btns.get(idx, 0)
        self._prev_btns[idx] = cur
        return cur == 1 and prev == 0

    def _dz(self, v: float) -> float:
        return v if abs(v) >= DEADZONE else 0.0

    # ── /joy callback ─────────────────────────────────────────────

    def _joy_cb(self, msg: Joy):
        self._last_joy   = self.get_clock().now()
        self._joy_count += 1

        ax  = lambda i: msg.axes[i]    if i < len(msg.axes)    else 0.0
        btn = lambda i: msg.buttons[i] if i < len(msg.buttons) else 0

        # Emergency stop
        if self._rising(BTN_START, btn(BTN_START)):
            self._estop = not self._estop
            if self._estop:
                self._stop_all()
                print('[ESTOP] ACTIVATED')
            else:
                print('[ESTOP] cleared — ready')

        if self._estop:
            for b in (BTN_B, BTN_X, BTN_LB, BTN_RB):
                self._prev_btns[b] = btn(b)
            return

        # Speed
        if self._rising(BTN_X, btn(BTN_X)):
            self._speed = round(min(1.0, self._speed + SPEED_STEP), 2)
            print(f'[SPEED] {self._speed:.2f}')

        if self._rising(BTN_B, btn(BTN_B)):
            self._speed = round(max(0.05, self._speed - SPEED_STEP), 2)
            print(f'[SPEED] {self._speed:.2f}')

        # Actuators (hold)
        if btn(BTN_LB):
            self._set_actuator(1)
        elif btn(BTN_RB):
            self._set_actuator(-1)
        else:
            self._set_actuator(0)

        # Drive — ONLY these two axes, all others ignored
        fwd  = self._dz(ax(AXIS_DRIVE))   # Right stick Y only
        turn = self._dz(ax(AXIS_TURN))    # Left  stick X only

        if abs(fwd) < 0.001 and abs(turn) < 0.001:
            self._stop_drive()
            return

        lf, rf = compute_arc(fwd, turn, self._ratio, self._speed)
        self._set_drive(lf, rf)

    # ── /arc_ratio topic (from GUI slider) ────────────────────────

    def _ratio_cb(self, msg: RosInt8):
        old = self._ratio
        self._ratio = max(0, min(100, int(msg.data)))
        if self._ratio != old:
            print(f'[RATIO] {old} -> {self._ratio}')

    # ── Watchdog ──────────────────────────────────────────────────

    def _watchdog(self):
        if self._ser is None or not self._ser.is_open:
            ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            if ports:
                self.get_logger().warn(f'Reconnecting: {ports[0]}')
                self._connect(ports[0])
            return
        if self._estop:
            return
        elapsed = (self.get_clock().now() - self._last_joy).nanoseconds / 1e9
        if elapsed > JOY_TIMEOUT:
            self._stop_drive()

    # ── Diagnostics every 5 s ─────────────────────────────────────

    def _diagnostics(self):
        serial_ok = self._ser is not None and self._ser.is_open
        elapsed   = (self.get_clock().now() - self._last_joy).nanoseconds / 1e9
        print(f'\n[DIAG] serial={serial_ok}  joy={self._joy_count}/5s  '
              f'last={elapsed:.1f}s  ratio={self._ratio}  '
              f'speed={self._speed:.2f}  estop={self._estop}')
        if self._joy_count == 0:
            print('[DIAG] WARNING: no /joy messages — check ROS_DOMAIN_ID=42')
        self._joy_count = 0

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._estop = True
            self._stop_all()
            print('[ESTOP] from topic')
        else:
            self._estop = False
            print('[ESTOP] cleared from topic')

    def destroy_node(self):
        self._stop_all()
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


# ═════════════════════════════════════════════════════════════════
#  MAIN
# ═════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)

    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    if not ports:
        print('ERROR: No Arduino at /dev/ttyACM* or /dev/ttyUSB*')
        print('Fix: sudo usermod -aG dialout $USER  then log out/in')
        rclpy.shutdown()
        return

    print(f'Serial ports found: {ports}')
    node = JoyToArduino(ports[0])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
