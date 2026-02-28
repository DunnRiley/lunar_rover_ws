#!/usr/bin/env python3
"""
joy_to_arduino.py  —  runs on the MINI PC

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  AXIS LAYOUT  (only these axes do anything — all others ignored)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Right stick Y  (axis 4)  ──  DRIVE forward / backward
  Left  stick X  (axis 0)  ──  TURN  left / right

  Right stick X  (axis 3)  ──  IGNORED
  Left  stick Y  (axis 1)  ──  IGNORED

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  TURN MODES
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  A (btn 0)  →  PIVOT mode
              Left side drives forward, right side drives BACKWARD.
              (or vice versa depending on direction)
              Classic skid-steer counter-rotation.

  Y (btn 3)  →  ARC mode
              BOTH sides drive in the same direction.
              Outer wheel = full drive speed.
              Inner wheel = (arc_ratio / 100) × outer speed.

              arc_ratio 0:
                Inner wheel STOPS. Outer drives around it.
                Tightest arc. (Stationary-center arc turn)

              arc_ratio 50:
                Inner at 50% speed. Medium curve.

              arc_ratio 100:
                Inner = outer speed. Drives straight.

              Arc ratio is set by the GUI slider (0–100).

  NOTE: When there is NO forward/back input and you push the
  turn stick, BOTH modes behave as pivot (counter-rotation).
  Arc only makes sense when you're actually moving forward/back.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  OTHER BUTTONS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  X  (btn 2)  →  Speed UP   (+5% per press)
  B  (btn 1)  →  Speed DOWN (-5% per press)
  LB (btn 4)  →  Actuator EXTEND  (hold)
  RB (btn 5)  →  Actuator RETRACT (hold)
  Start (btn 7) → Emergency stop toggle

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  VERIFY YOUR AXIS NUMBERS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Run:  ros2 topic echo /joy
  Push Right stick UP   → note which axis changes → set AXIS_DRIVE
  Push Left  stick RIGHT → note which axis changes → set AXIS_TURN

  Standard Xbox USB on Linux:
    Axis 0 = Left  X,  Axis 1 = Left  Y
    Axis 3 = Right X,  Axis 4 = Right Y

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  RIGHT SIDE MOTOR IS MIRRORED?
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  RIGHT_FLIP = True means direction byte is inverted for right side.
  If rover spins instead of going straight when pushing forward: flip it.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  PERMANENT SERIAL PERMISSION FIX
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  sudo usermod -aG dialout $USER   then log out and back in.
  (sudo chmod 666 /dev/ttyACM0 works but resets every reboot)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SERIAL PROTOCOL  [0xAA][Device][Speed][Direction][0x55]
    0x05  LEFT  side (FL + BL)
    0x06  RIGHT side (FR + BR)
    0x08  Both actuators
    0xFF  STOP ALL
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
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
START     = 0xAA
END       = 0x55
DEV_LEFT  = 0x05
DEV_RIGHT = 0x06
DEV_ACT   = 0x08
DEV_KILL  = 0xFF

# ═════════════════════════════════════════════════════════════════════════
#  CONTROLLER MAPPING
#  Update these if ros2 topic echo /joy shows different axis numbers.
# ═════════════════════════════════════════════════════════════════════════

AXIS_DRIVE = 4    # Right stick Y ONLY  — right stick X is ignored
AXIS_TURN  = 0    # Left  stick X ONLY  — left  stick Y is ignored

BTN_A     = 0     # PIVOT mode
BTN_B     = 1     # Speed DOWN
BTN_X     = 2     # Speed UP
BTN_Y     = 3     # ARC mode
BTN_LB    = 4     # Actuator EXTEND  (hold)
BTN_RB    = 5     # Actuator RETRACT (hold)
BTN_START = 7     # Emergency stop toggle

# ═════════════════════════════════════════════════════════════════════════
#  MOTOR DIRECTION — right side is physically mirrored on this rover
# ═════════════════════════════════════════════════════════════════════════

RIGHT_FLIP = True  # Inverts direction byte for right-side packets.
                   # Toggle if rover spins instead of driving straight.

# ═════════════════════════════════════════════════════════════════════════
#  TUNING
# ═════════════════════════════════════════════════════════════════════════

DEADZONE        = 0.10   # Stick input below this is treated as zero
MAX_MOTOR       = 200    # Max PWM value sent to Arduino (0–255)
MAX_SERIAL_HZ   = 20     # Max serial packets per second
MIN_SERIAL_GAP  = 1.0 / MAX_SERIAL_HZ

SPEED_START     = 0.50   # Starting speed multiplier (0.05 – 1.0)
SPEED_STEP      = 0.05
JOY_TIMEOUT     = 0.5    # Seconds without /joy before motors stop

ARC_RATIO_DEFAULT = 50   # 0 = inner stops, 100 = inner = outer (straight)

# ═════════════════════════════════════════════════════════════════════════
#  DRIVE CALCULATION
# ═════════════════════════════════════════════════════════════════════════

PIVOT = 'PIVOT'
ARC   = 'ARC'


def compute_drive(fwd: float, turn: float,
                  mode: str, arc_ratio: int, speed: float):
    """
    Returns (left_pct, right_pct) each in range [-1.0, 1.0].

    fwd        : forward input   [-1.0, 1.0]  positive = forward
    turn       : turn input      [-1.0, 1.0]  positive = right
    mode       : PIVOT or ARC
    arc_ratio  : 0–100
                   0   = inner wheel stops completely
                   100 = inner wheel same speed as outer (straight)
    speed      : overall multiplier [0.05–1.0]
    """

    # ── No movement ───────────────────────────────────────────────────────
    if abs(fwd) < 0.001 and abs(turn) < 0.001:
        return 0.0, 0.0

    # ── Pure turn (no forward input) → always pivot regardless of mode ───
    if abs(fwd) < 0.001:
        # Counter-rotation: one side forward, other backward
        left  =  turn * speed
        right = -turn * speed
        return left, right

    # ── PIVOT mode ────────────────────────────────────────────────────────
    if mode == PIVOT:
        # Classic differential mix: at full turn, inside reverses
        left  = (fwd + turn) * speed
        right = (fwd - turn) * speed
        # Normalise so neither exceeds ±1.0
        mx = max(abs(left), abs(right), 1.0)
        return left / mx, right / mx

    # ── ARC mode ──────────────────────────────────────────────────────────
    # Outer wheel drives at full forward speed.
    # Inner wheel drives at (arc_ratio / 100) * outer speed.
    # Both wheels go in the SAME direction.
    #
    # arc_ratio = 0:   inner = 0.0 * outer  →  inner stops,  outer drives
    # arc_ratio = 50:  inner = 0.5 * outer  →  medium curve
    # arc_ratio = 100: inner = 1.0 * outer  →  straight line
    #
    inner_fraction = arc_ratio / 100.0
    outer = fwd * speed
    inner = fwd * speed * inner_fraction

    if turn > 0:       # turning right → right wheel is inner
        left  = outer
        right = inner
    elif turn < 0:     # turning left  → left  wheel is inner
        left  = inner
        right = outer
    else:              # no turn → straight
        left  = outer
        right = outer

    # Scale by turn magnitude: blend from straight toward arc
    # (at small turn inputs we only partially reduce the inner wheel)
    blend = abs(turn)
    if blend < 1.0:
        straight = fwd * speed
        if turn > 0:
            left  = straight                               # outer unchanged
            right = straight * (1.0 - blend * (1.0 - inner_fraction))
        elif turn < 0:
            left  = straight * (1.0 - blend * (1.0 - inner_fraction))
            right = straight

    return left, right


# ═════════════════════════════════════════════════════════════════════════
#  ROS NODE
# ═════════════════════════════════════════════════════════════════════════

class JoyToArduino(Node):

    def __init__(self, port: str):
        super().__init__('joy_to_arduino')

        self._ser   = None
        self._lock  = threading.Lock()
        self._port  = port
        self._connect(port)

        self._speed      = SPEED_START
        self._mode       = ARC
        self._arc_ratio  = ARC_RATIO_DEFAULT
        self._emergency  = False
        self._last_joy   = self.get_clock().now()
        self._prev_btns  = {}
        self._joy_count  = 0

        # Change detection — skip redundant serial writes
        self._last_left  = (0, 0)
        self._last_right = (0, 0)
        self._last_act   = (0, 0)
        self._last_write = 0.0

        self.create_subscription(Joy,  '/joy',            self._joy_cb,   10)
        self.create_subscription(Bool, '/emergency_stop', self._estop_cb, 10)
        self.create_timer(0.1, self._watchdog)
        self.create_timer(5.0, self._diagnostics)

        self._banner()

    # ── Banner ────────────────────────────────────────────────────────────

    def _banner(self):
        L = self.get_logger().info
        L('=' * 62)
        L('  joy_to_arduino  |  miniPC serial bridge')
        L(f'  Port    : {self._port}  connected={self._ser is not None}')
        L(f'  DRIVE   : axis {AXIS_DRIVE} (Right stick Y only — X ignored)')
        L(f'  TURN    : axis {AXIS_TURN} (Left  stick X only — Y ignored)')
        L(f'  A={BTN_A}→PIVOT  Y={BTN_Y}→ARC  '
          f'X={BTN_X}→spd+  B={BTN_B}→spd-  '
          f'LB={BTN_LB}→extend  RB={BTN_RB}→retract')
        L(f'  Start mode=ARC  arc_ratio={self._arc_ratio}  speed={self._speed:.2f}')
        L(f'  RIGHT_FLIP={RIGHT_FLIP}')
        L('=' * 62)

    # ── Serial ────────────────────────────────────────────────────────────

    def _connect(self, port: str):
        try:
            self._ser = serial.Serial(port, 115200, timeout=1.0)
            time.sleep(2.0)
            self._ser.reset_input_buffer()
            self.get_logger().info(f'✓ Arduino connected: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'✗ Serial open failed: {e}')
            self.get_logger().error(
                '  Permanent fix: sudo usermod -aG dialout $USER  (log out/in)')
            self._ser = None

    def _send(self, device: int, speed: int, direction: int):
        """Send one 5-byte packet. Must be called with self._lock held."""
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(bytes([START, device,
                                       speed & 0xFF, direction & 0xFF, END]))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')
                self._ser = None

    def _stop_all(self):
        with self._lock:
            self._send(DEV_KILL, 0, 0)
        self._last_left  = (0, 0)
        self._last_right = (0, 0)
        self._last_act   = (0, 0)

    # ── Motor helpers ─────────────────────────────────────────────────────

    def _to_sd(self, v: float, flip: bool = False):
        """Convert float [-1, 1] → (speed_byte, direction_byte)."""
        spd = min(int(abs(v) * MAX_MOTOR), 255)
        d   = 0 if v >= 0 else 1
        if flip:
            d = 1 - d
        return (spd, d)

    def _set_drive(self, left_f: float, right_f: float):
        l = self._to_sd(left_f,  flip=False)
        r = self._to_sd(right_f, flip=RIGHT_FLIP)

        now     = time.monotonic()
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
        sd = (MAX_MOTOR, 0) if val > 0 else \
             (MAX_MOTOR, 1) if val < 0 else (0, 0)
        if sd != self._last_act:
            with self._lock:
                self._send(DEV_ACT, sd[0], sd[1])
            self._last_act = sd

    def _stop_drive(self):
        if self._last_left != (0, 0) or self._last_right != (0, 0):
            with self._lock:
                self._send(DEV_LEFT,  0, 0)
                self._send(DEV_RIGHT, 0, 0)
            self._last_left  = (0, 0)
            self._last_right = (0, 0)

    # ── Helpers ───────────────────────────────────────────────────────────

    def _rising(self, idx: int, cur: int) -> bool:
        prev = self._prev_btns.get(idx, 0)
        self._prev_btns[idx] = cur
        return cur == 1 and prev == 0

    def _dz(self, v: float) -> float:
        return v if abs(v) >= DEADZONE else 0.0

    # ── /joy callback ─────────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy):
        self._last_joy   = self.get_clock().now()
        self._joy_count += 1

        def ax(i):  return msg.axes[i]    if i < len(msg.axes)    else 0.0
        def btn(i): return msg.buttons[i] if i < len(msg.buttons) else 0

        # ── Emergency stop ────────────────────────────────────────────────
        if self._rising(BTN_START, btn(BTN_START)):
            self._emergency = not self._emergency
            if self._emergency:
                self._stop_all()
                self.get_logger().warn('⬛ EMERGENCY STOP')
            else:
                self.get_logger().info('✓ E-stop cleared — ready')

        if self._emergency:
            for b in (BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB):
                self._prev_btns[b] = btn(b)
            return

        # ── Turn mode ─────────────────────────────────────────────────────
        if self._rising(BTN_A, btn(BTN_A)):
            self._mode = PIVOT
            self.get_logger().info(
                'Mode → PIVOT  (counter-rotation, both directions)')

        if self._rising(BTN_Y, btn(BTN_Y)):
            self._mode = ARC
            self.get_logger().info(
                f'Mode → ARC  '
                f'ratio={self._arc_ratio}  '
                f'(0=inner stops, 100=straight)')

        # ── Speed ─────────────────────────────────────────────────────────
        if self._rising(BTN_X, btn(BTN_X)):
            self._speed = round(min(1.0, self._speed + SPEED_STEP), 2)
            self.get_logger().info(f'Speed: {self._speed:.2f}')

        if self._rising(BTN_B, btn(BTN_B)):
            self._speed = round(max(0.05, self._speed - SPEED_STEP), 2)
            self.get_logger().info(f'Speed: {self._speed:.2f}')

        # ── Actuators (hold) ──────────────────────────────────────────────
        if btn(BTN_LB):
            self._set_actuator(1)
        elif btn(BTN_RB):
            self._set_actuator(-1)
        else:
            self._set_actuator(0)

        # ── Drive ─────────────────────────────────────────────────────────
        # ONLY read the specific axis for each — ignore all other axes.
        fwd  = self._dz(ax(AXIS_DRIVE))   # Right stick Y only
        turn = self._dz(ax(AXIS_TURN))    # Left  stick X only

        if abs(fwd) < 0.001 and abs(turn) < 0.001:
            self._stop_drive()
            return

        left_f, right_f = compute_drive(
            fwd, turn, self._mode, self._arc_ratio, self._speed)

        self._set_drive(left_f, right_f)

    # ── Watchdog ──────────────────────────────────────────────────────────

    def _watchdog(self):
        # Attempt serial reconnect if dropped
        if self._ser is None or not self._ser.is_open:
            ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            if ports:
                self.get_logger().warn(f'Reconnecting serial: {ports[0]}')
                self._connect(ports[0])
            return

        if self._emergency:
            return

        elapsed = (self.get_clock().now() - self._last_joy).nanoseconds / 1e9
        if elapsed > JOY_TIMEOUT:
            self._stop_drive()
            self.get_logger().warn('Controller lost — motors stopped',
                                   throttle_duration_sec=2.0)

    # ── Diagnostics every 5 s ─────────────────────────────────────────────

    def _diagnostics(self):
        serial_ok = self._ser is not None and self._ser.is_open
        elapsed   = (self.get_clock().now() - self._last_joy).nanoseconds / 1e9
        self.get_logger().info(
            f'[diag] serial={serial_ok}  '
            f'joy={self._joy_count}/5s  '
            f'last_joy={elapsed:.1f}s ago  '
            f'mode={self._mode}  '
            f'arc_ratio={self._arc_ratio}  '
            f'speed={self._speed:.2f}  '
            f'estop={self._emergency}'
        )
        if self._joy_count == 0:
            self.get_logger().warn(
                '⚠ No /joy messages received — '
                'is joy_node running on laptop? ROS_DOMAIN_ID=42?')
        self._joy_count = 0

    # ── Emergency stop topic ──────────────────────────────────────────────

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._emergency = True
            self._stop_all()
            self.get_logger().error('⬛ EMERGENCY STOP (topic)')
        else:
            self._emergency = False
            self.get_logger().info('✓ E-stop cleared (topic)')

    # ── Cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop_all()
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


# ═════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)

    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    if not ports:
        print('ERROR: No Arduino found at /dev/ttyACM* or /dev/ttyUSB*')
        print('  Permanent fix: sudo usermod -aG dialout $USER  (log out/in)')
        print('  Temporary fix: sudo chmod 666 /dev/ttyACM0')
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