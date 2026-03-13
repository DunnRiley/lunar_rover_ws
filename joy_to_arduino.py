#!/usr/bin/env python3
"""
joy_to_arduino.py  —  runs on the MINI PC

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  TANK DRIVE LAYOUT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Left  stick Y  (axis 1)  ──  LEFT  wheels forward / backward
  Right stick Y  (axis 4)  ──  RIGHT wheels forward / backward

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SPEED CONTROL  (independent per side)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  LB (btn 4)  →  LEFT  speed UP   (+5% per press)
  LT (axis 2) →  LEFT  speed DOWN (-5% per press, trigger threshold)
  RB (btn 5)  →  RIGHT speed UP   (+5% per press)
  RT (axis 5) →  RIGHT speed DOWN (-5% per press, trigger threshold)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ACTUATOR POSITION CONTROL  (encoder-based)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  A (btn 0)  →  DUMP position    (fully retracted  — 0xB3)
  Y (btn 3)  →  DRIVE position   (level for driving — 0xA9)
  B (btn 1)  →  DIG  position    (furthest extend  — 0xA7)

  The Arduino state machine drives actuators to encoder target and
  stops automatically.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SERVO CONTROL  (continuous move / stop)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  D-pad RIGHT  →  CW  movement  (angle 135) while held; STOP on release
  D-pad LEFT   →  CCW movement  (angle  45) while held; STOP on release

  Protocol: 0x11, angle, 0x01
    90  = stop
    45  = counter-clockwise
    135 = clockwise

  D-pad UP/DOWN are unused for servo (reserved).

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  EMERGENCY STOP
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Start (btn 7) →  toggle emergency stop

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ENCODER PERSISTENCE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  On startup, the node checks for a saved encoder count file
  (~/.lunar_encoder_state.json).  If found it sends command 0xCB
  to restore the last known actuator encoder value to the Arduino
  so position moves remain accurate across power cycles.

  The file is updated every time a new encoder count arrives via
  telemetry (Serial2 on the Arduino).

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SERIAL PROTOCOL  [0xAA][Device][Speed][Direction][0x55]
    0x05  LEFT  side (FL + BL)
    0x06  RIGHT side (FR + BR)
    0x11  Servo  (Speed = angle 45/90/135, Direction = 0x01)
    0xA7  Actuator → DIG   position
    0xA9  Actuator → DRIVE position
    0xB3  Actuator → DUMP  position
    0xCA  Calibrate actuator (retract to hard stop, zero encoder)
    0xCB  Set encoder count  (Speed = high byte, Direction = low byte)
    0xFF  STOP ALL

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  TELEMETRY FORMAT (from Arduino on Serial2 → USB Serial2 reader)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  0xAA  START
  ax    4-byte int32 LE  (mm/s²)
  ay    4-byte int32 LE
  az    4-byte int32 LE
  gx    4-byte int32 LE  (mdps)
  gy    4-byte int32 LE
  gz    4-byte int32 LE
  0xA5  ENC marker
  enc   2-byte uint16 LE
  chk   1-byte XOR checksum
  0x55  END

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import glob
import json
import math
import os
import struct
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import serial

# ── Serial protocol ───────────────────────────────────────────────────────
START      = 0xAA
END        = 0x55
ENC_MARKER = 0xA5

DEV_LEFT   = 0x05
DEV_RIGHT  = 0x06
DEV_SERVO  = 0x11
DEV_KILL   = 0xFF

# Actuator position commands
CMD_DIG    = 0xA7
CMD_DRIVE  = 0xA9
CMD_DUMP   = 0xB3
CMD_CAL    = 0xCA
CMD_SET_ENC = 0xCB   # high byte in Speed field, low byte in Direction field

# Servo angles
SERVO_STOP = 90
SERVO_CCW  = 45
SERVO_CW   = 135

# ── Persistence file ──────────────────────────────────────────────────────
ENCODER_STATE_FILE = Path.home() / '.lunar_encoder_state.json'
SERVO_LOG_FILE     = Path.home() / 'lunar_servo_log.csv'

# ═════════════════════════════════════════════════════════════════════════
#  CONTROLLER MAPPIN
# ═════════════════════════════════════════════════════════════════════════

AXIS_LEFT   = 1   # Left  stick Y  → LEFT  wheels
AXIS_RIGHT  = 4   # Right stick Y  → RIGHT wheels

# Triggers (axis value: rest = +1.0, fully pressed = -1.0 on most Xbox pads)
AXIS_LT     = 2   # Left  trigger  → LEFT  speed DOWN
AXIS_RT     = 5   # Right trigger  → RIGHT speed DOWN
TRIGGER_THRESHOLD = 0.5   # trigger pressed threshold (raw axis < 1.0 - threshold)

# Bumpers
BTN_LB      = 4   # LEFT  speed UP
BTN_RB      = 5   # RIGHT speed UP

# Actuator position buttons
BTN_A       = 0   # → DUMP  position
BTN_Y       = 3   # → DRIVE position
BTN_B       = 1   # → DIG   position
BTN_X       = 2   # → CALIBRATE (retract to hard stop, zero encoder)

# Emergency stop
BTN_START   = 7

# D-pad axes (standard Xbox USB on Linux)
DPAD_AXIS_LR = 6   # -1 = left, +1 = right
DPAD_AXIS_UD = 7   # +1 = up,   -1 = down

# ═════════════════════════════════════════════════════════════════════════
#  TUNING
# ═════════════════════════════════════════════════════════════════════════

RIGHT_FLIP      = True
DEADZONE        = 0.10
MAX_MOTOR       = 200
MAX_SERIAL_HZ   = 20
MIN_SERIAL_GAP  = 1.0 / MAX_SERIAL_HZ
SPEED_START     = 1.0
SPEED_STEP      = 0.05
JOY_TIMEOUT     = 0.5   # seconds without /joy → stop motors

# ── Telemetry parse state ─────────────────────────────────────────────────
_TELEM_START  = 0
_TELEM_IMU    = 1
_TELEM_ENC_M  = 2
_TELEM_ENC    = 3
_TELEM_CHK    = 4


# ═════════════════════════════════════════════════════════════════════════
#  ENCODER STATE PERSISTENCE
# ═════════════════════════════════════════════════════════════════════════

def load_encoder_state() -> dict:
    """Load last known encoder state from disk.  Returns {} if not found."""
    try:
        with open(ENCODER_STATE_FILE, 'r') as f:
            data = json.load(f)
        return data
    except Exception:
        return {}


def save_encoder_state(enc_count: int) -> None:
    """Persist encoder count to disk."""
    try:
        data = {
            'encoder_count': enc_count,
            'saved_at': datetime.now().isoformat(),
        }
        with open(ENCODER_STATE_FILE, 'w') as f:
            json.dump(data, f, indent=2)
    except Exception as e:
        print(f'[encoder_state] save failed: {e}')


# ═════════════════════════════════════════════════════════════════════════
#  SERVO LOG
# ═════════════════════════════════════════════════════════════════════════

class ServoLog:
    """
    Logs servo movement events with timestamp and cumulative move duration.
    File: ~/lunar_servo_log.csv
    Columns: timestamp, event, direction, duration_s, cumulative_s
    """

    def __init__(self):
        self._lock       = threading.Lock()
        self._move_start: float | None = None
        self._direction  = ''
        self._cumulative = 0.0

        # Load cumulative from last log entry if file exists
        self._cumulative = self._load_cumulative()

        # Write header if file is new
        if not SERVO_LOG_FILE.exists():
            with open(SERVO_LOG_FILE, 'w') as f:
                f.write('timestamp,event,direction,duration_s,cumulative_s\n')

    def _load_cumulative(self) -> float:
        try:
            with open(SERVO_LOG_FILE, 'r') as f:
                lines = f.read().splitlines()
            if len(lines) > 1:
                last = lines[-1].split(',')
                return float(last[4])
        except Exception:
            pass
        return 0.0

    def _write(self, event: str, direction: str,
                duration: float, cumulative: float) -> None:
        ts = datetime.now().isoformat(timespec='milliseconds')
        with open(SERVO_LOG_FILE, 'a') as f:
            f.write(f'{ts},{event},{direction},{duration:.3f},{cumulative:.3f}\n')

    def start_move(self, direction: str) -> None:
        with self._lock:
            if self._move_start is not None:
                # Already moving — log the previous move first
                self._end_move_locked()
            self._move_start = time.monotonic()
            self._direction  = direction
        self._write('START', direction, 0.0, self._cumulative)

    def stop_move(self) -> None:
        with self._lock:
            self._end_move_locked()

    def _end_move_locked(self) -> None:
        if self._move_start is None:
            return
        dur = time.monotonic() - self._move_start
        self._cumulative += dur
        self._write('STOP', self._direction, dur, self._cumulative)
        self._move_start = None
        self._direction  = ''

    @property
    def cumulative_seconds(self) -> float:
        with self._lock:
            extra = 0.0
            if self._move_start is not None:
                extra = time.monotonic() - self._move_start
            return self._cumulative + extra


# ═════════════════════════════════════════════════════════════════════════
#  ENCODER LOG
# ═════════════════════════════════════════════════════════════════════════

ENCODER_LOG_FILE = Path.home() / 'lunar_encoder_log.csv'


class EncoderLog:
    """
    Logs actuator encoder counts whenever position-command events occur.
    File: ~/lunar_encoder_log.csv
    Columns: timestamp, event, encoder_count
    """

    def __init__(self):
        if not ENCODER_LOG_FILE.exists():
            with open(ENCODER_LOG_FILE, 'w') as f:
                f.write('timestamp,event,encoder_count\n')

    def log(self, event: str, count: int) -> None:
        ts = datetime.now().isoformat(timespec='milliseconds')
        with open(ENCODER_LOG_FILE, 'a') as f:
            f.write(f'{ts},{event},{count}\n')


# ═════════════════════════════════════════════════════════════════════════
#  TELEMETRY READER  (reads Serial2 output from Arduino via a second port
#  or the same port if the Arduino multiplexes it — adapt port as needed)
# ═════════════════════════════════════════════════════════════════════════

class TelemetryReader(threading.Thread):
    """
    Background thread that reads the Arduino telemetry stream and extracts
    IMU data and encoder counts.

    The Arduino sends telemetry on Serial2 at 115 200 baud.
    Typically the MiniPC connects to this as a second USB-serial adapter
    (e.g. /dev/ttyACM1 or /dev/ttyUSB0).

    If only one serial port is available the reader will silently do
    nothing (connect=False) — command-only operation still works fine.
    """

    def __init__(self, port: str, on_encoder_cb, logger=None):
        super().__init__(daemon=True)
        self._port         = port
        self._on_encoder   = on_encoder_cb   # callback(count: int)
        self._log          = logger
        self._ser          = None
        self._running      = True
        self.connected     = False

        try:
            self._ser  = serial.Serial(port, 115200, timeout=1.0)
            time.sleep(1.0)
            self._ser.reset_input_buffer()
            self.connected = True
        except serial.SerialException as e:
            if logger:
                logger.warn(f'[telemetry] cannot open {port}: {e}')

    def run(self):
        if not self.connected:
            return

        state    = _TELEM_START
        buf      = bytearray()
        imu_buf  = bytearray()
        enc_buf  = bytearray()
        chk_calc = 0

        while self._running:
            try:
                b = self._ser.read(1)
            except Exception:
                time.sleep(0.1)
                continue
            if not b:
                continue
            byte = b[0]

            if state == _TELEM_START:
                if byte == START:
                    state    = _TELEM_IMU
                    imu_buf  = bytearray()
                    enc_buf  = bytearray()
                    chk_calc = 0

            elif state == _TELEM_IMU:
                imu_buf.append(byte)
                chk_calc ^= byte
                if len(imu_buf) == 24:   # 6 × int32
                    state = _TELEM_ENC_M

            elif state == _TELEM_ENC_M:
                if byte == ENC_MARKER:
                    chk_calc ^= byte
                    state = _TELEM_ENC
                else:
                    state = _TELEM_START   # resync

            elif state == _TELEM_ENC:
                enc_buf.append(byte)
                chk_calc ^= byte
                if len(enc_buf) == 2:
                    state = _TELEM_CHK

            elif state == _TELEM_CHK:
                if byte == chk_calc:
                    enc_count = struct.unpack_from('<H', enc_buf)[0]
                    self._on_encoder(enc_count)
                state = _TELEM_START   # always resync after attempt

    def stop(self):
        self._running = False
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass


# ═════════════════════════════════════════════════════════════════════════
#  ROS NODE
# ═════════════════════════════════════════════════════════════════════════

class JoyToArduino(Node):

    def __init__(self, cmd_port: str, telem_port: str | None):
        super().__init__('joy_to_arduino')

        self._ser   = None
        self._lock  = threading.Lock()
        self._port  = cmd_port
        self._connect(cmd_port)

        # Speed multipliers (independent per side)
        self._speed_left  = SPEED_START
        self._speed_right = SPEED_START

        self._emergency    = False
        self._last_joy     = self.get_clock().now()
        self._prev_btns    = {}
        self._joy_count    = 0

        # D-pad state for edge detection
        self._prev_dpad_lr = 0.0
        self._prev_dpad_ud = 0.0
        self._servo_moving = False   # True while d-pad is held
        self._servo_dir    = ''

        # Trigger state (edge detection for speed down)
        self._lt_pressed   = False
        self._rt_pressed   = False

        # Last sent motor state (change detection)
        self._last_left    = (0, 0)
        self._last_right   = (0, 0)
        self._last_write   = 0.0

        # Latest encoder count (updated by telemetry reader)
        self._enc_count    = 32000   # default mid-range before calibration
        self._enc_lock     = threading.Lock()

        # Logging helpers
        self._servo_log   = ServoLog()
        self._encoder_log = EncoderLog()

        # ── Restore encoder state ─────────────────────────────────────────
        saved = load_encoder_state()
        if saved:
            restored = saved.get('encoder_count', None)
            if restored is not None:
                self.get_logger().info(
                    f'Restoring encoder count {restored} '
                    f'(saved {saved.get("saved_at","?")})')
                self._send_set_encoder(int(restored))
                with self._enc_lock:
                    self._enc_count = int(restored)
                self._encoder_log.log('RESTORE', int(restored))

        # ── Telemetry reader ──────────────────────────────────────────────
        self._telem_reader = None
        if telem_port:
            self._telem_reader = TelemetryReader(
                telem_port,
                on_encoder_cb=self._on_encoder_update,
                logger=self.get_logger(),
            )
            self._telem_reader.start()
            if self._telem_reader.connected:
                self.get_logger().info(
                    f'Telemetry reader connected on {telem_port}')
            else:
                self.get_logger().warn(
                    f'Telemetry port {telem_port} unavailable — '
                    'encoder persistence update disabled')

        # ── ROS subscriptions / timers ────────────────────────────────────
        self.create_subscription(Joy,  '/joy',            self._joy_cb,   10)
        self.create_subscription(Bool, '/emergency_stop', self._estop_cb, 10)
        self.create_timer(0.1, self._watchdog)
        self.create_timer(5.0, self._diagnostics)

        self._banner()

    # ─────────────────────────────────────────────────────────────────────
    # Banner
    # ─────────────────────────────────────────────────────────────────────

    def _banner(self):
        L = self.get_logger().info
        L('=' * 64)
        L('  joy_to_arduino  |  TANK DRIVE  |  miniPC serial bridge')
        L(f'  Cmd port : {self._port}  connected={self._ser is not None}')
        L(f'  LEFT  side: axis {AXIS_LEFT} (Left  stick Y only)')
        L(f'  RIGHT side: axis {AXIS_RIGHT} (Right stick Y only)')
        L(f'  LB=left spd+  LT=left spd-   RB=right spd+  RT=right spd-')
        L(f'  A=DUMP  Y=DRIVE  B=DIG  X=CALIBRATE  (actuator positions)')
        L(f'  D-pad RIGHT=CW  D-pad LEFT=CCW  (servo, release=stop)')
        L(f'  Start=e-stop')
        L(f'  RIGHT_FLIP={RIGHT_FLIP}')
        L(f'  Servo log  : {SERVO_LOG_FILE}')
        L(f'  Encoder log: {ENCODER_LOG_FILE}')
        L(f'  Encoder state: {ENCODER_STATE_FILE}')
        L('=' * 64)

    # ─────────────────────────────────────────────────────────────────────
    # Serial helpers
    # ─────────────────────────────────────────────────────────────────────

    def _connect(self, port: str):
        try:
            self._ser = serial.Serial(port, 115200, timeout=1.0)
            time.sleep(2.0)
            self._ser.reset_input_buffer()
            self.get_logger().info(f'✓ Arduino (cmd) connected: {port}')
            print(f'[SERIAL OK] Connected on {port}', flush=True)
        except serial.SerialException as e:
            self.get_logger().error(f'✗ Serial open failed: {e}')
            self.get_logger().error(
                '  Permanent fix: sudo usermod -aG dialout $USER  (log out/in)')
            print(f'[SERIAL FAIL] {port}: {e}', flush=True)
            print(f'  → Fix: sudo chmod 666 {port}  OR  sudo usermod -aG dialout $USER', flush=True)
            self._ser = None

    def _send(self, device: int, speed: int, direction: int):
        """Send one 5-byte packet. Must be called with self._lock held."""
        pkt = bytes([START, device, speed & 0xFF, direction & 0xFF, END])
        print(f'[TX] AA {device:02X} {speed & 0xFF:02X} {direction & 0xFF:02X} 55', flush=True)
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(pkt)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')
                print(f'[TX FAIL] {e}', flush=True)
                self._ser = None
        else:
            print(f'[TX DROPPED] serial not open — packet lost!', flush=True)

    def _stop_all(self):
        with self._lock:
            self._send(DEV_KILL, 0, 0)
        self._last_left  = (0, 0)
        self._last_right = (0, 0)

    # ─────────────────────────────────────────────────────────────────────
    # Encoder persistence callback
    # ─────────────────────────────────────────────────────────────────────

    def _on_encoder_update(self, count: int):
        with self._enc_lock:
            self._enc_count = count
        save_encoder_state(count)

    def _send_set_encoder(self, count: int):
        """Send 0xCB to tell Arduino the current encoder value."""
        high = (count >> 8) & 0xFF
        low  = count & 0xFF
        with self._lock:
            self._send(CMD_SET_ENC, high, low)

    # ─────────────────────────────────────────────────────────────────────
    # Motor helpers
    # ─────────────────────────────────────────────────────────────────────

    def _to_sd(self, v: float, flip: bool = False):
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

    def _stop_drive(self):
        if self._last_left != (0, 0) or self._last_right != (0, 0):
            with self._lock:
                self._send(DEV_LEFT,  0, 0)
                self._send(DEV_RIGHT, 0, 0)
            self._last_left  = (0, 0)
            self._last_right = (0, 0)

    # ─────────────────────────────────────────────────────────────────────
    # Servo helpers
    # ─────────────────────────────────────────────────────────────────────

    def _servo_start(self, angle: int, direction_name: str):
        with self._lock:
            self._send(DEV_SERVO, angle, 0x01)
        self._servo_moving = True
        self._servo_dir    = direction_name
        self._servo_log.start_move(direction_name)
        self.get_logger().info(f'Servo START {direction_name} (angle={angle})')

    def _servo_stop(self):
        with self._lock:
            self._send(DEV_SERVO, SERVO_STOP, 0x01)
        if self._servo_moving:
            self._servo_log.stop_move()
            cumul = self._servo_log.cumulative_seconds
            self.get_logger().info(
                f'Servo STOP  (cumulative move time: {cumul:.1f}s)')
        self._servo_moving = False
        self._servo_dir    = ''

    # ─────────────────────────────────────────────────────────────────────
    # Actuator position commands
    # ─────────────────────────────────────────────────────────────────────

    def _actuator_position(self, cmd: int, name: str):
        with self._enc_lock:
            current_enc = self._enc_count
        with self._lock:
            self._send(cmd, 0, 0)
        self._encoder_log.log(f'CMD_{name}', current_enc)
        self.get_logger().info(
            f'Actuator → {name}  (encoder now ~{current_enc})')

    # ─────────────────────────────────────────────────────────────────────
    # Edge detection helper
    # ─────────────────────────────────────────────────────────────────────

    def _rising(self, idx: int, cur: int) -> bool:
        prev = self._prev_btns.get(idx, 0)
        self._prev_btns[idx] = cur
        return cur == 1 and prev == 0

    def _dz(self, v: float) -> float:
        return v if abs(v) >= DEADZONE else 0.0

    # ─────────────────────────────────────────────────────────────────────
    # /joy callback  — the heart of the controller logic
    # ─────────────────────────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy):
        self._last_joy   = self.get_clock().now()
        self._joy_count += 1

        def ax(i):  return msg.axes[i]    if i < len(msg.axes)    else 0.0
        def btn(i): return msg.buttons[i] if i < len(msg.buttons) else 0

        # Print raw joy every 30 msgs (~1.5 s) so terminal is readable
        if self._joy_count % 30 == 1:
            axes_str = ' '.join(f'{ax(i):+.2f}' for i in range(min(8, len(msg.axes))))
            btns_str = ''.join(str(btn(i)) for i in range(min(12, len(msg.buttons))))
            print(f'[JOY #{self._joy_count}] axes=[{axes_str}]  btns=[{btns_str}]', flush=True)

        # ── Emergency stop ────────────────────────────────────────────────
        if self._rising(BTN_START, btn(BTN_START)):
            self._emergency = not self._emergency
            if self._emergency:
                self._stop_all()
                self._servo_stop()
                print('[ESTOP] ACTIVATED', flush=True)
                self.get_logger().warn('⬛ EMERGENCY STOP')
            else:
                print('[ESTOP] cleared', flush=True)
                self.get_logger().info('✓ E-stop cleared — ready')

        if self._emergency:
            # Still track button state so edges work after clearing
            for b in (BTN_LB, BTN_RB, BTN_A, BTN_Y, BTN_B, BTN_X):
                self._prev_btns[b] = btn(b)
            self._prev_dpad_lr = ax(DPAD_AXIS_LR)
            self._prev_dpad_ud = ax(DPAD_AXIS_UD)
            return

        # ── Speed control (bumpers / triggers, independent per side) ──────

        # LEFT speed UP — LB rising edge
        if self._rising(BTN_LB, btn(BTN_LB)):
            self._speed_left = round(min(1.0, self._speed_left + SPEED_STEP), 2)
            print(f'[BTN LB] LEFT speed UP → {self._speed_left:.2f}', flush=True)
            self.get_logger().info(f'LEFT  speed: {self._speed_left:.2f}')

        # LEFT speed DOWN — LT pressed (axis goes negative when pressed)
        lt_raw     = ax(AXIS_LT)
        lt_pressed = lt_raw < (1.0 - TRIGGER_THRESHOLD)
        if lt_pressed and not self._lt_pressed:
            self._speed_left = round(max(0.05, self._speed_left - SPEED_STEP), 2)
            print(f'[AXIS LT={lt_raw:.2f}] LEFT speed DOWN → {self._speed_left:.2f}', flush=True)
            self.get_logger().info(f'LEFT  speed: {self._speed_left:.2f}')
        self._lt_pressed = lt_pressed

        # RIGHT speed UP — RB rising edge
        if self._rising(BTN_RB, btn(BTN_RB)):
            self._speed_right = round(min(1.0, self._speed_right + SPEED_STEP), 2)
            print(f'[BTN RB] RIGHT speed UP → {self._speed_right:.2f}', flush=True)
            self.get_logger().info(f'RIGHT speed: {self._speed_right:.2f}')

        # RIGHT speed DOWN — RT pressed
        rt_raw     = ax(AXIS_RT)
        rt_pressed = rt_raw < (1.0 - TRIGGER_THRESHOLD)
        if rt_pressed and not self._rt_pressed:
            self._speed_right = round(max(0.05, self._speed_right - SPEED_STEP), 2)
            print(f'[AXIS RT={rt_raw:.2f}] RIGHT speed DOWN → {self._speed_right:.2f}', flush=True)
            self.get_logger().info(f'RIGHT speed: {self._speed_right:.2f}')
        self._rt_pressed = rt_pressed

        # ── Actuator position commands (A / Y / B) ────────────────────────
        if self._rising(BTN_A, btn(BTN_A)):
            print('[BTN A] → DUMP (0xB3)', flush=True)
            self._actuator_position(CMD_DUMP,  'DUMP')

        if self._rising(BTN_Y, btn(BTN_Y)):
            print('[BTN Y] → DRIVE (0xA9)', flush=True)
            self._actuator_position(CMD_DRIVE, 'DRIVE')

        if self._rising(BTN_B, btn(BTN_B)):
            print('[BTN B] → DIG (0xA7)', flush=True)
            self._actuator_position(CMD_DIG,   'DIG')

        if self._rising(BTN_X, btn(BTN_X)):
            print('[BTN X] → CALIBRATE (0xCA) — retracting to hard stop, zeroing encoder', flush=True)
            self.get_logger().warn('Actuator CALIBRATE — retracting to hard stop')
            with self._lock:
                self._send(CMD_CAL, 0, 0)
            with self._enc_lock:
                self._enc_count = 0
            save_encoder_state(0)
            self._encoder_log.log('CALIBRATE', 0)

        # ── Servo — D-pad left/right (hold to move, release to stop) ──────
        cur_lr = ax(DPAD_AXIS_LR)

        # RIGHT pressed (+1) → CW
        if cur_lr > 0.5 and self._prev_dpad_lr <= 0.5:
            print(f'[DPAD →] Servo CW', flush=True)
            self._servo_start(SERVO_CW, 'CW')

        # LEFT pressed (-1) → CCW
        elif cur_lr < -0.5 and self._prev_dpad_lr >= -0.5:
            print(f'[DPAD ←] Servo CCW', flush=True)
            self._servo_start(SERVO_CCW, 'CCW')

        # Released (back to 0) → stop
        elif abs(cur_lr) < 0.5 and abs(self._prev_dpad_lr) > 0.5:
            print(f'[DPAD release] Servo STOP', flush=True)
            self._servo_stop()

        self._prev_dpad_lr = cur_lr
        # (D-pad UD not used — just track to avoid stale state)
        self._prev_dpad_ud = ax(DPAD_AXIS_UD)

        # ── Tank drive ────────────────────────────────────────────────────
        raw_left  = self._dz(ax(AXIS_LEFT))
        raw_right = self._dz(ax(AXIS_RIGHT))

        if abs(raw_left) < 0.001 and abs(raw_right) < 0.001:
            self._stop_drive()
            return

        left_f  = raw_left  * self._speed_left
        right_f = raw_right * self._speed_right
        self._set_drive(left_f, right_f)

    # ─────────────────────────────────────────────────────────────────────
    # Watchdog — reconnect serial, stop on joy timeout
    # ─────────────────────────────────────────────────────────────────────

    def _watchdog(self):
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
            self.get_logger().warn(
                'Controller lost — motors stopped',
                throttle_duration_sec=2.0)

    # ─────────────────────────────────────────────────────────────────────
    # Diagnostics every 5 s
    # ─────────────────────────────────────────────────────────────────────

    def _diagnostics(self):
        serial_ok = self._ser is not None and self._ser.is_open
        elapsed   = (self.get_clock().now() - self._last_joy).nanoseconds / 1e9
        with self._enc_lock:
            enc = self._enc_count
        msg = (
            f'[DIAG] serial={serial_ok}  joy={self._joy_count}/5s  '
            f'last_joy={elapsed:.1f}s ago  '
            f'spd_L={self._speed_left:.2f}  spd_R={self._speed_right:.2f}  '
            f'enc={enc}  servo={self._servo_moving}({self._servo_dir})  '
            f'estop={self._emergency}'
        )
        self.get_logger().info(msg)
        print(msg, flush=True)
        if self._joy_count == 0:
            warn = '⚠ [DIAG] No /joy messages in last 5s — is joy_node running? Check: ros2 topic echo /joy'
            self.get_logger().warn(warn)
            print(warn, flush=True)
        if not serial_ok:
            ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            print(f'[DIAG] ⚠ Serial DOWN. Available ports: {ports}', flush=True)
        self._joy_count = 0

    # ─────────────────────────────────────────────────────────────────────
    # Emergency stop topic
    # ─────────────────────────────────────────────────────────────────────

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._emergency = True
            self._stop_all()
            self._servo_stop()
            self.get_logger().error('⬛ EMERGENCY STOP (topic)')
        else:
            self._emergency = False
            self.get_logger().info('✓ E-stop cleared (topic)')

    # ─────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop_all()
        self._servo_stop()
        if self._telem_reader:
            self._telem_reader.stop()
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


# ═════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)

    # ── Find command port (first ACM/USB = Arduino command port) ─────────
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    if not ports:
        print('ERROR: No Arduino found at /dev/ttyACM* or /dev/ttyUSB*')
        print('  Permanent fix: sudo usermod -aG dialout $USER  (log out/in)')
        print('  Temporary fix: sudo chmod 666 /dev/ttyACM0')
        rclpy.shutdown()
        return

    cmd_port = ports[0]
    print(f'Command port : {cmd_port}')

    # ── Find telemetry port (second port if available) ────────────────────
    # The Arduino's Serial2 is typically connected via a separate USB-serial
    # adapter.  If only one port exists, telemetry is disabled gracefully.
    telem_port = ports[1] if len(ports) > 1 else None
    if telem_port:
        print(f'Telemetry port: {telem_port}')
    else:
        print('Telemetry port: not found — encoder persistence updates disabled')

    node = JoyToArduino(cmd_port, telem_port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()