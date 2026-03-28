#!/usr/bin/env python3
"""
joy_to_arduino.py  --  runs on the MINI PC

TANK DRIVE LAYOUT
  Left  stick Y  (axis 1)  --  LEFT  wheels forward / backward
  Right stick Y  (axis 4)  --  RIGHT wheels forward / backward

SPEED CONTROL  (independent per side)
  LB (btn 4)  ->  LEFT  speed UP   (+5% per press, rising edge)
  LT (axis 2) ->  LEFT  speed DOWN (-5% per press, rising edge)
  RB (btn 5)  ->  RIGHT speed UP   (+5% per press, rising edge)
  RT (axis 5) ->  RIGHT speed DOWN (-5% per press, rising edge)

ACTUATOR POSITION CONTROL  (encoder-based, buttons)
  A (btn 0)  ->  DUMP position    (0xB3)
  Y (btn 3)  ->  DRIVE position   (0xA9)
  B (btn 1)  ->  DIG  position    (0xA7)
  D-pad UP   ->  Actuator EXTEND  (hold to move, release to stop)
  D-pad DOWN ->  Actuator RETRACT (hold to move, release to stop)

SERVO CONTROL  (continuous 360 servo)
  D-pad RIGHT  ->  CW  movement (angle 135) while held; STOP on release
  D-pad LEFT   ->  CCW movement (angle  45) while held; STOP on release

EMERGENCY STOP
  Start (btn 7) -> toggle emergency stop

DELAY SIMULATION
  /delay_enabled (Bool) -> toggle 5-second comms delay on/off
  When ON:  all drive/actuator/servo commands held for 5 s before Arduino
  /delay_status  (String, JSON) -> timer state published to laptop GUI

SERIAL PROTOCOL  [0xAA][Device][Speed][Direction][0x55]
  0x05  LEFT  side (FL + BL)
  0x06  RIGHT side (FR + BR)
  0x08  Both actuators (manual move)
  0x11  Servo  (Speed = angle 45/90/135, Direction = 0x01)
  0xA7  Actuator -> DIG   position
  0xA9  Actuator -> DRIVE position
  0xB3  Actuator -> DUMP  position
  0xCA  Calibrate actuator (retract to hard stop, zero encoder)
  0xCB  Set encoder count  (Speed = high byte, Direction = low byte)
  0xFF  STOP ALL
"""

import collections
import glob
import json
import os
import struct
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
import serial

# ── Serial protocol ───────────────────────────────────────────────────────
START       = 0xAA
END         = 0x55
ENC_MARKER  = 0xA5

DEV_LEFT    = 0x06
DEV_RIGHT   = 0x05
DEV_SERVO   = 0x11
DEV_ACT     = 0x08
DEV_KILL    = 0xFF

CMD_DIG     = 0xA7
CMD_DRIVE   = 0xA9
CMD_DUMP    = 0xB3
CMD_CAL     = 0xCA
CMD_SET_ENC = 0xCB

SERVO_STOP  = 90
SERVO_CCW   = 45
SERVO_CW    = 135

# ── Persistence files ─────────────────────────────────────────────────────
ENCODER_STATE_FILE = Path.home() / '.lunar_encoder_state.json'
SERVO_LOG_FILE     = Path.home() / 'lunar_servo_log.csv'
ENCODER_LOG_FILE   = Path.home() / 'lunar_encoder_log.csv'

# ── Controller mapping ────────────────────────────────────────────────────
AXIS_LEFT   = 1
AXIS_RIGHT  = 4
AXIS_LT     = 2
AXIS_RT     = 5
TRIGGER_THRESHOLD = 0.5

BTN_LB      = 4
BTN_RB      = 5
BTN_A       = 0
BTN_Y       = 3
BTN_B       = 1
BTN_X       = 2
BTN_START   = 7

DPAD_AXIS_LR = 6
DPAD_AXIS_UD = 7

# ── Tuning ────────────────────────────────────────────────────────────────
RIGHT_FLIP      = True
DEADZONE        = 0.10
MAX_MOTOR       = 200
MAX_SERIAL_HZ   = 20
MIN_SERIAL_GAP  = 1.0 / MAX_SERIAL_HZ
SPEED_START     = 1.0
SPEED_STEP      = 0.05
JOY_TIMEOUT     = 0.5
ACT_MANUAL_SPD  = 200

# ── Telemetry parse states ────────────────────────────────────────────────
_TELEM_START = 0
_TELEM_IMU   = 1
_TELEM_ENC_M = 2
_TELEM_ENC   = 3
_TELEM_CHK   = 4


# ===========================================================================
#  PERSISTENCE HELPERS
# ===========================================================================

def load_encoder_state() -> dict:
    try:
        with open(ENCODER_STATE_FILE, 'r') as f:
            return json.load(f)
    except Exception:
        return {}


def save_encoder_state(enc_count: int) -> None:
    try:
        with open(ENCODER_STATE_FILE, 'w') as f:
            json.dump({'encoder_count': enc_count,
                       'saved_at': datetime.now().isoformat()}, f, indent=2)
    except Exception as e:
        print(f'[encoder_state] save failed: {e}')


# ===========================================================================
#  SERVO LOG
# ===========================================================================

class ServoLog:
    def __init__(self):
        self._lock       = threading.Lock()
        self._move_start = None
        self._direction  = ''
        self._cumulative = self._load_cumulative()
        if not SERVO_LOG_FILE.exists():
            with open(SERVO_LOG_FILE, 'w') as f:
                f.write('timestamp,event,direction,duration_s,cumulative_s\n')

    def _load_cumulative(self) -> float:
        try:
            with open(SERVO_LOG_FILE, 'r') as f:
                lines = f.read().splitlines()
            if len(lines) > 1:
                return float(lines[-1].split(',')[4])
        except Exception:
            pass
        return 0.0

    def _write(self, event, direction, duration, cumulative):
        ts = datetime.now().isoformat(timespec='milliseconds')
        with open(SERVO_LOG_FILE, 'a') as f:
            f.write(f'{ts},{event},{direction},{duration:.3f},{cumulative:.3f}\n')

    def start_move(self, direction: str):
        with self._lock:
            if self._move_start is not None:
                self._end_move_locked()
            self._move_start = time.monotonic()
            self._direction  = direction
        self._write('START', direction, 0.0, self._cumulative)

    def stop_move(self):
        with self._lock:
            self._end_move_locked()

    def _end_move_locked(self):
        if self._move_start is None:
            return
        dur = time.monotonic() - self._move_start
        self._cumulative += dur
        self._write('STOP', self._direction, dur, self._cumulative)
        self._move_start = None
        self._direction  = ''

    @property
    def cumulative_seconds(self):
        with self._lock:
            extra = (time.monotonic() - self._move_start
                     if self._move_start is not None else 0.0)
            return self._cumulative + extra


# ===========================================================================
#  ENCODER LOG
# ===========================================================================

class EncoderLog:
    def __init__(self):
        if not ENCODER_LOG_FILE.exists():
            with open(ENCODER_LOG_FILE, 'w') as f:
                f.write('timestamp,event,encoder_count\n')

    def log(self, event: str, count: int):
        ts = datetime.now().isoformat(timespec='milliseconds')
        with open(ENCODER_LOG_FILE, 'a') as f:
            f.write(f'{ts},{event},{count}\n')


# ===========================================================================
#  TELEMETRY READER
# ===========================================================================

class TelemetryReader(threading.Thread):
    def __init__(self, port: str, on_encoder_cb, logger=None):
        super().__init__(daemon=True)
        self._port       = port
        self._on_encoder = on_encoder_cb
        self._log        = logger
        self._ser        = None
        self._running    = True
        self.connected   = False
        try:
            self._ser = serial.Serial(port, 115200, timeout=1.0)
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
                if len(imu_buf) == 24:
                    state = _TELEM_ENC_M
            elif state == _TELEM_ENC_M:
                if byte == ENC_MARKER:
                    chk_calc ^= byte
                    state = _TELEM_ENC
                else:
                    state = _TELEM_START
            elif state == _TELEM_ENC:
                enc_buf.append(byte)
                chk_calc ^= byte
                if len(enc_buf) == 2:
                    state = _TELEM_CHK
            elif state == _TELEM_CHK:
                if byte == chk_calc:
                    enc_count = struct.unpack_from('<H', enc_buf)[0]
                    self._on_encoder(enc_count)
                state = _TELEM_START

    def stop(self):
        self._running = False
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass


# ===========================================================================
#  ROS NODE
# ===========================================================================

class JoyToArduino(Node):

    def __init__(self, cmd_port: str, telem_port):
        super().__init__('joy_to_arduino')

        self._ser  = None
        self._lock = threading.Lock()
        self._port = cmd_port
        self._connect(cmd_port)   # open serial first -- everything below may use it

        # Speed multipliers
        self._speed_left  = SPEED_START
        self._speed_right = SPEED_START

        self._emergency   = False
        self._last_joy    = self.get_clock().now()
        self._prev_btns   = {}
        self._joy_count   = 0

        # D-pad edge detection
        self._prev_dpad_lr = 0.0
        self._prev_dpad_ud = 0.0
        self._servo_moving = False
        self._servo_dir    = ''
        self._act_moving   = False

        self._lt_prev = 1.0
        self._rt_prev = 1.0

        # Rate-limiting state
        self._last_left  = (0, 0)
        self._last_right = (0, 0)
        self._last_write = 0.0

        # Encoder
        self._enc_count = 32000
        self._enc_lock  = threading.Lock()

        # Logging
        self._servo_log   = ServoLog()
        self._encoder_log = EncoderLog()

        # ── Delay simulation ─────────────────────────────────────────────
        # Delay OFF:  _send() writes to serial immediately.
        # Delay ON:   _send() timestamps and queues the packet.
        #             _flush_delay_queue() (20 Hz ROS timer) pops and
        #             writes packets whose age >= _delay_sec.
        # No external dependency -- just a collections.deque + a timer.
        self._delay_enabled = False
        self._delay_sec     = 5.0
        self._delay_queue   = collections.deque()   # (enqueue_time, pkt_bytes)
        self._delay_lock    = threading.Lock()
        self._last_cmd_t    = None   # monotonic time of last _send(), for GUI

        _rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        # ─────────────────────────────────────────────────────────────────

        # Restore encoder state -- _send() fully works at this point
        saved = load_encoder_state()
        if saved:
            restored = saved.get('encoder_count')
            if restored is not None:
                self.get_logger().info(
                    f'Restoring encoder count {restored} '
                    f'(saved {saved.get("saved_at","?")})')
                self._send_set_encoder(int(restored))
                with self._enc_lock:
                    self._enc_count = int(restored)
                self._encoder_log.log('RESTORE', int(restored))

        # Telemetry reader
        self._telem_reader = None
        if telem_port:
            self._telem_reader = TelemetryReader(
                telem_port,
                on_encoder_cb=self._on_encoder_update,
                logger=self.get_logger())
            self._telem_reader.start()
            if self._telem_reader.connected:
                self.get_logger().info(f'Telemetry reader connected on {telem_port}')

        self.create_subscription(Joy,  '/joy',            self._joy_cb,         10)
        self.create_subscription(Bool, '/emergency_stop', self._estop_cb,        10)
        self.create_subscription(Bool, '/delay_enabled',  self._delay_toggle_cb, _rel)
        self._delay_pub = self.create_publisher(String, '/delay_status', _rel)

        self.create_timer(0.05, self._flush_delay_queue)   # 20 Hz drain
        self.create_timer(0.1,  self._watchdog)
        self.create_timer(0.1,  self._publish_delay_status)
        self.create_timer(5.0,  self._diagnostics)

        self._banner()

    # ── Delay methods ─────────────────────────────────────────────────────

    def _delay_toggle_cb(self, msg: Bool):
        self._delay_enabled = msg.data
        state = 'ENABLED' if msg.data else 'DISABLED'
        self.get_logger().info(f'Communication delay {state}')
        if not msg.data:
            # Flush queue immediately when delay is turned off
            with self._delay_lock:
                while self._delay_queue:
                    _, pkt = self._delay_queue.popleft()
                    self._raw_serial_write(pkt)

    def _flush_delay_queue(self):
        """20 Hz ROS timer: releases queued packets once their hold time expires."""
        if not self._delay_enabled:
            return
        now = time.monotonic()
        with self._delay_lock:
            while self._delay_queue:
                enqueue_t, pkt = self._delay_queue[0]
                if now - enqueue_t < self._delay_sec:
                    break
                self._delay_queue.popleft()
                self._raw_serial_write(pkt)

    def _publish_delay_status(self):
        now     = time.monotonic()
        elapsed = (now - self._last_cmd_t) if self._last_cmd_t is not None else None
        with self._delay_lock:
            pending = len(self._delay_queue)
        m      = String()
        m.data = json.dumps({
            'delay_enabled': self._delay_enabled,
            'delay_sec':     self._delay_sec,
            'last_cmd_age':  round(elapsed, 2) if elapsed is not None else None,
            'pending':       pending,
        })
        self._delay_pub.publish(m)

    # ── Banner ────────────────────────────────────────────────────────────

    def _banner(self):
        L = self.get_logger().info
        L('=' * 64)
        L('  joy_to_arduino  |  TANK DRIVE  |  miniPC serial bridge')
        L(f'  Cmd port : {self._port}  connected={self._ser is not None}')
        L(f'  LEFT  side: axis {AXIS_LEFT} (Left  stick Y)')
        L(f'  RIGHT side: axis {AXIS_RIGHT} (Right stick Y)')
        L(f'  LB=left spd+  LT=left spd-  RB=right spd+  RT=right spd-')
        L(f'  A=DUMP  Y=DRIVE  B=DIG  X=CALIBRATE  (actuator positions)')
        L(f'  D-pad RIGHT=Servo CW  D-pad LEFT=Servo CCW  (release=stop)')
        L(f'  D-pad UP=Act EXTEND   D-pad DOWN=Act RETRACT (hold to move)')
        L(f'  Start=e-stop')
        L('=' * 64)

    # ── Serial ────────────────────────────────────────────────────────────

    def _connect(self, port: str):
        try:
            self._ser = serial.Serial(port, 115200, timeout=1.0)
            time.sleep(2.0)
            self._ser.reset_input_buffer()
            self.get_logger().info(f'Arduino (cmd) connected: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial open failed: {e}')
            self._ser = None

    def _raw_serial_write(self, pkt: bytes):
        """Write a pre-built packet directly to serial. Never queued."""
        print(f'[TX] {" ".join(f"{b:02X}" for b in pkt)}', flush=True)
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(pkt)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')
                self._ser = None

    def _send(self, device: int, speed: int, direction: int):
        """
        Send one command to the Arduino.
        Delay OFF: writes to serial immediately.
        Delay ON:  queues the packet for _delay_sec seconds.
        Updates _last_cmd_t for the GUI timer in both cases.
        """
        pkt = bytes([START, device, speed & 0xFF, direction & 0xFF, END])
        self._last_cmd_t = time.monotonic()
        if self._delay_enabled:
            with self._delay_lock:
                self._delay_queue.append((self._last_cmd_t, pkt))
        else:
            self._raw_serial_write(pkt)

    def _stop_all(self):
        with self._lock:
            self._send(DEV_KILL, 0, 0)
        self._last_left  = (0, 0)
        self._last_right = (0, 0)

    # ── Encoder ───────────────────────────────────────────────────────────

    def _on_encoder_update(self, count: int):
        with self._enc_lock:
            self._enc_count = count
        save_encoder_state(count)

    def _send_set_encoder(self, count: int):
        high = (count >> 8) & 0xFF
        low  = count & 0xFF
        with self._lock:
            self._send(CMD_SET_ENC, high, low)

    # ── Motor helpers ─────────────────────────────────────────────────────

    def _to_sd(self, v: float, flip: bool = False):
        spd = min(int(abs(v) * MAX_MOTOR), 255)
        d   = 0 if v >= 0 else 1
        if flip:
            d = 1 - d
        return (spd, d)

    def _set_drive(self, left_f: float, right_f: float):
        l   = self._to_sd(left_f,  flip=False)
        r   = self._to_sd(right_f, flip=RIGHT_FLIP)
        now = time.monotonic()
        if (now - self._last_write) >= MIN_SERIAL_GAP:
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

    # ── Servo ─────────────────────────────────────────────────────────────

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
            self.get_logger().info(
                f'Servo STOP (cumul={self._servo_log.cumulative_seconds:.1f}s)')
        self._servo_moving = False
        self._servo_dir    = ''

    # ── Actuator position commands ────────────────────────────────────────

    def _actuator_position(self, cmd: int, name: str):
        with self._enc_lock:
            current_enc = self._enc_count
        with self._lock:
            self._send(cmd, 0, 0)
        self._encoder_log.log(f'CMD_{name}', current_enc)
        self.get_logger().info(f'Actuator -> {name}  (encoder ~{current_enc})')

    # ── Edge detection ────────────────────────────────────────────────────

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

        if self._joy_count % 30 == 1:
            axes_str = ' '.join(f'{ax(i):+.2f}' for i in range(min(8, len(msg.axes))))
            btns_str = ''.join(str(btn(i)) for i in range(min(12, len(msg.buttons))))
            print(f'[JOY #{self._joy_count}] axes=[{axes_str}]  btns=[{btns_str}]',
                  flush=True)

        # ── Emergency stop ────────────────────────────────────────────────
        if self._rising(BTN_START, btn(BTN_START)):
            self._emergency = not self._emergency
            if self._emergency:
                self._stop_all()
                self._servo_stop()
                print('[ESTOP] ACTIVATED', flush=True)
            else:
                print('[ESTOP] cleared', flush=True)

        if self._emergency:
            for b in (BTN_LB, BTN_RB, BTN_A, BTN_Y, BTN_B, BTN_X):
                self._prev_btns[b] = btn(b)
            self._prev_dpad_lr = ax(DPAD_AXIS_LR)
            self._prev_dpad_ud = ax(DPAD_AXIS_UD)
            self._lt_prev      = ax(AXIS_LT)
            self._rt_prev      = ax(AXIS_RT)
            return

        # ── Speed control -- LB/RB bumpers (rising edge) ─────────────────
        if self._rising(BTN_LB, btn(BTN_LB)):
            self._speed_left = round(min(1.0, self._speed_left + SPEED_STEP), 2)
            print(f'[BTN LB] LEFT speed UP -> {self._speed_left:.2f}', flush=True)
            self.get_logger().info(f'LEFT  speed: {self._speed_left:.2f}')

        if self._rising(BTN_RB, btn(BTN_RB)):
            self._speed_right = round(min(1.0, self._speed_right + SPEED_STEP), 2)
            print(f'[BTN RB] RIGHT speed UP -> {self._speed_right:.2f}', flush=True)
            self.get_logger().info(f'RIGHT speed: {self._speed_right:.2f}')

        # ── Speed control -- LT trigger (falling-edge detection) ──────────
        lt_cur         = ax(AXIS_LT)
        lt_pressed_now = lt_cur < TRIGGER_THRESHOLD
        lt_was_pressed = self._lt_prev < TRIGGER_THRESHOLD
        if lt_pressed_now and not lt_was_pressed:
            self._speed_left = round(max(0.05, self._speed_left - SPEED_STEP), 2)
            print(f'[LT={lt_cur:+.2f}] LEFT speed DOWN -> {self._speed_left:.2f}',
                  flush=True)
            self.get_logger().info(f'LEFT  speed: {self._speed_left:.2f}')
        self._lt_prev = lt_cur

        # ── Speed control -- RT trigger (falling-edge detection) ──────────
        rt_cur         = ax(AXIS_RT)
        rt_pressed_now = rt_cur < TRIGGER_THRESHOLD
        rt_was_pressed = self._rt_prev < TRIGGER_THRESHOLD
        if rt_pressed_now and not rt_was_pressed:
            self._speed_right = round(max(0.05, self._speed_right - SPEED_STEP), 2)
            print(f'[RT={rt_cur:+.2f}] RIGHT speed DOWN -> {self._speed_right:.2f}',
                  flush=True)
            self.get_logger().info(f'RIGHT speed: {self._speed_right:.2f}')
        self._rt_prev = rt_cur

        # ── Actuator position commands (A / Y / B / X) ───────────────────
        if self._rising(BTN_A, btn(BTN_A)):
            print('[BTN A] -> DUMP (0xB3)', flush=True)
            self._actuator_position(CMD_DUMP, 'DUMP')

        if self._rising(BTN_Y, btn(BTN_Y)):
            print('[BTN Y] -> DRIVE (0xA9)', flush=True)
            self._actuator_position(CMD_DRIVE, 'DRIVE')

        if self._rising(BTN_B, btn(BTN_B)):
            print('[BTN B] -> DIG (0xA7)', flush=True)
            self._actuator_position(CMD_DIG, 'DIG')

        if self._rising(BTN_X, btn(BTN_X)):
            print('[BTN X] -> CALIBRATE (0xCA)', flush=True)
            self.get_logger().warn('Actuator CALIBRATE -- retracting to hard stop')
            with self._lock:
                self._send(CMD_CAL, 0, 0)
            with self._enc_lock:
                self._enc_count = 0
            save_encoder_state(0)
            self._encoder_log.log('CALIBRATE', 0)

        # ── D-pad LEFT/RIGHT -> Servo (hold to move, release to stop) ────
        cur_lr = ax(DPAD_AXIS_LR)
        if cur_lr > 0.5 and self._prev_dpad_lr <= 0.5:
            print('[DPAD ->] Servo CW', flush=True)
            self._servo_start(SERVO_CW, 'CW')
        elif cur_lr < -0.5 and self._prev_dpad_lr >= -0.5:
            print('[DPAD <-] Servo CCW', flush=True)
            self._servo_start(SERVO_CCW, 'CCW')
        elif abs(cur_lr) < 0.5 and abs(self._prev_dpad_lr) > 0.5:
            print('[DPAD LR release] Servo STOP', flush=True)
            self._servo_stop()
        self._prev_dpad_lr = cur_lr

        # ── D-pad UP/DOWN -> Actuator manual (hold to move, release to stop)
        cur_ud = ax(DPAD_AXIS_UD)

        if cur_ud > 0.5 and self._prev_dpad_ud <= 0.5:
            print('[DPAD up] Actuator EXTEND (manual hold)', flush=True)
            with self._lock:
                self._send(DEV_ACT, ACT_MANUAL_SPD, 0)
            self._act_moving = True

        elif cur_ud < -0.5 and self._prev_dpad_ud >= -0.5:
            print('[DPAD down] Actuator RETRACT (manual hold)', flush=True)
            with self._lock:
                self._send(DEV_ACT, ACT_MANUAL_SPD, 1)
            self._act_moving = True

        elif abs(cur_ud) < 0.5 and abs(self._prev_dpad_ud) > 0.5:
            print('[DPAD UD release] Actuator STOP', flush=True)
            with self._lock:
                self._send(DEV_ACT, 0, 0)
            self._act_moving = False

        self._prev_dpad_ud = cur_ud

        # ── Tank drive ────────────────────────────────────────────────────
        raw_left  = self._dz(ax(AXIS_LEFT))
        raw_right = self._dz(ax(AXIS_RIGHT))

        if abs(raw_left) < 0.001 and abs(raw_right) < 0.001:
            self._stop_drive()
            return

        left_f  = raw_left  * self._speed_left
        right_f = raw_right * self._speed_right
        self._set_drive(left_f, right_f)

    # ── Watchdog ──────────────────────────────────────────────────────────

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

    # ── Diagnostics ───────────────────────────────────────────────────────

    def _diagnostics(self):
        serial_ok = self._ser is not None and self._ser.is_open
        elapsed   = (self.get_clock().now() - self._last_joy).nanoseconds / 1e9
        with self._enc_lock:
            enc = self._enc_count
        with self._delay_lock:
            pending = len(self._delay_queue)
        msg = (
            f'[DIAG] serial={serial_ok}  joy={self._joy_count}/5s  '
            f'last_joy={elapsed:.1f}s  '
            f'spd_L={self._speed_left:.2f}  spd_R={self._speed_right:.2f}  '
            f'enc={enc}  delay={self._delay_enabled}(pending={pending})  '
            f'servo={self._servo_moving}({self._servo_dir})  '
            f'act_manual={self._act_moving}  estop={self._emergency}'
        )
        self.get_logger().info(msg)
        print(msg, flush=True)
        if self._joy_count == 0:
            self.get_logger().warn(
                'No /joy messages -- is joy_node running? '
                'ros2 topic echo /joy')
        self._joy_count = 0

    # ── Emergency stop topic ──────────────────────────────────────────────

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._emergency = True
            self._stop_all()
            self._servo_stop()
            self.get_logger().error('EMERGENCY STOP (topic)')
        else:
            self._emergency = False
            self.get_logger().info('E-stop cleared (topic)')

    # ── Cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop_all()
        self._servo_stop()
        if self._telem_reader:
            self._telem_reader.stop()
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


# ===========================================================================
#  ENTRY POINT
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    if not ports:
        print('ERROR: No Arduino found at /dev/ttyACM* or /dev/ttyUSB*')
        rclpy.shutdown()
        return

    cmd_port   = ports[0]
    telem_port = ports[1] if len(ports) > 1 else None
    print(f'Command port   : {cmd_port}')
    print(f'Telemetry port : {telem_port or "not found"}')

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