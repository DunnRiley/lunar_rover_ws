#!/usr/bin/env python3
"""
minipc_teleop_standalone.py  —  runs ENTIRELY on the MINI PC
Plug Xbox controller directly into miniPC USB, run this script.
No laptop needed. No DDS/network needed for motor control.

Usage:
  python3 minipc_teleop_standalone.py
  python3 minipc_teleop_standalone.py --port /dev/ttyACM1   (override port)

What it does:
  1. Reads Xbox controller via /dev/input/js0 (no ROS joy_node needed)
     OR subscribes to /joy if joy_node is already running
  2. Converts stick input to differential drive
  3. Writes serial commands directly to Arduino

Protocol: [0xAA][Device][Speed][Direction][0x55]
  0x05  LEFT  side (FL+BL)
  0x06  RIGHT side (FR+BR)
  0x08  Both actuators
  0xFF  STOP ALL

Xbox controller mapping (USB):
  Left  stick Y   → forward/backward
  Right stick X   → left/right turn
  LB  (btn 4)     → actuator extend (hold)
  RB  (btn 5)     → actuator retract (hold)
  X   (btn 2)     → speed up
  B   (btn 1)     → speed down
  Start (btn 7)   → emergency stop toggle
"""

import sys
import os
import time
import threading
import glob
import argparse
import serial

# ── Serial protocol ───────────────────────────────────────────────────────
START      = 0xAA
END        = 0x55
DEV_LEFT   = 0x05
DEV_RIGHT  = 0x06
DEV_ACT    = 0x08
DEV_KILL   = 0xFF

# ── Controller tuning ─────────────────────────────────────────────────────
DEADZONE       = 0.12
ANGULAR_SCALE  = 1.2
MAX_MOTOR      = 200
MAX_SERIAL_HZ  = 20
MIN_SERIAL_GAP = 1.0 / MAX_SERIAL_HZ
SPEED_START    = 0.50
SPEED_STEP     = 0.05
JOY_TIMEOUT    = 0.5


# ══════════════════════════════════════════════════════════════════════════
# SERIAL DRIVER
# ══════════════════════════════════════════════════════════════════════════

class ArduinoSerial:
    def __init__(self, port: str):
        self._lock = threading.Lock()
        self._ser  = None
        self._port = port
        self._connect()

    def _connect(self):
        try:
            self._ser = serial.Serial(self._port, 115200, timeout=1.0)
            time.sleep(2.0)
            self._ser.reset_input_buffer()
            print(f'✓ Arduino connected on {self._port}')
        except serial.SerialException as e:
            print(f'✗ Serial failed on {self._port}: {e}')
            self._ser = None

    def send(self, device: int, speed: int, direction: int) -> bool:
        with self._lock:
            if not self._ser or not self._ser.is_open:
                return False
            try:
                self._ser.write(bytes([START, device,
                                       speed & 0xFF, direction & 0xFF, END]))
                return True
            except serial.SerialException as e:
                print(f'Serial write error: {e}')
                self._ser = None
                return False

    def stop_all(self):
        self.send(DEV_KILL, 0, 0)

    def close(self):
        self.stop_all()
        if self._ser and self._ser.is_open:
            self._ser.close()

    @property
    def ok(self):
        return self._ser is not None and self._ser.is_open


# ══════════════════════════════════════════════════════════════════════════
# DRIVE CONTROLLER  (shared by both input methods)
# ══════════════════════════════════════════════════════════════════════════

class DriveController:
    def __init__(self, arduino: ArduinoSerial):
        self._ard        = arduino
        self._speed      = SPEED_START
        self._emergency  = False
        self._last_left  = (0, 0)
        self._last_right = (0, 0)
        self._last_act   = (0, 0)
        self._last_write = 0.0
        self._last_input = time.monotonic()
        self._lock       = threading.Lock()

    def _to_sd(self, v):
        return (min(int(abs(v) * MAX_MOTOR), 255), 0 if v >= 0 else 1)

    def set_drive(self, left_f: float, right_f: float):
        with self._lock:
            if self._emergency:
                return
            self._last_input = time.monotonic()
            l = self._to_sd(left_f)
            r = self._to_sd(right_f)
            now     = time.monotonic()
            changed = (l != self._last_left or r != self._last_right)
            rate_ok = (now - self._last_write) >= MIN_SERIAL_GAP
            if changed and rate_ok:
                self._ard.send(DEV_LEFT,  l[0], l[1])
                self._ard.send(DEV_RIGHT, r[0], r[1])
                self._last_left  = l
                self._last_right = r
                self._last_write = now

    def set_actuator(self, val: int):
        with self._lock:
            if self._emergency:
                return
            sd = (MAX_MOTOR, 0) if val > 0 else \
                 (MAX_MOTOR, 1) if val < 0 else (0, 0)
            if sd != self._last_act:
                self._ard.send(DEV_ACT, sd[0], sd[1])
                self._last_act = sd

    def stop_drive(self):
        with self._lock:
            if self._last_left != (0, 0) or self._last_right != (0, 0):
                self._ard.send(DEV_LEFT,  0, 0)
                self._ard.send(DEV_RIGHT, 0, 0)
                self._last_left  = (0, 0)
                self._last_right = (0, 0)

    def toggle_estop(self):
        with self._lock:
            self._emergency = not self._emergency
            if self._emergency:
                self._ard.stop_all()
                self._last_left  = (0, 0)
                self._last_right = (0, 0)
                self._last_act   = (0, 0)
                print('\n⬛ EMERGENCY STOP ACTIVATED — press Start again to clear')
            else:
                print('\n✓ Emergency stop cleared')

    def speed_up(self):
        with self._lock:
            self._speed = round(min(1.0, self._speed + SPEED_STEP), 2)
            print(f'Speed: {self._speed:.2f}')

    def speed_down(self):
        with self._lock:
            self._speed = round(max(0.05, self._speed - SPEED_STEP), 2)
            print(f'Speed: {self._speed:.2f}')

    def process_axes(self, fwd_raw: float, turn_raw: float):
        """Apply deadzone, scale, differential mix, send."""
        fwd  = fwd_raw  if abs(fwd_raw)  >= DEADZONE else 0.0
        turn = turn_raw if abs(turn_raw) >= DEADZONE else 0.0

        with self._lock:
            spd = self._speed

        if fwd == 0.0 and turn == 0.0:
            self.stop_drive()
            return

        left  = (fwd - turn * ANGULAR_SCALE) * spd
        right = (fwd + turn * ANGULAR_SCALE) * spd
        mx    = max(abs(left), abs(right), 1.0)
        self.set_drive(left / mx, right / mx)

    def watchdog(self):
        """Call periodically — stops motors on input timeout."""
        with self._lock:
            if self._emergency:
                return
        elapsed = time.monotonic() - self._last_input
        if elapsed > JOY_TIMEOUT:
            self.stop_drive()


# ══════════════════════════════════════════════════════════════════════════
# INPUT METHOD 1: raw /dev/input/jsX  (no ROS needed)
# ══════════════════════════════════════════════════════════════════════════

import struct

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS   = 0x02
JS_EVENT_INIT   = 0x80

class RawJoystick:
    """Reads /dev/input/jsX directly — no ROS required."""

    def __init__(self, device: str, drive: DriveController):
        self._dev   = device
        self._drive = drive
        self._axes  = [0.0] * 10
        self._btns  = [0]   * 20
        self._prev  = {}
        self._stop  = False
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _rising(self, idx: int, cur: int) -> bool:
        prev = self._prev.get(idx, 0)
        self._prev[idx] = cur
        return cur == 1 and prev == 0

    def _read_loop(self):
        try:
            with open(self._dev, 'rb') as f:
                print(f'✓ Joystick opened: {self._dev}')
                print('  Move left stick to drive, right stick to turn')
                print('  LB=extend  RB=retract  X=spd+  B=spd-  Start=estop\n')
                while not self._stop:
                    data = f.read(8)
                    if len(data) < 8:
                        break
                    _time, value, etype, number = struct.unpack('IhBB', data)
                    etype &= ~JS_EVENT_INIT

                    if etype == JS_EVENT_AXIS and number < len(self._axes):
                        self._axes[number] = value / 32767.0
                        # Axes: 1=left Y, 3=right X (standard Xbox USB)
                        self._drive.process_axes(self._axes[1], self._axes[3])

                    elif etype == JS_EVENT_BUTTON and number < len(self._btns):
                        self._btns[number] = value
                        if self._rising(7, self._btns[7]):   # Start
                            self._drive.toggle_estop()
                        elif self._rising(2, self._btns[2]): # X
                            self._drive.speed_up()
                        elif self._rising(1, self._btns[1]): # B
                            self._drive.speed_down()

                        lb = self._btns[4]
                        rb = self._btns[5]
                        if lb:
                            self._drive.set_actuator(1)
                        elif rb:
                            self._drive.set_actuator(-1)
                        else:
                            self._drive.set_actuator(0)

        except OSError as e:
            print(f'Joystick read error: {e}')
            print('  Try: ls /dev/input/js*  to find your controller device')

    def stop(self):
        self._stop = True


# ══════════════════════════════════════════════════════════════════════════
# INPUT METHOD 2: ROS /joy topic  (if joy_node is running)
# ══════════════════════════════════════════════════════════════════════════

def run_with_ros(arduino: ArduinoSerial):
    """Fall back to ROS joy subscription if available."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Joy
        from std_msgs.msg import Bool
    except ImportError:
        print('rclpy not available — cannot use ROS mode')
        return False

    rclpy.init()
    drive = DriveController(arduino)

    class JoyNode(Node):
        def __init__(self):
            super().__init__('minipc_teleop')
            self._prev = {}
            self.create_subscription(Joy,  '/joy',            self._joy_cb,   10)
            self.create_subscription(Bool, '/emergency_stop', self._estop_cb, 10)
            self.create_timer(0.1, drive.watchdog)
            self.create_timer(3.0, self._heartbeat)
            self._count = 0
            self.get_logger().info('miniPC standalone teleop via ROS /joy')

        def _ax(self, msg, i):
            return msg.axes[i] if i < len(msg.axes) else 0.0

        def _btn(self, msg, i):
            return msg.buttons[i] if i < len(msg.buttons) else 0

        def _rising(self, idx, cur):
            prev = self._prev.get(idx, 0)
            self._prev[idx] = cur
            return cur == 1 and prev == 0

        def _joy_cb(self, msg):
            self._count += 1
            btn = lambda i: self._btn(msg, i)
            ax  = lambda i: self._ax(msg, i)

            if self._rising(7, btn(7)):
                drive.toggle_estop()
            if self._rising(2, btn(2)):
                drive.speed_up()
            if self._rising(1, btn(1)):
                drive.speed_down()

            lb, rb = btn(4), btn(5)
            if lb:    drive.set_actuator(1)
            elif rb:  drive.set_actuator(-1)
            else:     drive.set_actuator(0)

            drive.process_axes(ax(1), ax(3))

        def _estop_cb(self, msg):
            if msg.data:
                drive._emergency = True
                arduino.stop_all()

        def _heartbeat(self):
            if self._count == 0:
                self.get_logger().warn(
                    'No /joy messages received in 3s — is joy_node running?')
            self._count = 0

    node = JoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return True


# ══════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════

def find_arduino(override=None):
    if override:
        return override
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    if not ports:
        return None
    if len(ports) > 1:
        print(f'Multiple serial ports found: {ports}')
        print(f'Using {ports[0]} — pass --port /dev/ttyACMx to override')
    return ports[0]


def find_joystick():
    devs = glob.glob('/dev/input/js*')
    return devs[0] if devs else None


def main():
    parser = argparse.ArgumentParser(description='miniPC standalone teleop')
    parser.add_argument('--port',     help='Arduino serial port (e.g. /dev/ttyACM0)')
    parser.add_argument('--js',       help='Joystick device (e.g. /dev/input/js0)')
    parser.add_argument('--ros',      action='store_true',
                        help='Force ROS /joy mode instead of raw joystick')
    parser.add_argument('--no-ros',   action='store_true',
                        help='Force raw joystick mode, skip ROS')
    args = parser.parse_args()

    print('=' * 55)
    print('  Lunar Rover — miniPC Standalone Teleop')
    print('=' * 55)

    # ── Find Arduino ──────────────────────────────────────────────────────
    port = find_arduino(args.port)
    if not port:
        print('\n✗ No Arduino found!')
        print('  Check USB cable and run:  ls /dev/ttyACM*  ls /dev/ttyUSB*')
        print('  You may need:  sudo usermod -aG dialout $USER  (then log out/in)')
        sys.exit(1)

    arduino = ArduinoSerial(port)
    if not arduino.ok:
        print(f'\n✗ Could not open {port}')
        print('  Try:  sudo chmod 666 /dev/ttyACM0')
        sys.exit(1)

    # ── Choose input method ───────────────────────────────────────────────
    use_ros = False
    if not args.no_ros:
        # Try ROS first if available and joy_node seems to be running
        try:
            import rclpy
            use_ros = True
        except ImportError:
            pass

    if args.ros:
        use_ros = True
    if args.no_ros:
        use_ros = False

    if use_ros and not args.js:
        print('\nMode: ROS /joy subscription')
        print('  (pass --no-ros to use raw joystick instead)')
        run_with_ros(arduino)
    else:
        # Raw joystick mode
        js_dev = args.js or find_joystick()
        if not js_dev:
            print('\n✗ No joystick found at /dev/input/js*')
            print('  Is the controller plugged into THIS machine (miniPC)?')
            print('  Run:  ls /dev/input/js*')
            arduino.close()
            sys.exit(1)

        print(f'\nMode: Raw joystick ({js_dev})')
        print('  (pass --ros to use ROS /joy topic instead)')

        drive = DriveController(arduino)
        joy   = RawJoystick(js_dev, drive)

        # Watchdog loop
        try:
            while True:
                time.sleep(0.1)
                drive.watchdog()
        except KeyboardInterrupt:
            print('\nShutting down...')
        finally:
            joy.stop()
            arduino.close()
            print('✓ Motors stopped, serial closed')


if __name__ == '__main__':
    main()