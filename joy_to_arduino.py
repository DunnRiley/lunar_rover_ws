#!/usr/bin/env python3
import glob
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
import serial

# ── Protocol ──────────────────────────────────────────────────────────────────
START = 0xAA
END   = 0x55

def pkt(device, speed=0, direction=0, lobyte=0):
    d, sp, di, lo = device & 0xFF, speed & 0xFF, direction & 0xFF, lobyte & 0xFF
    return bytes([START, d, sp, di, lo, d ^ sp ^ di ^ lo, END])

DEV_LEFT  = 0x06
DEV_RIGHT = 0x05
DEV_ACT   = 0x08
DEV_SERVO = 0x11
DEV_KILL  = 0xFF
CMD_DIG   = 0xA7
CMD_DIG2  = 0x93
CMD_DRIVE = 0xA9
CMD_CAL   = 0xCA
CMD_DUMP  = 0xB3
SERVO_STOP, SERVO_CCW, SERVO_CW = 90, 45, 135

# ── Mapping ───────────────────────────────────────────────────────────────────
AXIS_LEFT  = 1
AXIS_RIGHT = 4
BTN_LB, BTN_RB = 4, 5
AXIS_LT    = 2
AXIS_RT    = 5
BTN_A, BTN_Y, BTN_B, BTN_X, BTN_START = 0, 3, 1, 2, 7
DPAD_LR = 6
DPAD_UD = 7
TRIG_TH = 0.5
DEADZONE = 0.10
MAX_MOTOR = 190
SPEED_STEP = 0.05
JOY_TIMEOUT_S = 0.5
RIGHT_FLIP = False   # compensates for invertRightDriveDirection=True in firmware


class JoyArduino(Node):

    def __init__(self, port):
        super().__init__("joy_to_arduino")
        self._port = port
        self._ser  = None
        self._lock = threading.Lock()
        self._connect(port)

        self._spd_l = 1.0; self._spd_r = 1.0
        self._estop = False
        self._last_joy = self.get_clock().now()
        self._prev_btns = {}
        self._lt_prev = 1.0; self._rt_prev = 1.0
        self._last_l = (0,0); self._last_r = (0,0)
        self._neutral_repeats = 0
        self._last_write = 0.0
        self._joy_n = 0

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Joy,  "/joy",            self._joy_cb,   be)
        self.create_subscription(Bool, "/emergency_stop", self._estop_cb, rel)
        self._stat_pub = self.create_publisher(String, "/joy_arduino_status", rel)
        self.create_timer(0.1, self._watchdog)
        self.create_timer(5.0, self._diag)

        self.get_logger().info("=" * 52)
        self.get_logger().info(f"  joy_to_arduino  port={port}")
        self.get_logger().info("  L-stick Y=left  R-stick Y=right  (tank)")
        self.get_logger().info("  A=DIG2 Y=DRIVE B=DIG1 X=CAL")
        self.get_logger().info("  LT/RT=actuator hold  LB/RB=servo hold")
        self.get_logger().info("  (D-pad unused; known unreliable on this controller)")
        self.get_logger().info("  Start=estop")
        self.get_logger().info("=" * 52)

    def _connect(self, port):
        try:
            self._ser = serial.Serial(port, 115200, timeout=1.0)
            time.sleep(2.0)
            self._ser.reset_input_buffer()
            self.get_logger().info(f"Arduino: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial failed: {e}")
            self._ser = None

    def _send(self, device, speed=0, direction=0, lobyte=0):
        p = pkt(device, speed, direction, lobyte)
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(p)
            except serial.SerialException as e:
                self.get_logger().error(f"Write: {e}")
                self._ser = None

    def _stop_all(self):
        with self._lock:
            self._send(DEV_KILL)
        self._last_l = (0,0); self._last_r = (0,0)
        self._neutral_repeats = 0

    def _sd(self, v, flip=False):
        sp = min(int(abs(v) * MAX_MOTOR), MAX_MOTOR)
        d  = 0 if v >= 0 else 1
        return (sp, 1-d if flip else d)

    def _drive(self, lf, rf):
        l = self._sd(lf); r = self._sd(rf, flip=RIGHT_FLIP)
        now = time.monotonic()
        if (l != self._last_l or r != self._last_r) and \
                (now - self._last_write) >= (1.0/20):
            with self._lock:
                self._send(DEV_LEFT,  l[0], l[1])
                self._send(DEV_RIGHT, r[0], r[1])
            self._last_l = l; self._last_r = r; self._last_write = now
            self._neutral_repeats = 0

    def _stop_drive(self):
        # Send repeated neutral packets to guard against occasional serial drops.
        if self._last_l != (0,0) or self._last_r != (0,0) or self._neutral_repeats < 2:
            with self._lock:
                self._send(DEV_LEFT, 0, 0)
                self._send(DEV_RIGHT, 0, 0)
            self._last_l = (0,0); self._last_r = (0,0)
            self._neutral_repeats += 1

    def _dz(self, v): return v if abs(v) >= DEADZONE else 0.0

    def _rising(self, idx, cur):
        prev = self._prev_btns.get(idx, 0)
        self._prev_btns[idx] = cur
        return cur == 1 and prev == 0

    def _joy_cb(self, msg: Joy):
        self._last_joy = self.get_clock().now()
        self._joy_n += 1

        ax  = lambda i: msg.axes[i]    if i < len(msg.axes)    else 0.0
        btn = lambda i: msg.buttons[i] if i < len(msg.buttons) else 0

        # Estop
        if self._rising(BTN_START, btn(BTN_START)):
            self._estop = not self._estop
            if self._estop:
                self._stop_all()
                self.get_logger().warn("ESTOP ON")
            else:
                self.get_logger().info("ESTOP off")

        if self._estop:
            return

        # Actuator manual (triggers, hold)
        lt = ax(AXIS_LT)
        rt = ax(AXIS_RT)
        lt_pressed = lt < TRIG_TH
        rt_pressed = rt < TRIG_TH
        if lt_pressed and not rt_pressed:
            with self._lock: self._send(DEV_ACT, 190, 1)
        elif rt_pressed and not lt_pressed:
            with self._lock: self._send(DEV_ACT, 190, 0)
        else:
            with self._lock: self._send(DEV_ACT, 0, 0)

        # Servo manual (360 servo on bumpers, hold)
        lb = btn(BTN_LB)
        rb = btn(BTN_RB)
        if lb and not rb:
            with self._lock: self._send(DEV_SERVO, SERVO_CCW)
        elif rb and not lb:
            with self._lock: self._send(DEV_SERVO, SERVO_CW)
        else:
            with self._lock: self._send(DEV_SERVO, SERVO_STOP)

    def _send_calibrate_dump(self):
        # Some firmware builds bind retract/evening behavior to CAL (0xCA),
        # others to DUMP (0xB3). Send both so calibration works reliably.
        with self._lock:
            self._send(CMD_CAL)
        time.sleep(0.03)
        with self._lock:
            self._send(CMD_DUMP)
        self.get_logger().warn("Act CAL/DUMP sequence sent (0xCA -> 0xB3)")

        # Actuator presets
        if self._rising(BTN_A, btn(BTN_A)):
            with self._lock: self._send(CMD_DIG2); self.get_logger().info("Act->DIG2")
        if self._rising(BTN_Y, btn(BTN_Y)):
            with self._lock: self._send(CMD_DRIVE); self.get_logger().info("Act->DRIVE")
        if self._rising(BTN_B, btn(BTN_B)):
            with self._lock: self._send(CMD_DIG); self.get_logger().info("Act->DIG1")
        if self._rising(BTN_X, btn(BTN_X)):
            self._send_calibrate_dump()

        # Tank drive
        l = self._dz(ax(AXIS_LEFT)); r = self._dz(ax(AXIS_RIGHT))
        if abs(l) < 0.001 and abs(r) < 0.001:
            self._stop_drive()
        else:
            self._drive(l * self._spd_l, r * self._spd_r)

    def _watchdog(self):
        if self._ser is None or not self._ser.is_open:
            ports = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
            if ports:
                self.get_logger().warn(f"Reconnecting {ports[0]}")
                self._connect(ports[0])
            return
        if self._estop: return
        if (self.get_clock().now() - self._last_joy).nanoseconds / 1e9 > JOY_TIMEOUT_S:
            self._stop_drive()

    def _diag(self):
        ok  = self._ser is not None and self._ser.is_open
        age = (self.get_clock().now() - self._last_joy).nanoseconds / 1e9
        msg = String()
        msg.data = json.dumps({
            "serial": ok, "spd_L": round(self._spd_l,2),
            "spd_R": round(self._spd_r,2), "estop": self._estop,
            "last_joy_s": round(age,1), "joy_msgs": self._joy_n,
        })
        self._stat_pub.publish(msg)
        if self._joy_n == 0:
            self.get_logger().warn("No /joy — is joy_node running?")
        self.get_logger().info(
            f"serial={ok} spd={self._spd_l:.2f}/{self._spd_r:.2f} "
            f"joy={self._joy_n}/5s age={age:.1f}s estop={self._estop}")
        self._joy_n = 0

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._estop = True; self._stop_all()
        else:
            self._estop = False

    def destroy_node(self):
        self._stop_all()
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    ports = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    if not ports:
        print("No Arduino found. Check USB.")
        rclpy.shutdown(); return
    node = JoyArduino(ports[0])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
