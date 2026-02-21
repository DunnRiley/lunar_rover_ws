#!/usr/bin/env python3
"""
ROS2 Motor Controller Node - Arduino Hardware Bridge
Protocol: [0xAA][Device][Speed][Direction][0x55]

Topics:
  /cmd_vel          geometry_msgs/Twist   Drive motors
  /actuator_cmd     std_msgs/Int8         Actuators  (+1=extend, -1=retract, 0=stop)
  /emergency_stop   std_msgs/Bool         Kill all
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Int8
import serial
import time

# ── Device IDs ───────────────────────────────────────────────────────────
FL    = 0x01   # individual wheel (legacy/debug only)
FR    = 0x02
BL    = 0x03
BR    = 0x04
LEFT  = 0x05   # FL + BL together  ← normal driving
RIGHT = 0x06   # FR + BR together  ← normal driving
AL    = 0xD4   # Left  actuator (individual override only)
AR    = 0xF7   # Right actuator (individual override only)
ACT   = 0x08   # Both actuators simultaneously — always use this
KILL  = 0xFF

START = 0xAA
END   = 0x55


class ArduinoMotorController(Node):

    def __init__(self):
        super().__init__('arduino_motor_controller')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('arduino_port',    '/dev/ttyACM0')
        self.declare_parameter('baudrate',         115200)
        self.declare_parameter('cmd_vel_timeout',  0.5)
        self.declare_parameter('deadzone_linear',  0.05)
        self.declare_parameter('deadzone_angular', 0.05)
        self.declare_parameter('max_motor_speed',  200)

        self.arduino_port    = self.get_parameter('arduino_port').value
        self.baudrate        = self.get_parameter('baudrate').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.deadzone_linear = self.get_parameter('deadzone_linear').value
        self.deadzone_ang    = self.get_parameter('deadzone_angular').value
        self.max_speed       = self.get_parameter('max_motor_speed').value

        # ── Serial connection ─────────────────────────────────────────────
        self.ser = None
        self._connect()

        # ── State ────────────────────────────────────────────────────────
        self.last_cmd_time    = self.get_clock().now()
        self.last_act_time    = self.get_clock().now()
        self.current_linear   = 0.0
        self.current_angular  = 0.0
        self.emergency        = False

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel',        self._cmd_vel_cb,  10)
        self.create_subscription(Bool,  '/emergency_stop', self._estop_cb,    10)
        self.create_subscription(Int8,  '/actuator_cmd',   self._actuator_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/motor_status', 10)

        # ── Timers ───────────────────────────────────────────────────────
        self.create_timer(0.1, self._watchdog)   # 10 Hz
        self.create_timer(1.0, self._status_cb)  #  1 Hz

        self.get_logger().info('='*50)
        self.get_logger().info('Arduino Motor Controller Ready')
        self.get_logger().info(f'Port: {self.arduino_port} @ {self.baudrate}')
        self.get_logger().info('Topics: /cmd_vel  /actuator_cmd  /emergency_stop')
        self.get_logger().info('='*50)

    # ── Serial helpers ────────────────────────────────────────────────────

    def _connect(self):
        try:
            self.ser = serial.Serial(
                self.arduino_port, self.baudrate, timeout=1.0)
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Connected to Arduino on {self.arduino_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {self.arduino_port}: {e}')
            self.ser = None

    @property
    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def _send(self, device: int, speed: int, direction: int):
        """Send [0xAA][device][speed][direction][0x55]"""
        if not self.is_connected:
            return
        try:
            self.ser.write(bytes([START, device,
                                  speed & 0xFF, direction & 0xFF, END]))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def _stop_drive(self):
        # 2 packets instead of 4 — both sides go to 0 simultaneously
        self._send(LEFT,  0, 0)
        self._send(RIGHT, 0, 0)

    def _stop_actuators(self):
        self._send(ACT, 0, 0)

    def _stop_all(self):
        self._send(KILL, 0, 0)

    # ── Drive helper ──────────────────────────────────────────────────────

    def _set_drive(self, left: float, right: float):
        """
        Drive both sides. left / right in range [-1.0, 1.0].
        Sends 2 serial packets per update (LEFT side, RIGHT side)
        instead of 4 — eliminates the sequential stutter.
        """
        def to_bytes(v):
            spd = min(int(abs(v) * self.max_speed), 255)
            direction = 0x00 if v >= 0 else 0x01
            return spd, direction

        ls, ld = to_bytes(left)
        rs, rd = to_bytes(right)
        # One packet drives FL+BL, one packet drives FR+BR
        self._send(LEFT,  ls, ld)
        self._send(RIGHT, rs, rd)

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        if self.emergency:
            return

        self.last_cmd_time = self.get_clock().now()
        lin = msg.linear.x
        ang = msg.angular.z

        if abs(lin) < self.deadzone_linear:  lin = 0.0
        if abs(ang) < self.deadzone_ang:     ang = 0.0

        self.current_linear  = lin
        self.current_angular = ang

        if abs(lin) < 0.01 and abs(ang) < 0.01:
            self._stop_drive()
            return

        # Differential drive mixing
        left  = lin - ang
        right = lin + ang
        max_val = max(abs(left), abs(right), 1.0)
        self._set_drive(left / max_val, right / max_val)

        self.get_logger().info(
            f'Drive  lin={lin:.2f}  ang={ang:.2f}',
            throttle_duration_sec=0.5)

    def _actuator_cb(self, msg: Int8):
        """
        +1 = extend   (direction 0x00)
        -1 = retract  (direction 0x01)
         0 = stop
        ALWAYS uses ACT (0x08) — drives both actuators in one packet.
        Never sends to AL/AR individually to prevent mechanical damage.
        """
        if self.emergency:
            return

        self.last_act_time = self.get_clock().now()
        val = msg.data
        spd = min(int(abs(val) * self.max_speed), 255) if val != 0 else 0

        if val > 0:
            self._send(ACT, spd, 0x00)
            self.get_logger().info('Actuators: EXTEND',
                                   throttle_duration_sec=0.5)
        elif val < 0:
            self._send(ACT, spd, 0x01)
            self.get_logger().info('Actuators: RETRACT',
                                   throttle_duration_sec=0.5)
        else:
            self._stop_actuators()

    def _estop_cb(self, msg: Bool):
        if msg.data and not self.emergency:
            self.emergency = True
            self._stop_all()
            self.get_logger().error('EMERGENCY STOP')
        elif not msg.data and self.emergency:
            self.emergency = False
            self.get_logger().info('Emergency stop cleared')

    def _watchdog(self):
        if self.emergency:
            return

        now = self.get_clock().now()

        # Stop drive if no cmd_vel recently
        drive_elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        if drive_elapsed > self.cmd_vel_timeout:
            if abs(self.current_linear) > 0.01 or abs(self.current_angular) > 0.01:
                self.get_logger().warn('Watchdog: stopping drive',
                                       throttle_duration_sec=2.0)
            self._stop_drive()
            self.current_linear  = 0.0
            self.current_angular = 0.0

        # Stop actuators if no actuator_cmd recently
        act_elapsed = (now - self.last_act_time).nanoseconds / 1e9
        if act_elapsed > self.cmd_vel_timeout:
            self._stop_actuators()

    def _status_cb(self):
        msg = String()
        if self.emergency:
            msg.data = 'EMERGENCY_STOP'
        elif not self.is_connected:
            msg.data = 'DISCONNECTED'
        elif abs(self.current_linear) > 0.01 or abs(self.current_angular) > 0.01:
            msg.data = 'MOVING'
        else:
            msg.data = 'IDLE'
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down — stopping all motors')
        self._stop_all()
        if self.is_connected:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArduinoMotorController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()