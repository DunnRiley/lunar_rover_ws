#!/usr/bin/env python3
"""
Gamepad Teleop for Lunar Rover (ROS2)

Requires joy_node running first:
  ros2 run joy joy_node

Controller Layout (Xbox / generic USB gamepad):
  Left  Stick Y     Forward / Backward
  Right Stick X     Pivot Left / Pivot Right
  RB  (button 5)    Speed UP   (+0.05)
  LB  (button 4)    Speed DOWN (-0.05)
  A   (button 0)    Actuators Extend  (hold)
  B   (button 1)    Actuators Retract (hold)
  Start (button 7)  Toggle Emergency Stop

If your controller axes/buttons differ run:
  ros2 topic echo /joy
and adjust the index constants below.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int8
from sensor_msgs.msg import Joy


# ── Controller index mapping ─────────────────────────────────────────────
AXIS_FWD    = 1   # Left  stick Y  (+1 = forward on most controllers)
AXIS_TURN   = 3   # Right stick X  (+1 = left)
BTN_A       = 0   # Actuators extend
BTN_B       = 1   # Actuators retract
BTN_LB      = 4   # Speed down
BTN_RB      = 5   # Speed up
BTN_START   = 7   # Emergency stop toggle

# ── Speed settings ───────────────────────────────────────────────────────
SPEED_MIN     = 0.10
SPEED_MAX     = 1.00
SPEED_STEP    = 0.05
SPEED_START   = 0.50
DEADZONE      = 0.10
ANGULAR_SCALE = 1.2

JOY_TIMEOUT   = 0.5   # seconds — stop if no joy message received


class ControllerTeleop(Node):

    def __init__(self):
        super().__init__('controller_teleop')

        # ── State ────────────────────────────────────────────────────────
        self.speed     = SPEED_START
        self.emergency = False
        self.last_joy  = self.get_clock().now()
        self._prev     = {BTN_LB: 0, BTN_RB: 0, BTN_START: 0}

        # ── Publishers ───────────────────────────────────────────────────
        self.cmd_pub   = self.create_publisher(Twist, '/cmd_vel',        10)
        self.estop_pub = self.create_publisher(Bool,  '/emergency_stop', 10)
        self.act_pub   = self.create_publisher(Int8,  '/actuator_cmd',   10)

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        # ── Watchdog 10 Hz ───────────────────────────────────────────────
        self.create_timer(0.1, self._watchdog)

        self._print_instructions()

    # ── Helpers ──────────────────────────────────────────────────────────

    def _btn(self, msg: Joy, i: int) -> int:
        return msg.buttons[i] if i < len(msg.buttons) else 0

    def _ax(self, msg: Joy, i: int) -> float:
        return msg.axes[i] if i < len(msg.axes) else 0.0

    def _dz(self, v: float) -> float:
        return v if abs(v) >= DEADZONE else 0.0

    def _rising(self, msg: Joy, idx: int) -> bool:
        cur  = self._btn(msg, idx)
        prev = self._prev.get(idx, 0)
        self._prev[idx] = cur
        return cur == 1 and prev == 0

    def _stop_drive(self):
        self.cmd_pub.publish(Twist())

    def _stop_actuators(self):
        m = Int8(); m.data = 0
        self.act_pub.publish(m)

    def _publish_actuator(self, val: int):
        m = Int8(); m.data = val
        self.act_pub.publish(m)

    # ── Joy callback ─────────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy):
        self.last_joy = self.get_clock().now()

        # ── Emergency stop (rising edge) ──────────────────────────────────
        if self._rising(msg, BTN_START):
            self.emergency = not self.emergency
            em = Bool(); em.data = self.emergency
            self.estop_pub.publish(em)
            self._stop_drive()
            self._stop_actuators()
            state = 'ACTIVATED' if self.emergency else 'cleared'
            self.get_logger().warn(f'Emergency stop {state}')

        if self.emergency:
            # Still consume edge detection for other buttons
            self._rising(msg, BTN_LB)
            self._rising(msg, BTN_RB)
            return

        # ── Speed control (rising edge) ───────────────────────────────────
        if self._rising(msg, BTN_RB):
            self.speed = round(min(SPEED_MAX, self.speed + SPEED_STEP), 2)
            self.get_logger().info(f'Speed: {self.speed:.2f}')

        if self._rising(msg, BTN_LB):
            self.speed = round(max(SPEED_MIN, self.speed - SPEED_STEP), 2)
            self.get_logger().info(f'Speed: {self.speed:.2f}')

        # ── Actuators (hold buttons — mutually exclusive) ─────────────────
        a_held = self._btn(msg, BTN_A)
        b_held = self._btn(msg, BTN_B)

        if a_held:
            self._publish_actuator(1)    # extend
            self.get_logger().info('Actuators: EXTEND',
                                   throttle_duration_sec=0.5)
        elif b_held:
            self._publish_actuator(-1)   # retract
            self.get_logger().info('Actuators: RETRACT',
                                   throttle_duration_sec=0.5)
        else:
            self._stop_actuators()

        # ── Drive ─────────────────────────────────────────────────────────
        fwd  = self._dz(self._ax(msg, AXIS_FWD))
        turn = self._dz(self._ax(msg, AXIS_TURN))

        twist = Twist()

        if fwd != 0.0 and turn == 0.0:
            twist.angular.z = turn * self.speed * ANGULAR_SCALE

        elif turn != 0.0 and fwd == 0.0:
            twist.angular.z = turn * self.speed * ANGULAR_SCALE
            twist.linear.x = fwd * self.speed

        elif fwd != 0.0 and turn != 0.0:
            # Both axes — gentle arc
            twist.linear.x  = fwd  * self.speed
            twist.angular.z = turn * self.speed * ANGULAR_SCALE

        self.cmd_pub.publish(twist)

    # ── Watchdog ──────────────────────────────────────────────────────────

    def _watchdog(self):
        if self.emergency:
            return
        elapsed = (self.get_clock().now() - self.last_joy).nanoseconds / 1e9
        if elapsed > JOY_TIMEOUT:
            self._stop_drive()
            self._stop_actuators()
            self.get_logger().warn('No joy message — motors stopped',
                                   throttle_duration_sec=2.0)

    # ── Cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop_drive()
        self._stop_actuators()
        em = Bool(); em.data = False
        self.estop_pub.publish(em)
        super().destroy_node()

    # ── Instructions ─────────────────────────────────────────────────────

    def _print_instructions(self):
        print('\n' + '='*55)
        print('  LUNAR ROVER  Controller Teleop')
        print('='*55)
        print('  Left  Stick Y    Forward / Backward')
        print('  Right Stick X    Pivot Left / Pivot Right')
        print('  RB  (btn 5)      Speed UP   (+0.05)')
        print('  LB  (btn 4)      Speed DOWN (-0.05)')
        print('  A   (btn 0)      Actuators Extend  (hold)')
        print('  B   (btn 1)      Actuators Retract (hold)')
        print('  Start(btn 7)     Toggle Emergency Stop')
        print('='*55)
        print(f'  Starting speed : {self.speed:.2f}')
        print(f'  Deadzone       : {DEADZONE}')
        print('='*55)
        print('\nMake sure joy_node is running:')
        print('  ros2 run joy joy_node\n')
        print('If buttons feel wrong run:')
        print('  ros2 topic echo /joy')
        print('and adjust the index constants at the top of the file.\n')


def main(args=None):
    rclpy.init(args=args)
    node = ControllerTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()