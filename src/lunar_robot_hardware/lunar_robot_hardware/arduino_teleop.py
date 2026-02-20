#!/usr/bin/env python3
"""
Keyboard Teleop for Lunar Rover (ROS2)
Uses curses for reliable hold-to-move behaviour.

Controls:
  W / S      Forward / Backward
  A / D      Pivot Left / Pivot Right
  O / L      Actuators Extend / Retract
  Z / C      Speed UP / DOWN  (step 0.05)
  SPACE      Toggle Emergency Stop
  Q          Quit
"""

import curses
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int8


SPEED_MIN     = 0.10
SPEED_MAX     = 1.00
SPEED_STEP    = 0.05
SPEED_START   = 0.50
ANGULAR_SCALE = 1.2


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        self.cmd_pub   = self.create_publisher(Twist, '/cmd_vel',        10)
        self.estop_pub = self.create_publisher(Bool,  '/emergency_stop', 10)
        self.act_pub   = self.create_publisher(Int8,  '/actuator_cmd',   10)

    def publish_drive(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self.cmd_pub.publish(t)

    def publish_actuator(self, val: int):
        """val: +1 extend, -1 retract, 0 stop"""
        m = Int8()
        m.data = val
        self.act_pub.publish(m)

    def publish_estop(self, state: bool):
        m = Bool()
        m.data = state
        self.estop_pub.publish(m)


def draw_ui(stdscr, speed: float, status: str, emergency: bool):
    stdscr.erase()
    _, w = stdscr.getmaxyx()

    title = 'LUNAR ROVER  Keyboard Teleop'
    stdscr.addstr(0, max(0, (w - len(title)) // 2), title, curses.A_BOLD)

    stdscr.addstr(2,  2, 'W / S      Forward / Backward')
    stdscr.addstr(3,  2, 'A / D      Pivot Left / Pivot Right')
    stdscr.addstr(4,  2, 'O / L      Actuators Extend / Retract')
    stdscr.addstr(5,  2, 'Z / C      Speed UP / DOWN  (step 0.05)')
    stdscr.addstr(6,  2, 'SPACE      Toggle Emergency Stop')
    stdscr.addstr(7,  2, 'Q          Quit')

    stdscr.addstr(9,  2, f'Speed : {speed:.2f}', curses.A_BOLD)

    if emergency:
        stdscr.addstr(10, 2, '*** EMERGENCY STOP ACTIVE ***',
                      curses.A_BOLD | curses.color_pair(1))
    else:
        stdscr.addstr(10, 2, f'Status: {status:<30}')

    stdscr.addstr(12, 2, 'Hold a key to move  release to stop', curses.A_DIM)
    stdscr.refresh()


def main_curses(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)   # non-blocking — returns -1 when no key held
    stdscr.timeout(50)     # wake every 50 ms to publish stop

    curses.start_color()
    curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)

    rclpy.init()
    node = KeyboardTeleop()

    speed     = SPEED_START
    emergency = False
    status    = 'Idle'

    draw_ui(stdscr, speed, status, emergency)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            key = stdscr.getch()

            # ── No key held → send stop for both drive and actuators ──────
            if key == -1:
                if not emergency:
                    node.publish_drive(0.0, 0.0)
                    node.publish_actuator(0)
                    status = 'Idle'
                draw_ui(stdscr, speed, status, emergency)
                continue

            ch = chr(key).lower() if 0 < key < 256 else ''

            # ── Quit ─────────────────────────────────────────────────────
            if ch == 'q':
                break

            # ── Emergency stop ────────────────────────────────────────────
            if ch == ' ':
                emergency = not emergency
                node.publish_estop(emergency)
                node.publish_drive(0.0, 0.0)
                node.publish_actuator(0)
                status = 'EMERGENCY STOP' if emergency else 'Idle'
                draw_ui(stdscr, speed, status, emergency)
                continue

            if emergency:
                draw_ui(stdscr, speed, status, emergency)
                continue

            # ── Speed control ─────────────────────────────────────────────
            if ch == 'z':
                speed = round(min(SPEED_MAX, speed + SPEED_STEP), 2)
                status = f'Speed up  {speed:.2f}'
                draw_ui(stdscr, speed, status, emergency)
                continue

            if ch == 'c':
                speed = round(max(SPEED_MIN, speed - SPEED_STEP), 2)
                status = f'Speed down  {speed:.2f}'
                draw_ui(stdscr, speed, status, emergency)
                continue

            # ── Drive keys ────────────────────────────────────────────────
            # Always stop actuators when driving
            node.publish_actuator(0)

            if ch == 'a':
                node.publish_drive(speed, 0.0)
                status = 'Pivot Left'

            elif ch == 'd':
                node.publish_drive(-speed, 0.0)
                tatus = 'Pivot Right'


            elif ch == 'w':
                node.publish_drive(0.0,  speed * ANGULAR_SCALE)
                status = 'Forward'


            elif ch == 's':
                node.publish_drive(0.0, -speed * ANGULAR_SCALE)
                status = 'Backward'

            # ── Actuator keys ─────────────────────────────────────────────
            # Stop drive when using actuators
            elif ch == 'o':
                node.publish_drive(0.0, 0.0)
                node.publish_actuator(1)    # +1 = extend
                status = 'Actuators Extend'

            elif ch == 'l':
                node.publish_drive(0.0, 0.0)
                node.publish_actuator(-1)   # -1 = retract
                status = 'Actuators Retract'

            draw_ui(stdscr, speed, status, emergency)

    finally:
        try:
            node.publish_drive(0.0, 0.0)
            node.publish_actuator(0)
            node.publish_estop(False)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


def main():
    curses.wrapper(main_curses)


if __name__ == '__main__':
    main()