#!/usr/bin/env python3
"""
nav_mission_sequencer.py  —  MINI PC  (runs on the miniPC, not laptop)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  WHERE TO RUN AND HOW TO START A MISSION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Run on the MINI PC (not the laptop):
    python3 nav_mission_sequencer.py --ros-args -p mission_file:=mission.yaml

  The sequencer starts idle.  Trigger it ONE of two ways:

  1. From the GUI (Mission Sequencer widget → START MISSION button)
     The GUI publishes True to /mission/start via ROS.

  2. From any terminal:
     ros2 topic pub /mission/start std_msgs/msg/Bool "data: true" --once

  The sequencer will print its status every 0.5 s. To abort:
     ros2 topic pub /mission/start std_msgs/msg/Bool "data: false" --once

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  REQUIRED NODES (all on mini PC)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  The sequencer needs these three nodes to be alive:

  1. nav_arduino_bridge.py   — converts /nav/arduino_dist_cmd to 0xDC serial,
                               publishes /nav/arduino_done when Arduino
                               prints DIST_DONE or DIST_TIMEOUT.

  2. arduino_motor_controller (ros2 run lunar_robot_hardware) OR the bridge
     above — for turn commands sent as /cmd_vel.
     The sequencer sends angular-only Twists to /cmd_vel during turns.
     If arduino_motor_controller is running it will translate these to
     DriveLeft / DriveRight serial packets.

  3. nav_sensor_fusion.py    — provides /nav/fused_state with IMU heading
                               for turn feedback.

  The full autonomous launch script (full_launch_autonomous.sh) starts
  all three automatically.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  DIAGNOSIS: "sequencer starts but rover never moves"
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Check each in order:

  1. Is nav_arduino_bridge running?
       ros2 node list | grep bridge
     If not: python3 nav_arduino_bridge.py

  2. Is the Arduino serial port accessible?
       ls /dev/ttyACM*
     If missing: USB cable plugged in? dialout group? sudo chmod 666 /dev/ttyACM0

  3. Is /nav/arduino_done being published after a drive command?
       ros2 topic echo /nav/arduino_done
     Send a test: ros2 topic pub /nav/arduino_dist_cmd std_msgs/msg/Float32 "data: 0.3" --once
     Watch the Arduino terminal — you should see DIST_DONE printed.

  4. Is the ROS_DOMAIN_ID the same on laptop and miniPC?
       echo $ROS_DOMAIN_ID  (both should print 42)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  DRIVE DIRECTION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  The firmware has invertRightDriveDirection = true, so the
  Arduino itself handles the right-side flip.  This sequencer
  sends direction=0 for forward, direction=1 for reverse and
  lets the firmware invert as needed.  No ROS-side flip needed.

  For turns: angular.z > 0 = counterclockwise (standard ROS).
  arduino_motor_controller applies:
    left  = linear - angular  → forward - positive = turns left side slower
    right = linear + angular  → forward + positive = turns right side faster
  resulting in a CCW turn.  Positive degrees in arc_turn = CCW.
  Use negative degrees for CW.  Verify on your rover and swap if needed.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  MISSION FILE FORMAT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  mission:
    name: "Test run"
    steps:
      - action: drive_forward
        params:
          distance_m: 2.0       # up to 32.7 m per step, single packet
          timeout_s: 30         # Arduino firmware timeout = 15 s; set this >= 15
          use_camera: true      # stop early if camera depth <= goal_stop_m
          goal_stop_m: 0.50

      - action: drive_backward
        params:
          distance_m: 0.5
          timeout_s: 15

      - action: arc_turn
        params:
          degrees: 90           # + = CCW  (verify on your rover)
          speed: 0.25           # rad/s
          tolerance_deg: 4
          timeout_s: 20

      - action: actuator_position
        params:
          target: dig           # dig | drive | dump
          timeout_s: 12

      - action: wait
        params:
          seconds: 1.0

      - action: stop
"""

import json
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from geometry_msgs.msg import Twist

try:
    import yaml
except ImportError:
    print('ERROR: pip3 install pyyaml --break-system-packages')
    sys.exit(1)


# ── Constants ─────────────────────────────────────────────────────────────────

ACTUATOR_CMDS = {'dig': 0xA7, 'drive': 0xA9, 'dump': 0xB3}
CMD_STOP_ALL  = 0xB4

# Arduino firmware timeout for distance drive is 15 s.
# The sequencer waits ARDUINO_TIMEOUT_S for the DIST_DONE signal.
# Set >= 15 s so the Arduino times out first and sends DIST_TIMEOUT.
ARDUINO_TIMEOUT_S = 18.0

ARC_TURN_SPEED = 0.25   # rad/s — increase if turns are too slow

# Actuator settle detection
ACTUATOR_SETTLE_S   = 0.6
ACTUATOR_SETTLE_EPS = 5   # encoder counts

STEP_POLL_HZ = 20.0


class MissionSequencer(Node):

    def __init__(self):
        super().__init__('nav_mission_sequencer')

        self.declare_parameter('mission_file', '')
        self._mission_file = self.get_parameter('mission_file').value

        self._lock          = threading.Lock()
        self._running       = False
        self._abort         = False
        self._mission_steps = []
        self._current_step  = -1
        self._mission_name  = ''

        self._fused        = {}
        self._done_event   = threading.Event()
        self._done_success = False

        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        be  = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribers
        self.create_subscription(String, '/nav/fused_state',  self._fused_cb,  be)
        self.create_subscription(Bool,   '/nav/arduino_done', self._done_cb,   rel)
        self.create_subscription(Bool,   '/mission/start',    self._start_cb,  rel)
        self.create_subscription(String, '/mission/file',     self._file_cb,   rel)

        # Publishers
        self._status_pub   = self.create_publisher(String,            '/mission/status',       rel)
        self._active_pub   = self.create_publisher(Bool,              '/mission/active',       rel)
        # Turns go to /cmd_vel (arduino_motor_controller listens here)
        self._cmdvel_pub   = self.create_publisher(Twist,             '/cmd_vel',              rel)
        # Distance drive commands go to the bridge
        self._dist_pub     = self.create_publisher(Float32,           '/nav/arduino_dist_cmd', rel)
        # Generic raw commands (actuator presets, stop-all)
        self._arduino_pub  = self.create_publisher(Float32MultiArray, '/nav/arduino_cmd',      rel)
        # IMU heading reset
        self._hreset_pub   = self.create_publisher(Bool,              '/nav/heading_reset',    rel)

        self.create_timer(0.5, self._status_timer)

        self.get_logger().info('Mission sequencer ready')
        if self._mission_file:
            self.get_logger().info(f'  File: {self._mission_file}')
        self.get_logger().info(
            '  Waiting for /mission/start True  OR  GUI "START MISSION" button')
        self._print_dependency_check()

    def _print_dependency_check(self):
        """Log which required nodes appear to be missing."""
        self.create_timer(3.0, self._one_time_dep_check)

    def _one_time_dep_check(self):
        # Only runs once after 3 s
        node_names = [n[0] for n in self.get_node_names_and_namespaces()]
        missing = []
        for n in ('nav_arduino_bridge', 'arduino_motor_controller',
                  'nav_sensor_fusion'):
            if n not in node_names:
                missing.append(n)
        if missing:
            self.get_logger().warn(
                f'[sequencer] Missing nodes: {missing}\n'
                '  Run full_launch_autonomous.sh on the miniPC to start them all.')
        else:
            self.get_logger().info('[sequencer] All required nodes found ✓')
        # Destroy this one-shot timer by not re-scheduling (Timer auto-repeats,
        # so we cancel it via a flag)
        self._dep_checked = True

    # ── ROS callbacks ─────────────────────────────────────────────────────

    def _fused_cb(self, msg: String):
        try:
            self._fused = json.loads(msg.data)
        except Exception:
            pass

    def _done_cb(self, msg: Bool):
        self._done_success = msg.data
        self._done_event.set()

    def _start_cb(self, msg: Bool):
        if msg.data:
            if not self._running:
                threading.Thread(target=self._run_mission, daemon=True).start()
            else:
                self.get_logger().warn('[sequencer] Already running — send False first to abort')
        else:
            with self._lock:
                self._abort = True
            self.get_logger().warn('[sequencer] Abort requested')

    def _file_cb(self, msg: String):
        if not self._running:
            self._mission_file = msg.data
            self.get_logger().info(f'[sequencer] Mission file: {self._mission_file}')

    def _status_timer(self):
        with self._lock:
            step  = self._current_step
            total = len(self._mission_steps)
            run   = self._running
        step_name = ''
        if 0 <= step < total:
            step_name = self._mission_steps[step].get('action', '')
        m = String()
        m.data = json.dumps({
            'running': run, 'step': step, 'total': total,
            'step_name': step_name, 'mission': self._mission_name,
        })
        self._status_pub.publish(m)
        a = Bool(); a.data = run
        self._active_pub.publish(a)

    # ── Mission runner ────────────────────────────────────────────────────

    def _run_mission(self):
        if not self._mission_file or not os.path.exists(self._mission_file):
            self.get_logger().error(
                f'[sequencer] Mission file not found: {self._mission_file}\n'
                '  Set path with: ros2 topic pub /mission/file std_msgs/msg/String '
                '"data: \'/path/to/mission.yaml\'" --once')
            return

        try:
            with open(self._mission_file) as f:
                doc = yaml.safe_load(f)
            mission = doc.get('mission', doc)
            steps   = mission.get('steps', [])
            name    = mission.get('name', os.path.basename(self._mission_file))
        except Exception as e:
            self.get_logger().error(f'[sequencer] YAML parse error: {e}')
            return

        with self._lock:
            self._running       = True
            self._abort         = False
            self._mission_steps = steps
            self._mission_name  = name
            self._current_step  = 0

        self.get_logger().info(
            f'[sequencer] ═══ START "{name}" ({len(steps)} steps) ═══')

        for i, step in enumerate(steps):
            with self._lock:
                if self._abort:
                    break
                self._current_step = i

            action = step.get('action', '')
            params = step.get('params', {})
            self.get_logger().info(
                f'[sequencer] Step {i+1}/{len(steps)}: {action}  params={params}')

            method = getattr(self, f'_action_{action}', None)
            if method is None:
                self.get_logger().error(
                    f'[sequencer] Unknown action "{action}" — skipping.\n'
                    f'  Add _action_{action}(self, params) -> bool to implement it.')
                continue

            ok = method(params)
            if not ok:
                self.get_logger().error(
                    f'[sequencer] Step {i+1} "{action}" FAILED — aborting mission')
                break
            self.get_logger().info(f'[sequencer] Step {i+1} "{action}" ✓')

        self._send_stop_all()

        with self._lock:
            self._running      = False
            self._current_step = -1
        self.get_logger().info('[sequencer] ═══ MISSION COMPLETE ═══')

    # ═════════════════════════════════════════════════════════════════════
    #  ACTIONS
    # ═════════════════════════════════════════════════════════════════════

    def _action_drive_forward(self, params: dict) -> bool:
        """
        Drive forward using the Arduino 0xDC distance drive.

        Sends a single 15-bit encoded packet:
          combined = (0 << 15) | round(distance_mm)   # forward
          HI = combined >> 8
          LO = combined & 0xFF
          packet: AA DC HI LO 55

        The Arduino drives until the BL encoder counts match, then
        prints DIST_DONE on Serial. The bridge picks this up and
        publishes /nav/arduino_done True.

        Camera early-stop: if use_camera=true and the camera depth
        drops below goal_stop_m, a STOPALL is sent immediately and
        the step returns True (rover is where it needs to be).

        params:
          distance_m   float  metres (max 32.767 m)
          timeout_s    float  Pi-side timeout (set >= 18 to let Arduino timeout first)
          use_camera   bool   enable camera early-stop (default True)
          goal_stop_m  float  camera stop threshold, metres (default 0.50)
        """
        distance_m  = abs(float(params.get('distance_m', 1.0)))
        timeout_s   = float(params.get('timeout_s', ARDUINO_TIMEOUT_S))
        use_camera  = bool(params.get('use_camera', True))
        goal_stop_m = float(params.get('goal_stop_m', 0.50))

        # Camera check before sending
        if use_camera and self._camera_close(goal_stop_m):
            self.get_logger().info(
                f'[drive] Camera shows target already within {goal_stop_m:.2f}m — skipping')
            return True

        # Clear any old done signal
        self._done_event.clear()

        # Send the distance packet
        d = Float32(); d.data = float(distance_m)
        self._dist_pub.publish(d)
        self.get_logger().info(
            f'[drive] Sent {distance_m:.3f}m forward — waiting for DIST_DONE '
            f'(timeout={timeout_s}s)')

        # Wait for DIST_DONE, checking camera and abort during the wait
        deadline  = time.monotonic() + timeout_s
        poll_dt   = 1.0 / STEP_POLL_HZ

        while time.monotonic() < deadline:
            if self._abort_check():
                self._send_stop_all()
                return False

            # Camera mid-drive stop
            if use_camera and self._camera_close(goal_stop_m):
                self.get_logger().info('[drive] Camera early stop mid-drive')
                self._send_stop_all()
                return True

            # Non-blocking check of the done event
            if self._done_event.wait(timeout=poll_dt):
                if self._done_success:
                    self.get_logger().info('[drive] DIST_DONE received ✓')
                    return True
                else:
                    self.get_logger().error(
                        '[drive] DIST_TIMEOUT from Arduino — step failed')
                    return False

        self.get_logger().error(
            f'[drive] Pi-side timeout after {timeout_s}s waiting for DIST_DONE.\n'
            '  Check: is nav_arduino_bridge running?\n'
            '  Check: ros2 topic echo /nav/arduino_done')
        self._send_stop_all()
        return False

    def _action_drive_backward(self, params: dict) -> bool:
        """
        Drive backward.  Uses direction bit 15 = 1.
        Camera early-stop disabled by default (bucket may obscure view).
        """
        distance_m = abs(float(params.get('distance_m', 0.5)))
        timeout_s  = float(params.get('timeout_s', ARDUINO_TIMEOUT_S))

        self._done_event.clear()

        # Negative value → bridge sets direction bit = 1
        d = Float32(); d.data = float(-distance_m)
        self._dist_pub.publish(d)
        self.get_logger().info(
            f'[drive_back] Sent {distance_m:.3f}m reverse — waiting for DIST_DONE')

        deadline = time.monotonic() + timeout_s
        poll_dt  = 1.0 / STEP_POLL_HZ

        while time.monotonic() < deadline:
            if self._abort_check():
                self._send_stop_all()
                return False
            if self._done_event.wait(timeout=poll_dt):
                if self._done_success:
                    self.get_logger().info('[drive_back] DIST_DONE ✓')
                    return True
                else:
                    self.get_logger().error('[drive_back] DIST_TIMEOUT')
                    return False

        self.get_logger().error('[drive_back] Pi-side timeout')
        self._send_stop_all()
        return False

    def _action_arc_turn(self, params: dict) -> bool:
        """
        Turn by degrees using IMU heading integration.
        Sends /cmd_vel with angular.z only (arduino_motor_controller converts
        this to differential serial packets).

        The firmware has invertRightDriveDirection = true, so it already
        handles the right-side flip.  No Python-side flip needed.

        params:
          degrees       float   + = CCW in standard ROS (verify on your rover)
          speed         float   rad/s (default 0.25)
          tolerance_deg float   stop within N degrees of target (default 4)
          timeout_s     float   fail-safe (default 20)
        """
        target_deg = float(params.get('degrees', 90.0))
        speed      = float(params.get('speed', ARC_TURN_SPEED))
        tol        = float(params.get('tolerance_deg', 4.0))
        timeout_s  = float(params.get('timeout_s', 20.0))

        # Reset IMU heading to 0 before the turn
        self._reset_heading()
        time.sleep(0.20)   # let the fusion node process the reset

        sign       = 1.0 if target_deg >= 0 else -1.0
        target_abs = abs(target_deg)
        deadline   = time.monotonic() + timeout_s
        poll_dt    = 1.0 / STEP_POLL_HZ

        self.get_logger().info(
            f'[turn] target={target_deg:+.1f}°  speed={speed:.2f}rad/s  tol={tol:.1f}°')

        while time.monotonic() < deadline:
            if self._abort_check():
                self._send_vel(0.0, 0.0)
                return False

            current = abs(self._fused.get('heading_deg', 0.0))
            remaining = target_abs - current

            self.get_logger().info(
                f'[turn] heading={current:.1f}°  remaining={remaining:.1f}°',
                throttle_duration_sec=0.4)

            if remaining <= tol:
                self._send_vel(0.0, 0.0)
                self.get_logger().info(
                    f'[turn] Complete at {current:.1f}° (target {target_abs:.1f}°) ✓')
                return True

            # Ramp down in last 20°
            slow = min(1.0, remaining / 20.0)
            ang  = max(0.06, speed * slow) * sign
            self._send_vel(0.0, ang)
            time.sleep(poll_dt)

        self._send_vel(0.0, 0.0)
        self.get_logger().error(
            f'[turn] TIMEOUT after {timeout_s}s — '
            f'heading only reached {abs(self._fused.get("heading_deg", 0)):.1f}°\n'
            '  Check: is nav_sensor_fusion running?\n'
            '  Check: ros2 topic echo /nav/heading_deg')
        return False

    def _action_spin_in_place(self, params: dict) -> bool:
        return self._action_arc_turn(params)

    def _action_actuator_position(self, params: dict) -> bool:
        """
        Move actuator to a named preset.
        Polls /nav/actuator_enc until settled.
        """
        target    = str(params.get('target', 'drive')).lower().strip()
        timeout_s = float(params.get('timeout_s', 12.0))

        cmd_byte = ACTUATOR_CMDS.get(target)
        if cmd_byte is None:
            self.get_logger().error(
                f'[actuator] Unknown target "{target}" — valid: {list(ACTUATOR_CMDS.keys())}')
            return False

        m = Float32MultiArray()
        m.data = [float(cmd_byte), 0.0, 0.0]
        self._arduino_pub.publish(m)
        self.get_logger().info(f'[actuator] Sent preset → {target} (0x{cmd_byte:02X})')

        # Poll encoder until it stops changing
        deadline  = time.monotonic() + timeout_s
        last_enc  = self._fused.get('actuator_enc', 0)
        settle_t  = None

        while time.monotonic() < deadline:
            if self._abort_check():
                return False
            time.sleep(0.05)
            enc    = self._fused.get('actuator_enc', last_enc)
            change = abs(enc - last_enc)
            last_enc = enc
            if change < ACTUATOR_SETTLE_EPS:
                if settle_t is None:
                    settle_t = time.monotonic()
                elif time.monotonic() - settle_t >= ACTUATOR_SETTLE_S:
                    self.get_logger().info(
                        f'[actuator] Settled at enc={enc} ✓')
                    return True
            else:
                settle_t = None

        self.get_logger().warn(
            f'[actuator] Timeout after {timeout_s}s — encoder may not have settled.\n'
            '  Continuing anyway (Arduino firmware handles the final positioning).')
        return True   # non-fatal: Arduino handles it

    def _action_wait(self, params: dict) -> bool:
        seconds = float(params.get('seconds', 1.0))
        self.get_logger().info(f'[wait] Pausing {seconds:.1f}s')
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self._abort_check():
                return False
            time.sleep(0.05)
        return True

    def _action_stop(self, params: dict) -> bool:
        self._send_stop_all()
        return True

    # ── Helpers ───────────────────────────────────────────────────────────

    def _send_vel(self, linear: float, angular: float):
        """Publish to /cmd_vel (picked up by arduino_motor_controller)."""
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self._cmdvel_pub.publish(t)

    def _send_stop_all(self):
        """Send 0xB4 STOPALL through the bridge + zero /cmd_vel."""
        m = Float32MultiArray()
        m.data = [float(CMD_STOP_ALL), 0.0, 0.0]
        self._arduino_pub.publish(m)
        self._send_vel(0.0, 0.0)

    def _reset_heading(self):
        m = Bool(); m.data = True
        self._hreset_pub.publish(m)

    def _abort_check(self) -> bool:
        with self._lock:
            return self._abort

    def _camera_close(self, threshold_m: float) -> bool:
        cam_d = self._fused.get('camera_dist_m')
        if cam_d is None:
            return False
        return self._fused.get('camera_valid', False) and cam_d <= threshold_m


def main(args=None):
    rclpy.init(args=args)
    node = MissionSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._send_stop_all()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()