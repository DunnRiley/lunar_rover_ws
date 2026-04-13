#!/usr/bin/env python3
"""
nav_mission_sequencer.py  —  MINI PC
=====================================

IMPORTANT — HOW THIS WORKS
  This sequencer publishes to:
    /nav/arduino_turn_cmd   (Float32MultiArray)  → bridge → C8+C9+start (0xDD default)
    /nav/arduino_cmd        (Float32MultiArray)  → bridge → raw packet
  Distance commands are sent as raw 0xDC so mission speed is honored.

  nav_arduino_bridge.py MUST be running — it is the only process that owns
  the Arduino serial port.  Start it before starting missions:
    python3 ~/lunar_rover_ws/nav_arduino_bridge.py &

  This design eliminates serial-port conflicts between joy_to_arduino, the
  bridge, and the sequencer.

WHY NOT DIRECT SERIAL?
  joy_to_arduino.py already has the Arduino serial port open during teleop.
  If the sequencer also tries to open the port it silently fails or corrupts
  packets.  Publishing ROS topics lets the bridge (which always owns serial)
  forward commands safely.

COMPLETION DETECTION
  The Arduino never sends "DIST_DONE".  We use two stages:
    1. Time estimate: (distance / speed_m_s) * TIME_BUFFER
       Set SPEED_ESTIMATE_M_S conservatively LOW (0.20).
    2. Flat settle: SETTLE_S extra seconds for the rover to actually stop.
  timeout_s in YAML steps is a hard cap — set it generously (60–120 s).

TUNING
  After deploying, run one 1-metre drive and time it.
  Set SPEED_ESTIMATE_M_S = 1.0 / measured_seconds (e.g. if 5 s → 0.20).
  TRACK_WIDTH_MM: measure wheel-centre to wheel-centre in mm.
"""

import math, os, json, threading, time, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String, Float32MultiArray

try:
    import yaml
except ImportError:
    print("ERROR: pip3 install pyyaml --break-system-packages"); sys.exit(1)

# ─── Tuning ──────────────────────────────────────────────────────────────────
TRACK_WIDTH_MM     = 700.0   # measure on your robot (mm, wheel-centre to wheel-centre)
SPEED_ESTIMATE_M_S = 0.20    # conservative speed at PWM=120 — tune after a test run
TIME_BUFFER        = 1.3     # multiply time estimate by this (30% safety margin)
SETTLE_S           = 2.0     # extra seconds after time estimate before declaring done
MAX_SPEED          = 190     # Arduino firmware cap for encoder commands
FLIP_TURN_DIR      = False   # set True if CW/CCW are backwards on your robot
ACT_TIMEOUT_S      = 20.0    # how long to wait for an actuator preset to complete
TURN_START_CMD     = 0xDD    # new firmware turn start command
# ─────────────────────────────────────────────────────────────────────────────


class Sequencer(Node):

    def __init__(self):
        super().__init__("nav_mission_sequencer")
        self.declare_parameter("mission_file", "")
        self._mfile = self.get_parameter("mission_file").value

        self._lock    = threading.Lock()
        self._running = False
        self._abort   = False
        self._steps   = []
        self._step_i  = -1
        self._name    = ""

        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Bool,   "/mission/start", self._cb_start, rel)
        self.create_subscription(String, "/mission/file",  self._cb_file,  rel)

        # ── Status publishers ─────────────────────────────────────────────────
        self._spub = self.create_publisher(String, "/mission/status", rel)
        self._apub = self.create_publisher(Bool,   "/mission/active", rel)

        # ── Command publishers (same topics as the GUI buttons) ───────────────
        self._turn_pub = self.create_publisher(
            Float32MultiArray,"/nav/arduino_turn_cmd",   rel)
        self._cmd_pub  = self.create_publisher(
            Float32MultiArray,"/nav/arduino_cmd",        rel)

        self.create_timer(0.5, self._pub_status)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  nav_mission_sequencer  ready")
        self.get_logger().info(f"  TRACK_WIDTH       = {TRACK_WIDTH_MM:.0f} mm")
        self.get_logger().info(f"  SPEED_ESTIMATE    = {SPEED_ESTIMATE_M_S:.2f} m/s at PWM=120")
        self.get_logger().info(f"  TIME_BUFFER       = {TIME_BUFFER}×")
        self.get_logger().info(f"  SETTLE_S          = {SETTLE_S} s")
        self.get_logger().info(f"  FLIP_TURN_DIR     = {FLIP_TURN_DIR}")
        self.get_logger().info("  Publishes to /nav/arduino_turn_cmd and /nav/arduino_cmd")
        self.get_logger().info("  (uses raw 0xDC for distance + 0xDD turn-start by default)")
        self.get_logger().info("  nav_arduino_bridge.py must be running!")
        self.get_logger().info("  Waiting for /mission/start True ...")
        self.get_logger().info("=" * 60)

    # ── ROS helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _encode_dist_mm(mm: float, reverse: bool):
        units = int(min(0x7FFF, max(0, round(abs(mm)))))
        combined = ((1 if reverse else 0) << 15) | units
        return (combined >> 8) & 0xFF, combined & 0xFF

    def _send_dist(self, metres: float, speed: int):
        """
        Send encoder-based straight command directly as raw 0xDC.
        This preserves mission speed setting (bridge /nav/arduino_dist_cmd is fixed at PWM=120).
        """
        db, lo = self._encode_dist_mm(abs(metres) * 1000.0, metres < 0)
        self._send_raw(0xDC, speed, db, lo)

    def _send_turn(self, arc_mm: float, speed: int, clockwise: bool, start_cmd: int = TURN_START_CMD):
        """Same as the GUI pivot preset buttons."""
        m = Float32MultiArray()
        m.data = [float(arc_mm), float(speed), float(1 if clockwise else 0), float(start_cmd)]
        self._turn_pub.publish(m)

    def _send_raw(self, device: int, speed: int = 0, direction: int = 0, lobyte: int = 0):
        """Same as the GUI actuator preset buttons."""
        m = Float32MultiArray()
        m.data = [float(device), float(speed), float(direction), float(lobyte)]
        self._cmd_pub.publish(m)

    def _send_stop(self):
        self._send_raw(0xFF)

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _cb_file(self, msg: String):
        if not self._running:
            path = os.path.expanduser(msg.data.strip())
            self._mfile = path
            self.get_logger().info(f"Mission file: {self._mfile}")
        else:
            self.get_logger().warn("Ignoring /mission/file — mission already running")

    def _cb_start(self, msg: Bool):
        if msg.data:
            if not self._running:
                threading.Thread(target=self._run, daemon=True).start()
            else:
                self.get_logger().warn("Already running — send False to abort first")
        else:
            with self._lock:
                self._abort = True
            self.get_logger().warn("ABORT requested")

    def _pub_status(self):
        with self._lock:
            i   = self._step_i
            tot = len(self._steps)
            run = self._running
        name = self._steps[i].get("action", "") if 0 <= i < tot else ""
        m = String()
        m.data = json.dumps({
            "running": run, "step": i, "total": tot,
            "step_name": name, "mission": self._name,
        })
        self._spub.publish(m)
        a = Bool(); a.data = run
        self._apub.publish(a)

    # ── Mission runner ────────────────────────────────────────────────────────

    def _run(self):
        if not self._mfile:
            self.get_logger().error("No mission file — send /mission/file first")
            return
        path = os.path.expanduser(self._mfile)
        if not os.path.exists(path):
            self.get_logger().error(f"File not found: {path}")
            return

        try:
            with open(path) as f:
                doc = yaml.safe_load(f)
            mission = doc.get("mission", doc)
            steps   = mission.get("steps", [])
            name    = mission.get("name", os.path.basename(path))
        except Exception as e:
            self.get_logger().error(f"YAML error: {e}")
            return

        if not steps:
            self.get_logger().error("No steps in YAML — nothing to run")
            return

        with self._lock:
            self._running = True
            self._abort   = False
            self._steps   = steps
            self._name    = name
            self._step_i  = 0

        self.get_logger().info("")
        self.get_logger().info(f"══ START '{name}'  ({len(steps)} steps) ══")
        self.get_logger().info("")

        for i, step in enumerate(steps):
            with self._lock:
                if self._abort:
                    self.get_logger().warn(f"  Aborted before step {i+1}")
                    break
                self._step_i = i

            action = step.get("action", "")
            params = step.get("params", {})
            self.get_logger().info(f"── Step {i+1}/{len(steps)}: {action}  params={params}")

            fn = getattr(self, f"_do_{action}", None)
            if fn is None:
                self.get_logger().error(
                    f"   Unknown action '{action}' — SKIPPING\n"
                    f"   Valid actions: drive_forward, drive_backward, pivot_turn, "
                    f"arc_turn, actuator_position, wait, stop")
                continue

            try:
                ok = fn(params)
            except Exception as e:
                self.get_logger().error(f"   Exception: {e}")
                ok = False

            if not ok:
                self.get_logger().error(f"   Step {i+1} FAILED — aborting mission")
                break
            self.get_logger().info(f"   Step {i+1} ✓")

        self._send_stop()
        with self._lock:
            self._running = False
            self._step_i  = -1
        self.get_logger().info("")
        self.get_logger().info("══ MISSION COMPLETE ══")
        self.get_logger().info("")

    # ── Sleep helpers ─────────────────────────────────────────────────────────

    def _aborted(self) -> bool:
        with self._lock: return self._abort

    def _sleep(self, secs: float) -> bool:
        """Sleep for `secs`, checking abort every 50 ms. Returns False if aborted."""
        end = time.monotonic() + secs
        while time.monotonic() < end:
            if self._aborted(): return False
            time.sleep(0.05)
        return True

    def _wait_drive(self, dist_m: float, speed: int, timeout: float) -> bool:
        """
        Wait for a drive command to complete.
        Stage 1: conservative time estimate (under-estimates on purpose)
        Stage 2: flat SETTLE_S buffer for the rover to physically stop
        """
        speed_ms = max(SPEED_ESTIMATE_M_S * (speed / 120.0), 0.05)
        est_s    = (abs(dist_m) / speed_ms) * TIME_BUFFER
        stage1_s = max(1.0, min(est_s, timeout - SETTLE_S - 0.5))

        self.get_logger().info(
            f"   Waiting: stage1={stage1_s:.1f}s (est={est_s:.1f}s) "
            f"+ settle={SETTLE_S}s  timeout cap={timeout}s")

        if not self._sleep(stage1_s):
            self._send_stop(); return False
        return self._sleep(SETTLE_S)   # flat settle

    # ── Actions ───────────────────────────────────────────────────────────────

    def _do_drive_forward(self, p: dict) -> bool:
        dist_m  = abs(float(p.get("distance_m", 1.0)))
        speed   = min(int(p.get("speed", 120)), MAX_SPEED)
        timeout = float(p.get("timeout_s", 120))

        if dist_m > 50.0:
            self.get_logger().error(
                f"   distance_m={dist_m} looks wrong (>50 m). "
                f"YAML uses METRES not mm. Did you mean {dist_m/1000:.3f} m?")

        self.get_logger().info(
            f"   Drive forward {dist_m:.3f} m  speed={speed}")
        self._send_dist(dist_m, speed)
        return self._wait_drive(dist_m, speed, timeout)

    def _do_drive_backward(self, p: dict) -> bool:
        dist_m  = abs(float(p.get("distance_m", 0.5)))
        speed   = min(int(p.get("speed", 120)), MAX_SPEED)
        timeout = float(p.get("timeout_s", 120))

        self.get_logger().info(
            f"   Drive backward {dist_m:.3f} m  speed={speed}")
        self._send_dist(-dist_m, speed)   # negative = reverse
        return self._wait_drive(dist_m, speed, timeout)

    def _do_pivot_turn(self, p: dict) -> bool:
        return self._turn(p)

    def _do_arc_turn(self, p: dict) -> bool:
        return self._turn(p)

    def _turn(self, p: dict) -> bool:
        degrees = float(p.get("degrees", 90.0))
        speed   = min(int(p.get("speed", 100)), MAX_SPEED)
        timeout = float(p.get("timeout_s", 60))

        if self._aborted(): return False

        arc_mm   = (TRACK_WIDTH_MM / 2.0) * abs(math.radians(degrees))
        clockwise = (degrees < 0) ^ FLIP_TURN_DIR   # negative degrees = CW

        self.get_logger().info(
            f"   Pivot {degrees:+.1f}°  arc={arc_mm:.0f} mm  "
            f"{'CW' if clockwise else 'CCW'}  speed={speed}")
        self._send_turn(arc_mm, speed, clockwise)

        speed_ms = max(SPEED_ESTIMATE_M_S * (speed / 120.0), 0.05)
        est_s    = ((arc_mm / 1000.0) / speed_ms) * TIME_BUFFER
        stage1_s = max(1.0, min(est_s, timeout - SETTLE_S - 0.5))

        self.get_logger().info(
            f"   Waiting: stage1={stage1_s:.1f}s + settle={SETTLE_S}s")

        if not self._sleep(stage1_s):
            self._send_stop(); return False
        return self._sleep(SETTLE_S)

    def _do_actuator_position(self, p: dict) -> bool:
        target  = str(p.get("target", "drive")).lower().strip()
        timeout = float(p.get("timeout_s", ACT_TIMEOUT_S))

        if self._aborted(): return False

        dev = {"dig": 0xA7, "drive": 0xA9, "dump": 0xB3}.get(target)
        if dev is None:
            self.get_logger().error(
                f"   Unknown actuator target '{target}'  (use: dig, drive, dump)")
            return False

        self.get_logger().info(
            f"   Actuator → {target}  (cmd=0x{dev:02X})  waiting {timeout}s")
        self._send_raw(dev)

        # Wait the timeout — the Arduino firmware drives to position automatically
        if not self._sleep(timeout):
            return False
        self.get_logger().info(f"   Actuator wait complete")
        return True

    def _do_wait(self, p: dict) -> bool:
        secs = float(p.get("seconds", 1.0))
        self.get_logger().info(f"   Wait {secs:.1f} s")
        return self._sleep(secs)

    def _do_stop(self, p: dict) -> bool:
        self._send_stop()
        self.get_logger().info("   STOP ALL sent")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = Sequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try: node._send_stop()
        except: pass
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass


if __name__ == "__main__":
    main()
