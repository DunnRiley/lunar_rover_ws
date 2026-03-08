#!/usr/bin/env python3
"""
nav_drive_simple.py  —  MINI PC
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Receives goal distance (metres) on /nav/goal_dist.
Drives forward by writing directly to the Arduino serial port
(bypasses ROS entirely — no mux, no cmd_vel chain).

Distance remaining is measured from the IMU: integrates the
gyro/accel forward-velocity estimate so it only decreases
when the rover ACTUALLY moves.

IMU PROTOCOL  (Arduino → Pi, 9600 baud):
  Request:   0xAA 0xD1 0x55
  Response:  variable-length packets containing:
             gyro/accel as signed 16-bit little-endian integers
             scaled by /1000 → degrees or m/s²

Motor PROTOCOL  (Pi → Arduino):
  Same serial port.  Two signed bytes: [left_speed, right_speed]
  Range -100..+100.  0 = stop.
  Edit MOTOR_FWD_SPEED and SERIAL_PORT below to match your Arduino.
"""

import struct, threading, time, math, glob
import serial          # pip install pyserial
import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg           import Float32, Bool, String, ColorRGBA
from geometry_msgs.msg      import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray

try:
    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    HAS_TF = True
except ImportError:
    HAS_TF = False

# ── EDIT THESE ────────────────────────────────────────────────────────────────
SERIAL_PORT      = ''        # leave blank to auto-detect /dev/ttyACM*
BAUD_RATE        = 9600
MOTOR_FWD_SPEED  = 40        # -100..100 sent to Arduino for both wheels
GOAL_TOL_M       = 0.12      # stop when this close
IMU_REQUEST      = bytes([0xAA, 0xD1, 0x55])   # request IMU packet
IMU_POLL_HZ      = 20        # how often to poll IMU for velocity
# ── END EDIT ──────────────────────────────────────────────────────────────────


def _find_port():
    if SERIAL_PORT:
        return SERIAL_PORT
    candidates = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if candidates:
        print(f'[SERIAL] auto-detected: {candidates[0]}', flush=True)
        return candidates[0]
    return None


def _qos(n=10):
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=n)


class DriveSimple(Node):
    def __init__(self):
        super().__init__('nav_drive_simple')
        q = _qos()

        self.create_subscription(Float32, '/nav/goal_dist', self._cb_goal,   q)
        self.create_subscription(Bool,    '/nav/cancel',    self._cb_cancel, q)

        self._viz     = self.create_publisher(MarkerArray, '/nav/viz_markers',    q)
        self._stat    = self.create_publisher(String,      '/nav/status',         q)
        self._rem_pub = self.create_publisher(Float32,     '/nav/dist_remaining', q)
        # Still publish /nav/cmd_vel so RViz / mux can see intent
        self._cmd_ros = self.create_publisher(Twist,       '/nav/cmd_vel',        q)

        self._tf_br = TransformBroadcaster(self) if HAS_TF else None

        # State
        self._lock       = threading.Lock()
        self._goal_m     = 0.0
        self._remaining  = 0.0
        self._driven_m   = 0.0
        self._driving    = False

        # Serial
        self._ser        = None
        self._imu_vx     = 0.0    # forward velocity from IMU (m/s)
        self._imu_lock   = threading.Lock()

        # Open serial
        port = _find_port()
        if port:
            try:
                self._ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
                print(f'[SERIAL] opened {port} @ {BAUD_RATE}', flush=True)
            except Exception as e:
                print(f'[SERIAL] ERROR opening {port}: {e}', flush=True)
        else:
            print('[SERIAL] WARNING: no Arduino found — motor commands disabled', flush=True)

        # IMU reader thread
        self._imu_thread = threading.Thread(target=self._imu_loop, daemon=True)
        self._imu_thread.start()

        # Drive thread handle
        self._drive_thread = None

        self.create_timer(0.05, self._viz_cb)

        self._pub_status('IDLE')
        print('\n' + '='*55, flush=True)
        print('  nav_drive_simple  READY', flush=True)
        print(f'  Serial: {port or "NOT FOUND"}', flush=True)
        print(f'  Motor speed: {MOTOR_FWD_SPEED}  Goal tol: {GOAL_TOL_M}m', flush=True)
        print('  Waiting for goal on /nav/goal_dist', flush=True)
        print('='*55 + '\n', flush=True)

    # ── IMU reader loop ───────────────────────────────────────────────────

    def _imu_loop(self):
        """
        Polls IMU at IMU_POLL_HZ.  Parses the response for forward
        acceleration, integrates to velocity, decays when motors off.
        """
        dt = 1.0 / IMU_POLL_HZ
        vx = 0.0
        prev_t = time.monotonic()

        while True:
            time.sleep(dt)
            if self._ser is None:
                continue

            try:
                # Request fresh IMU data
                self._ser.write(IMU_REQUEST)
                time.sleep(0.01)
                raw = self._ser.read(self._ser.in_waiting or 16)
            except Exception as e:
                print(f'[IMU] read error: {e}', flush=True)
                continue

            now = time.monotonic()
            real_dt = now - prev_t
            prev_t  = now

            # Parse all 2-byte little-endian signed words from response
            # The first word we can decode is treated as forward accel (m/s²)
            # scaled by /1000 per the protocol.
            # Adjust indices below once you've confirmed the packet layout.
            ax = 0.0
            if len(raw) >= 2:
                try:
                    ax_raw = struct.unpack('<h', raw[0:2])[0]
                    ax     = ax_raw / 1000.0   # m/s²
                except struct.error:
                    pass

            # Simple integration — decay when near zero accel
            with self._imu_lock:
                driving = self._driving

            vx += ax * real_dt
            if not driving:
                vx *= 0.7   # decay to zero when motors stopped

            vx = max(-3.0, min(3.0, vx))  # clamp

            with self._imu_lock:
                self._imu_vx = vx

    # ── Goal / cancel ─────────────────────────────────────────────────────

    def _cb_goal(self, msg: Float32):
        dist = float(msg.data)
        if not (0.10 < dist < 8.0):
            print(f'[GOAL] rejected {dist:.2f}m', flush=True)
            return

        # Cancel existing drive
        with self._lock:
            self._driving = False
        if self._drive_thread and self._drive_thread.is_alive():
            self._drive_thread.join(timeout=0.5)
        self._motors_stop()

        with self._lock:
            self._goal_m    = dist
            self._remaining = dist
            self._driven_m  = 0.0
            self._driving   = True

        print(f'[GOAL] {dist:.2f}m', flush=True)
        self._pub_status('NAVIGATING')
        self._drive_thread = threading.Thread(
            target=self._drive_fn, args=(dist,), daemon=True)
        self._drive_thread.start()

    def _cb_cancel(self, msg: Bool):
        if msg.data:
            with self._lock:
                self._driving   = False
                self._remaining = 0.0
            self._motors_stop()
            print('[GOAL] cancelled', flush=True)
            self._pub_status('IDLE')

    # ── Drive thread ──────────────────────────────────────────────────────

    def _drive_fn(self, goal_m: float):
        """
        Turns motors on.  Integrates IMU velocity to measure distance.
        Stops when driven distance >= goal - GOAL_TOL.
        """
        driven   = 0.0
        prev_t   = time.monotonic()
        tick     = 1.0 / IMU_POLL_HZ

        self._motors_fwd()

        while True:
            time.sleep(tick)
            now = time.monotonic()
            dt  = now - prev_t
            prev_t = now

            with self._lock:
                if not self._driving:
                    self._motors_stop()
                    return

            # Measure actual movement from IMU
            with self._imu_lock:
                vx = self._imu_vx

            step   = max(0.0, vx) * dt
            driven += step
            remaining = max(0.0, goal_m - driven)

            with self._lock:
                self._driven_m  = driven
                self._remaining = remaining

            # Publish remaining for GUI
            r = Float32(); r.data = float(remaining)
            self._rem_pub.publish(r)

            # Also publish ROS cmd_vel (informational)
            tw = Twist(); tw.linear.x = float(vx)
            self._cmd_ros.publish(tw)

            if not hasattr(self, '_pn'): self._pn = 0
            self._pn += 1
            if self._pn % IMU_POLL_HZ == 0:
                print(f'[DRIVE] driven={driven:.2f}m  remaining={remaining:.2f}m  '
                      f'vx={vx:.3f}m/s', flush=True)

            if remaining <= GOAL_TOL_M:
                break

        self._motors_stop()
        with self._lock:
            self._driving   = False
            self._remaining = 0.0

        r = Float32(); r.data = 0.0
        self._rem_pub.publish(r)
        print(f'[DRIVE] arrived  driven={driven:.2f}m', flush=True)
        self._pub_status('GOAL_REACHED')

    # ── Serial motor commands ─────────────────────────────────────────────

    def _motors_fwd(self):
        s = MOTOR_FWD_SPEED
        self._send_motors(s, s)
        print(f'[MOTOR] FORWARD speed={s}', flush=True)

    def _motors_stop(self):
        self._send_motors(0, 0)
        print('[MOTOR] STOP', flush=True)

    def _send_motors(self, left: int, right: int):
        """
        Send [left, right] as signed bytes to Arduino.
        Adjust this to match your actual motor protocol.
        """
        if self._ser is None:
            return
        try:
            packet = struct.pack('bb', int(left), int(right))
            self._ser.write(packet)
        except Exception as e:
            print(f'[MOTOR] write error: {e}', flush=True)

    # ── RViz markers + TF ─────────────────────────────────────────────────

    def _viz_cb(self):
        with self._lock:
            goal     = self._goal_m
            remaining= self._remaining
            driven   = self._driven_m
            driving  = self._driving

        stamp = self.get_clock().now().to_msg()

        if self._tf_br:
            tf = TransformStamped()
            tf.header.stamp    = stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id  = 'base_link'
            tf.transform.translation.x = float(driven)
            tf.transform.rotation.w    = 1.0
            self._tf_br.sendTransform(tf)

        ma = MarkerArray()

        def mk(ns, mid, mtype, x, z, r, g, b, a,
               sx=0.3, sy=0.3, sz=0.3, txt=''):
            m = Marker()
            m.header.stamp = stamp; m.header.frame_id = 'odom'
            m.ns = ns; m.id = mid
            m.type = mtype; m.action = Marker.ADD
            m.pose.position.x = float(x); m.pose.position.z = float(z)
            m.pose.orientation.w = 1.0
            m.scale.x = sx; m.scale.y = sy; m.scale.z = sz
            m.color = ColorRGBA(r=float(r),g=float(g),b=float(b),a=float(a))
            m.lifetime.sec = 1
            if txt: m.text = txt
            return m

        ma.markers.append(mk('rover',0,Marker.CYLINDER,driven,0.05,0.2,0.6,1.0,0.9,0.40,0.40,0.10))
        ma.markers.append(mk('rover',1,Marker.ARROW,driven,0.12,0.4,0.9,1.0,1.0,0.50,0.06,0.06))

        if goal > 0:
            ma.markers.append(mk('goal',0,Marker.SPHERE,goal,0.15,0.0,1.0,0.3,0.9))
            ln = Marker()
            ln.header.stamp = stamp; ln.header.frame_id = 'odom'
            ln.ns='path'; ln.id=0; ln.type=Marker.LINE_STRIP; ln.action=Marker.ADD
            ln.points=[Point(x=driven,z=0.03),Point(x=goal,z=0.03)]
            ln.scale.x=0.04; ln.color=ColorRGBA(r=0.1,g=1.0,b=0.2,a=0.9)
            ln.lifetime.sec=1; ma.markers.append(ln)
            lbl = f'{remaining:.2f}m' if driving else 'ARRIVED'
            ma.markers.append(mk('info',0,Marker.TEXT_VIEW_FACING,
                                 (driven+goal)/2,0.40,1.0,1.0,1.0,1.0,0.1,0.1,0.20,txt=lbl))
            t = 0.5
            while t < goal-0.1:
                ma.markers.append(mk('ticks',int(t*10),Marker.SPHERE,t,0.06,1.0,0.9,0.0,1.0,0.08,0.08,0.08))
                ma.markers.append(mk('tick_lbl',int(t*10),Marker.TEXT_VIEW_FACING,
                                     t,0.22,1.0,0.9,0.0,1.0,0.1,0.1,0.14,txt=f'{t:.1f}m'))
                t += 0.5

        self._viz.publish(ma)

    def _pub_status(self, s):
        m = String(); m.data = s
        self._stat.publish(m)
        print(f'[STATUS] {s}', flush=True)


def main():
    rclpy.init()
    node = DriveSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._motors_stop()
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass


if __name__ == '__main__':
    main()