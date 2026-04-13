#!/usr/bin/env python3
"""
nav_arduino_bridge.py  —  MINI PC

Bridges /nav/arduino_dist_cmd  (Float32, metres)  -> 0xDC serial packet
Bridges /nav/arduino_cmd       (Float32MultiArray) -> raw packet forwarder
Bridges /nav/arduino_turn_cmd  (Float32MultiArray [arc_mm, speed, clockwise_int])
                                -> 0xC8 + 0xC9 + turn-start (0xE8 default)

Publishes:
  /imu/gyro_deg_s  Float32MultiArray  [gx, gy, gz] from Serial2 telemetry
  /imu/accel_ms2   Float32MultiArray  [ax, ay, az]
  /nav/encoder_raw Float32            actuator encoder count

IMPORTANT: Arduino 0xDC does NOT send "DIST_DONE". The bridge does not
publish /nav/arduino_done for distance drives. The mission sequencer uses
time-based completion instead.
"""

import glob
import struct
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Float32MultiArray

START      = 0xAA
END        = 0x55
ENC_MARKER = 0xA5
IMU_BYTES  = 24
ENC_BYTES  = 2
BAUD       = 115200
TURN_START_DEFAULT = 0xE8


def pkt(device, speed=0, direction=0, lobyte=0):
    d, sp, di, lo = device & 0xFF, speed & 0xFF, direction & 0xFF, lobyte & 0xFF
    return bytes([START, d, sp, di, lo, d ^ sp ^ di ^ lo, END])


def encode_dist(mm, reverse=False):
    units    = int(min(0x7FFF, max(0, round(abs(mm)))))
    combined = ((1 if reverse else 0) << 15) | units
    return (combined >> 8) & 0xFF, combined & 0xFF


class Bridge(Node):

    def __init__(self):
        super().__init__("nav_arduino_bridge")
        self.declare_parameter("cmd_port",   "")
        self.declare_parameter("telem_port", "")
        self.declare_parameter("pivot_use_opposite_dirs", True)
        self.declare_parameter("pivot_flip_cw_ccw", False)
        self.declare_parameter("turn_start_cmd", TURN_START_DEFAULT)
        cmd_p   = self.get_parameter("cmd_port").value
        telem_p = self.get_parameter("telem_port").value
        self._pivot_use_opposite = bool(
            self.get_parameter("pivot_use_opposite_dirs").value)
        self._pivot_flip = bool(self.get_parameter("pivot_flip_cw_ccw").value)
        self._turn_start_cmd = int(self.get_parameter("turn_start_cmd").value) & 0xFF

        ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        self._cmd_port   = cmd_p   or (ports[0] if ports else "/dev/ttyACM0")
        self._telem_port = telem_p or (ports[1] if len(ports) > 1 else "")

        self._cmd_ser   = None
        self._telem_ser = None
        self._lock      = threading.Lock()   # protects serial write
        self._seq_lock  = threading.Lock()   # ensures C8+C9+E8 are never interleaved
        self._running   = True

        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self._gyro_pub  = self.create_publisher(Float32MultiArray, "/imu/gyro_deg_s",   rel)
        self._accel_pub = self.create_publisher(Float32MultiArray, "/imu/accel_ms2",    rel)
        self._enc_pub   = self.create_publisher(Float32,           "/nav/encoder_raw",  rel)

        self.create_subscription(Float32,           "/nav/arduino_dist_cmd", self._dist_cb, rel)
        self.create_subscription(Float32MultiArray, "/nav/arduino_cmd",      self._cmd_cb,  rel)
        self.create_subscription(Float32MultiArray, "/nav/arduino_turn_cmd", self._turn_cb, rel)

        self._connect_cmd()
        self._connect_telem()
        threading.Thread(target=self._telem_reader, daemon=True).start()

        self.get_logger().info(f"Arduino bridge: cmd={self._cmd_port}")
        self.get_logger().info(
            f"  telem={self._telem_port or 'NONE (no IMU/enc data)'}")
        self.get_logger().info(
            f"  pivot_use_opposite_dirs={self._pivot_use_opposite}  "
            f"pivot_flip_cw_ccw={self._pivot_flip}")
        self.get_logger().info(f"  turn_start_cmd=0x{self._turn_start_cmd:02X}")
        self.get_logger().info("  NOTE: 0xDC does not produce DIST_DONE output.")

    def _connect_cmd(self):
        try:
            self._cmd_ser = serial.Serial(self._cmd_port, BAUD, timeout=0.05)
            time.sleep(1.5)
            self._cmd_ser.reset_input_buffer()
            self.get_logger().info(f"Cmd port open: {self._cmd_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Cmd port: {e}")
            self._cmd_ser = None

    def _connect_telem(self):
        if not self._telem_port:
            return
        try:
            self._telem_ser = serial.Serial(self._telem_port, BAUD, timeout=0.1)
            time.sleep(1.5)
            self._telem_ser.reset_input_buffer()
            self.get_logger().info(f"Telem port open: {self._telem_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Telem port: {e}")
            self._telem_ser = None

    def _write(self, data: bytes):
        with self._lock:
            if self._cmd_ser and self._cmd_ser.is_open:
                try:
                    self._cmd_ser.write(data)
                except serial.SerialException as e:
                    self.get_logger().error(f"Write: {e}")
                    self._cmd_ser = None

    def _dist_cb(self, msg: Float32):
        metres = float(msg.data)
        dist_mm = abs(metres) * 1000.0
        db, lo = encode_dist(dist_mm, metres < 0)
        self._write(pkt(0xDC, 120, db, lo))
        self.get_logger().info(
            f"0xDC {metres:+.3f}m  [DC 78 {db:02X} {lo:02X}]")

    def _turn_cb(self, msg: Float32MultiArray):
        """
        [arc_mm, speed, clockwise_int, optional_start_cmd]
        clockwise_int: 1=CW, 0=CCW

        FIX for "second click goes forward":
          We use _seq_lock so two calls can never interleave their packets.
          We also send STOPALL before loading C8/C9 targets so that any
          previously running command's state cannot pollute direction bytes.
        """
        if len(msg.data) < 3:
            return
        arc_mm    = float(msg.data[0])
        speed     = int(msg.data[1])
        clockwise = bool(int(msg.data[2]))
        start_cmd = int(msg.data[3]) & 0xFF if len(msg.data) >= 4 else self._turn_start_cmd
        if self._pivot_flip:
            clockwise = not clockwise
        speed     = min(speed, 190)

        if self._pivot_use_opposite:
            # Most firmware variants need opposite packed direction bits for C8/C9
            # to produce an in-place pivot (instead of both sides moving straight).
            # CW  => left fwd, right rev ;  CCW => left rev, right fwd
            left_rev  = not clockwise
            right_rev = clockwise
        else:
            # Legacy mode: same direction bit sent to both sides.
            left_rev = right_rev = (not clockwise)

        l_db, l_lo = encode_dist(arc_mm, left_rev)
        r_db, r_lo = encode_dist(arc_mm, right_rev)

        with self._seq_lock:
            # Stop any running command first — clears ddDirection state on Arduino
            self._write(pkt(0xFF, 0, 0, 0))
            time.sleep(0.05)
            # Load left wheel target
            self._write(pkt(0xC8, speed, l_db, l_lo))
            time.sleep(0.04)
            # Load right wheel target
            self._write(pkt(0xC9, speed, r_db, r_lo))
            time.sleep(0.04)
            # Start turn (this firmware uses 0xE8 by default)
            self._write(pkt(start_cmd, 0, 0, 0))

        self.get_logger().info(
            f"Turn arc={arc_mm:.0f}mm speed={speed} "
            f"{'CW' if clockwise else 'CCW'} "
            f"[FF→C8({l_db:02X},{l_lo:02X})→C9({r_db:02X},{r_lo:02X})→{start_cmd:02X}]")

    def _cmd_cb(self, msg: Float32MultiArray):
        """[device, speed, direction] or [device, speed, direction, lobyte]"""
        if len(msg.data) < 3:
            return
        device    = int(msg.data[0]) & 0xFF
        speed     = int(msg.data[1]) & 0xFF
        direction = int(msg.data[2]) & 0xFF
        lobyte    = int(msg.data[3]) & 0xFF if len(msg.data) >= 4 else 0
        self._write(pkt(device, speed, direction, lobyte))

    def _telem_reader(self):
        (HUNT, IMU, ENC_M, ENC, CHK, DONE) = range(6)
        state = HUNT
        imu_buf = bytearray(); enc_buf = bytearray(); chk_calc = 0

        while self._running:
            if self._telem_ser is None or not self._telem_ser.is_open:
                time.sleep(3.0)
                self._connect_telem()
                state = HUNT; imu_buf = bytearray(); enc_buf = bytearray()
                continue
            try:
                b = self._telem_ser.read(1)
            except serial.SerialException:
                self._telem_ser = None; continue
            if not b: continue
            byte = b[0]

            if state == HUNT:
                if byte == START:
                    state = IMU; imu_buf = bytearray(); enc_buf = bytearray(); chk_calc = 0
            elif state == IMU:
                imu_buf.append(byte); chk_calc ^= byte
                if len(imu_buf) == IMU_BYTES: state = ENC_M
            elif state == ENC_M:
                if byte == ENC_MARKER: chk_calc ^= byte; state = ENC
                else: state = HUNT
            elif state == ENC:
                enc_buf.append(byte); chk_calc ^= byte
                if len(enc_buf) == ENC_BYTES: state = CHK
            elif state == CHK:
                if byte == chk_calc:
                    ax,ay,az,gx,gy,gz = struct.unpack_from("<6i", imu_buf)
                    g = Float32MultiArray(); g.data = [gx/1000.0, gy/1000.0, gz/1000.0]
                    self._gyro_pub.publish(g)
                    a = Float32MultiArray(); a.data = [ax/1000.0, ay/1000.0, az/1000.0]
                    self._accel_pub.publish(a)
                    ec = struct.unpack_from("<H", enc_buf)[0]
                    em = Float32(); em.data = float(ec)
                    self._enc_pub.publish(em)
                state = DONE
            elif state == DONE:
                state = HUNT; imu_buf = bytearray(); enc_buf = bytearray()

    def destroy_node(self):
        self._running = False
        for s in (self._cmd_ser, self._telem_ser):
            if s and s.is_open:
                try: s.close()
                except: pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass

if __name__ == "__main__":
    main()
