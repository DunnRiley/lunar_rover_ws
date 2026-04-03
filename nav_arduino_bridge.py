#!/usr/bin/env python3
"""
nav_arduino_bridge.py  —  MINI PC

Bridges ROS2 ↔ Arduino Mega serial for the mission sequencer.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  0xDC PACKET FORMAT  (new 15-bit encoding — matches firmware)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Packet bytes:  0xAA  0xDC  HI  LO  0x55

  The firmware does:
    combined = (HI << 8) | LO         — 16-bit value
    direction = (combined >> 15) & 1  — bit 15: 0=forward, 1=reverse
    units     = combined & 0x7FFF     — bits 14:0 = distance units
    dist_mm   = units * DIST_UNIT_MM  — DIST_UNIT_MM = 1.0 in firmware
    targetCts = dist_mm * COUNTS_PER_MM

  So to drive 2000 mm forward:
    combined  = 2000   (bit 15 = 0, units = 2000)
    HI = 2000 >> 8  = 7
    LO = 2000 & 0xFF = 208
    packet: AA DC 07 D0 55

  To drive 2000 mm reverse:
    combined = 0x8000 | 2000 = 34768
    HI = 34768 >> 8 = 135
    LO = 34768 & 0xFF = 208
    packet: AA DC 87 D0 55

  Max distance per packet: 0x7FFF = 32767 mm = 32.767 m.
  No chunking needed for normal mission distances.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  PORT SHARING WITH joy_to_arduino.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  joy_to_arduino.py writes directly to the Arduino serial port.
  This bridge also needs to write to the same port for mission
  commands.  Run both — they share the port safely because:
    - joy_to_arduino.py only writes when joystick is active
    - The bridge only writes when a mission command arrives
    - The Arduino's RX state machine handles byte-level interleaving
      (it resyncs on the 0xAA start byte)

  If they fight over the port file descriptor, set the param:
    --ros-args -p cmd_port:=/dev/ttyACM1
  pointing the bridge to a second USB cable, OR integrate the
  bridge's write logic directly into joy_to_arduino.py using
  the comment block at the bottom of this file.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SERIAL2 TELEMETRY  (Serial2 on Mega → separate USB-UART)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Packet (30 bytes):
    0xAA
    ax_mms2  int32 LE  (m/s² × 1000)
    ay_mms2  int32 LE
    az_mms2  int32 LE
    gx_scale int32 LE  (deg/s × 1000)
    gy_scale int32 LE
    gz_scale int32 LE
    0xA5
    leftActuatorCount uint16 LE  (actuator encoder, NOT wheel)
    checksum uint8
    0x55

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  PUBLISHED TOPICS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  /nav/arduino_done   Bool   True=DIST_DONE  False=DIST_TIMEOUT
  /nav/encoder_raw    Int32  actuator encoder count (from Serial2)
  /imu/gyro_deg_s     Float32MultiArray  [gx, gy, gz] deg/s
  /imu/accel_ms2      Float32MultiArray  [ax, ay, az] m/s²

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SUBSCRIBED TOPICS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  /nav/arduino_dist_cmd  Float32  metres (+ forward, - reverse)
  /nav/arduino_cmd       Float32MultiArray  [device, speed, dir]
                         Generic raw packet forwarder.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  INTEGRATING INTO joy_to_arduino.py  (no separate node needed)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  In JoyToArduino.__init__, add after self._lock = ...:

    from std_msgs.msg import Bool, Float32, Float32MultiArray
    self._done_pub = self.create_publisher(Bool, '/nav/arduino_done', rel)
    self.create_subscription(Float32, '/nav/arduino_dist_cmd',
                             self._dist_cmd_cb, rel)
    self.create_subscription(Float32MultiArray, '/nav/arduino_cmd',
                             self._generic_cmd_cb, rel)

  Add methods:

    def _dist_cmd_cb(self, msg):
        dist_m = float(msg.data)
        direction_bit = 1 if dist_m < 0 else 0
        units = int(min(0x7FFF, round(abs(dist_m) * 1000.0)))
        combined = (direction_bit << 15) | units
        hi = (combined >> 8) & 0xFF
        lo = combined & 0xFF
        with self._lock:
            self._send_raw(bytes([0xAA, 0xDC, hi, lo, 0x55]))

    def _generic_cmd_cb(self, msg):
        if len(msg.data) >= 3:
            with self._lock:
                self._send(int(msg.data[0]),
                           int(msg.data[1]),
                           int(msg.data[2]))

  In _on_encoder_update, add at end:
    # scan cmd_reader output for DIST_DONE (see bridge _cmd_reader)

  The cleanest integration is to monitor the serial readline output
  that joy_to_arduino already gets (0xAA echo + debug prints) for
  the DIST_DONE/DIST_TIMEOUT strings.
"""

import glob
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32

import serial

# ── Protocol constants ────────────────────────────────────────────────────────
START          = 0xAA
END            = 0x55
ENC_MARKER     = 0xA5
CMD_DRIVE_DIST = 0xDC
CMD_STOP_ALL   = 0xFF

# Firmware DIST_UNIT_MM = 1.0 → units are whole millimetres
DIST_UNIT_MM   = 1.0
MAX_UNITS      = 0x7FFF   # 32767 mm = 32.767 m max per packet

IMU_BYTES = 24
ENC_BYTES = 2
BAUD = 115200
RECONNECT_S = 3.0


class ArduinoBridge(Node):

    def __init__(self):
        super().__init__('nav_arduino_bridge')

        self.declare_parameter('cmd_port',   '')
        self.declare_parameter('telem_port', '')

        cmd_p   = self.get_parameter('cmd_port').value
        telem_p = self.get_parameter('telem_port').value

        # Auto-detect: first ACM/USB = command, second = telemetry
        ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
        self._cmd_port   = cmd_p   or (ports[0] if ports          else '/dev/ttyACM0')
        self._telem_port = telem_p or (ports[1] if len(ports) > 1 else '')

        self._cmd_ser   = None
        self._telem_ser = None
        self._cmd_lock  = threading.Lock()
        self._running   = True

        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        # Publishers
        self._done_pub  = self.create_publisher(Bool,              '/nav/arduino_done', rel)
        self._enc_pub   = self.create_publisher(Int32,             '/nav/encoder_raw',  rel)
        self._gyro_pub  = self.create_publisher(Float32MultiArray, '/imu/gyro_deg_s',   rel)
        self._accel_pub = self.create_publisher(Float32MultiArray, '/imu/accel_ms2',    rel)

        # Subscribers
        self.create_subscription(Float32,           '/nav/arduino_dist_cmd',
                                 self._dist_cmd_cb, rel)
        self.create_subscription(Float32MultiArray, '/nav/arduino_cmd',
                                 self._cmd_cb,      rel)

        self._connect_cmd()
        self._connect_telem()

        threading.Thread(target=self._telem_reader, daemon=True).start()
        threading.Thread(target=self._cmd_reader,   daemon=True).start()

        self.get_logger().info('nav_arduino_bridge ready')
        self.get_logger().info(f'  cmd_port   = {self._cmd_port}')
        self.get_logger().info(
            f'  telem_port = {self._telem_port or "NOT FOUND (no IMU/enc data)"}')
        self.get_logger().info(
            '  Distance encoding: 15-bit  (bit15=dir, bits14:0=mm units)')

    # ── Serial connections ────────────────────────────────────────────────

    def _connect_cmd(self):
        try:
            self._cmd_ser = serial.Serial(
                self._cmd_port, BAUD, timeout=0.05)
            time.sleep(1.5)
            self._cmd_ser.reset_input_buffer()
            self.get_logger().info(f'Command port open: {self._cmd_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open cmd port {self._cmd_port}: {e}')
            self._cmd_ser = None

    def _connect_telem(self):
        if not self._telem_port:
            self.get_logger().warn(
                'No telemetry port — IMU and actuator encoder data unavailable.\n'
                '  Wire Mega Serial2 TX (pin 16) to a USB-UART adapter,\n'
                '  then: --ros-args -p telem_port:=/dev/ttyUSBx')
            return
        try:
            self._telem_ser = serial.Serial(
                self._telem_port, BAUD, timeout=0.1)
            time.sleep(1.5)
            self._telem_ser.reset_input_buffer()
            self.get_logger().info(f'Telemetry port open: {self._telem_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open telem port {self._telem_port}: {e}')
            self._telem_ser = None

    # ── Command port reader  (watches for DIST_DONE / DIST_TIMEOUT) ───────

    def _cmd_reader(self):
        """Read text lines from the command port (Arduino Serial debug output)."""
        buf = b''
        while self._running:
            if self._cmd_ser is None or not self._cmd_ser.is_open:
                time.sleep(RECONNECT_S)
                self._connect_cmd()
                buf = b''
                continue
            try:
                chunk = self._cmd_ser.read(256)
            except serial.SerialException:
                self._cmd_ser = None
                continue
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('ascii', errors='ignore').strip()
                if not text:
                    continue
                self.get_logger().debug(f'[Arduino] {text}')
                if text.startswith('DIST_DONE'):
                    m = Bool(); m.data = True
                    self._done_pub.publish(m)
                    self.get_logger().info('[bridge] DIST_DONE → /nav/arduino_done True')
                elif text.startswith('DIST_TIMEOUT'):
                    m = Bool(); m.data = False
                    self._done_pub.publish(m)
                    self.get_logger().warn('[bridge] DIST_TIMEOUT → /nav/arduino_done False')
                elif text.startswith('DIST_START'):
                    self.get_logger().info(f'[bridge] Arduino: {text}')

    # ── Telemetry port reader  (binary IMU + actuator encoder) ────────────

    def _telem_reader(self):
        (STATE_HUNT, STATE_IMU, STATE_ENC_M,
         STATE_ENC, STATE_CHK, STATE_END) = range(6)

        state = STATE_HUNT
        imu_buf = bytearray()
        enc_buf = bytearray()
        chk_calc = 0

        while self._running:
            if self._telem_ser is None or not self._telem_ser.is_open:
                time.sleep(RECONNECT_S)
                self._connect_telem()
                state = STATE_HUNT
                imu_buf = bytearray(); enc_buf = bytearray(); chk_calc = 0
                continue
            try:
                b = self._telem_ser.read(1)
            except serial.SerialException:
                self._telem_ser = None
                continue
            if not b:
                continue
            byte = b[0]

            if state == STATE_HUNT:
                if byte == START:
                    state = STATE_IMU
                    imu_buf = bytearray(); enc_buf = bytearray(); chk_calc = 0
            elif state == STATE_IMU:
                imu_buf.append(byte)
                chk_calc ^= byte
                if len(imu_buf) == IMU_BYTES:
                    state = STATE_ENC_M
            elif state == STATE_ENC_M:
                if byte == ENC_MARKER:
                    chk_calc ^= byte
                    state = STATE_ENC
                else:
                    state = STATE_HUNT
            elif state == STATE_ENC:
                enc_buf.append(byte)
                chk_calc ^= byte
                if len(enc_buf) == ENC_BYTES:
                    state = STATE_CHK
            elif state == STATE_CHK:
                if byte == chk_calc:
                    self._dispatch_telemetry(imu_buf, enc_buf)
                else:
                    self.get_logger().warn(
                        f'[telem] checksum mismatch got=0x{byte:02X} '
                        f'expected=0x{chk_calc:02X}',
                        throttle_duration_sec=5.0)
                state = STATE_END
            elif state == STATE_END:
                if byte != END:
                    self.get_logger().warn(
                        f'[telem] expected 0x55 got 0x{byte:02X}',
                        throttle_duration_sec=5.0)
                state = STATE_HUNT
                imu_buf = bytearray(); enc_buf = bytearray(); chk_calc = 0

    def _dispatch_telemetry(self, imu_buf: bytearray, enc_buf: bytearray):
        ax, ay, az, gx, gy, gz = struct.unpack_from('<6i', imu_buf)
        ax_f = ax / 1000.0; ay_f = ay / 1000.0; az_f = az / 1000.0
        gx_f = gx / 1000.0; gy_f = gy / 1000.0; gz_f = gz / 1000.0

        g = Float32MultiArray(); g.data = [gx_f, gy_f, gz_f]
        self._gyro_pub.publish(g)
        a = Float32MultiArray(); a.data = [ax_f, ay_f, az_f]
        self._accel_pub.publish(a)

        enc_count = struct.unpack_from('<H', enc_buf)[0]
        enc_msg = Int32(); enc_msg.data = enc_count
        self._enc_pub.publish(enc_msg)

    # ── ROS subscribers ───────────────────────────────────────────────────

    def _dist_cmd_cb(self, msg: Float32):
        """
        Convert distance in metres to a 15-bit 0xDC packet.

        Firmware decoding:
          combined = (HI << 8) | LO
          direction = (combined >> 15) & 1   0=fwd, 1=rev
          units     = combined & 0x7FFF
          dist_mm   = units * 1.0 mm/unit

        Max distance: 32767 mm ≈ 32.8 m  (single packet, no chunking needed)
        """
        dist_m = float(msg.data)
        direction_bit = 1 if dist_m < 0 else 0
        dist_mm = abs(dist_m) * 1000.0
        units = int(min(MAX_UNITS, round(dist_mm / DIST_UNIT_MM)))

        if units == 0:
            self.get_logger().warn(
                f'[bridge] dist {dist_m:.4f}m rounds to 0 units — ignored')
            return

        combined = (direction_bit << 15) | units
        hi = (combined >> 8) & 0xFF
        lo = combined & 0xFF

        pkt = bytes([START, CMD_DRIVE_DIST, hi, lo, END])
        self._write_packet(pkt)
        self.get_logger().info(
            f'[bridge] dist_cmd {dist_m:+.3f}m → '
            f'0xDC combined=0x{combined:04X} '
            f'[AA DC {hi:02X} {lo:02X} 55]  '
            f'({units}mm {["FWD","REV"][direction_bit]})')

    def _cmd_cb(self, msg: Float32MultiArray):
        """Forward [device, speed, direction] as a raw 5-byte packet."""
        if len(msg.data) < 3:
            return
        self._write_packet(bytes([
            START,
            int(msg.data[0]) & 0xFF,
            int(msg.data[1]) & 0xFF,
            int(msg.data[2]) & 0xFF,
            END
        ]))

    def _write_packet(self, pkt: bytes):
        with self._cmd_lock:
            if self._cmd_ser and self._cmd_ser.is_open:
                try:
                    self._cmd_ser.write(pkt)
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')
                    self._cmd_ser = None

    # ── Cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        for ser in (self._cmd_ser, self._telem_ser):
            if ser and ser.is_open:
                try:
                    ser.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()