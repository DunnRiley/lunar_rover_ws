#!/usr/bin/env python3
"""
imu_reader.py  —  MINI PC
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Reads gyro + accelerometer from Arduino over serial.
Publishes /imu/gyro_deg_s and /imu/accel_ms2 (Float32MultiArray)
and prints live data to the terminal.

IMU PROTOCOL (from your notes):
  Request:   0xAA 0xD1 0x55   →  Arduino sends IMU packet now
  Response:  signed 16-bit little-endian integers, /1000 to get
             physical units (deg/s for gyro, m/s² for accel)

Edit SERIAL_PORT and BAUD_RATE below.
Edit PACKET_FORMAT to match how your Arduino packs the response.
"""

import struct, glob, time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, String

# ── EDIT THESE ────────────────────────────────────────────────────────────────
SERIAL_PORT   = ''       # blank = auto-detect /dev/ttyACM*
BAUD_RATE     = 9600
POLL_HZ       = 20

# Byte indices in the response packet for each field.
# Each field is a signed 16-bit LE integer at that byte offset.
# Adjust these once you've confirmed the packet layout with
# the raw dump printed at startup.
GYRO_X_BYTE  = 0    # roll rate  (deg/s * 1000)
GYRO_Y_BYTE  = 2    # pitch rate (deg/s * 1000)
GYRO_Z_BYTE  = 4    # yaw rate   (deg/s * 1000)
ACCEL_X_BYTE = 6    # forward    (m/s²  * 1000)
ACCEL_Y_BYTE = 8    # lateral    (m/s²  * 1000)
ACCEL_Z_BYTE = 10   # vertical   (m/s²  * 1000)
MIN_PACKET   = 12   # minimum response bytes expected
# ── END EDIT ──────────────────────────────────────────────────────────────────

IMU_REQUEST = bytes([0xAA, 0xD1, 0x55])


def _find_port():
    if SERIAL_PORT: return SERIAL_PORT
    for p in sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')):
        return p
    return None


def _parse(raw: bytes) -> dict:
    """Parse one IMU response packet."""
    out = {}
    def s16(offset):
        if offset+2 > len(raw): return 0.0
        return struct.unpack('<h', raw[offset:offset+2])[0] / 1000.0

    out['gx'] = s16(GYRO_X_BYTE)
    out['gy'] = s16(GYRO_Y_BYTE)
    out['gz'] = s16(GYRO_Z_BYTE)
    out['ax'] = s16(ACCEL_X_BYTE)
    out['ay'] = s16(ACCEL_Y_BYTE)
    out['az'] = s16(ACCEL_Z_BYTE)
    return out


class ImuReader(Node):
    def __init__(self):
        super().__init__('imu_reader')
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=10)

        self._gyro_pub  = self.create_publisher(Float32MultiArray, '/imu/gyro_deg_s', q)
        self._accel_pub = self.create_publisher(Float32MultiArray, '/imu/accel_ms2',  q)
        self._stat_pub  = self.create_publisher(String,            '/imu/status',     q)

        port = _find_port()
        self._ser = None
        if port:
            try:
                self._ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
                print(f'[IMU] opened {port} @ {BAUD_RATE}', flush=True)
            except Exception as e:
                print(f'[IMU] ERROR: {e}', flush=True)
        else:
            print('[IMU] WARNING: no serial port found', flush=True)

        # Print first raw packet so you can confirm byte layout
        self._dumped = False

        self.create_timer(1.0/POLL_HZ, self._poll)
        self._n = 0

        print('\n[IMU] ready — polling at', POLL_HZ, 'Hz', flush=True)
        print('[IMU] publishing /imu/gyro_deg_s and /imu/accel_ms2', flush=True)
        print('[IMU] First raw packet will be printed to confirm byte layout\n',
              flush=True)

    def _poll(self):
        if self._ser is None:
            return
        try:
            self._ser.write(IMU_REQUEST)
            time.sleep(0.008)
            raw = self._ser.read(self._ser.in_waiting or MIN_PACKET)
        except Exception as e:
            print(f'[IMU] poll error: {e}', flush=True)
            return

        if len(raw) < 2:
            return

        # First packet: dump raw bytes to confirm layout
        if not self._dumped:
            print(f'[IMU] FIRST PACKET ({len(raw)} bytes):', flush=True)
            print('  hex:', raw.hex(), flush=True)
            print('  as signed 16-bit LE words:',
                  [struct.unpack('<h', raw[i:i+2])[0]
                   for i in range(0, len(raw)-1, 2)], flush=True)
            print('  Adjust GYRO_*_BYTE / ACCEL_*_BYTE in imu_reader.py '
                  'to match your packet', flush=True)
            self._dumped = True

        d = _parse(raw)

        g = Float32MultiArray()
        g.data = [d['gx'], d['gy'], d['gz']]
        self._gyro_pub.publish(g)

        a = Float32MultiArray()
        a.data = [d['ax'], d['ay'], d['az']]
        self._accel_pub.publish(a)

        self._n += 1
        if self._n % POLL_HZ == 0:
            print(f'[IMU] gyro=({d["gx"]:+6.2f},{d["gy"]:+6.2f},{d["gz"]:+6.2f}) deg/s  '
                  f'accel=({d["ax"]:+6.3f},{d["ay"]:+6.3f},{d["az"]:+6.3f}) m/s²',
                  flush=True)


def main():
    rclpy.init()
    node = ImuReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass


if __name__ == '__main__':
    main()