import sys
import time
import threading
import serial
from pynput import keyboard

START = 0xAA
END   = 0x55

DEV_LEFT_DRIVE   = 0x05
DEV_RIGHT_DRIVE  = 0x06
DEV_SERVO1       = 0x11

DEV_DIG_POS      = 0xA7
DEV_DRIVE_POS    = 0xA9
DEV_DUMP_POS     = 0xB3
DEV_CALIBRATE    = 0xCA

DEV_STOPALL      = 0xFF

DIR_FWD = 1
DIR_REV = 0


class Teleop:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.05,
            write_timeout=0.05,
        )

        # Reset Arduino (DTR toggle)
        self.ser.dtr = False
        time.sleep(0.2)
        self.ser.dtr = True
        time.sleep(1.8)

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.speed = 160
        self.turn_speed = 160
        self.fwd = self.rev = self.left = self.right = False

        self._stop = False
        self._ack_seen = False

        # rate limit ONLY for wheel pair updates
        self._last_pair_send = 0.0
        self._min_pair_period = 0.03  # 30ms

        t = threading.Thread(target=self._reader, daemon=True)
        t.start()

        self._handshake()

    def close(self):
        try:
            # stop a couple times on exit
            self.send_cmd(DEV_STOPALL, 0, 0, retries=3)
        except Exception:
            pass
        self._stop = True
        try:
            self.ser.close()
        except Exception:
            pass

    def _reader(self):
        while not self._stop:
            try:
                b = self.ser.read(64)
                if not b:
                    continue
                for v in b:
                    if v == 0xAA:
                        self._ack_seen = True
            except Exception:
                break

    def _handshake(self):
        print("Waiting for Arduino ACK...")
        deadline = time.time() + 2.5
        while time.time() < deadline and not self._ack_seen:
            self.send_cmd(DEV_STOPALL, 0, 0, retries=1)
            time.sleep(0.1)
        print("Handshake OK." if self._ack_seen else "No ACK seen yet (may still work).")

    def _pkt(self, device: int, speed: int, direction: int) -> bytes:
        speed = max(0, min(255, int(speed)))
        direction = 1 if direction else 0
        return bytes([START, device & 0xFF, speed & 0xFF, direction & 0xFF, END])

    # Generic send with optional retries (use this for wheels/stop)
    def send_cmd(self, device: int, speed: int, direction: int, retries: int = 1):
        pkt = self._pkt(device, speed, direction)
        for _ in range(max(1, retries)):
            self.ser.write(pkt)
            time.sleep(0.002)

    # Send actuator "mode/state" commands exactly once (your request)
    def send_once(self, device: int):
        self.ser.write(self._pkt(device, 0, 0))

    # Wheels: send left+right together, rate-limited, with retries so it actually moves
    def send_pair(self, sp_left, dir_left, sp_right, dir_right, retries=2):
        now = time.time()
        if (now - self._last_pair_send) < self._min_pair_period:
            return
        self._last_pair_send = now

        pktL = self._pkt(DEV_LEFT_DRIVE, sp_left, dir_left)
        pktR = self._pkt(DEV_RIGHT_DRIVE, sp_right, dir_right)

        for _ in range(max(1, retries)):
            self.ser.write(pktL)
            self.ser.write(pktR)
            time.sleep(0.002)

    def _drive_update(self):
        if self.fwd and not self.rev:
            self.send_pair(self.speed, DIR_FWD, self.speed, DIR_FWD, retries=2)
            return
        if self.rev and not self.fwd:
            self.send_pair(self.speed, DIR_REV, self.speed, DIR_REV, retries=2)
            return
        if self.left and not self.right:
            self.send_pair(self.turn_speed, DIR_REV, self.turn_speed, DIR_FWD, retries=2)
            return
        if self.right and not self.left:
            self.send_pair(self.turn_speed, DIR_FWD, self.turn_speed, DIR_REV, retries=2)
            return

        # stop wheels (a couple retries helps too)
        self.send_pair(0, DIR_FWD, 0, DIR_FWD, retries=2)

    def on_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            k = None

        # wheels
        if k == 'w':
            self.fwd = True; self._drive_update()
        elif k == 's':
            self.rev = True; self._drive_update()
        elif k == 'a':
            self.left = True; self._drive_update()
        elif k == 'd':
            self.right = True; self._drive_update()

        # speed
        elif k == ']':
            self.speed = min(255, self.speed + 10)
            self.turn_speed = min(255, self.turn_speed + 10)
            print(f"speed={self.speed}")
            self._drive_update()
        elif k == '[':
            self.speed = max(0, self.speed - 10)
            self.turn_speed = max(0, self.turn_speed - 10)
            print(f"speed={self.speed}")
            self._drive_update()

        # actuator state machine (SENT ONCE)
        elif k == '1':
            self.send_once(DEV_DIG_POS)
            print("act: DIGPOSITION (sent once)")
        elif k == '2':
            self.send_once(DEV_DRIVE_POS)
            print("act: DRIVEPOSITION (sent once)")
        elif k == '3':
            self.send_once(DEV_DUMP_POS)
            print("act: DUMPBUCKET (sent once)")
        elif k == 'c':
            self.send_once(DEV_CALIBRATE)
            print("act: CALIBRATE (sent once)")

        # servo (once)
        elif k == 'q':
            self.send_cmd(DEV_SERVO1, 45, 0, retries=1)
        elif k == 'e':
            self.send_cmd(DEV_SERVO1, 135, 0, retries=1)
        elif k == 'r':
            self.send_cmd(DEV_SERVO1, 90, 0, retries=1)

        # stop all (send a few times so it always lands)
        elif k == ' ':
            self.send_cmd(DEV_STOPALL, 0, 0, retries=3)
            print("STOPALL")

        # quit
        elif k == 'x':
            print("Exiting...")
            self._stop = True
            return False

    def on_release(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            k = None

        if k == 'w':
            self.fwd = False; self._drive_update()
        elif k == 's':
            self.rev = False; self._drive_update()
        elif k == 'a':
            self.left = False; self._drive_update()
        elif k == 'd':
            self.right = False; self._drive_update()


def main():
    if len(sys.argv) < 2:
        print("Usage: python teleop.py COM7   (or /dev/ttyACM0)")
        sys.exit(1)

    teleop = Teleop(sys.argv[1])

    print("Controls:")
    print("  WASD: drive | [ / ]: speed down/up")
    print("  1/2/3: dig/drive/dump (sent once) | c: calibrate (sent once)")
    print("  q/e/r: servo CCW/CW/stop")
    print("  SPACE: STOPALL | x: quit")

    try:
        with keyboard.Listener(on_press=teleop.on_press, on_release=teleop.on_release) as listener:
            listener.join()
    finally:
        teleop.close()


if __name__ == "__main__":
    main()