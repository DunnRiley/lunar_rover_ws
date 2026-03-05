import time
import threading
import serial
from pynput import keyboard

START = 0xAA
END   = 0x55

DEVICE_ACTUATORS = 0x08
DEVICE_STOP      = 0xFF

# Change this:
PORT = "COM3"
BAUD = 115200

# Direction values must match your motor driver wiring:
# try EXTEND_DIR=1 / RETRACT_DIR=0; if reversed, swap them.
EXTEND_DIR  = 1
RETRACT_DIR = 0

MOVE_SPEED = 180  # 0-255 PWM

ser = serial.Serial(PORT, BAUD, timeout=0.1)

latest_left = 0
latest_right = 0
latest_time = 0.0

lock = threading.Lock()
running = True

# Track whether a key is currently held down so we don’t spam commands
moving_state = None  # "extend", "retract", None


def send_packet(device: int, speed: int, direction: int):
    pkt = bytes([START, device & 0xFF, speed & 0xFF, direction & 0xFF, END])
    ser.write(pkt)


def move_extend():
    send_packet(DEVICE_ACTUATORS, MOVE_SPEED, EXTEND_DIR)


def move_retract():
    send_packet(DEVICE_ACTUATORS, MOVE_SPEED, RETRACT_DIR)


def stop_all():
    send_packet(DEVICE_STOP, 0, 0)


def reader_thread():
    global latest_left, latest_right, latest_time
    while running:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            if line.startswith("ACT,"):
                parts = line.split(",")
                if len(parts) == 3:
                    with lock:
                        latest_left = int(parts[1])
                        latest_right = int(parts[2])
                        latest_time = time.time()
        except Exception:
            pass


def print_latest_counts(tag=""):
    with lock:
        l, r, t = latest_left, latest_right, latest_time
    age_ms = (time.time() - t) * 1000.0
    print(f"{tag}Left={l}  Right={r}  (age={age_ms:.0f} ms)")


def on_press(key):
    global moving_state
    try:
        if key.char.lower() == 'w':
            if moving_state != "extend":
                moving_state = "extend"
                move_extend()
                print("EXTEND (hold W)")
        elif key.char.lower() == 's':
            if moving_state != "retract":
                moving_state = "retract"
                move_retract()
                print("RETRACT (hold S)")
    except AttributeError:
        # special keys
        if key == keyboard.Key.space:
            moving_state = None
            stop_all()
            time.sleep(0.05)
            print_latest_counts(tag="STOP: ")
        elif key == keyboard.Key.esc:
            return False  # quit


def on_release(key):
    global moving_state
    try:
        if key.char.lower() in ['w', 's']:
            # When you release W or S: stop, then print counts
            moving_state = None
            stop_all()
            time.sleep(0.05)  # give Arduino a moment to push fresh ACT line
            print_latest_counts(tag="RELEASE: ")
    except AttributeError:
        pass


if __name__ == "__main__":
    t = threading.Thread(target=reader_thread, daemon=True)
    t.start()

    print("Controls:")
    print("  Hold W = extend")
    print("  Hold S = retract")
    print("  Space  = stop + print counts")
    print("  Esc    = quit")

    stop_all()

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    running = False
    stop_all()
    ser.close()