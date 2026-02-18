import serial
from pynput import keyboard

SERIAL_PORT = "/dev/ttyACM0"  # adjust if needed
BAUD = 115200

ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)

keymap = {
    'w': 'w',
    's': 's',
    'a': 'a',
    'd': 'd',
    'q': 'q',  # wa
    'e': 'e',  # wd
    'z': 'z',  # sa
    'c': 'c',  # sd
    'r': 'r',
    'f': 'f',
    'o': 'o',
    'l': 'l',
}

pressed = set()

def send():
    for k in pressed:
        ser.write(k.encode())

def on_press(key):
    try:
        if key.char in keymap:
            pressed.add(keymap[key.char])
            send()
    except AttributeError:
        pass

def on_release(key):
    try:
        if key.char in pressed:
            pressed.remove(key.char)
    except AttributeError:
        pass

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
