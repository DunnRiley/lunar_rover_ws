import serial
import keyboard
import time

PORT = '/dev/ttyACM0'   # change for Windows: COM3
BAUD = 115200

ser = serial.Serial(PORT, BAUD)
time.sleep(2)

START = 0xAA
END   = 0x55

FL = 0x01
FR = 0x02
BL = 0x03
BR = 0x04
AL = 0xD4
AR = 0xF7
KILL = 0xFF

def send_packet(device, speed, direction):
    packet = bytes([START, device, speed, direction, END])
    ser.write(packet)

speed = 150

print("Teleop Ready")

while True:

    if keyboard.is_pressed('w'):
        for d in [FL, FR, BL, BR]:
            send_packet(d, speed, 0x00)

    elif keyboard.is_pressed('s'):
        for d in [FL, FR, BL, BR]:
            send_packet(d, speed, 0x01)

    elif keyboard.is_pressed('a'):
        send_packet(FL, speed, 0x01)
        send_packet(BL, speed, 0x01)
        send_packet(FR, speed, 0x00)
        send_packet(BR, speed, 0x00)

    elif keyboard.is_pressed('d'):
        send_packet(FL, speed, 0x00)
        send_packet(BL, speed, 0x00)
        send_packet(FR, speed, 0x01)
        send_packet(BR, speed, 0x01)

    elif keyboard.is_pressed('p'):   # both actuators forward
        send_packet(AL, speed, 0x00)
        send_packet(AR, speed, 0x00)

    elif keyboard.is_pressed('l'):   # both actuators reverse
        send_packet(AL, speed, 0x01)
        send_packet(AR, speed, 0x01)

    elif keyboard.is_pressed('o'):   # left only forward
        send_packet(AL, speed, 0x00)

    elif keyboard.is_pressed('k'):   # left only reverse
        send_packet(AL, speed, 0x01)

    elif keyboard.is_pressed('i'):   # right only forward
        send_packet(AR, speed, 0x00)

    elif keyboard.is_pressed('j'):   # right only reverse
        send_packet(AR, speed, 0x01)

    elif keyboard.is_pressed('space'):
        send_packet(KILL, 0, 0)

    time.sleep(0.05)