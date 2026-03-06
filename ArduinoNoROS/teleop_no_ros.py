import curses
from typing import Self
import serial
import time

PORT = '/dev/ttyACM0'
BAUD = 115200

START = 0xAA
END   = 0x55

FL = 0x01
FR = 0x02
BL = 0x03
BR = 0x04
AL = 0xD4
AR = 0xF7
SERVO = 0x11
KILL = 0xFF

SPEED = 150
angle = 15
currAngle = 0


def send_packet(ser, device, speed, direction):
    packet = bytes([START, device, speed, direction, END])
    ser.write(packet)


def stop_drive(ser):
    for d in [FL, FR, BL, BR]:
        send_packet(ser, d, 0, 0)


def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)

    ser = serial.Serial(PORT, BAUD)
    time.sleep(2)  # Allow Arduino reset

    stdscr.addstr(0, 0, "Rover Teleop (Curses)")
    stdscr.addstr(1, 0, "P = Servo + 15 deg")
    stdscr.addstr(3, 0, "SPACE = Kill")
    stdscr.addstr(4, 0, "Q = Quit")

    while True:
        key = stdscr.getch()

        if key == ord('q'):
            break

        # -------- DRIVE --------
        elif key == ord('a'):
            for d in [FL, FR, BL, BR]:
                send_packet(ser, d, SPEED, 0x00)
            stdscr.addstr(10, 0, "Driving Forward      ")

        elif key == ord('d'):
            for d in [FL, FR, BL, BR]:
                send_packet(ser, d, SPEED, 0x01)
            stdscr.addstr(10, 0, "Driving Backward     ")

        elif key == ord('w'):
            send_packet(ser, FL, SPEED, 0x01)
            send_packet(ser, BL, SPEED, 0x01)
            send_packet(ser, FR, SPEED, 0x00)
            send_packet(ser, BR, SPEED, 0x00)
            stdscr.addstr(10, 0, "Turning Left         ")

        elif key == ord('s'):
            send_packet(ser, FL, SPEED, 0x00)
            send_packet(ser, BL, SPEED, 0x00)
            send_packet(ser, FR, SPEED, 0x01)
            send_packet(ser, BR, SPEED, 0x01)
            stdscr.addstr(10, 0, "Turning Right        ")

        # -------- INDIVIDUAL MOTOR TEST --------
        elif key == ord('1'):
            stop_drive(ser)
            send_packet(ser, FL, SPEED, 0x00)
            stdscr.addstr(10, 0, "Testing FL Motor     ")

        elif key == ord('2'):
            stop_drive(ser)
            send_packet(ser, FR, SPEED, 0x00)
            stdscr.addstr(10, 0, "Testing FR Motor     ")

        elif key == ord('3'):
            stop_drive(ser)
            send_packet(ser, BL, SPEED, 0x00)
            stdscr.addstr(10, 0, "Testing BL Motor     ")

        elif key == ord('4'):
            stop_drive(ser)
            send_packet(ser, BR, SPEED, 0x00)
            stdscr.addstr(10, 0, "Testing BR Motor     ")

        # -------- ACTUATORS --------
        elif key == ord('p'):
            send_packet(ser, AL, SPEED, 0x00)
            send_packet(ser, AR, SPEED, 0x00)
            stdscr.addstr(10, 0, "Actuators Forward    ")

        elif key == ord('l'):
            send_packet(ser, AL, SPEED, 0x01)
            send_packet(ser, AR, SPEED, 0x01)
            stdscr.addstr(10, 0, "Actuators Reverse    ")

        elif key == ord('o'):
            send_packet(ser, AL, SPEED, 0x00)
            stdscr.addstr(10, 0, "Left Actuator Fwd    ")

        elif key == ord('k'):
            send_packet(ser, AL, SPEED, 0x01)
            stdscr.addstr(10, 0, "Left Actuator Rev    ")

        elif key == ord('i'):
            send_packet(ser, AR, SPEED, 0x00)
            stdscr.addstr(10, 0, "Right Actuator Fwd   ")

        elif key == ord('j'):
            send_packet(ser, AR, SPEED, 0x01)
            stdscr.addstr(10, 0, "Right Actuator Rev   ")

        # -------- KILL --------
        elif key == ord(' '):
            send_packet(ser, KILL, 0, 0)
            stdscr.addstr(10, 0, "KILL                 ")

        else:
            stop_drive(ser)
            stdscr.addstr(10, 0, "Idle                 ")

        stdscr.refresh()

    stop_drive(ser)
    ser.close()


if __name__ == "__main__":
    curses.wrapper(main)

