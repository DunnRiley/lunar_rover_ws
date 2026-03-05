import serial
import time

START = 0xAA
END   = 0x55

CMD_DIG   = 0xA7
CMD_DRIVE = 0xA9
CMD_DUMP  = 0xB3
CMD_STOP  = 0xB4

PORT = "COM3"       # change this
BAUD = 115200


def send_packet(ser, device, speed=0, direction=0):
    packet = bytes([START, device & 0xFF, speed & 0xFF, direction & 0xFF, END])
    ser.write(packet)


def send_and_wait_ack(ser, device, speed=0, direction=0, timeout=0.2):
    send_packet(ser, device, speed, direction)
    t0 = time.time()

    while time.time() - t0 < timeout:
        if ser.in_waiting > 0:
            b = ser.read(1)
            if b == bytes([0xAA]):
                return True
    return False


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(2)  # allow Arduino reset after opening serial

    print("Commands:")
    print("  d = DIGPOSITION")
    print("  r = DRIVEPOSITION")
    print("  b = DUMPBUCKET")
    print("  s = STOP")
    print("  q = quit")

    try:
        while True:
            cmd = input("Enter command: ").strip().lower()

            if cmd == "d":
                ok = send_and_wait_ack(ser, CMD_DIG, 0, 0)
                print("Sent DIGPOSITION", "ACK" if ok else "NO ACK")

            elif cmd == "r":
                ok = send_and_wait_ack(ser, CMD_DRIVE, 0, 0)
                print("Sent DRIVEPOSITION", "ACK" if ok else "NO ACK")

            elif cmd == "b":
                ok = send_and_wait_ack(ser, CMD_DUMP, 0, 0)
                print("Sent DUMPBUCKET", "ACK" if ok else "NO ACK")

            elif cmd == "s":
                ok = send_and_wait_ack(ser, CMD_STOP, 0, 0)
                print("Sent STOP", "ACK" if ok else "NO ACK")

            elif cmd == "q":
                break

            else:
                print("Unknown command")

    finally:
        ser.close()


if __name__ == "__main__":
    main()