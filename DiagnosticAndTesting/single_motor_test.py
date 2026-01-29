import serial
import time

ser = serial.Serial(
    port='/dev/ttyUSB6',
    baudrate=9600,
    timeout=1
)

# Toggle DTR
ser.dtr = False
time.sleep(0.1)
ser.dtr = True

ser.close()
