import time
import serial

if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=0.05)
    while True:
        ser.write('halo'.encode())
        time.sleep(1)