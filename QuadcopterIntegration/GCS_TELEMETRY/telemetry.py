from dronekit import connect
import argparse
from QuadcopterIntegration.Utilities.dronekit_commands import *
import time
import serial
import pickle
import threading
from threading import Thread

class readThread(Thread):
    def __init__(self, serial_port):
        Thread.__init__(self)
        self.serial_port = serial_port
        self.daemon = True
        self.start()
    def run(self):
        while True:
            bytesToRead = self.serial_port.inWaiting()
            if bytesToRead > 0:
                msg = self.serial_port.read(bytesToRead)
                print(msg)
                telemetry = pickle.loads(msg)
                print(telemetry)


class sendThread(threading.Thread):
    def __init__(self, serial_port):
        Thread.__init__(self)
        self.serial_port = serial_port
        self.daemon = True
        self.start()
    def run(self):
        while True:
            command = "arm\n".encode()
            self.serial_port.write(command)
            time.sleep(1)


uav = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)

if __name__ == "__main__":
    read = readThread(uav)
    send = sendThread(uav)
    while True:
        pass