from dronekit import connect
import argparse
from QuadcopterIntegration.Utilities.dronekit_commands import *
import time
import serial
import pickle
from QuadcopterIntegration.Utilities.comm_definitions import telemetry
import threading
from threading import Thread
import os

class readThread(Thread):
    def __init__(self, serial_port, vehicle):
        Thread.__init__(self)
        self.serial_port = serial_port
        self.vehicle = vehicle
        self.interpreter = GCSCommInterpreter(vehicle)
        self.daemon = True
        self.start()
    def run(self):
        while True:
            msg = self.serial_port.readline(timeout=None)
            command = msg.decode()
            command, _ = command.split('\n')
            command = command.split(':')
            if len(command) > 1:
                value = command[1]
                command = command[0]
                self.interpreter(command, value)
            else:
                command = command[0]
                self.interpreter(command)
            

class sendThread(Thread):
    def __init__(self, serial_port, vehicle, freq=2):
        Thread.__init__(self)
        self.serial_port = serial_port
        self.vehicle = vehicle
        self.daemon = True
        self.freq = freq
        self.start()
    def run(self):
        while True:
            update_telemetry(telemetry, self.vehicle)
            serialized_telemetry = pickle.dumps(telemetry)
            gcs.write(serialized_telemetry)
            time.sleep(1/self.freq)


gcs = serial.Serial(port='/dev/pts/5', baudrate=115200)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='localhost:8000')
    args = parser.parse_args()
    print('Connecting to vehicle on: %s' % args.connect)
    vehicle = connect(args.connect, baud=921600, wait_ready=False)
    read = readThread(gcs, vehicle)
    send = sendThread(gcs, vehicle)
    while True:
        pass