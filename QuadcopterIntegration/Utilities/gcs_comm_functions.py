import pickle
import threading
import time
from threading import Thread

class readThread(Thread):
    def __init__(self, serial_port):
        Thread.__init__(self)
        self.serial_port = serial_port
        self.telemetry_set_event = threading.Event()
        self.telemetry = None
        self.daemon = True
        self.start()
    def run(self):
        while True:
            bytesToRead = self.serial_port.inWaiting()
            if bytesToRead > 0:
                msg = self.serial_port.read(bytesToRead)
                self.telemetry = pickle.loads(msg)
                self.telemetry_set_event.set()


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
