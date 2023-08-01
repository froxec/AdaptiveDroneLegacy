import time

from Factories.CommunicationFactory.Telemetry.telemetry_manager import TelemetryManager
from dronekit import connect
import argparse

if __name__ == "__main__":
    tm = TelemetryManager(serialport='/dev/pts/7',
                          baudrate=115200)
    while True:
        tm.update()
        time.sleep(0.1)


