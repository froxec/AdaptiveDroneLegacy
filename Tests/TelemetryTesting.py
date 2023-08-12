import time

from Factories.CommunicationFactory.Telemetry.telemetry_manager import TelemetryManager
from dronekit import connect
import argparse

if __name__ == "__main__":
    tm = TelemetryManager(serialport='/dev/ttyUSB1',
                          baudrate=115200,
                          lora_address=2,
                          lora_freq=868,
                          remote_lora_address=1,
                          remote_lora_freq=868)
    while True:
        tm.update()
        time.sleep(0.1)


