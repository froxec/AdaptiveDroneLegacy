from Factories.CommunicationFactory.Telemetry.mappings import COMMANDS_ASCII_MAPPING, SUBSCRIPTIONS_MAPPING,\
COMMANDS_TO_TELEMETRY_NAMES
import time

from Factories.CommunicationFactory.Telemetry.telemetry_manager import TelemetryManager
from dronekit import connect
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='localhost:8000')
    args = parser.parse_args()
    vehicle = connect(args.connect, baud=921600, wait_ready=True, rate=100)
    tm = TelemetryManager(serialport='/dev/ttyUSB0',
                          baudrate=115200,
                          comm_ascii_map=COMMANDS_ASCII_MAPPING,
                          subscriptions_mapping=SUBSCRIPTIONS_MAPPING,
                          commands_to_telemetry_names=COMMANDS_TO_TELEMETRY_NAMES,
                          vehicle=vehicle)


    while True:
        tm.update()
        time.sleep(0.01)


