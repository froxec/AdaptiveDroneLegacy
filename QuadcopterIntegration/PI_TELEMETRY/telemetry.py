from dronekit import connect
import argparse
from QuadcopterIntegration.Utilities.dronekit_commands import *
import time
import serial
import pickle

telemetry = {
    'position_local': None,
    'velocity': None,
    'armed': None
}

gcs = serial.Serial(port='/dev/ttyS0', baudrate=57600)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='/dev/ttyAMA1')
    args = parser.parse_args()
    print('Connecting to vehicle on: %s' % args.connect)
    vehicle = connect(args.connect, baud=921600, wait_ready=False)
    update_telemetry(telemetry,vehicle)
    serialized_telemetry = pickle.dumps(telemetry)
    gcs.write(serialized_telemetry)
    