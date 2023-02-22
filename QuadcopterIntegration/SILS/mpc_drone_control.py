from dronekit import connect, VehicleMode, LocationGlobalRelative
from QuadcopterIntegration.Utilities.dronekit_commands import *
from pymavlink import mavutil
import time

import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='192.168.0.27:8000')
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

arm_and_takeoff(vehicle, 20)

print("Take off complete")