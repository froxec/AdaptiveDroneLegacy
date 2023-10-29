from dronekit import connect
import argparse
import time

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='localhost:8500')
    args = parser.parse_args()
    print('Connecting to vehicle on: %s' % args.connect)
    vehicle = connect(args.connect, baud=921600, wait_ready=True)
    print('Connected to vehicle!')
    vehicle.arm()
    time.sleep(1)
    vehicle.disarm()