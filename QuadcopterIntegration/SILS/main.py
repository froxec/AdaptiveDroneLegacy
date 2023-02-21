from dronekit import connect

if __name__ == '__main__':
    vehicle = connect('tcp:192.168.0.12:8100', wait_ready=True)
    while True:
        print(vehicle.armed)
        vehicle.arm(wait=True)

#mavproxy.py --out 127.0.0.1:14550 --out 127.0.0.1:14551 --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out udp:192.168.0.27:8000
