from dronekit import connect

if __name__ == '__main__':
    vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True)
    while True:
        print(vehicle.armed)
        vehicle.arm(wait=True)
        