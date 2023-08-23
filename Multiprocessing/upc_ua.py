import time

from Factories.DataManagementFactory.OPC.opc_objects import DroneDataServer, DataAcquisition
from Multiprocessing.PARAMS import OPC_SERVER_ADDRESS
from dronekit import connect
from QuadcopterIntegration.Utilities import dronekit_commands
from oclock import Timer

if __name__ == "__main__":
    # set params
    FREQUENCY = 100
    DELTA_T = 1/FREQUENCY

    # connect to drone
    drone_addr = "localhost:8000"
    print("Connecting to drone {}".format(drone_addr))
    vehicle = connect(drone_addr, baud=921600, wait_ready=True, rate=100)
    print("Connection established!")

    # create opc server
    opc_addr = OPC_SERVER_ADDRESS
    server = DroneDataServer(opc_addr)

    # create data acquisition object
    da = DataAcquisition(vehicle, opc_addr)

    #init timer
    timer = Timer(interval=DELTA_T)

    while True:
        # update current state
        da.update_state()

        # set vehicle control
        u = da.get_control()
        if u is not None:
            dronekit_commands.set_attitude(vehicle, u[1], u[2], 0, u[0])
        timer.checkpt()