import time

from Factories.DataManagementFactory.OPC.opc_server import DroneDataServer
from QuadcopterIntegration.Utilities import dronekit_commands
from Multiprocessing.PARAMS import OPC_SERVER_ADDRESS
from dronekit import connect

if __name__ == "__main__":
    # set params
    FREQUENCY = 100

    # connect to drone
    drone_addr = "localhost:8000"
    print("Connecting to drone {}".format(drone_addr))
    vehicle = connect(drone_addr, baud=921600, wait_ready=True, rate=100)
    print("Connection established!")

    # create opc server
    opc_addr = OPC_SERVER_ADDRESS
    server = DroneDataServer(opc_addr)


    while True:
        state = dronekit_commands.get_state(vehicle)
        for i, x in enumerate(state):
            server.state_vars[i].set_value(x)
            time.sleep(1 / FREQUENCY)