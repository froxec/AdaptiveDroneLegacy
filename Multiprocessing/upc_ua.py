import time

from Factories.DataManagementFactory.OPC.opc_objects import DroneDataServer, DataAcquisition
from Factories.DataManagementFactory.OPC.opc_objects import TelemetryManagerClient
from Factories.CommunicationFactory.Telemetry.telemetry_manager import TelemetryManagerUAV
from Factories.CommunicationFactory.Telemetry.subscriptions import UAV_TELEMETRY_AGENT_SUBS, UAV_COMMAND_AGENT_SUBS
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

    # telemetry manager client
    client = TelemetryManagerClient(OPC_SERVER_ADDRESS)

    # setup telemetry managers
    tm = TelemetryManagerUAV(serialport='/dev/pts/2',
                             baudrate=115200,
                             vehicle=vehicle,
                             opc_client=client,
                             subscribed_comms='ALL',  # subscribed_comms=UAV_TELEMETRY_AGENT_SUBS,
                             send_telemetry=True,
                             lora_address=2,
                             lora_freq=868,
                             remote_lora_address=40,
                             remote_lora_freq=868)
    tm_commands = TelemetryManagerUAV(serialport='/dev/pts/2',
                                      baudrate=115200,
                                      vehicle=vehicle,
                                      opc_client=client,
                                      subscribed_comms=UAV_COMMAND_AGENT_SUBS,
                                      send_telemetry=False,
                                      lora_address=1,
                                      lora_freq=880)

    #init timer
    timer = Timer(interval=DELTA_T)

    while True:
        # update current state
        da.update_state()

        # set vehicle control
        u = da.get_control()
        if u is not None:
            print(u)
            dronekit_commands.set_attitude(vehicle, u[1], u[2], 0, u[0])

        #run telemetry managers
        tm.run()
        tm_commands.run()

        timer.checkpt()