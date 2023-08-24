import time

from Factories.DataManagementFactory.OPC.opc_objects import DroneDataServer, DataAcquisition
from Factories.DataManagementFactory.OPC.opc_objects import TelemetryManagerClient
from Factories.CommunicationFactory.Telemetry.telemetry_manager import TelemetryManagerUAVMultiprocessingThread
from Factories.CommunicationFactory.Telemetry.subscriptions import UAV_TELEMETRY_AGENT_SUBS, UAV_COMMAND_AGENT_SUBS
from Multiprocessing.PARAMS import OPC_SERVER_ADDRESS, DATA_FREQ
from dronekit import connect
from QuadcopterIntegration.Utilities import dronekit_commands
from oclock import Timer

if __name__ == "__main__":
    # set params
    FREQUENCY = DATA_FREQ
    DELTA_T = 1/FREQUENCY

    # connect to drone
    drone_addr = 'udp:192.168.0.27:8500'
    print("Connecting to drone {}".format(drone_addr))
    vehicle = connect(drone_addr, baud=921600, wait_ready=True, rate=DATA_FREQ)
    print("Connection established!")

    # create opc server
    opc_addr = OPC_SERVER_ADDRESS
    server = DroneDataServer(opc_addr)

    # create data acquisition object
    da = DataAcquisition(vehicle, opc_addr)

    # telemetry manager client
    client = TelemetryManagerClient(OPC_SERVER_ADDRESS)

    # setup telemetry managers
    tm = TelemetryManagerUAVMultiprocessingThread(serialport='/dev/ttyS0',
                             baudrate=115200,
                             update_freq=5,
                             vehicle=vehicle,
                             opc_client=client,
                             subscribed_comms='ALL',  # subscribed_comms=UAV_TELEMETRY_AGENT_SUBS,
                             send_telemetry=True,
                             lora_address=2,
                             lora_freq=868,
                             remote_lora_address=40,
                             remote_lora_freq=868)
    tm_commands = TelemetryManagerUAVMultiprocessingThread(serialport='/dev/ttyUSB0',
                                      baudrate=115200,
                                      update_freq=10,
                                      vehicle=vehicle,
                                      opc_client=client,
                                      subscribed_comms=UAV_COMMAND_AGENT_SUBS,
                                      send_telemetry=False,
                                      lora_address=1,
                                      lora_freq=880)

    #init timer
    timer = Timer(interval=DELTA_T)

    while True:
        t1 = time.time()
        # update current state
        da.update_state()
        # set vehicle control
        u = da.get_control()
        if u is not None:
            dronekit_commands.set_attitude(vehicle, u[1], u[2], 0, u[0])

        timer.checkpt()
        print(time.time() - t1)