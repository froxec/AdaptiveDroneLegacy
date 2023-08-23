from Factories.DataManagementFactory.OPC.opc_objects import TelemetryManagerClient
from Multiprocessing.PARAMS import OPC_SERVER_ADDRESS
from dronekit import connect
from Factories.CommunicationFactory.Telemetry.telemetry_manager import TelemetryManagerUAV
from Factories.CommunicationFactory.Telemetry.subscriptions import UAV_TELEMETRY_AGENT_SUBS, UAV_COMMAND_AGENT_SUBS
from oclock.timer import Timer

if __name__ == "__main__":
    # set params
    FREQUENCY = 10
    DELTA_T = 1/FREQUENCY

    # connect to drone
    drone_addr = "localhost:8008"
    print("Connecting to drone {}".format(drone_addr))
    vehicle = connect(drone_addr, baud=921600, wait_ready=True, rate=100)
    print("Connection established!")

    client = TelemetryManagerClient(OPC_SERVER_ADDRESS)

    # setup telemetry managers
    tm = TelemetryManagerUAV(serialport='/dev/pts/6',
                                            baudrate=115200,
                                            vehicle=vehicle,
                                            opc_client=client,
                                            subscribed_comms=UAV_TELEMETRY_AGENT_SUBS,
                                            send_telemetry=True,
                                            lora_address = 2,
                                            lora_freq = 868,
                                            remote_lora_address = 40,
                                            remote_lora_freq = 868)
    tm_commands = TelemetryManagerUAV(serialport='/dev/pts/6',
                                        baudrate=115200,
                                        vehicle=vehicle,
                                        opc_client=client,
                                        subscribed_comms=UAV_COMMAND_AGENT_SUBS,
                                        send_telemetry=False,
                                        lora_address=1,
                                        lora_freq=880)

    # init Timer
    timer = Timer(interval=DELTA_T)

    while True:
        tm.run()
        tm_commands.run()
        # control time
        timer.checkpt()