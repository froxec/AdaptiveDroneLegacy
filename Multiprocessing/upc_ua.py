import time
from Factories.CommunicationFactory.Telemetry.telemetry_manager import TelemetryManagerUAVMultiprocessingThread
from Factories.CommunicationFactory.Telemetry.subscriptions import UAV_TELEMETRY_AGENT_SUBS, UAV_COMMAND_AGENT_SUBS
from Factories.DataManagementFactory.data_writer import DataWriterThread
from Factories.DataManagementFactory.DataWriterConfigurations.online_writer_configuration import DATA_TO_WRITE_PI
from Multiprocessing.PARAMS import DATA_FREQ, SIM_IP, REAL_DRONE_IP
from dronekit import connect
from Multiprocessing.process_interfaces import Supervisor_Interface
from QuadcopterIntegration.Utilities import dronekit_commands
from oclock import Timer
from Factories.ToolsFactory.GeneralTools import LowPassLiveFilter
from Factories.IdentificationProceduresFactory.throttle_to_thrust import ThrottleToThrustIdentification
from gpio import 

if __name__ == "__main__":
    # set params
    FREQUENCY = DATA_FREQ
    DELTA_T = 1/FREQUENCY
    #time.sleep(20)
    # connect to drone
    drone_addr = SIM_IP
    print("Connecting to drone {}".format(drone_addr))
    vehicle = connect(drone_addr, baud=921600, wait_ready=True, rate=DATA_FREQ)
    print("Connection established!")
    
    #init vehicle
    dronekit_commands.initialize_drone(vehicle)

    # init db interface
    db_interface = Supervisor_Interface(vehicle)

    # setup data writer
    data_writer = DataWriterThread(DATA_TO_WRITE_PI, path='./logs')

    # setup telemetry managers
    tm = TelemetryManagerUAVMultiprocessingThread(serialport='/dev/ttyS0',
                             baudrate=115200,
                             update_freq=5,
                             vehicle=vehicle,
                             db_interface=db_interface,
                             data_writer=data_writer,
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
                                      db_interface=db_interface,
                                      data_writer=data_writer,
                                      subscribed_comms=UAV_COMMAND_AGENT_SUBS,
                                      send_telemetry=False,
                                      lora_address=1,
                                      lora_freq=880)

    # init throttle to thrust identification
    identification_procedure = ThrottleToThrustIdentification(db_interface,
                                   vehicle,
                                   logs_path = './identification_logs/')


    # init velocity filter
    velocity_filter = LowPassLiveFilter([5, 5, 5], fs=FREQUENCY, signals_num=3)

    #init timer
    timer = Timer(interval=DELTA_T)

    while True:
        t1 = time.time()
        # fetch db
        db_interface.fetch_db()

        # update current state
        x = dronekit_commands.get_state(vehicle)
        x[3:6] = velocity_filter(x[3:6])
        db_interface.set_drone_state(x)

        identification_running = identification_procedure.run(x)
        if identification_running:
            # update db state
            db_interface.update_db()
            continue
        # set vehicle control
        u = db_interface.get_control()
        if u is not None and vehicle.armed == True \
        and vehicle.location.global_relative_frame.alt > 0.95 * 2.5 \
        and db_interface.is_mpc_running():
            dronekit_commands.set_attitude(vehicle, u[1], u[2], 0, u[0])

        # update db state
        db_interface.update_db()

        # update data writer
        if data_writer.writing_event.is_set():
            if tm.telemetry is not None:
                data_writer.data = tm.telemetry
                data_writer.data_set.set()

        timer.checkpt()
        #print(time.time() - t1)