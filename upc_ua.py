import time
from Factories.CommunicationFactory.Telemetry.telemetry_manager import MQTT_TelemetryManager
from Factories.CommunicationFactory.Telemetry.subscriptions import UAV_TELEMETRY_AGENT_SUBS, UAV_COMMAND_AGENT_SUBS
from Factories.DataManagementFactory.data_writer import DataWriterThread
from Factories.DataManagementFactory.DataWriterConfigurations.online_writer_configuration import DATA_TO_WRITE_PI
from Multiprocessing.PARAMS import DATA_FREQ, SIM_IP, REAL_DRONE_IP, MQTT_HOST, MQTT_PORT
from dronekit import connect
from Multiprocessing.process_interfaces import Supervisor_Interface
from QuadcopterIntegration.Utilities import dronekit_commands
from oclock import Timer
from Factories.ToolsFactory.GeneralTools import LowPassLiveFilter
from Factories.IdentificationProceduresFactory.throttle_to_thrust import ThrottleToThrustIdentification
import numpy as np
from gpiozero import Buzzer
from Factories.SoundFactory.buzzing_signals import startup_signal, vehicle_connected_signal
from copy import deepcopy
def calculate_velocity(x, x_prev, dt):
    velocity = (x - x_prev) / dt
    return list(velocity)


if __name__ == "__main__":
    buzzer = Buzzer(23)
    # set params
    FREQUENCY = DATA_FREQ
    DELTA_T = 1/FREQUENCY

    #startup_signal(buzzer)
    time.sleep(2)
    drone_addr = SIM_IP
    print("Connecting to drone {}".format(drone_addr))
    vehicle = connect(drone_addr, baud=921600, wait_ready=True, rate=DATA_FREQ)
    print("Connection established!")
    #vehicle_connected_signal(buzzer)
    #init vehicle
    dronekit_commands.initialize_drone(vehicle)

    # init db interface
    db_interface = Supervisor_Interface(vehicle)

    # setup data writer
    data_writer = DataWriterThread(DATA_TO_WRITE_PI, path='/home/pi/AdaptiveDrone/logs/')

    # setup telemetry managers
    tm = MQTT_TelemetryManager(mqtt_host=MQTT_HOST,
                               mqtt_port=MQTT_PORT,
                             update_freq=5,
                             vehicle=vehicle,
                             db_interface=db_interface,
                             data_writer=data_writer,
                             subscribed_comms='ALL',  # subscribed_comms=UAV_TELEMETRY_AGENT_SUBS,
                             send_telemetry=True)
    tm_commands = MQTT_TelemetryManager(mqtt_host=MQTT_HOST,
                                       mqtt_port=MQTT_PORT,
                                      update_freq=10,
                                      vehicle=vehicle,
                                      db_interface=db_interface,
                                      data_writer=data_writer,
                                      subscribed_comms=UAV_COMMAND_AGENT_SUBS)

    # init throttle to thrust identification
    identification_procedure = ThrottleToThrustIdentification(db_interface,
                                                              vehicle,
                                                              logs_path='/home/pi/AdaptiveDrone/identification_logs/')


    # init velocity filter
    #velocity_filter = LowPassLiveFilter([5, 5, 5], fs=FREQUENCY, signals_num=3, filter_order=1)

    #init timer
    timer = Timer(interval=DELTA_T)
    x_prev = dronekit_commands.get_state(vehicle)
    while True:
        t1 = time.time()
        # fetch db
        db_interface.fetch_db()

        # update current state
        x = dronekit_commands.get_state(vehicle)
        # if None not in x:
        #     x[3:6] = calculate_velocity(np.array(x[0:3]), np.array(x_prev[0:3]), dt=1/FREQUENCY)
        #print(x)
        #x[3:6] = velocity_filter(x[3:6])
        db_interface.set_drone_state(x)

        identification_running = identification_procedure.run(x)
        if identification_running:
            # update db state
            db_interface.update_db()
            continue
        # set vehicle control
        u = db_interface.get_control()
        if u is not None and vehicle.armed == True \
        and vehicle.location.global_relative_frame.alt > 0.95 * 1 \
        and db_interface.is_mpc_running():
            dronekit_commands.set_attitude(vehicle, u[1], u[2], 0.0, u[0])

        # update db state
        db_interface.update_db()

        # update data writer
        if data_writer.writing_event.is_set():
            if tm.telemetry is not None:
                data_writer.data = deepcopy(tm.telemetry)
                data_writer.data_set.set()

        x_prev = x

        timer.checkpt()
        #print(time.time() - t1)
