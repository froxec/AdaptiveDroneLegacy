import numpy as np
import time
from Multiprocessing.PARAMS import OPC_SERVER_ADDRESS, ANGULAR_VELOCITY_RANGE, PWM_RANGE, NORMALIZE, HORIZON, MPC_MODE, \
    TRAJECTORY, PREDICTOR_PARAMETERS, MIN_ATTITUDE, MPC_CONSTRAINTS
from Factories.ControllersFactory.position_controllers.position_controller import PositionController
from Factories.ConfigurationsFactory.configurations import CustomMPCConfig
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw
from Factories.DataManagementFactory.data_holders import DataHolder
from  Multiprocessing.process_interfaces import MPC_Interface
from oclock import Timer
import redis

if __name__ == "__main__":
    # parameters
    FREQ = 10
    DELTA_T = 1/FREQ

    # init position controler
    parameters_holder = DataHolder(PREDICTOR_PARAMETERS)
    prediction_model = LinearizedQuadNoYaw(parameters_holder, Ts=1 / FREQ)
    controller_conf = CustomMPCConfig(prediction_model, None, FREQ, ANGULAR_VELOCITY_RANGE,
                                      PWM_RANGE, MPC_CONSTRAINTS, horizon=HORIZON, normalize_system=NORMALIZE,
                                      MPC_IMPLEMENTATION='SPARSE', direct_thrust_to_throttle=True)
    controller_conf.position_controller.switch_modes(MPC_MODE)
    position_controller = PositionController(controller_conf.position_controller,
                                                   controller_conf.position_controller_input_converter,
                                                   controller_conf.position_controller_output_converter,
                                                   TRAJECTORY,
                                                   ramp_saturation=None)


    # create redis interface
    db_interface = MPC_Interface(position_controller)

    # flush db on startup
    db_interface.redis_database.flushdb()


    # init Timer
    timer = Timer(interval=DELTA_T)

    # init vars
    u_prev = np.array([parameters_holder.m*parameters_holder.g, 0, 0])

    while True:
        t1 = time.time()
        # get current drone state from db
        db_interface.fetch_db()
        # get current state
        x = db_interface.get_drone_state()
        # compute control
        if (x is not None
                and db_interface.is_mpc_running()
                and db_interface.is_vehicle_armed()
                and db_interface.get_vehicle_mode() == 'GUIDED'
                and x[2] > MIN_ATTITUDE):
            u = position_controller(x, u_prev, convert_throttle=False)
            # update control
            db_interface.set_control(u)
            # save previous control
            u_prev = u
            print(u)
        # update current setpoint info
        db_interface.update_setpoint(position_controller.setpoint)
        # update mpc state
        db_interface.update_db()
        # watchdog
        timer.checkpt()
        print(time.time() - t1)