from process_interfaces import Adaptive_Interface
from Multiprocessing.PARAMS import OPC_SERVER_ADDRESS, BANDWIDTHS, As, ANGULAR_VELOCITY_RANGE, \
    TRAJECTORY, PREDICTOR_PARAMETERS, MIN_ATTITUDE
from Factories.ModelsFactory.uncertain_models import LinearQuadUncertain
from Factories.DataManagementFactory.data_holders import DataHolder
from Factories.ControllersFactory.adaptive_augmentation.l1_augmentation import L1_Predictor, L1_AdaptiveLaw, L1_LowPass, \
    L1_ControlConverter, L1_Augmentation
from Factories.ToolsFactory.Converters import MPC_input_converter, MPC_output_converter
import numpy as np
from oclock import Timer
import time

def command_convert(u):
    thrust = u[0]
    # for i, angle in enumerate(u[1:], 1):
    #     if angle > np.pi/4:
    #         u[i] = np.pi/4
    #     elif angle < -np.pi/4:
    #         u[i] = -np.pi/4
    u = -u
    if thrust > 1:
        thrust_converted = 1
    elif thrust < 0:
        thrust_converted = 0
    else:
        thrust_converted = thrust
    u[0] = thrust_converted
    return u

if __name__ == "__main__":
    # parameters
    FREQ = 100
    DELTA_T = 1 / FREQ

    # init input and output converters
    x_ss = np.zeros(6)
    parameters_holder = DataHolder(PREDICTOR_PARAMETERS)
    output_converter = MPC_output_converter(parameters_holder, ANGULAR_VELOCITY_RANGE)
    input_converter = MPC_input_converter(x_ss, parameters_holder)
    # init model
    uncertain_model = LinearQuadUncertain(parameters_holder)

    # init redis interface
    db_interface = Adaptive_Interface(input_converter=input_converter,
                                      output_converter=output_converter,
                                      prediction_model=uncertain_model)

    # flush db on startup
    db_interface.redis_database.flushdb()

    db_interface.fetch_db()


    # init adaptive controller
    x0 = db_interface.get_drone_state()
    if None in x0:
        x0 = np.zeros(6)
    z0 = x0[3:6]
    u0 = np.zeros(3)
    l1_predictor = L1_Predictor(uncertain_model, z0, 1 / FREQ, As)
    l1_adaptive_law = L1_AdaptiveLaw(uncertain_model, 1 / FREQ, As)
    l1_filter = L1_LowPass(bandwidths=BANDWIDTHS, fs=FREQ, signals_num=z0.shape[0], no_filtering=False)
    l1_converter = L1_ControlConverter()
    adaptive_controller = L1_Augmentation(l1_predictor, l1_adaptive_law, l1_filter, l1_converter, saturator=None)

    # init Timer
    timer = Timer(interval=DELTA_T)
    z, z_prev, u, u_prev = z0, z0, u0, u0

    while True:
        t1 = time.time()
        # fetch db
        db_interface.fetch_db()
        # check if adaptive controller is running
        if db_interface.is_adaptive_running():
            # get reference and current state
            ref = db_interface.get_ref()
            x = db_interface.get_drone_state()
            if (None not in ref
                    and None not in x
                    and db_interface.is_vehicle_armed()
                    and x[2] > MIN_ATTITUDE):
                delta_x, delta_u = input_converter(x, ref)
                u = delta_u
                z = delta_x[3:6]
                # calculate adaptive control
                u_output = adaptive_controller(z, z_prev, u, u_prev)
                z_prev = z
                u_prev = u
                u_output = output_converter(u_output, throttle=False)
        else:
            # pass mpc control
            ref = db_interface.get_ref()
            u_output = ref
        # process thrust to throttle
        if None not in u_output:
            # save not converter output
            u_not_converter = u_output
            # process thrust to throttle
            u_output = output_converter.convert_throttle(np.array(u_output).astype(float))
            # convert command
            u_output = command_convert(u_output)
            # set output
            db_interface.set_control(u_output)
            # set additional data
            db_interface.set_sigma_hat(adaptive_controller.adaptive_law.sigma_hat)
            db_interface.set_u_l1(adaptive_controller.lp_filter.u_l1)
            db_interface.set_ref(ref)
            db_interface.set_u_output(u_not_converter)

        # update db
        db_interface.update_db()

        timer.checkpt()
        print(time.time() - t1)