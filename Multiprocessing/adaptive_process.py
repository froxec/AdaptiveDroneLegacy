from Factories.DataManagementFactory.OPC.opc_objects import AdaptiveClient
from Multiprocessing.PARAMS import OPC_SERVER_ADDRESS, BANDWIDTHS, As, ANGULAR_VELOCITY_RANGE, \
    TRAJECTORY, PREDICTOR_PARAMETERS
from Factories.ModelsFactory.uncertain_models import LinearQuadUncertain
from Factories.DataManagementFactory.data_holders import DataHolder
from Factories.ControllersFactory.adaptive_augmentation.l1_augmentation import L1_Predictor, L1_AdaptiveLaw, L1_LowPass, \
    L1_ControlConverter, L1_Augmentation
from Factories.ToolsFactory.Converters import MPC_output_converter
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
    FREQ = 50
    DELTA_T = 1 / FREQ

    # init database client
    adaptive_client = AdaptiveClient(OPC_SERVER_ADDRESS)

    # init adaptive controller
    x0 = adaptive_client.get_current_state()
    if None in x0:
        x0 = np.zeros(6)
    z0 = x0[3:6]
    u0 = np.zeros(3)
    parameters_holder = DataHolder(PREDICTOR_PARAMETERS)
    uncertain_model = LinearQuadUncertain(parameters_holder)
    l1_predictor = L1_Predictor(uncertain_model, z0, 1 / FREQ, As)
    l1_adaptive_law = L1_AdaptiveLaw(uncertain_model, 1 / FREQ, As)
    l1_filter = L1_LowPass(bandwidths=BANDWIDTHS, fs=FREQ, signals_num=z0.shape[0], no_filtering=False)
    l1_converter = L1_ControlConverter()
    adaptive_controller = L1_Augmentation(l1_predictor, l1_adaptive_law, l1_filter, l1_converter, saturator=None)

    # init output converter
    output_converter = MPC_output_converter(parameters_holder, ANGULAR_VELOCITY_RANGE)

    # init Timer
    timer = Timer(interval=DELTA_T)
    z, z_prev, u, u_prev = z0, z0, u0, u0

    #turn on adaptive controller
    adaptive_client.adaptive_running_node.set_value(False)

    while True:
        t1 = time.time()
        # check if adaptive controller is running
        if adaptive_client.adaptive_running_node.get_value():
            # calculate adaptive control
            ref = adaptive_client.ref_node.get_value()
            u = adaptive_controller(z, z_prev, u, u_prev)
            z_prev = z
            u_prev = u
        else:
            # pass mpc control
            u = adaptive_client.ref_node.get_value()

        # process thrust to throttle
        if u is not None:
            # process thrust to throttle
            u = output_converter.convert_throttle(np.array(u).astype(float))
            # convert command
            u = command_convert(u)
            # set output
            adaptive_client.set_control(u)

        timer.checkpt()