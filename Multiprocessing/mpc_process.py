import numpy as np
import time
from Multiprocessing.PARAMS import OPC_SERVER_ADDRESS, ANGULAR_VELOCITY_RANGE, PWM_RANGE, NORMALIZE, HORIZON, MPC_MODE, \
    TRAJECTORY, PREDICTOR_PARAMETERS
from Factories.DataManagementFactory.OPC.opc_objects import MPCClient
from Factories.ControllersFactory.position_controllers.position_controller import PositionController
from Factories.ConfigurationsFactory.configurations import CustomMPCConfig
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw
from Factories.DataManagementFactory.data_holders import DataHolder
from oclock import Timer

if __name__ == "__main__":
    # parameters
    FREQ = 5
    DELTA_T = 1/FREQ

    # init database client
    mpc_client = MPCClient(OPC_SERVER_ADDRESS)

    # init position controler
    parameters_holder = DataHolder(PREDICTOR_PARAMETERS)
    prediction_model = LinearizedQuadNoYaw(parameters_holder, Ts=1 / FREQ)
    controller_conf = CustomMPCConfig(prediction_model, None, FREQ, ANGULAR_VELOCITY_RANGE,
                                      PWM_RANGE, horizon=HORIZON, normalize_system=NORMALIZE,
                                      MPC_IMPLEMENTATION='SPARSE')
    controller_conf.position_controller.switch_modes(MPC_MODE)
    position_controller = PositionController(controller_conf.position_controller,
                                                   controller_conf.position_controller_input_converter,
                                                   controller_conf.position_controller_output_converter,
                                                   TRAJECTORY,
                                                   ramp_saturation=None)

    # init Timer
    timer = Timer(interval=DELTA_T)

    while True:
        t1 = time.time()
        # get current state and previous control
        x = mpc_client.get_current_state()
        u_prev = mpc_client.get_previous_control()
        if None in u_prev:
            u_prev = np.zeros(3)

        # compute control
        u = position_controller(x, u_prev, convert_throttle=False)
        # update control
        mpc_client.set_control(u)

        # watchdog
        timer.checkpt()
        print(time.time() - t1)
