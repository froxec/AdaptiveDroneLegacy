from dronekit import connect
from QuadcopterIntegration.Utilities.dronekit_commands import *
from Factories.ModelsFactory.model_parameters import arducopter_parameters
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SinglePoint, SpiralTrajectory
from Factories.ModelsFactory.linear_models import AugmentedLinearizedQuadNoYaw, LinearizedQuadNoYaw
from QuadcopterIntegration.SILS.simulation_parameters import *
from Factories.ControllersFactory.position_controllers.mpc import ModelPredictiveControl
from Factories.ConfigurationsFactory.configurations import QuadConfiguration, CustomMPCConfig
from Factories.ConfigurationsFactory.modes import MPCModes
from Factories.ControllersFactory.adaptive_augmentation.l1_augmentation import *
from Factories.ModelsFactory.uncertain_models import QuadTranslationalDynamicsUncertain
from QuadcopterIntegration.Utilities import dronekit_commands
from Factories.ToolsFactory.GeneralTools import time_control
import argparse
import numpy as np

def mpc_command_convert(u, thrust_min, thrust_max):
    thrust = u[0]
    u = -u
    if thrust > 1:
        thrust_converted = 1
    else:
        thrust_converted = thrust
    u[0] = thrust_converted
    return u
@time_control
def run_controller(controller, x=None):
    if x is None:
        controller()
        return
    else:
        u = controller(x)
        return u

trajectory = SinglePoint([0, 100, 20])
parameters = arducopter_parameters

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='localhost:8000')
    args = parser.parse_args()
    print('Connecting to vehicle on: %s' % args.connect)
    vehicle = connect(args.connect, baud=57600, wait_ready=True)


    ## model predictive controller
    prediction_model = LinearizedQuadNoYaw(parameters, Ts = 1 / OUTER_LOOP_FREQ)
    mpc = ModelPredictiveControl(prediction_model, OUTER_LOOP_FREQ, pred_horizon=10)
    controller_conf = CustomMPCConfig(prediction_model, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE,
                                      PWM_RANGE, horizon=10)
    controller_conf.position_controller.switch_modes(MPCModes.UNCONSTRAINED)

    x0 = np.array(dronekit_commands.get_state(vehicle))
    ## adaptive controller
    z0 = x0[3:6]
    As = np.diag([-5, -5, -5])
    uncertain_model = QuadTranslationalDynamicsUncertain(parameters)
    l1_predictor = L1_Predictor(uncertain_model, z0, 1 / INNER_LOOP_FREQ, As)
    l1_adaptive_law = L1_AdaptiveLaw(uncertain_model, 1 / INNER_LOOP_FREQ, As)
    l1_filter = L1_LowPass(bandwidths=[0.2, 0.2, 0.2], fs=INNER_LOOP_FREQ, signals_num=z0.shape[0], no_filtering=False)
    l1_converter = L1_ControlConverter()
    adaptive_controller = L1_Augmentation(l1_predictor, l1_adaptive_law, l1_filter, l1_converter)

    arm_and_takeoff(vehicle, 20)
    print("Take off complete")

    while True:
        x = np.array(dronekit_commands.get_state(vehicle))
        u = run_controller(position_controller, 1 / OUTER_LOOP_FREQ, x=x)
        print("MPC control", u)
        u = mpc_command_convert(u, 0, 2*parameters['m']*parameters['g'])
        print("Current state:", x)
        print("Control processed", u)
        set_attitude(vehicle, u[1], u[2], 0, u[0])