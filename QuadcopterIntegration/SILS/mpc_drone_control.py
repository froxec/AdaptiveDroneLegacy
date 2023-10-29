from dronekit import connect
from QuadcopterIntegration.Utilities.dronekit_commands import *
from Factories.ModelsFactory.model_parameters import arducopter_parameters
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SinglePoint, SpiralTrajectory
from Factories.ModelsFactory.linear_models import AugmentedLinearizedQuadNoYaw, LinearizedQuadNoYaw
from QuadcopterIntegration.SILS.simulation_parameters import *
from Factories.ControllersFactory.position_controllers.position_controller import PositionController
from Factories.ControllersFactory.position_controllers.mpc import ModelPredictiveControl
from Factories.ToolsFactory.Converters import MPC_input_converter, MPC_output_converter
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

prediction_model = LinearizedQuadNoYaw(parameters, Ts=1 / OUTER_LOOP_FREQ)
mpc = ModelPredictiveControl(prediction_model, OUTER_LOOP_FREQ, pred_horizon=10)
input_converter = MPC_input_converter(np.zeros(6), np.array([parameters['m'] * parameters['g'], 0, 0]))
output_converter = MPC_output_converter(np.array([parameters['m'] * parameters['g'], 0, 0]), parameters['Kt'],
                                        ANGULAR_VELOCITY_RANGE)
position_controller = PositionController(mpc,
                                         input_converter,
                                         output_converter,
                                         trajectory)
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='localhost:8000')
    args = parser.parse_args()
    print('Connecting to vehicle on: %s' % args.connect)
    vehicle = connect(args.connect, baud=57600, wait_ready=True)

    arm_and_takeoff(vehicle, 20)
    print("Take off complete")

    while True:
        x = np.array(get_state(vehicle))
        u = run_controller(position_controller, 1/OUTER_LOOP_FREQ, x=x)
        print("MPC control", u)
        u = mpc_command_convert(u, 0, 2*parameters['m']*parameters['g'])
        print("Current state:", x)
        print("Control processed", u)
        set_attitude(vehicle, u[1], u[2], 0, u[0])