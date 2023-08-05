from Factories.ModelsFactory.model_parameters import arducopter_parameters, Z550_parameters
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SinglePoint, SpiralTrajectory
from Factories.ModelsFactory.linear_models import AugmentedLinearizedQuadNoYaw, LinearizedQuadNoYaw, LinearTranslationalMotionDynamics
from QuadcopterIntegration.SILS.simulation_parameters import *
from Factories.ControllersFactory.position_controllers.mpc import ModelPredictiveControl
from Factories.ConfigurationsFactory.configurations import QuadConfiguration, CustomMPCConfig
from Factories.ConfigurationsFactory.modes import MPCModes
from Factories.ControllersFactory.adaptive_augmentation.l1_augmentation import *
from Factories.ModelsFactory.uncertain_models import QuadTranslationalDynamicsUncertain, LinearQuadUncertain
from Factories.ControllersFactory.position_controllers.position_controller import PositionControllerThread
from Factories.ControllersFactory.control_tools.ControlSupervisor import ControlSupervisor
from QuadcopterIntegration.Utilities import dronekit_commands
from Factories.ToolsFactory.GeneralTools import time_control
from QuadcopterIntegration.PI_TELEMETRY.telemetry import *
from Factories.CommunicationFactory.Telemetry.telemetry_manager import TelemetryManager, TelemetryManagerThread, \
    TelemetryManagerThreadUAV
import serial
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


#TESTING OPTIONS
NORMALIZE = True
MODEL = 0 # 0 - linearized, 1 - translational dynamics, #2 hybrid
USE_ADAPTIVE = True
MPC_MODE = MPCModes.CONSTRAINED
HORIZON = 20

trajectory = SinglePoint([0, 0, 10])
parameters = Z550_parameters

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='localhost:8000')
    args = parser.parse_args()
    print('Connecting to vehicle on: %s' % args.connect)
    vehicle = connect(args.connect, baud=921600, wait_ready=True, rate=100)


    ## model predictive controller
    if MODEL == 0 or MODEL == 2:
        prediction_model = LinearizedQuadNoYaw(parameters, Ts = 1 / OUTER_LOOP_FREQ)
    if MODEL == 1:
        prediction_model = LinearTranslationalMotionDynamics(parameters, 1 / OUTER_LOOP_FREQ)
    controller_conf = CustomMPCConfig(prediction_model, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE,
                                      PWM_RANGE, horizon=HORIZON, normalize_system=NORMALIZE)
    controller_conf.position_controller.switch_modes(MPC_MODE)
    position_controller = PositionControllerThread(controller_conf.position_controller,
                           controller_conf.position_controller_input_converter,
                           controller_conf.position_controller_output_converter,
                           trajectory)

    x0 = np.array(dronekit_commands.get_state(vehicle))
    ## adaptive controller
    z0 = x0[3:6]
    if MODEL == 0:
        As = np.diag([-0.1, -0.1, -0.1])
        bandwidths = [0.1, 0.1, 0.1]
    elif MODEL == 1 or MODEL == 2:
        As = np.diag([-0.1, -0.1, -0.1])
        bandwidths = [.1, .1, .1]
    if isinstance(prediction_model, LinearizedQuadNoYaw):
        uncertain_model = LinearQuadUncertain(parameters)
    else:
        uncertain_model = QuadTranslationalDynamicsUncertain(parameters)
    if MODEL == 2:
        uncertain_model = QuadTranslationalDynamicsUncertain(parameters)
    l1_predictor = L1_Predictor(uncertain_model, z0, 1 / INNER_LOOP_FREQ, As)
    l1_adaptive_law = L1_AdaptiveLaw(uncertain_model, 1 / INNER_LOOP_FREQ, As)
    l1_filter = L1_LowPass(bandwidths=bandwidths, fs=INNER_LOOP_FREQ, signals_num=z0.shape[0], no_filtering=False)
    l1_converter = L1_ControlConverter()
    l1_saturator = L1_ControlSaturator([np.Inf, np.pi / 5, np.pi / 5])
    if USE_ADAPTIVE:
        adaptive_controller = L1_AugmentationThread(l1_predictor, l1_adaptive_law, l1_filter, l1_converter, l1_saturator)
    else:
        adaptive_controller = None
    ## control supervisor
    control_supervisor = ControlSupervisor(vehicle, position_controller, adaptive_controller)

    ## telemetry manager
    tm = TelemetryManagerThreadUAV(serialport='/dev/pts/6',
                          baudrate=115200,
                          update_freq=10,
                          vehicle=vehicle,
                          position_controller=position_controller,
                          adaptive_augmentation=adaptive_controller)

    ## ground control station connection
    # gcs = serial.Serial('/dev/pts/5', baudrate=115200, timeout=0.05)
    # read = readThread(gcs, vehicle)
    # send = sendThread(gcs, vehicle)

    initialize_drone(vehicle)

    # arm_and_takeoff(vehicle, 20)
    # print("Take off complete")

    while True:
        if vehicle.armed == True and vehicle.location.global_relative_frame.alt > 0.95 * 5.0:
            control_supervisor.supervise()
        else:
            ("Waiting for drone to reach required attitude.")
        tm.update()
        time.sleep(0.01) #this sleep guarantees that other threads are not blocked by the main thread !!IMPORTANT
