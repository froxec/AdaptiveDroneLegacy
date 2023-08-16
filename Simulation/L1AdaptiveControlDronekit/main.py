import os

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
from Factories.DataManagementFactory.data_holders import DataHolder
from Factories.DataManagementFactory.data_managers import ParametersManager
from Factories.GaussianProcessFactory.kernels import RBF_Kernel
from Factories.GaussianProcessFactory.gaussian_process import EfficientGaussianProcess
from Factories.RLFactory.Agents.Tools.convergenceChecker import ConvergenceChecker
from Factories.ModelsFactory.models_for_estimation import NonlinearTranslationalModel
from Factories.RLFactory.Agents.BanditEstimatorAgent import BanditEstimatorThread
from Factories.ToolsFactory.Converters import RampSaturationWithManager
from Factories.CommunicationFactory.Telemetry.subscriptions import *
from Factories.DataManagementFactory.data_writer import DataWriterThread
from Factories.DataManagementFactory.DataWriterConfigurations.online_writer_configuration import DATA_TO_WRITE_PI

import dronekit
import serial
import argparse
import numpy as np
import subprocess
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

#RUN_MAVLINK_LOGS?
RUN_MAVLINK_LOGS = True
#DRONE ADDR
DRONE_ADDR = "localhost:8000"
#TESTING OPTIONS
NORMALIZE = True
MODEL = 0 # 0 - linearized, 1 - translational dynamics, #2 hybrid
USE_ADAPTIVE = True
USE_ESTIMATOR = True
ESTIMATOR_MODE = 'VELOCITY_CONTROL'  #only available
ADAPTIVE_FREQ = 100
OUTER_LOOP_FREQ = 5
MPC_MODE = MPCModes.CONSTRAINED
HORIZON = 10
QUAD_NOMINAL_MASS = 0.5

trajectory = SinglePoint([0, 0, 10])
Z550_parameters['m'] = QUAD_NOMINAL_MASS
parameters = Z550_parameters

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='localhost:8000')
    args = parser.parse_args()
    print('Connecting to vehicle on: %s' % args.connect)
    vehicle = connect(args.connect, baud=921600, wait_ready=True, rate=100)

    ## parameters holder
    parameters_holder = DataHolder(parameters)

    ## model predictive controller
    if MODEL == 0 or MODEL == 2:
        prediction_model = LinearizedQuadNoYaw(parameters_holder, Ts = 1 / OUTER_LOOP_FREQ)
    if MODEL == 1:
        prediction_model = LinearTranslationalMotionDynamics(parameters_holder, 1 / OUTER_LOOP_FREQ)
    controller_conf = CustomMPCConfig(prediction_model, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE,
                                      PWM_RANGE, horizon=HORIZON, normalize_system=NORMALIZE, MPC_IMPLEMENTATION='SPARSE')
    controller_conf.position_controller.switch_modes(MPC_MODE)
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
        uncertain_model = LinearQuadUncertain(parameters_holder)
    else:
        uncertain_model = QuadTranslationalDynamicsUncertain(parameters_holder)
    if MODEL == 2:
        uncertain_model = QuadTranslationalDynamicsUncertain(parameters_holder)
    l1_predictor = L1_Predictor(uncertain_model, z0, 1 / ADAPTIVE_FREQ, As)
    l1_adaptive_law = L1_AdaptiveLaw(uncertain_model, 1 / ADAPTIVE_FREQ, As)
    l1_filter = L1_LowPass(bandwidths=bandwidths, fs=ADAPTIVE_FREQ, signals_num=z0.shape[0], no_filtering=False)
    l1_converter = L1_ControlConverter()
    l1_saturator = L1_ControlSaturator(lower_bounds=[-parameters_holder.m*parameters_holder.g, -np.pi / 5, -np.pi / 5],
                                       upper_bounds=[3*parameters_holder.m*parameters_holder.g, np.pi / 5, np.pi / 5])
    if USE_ADAPTIVE:
        adaptive_controller = L1_AugmentationThread(l1_predictor, l1_adaptive_law, l1_filter, l1_converter, l1_saturator)
    else:
        adaptive_controller = None

    ramp_saturation_slope = {'lower_bound': np.array([-np.Inf, -0.78, -0.78]),
                             'upper_bound': np.array([np.Inf, 0.78, 0.78])}
    ramp_saturation = RampSaturationWithManager(slope=ramp_saturation_slope, Ts=1 / OUTER_LOOP_FREQ,
                                                output_saturation=l1_saturator)
    #ramp_saturation = RampSaturation(slope=ramp_saturation_slope, Ts=1 / OUTER_LOOP_FREQ)
    position_controller = PositionControllerThread(controller_conf.position_controller,
                                                   controller_conf.position_controller_input_converter,
                                                   controller_conf.position_controller_output_converter,
                                                   trajectory,
                                                   ramp_saturation=ramp_saturation)

    ## parameters manager
    parameters_manager = ParametersManager(parameters_holder=parameters_holder,
                                           predictive_model=position_controller.controller.model,
                                           input_converter=position_controller.input_converter,
                                           output_converter=position_controller.output_converter,
                                           uncertain_predictive_model=l1_predictor.ref_model)

    ## estimation agent
    SAMPLING_FREQ = OUTER_LOOP_FREQ  # Hz
    STEP_TIME = 1  # s
    ATOMIC_TRAJ_SAMPLES_NUM = int(STEP_TIME * SAMPLING_FREQ)
    samples_num = 100
    MASS_MIN, MASS_MAX = (0.2, 2.0)
    domain = (MASS_MIN, MASS_MAX)
    X0 = np.linspace(domain[0], domain[1], samples_num).reshape(-1, 1)
    rbf_kernel = RBF_Kernel(length=0.1)
    gp = EfficientGaussianProcess(X0, rbf_kernel, noise_std=0.5, max_samples=100, overflow_handling_mode='IMPORTANCE')
    estimator_prediction_model = NonlinearTranslationalModel(parameters_holder)
    convergence_checker = ConvergenceChecker(30, 0.05)
    if USE_ESTIMATOR:
        estimator_agent = BanditEstimatorThread(parameters_manager=parameters_manager,
                                                      prediction_model=estimator_prediction_model,
                                                      gp=gp,
                                                      convergence_checker=convergence_checker,
                                                      pen_moving_window=None,
                                                      variance_threshold=0,
                                                      epsilon_episode_steps=0,
                                                      mode=ESTIMATOR_MODE)
    else:
        estimator_agent = None
    ## control supervisor
    control_supervisor = ControlSupervisor(vehicle,
                                           position_controller,
                                           adaptive_controller,
                                           estimator_agent)

    data_writer = DataWriterThread(DATA_TO_WRITE_PI, path='./logs/')

    ## telemetry manager
    tm = TelemetryManagerThreadUAV(serialport='/dev/pts/6',
                                   baudrate=115200,
                                   update_freq=5,
                                   vehicle=vehicle,
                                   position_controller=position_controller,
                                   control_supervisor=control_supervisor,
                                   adaptive_augmentation=adaptive_controller,
                                   data_writer=data_writer,
                                   subscribed_comms='ALL', #subscribed_comms=UAV_TELEMETRY_AGENT_SUBS,
                                   lora_address=2,
                                   lora_freq=868,
                                   remote_lora_address=40,
                                   remote_lora_freq=868)

    tm_commands = TelemetryManagerThreadUAV(serialport='/dev/pts/6',
                                            baudrate=115200,
                                            update_freq=10,
                                            vehicle=vehicle,
                                            position_controller=position_controller,
                                            control_supervisor=control_supervisor,
                                            adaptive_augmentation=adaptive_controller,
                                            data_writer=data_writer,
                                            subscribed_comms=UAV_COMMAND_AGENT_SUBS,
                                            send_telemetry=False,
                                            lora_address=1,
                                            lora_freq=880)

    ## ground control station connection
    # gcs = serial.Serial('/dev/pts/5', baudrate=115200, timeout=0.05)
    # read = readThread(gcs, vehicle)
    # send = sendThread(gcs, vehicle)

    initialize_drone(vehicle)

    # arm_and_takeoff(vehicle, 20)
    # print("Take off complete")

    while True:
        tm.update()
        if vehicle.armed == True and vehicle.location.global_relative_frame.alt > 0.95 * 5.0:
            control_supervisor.supervise()
        else:
            ("Waiting for drone to reach required attitude.")
        if data_writer.writing_event.is_set():
            if tm.telemetry is not None:
                data_writer.data = tm.telemetry
                data_writer.data_set.set()
        time.sleep(1/ADAPTIVE_FREQ)# this sleep guarantees that other threads are not blocked by the main thread !!IMPORTANT
