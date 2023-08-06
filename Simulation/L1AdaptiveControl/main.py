import numpy as np
from Factories.ModelsFactory.model_parameters import Z550_parameters, pendulum_parameters
from Factories.ConfigurationsFactory.configurations import QuadConfiguration, CustomMPCConfig
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw, LinearTranslationalMotionDynamics
from Factories.ConfigurationsFactory.modes import MPCModes
from Factories.SimulationsFactory.SITL import SoftwareInTheLoopLegacy, SoftwareInTheLoop
from Factories.ToolsFactory.AnalysisTools import ParametersPerturber
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SpiralTrajectory, RectangularTrajectory, SinglePoint, \
    RectangularTrajectoryWithTerminals
from Factories.ControllersFactory.adaptive_augmentation.l1_augmentation import *
from Factories.ControllersFactory.position_controllers.position_controller import PositionController
from Factories.ModelsFactory.uncertain_models import QuadTranslationalDynamicsUncertain, LinearQuadUncertain
from Factories.ModelsFactory.external_force_models import WindModel, RandomAdditiveNoiseWind, RandomWalkWind, SinusoidalWind
from Simulation.plots import plotTrajectory, plotTrajectory3d
from Factories.DataManagementFactory.data_holders import DataHolder
from Factories.DataManagementFactory.data_managers import ParametersManager
from Factories.RLFactory.Agents.BanditEstimatorAgent import BanditEstimatorAgent, BanditEstimatorAcceleration
from Factories.ModelsFactory.models_for_estimation import NonlinearTranslationalModel
from Factories.GaussianProcessFactory.kernels import RBF_Kernel
from Factories.GaussianProcessFactory.gaussian_process import EfficientGaussianProcess
from Factories.RLFactory.Agents.Tools.convergenceChecker import ConvergenceChecker

#TESTING OPTIONS
NORMALIZE = True
MODEL = 0 # 0 - linearized, 1 - translational dynamics, #2 hybrid
USE_ADAPTIVE = False
USE_ESTIMATOR = False
MPC_MODE = MPCModes.UNCONSTRAINED
HORIZON = 20

INNER_LOOP_FREQ = 100
deltaT = 1 / INNER_LOOP_FREQ
OUTER_LOOP_FREQ = 10
MODULO_FACTOR = int(INNER_LOOP_FREQ/OUTER_LOOP_FREQ)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
trajectory = SinglePoint([0, 0, 20])
if __name__ == "__main__":
    perturber = ParametersPerturber(Z550_parameters)
    perturber({'m': 0.6})

    ## parameters holder
    parameters_holder = DataHolder(perturber.perturbed_parameters)

    ##External Disturbances
    wind_force = WindModel(direction_vector=[0, 1, 0], strength=0)
    #wind_force = RandomAdditiveNoiseWind(direction_vector=[1, 1, 1], strength=1, scale=2)
    #wind_force = RandomWalkWind(direction_vector=[1, 1, 1], strength=3.0, dir_vec_scale=0.5, strength_scale=0.05, weight=0.01)
    #wind_force = SinusoidalWind(0.1, INNER_LOOP_FREQ, direction_vector=[0, 1, 0], max_strength=2)
    ## Model configuration
    x0 = np.zeros(12)
    x0[2] = 20
    quad_conf = QuadConfiguration(perturber.nominal_parameters, pendulum_parameters, x0, np.zeros(4), PWM_RANGE,
                                  ANGULAR_VELOCITY_RANGE, external_disturbance=wind_force)
    x0 = np.concatenate([quad_conf.quad0, quad_conf.load0])
    u0 = np.zeros(3)

    ## Controller configuration
    if MODEL == 0 or MODEL == 2:
        prediction_model = LinearizedQuadNoYaw(parameters_holder, 1 / OUTER_LOOP_FREQ)
    elif MODEL == 1:
        prediction_model = LinearTranslationalMotionDynamics(parameters_holder, 1 / OUTER_LOOP_FREQ)
    controller_conf = CustomMPCConfig(prediction_model, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE,
                                      PWM_RANGE, horizon=HORIZON, normalize_system=NORMALIZE)
    controller_conf.position_controller.switch_modes(MPC_MODE)
    position_controller = PositionController(controller_conf.position_controller,
                                                   controller_conf.position_controller_input_converter,
                                                   controller_conf.position_controller_output_converter,
                                                   trajectory)

    ## Adaptive Controller configuration
    z0 = x0[3:6]
    if MODEL == 0:
        As = np.diag([-0.1, -0.1, -0.1])
        bandwidths = [1, 0.2, 0.2]
    elif MODEL == 1 or MODEL == 2:
        As = np.diag([-0.1, -0.1, -0.1])
        bandwidths = [.1, .1, .1]
    if isinstance(prediction_model, LinearizedQuadNoYaw):
        uncertain_model = LinearQuadUncertain(parameters_holder)
    else:
        uncertain_model = QuadTranslationalDynamicsUncertain(parameters_holder)
    if MODEL == 2:
        uncertain_model = QuadTranslationalDynamicsUncertain(parameters_holder)
    l1_predictor = L1_Predictor(uncertain_model, z0, 1 / INNER_LOOP_FREQ, As)
    l1_adaptive_law = L1_AdaptiveLaw(uncertain_model, 1 / INNER_LOOP_FREQ, As)
    l1_filter = L1_LowPass(bandwidths=bandwidths, fs=INNER_LOOP_FREQ, signals_num=z0.shape[0], no_filtering=False)
    l1_converter = L1_ControlConverter()
    l1_saturator = L1_ControlSaturator(lower_bounds=[-parameters_holder.m*parameters_holder.g, -np.pi / 5, -np.pi / 5],
                                       upper_bounds=[parameters_holder.m*parameters_holder.g, np.pi / 5, np.pi / 5])
    if USE_ADAPTIVE:
        adaptive_controller = L1_Augmentation(l1_predictor, l1_adaptive_law, l1_filter, l1_converter, l1_saturator)
    else:
        adaptive_controller = None

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
    MASS_MIN, MASS_MAX = (0.5, 2.0)
    domain = (MASS_MIN, MASS_MAX)
    X0 = np.linspace(domain[0], domain[1], samples_num).reshape(-1, 1)
    rbf_kernel = RBF_Kernel(length=0.1)
    gp = EfficientGaussianProcess(X0, rbf_kernel, noise_std=1.0)
    estimator_prediction_model = NonlinearTranslationalModel(parameters_holder)
    convergence_checker = ConvergenceChecker(10, 0.1)
    if USE_ESTIMATOR:
        estimator_agent = BanditEstimatorAcceleration(parameters_manager=parameters_manager,
                                                      prediction_model=estimator_prediction_model,
                                                      gp=gp,
                                                      convergence_checker=convergence_checker)
    else:
        estimator_agent = None


    # Simulation
    ramp_saturation_slope = np.array([np.Inf, 0.78, 0.78])
    simulator = SoftwareInTheLoop(quad_conf.quadcopter, trajectory, position_controller,
                                  controller_conf.attitude_controller,quad_conf.esc,
                                  INNER_LOOP_FREQ, OUTER_LOOP_FREQ, adaptive_controller=adaptive_controller, estimator=estimator_agent, acceleration_noise=[0, 0, 0])
    t, x = simulator.run(30, deltaT, x0[0:12], u0, trajectory)
    simulator.quad.external_disturbance.plot_history()
    if USE_ADAPTIVE:
        simulator.adaptive_controller.plot_history('sigma_hat')
        simulator.adaptive_controller.plot_history('u_l1')
    plotTrajectory(t, x.transpose()[0:12], 4, 3)
    #plotTrajectory(t, x.transpose()[0:12], 4, 3, [1, 2, 4, 5, 7, 8, 9, 10, 11, 12])