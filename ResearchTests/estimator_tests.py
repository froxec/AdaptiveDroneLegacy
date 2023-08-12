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
from Factories.ToolsFactory.Converters import RampSaturationWithManager
from copy import deepcopy
from tqdm import tqdm

# TESTING OPTIONS
NORMALIZE = True
MODEL = 0 # 0 - linearized, 1 - translational dynamics, #2 hybrid
USE_ADAPTIVE = False
USE_ESTIMATOR = True
ESTIMATOR_MODE = 'VELOCITY_CONTROL' #only available
MPC_MODE = MPCModes.CONSTRAINED
HORIZON = 20
TRAJECTORY_TYPE = 0 #0 - equilibrium point
MASS_RANGE = (0.4,  2.0)
EXT_DIST_ID = 0 #0-no disturbance, 1-constant force, 2 - random_additive_wind, 3 - RandomWalkWind, 4- sinusoidal_wind
WIND_STRENGTH_RANGE = (0, 5)
WIND_NOISE_SCALE_RANGE = (0.1, 2)
WIND_DIRECTION_NOISE_SCALE = (0.1, 2)
SINUSOIDAL_WIND_FREQ_RANGE = (0.1, 10)

# AGENT OPTIONS
AGENT_SAMPLES_NUM = 100
AGENT_HANDLING_METHOD = 'IMPORTANCE'
AGENT_NOISE = 0.5
AGENT_KERNEL_LENGTH = 0.1
AGENT_MAX_STEPS = 150
TESTING_POINTS_NUM = 100 ## estimator testing points
CONVERGENCE_N = 30
CONVERGENCE_EPSILON = 0.1

ACCELERATION_NOISE = [1, 1, 1]
NUM_TESTS = 5

# PARAMETERS
INNER_LOOP_FREQ = 100
deltaT = 1 / INNER_LOOP_FREQ
OUTER_LOOP_FREQ = 10
MODULO_FACTOR = int(INNER_LOOP_FREQ/OUTER_LOOP_FREQ)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
parameters = deepcopy(Z550_parameters)

if __name__ == "__main__":
    for i in tqdm(range(NUM_TESTS)):
        quad_mass = np.random.uniform(low=MASS_RANGE[0], high=MASS_RANGE[1], size=1)[0]
        mass_perturbed = np.random.uniform(low=MASS_RANGE[0], high=MASS_RANGE[1], size=1)[0]
        parameters['m'] = quad_mass
        perturber = ParametersPerturber(parameters)
        perturber({'m': mass_perturbed - quad_mass})

        ## parameters holder
        parameters_holder = DataHolder(perturber.perturbed_parameters)

        ##External Disturbances
        strength = np.random.uniform(low = WIND_STRENGTH_RANGE[0], high=WIND_STRENGTH_RANGE[1], size=1)[0]
        direction_vector = np.random.rand(3)
        direction_vector = direction_vector / np.linalg.norm(direction_vector)
        scale = np.random.uniform(low=WIND_NOISE_SCALE_RANGE[0], high=WIND_NOISE_SCALE_RANGE[1])
        dir_scale = np.random.uniform(low=WIND_DIRECTION_NOISE_SCALE[0], high=WIND_DIRECTION_NOISE_SCALE[1])
        if EXT_DIST_ID == 0:
            wind_force = None
        elif EXT_DIST_ID == 1:
            wind_force = WindModel(direction_vector=direction_vector, strength=strength)
        elif EXT_DIST_ID == 2:
            wind_force = RandomAdditiveNoiseWind(direction_vector=direction_vector, strength=strength, scale=scale)
        elif EXT_DIST_ID == 3:
            wind_force = RandomWalkWind(direction_vector=direction_vector, strength=strength, dir_vec_scale=dir_scale, strength_scale=scale, weight=0.01)
        elif EXT_DIST_ID == 4:
            wind_force = SinusoidalWind(SINUSOIDAL_WIND_FREQ_RANGE, INNER_LOOP_FREQ, direction_vector=direction_vector, max_strength=strength)

        ## Model configuration
        if TRAJECTORY_TYPE == 0:
            x0 = np.zeros(12)
            trajectory = SinglePoint([0, 0, 0])
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

        ## Adaptive Controller configuration
        z0 = x0[3:6]
        if MODEL == 0:
            As = np.diag([-5, -5, -5])
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
        l1_predictor = L1_Predictor(uncertain_model, z0, 1 / INNER_LOOP_FREQ, As)
        l1_adaptive_law = L1_AdaptiveLaw(uncertain_model, 1 / INNER_LOOP_FREQ, As)
        l1_filter = L1_LowPass(bandwidths=bandwidths, fs=INNER_LOOP_FREQ, signals_num=z0.shape[0], no_filtering=False)
        l1_converter = L1_ControlConverter()
        l1_saturator = L1_ControlSaturator(
            lower_bounds=[-parameters_holder.m * parameters_holder.g, -np.pi / 5, -np.pi / 5],
            upper_bounds=[parameters_holder.m * parameters_holder.g, np.pi / 5, np.pi / 5])
        if USE_ADAPTIVE:
            adaptive_controller = L1_Augmentation(l1_predictor, l1_adaptive_law, l1_filter, l1_converter, l1_saturator)
        else:
            adaptive_controller = None

        ramp_saturation_slope = {'lower_bound': np.array([-np.Inf, -0.78, -0.78]),
                                 'upper_bound': np.array([2, 0.78, 0.78])}
        ramp_saturation = RampSaturationWithManager(slope=ramp_saturation_slope, Ts=1 / OUTER_LOOP_FREQ,
                                                    output_saturation=l1_saturator)
        position_controller = PositionController(controller_conf.position_controller,
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
        domain = (MASS_RANGE[0], MASS_RANGE[1])
        X0 = np.linspace(domain[0], domain[1], TESTING_POINTS_NUM).reshape(-1, 1)
        rbf_kernel = RBF_Kernel(length=AGENT_KERNEL_LENGTH)
        gp = EfficientGaussianProcess(X0, rbf_kernel, noise_std=AGENT_NOISE, max_samples=AGENT_SAMPLES_NUM,
                                      overflow_handling_mode=AGENT_HANDLING_METHOD)
        estimator_prediction_model = NonlinearTranslationalModel(parameters_holder)
        convergence_checker = ConvergenceChecker(CONVERGENCE_N, CONVERGENCE_EPSILON)
        if USE_ESTIMATOR:
            estimator_agent = BanditEstimatorAcceleration(parameters_manager=parameters_manager,
                                                          prediction_model=estimator_prediction_model,
                                                          gp=gp,
                                                          convergence_checker=convergence_checker,
                                                          pen_moving_window=None,
                                                          variance_threshold=0,
                                                          epsilon_episode_steps=0,
                                                          mode=ESTIMATOR_MODE,
                                                          max_steps=AGENT_MAX_STEPS,
                                                          testing_mode=True,
                                                          save_images=False)
        else:
            estimator_agent = None

        simulator = SoftwareInTheLoop(quad_conf.quadcopter, trajectory, position_controller,
                                      controller_conf.attitude_controller, quad_conf.esc,
                                      INNER_LOOP_FREQ, OUTER_LOOP_FREQ, adaptive_controller=adaptive_controller,
                                      estimator=estimator_agent, acceleration_noise=ACCELERATION_NOISE)
        t, x = simulator.run(30, deltaT, x0[0:12], u0, trajectory)