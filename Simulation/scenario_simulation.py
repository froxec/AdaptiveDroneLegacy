from Factories.ConfigurationsFactory.configurations import QuadConfiguration, positionControllerWrapper, CustomMPCConfig
from Factories.RLFactory.Agents.BanditEstimatorAgent import BanditEstimatorAgent
from Factories.SimulationsFactory.SITL import SoftwareInTheLoop
from Factories.ModelsFactory.model_parameters import pendulum_parameters, Z550_parameters
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw, AugmentedLinearizedQuadNoYaw
from Factories.ConfigurationsFactory.modes import MPCModes
import numpy as np
from plots import plotTrajectory, plotTrajectory3d
import time
from Factories.ToolsFactory.AnalysisTools import ParametersPerturber
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SpiralTrajectory, RectangularTrajectory, RectangularTrajectoryWithTerminals, SinglePoint
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw
from Factories.GaussianProcessFactory.gaussian_process import *
from Factories.GaussianProcessFactory.kernels import *
from Factories.DataManagementFactory.data_holders import DataHolder
from Factories.DataManagementFactory.data_managers import ParametersManager
from Factories.ControllersFactory.position_controllers.position_controller import PositionController

FPS = 30
PAUSE_INCREMENT = 1e-5
INNER_LOOP_FREQ = 100
OUTER_LOOP_FREQ = 10
MODULO_FACTOR = int(INNER_LOOP_FREQ/OUTER_LOOP_FREQ)
SAMPLING_FREQ = INNER_LOOP_FREQ
STEP_TIME = 1
ATOMIC_TRAJ_SAMPLES_NUM = int(STEP_TIME*SAMPLING_FREQ)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
spiral_trajectory = SpiralTrajectory(15)
rectangular_trajectory = RectangularTrajectory()
rectangular_trajectory_with_terminals = RectangularTrajectoryWithTerminals()
single_point_trajectory = SinglePoint([0, 0, 10])
trajectory = single_point_trajectory
samples_num = 100
domain = (0.2, 4)
X0 = np.linspace(domain[0], domain[1], samples_num).reshape(-1, 1)

if __name__ == '__main__':
    quad_conf = QuadConfiguration(Z550_parameters, pendulum_parameters, np.zeros(12), np.zeros(4), PWM_RANGE,
                                  ANGULAR_VELOCITY_RANGE)

    perturber = ParametersPerturber(Z550_parameters)
    perturber({'m': 0.2})
    print(perturber.perturbed_parameters)

    parameters_holder = DataHolder(perturber.perturbed_parameters)

    prediction_model = LinearizedQuadNoYaw(parameters_holder, 1 / OUTER_LOOP_FREQ)
    #prediction_model2 = AugmentedLinearizedQuadNoYaw(perturber.perturbed_parameters, 1 / OUTER_LOOP_FREQ)
    controller_conf = CustomMPCConfig(prediction_model, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE,
                                      PWM_RANGE, horizon=10)
    controller_conf.position_controller.switch_modes(MPCModes.UNCONSTRAINED)
    position_controller_conf = positionControllerWrapper(controller_conf)
    position_controller = PositionController(controller_conf.position_controller,
                                                   controller_conf.position_controller_input_converter,
                                                   controller_conf.position_controller_output_converter,
                                                   trajectory)

    ## parameters manager
    parameters_manager = ParametersManager(parameters_holder=parameters_holder,
                                           predictive_model=position_controller.controller.model,
                                           input_converter=position_controller.input_converter,
                                           output_converter=position_controller.output_converter)
    rbf_kernel = RBF_Kernel(length=1)
    gp = EfficientGaussianProcess(X0, rbf_kernel, noise_std=0.0)
    estimator_agent = BanditEstimatorAgent(parameters_manager, prediction_model, gp, deltaT=1/SAMPLING_FREQ, atomic_traj_samples_num=ATOMIC_TRAJ_SAMPLES_NUM)

    simulator = SoftwareInTheLoop(quad_conf.quadcopter, trajectory, position_controller,
                                  controller_conf.attitude_controller, quad_conf.esc,
                                  INNER_LOOP_FREQ, OUTER_LOOP_FREQ,
                                  estimator=estimator_agent)
    state0 = np.concatenate([quad_conf.quad0, quad_conf.load0])
    u0 = np.array([0, 0, 0])

    t, x = simulator.run(30, 1/INNER_LOOP_FREQ, state0[0:12], u0, trajectory)
    plotTrajectory3d(x, trajectory.generated_trajectory)
    mass_estimator.plot()
    plotTrajectory(t, x.transpose()[0:12], 4, 3)