from Factories.ConfigurationsFactory.configurations import QuadConfiguration, \
    ControllerConfiguration, ControllerWithCompensatorConfiguration, \
    GekkoConfiguration, positionControllerWrapper
from Factories.RLFactory.Agents.BanditEstimatorAgent import BanditEstimatorAgent
from Factories.SimulationsFactory.SITL import SoftwareInTheLoop
from Factories.ModelsFactory.model_parameters import pendulum_parameters, Z550_parameters
import numpy as np
from plots import plotTrajectory, plotTrajectory3d
import time
from Factories.ToolsFactory.AnalysisTools import ParametersPerturber
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SpiralTrajectory, RectangularTrajectory, RectangularTrajectoryWithTerminals
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw
from Factories.GaussianProcessFactory.gaussian_process import *
from Factories.GaussianProcessFactory.kernels import *

FPS = 30
PAUSE_INCREMENT = 1e-5
INNER_LOOP_FREQ = 100
OUTER_LOOP_FREQ = 10
MODULO_FACTOR = int(INNER_LOOP_FREQ/OUTER_LOOP_FREQ)
SAMPLING_FREQ = INNER_LOOP_FREQ
STEP_TIME = 0.1
ATOMIC_TRAJ_SAMPLES_NUM = int(STEP_TIME*SAMPLING_FREQ)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
spiral_trajectory = SpiralTrajectory(15)
rectangular_trajectory = RectangularTrajectory()
rectangular_trajectory_with_terminals = RectangularTrajectoryWithTerminals()
trajectory = rectangular_trajectory_with_terminals
samples_num = 100
domain = (0.2, 4)
X0 = np.linspace(domain[0], domain[1], samples_num).reshape(-1, 1)

if __name__ == '__main__':
    quad_conf = QuadConfiguration(Z550_parameters, pendulum_parameters, np.zeros(12), np.zeros(4), PWM_RANGE,
                                  ANGULAR_VELOCITY_RANGE)

    perturber = ParametersPerturber(Z550_parameters)
    perturber({'m': 0.0})
    print(perturber.perturbed_parameters)
    controller_compensator_conf = ControllerWithCompensatorConfiguration(perturber.perturbed_parameters, position0=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                                           trajectory=trajectory,
                                           u_ss=[perturber.perturbed_parameters['m']*perturber.perturbed_parameters['g'], 0.0, 0.0],
                                           x_ss = np.array([0, 0, 0, 0, 0, 0]),
                                           prediction_model=LinearizedQuadNoYaw,
                                           INNER_LOOP_FREQ=INNER_LOOP_FREQ, OUTER_LOOP_FREQ=OUTER_LOOP_FREQ,
                                           ANGULAR_VELOCITY_RANGE=ANGULAR_VELOCITY_RANGE, PWM_RANGE=PWM_RANGE)

    position_controller_conf = positionControllerWrapper(controller_compensator_conf)
    prediction_model = LinearizedQuadNoYaw(perturber.perturbed_parameters)
    rbf_kernel = RBF_Kernel(length=2)
    gp = EfficientGaussianProcess(X0, rbf_kernel, noise_std=0.0)
    mass_estimator = BanditEstimatorAgent(position_controller_conf, prediction_model, gp, ATOMIC_TRAJ_SAMPLES_NUM, deltaT=1/INNER_LOOP_FREQ)

    simulator = SoftwareInTheLoop(quad_conf.quadcopter, quad_conf.load, trajectory, controller_compensator_conf.position_controller, controller_compensator_conf.attitude_controller,
                                  [controller_compensator_conf.position_controller_input_converter, controller_compensator_conf.position_controller_output_converter]
                                  , quad_conf.esc, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, thrust_compensator=None, estimator=mass_estimator)
    state0 = np.concatenate([quad_conf.quad0, quad_conf.load0])
    u0 = np.array([0, 0, 0])

    t, x = simulator.run(250, 1/INNER_LOOP_FREQ, state0[0:12], u0, trajectory)
    plotTrajectory3d(x, trajectory.generated_trajectory)
    mass_estimator.plot()
    plotTrajectory(t, x.transpose()[0:12], 4, 3)