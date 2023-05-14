from Factories.GaussianProcessFactory.kernels import  RBF_Kernel
from Factories.GaussianProcessFactory.gaussian_process import GaussianProcess, EfficientGaussianProcess
from Factories.ToolsFactory.GeneralTools import plot_signal, RollBuffers
from Factories.ToolsFactory.Converters import convert_trajectory
from Factories.ControllersFactory.position_controllers.position_controller import PositionController
from Factories.RLFactory.Environments.base_env import ControlLoopEnvironment
from Factories.ConfigurationsFactory.configurations import *
from Factories.ModelsFactory.model_parameters import Z550_parameters, pendulum_parameters
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw, AugmentedLinearizedQuadNoYaw
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import *
from Factories.ToolsFactory.AnalysisTools import ParametersPerturber
from Simulation.plots import plotTrajectory, plotTrajectory3d

import numpy as np

IMAGES_PATH = "../images/gp/"

INNER_LOOP_FREQ = 100
OUTER_LOOP_FREQ = 10
SAMPLING_FREQ = OUTER_LOOP_FREQ #Hz
STEP_TIME = 1 #s
CONTROLS_NUM = 3
STATES_NUM = 6
ATOMIC_TRAJ_SAMPLES_NUM = int(STEP_TIME*SAMPLING_FREQ)
MAX_EPISODE_TIME = 100 #s
MAX_STEPS_NUM = int(MAX_EPISODE_TIME/STEP_TIME)
QUAD_STATE0 = np.zeros(12)
LOAD_STATE0 = np.zeros(4)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
MASS_MIN = 0.2
MASS_MAX = 2

spiral_trajectory = SpiralTrajectory(15)
rectangular_trajectory = RectangularTrajectory()
trajectory = SinglePoint([0, 0, 10])

if __name__ == "__main__":
    #Estimation domain
    deltaT = STEP_TIME/OUTER_LOOP_FREQ
    samples_num = 100
    domain = (0.2, 4)
    X0 = np.linspace(domain[0], domain[1], samples_num).reshape(-1, 1)

    #Parameters perturbation
    perturber = ParametersPerturber(Z550_parameters)
    perturber({'m': 1.5})
    print(perturber.perturbed_parameters)

    #Configurations
    quad_conf = QuadConfiguration(perturber.nominal_parameters, pendulum_parameters, QUAD_STATE0, LOAD_STATE0, PWM_RANGE,
                                  ANGULAR_VELOCITY_RANGE)
    attitude_controller_conf = AttitudeControllerConfiguration(INNER_LOOP_FREQ, PWM_RANGE, PWM0=PWM_RANGE[0])
    prediction_model = LinearizedQuadNoYaw(perturber.perturbed_parameters, Ts=1 / OUTER_LOOP_FREQ)
    mpc = ModelPredictiveControl(prediction_model, OUTER_LOOP_FREQ, pred_horizon=10)
    input_converter = MPC_input_converter(np.zeros(6), np.array([perturber.perturbed_parameters['m'] * perturber.perturbed_parameters['g'], 0, 0]))
    output_converter = MPC_output_converter(np.array([perturber.perturbed_parameters['m'] * perturber.perturbed_parameters['g'], 0, 0]),
                                            perturber.perturbed_parameters['Kt'],
                                            ANGULAR_VELOCITY_RANGE)
    position_controller = PositionController(mpc,
                                             input_converter,
                                             output_converter,
                                             trajectory)

    step_trajectory_buffer = RollBuffers(['state', 'state_prediction', 'control_input'],
                                         [(STATES_NUM,), (STATES_NUM,), (CONTROLS_NUM,)],
                                         buffer_size=ATOMIC_TRAJ_SAMPLES_NUM)

    environment = ControlLoopEnvironment(quad_conf.quadcopter,
                                         quad_conf.load,
                                         position_controller,
                                         attitude_controller_conf.attitude_controller,
                                         quad_conf.esc,
                                         prediction_model,
                                         step_trajectory_buffer,
                                         STEP_TIME, MAX_STEPS_NUM, INNER_LOOP_FREQ, OUTER_LOOP_FREQ)
    # observations
    action = np.random.uniform(domain[0], domain[1], size=(1,))

    rbf_kernel = RBF_Kernel(length=2)
    gp = EfficientGaussianProcess(X0, rbf_kernel, noise_std=0.0)

    done = False
    environment.reset(Z550_parameters, Z550_parameters)
    action = np.array([perturber.perturbed_parameters['m']])
    while not done:
        state_next, reward, done = environment.step(action.item())
        #convert_trajectory(state_next[:, 3:6], state_next[:, 6], deltaT) # context
        gp(np.array(action).reshape(-1, 1), [reward])
        gp.plot(IMAGES_PATH)
        best = gp.Thompson_sampling(mode='min', number_of_samples=1)
        action = best['best_action']
        #action = np.array([quad_conf.model_parameters['m']])
        print("Best action", action)
        predicted_reward = best['predicted_reward']

    plotTrajectory3d(np.array(environment.history['x']), trajectory.generated_trajectory)