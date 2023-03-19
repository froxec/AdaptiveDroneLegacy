from Factories.GaussianProcessFactory.kernels import  RBF_Kernel
from Factories.GaussianProcessFactory.gaussian_process import GaussianProcess
from Factories.ToolsFactory.GeneralTools import plot_signal

from Factories.RLFactory.Agents.quad_mass_estimation_agent import QuadMassEstimator, PolicyGradientLearning, ReplayBuffer, RollBuffers
from Factories.RLFactory.Environments.base_env import ControlLoopEnvironment
from Factories.ConfigurationsFactory.configurations import ControllerConfiguration, QuadConfiguration
from Factories.ModelsFactory.model_parameters import Z550_parameters, pendulum_parameters
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw
from Factories.ToolsFactory.AnalysisTools import RLMonitor

import numpy as np

INNER_LOOP_FREQ = 100
OUTER_LOOP_FREQ = 50
SAMPLING_FREQ = OUTER_LOOP_FREQ#Hz
STEP_TIME = 1 #s
CONTROLS_NUM = 3
STATES_NUM = 6
ATOMIC_TRAJ_SAMPLES_NUM = int(STEP_TIME*SAMPLING_FREQ)
MAX_EPISODE_TIME = 10 #s
MAX_STEPS_NUM = int(MAX_EPISODE_TIME/STEP_TIME)
QUAD_STATE0 = np.zeros(12)
LOAD_STATE0 = np.zeros(4)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
MASS_MIN = 0.2
MASS_MAX = 2

if __name__ == "__main__":
    samples_num = 100
    domain = (0.2, 2)
    X0 = np.linspace(domain[0], domain[1], samples_num).reshape(-1, 1)

    quad_conf = QuadConfiguration(Z550_parameters, pendulum_parameters, QUAD_STATE0, LOAD_STATE0, PWM_RANGE,
                                  ANGULAR_VELOCITY_RANGE)
    position0 = QUAD_STATE0[:6]
    position_ref = np.array([0, 0, 10, 0, 0, 0])
    u_ss = [quad_conf.model_parameters['m'] * quad_conf.model_parameters['g'], 0, 0]
    nominal_control_conf = ControllerConfiguration(Z550_parameters, position0, position_ref, u_ss, INNER_LOOP_FREQ,
                                                   OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE)
    prediction_model = LinearizedQuadNoYaw(Z550_parameters)
    step_trajectory_buffer = RollBuffers(['state', 'state_prediction', 'control_input'],
                                         [(STATES_NUM,), (STATES_NUM,), (CONTROLS_NUM,)],
                                         buffer_size=ATOMIC_TRAJ_SAMPLES_NUM)
    environment = ControlLoopEnvironment(quad_conf.quadcopter, quad_conf.load, nominal_control_conf.position_controller,
                                         nominal_control_conf.attitude_controller,
                                         nominal_control_conf.position_controller_output_converter,
                                         quad_conf.esc,
                                         STEP_TIME, MAX_STEPS_NUM, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, prediction_model,
                                         step_trajectory_buffer)

    # observations
    action = np.random.uniform(domain[0], domain[1], size=(1,))

    rbf_kernel = RBF_Kernel(length=0.3)
    gp = GaussianProcess(X0, rbf_kernel, noise_std=0.0)

    done = False
    environment.reset(Z550_parameters, Z550_parameters)
    while not done:
        state_next, reward, done = environment.step(action.item())
        gp(np.array(action).reshape(-1, 1), [reward])
        gp.plot()
        best = gp.Thompson_sampling(mode='min', number_of_samples=10)
        action = best['best_action']
        print("Best action", action)
        predicted_reward = best['predicted_reward']
