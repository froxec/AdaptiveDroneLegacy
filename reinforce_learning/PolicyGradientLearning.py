import torch

from Factories.RLFactory.Agents.quad_mass_estimation_agent import QuadMassEstimator, PolicyGradientLearning, ReplayBuffer, RollBuffers
from Factories.RLFactory.Environments.base_env import ControlLoopEnvironment
from Factories.ConfigurationsFactory.configurations import ControllerConfiguration, QuadConfiguration
from Factories.ModelsFactory.model_parameters import Z550_parameters, pendulum_parameters
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw
import numpy as np
INNER_LOOP_FREQ = 100
OUTER_LOOP_FREQ = 10
SAMPLING_FREQ = OUTER_LOOP_FREQ#Hz
STEP_TIME = 1 #s
CONTROLS_NUM = 3
STATES_NUM = 6
CHANNELS_NUM = STATES_NUM + CONTROLS_NUM #real trajectory, control input
ATOMIC_TRAJ_SAMPLES_NUM = STEP_TIME*SAMPLING_FREQ
MAX_EPISODE_TIME = 15 #s
EPISODES_NUM = 1000
ESTIMATOR_INPUT_SHAPE = (CHANNELS_NUM, ATOMIC_TRAJ_SAMPLES_NUM)
ESTIMATOR_OUTPUT_SHAPE = 2 #mean and variance
QUAD_STATE0 = np.zeros(12)
LOAD_STATE0 = np.zeros(4)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
MASS_MIN = 0.2
MASS_MAX = 2
GAMMA = 0.0
ALPHA = 1e-5

if __name__ == "__main__":
    quad_conf = QuadConfiguration(Z550_parameters, pendulum_parameters, QUAD_STATE0, LOAD_STATE0, PWM_RANGE, ANGULAR_VELOCITY_RANGE)
    position0 = QUAD_STATE0[:6]
    position_ref = np.array([0, 0, 10, 0, 0, 0])
    u_ss = [quad_conf.model_parameters['m']*quad_conf.model_parameters['g'], 0, 0]
    nominal_control_conf = ControllerConfiguration(Z550_parameters, position0, position_ref, u_ss, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE)
    prediction_model = LinearizedQuadNoYaw(Z550_parameters)
    mass_estimator = QuadMassEstimator(ESTIMATOR_INPUT_SHAPE, ESTIMATOR_OUTPUT_SHAPE, MASS_MIN, MASS_MAX)
    learning_algorithm = PolicyGradientLearning(mass_estimator.policy_network, ALPHA, GAMMA)
    step_trajectory_buffer = RollBuffers(['state', 'state_prediction', 'control_input'], [(STATES_NUM,), (STATES_NUM,), (CONTROLS_NUM,)], buffer_size=ATOMIC_TRAJ_SAMPLES_NUM)
    replay_buffer = ReplayBuffer(['state', 'action', 'reward'], [ESTIMATOR_INPUT_SHAPE, (1,), (1,)])
    environment = ControlLoopEnvironment(quad_conf.quadcopter, quad_conf.load, nominal_control_conf.position_controller,
                                         nominal_control_conf.attitude_controller,
                                         nominal_control_conf.position_controller_output_converter,
                                         quad_conf.esc,
                                         STEP_TIME, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, prediction_model, step_trajectory_buffer)
    for i in range(EPISODES_NUM):
        state = np.zeros([10, 9])  # channels first
        done = False
        environment.reset(Z550_parameters, Z550_parameters)
        while not done:
            state = state.transpose().astype(np.float32)
            state = np.expand_dims(state, axis = 0)
            action = mass_estimator.predict(torch.from_numpy(state))
            print("Action", action)
            state_next, reward, done = environment.step(action)
            print("Reward", reward)
            replay_buffer.add_sample(['state', 'action', 'reward'], [state, action, reward])
            state = state_next