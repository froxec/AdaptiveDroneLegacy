from Factories.GaussianProcessFactory.kernels import  RBF_Kernel
from Factories.GaussianProcessFactory.gaussian_process import GaussianProcess, ContextualGaussianProcess
from Factories.ToolsFactory.GeneralTools import plot_signal, RollBuffers, minmax_normalize, minmax_rescale
from Factories.ToolsFactory.Converters import convert_trajectory

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
ATOMIC_TRAJ_SAMPLES_NUM = int(STEP_TIME*SAMPLING_FREQ)
MAX_EPISODE_TIME = 10 #s
MAX_STEPS_NUM = int(MAX_EPISODE_TIME/STEP_TIME)
QUAD_STATE0 = np.zeros(12)
LOAD_STATE0 = np.zeros(4)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
MASS_DOMAIN = (0.2, 2)
THROTTLE_DOMAIN = (0, 40)
ACCELERATION_DOMAIN = (0, 40)

if __name__ == "__main__":
    Z550_parameters['m'] = 0.4
    deltaT = STEP_TIME/OUTER_LOOP_FREQ
    samples_num = 10

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

    action_points = np.linspace(MASS_DOMAIN[0], MASS_DOMAIN[1], samples_num)
    throttle_points = np.linspace(THROTTLE_DOMAIN[0], THROTTLE_DOMAIN[1], samples_num)
    acceleration_points = np.linspace(ACCELERATION_DOMAIN[0], ACCELERATION_DOMAIN[1], samples_num)
    normalized_action_points = minmax_normalize(action_points, MASS_DOMAIN)
    normalized_throttle_points = minmax_normalize(throttle_points, THROTTLE_DOMAIN)
    normalized_acceleration_points = minmax_normalize(acceleration_points, ACCELERATION_DOMAIN)
    mesh = np.meshgrid(normalized_action_points, normalized_throttle_points, normalized_acceleration_points, indexing='ij')
    X0 = np.array([list(zip(mesh[0].flatten(), mesh[1].flatten(), mesh[2].flatten()))]).squeeze()
    # observations

    rbf_kernel = RBF_Kernel(length=0.3)
    contextual_gp = ContextualGaussianProcess(X0, rbf_kernel, noise_std=0.0)
    done = False
    action = np.random.uniform(0, 1, size=(1,)).item()
    action_rescaled = minmax_rescale(action, MASS_DOMAIN)
    for i in range(10):
        environment.reset(Z550_parameters, Z550_parameters)
        done = False
        while not done:
            state_next, reward, done = environment.step(action_rescaled)
            average_throttle, average_acceleration = convert_trajectory(state_next[:, 3:6], state_next[:, 6], deltaT)
            normalized_throttle, normalized_acceleration = minmax_normalize(average_throttle, THROTTLE_DOMAIN), minmax_normalize(average_acceleration, ACCELERATION_DOMAIN)
            contextual_gp([action, normalized_throttle, normalized_acceleration], reward)
            best = contextual_gp.Thompson_sampling([normalized_throttle, normalized_acceleration], mode='min', number_of_samples=10)
            contextual_gp.plot(MASS_DOMAIN)
            action = best['best_action']
            action_rescaled = minmax_rescale(action, MASS_DOMAIN)
            print("Best action", action_rescaled)
            predicted_reward = best['predicted_reward']
