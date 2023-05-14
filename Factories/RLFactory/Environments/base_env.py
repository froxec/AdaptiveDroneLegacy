import numpy as np
from Simulation.model import system
from Factories.ToolsFactory.GeneralTools import manhattan_distance, sigmoid_function
from typing import Type

from Simulation.model import quadcopterModel, loadPendulum
from Factories.ControllersFactory.position_controllers.position_controller import PositionController
from Simulation.attitude_control import attitudeControler
from Factories.ModelsFactory.general_models import ElectronicSpeedControler
from Factories.ModelsFactory.linear_models import LinearizedQuad
from Factories.ToolsFactory.GeneralTools import RollBuffers
class ControlLoopEnvironment():
    def __init__(self,
                 quad: Type[quadcopterModel],
                 load: Type[loadPendulum],
                 position_controller: Type[PositionController],
                 attitude_controller: Type[attitudeControler],
                 esc: Type[ElectronicSpeedControler],
                 prediction_model: Type[LinearizedQuad],
                 step_buffer: Type[RollBuffers],
                 step_time: int,
                 MAX_STEPS_NUM: int,
                 inner_loop_freq: int,
                 outer_loop_freq: int,
                 x0=np.zeros(6),
                 u0=np.zeros(3)):
        self.quad = quad
        self.load = load
        self.step_time = step_time
        self.inner_loop_freq = inner_loop_freq
        self.outer_loop_freq = outer_loop_freq
        self.deltaT = 1 / inner_loop_freq
        self.prediction_deltaT = 1/outer_loop_freq
        self.MODULO_FACTOR = int(self.inner_loop_freq / self.outer_loop_freq)
        self.samples_per_step = int(outer_loop_freq*step_time) - 1 # one less prediction than number of steps
        self.prediction_model = prediction_model
        self.prediction_model_parameters = prediction_model.parameters
        self.x0 = x0
        self.u0 = u0
        self.x = self.x0
        self.u_prev = None
        self.x_prev = self.x
        self.trajectory_buffer = step_buffer
        self.position_controller = position_controller
        self.attitude_controller = attitude_controller
        self.esc = esc
        self.MAX_STEPS_NUM = MAX_STEPS_NUM
        self.done = False
        self.steps_done = 0
        self.REWARD_COEFFICIENT = 1
        self.history = {'x': [], 'u': []}
    def step(self, mass):
        prediction_prev = self.x_prev[:6]
        self.prediction_model_parameters['m'] = mass
        self.prediction_model.update_parameters(self.prediction_model_parameters)
        for i in range(int(self.inner_loop_freq*self.step_time)+1):
            #TODO - kod zakłada, że MAB jest aktualizowany z częstotliwością równą częstotliwości OUTER_LOOP - potencjalne bugi
            if (i % self.MODULO_FACTOR) == 0:
                ref_converted = self.position_controller(self.x)
                attitude_setpoint = np.concatenate([ref_converted[1:], np.array([0])])
                throttle = ref_converted[0]
                if self.u_prev is not None:
                    prediction = self.prediction_model.discrete_prediction(prediction_prev, self.u_prev)
                    self.add_sample_to_buffer(self.x[:6], prediction, self.u_prev)
                    prediction_prev = prediction
                self.u_prev = self.position_controller.output_converter.nominal_u
                self.x_prev = self.x
            ESC_PWMs = self.attitude_controller(attitude_setpoint, self.quad.state[6:9], self.quad.state[9:12],
                                                throttle)
            motors = self.esc(ESC_PWMs)
            self.x = system(np.array(motors), self.deltaT, self.quad, self.load)[:6]
            self.history['x'].append(list(self.x))
            self.history['u'].append(list(self.u_prev))
        penalty = self.calculate_penalty()
        penalty = self.normalize_penalty(penalty)
        #reward = np.log10(penalty**3 + 1)
        #reward = sigmoid_function(0.2*penalty**2, 10, 0)
        reward = penalty
        env_state = np.concatenate((self.trajectory_buffer['state'], self.trajectory_buffer['control_input']), axis=1)
        self.update_done()
        return env_state, reward, self.done
    def reset(self, real_quad_parameters, prediction_model_parameters):
        self.prediction_model.update_parameters(prediction_model_parameters)
        self.quad.update_parameters(real_quad_parameters)
        self.x = self.x0
        self.u_prev = None
        self.done = False
        self.steps_done = 0
        self.trajectory_buffer.flush()

    def add_sample_to_buffer(self, state, state_prediction, control_input):
        self.trajectory_buffer.add_sample(['state', 'state_prediction', 'control_input'], [state, state_prediction, control_input])

    def calculate_penalty(self):
        reward = 0
        state, state_prediction = self.trajectory_buffer['state'][:, 3:], self.trajectory_buffer['state_prediction'][:, 3:]
        # normalized_state = self.normalize_trajectory(state)
        # normalized_prediction = self.normalize_trajectory(state_prediction)
        for i in range(self.samples_per_step):
            reward += manhattan_distance(state[i], state_prediction[i])*self.prediction_deltaT
        return reward

    def normalize_penalty(self, penalty):
        return penalty/self.step_time
    def normalize_trajectory(self, trajectory):
        max_values = np.max(trajectory, axis=0)
        min_values = np.min(trajectory, axis=0)

        normalized_trajectory = (trajectory - min_values) / (max_values - min_values)
        return normalized_trajectory

    def update_done(self):
        self.steps_done += 1
        if self.steps_done > self.MAX_STEPS_NUM:
            self.done = True




