import numpy as np
from Simulation.model import system
class ControlLoopEnvironment():
    def __init__(self, quad, load, position_controller, attitude_controller, mpc_output_converter, step_time, inner_loop_freq, outer_loop_freq, prediction_model):
        self.quad = quad
        self.load = load
        self.step_time = step_time
        self.inner_loop_freq = inner_loop_freq
        self.outer_loop_freq = outer_loop_freq
        self.deltaT = 1 / inner_loop_freq
        self.MODULO_FACTOR = self.INNER_LOOP_FREQ / self.OUTER_LOOP_FREQ
        self.samples_per_step = int(outer_loop_freq*step_time)
        self.prediction_model = prediction_model
        self.x0 = np.array([0, 0, 0, 0, 0, 0])
        self.x = self.x0
        self.x_prev = self.x0
        self.position_controller = position_controller
        self.attitude_controller = attitude_controller
        self.mpc_output_converter = mpc_output_converter
    def step(self, mass):
        self.quad_parameters['m'] = mass
        self.prediction_model.update_parameters(self.quad_parameters)
        x = []
        x_estimated = []
        for i in range(1, int(self.inner_loop_freq*self.step_time)):
            if (i % self.MODULO_FACTOR) == 1:
                ref = self.position_controller.update_state_control(self.x_prev)
                ref_converted = self.mpc_output_converter(ref)
                attitude_setpoint = np.concatenate([ref_converted[1:], np.array([0])])
                throttle = ref_converted[0]
            ESC_PWMs = self.attitude_controller(attitude_setpoint, self.quad.state[6:9], self.quad.state[9:12],
                                                throttle)
            motors = self.esc(ESC_PWMs)
            self.x_prev = x
            x = system(np.array(motors), self.deltaT, self.quad, self.load)[:12]

    def env_reset(self, real_quad_parameters, prediction_model_parameters):
        self.prediction_model.update_parameters(prediction_model_parameters)
        self.quad.update_parameters(real_quad_parameters)
        self.x = self.x0
        self.x_prev = self.x0