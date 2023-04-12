from Factories.ControllersFactory.position_controllers.explicit_mpc_controller import ModelPredictiveController
from Factories.ModelsFactory.general_models import ElectronicSpeedControler
from Factories.ToolsFactory.Converters import MPC_output_converter
from Simulation import attitude_control as control
from Simulation.model import quadcopterModel, loadPendulum
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw


class QuadConfiguration:
    def __init__(self, model_parameters, pendulum_parameters, quad0, load0, pwm_range, angular_velocity_range):
        self.quad0 = quad0
        self.load0 = load0
        self.model_parameters = model_parameters
        self.quadcopter = quadcopterModel(self.quad0, model_parameters)
        self.load = loadPendulum(self.load0, pendulum_parameters, self.quadcopter.translational_accelerations, self.quadcopter.state)
        self.esc = ElectronicSpeedControler(pwm_range=pwm_range, angular_velocity_range=angular_velocity_range)


class ControllerConfiguration:
    def __init__(self, model_parameters, position0, position_ref, u_ss, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE, PWM_RANGE):
        self.position_controller = ModelPredictiveController(quad_parameters=model_parameters, x0=position0,
                                                   xref=position_ref, Ts= 1 / OUTER_LOOP_FREQ, angular_velocity_range=ANGULAR_VELOCITY_RANGE, linear_model=LinearizedQuadNoYaw)
        self.position_controller_output_converter = MPC_output_converter(u_ss, model_parameters['Kt'], ANGULAR_VELOCITY_RANGE)
        PWM0 = PWM_RANGE[0]
        self.attitude_controller = control.quadControler(1 / INNER_LOOP_FREQ, PWM_RANGE, PWM0)
        self.inner_loop_freq = INNER_LOOP_FREQ
        self.outer_loop_freq = OUTER_LOOP_FREQ

class AttitudeControllerConfiguration():
    def __init__(self, LOOP_FREQ, PWM_RANGE, PWM0):
        self.attitude_controller = control.quadControler(1 / LOOP_FREQ, PWM_RANGE, PWM0)
        self.loop_freq = LOOP_FREQ
        self.PWM_RANGE = PWM_RANGE
        self.PWM0 = PWM0