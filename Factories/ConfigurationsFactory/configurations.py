from Factories.ControllersFactory.position_controllers.mpc_controllers import ModelPredictiveController
from Factories.ModelsFactory.general_models import ElectronicSpeedControler
from Factories.ToolsFactory.Converters import MPC_output_converter, MPC_input_converter
from Simulation import attitude_control as control
from Simulation.model import quadcopterModel, loadPendulum
from Factories.ControllersFactory.position_controllers.thrust_compensator import ThrustCompensator
from Factories.ControllersFactory.position_controllers.position_controller_parameters import thrust_compensator_parameters
import numpy as np
from Factories.ControllersFactory.position_controllers.mpc_controllers import gekkoMPC
from Factories.ControllersFactory.position_controllers.mpc import ModelPredictiveControl
from Factories.ControllersFactory.position_controllers.constrained_mpc import ConstrainedMPC
from Factories.ModelsFactory.linear_models import LinearizedQuad, LinearTranslationalMotionDynamics
from typing import Type

class Configuration():
    def __init__(self):
        raise NotImplementedError
    def __call__(self):
        raise NotImplementedError

class ControllerConfigurationBase(Configuration):
    def __init__(self):
        self.position_controller = None
        self.position_controller_output_converter = None
        self.position_controller_input_converter = None
        self.attitude_controller = None
        self.inner_loop_freq = None
        self.outer_loop_freq = None

class PositionControllerConfiguration(ControllerConfigurationBase):
    def __init__(self, position_controller: Type[ModelPredictiveController],
                 input_converter: Type[MPC_input_converter],
                 output_converter: Type[MPC_output_converter],
                 position_controller_freq: int):
        self.position_controller = position_controller
        self.input_converter = input_converter
        self.output_converter = output_converter
        self.freq = position_controller_freq

class QuadConfiguration(Configuration):
    def __init__(self, model_parameters, pendulum_parameters, quad0, load0, pwm_range, angular_velocity_range, external_disturbance=None):
        self.quad0 = quad0
        self.load0 = load0
        self.model_parameters = model_parameters
        self.quadcopter = quadcopterModel(self.quad0, model_parameters, external_disturbance=external_disturbance)
        self.load = loadPendulum(self.load0, pendulum_parameters, self.quadcopter.translational_accelerations, self.quadcopter.state)
        self.esc = ElectronicSpeedControler(pwm_range=pwm_range, angular_velocity_range=angular_velocity_range)


class CustomMPCConfig(ControllerConfigurationBase):
    def __init__(self, prediction_model, INNER_LOOP_FREQ,
                 OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE, PWM_RANGE, horizon=10, normalize_system=False,
                 MPC_IMPLEMENTATION='SPARSE'):
        if isinstance(prediction_model, LinearTranslationalMotionDynamics):
            mode = 'transDynamicsModel'
        elif isinstance(prediction_model, LinearizedQuad):
            mode = 'proprietary'
        if MPC_IMPLEMENTATION == 'SPARSE':
            self.position_controller = ConstrainedMPC(prediction_model, OUTER_LOOP_FREQ, horizon, normalize_system=normalize_system)
        else:
            self.position_controller = ModelPredictiveControl(prediction_model, OUTER_LOOP_FREQ, horizon, normalize_system=normalize_system)
        x_ss = np.zeros(prediction_model.A.shape[0])
        self.position_controller_output_converter = MPC_output_converter(prediction_model.parameters_holder,
                                                                         ANGULAR_VELOCITY_RANGE, mode)
        self.position_controller_input_converter = MPC_input_converter(x_ss, prediction_model.parameters_holder, mode)
        PWM0 = PWM_RANGE[0]
        self.attitude_controller = control.quadControler(1 / INNER_LOOP_FREQ, PWM_RANGE, PWM0)
        self.inner_loop_freq = INNER_LOOP_FREQ
        self.outer_loop_freq = OUTER_LOOP_FREQ

class ControllerConfiguration(ControllerConfigurationBase):
    def __init__(self, model_parameters, position0, trajectory,x_ss, u_ss, prediction_model, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE, PWM_RANGE):
        self.position_controller = ModelPredictiveController(quad_parameters=model_parameters, x0=position0,
                                                   trajectory=trajectory, Ts= 1 / OUTER_LOOP_FREQ, angular_velocity_range=ANGULAR_VELOCITY_RANGE, linear_model=prediction_model)
        if isinstance(prediction_model, LinearTranslationalMotionDynamics):
            mode = 'transDynamicsModel'
        elif isinstance(prediction_model, LinearizedQuad):
            mode = 'proprietary'
        self.position_controller_output_converter = MPC_output_converter(prediction_model.parameters_holder,
                                                                         ANGULAR_VELOCITY_RANGE, mode)
        self.position_controller_input_converter = MPC_input_converter(x_ss, prediction_model.parameters_holder, mode)
        PWM0 = PWM_RANGE[0]
        self.attitude_controller = control.quadControler(1 / INNER_LOOP_FREQ, PWM_RANGE, PWM0)
        self.inner_loop_freq = INNER_LOOP_FREQ
        self.outer_loop_freq = OUTER_LOOP_FREQ

# class GekkoConfiguration(ControllerConfiguration):
#     def __init__(self, model_parameters, position0, trajectory, x_ss, u_ss, prediction_model, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE, PWM_RANGE, control_horizon):
#         self.model = prediction_model(model_parameters)
#         self.position_controller = gekkoMPC(self.model, control_horizon, OUTER_LOOP_FREQ)
#         self.position_controller_output_converter = MPC_output_converter(u_ss, model_parameters['Kt'], ANGULAR_VELOCITY_RANGE)
#         self.position_controller_input_converter = MPC_input_converter(x_ss, u_ss)
#         PWM0 = PWM_RANGE[0]
#         self.attitude_controller = control.quadControler(1 / INNER_LOOP_FREQ, PWM_RANGE, PWM0)
#         self.inner_loop_freq = INNER_LOOP_FREQ
#         self.outer_loop_freq = OUTER_LOOP_FREQ

# class ControllerWithCompensatorConfiguration(ControllerConfiguration):
#     def __init__(self, model_parameters, position0, trajectory, x_ss, u_ss, prediction_model, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE, PWM_RANGE):
#         super().__init__(model_parameters, position0, trajectory, x_ss, u_ss, prediction_model, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE, PWM_RANGE)
#         thrust0 = np.array([0, 0, 0])
#         state0 = position0
#         self.thrust_compensator = ThrustCompensator(thrust_compensator_parameters, u_ss, model_parameters, prediction_model=prediction_model, state0=state0, thrust0=thrust0, deltaT=1/INNER_LOOP_FREQ)

class AttitudeControllerConfiguration(ControllerConfigurationBase):
    def __init__(self, LOOP_FREQ, PWM_RANGE, PWM0):
        self.attitude_controller = control.quadControler(1 / LOOP_FREQ, PWM_RANGE, PWM0)
        self.loop_freq = LOOP_FREQ
        self.PWM_RANGE = PWM_RANGE
        self.PWM0 = PWM0

def positionControllerWrapper(ControllerConfiguration: Type[ControllerConfigurationBase]) -> Type[PositionControllerConfiguration]:
    base_conf = ControllerConfiguration
    controller = base_conf.position_controller
    input_converter = base_conf.position_controller_input_converter
    output_converter = base_conf.position_controller_output_converter
    freq = base_conf.outer_loop_freq
    position_controller_configuration = PositionControllerConfiguration(position_controller=controller,
                                                                        input_converter=input_converter,
                                                                        output_converter=output_converter,
                                                                        position_controller_freq=freq)
    return position_controller_configuration
