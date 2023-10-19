from Factories.SimulationsFactory.SITL import InnerLoopSITL
from Factories.ConfigurationsFactory.configurations import QuadConfiguration, AttitudeControllerConfiguration
import Factories.ControllersFactory.attitude_controllers.controler_parameters
from Factories.ModelsFactory.model_parameters import Z550_parameters, pendulum_parameters
from Factories.ToolsFactory.Converters import LinearScaler, Normalizer, ThrustToAngularVelocity

import numpy as np

INNER_LOOP_FREQ = 1000
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
THRUST_SS = Z550_parameters['m']*Z550_parameters['g']
if __name__ == "__main__":
    quad_conf = QuadConfiguration(Z550_parameters, pendulum_parameters, np.zeros(12), np.zeros(4), PWM_RANGE,
                                  ANGULAR_VELOCITY_RANGE)

    thrust_converter = ThrustToAngularVelocity(Z550_parameters['Kt'])
    angular_vel_normalizer = Normalizer(min=ANGULAR_VELOCITY_RANGE[0], max=ANGULAR_VELOCITY_RANGE[1])
    throttle_to_pwm = LinearScaler(min=PWM_RANGE[0], max=PWM_RANGE[1])
    attitude_ref = np.array([np.pi /6, 0, 0])
    angular_speed0 = thrust_converter(THRUST_SS)
    throttle = angular_vel_normalizer(angular_speed0)
    PWM0 = throttle_to_pwm(throttle)

    control_conf = AttitudeControllerConfiguration(INNER_LOOP_FREQ, PWM_RANGE, PWM0)

    sim = InnerLoopSITL(quad_conf.quadcopter, quad_conf.load, control_conf.attitude_controller, quad_conf.esc,
                        INNER_LOOP_FREQ)

    x0 = np.zeros((12))
    u0 = np.array([angular_speed0,  angular_speed0,  angular_speed0,  angular_speed0])
    stop_time = 1
    sim.run(attitude_ref, throttle, stop_time, x0, u0)
    sim.plot_attitude_trajectory(tested_variable =0)