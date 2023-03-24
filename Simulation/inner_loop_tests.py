from Factories.SimulationsFactory.SITL import InnerLoopSITL
from Factories.ConfigurationsFactory.configurations import QuadConfiguration
import Factories.ControllersFactory.attitude_controllers.controler_parameters
from Factories.ModelsFactory.model_parameters import Z550_parameters, pendulum_parameters

import numpy as np

INNER_LOOP_FREQ = 100
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]

if __name__ == "__main__":
    quad_conf = QuadConfiguration(Z550_parameters, pendulum_parameters, np.zeros(12), np.zeros(4), PWM_RANGE,
                                  ANGULAR_VELOCITY_RANGE)
    sim = InnerLoopSITL(quad_conf.quadcopter, quad_conf.load, attitude_controler, esc, inner_loop_freq, outer_loop_freq)