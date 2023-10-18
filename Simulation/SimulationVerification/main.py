from Factories.ConfigurationsFactory.configurations import QuadConfiguration
from Factories.ModelsFactory.model_parameters import Z550_parameters
from Simulation.SimulationVerification.PARAMETERS import PWM_RANGE, ANGULAR_VELOCITY_RANGE, DELTA_T
from Factories.SimulationsFactory.SITL import VerificationSITL
import numpy as np
if __name__ == "__main__":
    # create quadcopter configuration
    quad_config = QuadConfiguration(Z550_parameters, None, np.zeros(12), None,
                                    pwm_range=PWM_RANGE, angular_velocity_range=ANGULAR_VELOCITY_RANGE)
    # create simulator
    simulator = VerificationSITL(quad_config.quadcopter)
    # define control vector (motors speed [rad/s])
    control_vector = np.array([545, 555, 545, 555])
    # define initial state x0
    x0 = np.zeros(12)
    t, x, u = simulator.run(20, DELTA_T, x0, control_vector)
    simulator.plot_trajectory(t, x)