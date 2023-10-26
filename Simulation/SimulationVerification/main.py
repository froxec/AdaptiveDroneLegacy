from Factories.ConfigurationsFactory.configurations import QuadConfiguration
from Factories.ModelsFactory.model_parameters import Z550_parameters
from Simulation.SimulationVerification.PARAMETERS import PWM_RANGE, ANGULAR_VELOCITY_RANGE, DELTA_T
from Factories.SimulationsFactory.SITL import VerificationSITL
import numpy as np
if __name__ == "__main__":
    # define initial state x0
    x0 = np.zeros(12)
    x0[8] = np.pi / 4

    # create quadcopter configuration
    quad_config = QuadConfiguration(Z550_parameters, None, x0, None,
                                    pwm_range=PWM_RANGE, angular_velocity_range=ANGULAR_VELOCITY_RANGE)
    # create simulator
    simulator = VerificationSITL(quad_config.quadcopter, results_save_path='../../ResearchTests/SimulationVerification/', filename='move_theta_psi')
    # define control vector (motors speed [rad/s])
    control_vector = np.array([548, 552, 548, 552])
    t, x, u = simulator.run(20, DELTA_T, x0, control_vector)
    simulator.plot_trajectory(t, x)