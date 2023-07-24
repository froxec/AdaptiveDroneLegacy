import numpy as np
from Factories.ModelsFactory.model_parameters import Z550_parameters, pendulum_parameters
from Factories.ConfigurationsFactory.configurations import QuadConfiguration, CustomMPCConfig
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw
from Factories.ConfigurationsFactory.modes import MPCModes
from Factories.SimulationsFactory.SITL import SoftwareInTheLoop
from Factories.ToolsFactory.AnalysisTools import ParametersPerturber
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SpiralTrajectory, RectangularTrajectory, SinglePoint, \
    RectangularTrajectoryWithTerminals
from Factories.ControllersFactory.adaptive_augmentation.l1_augmentation import *
from Factories.ModelsFactory.uncertain_models import QuadTranslationalDynamicsUncertain
from Factories.ModelsFactory.external_force_models import WindModel
from Simulation.plots import plotTrajectory, plotTrajectory3d

INNER_LOOP_FREQ = 100
deltaT = 1 / INNER_LOOP_FREQ
OUTER_LOOP_FREQ = 10
MODULO_FACTOR = int(INNER_LOOP_FREQ/OUTER_LOOP_FREQ)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
trajectory = SinglePoint([50, 50, 20])
if __name__ == "__main__":
    perturber = ParametersPerturber(Z550_parameters)
    perturber({'m': -0.5})

    ##External Disturbances
    wind_force = WindModel(direction_vector=[1, 1, 1], strength=1)

    ## Model configuration
    quad_conf = QuadConfiguration(perturber.perturbed_parameters, pendulum_parameters, np.zeros(12), np.zeros(4), PWM_RANGE,
                                  ANGULAR_VELOCITY_RANGE, external_disturbance=wind_force)
    x0 = np.concatenate([quad_conf.quad0, quad_conf.load0])
    u0 = np.zeros(3)

    ## Controller configuration
    prediction_model = LinearizedQuadNoYaw(Z550_parameters, 1 / OUTER_LOOP_FREQ)
    controller_conf = CustomMPCConfig(prediction_model, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE,
                                      PWM_RANGE, horizon=10)
    controller_conf.position_controller.switch_modes(MPCModes.UNCONSTRAINED)

    ## Adaptive Controller configuration
    z0 = x0[3:6]
    As = np.diag([-0.2, -0.2, -0.2])
    uncertain_model = QuadTranslationalDynamicsUncertain(Z550_parameters)
    l1_predictor = L1_Predictor(uncertain_model, z0, 1 / INNER_LOOP_FREQ, As)
    l1_adaptive_law = L1_AdaptiveLaw(uncertain_model, 1 / INNER_LOOP_FREQ, As)
    l1_filter = L1_LowPass(bandwidths=[0.1, 0.01, 0.1], fs=INNER_LOOP_FREQ, signals_num=z0.shape[0], no_filtering=False)
    l1_converter = L1_ControlConverter()
    adaptive_controller = L1_Augmentation(l1_predictor, l1_adaptive_law, l1_filter, l1_converter)

    # Simulation
    ramp_saturation_slope = np.array([np.Inf, 0.78, 0.78])
    simulator = SoftwareInTheLoop(quad_conf.quadcopter, quad_conf.load, trajectory, controller_conf.position_controller,
                                  controller_conf.attitude_controller,
                                  [controller_conf.position_controller_input_converter,
                                   controller_conf.position_controller_output_converter]
                                  ,quad_conf.esc, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, adaptive_controller=adaptive_controller, ramp_saturation_slope=ramp_saturation_slope)
    t, x = simulator.run(50, deltaT, x0[0:12], u0, trajectory)
    plotTrajectory(t, x.transpose()[0:12], 4, 3)
    #plotTrajectory(t, x.transpose()[0:12], 4, 3, [1, 2, 4, 5, 7, 8, 9, 10, 11, 12])