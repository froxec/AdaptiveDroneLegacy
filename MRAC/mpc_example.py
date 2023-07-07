from Factories.ModelsFactory.linear_models import LinearizedQuadNoYawWithUncertainty
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SpiralTrajectory, RectangularTrajectory, SinglePoint
from Factories.ConfigurationsFactory.configurations import QuadConfiguration, ControllerWithCompensatorConfiguration, CustomMPCConfig
from Factories.ModelsFactory.model_parameters import pendulum_parameters, Z550_parameters
from Factories.ToolsFactory.AnalysisTools import ParametersPerturber
import numpy as np

INNER_LOOP_FREQ = 100
OUTER_LOOP_FREQ = 10
MODULO_FACTOR = int(INNER_LOOP_FREQ/OUTER_LOOP_FREQ)
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
single_point_traj = SinglePoint(np.array([0, 0, 10]))


if __name__ == '__main__':
    deltaT = 1 / INNER_LOOP_FREQ
    quad_conf = QuadConfiguration(Z550_parameters, pendulum_parameters, np.zeros(12), np.zeros(4), PWM_RANGE,
                                  ANGULAR_VELOCITY_RANGE)

    perturber = ParametersPerturber(Z550_parameters)
    perturber({'m': 0.0})
    print(perturber.perturbed_parameters)
    prediction_model = LinearizedQuadNoYawWithUncertainty(perturber.perturbed_parameters, 1/OUTER_LOOP_FREQ)
    controller_conf = CustomMPCConfig(prediction_model, INNER_LOOP_FREQ, OUTER_LOOP_FREQ, ANGULAR_VELOCITY_RANGE,
                                      PWM_RANGE, horizon=10)
