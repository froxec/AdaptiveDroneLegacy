from Factories.ConfigurationsFactory.modes import MPCModes
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SinglePoint
from Factories.ModelsFactory.model_parameters import Z550_parameters

OPC_SERVER_ADDRESS = "opc.tcp://localhost:8085"

# MODEL
NORMALIZE = True
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]

#MPC PARAMES
HORIZON = 10
MPC_MODE = MPCModes.CONSTRAINED
TRAJECTORY = SinglePoint([0, 0, 10])
PREDICTOR_PARAMETERS = Z550_parameters