from Factories.ConfigurationsFactory.modes import MPCModes
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SinglePoint
from Factories.ModelsFactory.model_parameters import Z550_parameters
import numpy as np

# OPC SERVER
OPC_SERVER_ADDRESS = "opc.tcp://localhost:8085"

# REDIS DATABASE
REDIS_HOST = "127.0.0.1"
REDIS_PORT = 6379
DB_NUM = 0

# MODEL
NORMALIZE = True
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]

# MPC PARAMES
HORIZON = 10
MPC_MODE = MPCModes.CONSTRAINED
TRAJECTORY = SinglePoint([0, 0, 10])
PREDICTOR_PARAMETERS = Z550_parameters
PREDICTOR_PARAMETERS['m'] = 0.6

# ADAPTIVE_PARAMETERS:
As = np.diag([-15, -15, -15])
BANDWIDTHS = [0.5, 0.2, 0.2]

# CONTROLERS
MIN_ATTITUDE = 2.5