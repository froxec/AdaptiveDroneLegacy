
from Factories.ConfigurationsFactory.modes import MPCModes
from Factories.ModelsFactory.model_parameters import Z550_parameters_new, Iris_parameters
import numpy as np

# OPC SERVER
OPC_SERVER_ADDRESS = "opc.tcp://localhost:8085"
DATA_FREQ = 100

# Drone addresses
SIM_IP = 'udp:169.254.112.124:8500'
REAL_DRONE_IP = '/dev/ttyAMA1'

# REDIS DATABASE
REDIS_HOST = "127.0.0.1"
REDIS_PORT = 6379
DB_NUM = 0

# SQUARE TRAJECTORY
SQUARE_TRAJECTORY_HEIGHT = 4

# MODEL
NORMALIZE = True
ANGULAR_VELOCITY_RANGE = [0, 800]
PWM_RANGE = [1120, 1920]
MASS_PERTURBATION = 0.0
PREDICTOR_PARAMETERS = Iris_parameters
PREDICTOR_PARAMETERS['m'] = PREDICTOR_PARAMETERS['m'] + MASS_PERTURBATION

# MPC PARAMES
HORIZON = 10
MPC_MODE = MPCModes.CONSTRAINED
TRAJECTORY = [0, 0, 3]
THROTTLE_MIN = 0.1
THRUST_MIN, THRUST_MAX = THROTTLE_MIN * PREDICTOR_PARAMETERS['throttle_thrust_slope'] + PREDICTOR_PARAMETERS['throttle_thrust_intercept'],\
                        PREDICTOR_PARAMETERS['throttle_max'] * PREDICTOR_PARAMETERS['throttle_thrust_slope'] + PREDICTOR_PARAMETERS['throttle_thrust_intercept']
MPC_CONSTRAINTS = {"x_bounds": {'lower': np.array([-100000, -100000, -100000, -5, -5, -5]),
                                'upper': np.array([100000, 100000, 100000, 5, 5, 5])},
                   "u_bounds": {'lower': np.array([THRUST_MIN, -np.pi/6, -np.pi/6]),
                                'upper': np.array([THRUST_MAX, np.pi/6, np.pi/6])},
                   "delta_x_bounds": {'lower': np.array([-1000, -1000, -1000, -1000, -1000, -1000]),
                                   'upper': np.array([1000, 1000, 1000, 1000, 1000, 1000])},
                   "delta_u_bounds": {'lower': np.array([-100, -np.pi/24, -np.pi/24]),
                                   'upper': np.array([100, np.pi/24, np.pi/24])}}

# ADAPTIVE_PARAMETERS:
As = np.diag([-15, -15, -1])
BANDWIDTHS = [5, 0.5, 0.5]
L1_INPUT_BANDWIDTHS = [5, 2, 2]
# ESTIMATOR PARAMETERS
SAMPLING_FREQ = 100
MASS_MIN, MASS_MAX = (1.0, 3.0)
domain = (MASS_MIN, MASS_MAX)
samples_num = 100
X0 = np.linspace(domain[0], domain[1], samples_num).reshape(-1, 1)
CONVERGENCE_SAMPLES_REQUIRED = 15
CONVERGENCE_EPSILON_NEIGHBOURHOOD = 0.15

# CONTROLERS
MIN_ATTITUDE = 1

# TELEMETRY MANAGER
MQTT = True
MQTT_HOST = "localhost"
MQTT_PORT = 8955
