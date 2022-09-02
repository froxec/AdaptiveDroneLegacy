import numpy as np
from numpy import deg2rad
quad_parameters = {
    'm': 1.025,
    'g': 9.81,
    'Kt': 3.122e-6,
    'Kd': 4.759e-9,
    'I': np.array([0.12, 0.12, 0.048]),
    'l': 0.27,
    'arm_angle': deg2rad(45) # X-configuration, angle between arm 2 or 4 and x axis
}
pendulum_parameters = {
    'm': 1,
    'l': 0.2,
    'g': 9.81
}