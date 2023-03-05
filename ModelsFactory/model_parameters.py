import numpy as np
from numpy import deg2rad
quad_parameters = {
    'm': 1.0,
    'g': 9.81,
    'Kt': 3e-6,
    'Kd': 4e-9,
    'I': np.array([0.1, 0.1, 0.05]),
    'l': 0.25,
    'arm_angle': deg2rad(45) # X-configuration, angle between arm 2 or 4 and x axis
}
pendulum_parameters = {
    'm': 1,
    'l': 0.2,
    'g': 9.81,
    'r': 0.1,
    'Cd': 0.47,
    'ro': 1.23
}
Z550_parameters = {
    'm': 0.940 + 0.182, #quad_mass + lipo_mass
    'Kt': 1.3812635522368278e-06,
    'Kd': 2.4953940390301706e-07,
    'I': np.array([0.1, 0.1, 0.05]),
    'l': 0.29,
    'arm_angle': deg2rad(45)
}
arducopter_parameters = {
    'm': 1.0,
    'g': 9.81,
    'Kt': 3e-6,
    'Kd': 4e-9,
    'I': np.array([0.1, 0.1, 0.05]),
    'l': 0.25,
    'arm_angle': deg2rad(45) # X-configuration, angle between arm 2 or 4 and x axis
}
