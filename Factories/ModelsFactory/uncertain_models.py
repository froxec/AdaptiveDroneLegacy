import numpy as np
from copy import deepcopy
class QuadTranslationalDynamicsUncertain:
    def __init__(self, parameters):
        self.parameters = deepcopy(parameters)
        self.m = parameters['m']
        self.g = parameters['g']

    def __call__(self, z, u, u_l1, sigma_hat):
        f = (1 / self.m) * u - np.array([0, 0, 1]) * self.g
        g = (1 / self.m) * (u_l1 + sigma_hat)
        dyn = f + g
        return dyn
