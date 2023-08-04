import numpy as np
from copy import deepcopy
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw
class QuadTranslationalDynamicsUncertain:
    def __init__(self, parameters):
        self.parameters = deepcopy(parameters)
        self.m = parameters['m']
        self.g = parameters['g']
        self.G = 1/self.m
    def __call__(self, z, u, u_l1, sigma_hat):
        f = (1 / self.m) * u - np.array([0, 0, 1]) * self.g
        g = (1 / self.m) * (u_l1 + sigma_hat)
        dyn = f + g
        return dyn

class NonlinearQuadUncertain:
    def __init__(self):
        pass
    def __call__(self):
        pass

class LinearQuadUncertain(LinearizedQuadNoYaw):
    def __init__(self, parameters ,yaw_ss=0.0, x_ref=0.0, y_ref=0.0, z_ref=0.0):
        super().__init__(parameters, Ts=None, yaw_ss=yaw_ss, x_ref=x_ref, y_ref=y_ref, z_ref=z_ref)
        self.A = self.A[3:6, 3:6]
        self.B = self.B[3:6, :]
        self.C = self.C[3:6, 3:6]
        self.D = self.D[3:6, :]
        self.G = self.B

    def __call__(self, z, u, u_l1, sigma_hat):
        f = self.A @ z + self.B @ u
        g = self.B @ (u_l1 + sigma_hat)
        dyn = f + g
        return dyn


if __name__ == "__main__":
    from Factories.ModelsFactory.model_parameters import Z550_parameters
    model = LinearQuadUncertain(Z550_parameters)