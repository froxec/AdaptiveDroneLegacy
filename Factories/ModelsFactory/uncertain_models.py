import numpy as np
from copy import deepcopy
from typing import Type
from Factories.ModelsFactory.linear_models import LinearizedQuadNoYaw
from Factories.DataManagementFactory.data_holders import DataHolder
class QuadTranslationalDynamicsUncertain:
    def __init__(self, parameters_holder: Type[DataHolder]):
        self.parameters_holder = parameters_holder
        self.parameters = parameters_holder.get_data()
        self.m = self.parameters_holder.m
        self.g = self.parameters_holder.g
        self.G = 1/self.m
    def __call__(self, z, u, u_l1, sigma_hat):
        f = (1 / self.m) * u - np.array([0, 0, 1]) * self.g
        g = (1 / self.m) * (u_l1 + sigma_hat)
        dyn = f + g
        return dyn

    def update_parameters(self):
        self.parameters = self.parameters_holder.get_data()
        self.m = self.parameters_holder.m
        self.g = self.parameters_holder.g

class NonlinearQuadUncertain:
    def __init__(self):
        pass
    def __call__(self):
        pass

class LinearQuadUncertain(LinearizedQuadNoYaw):
    def __init__(self, parameters_holder: Type[DataHolder] ,yaw_ss=0.0, x_ref=0.0, y_ref=0.0, z_ref=0.0):
        super().__init__(parameters_holder, Ts=None, yaw_ss=yaw_ss, x_ref=x_ref, y_ref=y_ref, z_ref=z_ref)
        self.A = self.A[3:6, 3:6]
        self.B = self.B[3:6, :]
        self.C = self.C[3:6, 3:6]
        self.D = self.D[3:6, :]
        self.G = self.B
        if isinstance(self.G, np.ndarray):
            self.G_Inv = np.linalg.inv(self.G)
        else:
            self.G_Inv = 1 / self.G

    def __call__(self, z, u, u_l1, sigma_hat):
        f = self.A @ z + self.B @ u
        g = self.B @ (u_l1 + sigma_hat)
        dyn = f + g
        return dyn
    def update_parameters(self):
        self.m = self.parameters_holder.m
        self.g = self.parameters_holder.g
        self.parameters = self.parameters_holder.get_data()
        self.B = np.array([[0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, self.g * np.sin(self.yaw_ss), self.g * np.cos(self.yaw_ss)],
                           [0.0, -self.g * np.cos(self.yaw_ss), self.g * np.sin(self.yaw_ss)],
                           [1 / self.m, 0.0, 0.0]])
        self.B = self.B[3:6, :]
        self.G = self.B
        if isinstance(self.G, np.ndarray):
            self.G_Inv = np.linalg.inv(self.G)
        else:
            self.G_Inv = 1/self.G

if __name__ == "__main__":
    from Factories.ModelsFactory.model_parameters import Z550_parameters
    model = LinearQuadUncertain(Z550_parameters)