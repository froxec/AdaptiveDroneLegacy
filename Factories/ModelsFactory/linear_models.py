import numpy as np
from copy import deepcopy
class LinearizedQuad():
    def __init__(self, parameters, u4_ss=0, x_ref=0, y_ref=0, z_ref=0):
        self.parameters = deepcopy(parameters)
        self.m = parameters['m']
        self.g = parameters['g']
        self.u4_ss = u4_ss
        self.A = np.array([[0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0]])
        self.B = np.array([[0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, self.g * np.sin(u4_ss), self.g * np.cos(u4_ss), 0],
                      [0, -self.g * np.cos(u4_ss), self.g * np.sin(u4_ss), 0],
                      [1 / self.m, 0, 0, 0]])
        self.C = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])
        self.D = np.array([[0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]])
        self.X_OP = np.array([x_ref, y_ref, z_ref, 0, 0, 0])
        self.Y_OP = self.C @ self.X_OP
        self.U_OP = np.array([self.m*self.g, 0, 0, u4_ss])

    def update(self, u4_ss=0, position_ref=None):
        self.B = np.array([[0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, self.g * np.sin(u4_ss), self.g * np.cos(u4_ss), 0],
                      [0, -self.g * np.cos(u4_ss), self.g * np.sin(u4_ss), 0],
                      [1 / self.m, 0, 0, 0]])
        if position_ref != None:
            self.X_OP = np.array([position_ref[0], position_ref[1], position_ref[2], 0, 0, 0])
            self.Y_OP = self.C @ self.X_OP
        self.U_OP = np.array([ self.m*self.g, 0, 0, u4_ss])

class LinearizedQuadNoYaw(LinearizedQuad):
    def __init__(self, parameters, Ts, yaw_ss=0.0, x_ref=0.0, y_ref=0.0, z_ref=0.0):
        super().__init__(parameters, u4_ss=yaw_ss, x_ref=x_ref, y_ref=y_ref, z_ref=z_ref)
        self.yaw_ss=yaw_ss
        self.B = np.array([[0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, self.g * np.sin(yaw_ss), self.g * np.cos(yaw_ss)],
                           [0.0, -self.g * np.cos(yaw_ss), self.g * np.sin(yaw_ss)],
                           [1 / self.m, 0.0, 0.0]])
        self.D = np.array([[0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0]])
        self.U_OP = np.array([self.m * self.g, 0.0, 0.0, 0.0])
        if Ts is not None:
            self.Ts = Ts
            self.discretize_model(Ts)
        else:
            self.Ts = None
    def update_parameters(self, parameters):
        self.parameters = deepcopy(parameters)
        self.g = parameters['g']
        self.m = parameters['m']
        self.B = np.array([[0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, self.g * np.sin(self.yaw_ss), self.g * np.cos(self.yaw_ss)],
                           [0.0, -self.g * np.cos(self.yaw_ss), self.g * np.sin(self.yaw_ss)],
                           [1 / self.m, 0.0, 0.0]])
        self.U_OP = np.array([self.m*self.g, 0.0, 0.0, self.u4_ss])
        if self.Ts is not None:
            self.discretize_model(self.Ts)
    def discretize_model(self, Ts):
        self.Ad = np.eye(self.A.shape[0]) + self.A * Ts
        self.Bd = self.B * Ts
        self.Cd = self.C
        self.Dd = self.D
        return self.Ad, self.Bd, self.Cd, self.Dd
    def discrete_prediction(self, x, u):
        delta_x = x - self.X_OP
        delta_u = u - self.U_OP[:-1]
        delta_x_next = self.Ad@delta_x + self.Bd@delta_u
        x_next = delta_x_next + self.X_OP
        return x_next

class AugmentedLinearizedQuadNoYaw(LinearizedQuadNoYaw):
    def __init__(self, parameters, Ts, yaw_ss=0.0, x_ref=0.0, y_ref=0.0, z_ref=0.0):
        super().__init__(parameters, Ts, yaw_ss, x_ref, y_ref, z_ref)
        self.construct_augmented_system()
    def construct_augmented_system(self):
        temp1 = np.concatenate([self.Ad, np.zeros((self.C.shape[1], self.C.shape[0]))], axis=1)
        temp2 = np.concatenate([self.Cd, np.eye(self.Cd.shape[0])], axis=1)
        self.Ad = np.concatenate([temp1, temp2], axis=0)
        self.Bd = np.concatenate([self.Bd, np.zeros((self.C.shape[0], self.Bd.shape[1]))], axis=0)
        self.Cd = np.concatenate([self.Cd, np.eye(self.Cd.shape[0])], axis=1)

    def update_parameters(self, parameters):
        super().update_parameters(parameters)
        self.construct_augmented_system()