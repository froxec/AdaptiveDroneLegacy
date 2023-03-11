import numpy as np
class LinearizedQuad():
    def __init__(self, parameters, dt=0, u4_ss=0, x_ref=0, y_ref=0, z_ref=0):
        self.parameters = parameters
        self.m = parameters['m']
        self.g = parameters['g']
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
    def __init__(self, parameters, dt=0, yaw_ss=0, x_ref=0, y_ref=0, z_ref=0):
        super().__init__(parameters, dt=0, u4_ss=0, x_ref=0, y_ref=0, z_ref=0)
        self.yaw_ss=yaw_ss
        self.B = np.array([[0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [0, self.g * np.sin(yaw_ss), self.g * np.cos(yaw_ss)],
                           [0, -self.g * np.cos(yaw_ss), self.g * np.sin(yaw_ss)],
                           [1 / self.m, 0, 0]])
        self.D = np.array([[0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0]])
        self.U_OP = np.array([self.m * self.g, 0, 0])
    def update_parameters(self, parameters):
        self.parameters = parameters
        self.g = parameters['g']
        self.m = parameters['m']
        self.B = np.array([[0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [0, self.g * np.sin(self.yaw_ss), self.g * np.cos(self.yaw_ss)],
                           [0, -self.g * np.cos(self.yaw_ss), self.g * np.sin(self.yaw_ss)],
                           [1 / self.m, 0, 0]])