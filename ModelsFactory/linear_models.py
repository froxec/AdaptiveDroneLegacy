from ModelsFactory.model_parameters import quad_parameters
import numpy as np
class LinearizedQuad():
    def __init__(self, dt=0, u4_ss=0, x_ref=0, y_ref=0, z_ref=0):
        self.m = quad_parameters['m']
        self.g = quad_parameters['g']
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
        self.U_OP = np.array([ self.m*self.g, 0, 0, u4_ss])
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