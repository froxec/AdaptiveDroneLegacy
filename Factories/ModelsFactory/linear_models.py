import numpy as np
from copy import deepcopy
import control
class LinearTranslationalMotionDynamics:
    def __init__(self, parameters, Ts, xref=0.0, yref=0.0, zref=0.0, max_distances=[50, 50, 50]):
        self.parameters = deepcopy(parameters)
        self.m = parameters['m']
        self.g = parameters['g']
        self.A = np.array([[0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0]])
        self.B = np.array([[0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [1/self.m, 0, 0],
                           [0, 1/self.m, 0],
                           [0, 0, 1/self.m]])
        self.C = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])
        self.D = np.array([[0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0]])

        max_distances = max_distances
        self.Nx = np.diag(np.concatenate([np.array(max_distances), np.ones(3)*5]))
        self.Nu = np.diag(np.array([self.m*self.g, self.m*self.g/2, self.m*self.g/2]))
        self.Ny = np.diag(np.ones(self.C.shape[0]))
        self.normalize_system(self.Nx, self.Nu, self.Ny)

        if Ts is not None:
            self.Ts = Ts
            self.discretize_model(Ts)
            self.discretize_model(Ts, normalized_model=True)
        else:
            self.Ts = None

        self.X_OP = np.array([xref, yref, zref, 0, 0, 0])
        self.Y_OP = self.C @ self.X_OP
        self.U_OP = np.array([0, 0, self.m*self.g])

    def controllability_check(self):
        Cm = control.ctrb(self.A, self.B)
        rank = np.linalg.matrix_rank(Cm)
        print("Controllability matrix rank:", rank)

    def discretize_model(self, Ts, normalized_model=False):
        if not normalized_model:
            self.Ad = np.eye(self.A.shape[0]) + self.A * Ts
            self.Bd = self.B * Ts
            self.Cd = self.C
            self.Dd = self.D
            return self.Ad, self.Bd, self.Cd, self.Dd
        else:
            self.Adn = np.eye(self.An.shape[0]) + self.An * Ts
            self.Bdn = self.Bn * Ts
            self.Cdn = self.Cn
            self.Ddn = self.Dn
            return self.Adn, self.Bdn, self.Cdn, self.Ddn

    def discrete_prediction(self, x, u):
        delta_x = x - self.X_OP
        delta_u = u - self.U_OP
        delta_x_next = self.Ad @ delta_x + self.Bd @ delta_u
        x_next = delta_x_next + self.X_OP
        return x_next

    def update_parameters(self, parameters):
        self.parameters = deepcopy(parameters)
        self.g = parameters['g']
        self.m = parameters['m']
        self.B = np.array([[0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0],
                           [1 / self.m, 0, 0],
                           [0, 1 / self.m, 0],
                           [0, 0, 1 / self.m]])
        np.array([0, 0, self.m*self.g])
        self.normalize_system(self.Nx, self.Nu, self.Ny)
        if self.Ts is not None:
            self.discretize_model(self.Ts)
            self.discretize_model(self.Ts, normalized_model=True)

    def normalize_system(self, Nx, Nu, Ny):
        Nx_inv = np.diag(1/np.diagonal(Nx))
        Ny_inv = np.diag(1/np.diagonal(Ny))
        self.An = Nx_inv @ self.A @ Nx # matrix is diagonal so inverse is just inverse of elements
        self.Bn = Nx_inv @ self.B @ Nu
        self.Cn =  Ny_inv @ self.C @ Nx
        self.Dn =  Ny_inv @ self.D @ Nu
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
        self.x_num = self.A.shape[0]
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
    def __init__(self, parameters, Ts, yaw_ss=0.0, x_ref=0.0, y_ref=0.0, z_ref=0.0, max_distances=[50, 50, 50]):
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
        max_distances = max_distances
        self.Nx = np.diag(np.concatenate([np.array(max_distances), np.ones(3)*5]))
        self.Nu = np.diag(np.array([self.m*self.g, np.pi/6, np.pi/6]))
        self.Ny = np.diag(np.ones(self.C.shape[0]))
        self.normalize_system(self.Nx, self.Nu, self.Ny)
        if Ts is not None:
            self.Ts = Ts
            self.discretize_model(Ts)
            self.discretize_model(Ts, normalized_model=True)
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
        self.normalize_system(self.Nx, self.Nu, self.Ny)
        if self.Ts is not None:
            self.discretize_model(self.Ts)
            self.discretize_model(self.Ts, normalized_model=True)

    def discretize_model(self, Ts, normalized_model=False):
        if not normalized_model:
            self.Ad = np.eye(self.A.shape[0]) + self.A * Ts
            self.Bd = self.B * Ts
            self.Cd = self.C
            self.Dd = self.D
            return self.Ad, self.Bd, self.Cd, self.Dd
        else:
            self.Adn = np.eye(self.An.shape[0]) + self.An * Ts
            self.Bdn = self.Bn * Ts
            self.Cdn = self.Cn
            self.Ddn = self.Dn
            return self.Adn, self.Bdn, self.Cdn, self.Ddn
    def discrete_prediction(self, x, u):
        delta_x = x - self.X_OP
        delta_u = u - self.U_OP[:-1]
        delta_x_next = self.Ad@delta_x + self.Bd@delta_u
        x_next = delta_x_next + self.X_OP
        return x_next

    def normalize_system(self, Nx, Nu, Ny):
        Nx_inv = np.diag(1 / np.diagonal(Nx))
        Ny_inv = np.diag(1 / np.diagonal(Ny))
        self.An = Nx_inv @ self.A @ Nx  # matrix is diagonal so inverse is just inverse of elements
        self.Bn = Nx_inv @ self.B @ Nu
        self.Cn = Ny_inv @ self.C @ Nx
        self.Dn = Ny_inv @ self.D @ Nu

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

class LinearizedQuadNoYawWithUncertainty(LinearizedQuad):
    def __init__(self, parameters, Ts, weight_matrix, yaw_ss=0.0, x_ref=0.0, y_ref=0.0, z_ref=0.0):
        super().__init__(parameters, None, yaw_ss, x_ref, y_ref, z_ref)
        self.Ts = Ts
        self.weight_matrix = weight_matrix
        self.construct_augmented_system()
        self.discretize_model(Ts)

    def construct_augmented_system(self):
        temp1 = np.concatenate([self.A, self.weight_matrix], axis=1)
        temp2 = np.concatenate([np.zeros((self.weight_matrix.shape[1], self.A.shape[0])), np.zeros((self.weight_matrix.shape[1], self.weight_matrix.shape[1]))], axis=1)
        self.A_aug = np.concatenate([temp1, temp2], axis=0)
        self.B_aug = np.concatenate([self.B, np.zeros((self.weight_matrix.shape[1], self.B.shape[1]))], axis=0)
        self.C_aug = np.concatenate([self.C, np.zeros((self.C.shape[0], self.weight_matrix.shape[1]))], axis=1)
        self.D_aug = np.zeros((self.C.shape[0], self.A.shape[0] + self.weight_matrix.shape[1]))

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
        self.U_OP = np.array([self.m * self.g, 0.0, 0.0, self.u4_ss])
        self.construct_augmented_system()
        if self.Ts is not None:
            self.discretize_model(self.Ts)
            
    def discretize_model(self, Ts):
        self.Ad = np.eye(self.A_aug.shape[0]) + self.A_aug * Ts
        self.Bd = self.B_aug * Ts
        self.Cd = self.C_aug
        self.Dd = self.D_aug
        return self.Ad, self.Bd, self.Cd, self.Dd

if __name__ == "__main__":
    from Factories.ModelsFactory.model_parameters import Z550_parameters
    transMotion = LinearTranslationalMotionDynamics(parameters=Z550_parameters, Ts=None)
    transMotion.controllability_check()