from pyMPC.mpc import MPCController
from Factories.ModelsFactory.model_parameters import *
import scipy.sparse as sparse
import time
import matplotlib.pyplot as plt
from Factories.ModelsFactory.linear_models import LinearizedQuad

class ModelPredictiveController():
    def __init__(self, quad_parameters, x0, xref, Ts, linear_model=LinearizedQuad, save_history=False):
        self.Ts = Ts
        self.linear_model = linear_model(quad_parameters, x_ref=xref[0], y_ref=xref[1], z_ref=xref[2])
        self.Ac = self.linear_model.A
        self.Bc = self.linear_model.B
        self.x_num, self.u_num = self.Bc.shape
        self.Ad = np.eye(self.x_num) + self.Ac*self.Ts
        self.Bd = self.Bc*Ts
        #reference values
        self.xref = xref
        self.uminus1 = np.array([0, 0.0, 0.0, 0.0])

        #constraints
        self.xmin = np.array([-np.inf, -np.inf, -np.inf, -10, -10, -10])
        self.xmax = np.array([np.inf, np.inf, np.inf, 10, 10, 10])

        self.umin = np.array([-quad_parameters['m'] * quad_parameters['g'], -np.pi / 6, -np.pi / 6, -np.inf])
        self.umax = np.array([quad_parameters['m'] * quad_parameters['g'], np.pi / 6, np.pi / 6, np.inf])

        self.Dumin = np.array([-2, -np.pi *0.025 , -np.pi *0.025, -np.pi *0.025])
        self.Dumax = np.array([5, np.pi *0.025, np.pi *0.025, np.pi *0.025])

        #cost parameters
        self.Qx = sparse.diags([0.5,0.5, 3, 0.5, 0.5, 0.5])  # Quadratic cost for states x0, x1, ..., x_N-1
        self.QxN = sparse.diags([1, 1, 1, 0, 0, 0])  # Quadratic cost for xN
        self.Qu = sparse.diags([0.5, 50, 50, 50])  # Quadratic cost for u0, u1, ...., u_N-1
        self.QDu = sparse.diags([0, 0, 0, 0])  # Quadratic cost for Du0, Du1, ...., Du_N-1

        self.x0 = x0
        self.x = x0
        self.u_prev = self.uminus1
        self.Np = 50

        self.MPC = MPCController(self.Ad, self.Bd, Np=self.Np, x0=self.x0, xref=self.xref, uminus1=self.uminus1,
                                 Qx=self.Qx, QxN=self.QxN, Qu=self.Qu, QDu=self.QDu,
                                 xmin=self.xmin, xmax=self.xmax, umin=self.umin, umax=self.umax, Dumin=self.Dumin,
                                 Dumax=self.Dumax)
        self.MPC.setup()

        self.history = []
    def update_state_control(self, x):
        self.x = x
        self.MPC.update(self.x, self.u_prev)
        u = self.MPC.output()
        self.u_prev = u
        self.save_history(u)
        return u
    def save_history(self, u):
        self.history.append(u)
