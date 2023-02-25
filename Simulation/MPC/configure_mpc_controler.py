from ModelsFactory.linear_models import LinearizedQuad
from QuadcopterIntegration.Utilities.dronekit_commands import *
from pyMPC.mpc import MPCController
import scipy.sparse as sparse
import numpy as np

class MPC_configurator():
    def __init__(self, quad_parameters, x0, xref, sample_time=2, linear_model=LinearizedQuad):
        self.Ts = sample_time
        self.quad = linear_model(x_ref=xref[0], y_ref=xref[1], z_ref=xref[2])

        Ac = self.quad.A
        Bc = self.quad.B
        [nx, nu] = Bc.shape #state, input number

        # Discretization
        self.Ad = np.eye(nx) + Ac*self.Ts
        self.Bd = Bc*self.Ts
        
        #reference values
        self.xref = xref
        self.uminus1 = np.array([0, 0.0, 0.0, 0.0])

        # Constraints
        self.xmin = np.array([-np.inf, -np.inf, -np.inf, -10, -10, -10])
        self.xmax = np.array([np.inf, np.inf, np.inf, 10, 10, 10])

        self.umin = np.array([-quad_parameters['m']*quad_parameters['g'], -np.pi/6, -np.pi/6, -np.inf])
        self.umax = np.array([quad_parameters['m']*quad_parameters['g'], np.pi/6, np.pi/6, np.inf])

        self.Dumin = np.array([-5, -np.pi/12, -np.pi/12, -np.pi/12])
        self.Dumax = np.array([5, np.pi/12, np.pi/12, np.pi/12])

        # Cost matrices

        self.Qx = sparse.diags([10, 10, 10, 1, 1, 1])  # Quadratic cost for states x0, x1, ..., x_N-1
        self.QxN = sparse.diags([10, 10, 10, 10, 10, 10])  # Quadratic cost for xN
        self.Qu = sparse.diags([10, 10, 10, 10])  # Quadratic cost for u0, u1, ...., u_N-1
        self.QDu = sparse.diags([0, 0, 0, 0])  # Quadratic cost for Du0, Du1, ...., Du_N-1

        self.x0 = x0
        self.x = x0
        self.u_prev = self.uminus1
        self.Np = 3

        self.MPC = MPCController(self.Ad, self.Bd, Np=self.Np, x0=self.x0, xref=self.xref, uminus1=self.uminus1,
                    Qx=self.Qx, QxN=self.QxN, Qu=self.Qu, QDu=self.QDu,
                    xmin=self.xmin, xmax=self.xmax, umin=self.umin, umax=self.umax, Dumin=self.Dumin, Dumax=self.Dumax)
        self.MPC.setup()
    
    def update_state_control(self, x):
        self.x = x
        self.MPC.update(self.x, self.u_prev)
        u = self.MPC.output()
        self.u_prev = u
        return u