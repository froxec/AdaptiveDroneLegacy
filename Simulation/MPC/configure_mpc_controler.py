from ModelsFactory.linear_models import LinearizedQuad
from pyMPC.mpc import MPCController
import scipy.sparse as sparse
import numpy as np

class MPC_configurator():
    def __init__(self, quad_parameters, sample_time=0.01, linear_model=LinearizedQuad()):
        self.Ts = sample_time
        self.quad = linear_model

        Ac = self.quad.A
        Bc = self.quad.B
        [nx, nu] = Bc.shape #state, input number

        # Discretization
        self.Ad = np.eye(nx) + Ac*self.Ts
        self.Bd = Bc*self.Ts
        
        #reference values
        self.xref = np.array([0, 10, 100, 0, 0, 0])
        self.uref = np.array([0, 0, 0, 0])
        self.uminus1 = np.array([0, 0.0, 0.0, 0.0])

        # Constraints
        self.xmin = np.array([-np.inf, -np.inf, -np.inf, -10, -10, -10])
        self.xmax = np.array([np.inf, np.inf, np.inf, 10, 10, 10])

        self.umin = np.array([-quad_parameters['m']*quad_parameters['g'], -np.pi/6, -np.pi/6, -np.inf])
        self.umax = np.array([quad_parameters['m']*quad_parameters['g'], np.pi/6, np.pi/6, np.inf])

        self.Dumin = np.array([-5, -np.pi/12, -np.pi/12, -np.pi/12])
        self.Dumax = np.array([5, np.pi/12, np.pi/12, np.pi/12])

        # Cost matrices

        Qx = sparse.diags([100000, 10, 100, 1, 1, 1])  # Quadratic cost for states x0, x1, ..., x_N-1
        QxN = sparse.diags([1, 1, 1, 1, 1, 1])  # Quadratic cost for xN
        Qu = sparse.diags([10, 0, 0, 0])  # Quadratic cost for u0, u1, ...., u_N-1
        QDu = sparse.diags([0, 0, 0, 0])  # Quadratic cost for Du0, Du1, ...., Du_N-1