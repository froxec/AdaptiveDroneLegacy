from Factories.ModelsFactory.linear_models import LinearizedQuad, LinearizedQuadNoYaw, AugmentedLinearizedQuadNoYaw, \
    LinearTranslationalMotionDynamics
from Factories.ControllersFactory.position_controllers.mpc_parameters import MPC_PARAMETERS_MAPPING
from Factories.ConfigurationsFactory.modes import MPCModes
import numpy as np
from qpsolvers import solve_qp
from typing import Type
import time
class ConstrainedMPC:
    def __init__(self,
                 model,
                 freq,
                 pred_horizon,
                 normalize_system=False,
                 x_bounds={'lower': np.array([-1000, -1000, -1000, -100, -100, -100]),
                           'upper': np.array([1000, 1000, 1000, 100, 100, 100])},
                 u_bounds={'lower': np.array([-1000, -np.pi/6, -np.pi/6]),
                           'upper': np.array([1000, np.pi/6, np.pi/6])},
                 delta_x_bounds={'lower': np.array([-1000, -1000, -1000, -1000, -1000, -1000]),
                                   'upper': np.array([1000, 1000, 1000, 1000, 1000, 1000])},
                 delta_u_bounds = {'lower': np.array([-8, -np.pi/6, -np.pi/6]),
                                   'upper': np.array([8, np.pi/6, np.pi/6])}):
        self.model = model
        self.freq = freq
        self.pred_horizon = pred_horizon
        if not normalize_system:
            self.x_bounds = x_bounds
            self.u_bounds = u_bounds
            self.delta_u_bounds = delta_u_bounds
            self.delta_x_bounds = delta_x_bounds
        else:
            x_bounds['lower'], u_bounds['lower'] = self._normalize_state(x_bounds['lower'], u_bounds['lower'])
            x_bounds['upper'], u_bounds['upper'] = self._normalize_state(x_bounds['upper'], u_bounds['upper'])
            delta_x_bounds['lower'], delta_u_bounds['lower'] = self._normalize_state(x=delta_x_bounds['lower'], u=delta_u_bounds['lower'])
            delta_x_bounds['upper'], delta_u_bounds['upper'] = self._normalize_state(x=delta_x_bounds['upper'], u=delta_u_bounds['upper'])
            self.x_bounds = x_bounds
            self.u_bounds = u_bounds
            self.delta_u_bounds = delta_u_bounds
            self.delta_x_bounds = delta_x_bounds
        self.n = self.model.A.shape[0]
        self.m = self.model.B.shape[1]
        if isinstance(model, LinearizedQuadNoYaw):
            if normalize_system:
                parameters_type = 'LINEARIZED_NORMALIZED'
            else:
                parameters_type = 'LINEARIZED'
        elif isinstance(model, LinearTranslationalMotionDynamics):
            if normalize_system:
                parameters_type = 'TRANSLATIONAL_DYNAMICS_NORMALIZED'
            else:
                parameters_type = 'TRANSLATIONAL_DYNAMICS'
        self.Q_base = np.array(MPC_PARAMETERS_MAPPING[parameters_type]['Q_base']) + np.ones(6)*1e-6
        self.P_base = np.array(MPC_PARAMETERS_MAPPING[parameters_type]['P_base'])
        self.Qn_base = np.array([100, 100, 100, 100, 100, 100]) + np.ones(6)*1e-6
        self.R_base = np.ones(self.n) * 1000
        self.Q = np.diag(np.concatenate([np.tile(self.Q_base, pred_horizon-1), self.Qn_base]))
        self.P = np.diag(np.tile(self.P_base, pred_horizon))
        self.R = np.diag(np.tile(self.R_base, pred_horizon))
        self.mode = self.mode = MPCModes.CONSTRAINED
        self.setpoint = None
        self.normalize_state = normalize_system
        self.epsilon = 1e-6
        self.H, self.f = self.calculate_cost_matrices(self.Q, self.P)
        self.calculacte_nonequality_constraints()
    def calculate_cost_matrices(self, Q, R):
        top_right = np.zeros((Q.shape[0], R.shape[1]))
        bottom_left = np.zeros((R.shape[0], Q.shape[1]))
        top = np.concatenate([Q, top_right], axis=1)
        bottom = np.concatenate([bottom_left, R], axis=1)
        self.H = np.concatenate([top, bottom], axis=0)
        self.f = np.zeros(self.H.shape[0])
        return self.H, self.f

    def calculate_equality_constraints(self):
        if self.normalize_state:
            A = self.model.Adn
            B = self.model.Bdn
        else:
            A = self.model.Ad
            B = self.model.Bd
        I = np.identity(A.shape[0])
        A_map = np.eye(self.pred_horizon, dtype=int)[:-1, :]
        I_map = np.eye(self.pred_horizon, k=1, dtype=int)[:-1, :]
        left = np.kron(A_map, A) + np.kron(I_map, -I)
        right = np.kron(np.eye(self.pred_horizon - 1, dtype=int), B)
        u0_columns = np.zeros((left.shape[0], self.m))
        right = np.concatenate([u0_columns, right], axis=1)
        self.Aeq = np.concatenate([left, right], axis=1)
        self.beq = np.zeros(self.Aeq.shape[0])
        return self.Aeq, self.beq

    def calculacte_nonequality_constraints(self):
        I = np.identity(self.m)
        left = np.zeros((self.m*(self.pred_horizon-1), self.n*self.pred_horizon))
        I_map = np.eye(self.pred_horizon ,dtype=int)[:-1, :]
        I_neg_map = np.eye(self.pred_horizon, k=1, dtype=int)[:-1, :]
        right = np.kron(I_map, I) + np.kron(I_neg_map, -I)
        top_G = np.concatenate([left, right], axis=1)
        bottom_G = -top_G
        G = np.concatenate([top_G, bottom_G], axis=0)
        discrete_delta_u_lb = self.delta_u_bounds['lower'] * (1/self.freq)
        discrete_delta_u_ub = self.delta_u_bounds['upper'] * (1/self.freq)
        top_b = np.tile(discrete_delta_u_ub, self.pred_horizon - 1)
        bottom_b = -np.tile(discrete_delta_u_lb, self.pred_horizon-1)
        h = np.concatenate([top_b, bottom_b], axis=0)
        #G, h = self.add_state_ramp_constraints(G, h)
        self.G = G
        self.h = h
        return G, h

    def set_boundaries(self, x0, u0):
        lb_top = np.tile(self.x_bounds['lower'], self.pred_horizon)
        lb_top[0:6] = x0[:]
        lb_bottom = np.tile(self.u_bounds['lower'], self.pred_horizon)
        lb_bottom[0:3] = u0[:]
        ub_top = np.tile(self.x_bounds['upper'], self.pred_horizon)
        ub_top[0:6] = x0[:]
        ub_bottom = np.tile(self.u_bounds['upper'], self.pred_horizon)
        ub_bottom[0:3] = u0[:]
        lb = np.concatenate([lb_top, lb_bottom])
        ub = np.concatenate([ub_top, ub_bottom])
        self.lb = lb
        self.ub = ub
        return self.lb, self.ub
    def predict(self, delta_x, delta_u):
        x = delta_x
        u = delta_u
        if self.normalize_state:
            x, u = self._normalize_state(x, u)
        self.calculate_equality_constraints() # might be calculated only on parameters change
        lb, ub = self.set_boundaries(x, u)
        solution = solve_qp(self.H, self.f, G=self.G, h=self.h, A=self.Aeq, b=self.beq, lb=lb, ub=ub, solver='osqp')
        u_k = solution[self.pred_horizon*self.model.A.shape[0]+3:self.pred_horizon*self.model.A.shape[0]+6]# first control is dummy
        if self.normalize_state:
            u_k = self._denormalize_control(u_k)
        return u_k

    def add_state_ramp_constraints(self, G, h):
        I = np.identity(self.n)
        I_map = np.eye(self.pred_horizon, dtype=int)[:-1, :]
        I_neg_map = np.eye(self.pred_horizon,  k=1, dtype=int)[:-1, :]
        left = np.kron(I_map, I) + np.kron(I_neg_map, I)
        right = np.zeros((left.shape[0], self.m*self.pred_horizon))
        top_G = np.concatenate([left, right], axis=1)
        bottom_G = -top_G
        G_new = np.concatenate([top_G, bottom_G], axis=0)
        discrete_delta_x_lb = self.delta_x_bounds['lower'] * (1 / self.freq)
        discrete_delta_x_ub = self.delta_x_bounds['upper'] * (1 / self.freq)
        top_h = np.tile(discrete_delta_x_ub, self.pred_horizon - 1)
        bottom_h = -np.tile(discrete_delta_x_lb, self.pred_horizon - 1)
        h_new = np.concatenate([top_h, bottom_h], axis=0)
        G = np.concatenate([G, G_new], axis=0)
        h = np.concatenate([h, h_new], axis=0)
        return G, h
    def _normalize_state(self, x=None, u=None):
        if x is not None:
            x = np.diag(1/np.diagonal(self.model.Nx)) @ x
        if u is not None:
            u = np.diag(1/np.diagonal(self.model.Nu)) @ u
        return x, u

    def _denormalize_control(self, u):
        u = self.model.Nu @ u
        return u

    def switch_modes(self,
                     mode: Type[MPCModes]):
        self.mode = mode

    def change_setpoint(self, setpoint):
        self.setpoint=setpoint
        extended_ref = np.repeat(np.zeros_like(self.setpoint), self.pred_horizon, axis=1).transpose(1, 0)
        self.ref = extended_ref

if __name__ == "__main__":
    from Factories.ModelsFactory.model_parameters import arducopter_parameters, Z550_parameters
    from Factories.DataManagementFactory.data_holders import DataHolder
    OUTER_LOOP_FREQ = 10
    parameters_holder = DataHolder(Z550_parameters)
    prediction_model = LinearizedQuadNoYaw(parameters_holder, Ts=1 / OUTER_LOOP_FREQ)
    mpc = ConstrainedMPC(prediction_model,
                         OUTER_LOOP_FREQ,
                         30)
    x0 = np.array([0, 0, 10, 0, 0, 0])
    u0 = np.array((0, 0, 0))
    mpc.predict(x0, u0)