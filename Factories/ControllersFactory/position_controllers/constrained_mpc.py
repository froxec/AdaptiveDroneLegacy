from Factories.ModelsFactory.linear_models import LinearizedQuad, LinearizedQuadNoYaw, AugmentedLinearizedQuadNoYaw, \
    LinearTranslationalMotionDynamics
from Factories.ControllersFactory.position_controllers.mpc_parameters import MPC_PARAMETERS_MAPPING
from Factories.ConfigurationsFactory.modes import MPCModes
import numpy as np
from qpsolvers import solve_qp
import qpSWIFT
from typing import Type
import time
class ConstrainedMPC:
    def __init__(self,
                 model,
                 freq,
                 pred_horizon,
                 normalize_system=False,
                 x_bounds={'lower': np.array([-100000, -100000, -100000, -5, -5, -5]),
                           'upper': np.array([100000, 100000, 100000, 5, 5, 5])},
                 u_bounds={'lower': np.array([-1000, -np.pi/6, -np.pi/6]),
                           'upper': np.array([1000, np.pi/6, np.pi/6])},
                 delta_x_bounds={'lower': np.array([-1000, -1000, -1000, -1000, -1000, -1000]),
                                   'upper': np.array([1000, 1000, 1000, 1000, 1000, 1000])},
                 delta_u_bounds = {'lower': np.array([-3, -np.pi/12, -np.pi/12]),
                                   'upper': np.array([3, np.pi/12,np.pi/12])},
                 soft_constraints=True):
        self.model = model
        self.freq = freq
        self.pred_horizon = pred_horizon
        self.soft_constraints = soft_constraints
        self.opts = {'MAXITER':100,'VERBOSE':0,'OUTPUT':2}
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
        self.Qn_base = np.array(MPC_PARAMETERS_MAPPING[parameters_type]['Q_base'])*10 + np.ones(6)*1e-6
        self.R_base = np.ones(self.n) * 1000000
        self.Q = np.diag(np.concatenate([np.tile(self.Q_base, pred_horizon-1), self.Qn_base]))
        self.P = np.diag(np.tile(self.P_base, pred_horizon))
        self.R = np.diag(np.tile(self.R_base, pred_horizon - 1))
        self.mode = self.mode = MPCModes.CONSTRAINED
        self.setpoint = None
        self.normalize_state = normalize_system
        self.epsilon = 1e-6
        if self.soft_constraints:
            self.H, self.f = self.calculate_cost_matrices(self.Q, self.P, self.R)
        else:
            self.H, self.f = self.calculate_cost_matrices(self.Q, self.P)
        self.calculacte_nonequality_constraints()
        self.calculate_equality_constraints() # might be calculated only on parameters change
        self.G, self.h = self._add_boundaries(self.G, self.h)
    def calculate_cost_matrices(self, Q, R, P=None):
        top_right = np.zeros((Q.shape[0], R.shape[1]))
        bottom_left = np.zeros((R.shape[0], Q.shape[1]))
        top = np.concatenate([Q, top_right], axis=1)
        bottom = np.concatenate([bottom_left, R], axis=1)
        self.H = np.concatenate([top, bottom], axis=0)
        if P is not None:
            soft_left = np.zeros((P.shape[0], self.H.shape[1]))
            soft_top = np.zeros((self.H.shape[0], P.shape[1]))
            H_top = np.concatenate([self.H, soft_top], axis=1)
            H_bottom = np.concatenate([soft_left, P], axis=1)
            self.H = np.concatenate([H_top, H_bottom], axis=0)
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
        if self.soft_constraints:
            right_fill = np.zeros((self.Aeq.shape[0], self.R.shape[1]))
            self.Aeq = np.concatenate([self.Aeq, right_fill], axis=1)
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
        G, h = self.add_state_ramp_constraints(G, h)
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
        if self.soft_constraints:
            self.lb = np.concatenate([self.lb, -np.ones(self.R.shape[0])*np.Inf])
            self.ub = np.concatenate([self.ub, np.ones(self.R.shape[0])*np.Inf])
        return self.lb, self.ub

    def _add_boundaries(self, G, h):
        I = np.identity(self.n)
        Imap = np.eye(self.pred_horizon, dtype=int)
        G_x_top = np.kron(Imap, I)
        G_x_bottom = np.kron(Imap, -I)
        h_x_top = np.tile(self.x_bounds['upper'], self.pred_horizon)
        h_x_bottom = -np.tile(self.x_bounds['lower'], self.pred_horizon)
        G_x_left = np.concatenate([G_x_top, G_x_bottom], axis=0)
        G_x_right = np.zeros((G_x_left.shape[0], G.shape[1] - G_x_left.shape[1]))
        G_x = np.concatenate([G_x_left, G_x_right], axis=1)
        h_x = np.concatenate([h_x_top, h_x_bottom])
        I = np.identity(self.m)
        G_u_top = np.kron(Imap, I)
        G_u_bottom = np.kron(Imap, -I)
        h_u_top = np.tile(self.u_bounds['upper'], self.pred_horizon)
        h_u_bottom = -np.tile(self.u_bounds['lower'], self.pred_horizon)
        G_u_middle = np.concatenate([G_u_top, G_u_bottom], axis=0)
        G_u_left = np.zeros((G_u_middle.shape[0], G_x_left.shape[1]))
        G_u_left = np.concatenate([G_u_left, G_u_middle], axis=1)
        G_u_right = np.zeros((G_u_middle.shape[0], G.shape[1] - G_u_left.shape[1]))
        G_u = np.concatenate([G_u_left, G_u_right], axis=1)
        h_u = np.concatenate([h_u_top, h_u_bottom])
        G = np.concatenate([G, G_x, G_u], axis=0)
        h = np.concatenate([h, h_x, h_u])
        return G, h
    def _add_initial_condition(self, Aeq, beq, x, u):
        x_left = np.identity(self.n)
        x_right = np.zeros((self.n, self.Aeq.shape[1] - x_left.shape[1]))
        x0_constraint = np.concatenate([x_left, x_right], axis=1)
        u_middle = np.identity(self.m)
        u_left = np.zeros((u_middle.shape[0], self.n*self.pred_horizon))
        u_left = np.concatenate([u_left, u_middle], axis=1)
        u_right = np.zeros((u_middle.shape[0], Aeq.shape[1] - u_left.shape[1]))
        u0_constraint = np.concatenate([u_left, u_right], axis=1)
        Aeq = np.concatenate([Aeq, x0_constraint, u0_constraint], axis=0)
        beq = np.concatenate([beq, x, u])
        return Aeq, beq
    def predict(self, delta_x, delta_u):
        import time
        t1 = time.time()
        x = delta_x
        u = delta_u
        if self.normalize_state:
            x, u = self._normalize_state(x, u)
        Aeq, beq = self._add_initial_condition(self.Aeq, self.beq, x, u)
        #lb, ub = self.set_boundaries(x, u)
        #solution = solve_qp(self.H, self.f, G=self.G, h=self.h, A=Aeq, b=beq, solver='osqp') #lb and ub must be here, for state feedback
        solution = qpSWIFT.run(P=self.H, c=self.f, G=self.G, h=self.h, A=Aeq, b=beq, opts=self.opts)['sol']
        u_k = solution[self.pred_horizon*self.model.A.shape[0]+3:self.pred_horizon*self.model.A.shape[0]+6]# first control is dummy
        if self.normalize_state:
            u_k = self._denormalize_control(u_k)
        print(time.time() - t1)
        return u_k

    def add_state_ramp_constraints(self, G, h):
        I = np.identity(self.n)
        I_map = np.eye(self.pred_horizon, dtype=int)[:-1, :]
        I_neg_map = np.eye(self.pred_horizon,  k=1, dtype=int)[:-1, :]
        left = np.kron(I_map, I) + np.kron(I_neg_map, -I)
        right = np.zeros((left.shape[0], self.m*self.pred_horizon))
        top_G = np.concatenate([left, right], axis=1)
        bottom_G = -top_G
        G_new = np.concatenate([top_G, bottom_G], axis=0)
        discrete_delta_x_lb = self.delta_x_bounds['lower'] * (1 / self.freq)
        discrete_delta_x_ub = self.delta_x_bounds['upper'] * (1 / self.freq)
        top_h = np.tile(discrete_delta_x_ub, self.pred_horizon - 1)
        bottom_h = -np.tile(discrete_delta_x_lb, self.pred_horizon - 1)
        h_new = np.concatenate([top_h, bottom_h], axis=0)
        if self.soft_constraints:
            G_top_fill = np.zeros((G.shape[0], self.R.shape[1]))
            G = np.concatenate([G, G_top_fill], axis=1)
            I = np.identity(self.n)
            I_map = np.eye(self.pred_horizon-1, dtype=int)
            G_new_soft_constraints_top = np.kron(I_map, I)
            G_new_soft_constraints_bottom = np.kron(I_map, -I)
            G_new_soft_constraints = np.concatenate([G_new_soft_constraints_top, G_new_soft_constraints_bottom], axis=0)
            G_new = np.concatenate([G_new, G_new_soft_constraints], axis=1)
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
    x0 = np.array([0, -50, 0, 0, 1.9, 0])
    u0 = np.array((0, 0, 0))
    mpc.predict(x0, u0)