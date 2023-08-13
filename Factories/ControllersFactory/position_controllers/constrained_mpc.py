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
                 x_bounds={'lower': np.array([-np.Inf, -np.Inf, -np.Inf, -3, -3, -3]),
                           'upper': np.array([np.Inf, np.Inf, np.Inf, 3, 3, 3])},
                 u_bounds={'lower': np.array([-np.Inf, -np.pi/6, -np.pi/6]),
                           'upper': np.array([np.Inf, np.pi/6, np.pi/6])}):
        self.model = model
        self.freq = freq
        self.pred_horizon = pred_horizon
        self.x_bounds = x_bounds
        self.u_bounds = u_bounds
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
        self.Q = np.diag(np.tile(self.Q_base, pred_horizon))
        self.P = np.diag(np.tile(self.P_base, pred_horizon-1))
        self.mode = self.mode = MPCModes.CONSTRAINED
        self.setpoint = None
        self.normalize_state = normalize_system
        self.epsilon = 1e-6
        self.H, self.f = self.calculate_cost_matrices(self.Q, self.P)
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
        self.Aeq = np.concatenate([left, right], axis=1)
        self.beq = np.zeros(self.Aeq.shape[0])
        return self.Aeq, self.beq

    def set_boundaries(self, x0):
        lb_top = np.tile(self.x_bounds['lower'], self.pred_horizon)
        lb_bottom = np.tile(self.u_bounds['lower'], self.pred_horizon - 1)
        ub_top = np.tile(self.x_bounds['upper'], self.pred_horizon)
        ub_bottom = np.tile(self.u_bounds['upper'], self.pred_horizon - 1)
        lb = np.concatenate([lb_top, lb_bottom])
        ub = np.concatenate([ub_top, ub_bottom])
        lb[0:6] = x0[:]
        ub[0:6] = x0[:]
        self.lb = lb
        self.ub = ub
        return self.lb, self.ub
    def predict(self, delta_x):
        t1 = time.time()
        x = delta_x
        if self.normalize_state:
            x = self._normalize_state(x)
        self.calculate_equality_constraints() # might be calculated only on parameters change
        lb, ub = self.set_boundaries(x)
        solution = solve_qp(self.H, self.f, A=self.Aeq, b=self.beq, lb=lb, ub=ub, solver='osqp')
        return solution[self.pred_horizon*self.model.A.shape[0]:self.pred_horizon*self.model.A.shape[0]+3]


    def _normalize_state(self, x):
        x = np.diag(1/np.diagonal(self.model.Nx)) @ x
        return x

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
    mpc.predict(x0)