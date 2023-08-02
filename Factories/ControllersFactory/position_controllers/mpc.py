import numpy as np
from qpsolvers import solve_qp
from Factories.ToolsFactory.GeneralTools import construct_ext_obs_mat, construct_low_tril_Toeplitz, euclidean_distance
from Factories.ModelsFactory.linear_models import LinearizedQuad, LinearizedQuadNoYaw, AugmentedLinearizedQuadNoYaw
from Factories.ModelsFactory.model_parameters import pendulum_parameters, Z550_parameters
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import Trajectory
from Factories.ConfigurationsFactory.modes import MPCModes
from plotly.subplots import make_subplots
from typing import Any, Type
import plotly.graph_objects as go



class ModelPredictiveControl():
    def __init__(self,
                 model: Type[LinearizedQuad],
                 freq: int,
                 pred_horizon: int,
                 normalize_system=False):
        self.model = model
        self.freq = freq
        self.pred_horizon = pred_horizon
        # self.Q_base = np.array([10, 10, 10, 0.1, 0.1, 0.1]) + np.ones(6)*1e-6
        # self.P_base = np.array([0.01, 0.1, 0.1]) #  self.P_base = np.array([1, 200, 200]) #linearized normalized
        self.Q_base = np.array([10, 10, 10, 10, 10, 10]) + np.ones(6) * 1e-6
        self.P_base = np.array([10, 10, 0.01]) #TranslationalDynamics non normalized
        # self.Q_base = np.array([10, 10, 10, 1, 1, 1]) + np.ones(6) * 1e-6
        # self.P_base = np.array([10, 10, 0.01]) #  self.P_base = np.array([1, 200, 200])
        self.Q = np.diag(np.tile(self.Q_base, pred_horizon))
        self.P = np.diag(np.tile(self.P_base, pred_horizon))
        self.Ol = construct_ext_obs_mat(self.model.Ad, self.model.Cd, horizon=self.pred_horizon)
        self.Hl = construct_low_tril_Toeplitz(self.model.Ad, self.model.Bd, self.model.Cd,
                                              horizon=self.pred_horizon)
        self.mode = MPCModes.UNCONSTRAINED
        self.control_history = []
        self.prev_delta_x = np.zeros(6)
        self.prev_y = np.zeros(6)
        self.prev_delta_u = np.zeros(3)
        self.setpoint = None
        self.normalize_state = normalize_system
        self.epsilon = 1e-6

    def calculate_feedback(self, x_k):
        if self.normalize_state:
            A = self.model.Adn
            B = self.model.Bdn
        else:
            A = self.model.Ad
            B = self.model.Bd
        # calculate pl
        temp = A @ x_k
        pl = self.Ol @ temp

        #calculate Fl
        temp = self.Ol @ B
        Fl = np.concatenate([temp, self.Hl], axis=1)

        return pl, Fl

    def calculate_cost_matrices(self, pl, Fl, ref):
        ref = ref.reshape(ref.shape[0]*ref.shape[1], -1)
        #calculate H
        Fl_transposed = Fl.transpose(1, 0)
        temp1 = self.Q @ Fl
        temp2 = Fl_transposed @ temp1
        H = temp2 + self.P
        #calculate f
        temp1 = (pl - ref)
        temp2 = self.Q @  temp1
        f = Fl_transposed @ temp2
        #calculate J0
        J0 = temp1.transpose(1, 0) @ temp2
        return H, f, J0

    def calculate_cost(self, H, f, J0, u):
        temp = np.tensordot(H, u, axes=((3, 1), (1, 0)))
        term1 = np.tensordot(u.transpose(0, 2, 1), temp, axes=((2, 0), (1, 0)))
        term2 = 2*np.tensordot(f.transpose(0, 2, 1), u, axes=((2, 0), (1, 0)))
        return term1 + term2 + J0

    def predict(self, delta_x0):
        if isinstance(self.model, AugmentedLinearizedQuadNoYaw):
            x = delta_x0 - self.prev_delta_x
            x = np.concatenate([x, self.prev_y])
        else:
            x = delta_x0
        if self.normalize_state:
            x = self._normalize_state(x)
        pl, Fl = self.calculate_feedback(x.reshape(-1, 1))
        H, f, J0 = self.calculate_cost_matrices(pl, Fl, self.ref)
        if self.mode == MPCModes.UNCONSTRAINED:
            # u = np.tensordot(-np.linalg.inv(H), f, axes=((3, 1), (1, 0)))
            u = np.linalg.solve(H, -f)
            u = u.reshape((self.pred_horizon, -1))
            # prediction = self.prediction(pl, Fl, u)
            u = u[0]
        elif self.mode == MPCModes.UNCONSTRAINED_WITH_SOLVER:
            solution = solve_qp(H, f, solver="quadprog")
            u = solution.reshape((self.pred_horizon, -1))[0]
        elif self.mode == MPCModes.CONSTRAINED:
            lb, ub = self.bounds()
            solution = solve_qp(H, f, lb=lb, ub=ub, solver="quadprog")
            u = solution.reshape((self.pred_horizon, -1))[0]
        if isinstance(self.model, AugmentedLinearizedQuadNoYaw):
            u_k = u + self.prev_delta_u
        else:
            u_k = u
        if self.normalize_state:
            u_k = self._denormalize_control(u_k)
        self.control_history.append(list(u_k))
        self.prev_delta_x = delta_x0
        self.prev_y = self.model.Cd @ x
        self.prev_delta_u = u_k
        return u_k

    def flatten_problem(self, H, f):
        Hb = H.transpose(0, 2, 1, 3).reshape((H.shape[0]*H.shape[2], H.shape[1]*H.shape[3]))
        f = f.reshape((f.shape[0]*f.shape[1]))
        return Hb, f

    def bounds(self):
        u_constraints = np.array([self.model.parameters['m']*self.model.parameters['g'], np.pi/6, np.pi/6])
        lb = -np.tile(u_constraints, self.pred_horizon)
        ub = np.tile(u_constraints, self.pred_horizon)
        return lb, ub

    def prediction(self, pl, Fl, u):
        Fl = Fl.transpose(0, 2, 1, 3).reshape((Fl.shape[0] * Fl.shape[2], Fl.shape[1] * Fl.shape[3]))
        u = u.reshape(-1, 1)
        temp = Fl @ u
        return pl.reshape((self.pred_horizon, -1)) + temp.reshape((self.pred_horizon, -1))

    def switch_modes(self,
                     mode: Type[MPCModes]):
        self.mode = mode

    def update_model_parameters(self, parameters):
        ## TODO update converters
        self.model.update_parameters(parameters)

    def change_setpoint(self, setpoint):
        self.setpoint=setpoint
        extended_ref = np.repeat(np.zeros_like(self.setpoint), self.pred_horizon, axis=1).transpose(1, 0)
        self.ref = extended_ref
    def _normalize_state(self, x):
        x = np.diag(1/np.diagonal(self.model.Nx)) @ x
        return x

    def _denormalize_control(self, u):
        u = self.model.Nu @ u
        return u
    def plot_history(self):
        fig = make_subplots(rows=3, cols=1, x_title='Czas [s]',
                            subplot_titles=('Thrust', 'Phi',
                                            'Theta'))
        fig.add_trace(go.Scatter(x=list(range(len(self.control_history))), y=np.array(self.control_history)[:, 0], name="Thrust"), row=1, col=1)
        fig.add_trace(go.Scatter(x=list(range(len(self.control_history))), y=np.array(self.control_history)[:, 1],name="Phi"), row=2, col=1)
        fig.add_trace(go.Scatter(x=list(range(len(self.control_history))), y=np.array(self.control_history)[:, 2], name="Theta"), row=3, col=1)
        fig.show()

if __name__ == "__main__":
    freq = 10
    horizon = 10
    prediction_model = LinearizedQuadNoYaw(Z550_parameters)
    mpc = ModelPredictiveControl(prediction_model, freq, 10)
    u = mpc.predict(np.zeros(6), [], np.array([0, 10, 10, 0, 0, 0]))
    print(u)