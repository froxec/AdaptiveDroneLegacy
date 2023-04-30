import numpy as np
from Factories.ToolsFactory.GeneralTools import construct_ext_obs_mat, construct_low_tril_Toeplitz
from Factories.ModelsFactory.linear_models import LinearizedQuad, LinearizedQuadNoYaw
from Factories.ModelsFactory.model_parameters import pendulum_parameters, Z550_parameters
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import Trajectory
from plotly.subplots import make_subplots
from typing import Type
import plotly.graph_objects as go
class ModelPredictiveControl():
    def __init__(self,
                 model: Type[LinearizedQuad],
                 freq: int,
                 pred_horizon: int):
        self.model = model
        self.model.discretize_model(freq)
        self.freq = freq
        self.pred_horizon = pred_horizon
        self.Q = np.diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
        self.P = np.diag([1000, 500000, 500000])
        self.Ol = construct_ext_obs_mat(self.model.Ad, self.model.Cd, horizon=self.pred_horizon)
        self.Hl = construct_low_tril_Toeplitz(self.model.Ad, self.model.Bd, self.model.Cd, horizon=self.pred_horizon)
        self.control_history = []

    def set_reference(self, ref):
        if isinstance(ref, Trajectory):
            self.trajectory = ref
            ref = self.trajectory.generated_trajectory
        if isinstance(ref, np.ndarray):
            ref = ref.reshape(-1, 1)
        else:
            ref = np.array(ref).reshape(-1, 1)
        if ref.shape[0] == 3:
            ref = np.concatenate([ref, np.zeros(3).reshape((-1, 1))], axis=0)
        extended_ref = np.repeat(ref, self.pred_horizon, axis=1).transpose(1, 0)[:, :, None]
        self.ref = extended_ref

    def calculate_feedback(self, x_k):
        # calculate pl
        temp = self.model.Ad @ x_k
        pl = self.Ol @ temp

        #calculate Fl
        temp = self.Ol @ self.model.Bd
        Fl = np.concatenate([temp[:, None, :, :], self.Hl], axis=1)

        return pl, Fl

    def calculate_cost_matrices(self, pl, Fl, ref):
        #calculate H
        Fl_transposed = Fl.transpose(0, 1, 3, 2)
        temp1 = self.Q @ Fl
        temp2 = Fl_transposed @ temp1
        H = temp2 + self.P
        #calculate f
        temp1 = (pl - ref)
        temp2 = self.Q @  temp1
        f = np.tensordot(Fl_transposed, temp2, axes=((3, 1), (1, 0)))
        #calculate J0
        J0 = np.tensordot(temp1.transpose(0, 2, 1), temp2, axes=((2, 0), (1, 0)))
        return H, f, J0

    def calculate_cost(self, H, f, J0, u):
        temp = np.tensordot(H, u, axes=((3, 1), (1, 0)))
        term1 = np.tensordot(u.transpose(0, 2, 1), temp, axes=((2, 0), (1, 0)))
        term2 = 2*np.tensordot(f.transpose(0, 2, 1), u, axes=((2, 0), (1, 0)))
        return term1 + term2 + J0

    def predict(self, delta_x0, delta_u0, setpoint):
        self.set_reference(setpoint)
        pl, Fl = self.calculate_feedback(delta_x0.reshape(-1, 1))
        H, f, J0 = self.calculate_cost_matrices(pl, Fl, self.ref)
        u = np.tensordot(-np.linalg.inv(H), f, axes=((3, 1), (1, 0)))
        u = u[0].flatten()
        self.control_history.append(list(u))
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