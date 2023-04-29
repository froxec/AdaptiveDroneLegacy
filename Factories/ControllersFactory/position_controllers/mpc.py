import numpy as np
from Factories.ToolsFactory.GeneralTools import construct_ext_obs_mat, construct_low_tril_Toeplitz
from Factories.ModelsFactory.linear_models import LinearizedQuad, LinearizedQuadNoYaw
from Factories.ModelsFactory.model_parameters import pendulum_parameters, Z550_parameters
from typing import Type
class ModelPredictiveControl():
    def __init__(self,
                 model: Type[LinearizedQuad],
                 freq: int,
                 pred_horizon: int):
        self.model = model
        self.freq = freq
        self.pred_horizon = pred_horizon
        self.Q = np.diag([1, 1, 1, 1, 1, 1])
        self.P = np.diag([1, 1, 1])
        self.Ol = construct_ext_obs_mat(self.model.Ad, self.model.Cd, horizon=self.pred_horizon)
        self.Hl = construct_low_tril_Toeplitz(self.model.Ad, self.model.Bd, self.model.Cd, horizon=self.pred_horizon)

    def set_reference(self, ref):
        self.ref = ref

    def calculate_feedback(self, x_k):
        # calculate pl
        temp = self.model.Ad @ x_k
        pl = np.dot(self.Ol, temp)[:, :, None]

        #calculate Fl
        temp = np.dot(self.Ol, self.model.Bd)
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
        f = np.tensordot(Fl_transposed, temp2, axes=((1, 3), (0, 1)))
        #calculate J0
        J0 = np.tensordot(temp1.transpose(0, 2, 1), temp2, axes=((0, 2), (0, 1)))
        return H, f, J0

    def calculate_cost(self, H, f, J0, u):
        temp = np.tensordot(H, u, axes=((1, 3), (0, 1)))
        term1 = np.tensordot(u.transpose(0, 2, 1), temp, axes=((0, 2), (0, 1)))
        term2 = 2*np.tensordot(f.transpose(0, 2, 1), u, axes=((0, 2), (0, 1)))
        return term1 + term2 + J0
if __name__ == "__main__":
    freq = 10
    horizon = 10
    ref = np.array([0, 0, 10, 0, 0, 0]).reshape(-1, 1)
    extended_ref = np.repeat(ref, horizon, axis=1).transpose(1, 0)[:, :, None]
    prediction_model = LinearizedQuadNoYaw(Z550_parameters)
    prediction_model.discretize_model(freq)
    mpc = ModelPredictiveControl(prediction_model, freq, 10)
    pl, Fl = mpc.calculate_feedback(np.array([1, 1, 1, 1, 1, 1]))
    H, f, J0 = mpc.calculate_cost_matrices(pl, Fl, extended_ref)
    u = np.tensordot(-np.linalg.inv(H), f, axes=((1, 3), (0, 1)))
    cost = mpc.calculate_cost(H, f, J0, u)
    print(cost)