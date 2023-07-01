import numpy as np
from scipy.linalg import solve_continuous_lyapunov
from tools import RadialBasisFunction
import itertools
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from Simulation.model import RungeKutta4
class WingRockSystem:
    def __init__(self):
        self.A = np.array([[0, 1],
                           [0, 0]])
        self.B = np.array([[0],
                           [1]])
        self.x0 = np.array([[0],
                            [0]])
        self.state = self.x0
    def updateStateOde(self, x, u):
        dstate = self.A @ x + self.B @ u
        return dstate
class ReferenceModel:
    def __init__(self, Ts=None):
        self.A = np.array([[0, 1],
                           [-4, -2]])
        self.B = np.array([[0],
                           [4]])
        self.x0 = np.array([[0],
                            [0]])
        self.Ad = None
        self.Bd = None
        self.x_hat = None
        self.Ts = Ts
        if Ts is not None:
            self.discretize_model(Ts)

    def discretize_model(self, Ts):
        self.Ad = np.eye(self.A.shape[0]) + self.A * Ts
        self.Bd = self.B * Ts

    def predict(self, x, u):
        if self.Ad is None:
            raise "You have to discretize the model in order to make predictions..."
        self.x_hat = self.Ad @ x + self.Bd @ u
        return self.x_hat
class FeedbackFeedforwardController:
    def __init__(self):
        self.Kx = np.array([[4, 2]])
        self.Kr = np.array([[4]])

    def __call__(self, x, r):
        u = - self.Kx @ x + self.Kr @ r
        return u

class ExternalDisturbance:
    def __init__(self, weights, temporal=60, temporal_weight=10):
        self.weights = np.array(weights).astype(float)
        self.basis_functions = [lambda t: np.ones_like(t), lambda t: t, lambda t: np.abs(t)*t, lambda p: np.abs(p)*p,
                                lambda t: np.power(t, 3)]
        self.temporal = temporal
        self.temporal_weight = temporal_weight
        self.history = []
    def __call__(self, x, t=None):
        delta = 0
        theta = x[0]
        p = x[1]
        args = [theta, theta, theta, p, theta]
        for i in range(len(self.weights)):
            delta += self.weights[i]*self.basis_functions[i](args[i])
        if self.temporal is not None and t is not None:
            delta += self.temporal_weight*np.tanh(t/self.temporal)
        delta = delta.item()
        self.history.append(delta)
        return delta

class RBFUncertaintyEstimator:
    def __init__(self, N, cen_min, cen_max, bandwidth, m):
        self.N = N
        self.cen_min = cen_min
        self.cen_max = cen_max
        self.bandwidth = bandwidth
        self.rbfs = []
        self.generate_rbfs()
        self.weights = np.zeros((len(self.rbfs) + m, m))
        self.current_features = np.zeros(len(self.rbfs) + m)
    def __call__(self, x, un):
        feat = []
        for rbf in self.rbfs:
            feat.append(rbf(x))
        feat = np.array(feat)
        un = un.flatten()
        feat = np.concatenate([feat, un])
        self.current_features = feat
        u_ad = self.weights.T @ feat.reshape((-1, 1))
        u_ad = u_ad.item()
        return u_ad
    def generate_rbfs(self):
        cen1d = np.linspace(self.cen_min, self.cen_max, self.N)
        cen2d = []
        for center in itertools.product(cen1d, cen1d):
            self.rbfs.append(RadialBasisFunction(np.array(center), self.bandwidth))
        return self.rbfs

class EstimatorUpdater:
    def __init__(self, estimator, P, Bp, gamma):
        self.est = estimator
        self.PBp = P @ Bp
        self.gamma = gamma
    def __call__(self, xp, xr):
        update = (xp - xr).reshape(-1, 1).T @ self.PBp
        update = self.est.current_features.reshape(-1, 1) @ update
        update = self.gamma * update
        self.est.weights = self.est.weights + update

class GPUncertaintyEstimator:
    def __init__(self):
        pass
    def __call__(self):
        pass

class AdaptiveControl:
    def __init__(self, ref_model, unc_estimator):
        self.gamma = 0.1
        self.Q = -np.identity(ref_model.A.shape[0])
        self.P = solve_continuous_lyapunov(ref_model.A, self.Q)
        self.ref_model = ref_model
        self.est = unc_estimator
        self.updater = EstimatorUpdater(self.est, self.P, ref_model.B, self.gamma)
        self.history = []

    def __call__(self, x, un, ref):
        ad = self.est(x, un)
        self.history.append(ad)
        return un - ad
    def update_estimator(self, x, x_prev, ref):
        x_hat = self.ref_model.predict(x_prev, ref)
        self.updater(x, x_hat)

def simulate(start, stop, dT, model, ff_controller, ada_controller=None, ext_distb=None):
    t = np.arange(start, stop, dT)
    x = np.zeros((t.shape[0], model.A.shape[0]))
    u_history = np.zeros(t.shape[0]-1)
    ref = np.array([10.0])
    x[0] = model.x0.flatten()
    for i in range(1, t.shape[0]):
        u = ff_controller(model.state, ref)
        if ada_controller is not None:
            u = ada_controller(model.state, u, ref)
        u_history[i - 1] = u
        if ext_distb is not None:
            delta = ext_distb(model.state, t[i])
            u = u + delta
        RungeKutta4(dT, model, u.reshape(-1, 1))
        x[i] = model.state.flatten()
        if ada_controller is not None:
            ada_controller.update_estimator(model.state.flatten(), x[i-1], ref)
    return t, x, u_history

def plot_trajectories(t, x, u, ad, dist_history=None):
    fig_state = make_subplots(2, 1)
    for i in range(x.shape[1]):
        fig_state.add_trace(go.Scatter(x=t, y=x[:, i]), row=i + 1, col=1)
    fig_control = go.Figure()
    fig_control.add_trace(go.Scatter(x=t, y=u))
    fig_ad = go.Figure()
    fig_ad.add_trace(go.Scatter(x=t, y=ad))
    if dist_history is not None:
        fig_dist = go.Figure()
        fig_dist.add_trace(go.Scatter(x=t, y=dist_history))
    fig_state.show()
    fig_control.show()
    fig_ad.show()
    fig_dist.show()

if __name__ == "__main__":
    dT = 0.01
    ref_model = ReferenceModel(dT)
    plant = WingRockSystem()
    ext_disturbance = ExternalDisturbance(weights=[10, 1, 1, 1, 1])
    ff_controller = FeedbackFeedforwardController()
    estimator = RBFUncertaintyEstimator(10, -2, 2, 25, ref_model.B.shape[1])
    ada_controller = AdaptiveControl(ref_model, estimator)
    t, x, u = simulate(0, 10, dT, plant, ff_controller, ada_controller=ada_controller, ext_distb=ext_disturbance)
    plot_trajectories(t, x, u, ada_controller.history, ext_disturbance.history)