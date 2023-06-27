import numpy as np
from scipy.linalg import solve_continuous_lyapunov
from tools import RadialBasisFunction
class WingRockSystem:
    def __init__(self):
        self.A = np.array([[0, 1],
                           [0, 0]])
        self.B = np.array([[0],
                           [1]])
        self.x0 = np.array([[0],
                            [0]])
class ReferenceModel:
    def __init__(self):
        self.A = np.array([[0, 1],
                           [-4, -2]])
        self.B = np.array([[0],
                           [4]])
        self.x0 = np.array([[0],
                            [0]])

class FeedbackFeedforwardController:
    def __init__(self):
        self.Kx = np.array([[-4, -2]])
        self.Kr = np.array([[4]])

    def __call__(self, x, r, u_ad):
        u = - self.Kx @ x + self.Kr @ r - u_ad
        return u

class ExternalDisturbance:
    def __init__(self, weights):
        self.weights = weights
        self.basis_functions = [lambda t: np.ones_like(t), lambda t: t, lambda t: np.abs(t)*t, lambda p: np.abs(p)*p,
                                lambda t: np.power(t, 3)]
    def __call__(self, x):
        delta = 0
        theta = x[0]
        p = x[1]
        args = [theta, theta, theta, p, theta]
        for i in range(len(self.weights)):
            delta += self.weights[i]*self.basis_functions[i](args[i])
        return delta

class RBFUncertaintyEstimator:
    def __init__(self, N, cen_min, cen_max, bandwidth):
        self.N = N
        self.cen_min = cen_min
        self.cen_max = cen_max
        self.bandwidth = bandwidth

    def generate_rbfs(self):
        pass



class AdaptiveControl:
    def __init__(self, ref_model, unc_estimator):
        self.gamma = 10
        self.Q = -np.identity(ref_model.A.shape[0])
        self.P = solve_continuous_lyapunov(ref_model.A, self.Q)
        print(self.P)

if __name__ == "__main__":
    ref_model = ReferenceModel()
    plant = WingRockSystem()
    ff_controller = FeedbackFeedforwardController()
    ada_controller = AdaptiveControl(ref_model, ff_controller)