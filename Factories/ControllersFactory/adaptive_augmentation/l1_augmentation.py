import numpy as np
from scipy.signal import butter, filtfilt
class L1_Augmentation:
    def __init__(self, predictor, adaptive_law, lp_filter):
        self.predictor = predictor
        self.adaptive_law = adaptive_law
        self.lp_filter = lp_filter

    def __call__(self, z, u):
        z_hat = self.predictor(z, u, self.lp_filter.u_l1, self.adaptive_law.sigma_hat)
        sigma_hat = self.adaptive_law(z_hat, z)
        u_l1 = self.lp_filter(sigma_hat)
        return u_l1


class L1_Predictor:
    def __init__(self, ref_model, z0, Ts, As):
        self.ref_model = ref_model
        self.z_hat = z0
        self.Ts = Ts
        self.As = As
    def __call__(self, z, u, u_l1, sigma_hat):
        error = self.z_hat - z
        component1 = self.ref_model(z, u, u_l1, sigma_hat)
        component2 = self.As @ error
        z_hat_prim = component1 + component2
        z_hat = self.z_hat + self.Ts*z_hat_prim
        self.z_hat = z_hat
        return z_hat

class L1_AdaptiveLaw:
    def __init__(self, ref_model, As, Ts):
        self.As = As
        self.Ts = Ts
        self.ref_model = ref_model
        self.As_Inv = np.linalg.inv(As)
        self.PHI = self.As_Inv @ (np.exp(As*Ts) - np.identity(As.shape[0]))
        self.PHI_Inv = np.linalg.inv(self.PHI)
        self.g_inv = np.linalg.inv(self.ref_model.g)
        self.exp_As_Ts = np.exp(self.As * self.Ts)

    def __call__(self, z_hat, z):
        error = z_hat - z
        miu = self.exp_As_Ts @ error
        self.sigma_hat = -self.g_inv @ self.PHI_Inv @ miu
        return self.sigma_hat

class L1_LowPass:
    def __init__(self, bandwidth, fs, order):
        self.bandwidth = bandwidth
        self.order = order
        self.fs = fs
        self.nyq = fs/2
        b, a = butter(order, bandwidth, btype='low', analog=False, fs=self.fs)
        self.a = a
        self.b = b

    def __call__(self, sigma_hat):
        self.u_l1 = filtfilt(self.b, self.a, sigma_hat)
        return self.u_l1


