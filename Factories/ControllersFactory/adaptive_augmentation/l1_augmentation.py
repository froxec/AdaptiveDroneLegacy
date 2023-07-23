import numpy as np
from scipy.signal import butter, filtfilt
from Factories.ToolsFactory.GeneralTools import LowPassLiveFilter
class L1_Augmentation:
    def __init__(self, predictor, adaptive_law, lp_filter, converter):
        self.predictor = predictor
        self.adaptive_law = adaptive_law
        self.lp_filter = lp_filter
        self.converter = converter

    def __call__(self, z, z_prev, u, u_prev):
        u = self.converter.convert_to_vector(u[0], u[1:])
        u_prev = self.converter.convert_to_vector(u_prev[0], u_prev[1:])
        z_hat = self.predictor(z_prev, u_prev, self.lp_filter.u_l1, self.adaptive_law.sigma_hat)
        sigma_hat = self.adaptive_law(z_hat, z)
        print(sigma_hat)
        u_l1 = self.lp_filter(sigma_hat)
        u_composite = u + u_l1
        u_composite = self.converter.convert_from_vector(u_composite)
        return u_composite


class L1_Predictor:
    def __init__(self, ref_model, z0, Ts, As):
        self.ref_model = ref_model
        self.z_hat = z0
        self.Ts = Ts
        self.As = As
    def __call__(self, z_prev, u_prev, u_l1_prev, sigma_hat_prev):
        error = self.z_hat - z_prev
        component1 = self.ref_model(z_prev, u_prev, u_l1_prev, sigma_hat_prev)
        component2 = self.As @ error
        z_hat_prim = component1 + component2
        z_hat = self.z_hat + self.Ts*z_hat_prim
        self.z_hat = z_hat
        return self.z_hat

class L1_AdaptiveLaw:
    def __init__(self, ref_model, Ts, As):
        self.As = As
        self.Ts = Ts
        self.ref_model = ref_model
        self.As_Inv = np.linalg.inv(As)
        self.exp_As_Ts = np.diag(np.exp(np.diag(self.As * self.Ts)))
        self.PHI = self.As_Inv @ (self.exp_As_Ts - np.identity(As.shape[0]))
        self.PHI_Inv = np.linalg.inv(self.PHI)
        self.g_inv = 1/self.ref_model.m
        self.sigma_hat = np.zeros(3)

    def __call__(self, z_hat, z):
        error = z_hat - z
        miu = self.exp_As_Ts @ error
        self.sigma_hat = - self.g_inv * self.PHI_Inv @ miu
        return self.sigma_hat


class L1_LowPass(LowPassLiveFilter):
    
    def __init__(self, bandwidth, fs, signals_num, no_filtering=False):
        super().__init__(bandwidth, fs, signals_num)
        self.u_l1 = np.zeros(self.signals_num)
        self.no_filtering = no_filtering
    def __call__(self, x):
        if self.no_filtering == False:
            u_l1 = -self.process(x)
            self.u_l1 = u_l1
        else:
            u_l1 = -x
        return u_l1
class L1_ControlConverter:
    def __init__(self):
        self.epsilon = 1e-15

    def convert_to_vector(self, force_norm, angles):
        c = np.cos
        s = np.sin
        phi = angles[0]
        theta = angles[1]
        psi = angles[2]

        fx = force_norm * (c(phi)*s(theta)*c(psi) + s(phi)*s(psi))
        fy = force_norm * (c(phi)*s(psi)*s(theta) + c(psi)*s(phi))
        fz = force_norm * (c(phi)*c(theta))
        return np.array([fx, fy, fz])

    def convert_from_vector(self, force):
        f_xz = force[[0, 2]]
        f_yz = force[[1, 2]]
        fx = force[0]
        fy = force[1]
        force_norm = np.linalg.norm(force)
        phi = np.arcsin(fy / (np.linalg.norm(f_yz) + self.epsilon))
        theta = np.arcsin(fx / (np.linalg.norm(f_xz) + self.epsilon))
        return np.array([force_norm, phi, theta])

if __name__ == "__main__":
    vector = np.array([100, 0, 10])
    converter = L1_ControlConverter()
    vector_angles = converter.convert_from_vector(vector)
    vector_converted = converter.convert_to_vector(vector_angles[0], np.concatenate([vector_angles[1:], np.zeros(1)]))
    print("Original vector", vector)
    print("Angles vector", vector_angles)
    print("Vector after conversion", vector_converted)

