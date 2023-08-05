import numpy as np
class NonlinearTranslationalModel:
    def __init__(self,
                 parameters_holder):
        self.parameters_holder = parameters_holder

    def __call__(self, force_norm, angles):
        c = np.cos
        s = np.sin
        phi = angles[0]
        theta = angles[1]
        psi = angles[2]

        fx = force_norm * (c(phi)*s(theta)*c(psi) + s(phi)*s(psi))
        fy = force_norm * (c(phi)*s(psi)*s(theta) - c(psi)*s(phi))
        fz = force_norm * (c(phi)*c(theta)) - self.parameters_holder.m * self.parameters_holder.g

        accelerations_hat = (1/self.parameters_holder.m) * np.array([fx, fy, fz])
        return accelerations_hat
