import numpy as np
from copy import deepcopy
class MPC_output_converter():
    def __init__(self, u_ss, Kt, angular_velocity_range):
        self.angular_vel_min = angular_velocity_range[0]
        self.angular_vel_max = angular_velocity_range[1]
        self.u_ss = u_ss
        self.thrust_converter = ThrustToAngularVelocity(Kt)
        self.angular_vel_normalizer = Normalizer(min=self.angular_vel_min, max=self.angular_vel_max)
        self.nominal_u = None
    def __call__(self, delta_u):
        u = delta_u + self.u_ss
        self.nominal_u = deepcopy(u)
        omega = self.thrust_converter(u[0])
        throttle = self.angular_vel_normalizer(omega)
        u[0] = throttle
        #u[3] = 0
        return u

    def update(self, u_ss, Kt):
        self.u_ss = u_ss
        self.thrust_converter = ThrustToAngularVelocity(Kt)
class MPC_input_converter():
    def __init__(self, x_ss, u_ss):
        self.x_ss = x_ss
        self.u_ss = u_ss

    def __call__(self, x0, u0):
        if u0 is None:
            u0 = np.array([0, 0, 0])
        delta_x0 = x0 - self.x_ss
        delta_u0 = u0 - self.u_ss
        return delta_x0, delta_u0

    def update(self, u_ss):
        self.u_ss = u_ss

def convert_trajectory(velocity_trajectory, input_trajectory, deltaT):
    ## pitch, roll, yaw are ommited (not required)
    velocity_trajectory = np.flip(velocity_trajectory, axis=0)
    input_trajectory = np.flip(input_trajectory)
    average_input = np.mean(input_trajectory)
    V_norm = np.linalg.norm(velocity_trajectory, axis=1)
    acceleration = (V_norm[1:] - V_norm[:-1])/deltaT
    average_acceleration = np.mean(acceleration)
    return average_input, average_acceleration
    '''
    :param trajectory: subsequent state x and input u samples 
    :return: trajectory converted to average throttle input and average acceleration over time horizon
    '''


class Saturation():
    def __init__(self, lower_limit, upper_limit):
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit
    def __call__ (self, signal):
        if isinstance(signal,np.ndarray):
            for i in range(signal.shape[0]):
                if signal[i] < self.lower_limit:
                    signal[i] = self.lower_limit
                elif signal[i] > self.upper_limit:
                    signal[i] = self.upper_limit
        else:
            if signal < self.lower_limit:
                signal = self.lower_limit
            elif signal > self.upper_limit:
                signal = self.upper_limit
        return signal


class RateOfChangeSaturation():
    def __init__(self, rate_limit, deltaT):
        self.rate_limit = rate_limit
        self.deltaT = deltaT

    def __call__(self, value, value_prev):
        value = np.array(value)
        value_prev = np.array(value_prev)
        for i in range(value.shape[0]):
            rate = (value[i] - value_prev[i]) / self.deltaT
            if rate > self.rate_limit:
                value[i] = value_prev[i] + self.rate_limit*self.deltaT
            elif rate < -self.rate_limit:
                value[i] = value_prev[i] - self.rate_limit * self.deltaT
        return value


class LinearScaler():
    def __init__(self, min, max):
        self.min = min
        self.max = max
    def __call__(self, normalized_signal):
        scaled_signal = normalized_signal*(self.max - self.min) + self.min
        return scaled_signal


class Normalizer():
    def __init__(self, min, max):
        self.min = min
        self.max = max
    def __call__(self, signal):
        normalized_signal = (signal - self.min)/(self.max - self.min)
        return normalized_signal


class ThrustToAngularVelocity():
    def __init__(self, Kt):
        self.Kt = Kt
    def __call__(self, thrust):
        if thrust < 0:
            thrust = 0
        angular_velocity = np.sqrt(0.25*thrust/self.Kt)
        return angular_velocity

class AngularVelocityToThrust():
    def __init__(self, Kt):
        self.Kt = Kt
    def __call__(self, angular_velocity):
        if angular_velocity < 0:
            thrust = 0
        thrust = 4*self.Kt*angular_velocity**2
        return thrust
