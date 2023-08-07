import numpy as np
from copy import deepcopy
from typing import Type
from Factories.DataManagementFactory.data_holders import DataHolder
class MPC_output_converter():
    def __init__(self, parameters_holder: Type[DataHolder], angular_velocity_range, mode='proprietary'):
        self.parameters_holder = parameters_holder
        self.angular_vel_min = angular_velocity_range[0]
        self.angular_vel_max = angular_velocity_range[1]
        self.thrust_converter = ThrustToAngularVelocity(self.parameters_holder.Kt)
        self.angular_vel_normalizer = Normalizer(min=self.angular_vel_min, max=self.angular_vel_max)
        self.nominal_u = None
        self.valid_modes = {'proprietary', 'ardupilot', 'transDynamicsModel'}
        if mode not in self.valid_modes:
            raise ValueError("mode must be one of %r." % self.valid_modes)
        self.mode = mode
        if self.mode == 'transDynamicsModel':
            self.u_ss = np.array([0, 0, self.parameters_holder.m * self.parameters_holder.g])
        elif self.mode == 'proprietary':
            self.u_ss = np.array([self.parameters_holder.m * self.parameters_holder.g, 0, 0])
        self.epsilon = 1e-15
    def __call__(self, delta_u, throttle=True):
        u = delta_u + self.u_ss
        self.nominal_u = deepcopy(u)
        if self.mode == 'transDynamicsModel':
            print("Before conversion:", u)
            u = self.convert_force_vector_to_u(u)
            print("After  conversion:", u)
        if throttle:
            omega = self.thrust_converter(u[0])
            throttle = self.angular_vel_normalizer(omega)
            u[0] = throttle
            #u[3] = 0
        return u

    def convert_throttle(self, u):
        u = deepcopy(u)
        omega = self.thrust_converter(u[0])
        throttle = self.angular_vel_normalizer(omega)
        u[0] = throttle
        # u[3] = 0
        return u

    def convert_force_vector_to_u(self, force):
        f_xz = force[[0, 2]]
        f_yz = force[[1, 2]]
        fx = force[0]
        fy = force[1]
        force_norm = np.sign(force[2])*np.linalg.norm(force)
        phi = -np.arcsin(fy / (np.linalg.norm(f_yz) + self.epsilon))
        theta = np.arcsin(fx / (np.linalg.norm(f_xz) + self.epsilon))
        return np.array([force_norm, phi, theta])

    def convert(self):
        ## TODO x
        pass

    def update(self):
        if self.mode == 'transDynamicsModel':
            self.u_ss = np.array([0, 0, self.parameters_holder.m * self.parameters_holder.g])
        elif self.mode == 'proprietary':
            self.u_ss = np.array([self.parameters_holder.m * self.parameters_holder.g, 0, 0])
        self.thrust_converter = ThrustToAngularVelocity(self.parameters_holder.Kt)
        print("Output converter parameters updated!")

class MPC_input_converter():
    def __init__(self, x_ss, parameters_holder, mode='proprietary'):
        self.parameters_holder = parameters_holder
        self.x_ss = x_ss
        self.valid_modes = {'proprietary', 'ardupilot', 'transDynamicsModel'}
        if mode not in self.valid_modes:
            raise ValueError("mode must be one of %r." % self.valid_modes)
        self.mode = mode
        if self.mode == 'transDynamicsModel':
            self.u_ss = np.array([0, 0, self.parameters_holder.m * self.parameters_holder.g])
        elif self.mode == 'proprietary':
            self.u_ss = np.array([self.parameters_holder.m * self.parameters_holder.g, 0, 0])
    def __call__(self, x0, u0):
        if u0 is None:
            u0 = np.array([0, 0, 0])
        delta_x0 = x0 - self.x_ss
        delta_u0 = u0 - self.u_ss
        return delta_x0, delta_u0

    def update(self, x_ss=None, update_u_ss=False):
        if x_ss is not None:
            self.x_ss = x_ss
        if update_u_ss:
            if self.mode == 'transDynamicsModel':
                self.u_ss = np.array([0, 0, self.parameters_holder.m * self.parameters_holder.g])
            elif self.mode == 'proprietary':
                self.u_ss = np.array([self.parameters_holder.m * self.parameters_holder.g, 0, 0])
            print("Input converter parameters updated!")
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


class RampSaturation:
    def __init__(self, slope, Ts):
        if not isinstance(slope, dict):
            raise ValueError("slope should be dict with upper_bound, and lower_bound keys")
        self.Ts = Ts
        self.slope_lb = slope['lower_bound']
        self.slope_ub = slope['upper_bound']
        self.prev = np.zeros_like(self.slope_lb)
    def __call__(self, curr):
        if self.prev is None:
            self.prev = curr
        if (curr.flatten().shape[0] != self.slope_lb.flatten().shape[0] or
            self.prev.flatten().shape[0] != self.slope_ub.flatten().shape[0]):
            raise ValueError("Length of signal vector and slope limit vector should be equal.. {} != {}".format(curr.flatten().shape, self.slope_max.flatten().shape))
        derivative = (curr - self.prev) / self.Ts
        output = np.zeros_like(derivative)
        derivative = derivative.flatten()
        for i in range(derivative.shape[0]):
            if derivative[i] < self.slope_lb[i]:
                output[i] = self.prev[i] + self.slope_lb[i] * self.Ts
            elif derivative[i] > self.slope_ub[i]:
                output[i] = self.prev[i] + self.slope_ub[i] * self.Ts
            else:
                output[i] = curr[i]
        self.prev = output
        return output

if __name__ == "__main__":
    u = np.array([0, 0, -10])
    converter = MPC_output_converter(None, None, [None, None])
    u = converter.convert_force_vector_to_u(u)
