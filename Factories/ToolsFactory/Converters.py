from Simulation.attitude_control import ThrustToAngularVelocity, Normalizer
import numpy as np

class MPC_output_converter():
    def __init__(self, u_ss, Kt, angular_velocity_range):
        self.angular_vel_min = angular_velocity_range[0]
        self.angular_vel_max = angular_velocity_range[1]
        self.u_ss = u_ss
        self.thrust_converter = ThrustToAngularVelocity(Kt)
        self.angular_vel_normalizer = Normalizer(min=self.angular_vel_min, max=self.angular_vel_max)
    def __call__(self, delta_u):
        u = delta_u + self.u_ss
        omega = self.thrust_converter(u[0])
        throttle = self.angular_vel_normalizer(omega)
        u[0] = throttle
        #u[3] = 0
        return u

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
