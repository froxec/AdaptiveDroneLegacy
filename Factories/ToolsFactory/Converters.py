from Simulation.attitude_control import ThrustToAngularVelocity, Normalizer


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
