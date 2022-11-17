import numpy as np
from Simulation.model_parameters import quad_parameters


class QuadAccelerationModel:
    def __init__(self, parameters):
        self.mass = parameters['m']
        self.g = parameters['g']
        self.inertia = parameters['I']
        self.arm_length = parameters['l']
        self.arm_angle = parameters['arm_angle']
        self.Kt = parameters['Kt']
        self.Kd = parameters['Kd']

    def __call__(self, x0, rotor_speeds):
        return self.calculate_acceleration(x0, rotor_speeds)

    def calculate_acceleration(self, x0, rotor_speeds):
        F = self.inputToForces(rotor_speeds)
        input_force = np.sum(F)
        phi = x0[6]
        theta = x0[7]
        psi = x0[8]
        acceleration_x = (1 / self.mass) * input_force * (
                    np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi))
        acceleration_y = (1 / self.mass) * input_force * (
                    np.cos(phi) * np.sin(psi) * np.sin(theta) - np.cos(psi) * np.sin(theta))
        acceleration_z = (1 / self.mass) * (input_force * np.cos(phi) * np.cos(theta) - self.mass * self.g)
        return acceleration_x, acceleration_y, acceleration_z

    def inputToForces(self, omega):
        self.F = np.multiply(self.Kt, omega ** 2)
        return self.F


def main():
    quad_accel_model = QuadAccelerationModel(quad_parameters)
    x0 = np.zeros(12)
    rotation = 1000
    rotor_speeds = np.array([rotation, rotation, rotation, rotation])
    print(quad_accel_model(x0, rotor_speeds))

if __name__ == "__main__":
    main()
