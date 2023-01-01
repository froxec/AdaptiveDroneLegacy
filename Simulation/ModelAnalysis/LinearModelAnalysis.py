import control
import numpy as np
from Simulation.model_parameters import *
import matplotlib.pyplot as plt
import control.optimal as obc
class LinearizedQuad(control.StateSpace):
    def __init__(self, dt=0, u4_ss=0, x_ref=0, y_ref=0, z_ref=0):
        m = quad_parameters['m']
        g = quad_parameters['g']
        A = np.array([[0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0]])
        B = np.array([[0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, g * np.sin(u4_ss), g * np.cos(u4_ss), 0],
                      [0, -g * np.cos(u4_ss), g * np.sin(u4_ss), 0],
                      [1 / m, 0, 0, 0]])
        C = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])
        D = np.array([[0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]])
        super().__init__(A, B, C, D, dt)
        self.X_OP = np.array([x_ref, y_ref, z_ref, 0, 0, 0])
        self.Y_OP = C @ self.X_OP

quad = LinearizedQuad()
quadIO = control.LinearIOSystem(quad, name='quad', inputs=('f', 'phi', 'theta', 'psi'), states=('x', 'y', 'z', 'Vx', 'Vy', 'Vz'), outputs=('x', 'y', 'z', 'Vx', 'Vy', 'Vz'))

Q = np.diag([1, 1, 1, 1, 1, 1])
R = np.diag([1, 1, 1, 1])
xf = np.array([100, 0, 0, 0, 0, 0])
uf = np.array([quad_parameters['m']*quad_parameters['g'], 0, 0, 0])
# t, y = control.impulse_response(quad)
# plt.plot(t, y[5][0])
# plt.show()
t = np.linspace(0, 10, 1, endpoint=True)
quadratic_cost = obc.quadratic_cost(quadIO, Q, R, x0=xf, u0=uf)
X0 = np.array([0, 0, 0, 0, 0, 0])
constraints = [obc.input_range_constraint(quadIO, [0, -np.pi/6, -np.pi/6, -np.inf], [20000, np.pi/6, np.pi/6, np.inf])]
res = obc.solve_ocp(quadIO, t, X0, quadratic_cost, constraints, initial_guess=np.array([0, 0, 0, 0]))