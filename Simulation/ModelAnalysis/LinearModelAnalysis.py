import control
import numpy as np
from ModelsFactory.model_parameters import *
import matplotlib.pyplot as plt
import control.optimal as obc
import time
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

if __name__ == "__main__":
    quad = LinearizedQuad()
    poles = quad.poles()
    quadIO = control.LinearIOSystem(quad, name='quad', inputs=('f', 'phi', 'theta', 'psi'), states=('x', 'y', 'z', 'Vx', 'Vy', 'Vz'), outputs=('x', 'y', 'z', 'Vx', 'Vy', 'Vz'), )

    Q = np.diag([1, 1, 1, 10, 10, 10])
    R = np.diag([1, 1, 10, 10])
    P = np.diag([1, 1, 10, 10, 10, 10])
    xf = np.array([5, 5, 10, 0, 0, 0])
    uf = np.array([0, 0, 0, 0])
    Tf = 10
    # t, y = control.impulse_response(quad)
    # plt.plot(t, y[2][0])
    # plt.show()
    t = np.linspace(.1, Tf, 3, endpoint=True)
    quadratic_cost = obc.quadratic_cost(quadIO, Q, R, x0=xf, u0=uf)
    term_cost = obc.quadratic_cost(quadIO, P, 0, x0=xf)
    X0 = np.array([0., 0., 0., 0., 0., 0.])
    constraints = [obc.input_range_constraint(quadIO, np.array([-quad_parameters['m']*quad_parameters['g'], -np.pi/6, -np.pi/6, -np.inf]), np.array([20, np.pi/6, np.pi/6, np.inf]))]
    t1 = time.time()
    res = obc.solve_ocp(quadIO, t, X0, cost=quadratic_cost, constraints=constraints, terminal_cost=term_cost, minimize_method='SLSQP')
    print("Solved in time:", time.time()-t1)
    resp = control.input_output_response(
        quadIO, t, res.inputs, X0,
        t_eval=np.linspace(0.1, Tf, 10))
    t, y, u = resp.time, resp.outputs, resp.inputs
    print(res.success)
    plt.subplot(3, 2, 1)
    plt.plot(t, y[0])
    #plt.plot(X0[0], X0[1], 'ro', xf[0], xf[1], 'ro')
    plt.xlabel("t [sec]")
    plt.ylabel("x [m]")

    plt.subplot(3, 2, 3)
    plt.plot(t, y[1])
    #plt.plot(X0[0], X0[1], 'ro', xf[0], xf[1], 'ro')
    plt.xlabel("t [sec]")
    plt.ylabel("y [m]")

    plt.subplot(3, 2, 5)
    plt.plot(t, y[2])
    #plt.plot(X0[0], X0[1], 'ro', xf[0], xf[1], 'ro')
    plt.xlabel("t [sec]")
    plt.ylabel("z [m]")

    plt.subplot(3, 2, 2)
    plt.plot(t, y[3])
    #plt.plot(X0[0], X0[1], 'ro', xf[0], xf[1], 'ro')
    plt.xlabel("t [sec]")
    plt.ylabel("Vx [m]")

    plt.subplot(3, 2, 4)
    plt.plot(t, y[4])
    #plt.plot(X0[0], X0[1], 'ro', xf[0], xf[1], 'ro')
    plt.xlabel("t [sec]")
    plt.ylabel("Vy [m]")

    plt.subplot(3, 2, 6)
    plt.plot(t, y[5])
    #plt.plot(X0[0], X0[1], 'ro', xf[0], xf[1], 'ro')
    plt.xlabel("t [sec]")
    plt.ylabel("Vz [m]")

    plt.show(block=False)
    fig = plt.figure()
    plt.subplot(4, 1, 1)
    plt.step(t, u[0])
    plt.xlabel("t [sec]")
    plt.ylabel("F [N]")

    plt.subplot(4, 1, 2)
    plt.step(t, u[1])
    plt.xlabel("t [sec]")
    plt.ylabel("phi [rad]")

    plt.subplot(4, 1, 3)
    plt.step(t, u[2])
    plt.xlabel("t [sec]")
    plt.ylabel("theta [rad]")

    plt.subplot(4, 1, 4)
    plt.step(t, u[3])
    plt.xlabel("t [sec]")
    plt.ylabel("psi [rad]")

    plt.suptitle("Wznos na 100 m")
    plt.tight_layout()
    plt.show()