from pyMPC.mpc import MPCController
import numpy as np
from Simulation.model_parameters import *
import scipy.sparse as sparse
import time
import matplotlib.pyplot as plt

class LinearizedQuad():
    def __init__(self, dt=0, u4_ss=0, x_ref=0, y_ref=0, z_ref=0):
        self.m = quad_parameters['m']
        self.g = quad_parameters['g']
        self.A = np.array([[0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0]])
        self.B = np.array([[0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, self.g * np.sin(u4_ss), self.g * np.cos(u4_ss), 0],
                      [0, -self.g * np.cos(u4_ss), self.g * np.sin(u4_ss), 0],
                      [1 / self.m, 0, 0, 0]])
        self.C = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])
        self.D = np.array([[0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]])
        self.X_OP = np.array([x_ref, y_ref, z_ref, 0, 0, 0])
        self.Y_OP = self.C @ self.X_OP

if __name__ == "__main__":
    #parameters
    Ts = 0.01
    quad = LinearizedQuad()
    Ac = quad.A
    Bc = quad.B
    [nx, nu] = Bc.shape #state, input number

    # Discretization
    Ad = np.eye(nx) + Ac*Ts
    Bd = Bc*Ts
    xref = np.array([0, 10, 100, 0, 0, 0])
    uref = np.array([0, 0, 0, 0])
    uminus1 = np.array([0, 0.0, 0.0, 0.0])
    # Constraints
    xmin = np.array([-np.inf, -np.inf, -np.inf, -10, -10, -10])
    xmax = np.array([np.inf, np.inf, np.inf, 10, 10, 10])

    umin = np.array([-quad_parameters['m']*quad_parameters['g'], -np.pi/6, -np.pi/6, -np.inf])
    umax = np.array([quad_parameters['m']*quad_parameters['g'], np.pi/6, np.pi/6, np.inf])

    Dumin = np.array([-5, -np.pi/12, -np.pi/12, -np.pi/12])
    Dumax = np.array([5, np.pi/12, np.pi/12, np.pi/12])

    Qx = sparse.diags([100000, 10, 100, 1, 1, 1])  # Quadratic cost for states x0, x1, ..., x_N-1
    QxN = sparse.diags([1, 1, 1, 1, 1, 1])  # Quadratic cost for xN
    Qu = sparse.diags([10, 0, 0, 0])  # Quadratic cost for u0, u1, ...., u_N-1
    QDu = sparse.diags([0, 0, 0, 0])  # Quadratic cost for Du0, Du1, ...., Du_N-1
    x0 = np.zeros(6)
    x0[2] = 100
    Np = 5

    K = MPCController(Ad, Bd, Np=Np, x0=x0, xref=xref, uminus1=uminus1,
                      Qx=Qx, QxN=QxN, Qu=Qu, QDu=QDu,
                      xmin=xmin, xmax=xmax, umin=umin, umax=umax, Dumin=Dumin, Dumax=Dumax)
    K.setup()  # this initializes the QP problem for the first step

    # Simulate in closed loop. Use MPC model as real system

    [nx, nu] = Bd.shape  # number of states and number or inputs
    len_sim = 20  # simulation length (s)
    nsim = int(len_sim / Ts)  # simulation length(timesteps)
    xsim = np.zeros((nsim, nx))
    usim = np.zeros((nsim, nu))
    tsim = np.arange(0, nsim) * Ts

    time_start = time.time()

    xstep = x0
    uMPC = uminus1
    for i in range(nsim):
        xsim[i, :] = xstep

        # MPC update and step. Could be in just one function call
        K.update(xstep, uMPC)  # update with measurement
        uMPC = K.output()  # MPC step (u_k value)
        uMPC[3] = 0
        usim[i, :] = uMPC

        xstep = Ad.dot(xstep) + Bd.dot(uMPC)  # Real system step (x_k+1 value)

    time_sim = time.time() - time_start
    y = xsim
    t = tsim
    u = usim
    plt.subplot(3, 2, 1)
    plt.plot(t, y[:, 0])
    # plt.plot(X0[0], X0[1], 'ro', xf[0], xf[1], 'ro')
    plt.xlabel("t [sec]")
    plt.ylabel("x [m]")

    plt.subplot(3, 2, 3)
    plt.plot(t, y[:, 1])
    # plt.plot(X0[0], X0[1], 'ro', xf[0], xf[1], 'ro')
    plt.xlabel("t [sec]")
    plt.ylabel("y [m]")

    plt.subplot(3, 2, 5)
    plt.plot(t, y[:, 2])
    # plt.plot(X0[0], X0[1], 'ro', xf[0], xf[1], 'ro')
    plt.xlabel("t [sec]")
    plt.ylim([0, 105])
    plt.ylabel("z [m]")

    plt.subplot(3, 2, 2)
    plt.plot(t, y[:, 3])
    # plt.plot(X0[0], X0[1], 'ro', xf[0], xf[1], 'ro')
    plt.xlabel("t [sec]")
    plt.ylabel("Vx [m]")

    plt.subplot(3, 2, 4)
    plt.plot(t, y[:, 4])
    # plt.plot(X0[0], X0[1], 'ro', xf[0], xf[1], 'ro')
    plt.xlabel("t [sec]")
    plt.ylabel("Vy [m]")

    plt.subplot(3, 2, 6)
    plt.plot(t, y[:, 5])
    # plt.plot(X0[0], X0[1], 'ro', xf[0], xf[1], 'ro')
    plt.xlabel("t [sec]")
    plt.ylabel("Vz [m]")

    plt.suptitle("Trajektoria zmiennych stanu")

    plt.show(block=False)
    fig = plt.figure()
    plt.subplot(4, 1, 1)
    plt.step(t, u[:, 0])
    plt.xlabel("t [sec]")
    plt.ylabel("F [N]")

    plt.subplot(4, 1, 2)
    plt.step(t, u[:, 1])
    plt.xlabel("t [sec]")
    plt.ylabel("phi [rad]")

    plt.subplot(4, 1, 3)
    plt.step(t, u[:, 2])
    plt.xlabel("t [sec]")
    plt.ylabel("theta [rad]")

    plt.subplot(4, 1, 4)
    plt.step(t, u[:, 3])
    plt.xlabel("t [sec]")
    plt.ylabel("psi [rad]")

    plt.suptitle("Trajektoria zmiennych sterujących")
    plt.show()