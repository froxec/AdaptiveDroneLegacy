from numpy import linspace
from model import quadcopterModel, loadPendulum, odeSystem
from model_parameters import quad_parameters, pendulum_parameters
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt



if __name__ == '__main__':
    quad0 = np.zeros(12)
    load0 = np.zeros(4)
    quad = quadcopterModel(quad0, quad_parameters)
    load = loadPendulum(load0, pendulum_parameters, quad.translational_accelerations)
    t = linspace(0, 25, 100)
    x = odeint(odeSystem, np.concatenate([quad0, load0]), t, args=(quad, load))
    plt.plot(t, x[:, 2])
    plt.show()