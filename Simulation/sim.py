from numpy import linspace
from model import quadcopterModel
from model_parameters import quad_parameters, pendulum_parameters
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':
    x0 = np.zeros(12)
    quad = quadcopterModel(x0, quad_parameters)
    t = linspace(0, 25, 100)
    x = odeint(quad.model_ode, x0, t)
    plt.plot(t, x[:, 7])
    plt.show()