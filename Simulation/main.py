from numpy import linspace
from model import model
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    t = linspace(0, 25, 100)
    x0 = np.zeros(12)
    x = odeint(model, x0, t)
    print(x.shape)
    plt.plot(t, x[:, 5])
    plt.show()