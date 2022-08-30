import matplotlib.pyplot as plt
import numpy as np
def plotQuadTrajectory(t, x):
    x = x.transpose()
    f1, axs1 = plt.subplots(4, 3)
    f2, axs2 = plt.subplots(2, 2)
    for i, ax in enumerate(axs1.reshape(-1)):
        ax.plot(t, x[i]) 
    for i, ax in enumerate(axs2.reshape(-1)):
        ax.plot(t, x[i + 12])
    plt.show()