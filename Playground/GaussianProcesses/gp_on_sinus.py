from Factories.GaussianProcessFactory.kernels import  RBF_Kernel
from Factories.GaussianProcessFactory.gaussian_process import GaussianProcess
from Factories.ToolsFactory.GeneralTools import plot_signal
import numpy as np
if __name__ == "__main__":
    samples_num = 100
    observations = 5
    domain = (0, 2*3.14)
    X0 = np.linspace(domain[0], domain[1], samples_num).reshape(-1, 1)

    # observations
    x1 = np.random.uniform(domain[0], domain[1], size=(1, 1))
    y1 = np.sin(x1).flatten()

    rbf_kernel = RBF_Kernel(length=2)
    gp = GaussianProcess(X0, rbf_kernel, noise_std=0)
    gp(x1, y1)
    gp.plot()
    for i in range(observations):
        best = gp.Thompson_sampling()
        action = best['best_action']
        y = np.sin(action).flatten()
        gp(np.array([action]).reshape(-1, 1), y)
        gp.plot()