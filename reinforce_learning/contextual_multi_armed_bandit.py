from Factories.GaussianProcessFactory.kernels import  RBF_Kernel
from Factories.GaussianProcessFactory.gaussian_process import GaussianProcess
from Factories.ToolsFactory.GeneralTools import plot_signal
import numpy as np
if __name__ == "__main__":
    observations = 10
    samples_num = 100
    domain = (-5, 5)
    X0 = np.linspace(domain[0], domain[1], samples_num).reshape(-1, 1)

    # observations
    x1 = np.random.uniform(domain[0], domain[1], size=(observations, 1))
    y1 = np.sin(x1).flatten()

    rbf_kernel = RBF_Kernel(length=1)
    gp = GaussianProcess(X0, rbf_kernel, noise_std=0)

    gp(x1, y1)
    gp.plot()
