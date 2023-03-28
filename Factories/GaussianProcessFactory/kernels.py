import numpy as np
import itertools
class RBF_Kernel():
    def __init__(self, length = 1):
        self.length = length
    def __call__(self, x1, x2):
        x = np.array(list(itertools.product(x1, x2)))
        cov_mat = self.calculate_covariance(x[:, 0, :], x[:, 1, :])
        cov_mat = cov_mat.reshape((int(np.sqrt(cov_mat.shape[0])), -1))
        return cov_mat
    def calculate_covariance(self, x1, x2):
        distance = np.linalg.norm(x1 - x2, axis=1)
        return np.exp(-(distance ** 2) / (2 * self.length ** 2))