import numpy as np

class RBF_Kernel():
    def __init__(self, length = 1):
        self.length = length
    def __call__(self, x1, x2):
        cov_mat = np.array([[self.calculate_covariance(element1, element2) for element1 in x1 ] for element2 in x2])
        return cov_mat
    def calculate_covariance(self, x1, x2):
        distance = np.linalg.norm(x1 - x2)
        return np.exp(-(distance ** 2) / (2 * self.length ** 2))