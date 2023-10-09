import numpy as np
import itertools
class RBF_Kernel():
    def __init__(self, length = 1, uncertainty=1):
        self.length = length
        self.uncertainty = uncertainty
    def __call__(self, x1, x2):
        #x = np.array(list(itertools.product(x1, x2)))
        x1 = x1.flatten()
        x2 = x2.flatten()
        x = self.cartesian_product(*(x1, x2))
        cov_mat = self.calculate_covariance(x[:, 0], x[:, 1])
        cov_mat = cov_mat.reshape((x1.shape[0], -1))
        return cov_mat
    def calculate_covariance(self, x1, x2):
        #distance = np.linalg.norm(x1 - x2, axis=0)
        distance = np.sqrt((x1-x2)**2)
        return self.uncertainty*np.exp(-(distance ** 2) / (2 * self.length ** 2))

    def cartesian_product(self, *arrays):
        ## function from https://stackoverflow.com/questions/11144513/cartesian-product-of-x-and-y-array-points-into-single-array-of-2d-points
        arrays_num = len(arrays)
        dtype = np.result_type(*arrays)
        arr = np.empty([len(a) for a in arrays] + [arrays_num], dtype=dtype)
        for i, a in enumerate(np.ix_(*arrays)):
            arr[..., i] = a
        return arr.reshape(-1,arrays_num)