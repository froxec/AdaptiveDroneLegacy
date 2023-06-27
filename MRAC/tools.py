import numpy as np
class RadialBasisFunction:
    def __init__(self, center, bandwidth):
        self.center = center
        self.bandwidth = bandwidth

    def __call__(self, x):
        diff = x - self.center
        dist = np.transpose(diff) @ diff
        return np.exp(-dist/self.bandwidth)