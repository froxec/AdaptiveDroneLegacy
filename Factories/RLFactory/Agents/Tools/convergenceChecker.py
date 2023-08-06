from collections import Counter, deque

import numpy as np


class ConvergenceChecker:
    def __init__(self, n, epsilon):
        self.n = n
        self.epsilon = epsilon
        self.average = None
        self.deque = deque(maxlen=self.n)
        self.counter = None

    def __call__(self, action):
        self.deque.appendleft(action)
        converged = self.check_for_convergence()
        return converged

    def check_for_convergence(self):
        if len(self.deque) < self.n:
            return False
        counter = Counter(self.deque)
        values = list(counter.keys())
        counts = list(counter.values())
        average = np.average(values, weights=counts)
        unique_values = np.unique(values)
        conditions = [element > average-self.epsilon and element < average+self.epsilon for element in unique_values]
        if all(conditions):
            self.average = average
            return True
        else:
            return False

