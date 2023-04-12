from gekko import GEKKO
import numpy as np

class gekkoMPC():
    def __init__(self, control_horizon, outer_loop_freq, ):
     state_bounds = {'lower': [-np.Inf, -np.Inf, -np.Inf, -2.5, -2.5, -2.5],
                    'upper': [[np.Inf, np.Inf, np.Inf, 2.5, 2.5, 2.5]]}
     state0 = [0, 0, 0, 0, 0, 0]
     m = GEKKO()
     m.time = np.linspace(0, control_horizon, 1/outer_loop_freq)
     m.x = [m.CV(value=state0[i], lb=state_bounds['lower'][i], ub=state_bounds['upper'][i]) for i in range(10)]

m = GEKKO
m.time = np.linspace()