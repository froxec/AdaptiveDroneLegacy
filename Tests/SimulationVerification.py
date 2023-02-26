
from Simulation.model import quadcopterModel, loadPendulum, odeSystem, system
from ModelsFactory.model_parameters import quad_parameters, pendulum_parameters
import numpy as np
from Simulation.plots import plotDataPID, plotTrajectory, groupDataFromPIDs
import time

FPS = 30
PAUSE_INCREMENT = 1e-5

if __name__ == '__main__':
    load0 = np.zeros(4)
    quad0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, np.pi/4, 0, 0, 0])
    quad = quadcopterModel(quad0, quad_parameters)
    load = loadPendulum(load0, pendulum_parameters, quad.translational_accelerations, quad.state[3:6])
    deltaT = 0.01
    state0 = np.concatenate([quad0, load0])
    #t = linspace(0, 25, 100)
    #x = odeint(odeSystem, state0, t, args=(quad, load))
    t = np.arange(0, 10, deltaT)
    x = np.zeros((t.size, 12))
    u = np.array([945, 955, 945, 955])

    prev_stop_time = deltaT
    start = time.time()
    for i, t_i in enumerate(t):
        x[i] = system(u, deltaT, quad)
    print(time.time() - start)
    plotTrajectory(t, x.transpose()[0:12], 4, 3, fix_scale=[7, 10, 9, 12])

    time.sleep(0)