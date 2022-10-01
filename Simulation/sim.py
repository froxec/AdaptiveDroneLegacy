from numpy import linspace
from model import quadcopterModel, loadPendulum, odeSystem, system
from model_parameters import quad_parameters, pendulum_parameters
from scipy.integrate import odeint
import numpy as np
from plots import plotDataPID, plotTrajectory, groupDataFromPIDs
import attitude_control as control
from numpy import deg2rad
if __name__ == '__main__':
    quad0 = np.zeros(12)
    load0 = np.zeros(4)
    quad = quadcopterModel(quad0, quad_parameters)
    load = loadPendulum(load0, pendulum_parameters, quad.translational_accelerations, quad.state[3:6])
    deltaT = 0.001
    controler = control.quadControler(deltaT)
    state0 = np.concatenate([quad0, load0])
    #t = linspace(0, 25, 100)
    #x = odeint(odeSystem, state0, t, args=(quad, load))
    t = linspace(0, 10, 10000)
    x = np.zeros((10000, 16))
    thrust_setpoint = 1200
    attitude_setpoint = np.array([deg2rad(0), deg2rad(5), deg2rad(0)])
    for i, time in enumerate(t):
        motors = controler(attitude_setpoint, quad.state[6:9], quad.state[9:12], quad.angular_accelerations, thrust_setpoint)
        x[i] = system(np.array(motors), deltaT, quad, load)
    plotTrajectory(t, x.transpose()[0:12], 4, 3)
    #plotDataPID(t, controler, 'roll')
    plotTrajectory(t, x.transpose()[12:16], 2, 2)