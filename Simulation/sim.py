from numpy import linspace
from model import quadcopterModel, loadPendulum, odeSystem, system
from model_parameters import quad_parameters, pendulum_parameters
from scipy.integrate import odeint
import numpy as np
from plots import plotQuadTrajectory
import attitude_control as control
from numpy import deg2rad
if __name__ == '__main__':
    quad0 = np.zeros(12)
    load0 = np.zeros(4)
    quad = quadcopterModel(quad0, quad_parameters)
    load = loadPendulum(load0, pendulum_parameters, quad.translational_accelerations)
    state0 = np.concatenate([quad0, load0])
    #t = linspace(0, 25, 100)
    #x = odeint(odeSystem, state0, t, args=(quad, load))
    deltaT = 0.1
    t = linspace(0, 100, 1000)
    x = np.zeros((1000, 16))
    thrust_setpoint = 1500
    attitude_setpoint = np.array([deg2rad(0), deg2rad(10), deg2rad(0)])
    for i, time in enumerate(t):
        motors = control.quadControl(attitude_setpoint, quad.state[6:9], quad.state[9:12], thrust_setpoint, deltaT)
        x[i] = system(np.array(motors), deltaT, quad, load)
    plotQuadTrajectory(t,x)