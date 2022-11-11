from numpy import linspace
from model import quadcopterModel, loadPendulum, odeSystem, system
from model_parameters import quad_parameters, pendulum_parameters
from scipy.integrate import odeint
import numpy as np
from plots import plotDataPID, plotTrajectory, groupDataFromPIDs
import attitude_control as control
from numpy import deg2rad
from Simulation.Visualization.vis import Visualizer, ParallelVisualizer
import time
import multiprocessing as mp

if __name__ == '__main__':
    load0 = np.zeros(4)
    quad0 = np.zeros(12)
    quad = quadcopterModel(quad0, quad_parameters)
    load = loadPendulum(load0, pendulum_parameters, quad.translational_accelerations, quad.state[3:6])
    visualizer = ParallelVisualizer()
    plot_pipe, remote_end = mp.Pipe()
    plot_process = mp.Process(
        target=visualizer,
        args=(remote_end,),
        daemon=True
    )
    deltaT = 0.01
    controler = control.quadControler(deltaT)
    state0 = np.concatenate([quad0, load0])
    #t = linspace(0, 25, 100)
    #x = odeint(odeSystem, state0, t, args=(quad, load))
    t = np.arange(0, 10, deltaT)
    x = np.zeros((t.size, 16))
    thrust_setpoint = 1000
    attitude_setpoint = np.array([deg2rad(0), deg2rad(0), deg2rad(30)])
    start = time.time()
    plot_process.start()
    for i, t_i in enumerate(t):
        motors = controler(attitude_setpoint, quad.state[6:9], quad.state[9:12], quad.angular_accelerations, thrust_setpoint)
        x[i] = system(np.array(motors), deltaT, quad, load)
        if (i % 10//deltaT) == 0:
            #visualizer(quad.state[0:3], quad.state[6:9], t_i)
            send = plot_pipe.send
            data_to_send = np.concatenate((quad.state[0:3], quad.state[6:9], np.array([t_i])))
            send(data_to_send)
        print(t_i)
        time.sleep(deltaT)
    send(None)
    print(time.time() - start)
    plotTrajectory(t, x.transpose()[0:12], 4, 3)
    #plotDataPID(t, controler, 'roll')
    plotTrajectory(t, x.transpose()[12:16], 2, 2)