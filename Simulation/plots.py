import matplotlib.pyplot as plt
import numpy as np

def plotTrajectory(t, x, rows, cols):
    #x = x.transpose()
    f1, axs = plt.subplots(rows, cols)
    for i, ax in enumerate(axs.reshape(-1)):
        ax.plot(t, x[i]) 
    plt.show(block=True)

def groupDataFromPIDs(controler_object):
    controler_names = ('roll', 'pitch', 'yaw')
    pid_names = ('angle', 'rate', 'acceleration')
    data_error = {}
    data_signals = {}
    for controler, controler_name in zip(controler_object.attitude_control.controlers, controler_names):
        data_error[controler_name] = {pid_name: pid.error_history for pid, pid_name in zip(controler.pids, pid_names)}
    for controler, controler_name in zip(controler_object.attitude_control.controlers, controler_names):
        data_signals[controler_name] = {pid_name: pid.PID_history for pid, pid_name in zip(controler.pids, pid_names)}
    return (data_error, data_signals)

def plotDataPID(t, controler_object, name):
    data_error, data_signals = groupDataFromPIDs(controler_object)
    x = []
    for key in data_error[name]:
        x.append(data_error[name][key])
    for key in data_signals[name]:
        x.append(data_signals[name][key])
    plotTrajectory(t, x, 3, 2)
