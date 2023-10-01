import matplotlib.pyplot as plt
import numpy as np
if __name__ == "__main__":
    plt.style.use('../../Factories/PlottingFactory/plotstyle.mplstyle')
    fig, ax = plt.subplots(nrows=3, ncols=1, sharex=True, dpi=100)
    t = np.arange(0, 10, 0.01)
    f = 2
    snt_data1 = np.sin(0.25 * np.pi*f*t)
    snt_data2 = np.sin(0.5 * np.pi * f * t)
    snt_data3 = np.sin(1 * np.pi * f * t)
    snt_data4 = np.sin(2 * np.pi * f * t)
    data = [snt_data1, snt_data2, snt_data3]
    for i, line in enumerate(data):
        ax[i].plot(t, line)
        ax[i].set_ylabel(r"\textbf{x}")
    ax[-1].set_xlabel("time")
    fig.suptitle("Title")
    plt.show(block=True)