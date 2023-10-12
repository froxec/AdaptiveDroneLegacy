from scipy import signal
import numpy as np
import matplotlib.pyplot as plt
def filter_sbs():
    dt = 0.01
    fs = 1
    x = np.arange(0, 1, step=0.001)
    data = np.sin(2*np.pi*50*x)
    data[0] = 1000
    b = signal.firwin(150, 0.01)
    z = np.zeros(b.size-1)
    result = np.zeros(data.size)
    for i, sig in enumerate(data):
        result[i], z = signal.lfilter(b, 1, [sig], zi=z)
    return x, data, result



if __name__ == '__main__':
    x, y, y_filtered = filter_sbs()
    plt.plot(x, y)
    plt.plot(x, y_filtered)
    plt.show()