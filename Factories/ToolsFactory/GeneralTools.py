import numpy as np
import plotly.graph_objects as go
from scipy.linalg import toeplitz
import time
from scipy.signal import iirfilter
from collections import deque
def manhattan_distance(a, b):
    return np.abs(a - b).sum()

def euclidean_distance(a, b, axis=None):
    if axis is None:
        return np.linalg.norm(a - b)
    else:
        return np.linalg.norm(a - b, axis=axis)

def sigmoid_function(x, L, a):
    return L/(1 + np.exp(-(x - a)))

def construct_ext_obs_mat(A, C, horizon):
    """
    Constructs extended observability matrix

    :param A: State matrix (nxn)
    :param C: Output matrix (qxn)
    :param horizon: prediction_horizon (int)
    """
    #kolejne elementy macierzy to macierze (qxn)
    #wykorzystanie mnożenia jest 2 razy szybsze niż potęgowanie
    assert C.shape[1] == A.shape[0]
    n = A.shape[0]
    q = C.shape[0]
    O_horizon = np.zeros((horizon, q, n))
    A_pow = np.identity(A.shape[0])
    for i in range(horizon):
        #A_pow = np.linalg.matrix_power(A, i)
        if i > 0:
            A_pow = A_pow @ A
        O_horizon[i, :, :] = C @ A_pow
    O_horizon = O_horizon.reshape((O_horizon.shape[0]*O_horizon.shape[1],
                                   O_horizon.shape[2]))
    return O_horizon

def construct_low_tril_Toeplitz(A, B, C, D=None, horizon=10):
    """
    Constructs extended observability matrix

    :param A: State matrix (nxn)
    :param B: Input matrix (nxm)
    :param C: Output matrix (qxn)
    :param D: Direct input to output matrix (qxm)
    :param horizon: prediction_horizon (int)
    """
    n = A.shape[0]
    m = B.shape[1]
    q = C.shape[0]
    if D is None:
        col = np.zeros((q, m, horizon))
        A_pow = np.identity(A.shape[0])
        for i in range(1, horizon):
            if i > 1:
                A_pow = A_pow @ A
            col[:, :, i] = C @ A_pow @ B
    first_col_idx = np.arange(horizon, dtype=int)
    first_row_idx = np.zeros(horizon-1, dtype=int)
    indices_matrix = toeplitz(first_col_idx, first_row_idx)
    toeplitz_matrix = col[:, :, indices_matrix].transpose(2, 3, 0, 1)
    toeplitz_matrix = toeplitz_matrix.transpose((0, 2, 1, 3)).reshape((toeplitz_matrix.shape[0]*toeplitz_matrix.shape[2],
                                                             toeplitz_matrix.shape[1]*toeplitz_matrix.shape[3]))
    return toeplitz_matrix
def plot_signal(signals):
    """
    Function for plotting signals.

    :param signals: dict with keys ('x' - signal domain, 'y' - signal values, 'name' - signal name)
    """
    fig = go.Figure()
    for signal in signals:
        fig.add_trace(go.Scatter(x=signal['x'], y=signal['y'], name=signal['name']))
    fig.show()

def minmax_normalize(signal, domain):
    normalized_signal = (signal - domain[0])/(domain[1] - domain[0])
    return normalized_signal

def minmax_rescale(normalized_signal, domain):
    signal_rescaled = normalized_signal*(domain[1] - domain[0]) + domain[0]
    return signal_rescaled
class RollBuffers():
    def __init__(self, names, sample_shapes, buffer_size=100):
        '''shapes - subsequent buffers sample shapes, resultant shape is (buffer_size, sample_shape),
        names - subsequent buffers names - used to create dict'''
        self.buffer_size = (buffer_size,)
        self.sample_shapes = sample_shapes
        self.names = names
        self.buffer_shapes = [self.buffer_size + shape for shape in sample_shapes]
        self.buffers = {self.names[i]: np.zeros(self.buffer_shapes[i]) for i in range(len(names))}
        self.full = {name: False for name in self.names}
        self.samples_to_fulfil = {name: buffer_size for name in self.names}
    def add_sample(self, names, samples):
        for i, name in enumerate(names):
            if name not in self.names:
                return print("Name {} refers to non existent buffer. Try one of valid names: {}".format(name, self.names))
            self.buffers[name] = np.roll(self.buffers[name], 1, axis=0)
            self.buffers[name][0] = samples[i]
            if self.samples_to_fulfil[name] > 0:
                self.samples_to_fulfil[name] -= 1
            elif self.full[name] == False:
                self.full[name] = True
    def flush(self):
        for i, name in enumerate(self.names):
            self.buffers[name] = np.zeros(self.buffer_shapes[i])
            self.full[name] = False
            self.samples_to_fulfil[name] = self.buffer_size[0]
    def __len__(self):
        return self.buffer_size[0]
    def __getitem__(self, name):
        return self.buffers[name]


class ReplayBuffer(RollBuffers):
    def __init__(self, names, sample_shapes, buffer_size=100):
        super().__init__(names, sample_shapes, buffer_size)
        self.data_counter = {name: 0 for name in names}
    def add_sample(self, names, samples):
        for i, name in enumerate(names):
            if name not in self.names:
                return print("Name {} refers to non existent buffer. Try one of valid names: {}".format(name, self.names))
            self.buffers[name] = np.roll(self.buffers[name], 1, axis=0)
            self.buffers[name][0] = samples[i]
            self.data_counter[name] += 1
            if self.samples_to_fulfil[name] > 0:
                self.samples_to_fulfil[name] -= 1
            elif self.full[name] == False:
                self.full[name] = True
    def sample_batch(self, batch_size):
        basic_samples_to_fulfil = self.samples_to_fulfil[self.names[0]]
        basic_data_counter = self.data_counter[self.names[0]]
        if basic_samples_to_fulfil == self.buffer_size[0]:
            print("At least one of the buffers empty. Cannot sample.")
            return
        buffer_size = self.buffer_size[0]
        if len(self.names) > 1:
            for name in self.names[1:]:
                assert self.data_counter[name] == basic_data_counter, f"number of elements added to each buffer should be equal (data shift prevention)"
        if batch_size > buffer_size - basic_samples_to_fulfil:
            batch_size = buffer_size - basic_samples_to_fulfil
            print("Batch size is bigger than amount of samples. Returning batch of size {}".format(buffer_size - basic_samples_to_fulfil))
        indices = np.random.choice(range(buffer_size - basic_samples_to_fulfil), size=batch_size, replace=False)
        batches = {name: torch.from_numpy(self.buffers[name][indices].astype(np.float32)) for name in self.names}
        return batches

def time_control(func):
    def wrapper(simulation, time_rate, **args):
        start = time.perf_counter()
        if 'x' in args.keys():
            x = args['x']
            u = func(simulation, x)
        else:
            u = func(simulation)
        finish = time.perf_counter()
        while finish - start < time_rate:
            finish = time.perf_counter()
        return u
    return wrapper

class LiveFilter:
    #implementation based on https://www.samproell.io/posts/yarppg/yarppg-live-digital-filter/
    def __call__(self, x):
        y = self.process(x)
        return y
    def process(self, x):
        return self._process(x)

    def _process(self, x):
        return NotImplementedError("Function not implemented..")

class LowPassLiveFilter(LiveFilter):
    # implementation based on https://www.samproell.io/posts/yarppg/yarppg-live-digital-filter/
    def __init__(self, bandwidths, fs, signals_num):
        if isinstance(bandwidths, (list, np.ndarray)) == False:
            raise ValueError("bandwidths should be either list or np.ndarray")
        if len(list(bandwidths)) != signals_num:
            raise ValueError("bandwidths num should be equal to signals num (one BW value per signal)")
        self.fs = fs
        self.bandwidth = bandwidths
        self.signals_num = signals_num
        a_list = []
        b_list = []
        for i in range(signals_num):
            b, a = iirfilter(2, Wn=bandwidths[i], fs=fs, btype="low", ftype="butter")
            a_list.append(a)
            b_list.append(b)
        self.a = a_list
        self.b = b_list
        self.x_que = [deque([0] * len(self.b[i]), maxlen=len(self.b[i])) for i in range(signals_num)]
        self.y_que = [deque([0] * (len(self.a[i]) -1), maxlen=len(self.a[i])-1) for i in range(signals_num)]

    def _process(self, x):
        signals_processed = []
        for i, element in enumerate(x):
            self.x_que[i].appendleft(element)
            y = np.dot(self.b[i], self.x_que[i]) - np.dot(self.a[i][1:], self.y_que[i])
            y = y / self.a[i][0]
            self.y_que[i].appendleft(y)
            signals_processed.append(y)
        return np.array(signals_processed)

    def reset(self):
        self.x_que = [deque([0] * len(self.b[i]), maxlen=len(self.b[i])) for i in range(self.signals_num)]
        self.y_que = [deque([0] * (len(self.a[i]) - 1), maxlen=len(self.a[i]) - 1) for i in range(self.signals_num)]
class BidirectionalDict(dict):
    def __init__(self):
        super().__init__()
        self.nominal_keys = []
        self.nominal_values = []
    def __setitem__(self, key, value):
        if key in self:
            print("{} already in dict .. substituting with new entry.".format(key))
            del self[key]
        if value in self:
            print("{} already in dict .. substituting with new entry.".format(value))
            del self[value]
        dict.__setitem__(self, key, value)
        dict.__setitem__(self, value, key)
        self.nominal_keys.append(key)
        self.nominal_values.append(value)
    def __delitem__(self, key):
        dict.__delitem__(self, self[key])
        dict.__delitem__(self, key)
    def __len__(self):
        return int(dict.__len__(self)/2)

