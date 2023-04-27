import numpy as np
import plotly.graph_objects as go
def manhattan_distance(a, b):
    return np.abs(a - b).sum()

def euclidean_distance(a, b, axis=None):
    if axis is None:
        return np.linalg.norm(a - b)
    else:
        return np.linalg.norm(a - b, axis=axis)

def sigmoid_function(x, L):
    return L/(1 + np.exp(-x))
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
