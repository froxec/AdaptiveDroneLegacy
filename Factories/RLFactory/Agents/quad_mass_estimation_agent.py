import torch
import math
import numpy as np
class QuadMassEstimator:
    def __init__(self, input_shape, output_shape):
        self.input_shape = input_shape
        self.output_shape = output_shape
        self.signals_num = input_shape[0]
        self.samples = input_shape[1]
        self.policy_network = torch.nn.Sequential(
            torch.nn.Conv1d(self.signals_num, 4, kernel_size=3),
            torch.nn.ReLU(),
            torch.nn.Conv1d(4, 16,  kernel_size=3),
            torch.nn.ReLU(),
            torch.nn.Flatten(),
            torch.nn.Linear(16*(self.samples - 4), 32),
            torch.nn.ReLU(),
            torch.nn.Linear(32, 16),
            torch.nn.ReLU(),
            torch.nn.Linear(16, 8),
            torch.nn.ReLU(),
            torch.nn.Linear(8, 4),
            torch.nn.ReLU(),
            torch.nn.Linear(4, 2)
        )
    def get_mass(self, state):
        loc, scale = self.policy_network(state)
        mass = np.random.normal(loc, scale)
        return mass
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
        self.buffer = np.zeros(self.buffer_shape)
        self.full = False
        self.samples_to_fulfil = self.buffer_size[0]
    def __len__(self):
        return self.buffer_size[0]
    def __getitem__(self, name):
        return self.buffers[name]

class ReplayBuffer(RollBuffers):
    def __init__(self, sample_shapes, names, buffer_size=100):
        super().__init__(sample_shapes, names, buffer_size)
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
        batches = {name: torch.from_numpy(self.buffers[name][indices]) for name in self.names}
        return batches
class PolicyGradientLearning:
    def __init__(self, policy_network, alpha, gamma, max_episodes=1000):
        self.alpha = alpha
        self.gamma = gamma
        self.max_episodes = max_episodes
        self.policy_network = policy_network
        self.pi = torch.tensor(math.pi)
        self.epsilon = 1e-6
        self.optimizer = torch.optim.Adam(self.policy_network.parameters(), lr=alpha)

    def log_gaussian_loss(self, state, action, reward):
        mean, variance = self.policy_network(state)
        std = torch.sqrt(variance)
        action_prob = torch.exp(-0.5 * ((action - mean)**2/variance))/(std*torch.sqrt(2*self.pi))
        loss = torch.log(action_prob + self.epsilon)*reward
        return loss

    def update_policy(self, state, action, reward):
        loss = self.log_gaussian_loss(state, action, reward)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()


