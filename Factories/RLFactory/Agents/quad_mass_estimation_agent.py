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
class ReplayBuffer():
    def __init__(self, sample_shape, buffer_size=100):
        self.buffer_size = (buffer_size,)
        self.sample_shape = sample_shape
        self.buffer_shape = self.buffer_size + sample_shape
        self.buffer = np.zeros(self.buffer_shape)
        self.full = False
        self.samples_to_fulfil = buffer_size
    def add_sample(self, sample):
        self.buffer = np.roll(self.buffer, 1)
        self.buffer[0] = sample
        if self.samples_to_fulfil > 0:
            self.samples_to_fulfil -= 1
        elif self.full == False:
            self.full = True
    def sample_batch(self, batch_size):
        if batch_size > len(self):
            batch_size = len(self)
            print("Batch size is bigger than amount of samples. Returning batch of size {}".format(len(self)))
        indices = np.random.choice(range(len(self)), size=batch_size, replace=False)
        batches = self.buffer[indices]
        batches_torch = torch.from_numpy(batches)
        return batches_torch
    def __len__(self):
        return self.buffer_size[0] - self.samples_to_fulfil
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


