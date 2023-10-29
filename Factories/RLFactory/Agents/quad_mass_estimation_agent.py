import torch
import torch.nn as nn
import math


class QuadMassEstimator(nn.Module):
    def __init__(self, input_shape, output_shape, mass_min, mass_max):
        super().__init__()
        self.input_shape = input_shape
        self.output_shape = output_shape
        self.signals_num = input_shape[0]
        self.samples = input_shape[1]
        self.mass_min = mass_min
        self.mass_max = mass_max
        self.feature_extractor = torch.nn.Sequential(
            #torch.nn.BatchNorm1d(self.signals_num),
            torch.nn.Conv1d(self.signals_num, 4, kernel_size=3),
            #torch.nn.BatchNorm1d(4),
            torch.nn.ReLU(),
            torch.nn.Conv1d(4, 8, kernel_size=3),
           # torch.nn.BatchNorm1d(8),
            torch.nn.ReLU(),
            torch.nn.Flatten(),
        )
        self.mean_head = torch.nn.Sequential(
            torch.nn.Linear((self.samples-4)*8, 16),
            torch.nn.ReLU(),
            torch.nn.Linear(16, 4),
            torch.nn.ReLU(),
            torch.nn.Linear(4, 1),
            torch.nn.Tanh()
        )
        self.std_head = torch.nn.Sequential(
            torch.nn.Linear((self.samples - 4) * 8, 16),
            torch.nn.ReLU(),
            torch.nn.Linear(16, 4),
            torch.nn.ReLU(),
            torch.nn.Linear(4, 1),
            torch.nn.Tanh()
        )
    def predict(self, state):
        ##TODO zmienić wyjście, tak aby generowało wartości tylko z podanego zakresu
        # w pewien sposób zrealizowano poprzez funkcję aktywacji
        ##inicjalizacja biasu
        features = self.feature_extractor(state)
        loc = (self.mean_head(features) + 1)  + 0.2
        scale = (self.std_head(features) + 1) + 1e-5
        return loc, scale


class PolicyGradientLearning:
    def __init__(self, policy_network, alpha, gamma, max_episodes=1000):
        self.alpha = alpha
        self.gamma = gamma
        self.max_episodes = max_episodes
        self.policy_network = policy_network
        self.pi = torch.tensor(math.pi)
        self.epsilon = 1e-6
        self.optimizer = torch.optim.Adam(self.policy_network.parameters(), lr=alpha)
        self.current_loss = 0.0

    def log_gaussian_loss(self, state, action, reward):
        state, action, reward = torch.squeeze(state), torch.squeeze(action), torch.squeeze(reward)
        mean, std = self.policy_network.predict(state)
        max_pdb = torch.exp(-0.5 * ((mean - mean)/std)**2)/(std*torch.sqrt(2*self.pi))
        action_pdb = torch.exp(-0.5 * ((action - mean)/std)**2)/(std*torch.sqrt(2*self.pi))
        scaled_action_pdb = action_pdb/max_pdb
        loss = -torch.sum(torch.log(scaled_action_pdb + self.epsilon)*reward)
        self.current_loss = loss.detach().numpy().item()
        return loss

    def update_policy(self, state, action, reward):
        loss = self.log_gaussian_loss(state, action, reward)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()


