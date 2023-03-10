import torch

class QuadMassEstimator:
    def __init__(self, input_shape, output_shape):
        self.input_shape = input_shape
        self.output_shape = output_shape

        self.policy = torch.nn.Sequential(
            torch.nn.Linear(self.input_shape, 32),
            torch.nn.ReLU(),
            torch.nn.Linear(32, 16),
            torch.nn.ReLU(),
            torch.nn.Linear(16, 8),

        )
class PolicyGradientLearning:
    def __init__(self, alpha, gamma, max_episodes=1000, batch_size=64):
        self.alpha = alpha
        self.gamma = gamma
        self.max_episodes = max_episodes
        self.batch_size = batch_size

