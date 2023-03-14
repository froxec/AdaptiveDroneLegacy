from typing import Dict
class ParametersPerturber():
    def __init__(self, nominal_parameters: Dict):
        self.nominal_parameters = nominal_parameters
        self.perturbed_parameters = nominal_parameters
        self.keys = nominal_parameters.keys()
    def __call__(self, perturbation):
        for key in perturbation.keys():
            self.perturbed_parameters[key] = self.nominal_parameters[key] + perturbation[key]
        return self.perturbed_parameters

class RLMonitor():
    def __init__(self):
        self.loss = []
        self.mean_reward = []
        self.std_reward = []
        self.action_mean = []
        self.action_std = []

        self.current_episode_action_mean = []
        self.current_episode_action_std = []
        self.current_episode_rewards = []

    def update_episodic_data(self, action_mean, action_std):
        self.current_episode_action_mean.append(action_mean)
        self.current_episode_action_std.append(action_std)
        self.current_episode_rewards.append()

