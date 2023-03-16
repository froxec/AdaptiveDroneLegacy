from typing import Dict
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
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
    def __init__(self, images_dir):
        self.loss = []
        self.mean_reward = []
        self.std_reward = []
        self.action_mean = []
        self.action_std = []

        self.current_episode_action_mean = []
        self.current_episode_action_std = []
        self.current_episode_rewards = []
        self.images_dir = images_dir
    def update_episodic_data(self, action_mean, action_std, reward):
        self.current_episode_action_mean.append(action_mean)
        self.current_episode_action_std.append(action_std)
        self.current_episode_rewards.append(reward)

    def accumulate_data(self, loss):
        self.loss.append(loss)
        mean_reward = self.__average(self.current_episode_rewards)
        std_reward = self.__std(self.current_episode_rewards)
        action_mean = self.__average(self.current_episode_action_mean)
        action_std = self.__average(self.current_episode_action_std)

        self.mean_reward.append(mean_reward)
        self.std_reward.append(std_reward)

        self.action_mean.append(action_mean)
        self.action_std.append(action_std)

        self.__reset_episodic_data()
        self.__save_plots()

    def __average(self, list):
        return sum(list) / len(list)

    def __std(self, list):
        mean = self.__average(list)
        variance = sum([((element - mean)**2) for element in list]) /  len(list)
        std = variance ** 0.5
        return std
    def __reset_episodic_data(self):
        self.current_episode_action_mean = []
        self.current_episode_action_std = []
        self.current_episode_rewards = []

    def __save_plots(self):
        sns.set_theme()
        self.__plot_data_with_bounds(self.mean_reward, self.std_reward, 'rewards.png')
        self.__plot_data_with_bounds(self.action_mean, self.action_std, 'action_distribution.png')
        self.__plot_data(self.loss, 'loss.png')
        plt.close('all')
    def __plot_data_with_bounds(self, mean, std, filename):
        sns.set_theme()
        lower_bounds = np.array(mean) - np.array(std)
        upper_bounds = np.array(mean) + np.array(std)
        fig, ax = plt.subplots()
        with sns.axes_style("darkgrid"):
            ax.plot(mean)
            ax.fill_between(range(len(mean)), lower_bounds, upper_bounds, alpha=0.3)
            plt.savefig(self.images_dir + filename)
    def __plot_data(self, data, filename):
        sns.set_theme()
        fig, ax = plt.subplots()
        with sns.axes_style("darkgrid"):
            ax.plot(range(len(data)), data)
            plt.savefig(self.images_dir + filename)