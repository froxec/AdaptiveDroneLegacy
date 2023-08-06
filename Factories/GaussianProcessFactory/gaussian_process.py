import numpy as np
import scipy.linalg
import plotly.graph_objects as go
import itertools
from Factories.ToolsFactory.GeneralTools import minmax_rescale
from scipy.linalg import cholesky, cho_solve
from collections import Counter
class GaussianProcess():
    def __init__(self, predict_at, kernel_function, noise_std=0, max_samples=100):
        self.kernel_function = kernel_function
        self.X = predict_at
        self.cov22 = self.kernel_function(self.X, self.X)
        self.noise_std = noise_std
        self.sample = None
        self.best_action_idx = None
        self.plots = 0
        self.max_samples=max_samples
        self.memory = {'obs_x': [],
                       'obs_y': []}
    def __call__(self, obs_x, obs_y):
        self.memory['obs_x'].extend(obs_x.flatten().tolist())
        self.memory['obs_y'].extend(obs_y)
        obs_x = np.array(self.memory['obs_x']).reshape(-1, 1)
        cov11 = self.kernel_function(obs_x, obs_x) + np.eye(obs_x.shape[0])*(self.noise_std**2 + 3e-7)
        cov12 = self.kernel_function(obs_x, self.X)

        K = scipy.linalg.solve(cov11, cov12, assume_a='pos').T
        self.mean = K @ self.memory['obs_y']
        self.cov22 = self.kernel_function(self.X, self.X)
        self.cov = self.cov22 - (K @ cov12)
        self.std = np.sqrt(np.diag(self.cov))

        return self.mean, self.cov
    def reset(self):
        self.cov22 = self.kernel_function(self.X, self.X)
        self.sample = None
        self.best_action_idx = None
        self.plots = 0
        self.memory = {'obs_x': [],
                       'obs_y': []}
    def sample_functions(self, number_of_functions=1):
        self.sample = np.random.multivariate_normal(self.mean, self.cov, size=number_of_functions)
        return {'x':self.X.flatten(), 'y': self.sample, 'name': 'GaussianProcessRealization'}
    def Thompson_sampling(self, mode='max', number_of_samples=1):
        sample_signal = self.sample_functions(number_of_functions=number_of_samples)
        x = sample_signal['x']
        y = sample_signal['y']
        if mode == 'max':
            best_action_idx = np.unravel_index(np.argmax(y), y.shape)
        else:
            best_action_idx = np.unravel_index(np.argmin(y), y.shape)
        self.best_action_idx = best_action_idx
        best_action = x[best_action_idx[1]]
        predicted_reward = y[best_action_idx]
        return {'best_action': best_action, 'predicted_reward': predicted_reward}
    def plot(self, img_path):
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=self.X.flatten(), y=self.mean, name='mean', line=dict(width=4)))
        fig.add_trace(go.Scatter(x=self.memory['obs_x'], y=self.memory['obs_y'], name="Obserwacja", mode='markers',
                                 marker=dict(size=16)))
        fig.add_traces(
            [go.Scatter(x=self.X.flatten(), y=self.mean + self.std, name="std", showlegend=False, mode='lines',
                        line_color='rgba(0,0,0,0)'),
             go.Scatter(x=self.X.flatten(), y=self.mean - self.std, name="std", mode='lines',
                        line_color='rgba(0, 0, 255, 0.2)',
                        fill='tonexty', fillcolor='rgba(0, 0, 255, 0.2)')])
        fig.update_layout(
            autosize=False,
            width=1600,
            height=800,
            xaxis_title="X",
            yaxis_title="Y",
            font=dict(
                family="Courier New, monospace",
                size=28,
                color="RebeccaPurple"
            ))
        fig.write_image(img_path + "gp_plot{}.jpg".format(self.plots))
        self.plots += 1

    def plot_with_sample(self):
        sample = np.reshape(self.sample, (-1, 1))
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=self.X.flatten(), y=self.mean, name='mean', line=dict(width=4)))
        fig.add_trace(go.Scatter(x=self.memory['obs_x'], y=self.memory['obs_y'], name="Obserwacja", mode='markers', marker=dict(size=16)))
        fig.add_traces(
            [go.Scatter(x=self.X.flatten(), y=self.mean + self.std, name="std", showlegend=False, mode='lines', line_color='rgba(0,0,0,0)'),
             go.Scatter(x=self.X.flatten(), y=self.mean - self.std, name="std", mode='lines', line_color='rgba(0, 0, 255, 0.2)',
                        fill='tonexty', fillcolor='rgba(0, 0, 255, 0.2)')])
        fig.update_layout(
            autosize=False,
            width=1600,
            height=800,
            xaxis_title="X",
            yaxis_title="Y",
            font=dict(
                family="Courier New, monospace",
                size=28,
                color="RebeccaPurple"
            ))
        fig.add_trace(
            go.Scatter(x=self.X.flatten(), y=sample.flatten(), name="Realizacja procesu", line=dict(width=4, color='rgb(0, 255, 0)')))
        fig.add_trace(
            go.Scatter(x=[self.X.reshape(1, -1)[self.best_action_idx]], y=[sample.reshape(1, -1)[self.best_action_idx]], name="Najlepsza akcja", mode='markers', marker=dict(size=16, color ='rgba(255, 0, 0, 0.5)')))
        fig.write_image("../../images/gp/gp_plot{}.jpg".format(self.plots))
        self.plots += 1

    def _handle_overflow(self):
        if len(self.memory['obs_x']) > self.max_samples:
            ctn = Counter(self.memory['obs_x'])
            values = list(ctn.keys())
            counts = list(ctn.values())
            unique_probabilities = {value: count for value, count in zip(values, list(np.array(counts)/sum(counts)))}
            p = [unique_probabilities[x]/ctn[x] for x in self.memory['obs_x']]
            id = np.random.choice(range(len(self.memory['obs_x'])), size=1, p=p)[0]
            del self.memory['obs_x'][id]
            del self.memory['obs_y'][id]
class EfficientGaussianProcess(GaussianProcess):
    def __call__(self, obs_x, obs_y):
        self.memory['obs_x'].extend(obs_x.flatten().tolist())
        self.memory['obs_y'].extend(obs_y)
        self._handle_overflow()
        obs_x = np.array(self.memory['obs_x']).reshape(-1, 1)
        cov11 = self.kernel_function(obs_x, obs_x) + np.eye(obs_x.shape[0]) * (self.noise_std ** 2 + 3e-7)
        cov12 = self.kernel_function(self.X, obs_x)

        L = cholesky(cov11, lower=True)
        self.alpha = cho_solve((L, True), self.memory['obs_y'])
        self.L = L
        self.mean = cov12.dot(self.alpha)
        v = cho_solve((L, True), cov12.T)
        self.cov = self.cov22 - cov12.dot(v)
        self.std = np.sqrt(np.diag(self.cov))

        return self.mean, self.cov

class ContextualGaussianProcess(GaussianProcess):
    def __init__(self, predict_at, kernel, noise_std):
        self.X = predict_at
        self.kernel = kernel
        self.noise_std = noise_std
        self.sample = None
        self.subspace_mean = None
        self.subspace_std = None
        self.subspace_x = None
        self.memory = {'obs_x': [],
                       'obs_y': []}
        self.cov22 = self.kernel(self.X, self.X)
    def __call__(self, obs_x, obs_y):
        self.memory['obs_x'].append(obs_x)
        self.memory['obs_y'].append(obs_y)
        obs_x = np.array(self.memory['obs_x'])
        obs_y = np.array(self.memory['obs_y'])
        cov11 = self.kernel(obs_x, obs_x) + np.eye(obs_x.shape[0])*(self.noise_std**2 + 3e-7)
        cov12 = self.kernel(self.X, obs_x)
        K = scipy.linalg.solve(cov11, cov12, assume_a='pos').T
        self.mean = K @ obs_y
        self.cov = self.cov22 - (K @ cov12)
        self.std = np.sqrt(np.diag(self.cov))

        return self.mean, self.cov

    def Thompson_sampling(self, context, mode='max', number_of_samples=1):
        grids1d = [np.unique(self.X[:, 1]), np.unique(self.X[:, 2])]
        context_specific_points = self.mine_context_specific_points(context, grids1d)
        mean, cov_mat = self.subspace_distribution(context_specific_points['vector_indices'])
        sample_signal = self.sample_functions(context_specific_points['coordinates'], mean, cov_mat, number_of_functions=number_of_samples)
        x = np.array(sample_signal['x'])
        self.subspace_x = x
        y = sample_signal['y']
        if mode == 'max':
            best_action_idx = np.unravel_index(np.argmax(y), y.shape)
        else:
            best_action_idx = np.unravel_index(np.argmin(y), y.shape)
        best_action = x[best_action_idx[1]][0]
        predicted_reward = y[best_action_idx]
        return {'best_action': best_action, 'predicted_reward': predicted_reward}
    def find_closest_point(self, context, grids1d):
        x_idx = np.searchsorted(grids1d[0], context[0])
        if x_idx > 0 and (np.abs(context[0] - grids1d[0][x_idx]) > np.abs(context[0] - grids1d[0][x_idx-1])):
            x_idx = x_idx-1
        y_idx = np.searchsorted(grids1d[1], context[1])
        if y_idx > 0 and (np.abs(context[1] - grids1d[1][y_idx]) > np.abs(context[1] - grids1d[1][y_idx-1])):
            y_idx = y_idx-1
        return grids1d[0][x_idx], grids1d[1][y_idx]
    def mine_context_specific_points(self, context, grids1d):
        x, y = self.find_closest_point(context, grids1d)
        context_specific_points = {'vector_indices': [], 'coordinates': []}
        for i, point in enumerate(self.X):
            if (point[1:] == np.array([x, y])).all():
                context_specific_points['vector_indices'].append(i)
                context_specific_points['coordinates'].append(point.tolist())
        return context_specific_points
    def subspace_distribution(self, points_indices):
        mean = self.mean[points_indices]
        cov_indices = [index for index in itertools.product(points_indices, points_indices)]
        cov_matrix = self.cov[tuple
        (zip(*cov_indices))].reshape((mean.shape[0], -1))
        self.subspace_mean = mean
        self.subspace_std = np.sqrt(np.diag(cov_matrix))
        return mean, cov_matrix

    def sample_functions(self, x, mean, cov, number_of_functions=1):
        self.sample = np.random.multivariate_normal(mean, cov, size=number_of_functions)
        return {'x': x, 'y': self.sample, 'name': 'GaussianProcessRealization'}

    def calculate_joint_cov(self, covs):
        action_cov = covs[0]
        context1_cov = covs[1]
        context2_cov = covs[2]
        joint_cov = np.array([[[element1*element2*element3 for element3 in context2_cov]for element2 in context1_cov] for element1 in action_cov])
        return  joint_cov

    def plot(self, x_domain):
        x = minmax_rescale(self.subspace_x[:, 0], x_domain)
        obs_x = minmax_rescale(np.array(self.memory['obs_x'])[:, 0], x_domain)
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=x, y=self.subspace_mean))
        fig.add_trace(go.Scatter(x=obs_x, y=self.memory['obs_y'], mode='markers'))
        fig.add_traces([go.Scatter(x=x, y=self.subspace_mean + self.subspace_std, mode='lines', line_color='rgba(0,0,0,0)'),
                        go.Scatter(x=x, y=self.subspace_mean - self.subspace_std, mode='lines', line_color='rgba(0,0,0,0)',
                                   fill='tonexty', fillcolor='rgba(0, 0, 255, 0.2)')])
        fig.show()
