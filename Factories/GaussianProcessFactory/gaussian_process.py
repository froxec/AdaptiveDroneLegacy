import numpy as np
import scipy.linalg
import plotly.graph_objects as go
class GaussianProcess():
    def __init__(self, predict_at, kernel_function, noise_std=0):
        self.kernel_function = kernel_function
        self.X2 = predict_at
        self.cov22 = self.kernel_function(self.X2, self.X2)
        self.noise_std = noise_std
        self.sample = None
        self.memory = {'obs_x': [],
                       'obs_y': []}
    def __call__(self, obs_x, obs_y):
        self.memory['obs_x'].extend(obs_x.flatten().tolist())
        self.memory['obs_y'].extend(obs_y)
        obs_x = np.array(self.memory['obs_x']).reshape(-1, 1)
        cov11 = self.kernel_function(obs_x, obs_x) + np.eye(obs_x.shape[0])*(self.noise_std**2 + 3e-7)
        cov12 = self.kernel_function(self.X2, obs_x)

        K = scipy.linalg.solve(cov11, cov12, assume_a='pos').T
        self.mean = K @ self.memory['obs_y']
        self.cov22 = self.kernel_function(self.X2, self.X2)
        self.cov = self.cov22 - (K @ cov12)
        self.std = np.sqrt(np.diag(self.cov))

        return self.mean, self.cov
    def sample_functions(self, number_of_functions=1):
        self.sample = np.random.multivariate_normal(self.mean, self.cov, size=number_of_functions)
        return {'x':self.X2.flatten(), 'y': self.sample, 'name': 'GaussianProcessRealization'}
    def Thompson_sampling(self, mode='max', number_of_samples=1):
        sample_signal = self.sample_functions(number_of_functions=number_of_samples)
        x = sample_signal['x']
        y = sample_signal['y']
        if mode == 'max':
            best_action_idx = np.unravel_index(np.argmax(y), y.shape)
        else:
            best_action_idx = np.unravel_index(np.argmin(y), y.shape)
        best_action = x[best_action_idx[1]]
        predicted_reward = y[best_action_idx]
        return {'best_action': best_action, 'predicted_reward': predicted_reward}
    def plot(self):
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=self.X2.flatten(), y=self.mean))
        fig.add_trace(go.Scatter(x=self.memory['obs_x'], y=self.memory['obs_y'], mode='markers'))
        fig.add_traces([go.Scatter(x=self.X2.flatten(), y=self.mean + self.std, mode='lines',line_color='rgba(0,0,0,0)'),
                        go.Scatter(x=self.X2.flatten(), y=self.mean - self.std, mode='lines',line_color='rgba(0,0,0,0)',
                                   fill='tonexty', fillcolor='rgba(0, 0, 255, 0.2)')])
        fig.show()