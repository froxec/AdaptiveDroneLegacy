import numpy as np
import scipy.linalg
import plotly.graph_objects as go
class GaussianProcess():
    def __init__(self, predict_at, kernel_function, noise_std=0):
        self.kernel_function = kernel_function
        self.X = predict_at
        self.cov22 = self.kernel_function(self.X, self.X)
        self.noise_std = noise_std
        self.sample = None
        self.memory = {'obs_x': [],
                       'obs_y': []}
    def __call__(self, obs_x, obs_y):
        self.memory['obs_x'].extend(obs_x.flatten().tolist())
        self.memory['obs_y'].extend(obs_y)
        obs_x = np.array(self.memory['obs_x']).reshape(-1, 1)
        cov11 = self.kernel_function(obs_x, obs_x) + np.eye(obs_x.shape[0])*(self.noise_std**2 + 3e-7)
        cov12 = self.kernel_function(self.X, obs_x)

        K = scipy.linalg.solve(cov11, cov12, assume_a='pos').T
        self.mean = K @ self.memory['obs_y']
        self.cov22 = self.kernel_function(self.X, self.X)
        self.cov = self.cov22 - (K @ cov12)
        self.std = np.sqrt(np.diag(self.cov))

        return self.mean, self.cov
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
        best_action = x[best_action_idx[1]]
        predicted_reward = y[best_action_idx]
        return {'best_action': best_action, 'predicted_reward': predicted_reward}
    def plot(self):
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=self.X.flatten(), y=self.mean))
        fig.add_trace(go.Scatter(x=self.memory['obs_x'], y=self.memory['obs_y'], mode='markers'))
        fig.add_traces([go.Scatter(x=self.X.flatten(), y=self.mean + self.std, mode='lines', line_color='rgba(0,0,0,0)'),
                        go.Scatter(x=self.X.flatten(), y=self.mean - self.std, mode='lines', line_color='rgba(0,0,0,0)',
                                   fill='tonexty', fillcolor='rgba(0, 0, 255, 0.2)')])
        fig.show()

class ContextualGaussianProcess(GaussianProcess):
    def __init__(self, predict_at, contextual_kernel, action_kernel, noise_std):
        self.X = predict_at
        self.contextual_kernel = contextual_kernel
        self.action_kernel = action_kernel
        self.noise_std = noise_std
        self.sample = None
        self.memory = {'context': [],
                       'action': [],
                       'reward': []}
    def __call__(self, context, action, reward):
        self.memory['context'].extend(context.tolist())
        self.memory['action'].extend(action.tolist())
        self.memory['reward'].extend(reward)
        context = np.array(self.memory['context'])
        action = np.array(self.memory['action'])
        reward = np.array(self.memory['reward'])
        action_cov11 = self.action_kernel(action, action)
        context1_cov11 = self.action_kernel(context[:, 0], context[:, 0])
        context2_cov11 = self.action_kernel(context[:, 1], context[:, 1])
        action_cov12 = self.action_kernel(self.X[:, 0], action)
        context1_cov12 = self.action_kernel(self.X[:, 1], context[:, 0])
        context2_cov12 = self.action_kernel(self.X[:, 2], context[:, 1])
        action_cov22 = self.action_kernel(self.X[:, 0], self.X[:, 0])
        context1_cov22 = self.action_kernel(self.X[:, 1], self.X[:, 1])
        context2_cov22 = self.action_kernel(self.X[:, 2], self.X[:, 2])
        joint_cov11 = self.calculate_joint_cov([action_cov11, context1_cov11, context2_cov11]) + np.eye((context[:,0].shape[0]**2)*action.shape[0])*(self.noise_std**2 + 3e-7)
        joint_cov12 = self.calculate_joint_cov([action_cov12, context1_cov12, context2_cov12])
        joint_cov22 = self.calculate_joint_cov([action_cov22, context1_cov22, context2_cov22])

        K = scipy.linalg.solve(joint_cov11, joint_cov12, assume_a='pos').T
        self.mean = K @ reward
        self.cov = joint_cov22 - (K @ joint_cov12)
        self.std = np.sqrt(np.diag(self.cov))

        return self.mean, self.cov
    def calculate_joint_cov(self, covs):
        action_cov = covs[0]
        context1_cov = covs[1]
        context2_cov = covs[2]
        joint_cov = np.array([[[element1*element2*element3 for element3 in context2_cov]for element2 in context1_cov] for element1 in action_cov])
        return  joint_cov
