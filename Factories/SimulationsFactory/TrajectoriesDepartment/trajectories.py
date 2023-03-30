import numpy as np
import plotly.graph_objects as go
class Trajectory():
    def __init__(self):
        self.waypoints_num = None
        self.fun_x = None
        self.fun_y = None
        self.fun_z = None
        self.generated_trajectory = None
    def plot_trajectory(self):
        raise NotImplementedError
class SpiralTrajectory(Trajectory):
    def __init__(self, waypoints_num):
        super().__init__()
        self.waypoints_num = waypoints_num
        self.fun_x = lambda r: r*np.sin(r)
        self.fun_y = lambda r: r*np.cos(r)
        self.fun_z = lambda r: r

        r = np.linspace(0, 10, waypoints_num).reshape((-1, 1))
        self.generated_trajectory = np.concatenate([self.fun_x(r), self.fun_y(r), self.fun_z(r)], axis=1)

    def plot_trajectory(self):
        fig = go.Figure(data=[go.Scatter3d(x=self.generated_trajectory[:, 0], y=self.generated_trajectory[:, 1], z=self.generated_trajectory[:, 2])])
        fig.show()