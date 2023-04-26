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
        fig = go.Figure(data=[go.Scatter3d(x=self.generated_trajectory[:, 0], y=self.generated_trajectory[:, 1],
                                           z=self.generated_trajectory[:, 2])])
        fig.show()

class TrajectoryWithTerminals(Trajectory):
    def __init__(self):
        super().__init__()
        self.terminals = None
        self.terminals_payload = None


class SpiralTrajectory(Trajectory):
    def __init__(self, waypoints_num):
        super().__init__()
        self.waypoints_num = waypoints_num
        self.fun_x = lambda r: r*np.sin(r)
        self.fun_y = lambda r: r*np.cos(r)
        self.fun_z = lambda r: r

        r = np.linspace(0, 10, waypoints_num).reshape((-1, 1))
        self.generated_trajectory = np.concatenate([self.fun_x(r), self.fun_y(r), self.fun_z(r)], axis=1)

class RectangularTrajectory(Trajectory):
    def __init__(self):
        self.generated_trajectory = np.array([[0, 0, 20], [0, 20, 20], [20, 20, 20], [20, 0, 20], [0, 0, 20], [0, 0, 0]])

class RectangularTrajectoryWithTerminals(TrajectoryWithTerminals):
    def __init__(self):
        self.generated_trajectory = np.array([[0, 0, 20], [0, 20, 20], [0, 20, 0], [0, 20, 20],
                                              [20, 20, 20], [20, 20, 0], [20, 20, 20],
                                              [20, 0, 20], [20, 0, 0], [20, 0, 20], [0, 0, 20], [0, 0, 0]])
        self.terminals = np.array([[0, 20, 0], [20, 20, 0], [20, 0, 0]])
        self.terminals_payload = [0.1, 0.5, 0.2]


