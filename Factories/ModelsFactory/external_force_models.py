import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
class ExternalDisturbance:
    def __init__(self):
        self.time_prev = 0.0
        self.disturbance_history = {'time': [],
                                    'signal': []}
    def __call__(self, time):
        disturbance = self.ask_for_disturbance(time)
        if self.time_prev == time:
            pass
        else:
            self.disturbance_history['signal'].append(list(disturbance.flatten()))
            self.disturbance_history['time'].append(time)
        self.time_prev = time
        return disturbance

    def ask_for_disturbance(self):
        raise NotImplementedError("Method ask_for_disturbance() should be implemented in child class..")

    def plot_history(self):
        fig = make_subplots(rows=3, cols=1, x_title='Czas [s]',
                            subplot_titles=('Zakłócenie w osi x [N]', 'Zakłócenie w osi y [N]',
                                            'Zakłócenie w osi z [N]'))
        time = self.disturbance_history['time']
        data = np.array(self.disturbance_history['signal'])

        fig.add_trace(go.Scatter(x=time, y=data[:, 0]), row=1, col=1)
        fig.add_trace(go.Scatter(x=time, y=data[:, 1]), row=2, col=1)
        fig.add_trace(go.Scatter(x=time, y=data[:, 2]), row=3, col=1)
        fig.show()

class WindModel(ExternalDisturbance):
    def __init__(self, direction_vector=np.array([0.0, 1.0, 0.0]), strength=1.0):
        super().__init__()
        if isinstance(direction_vector, list):
            direction_vector = np.array(direction_vector)
        direction_vector = direction_vector.astype('float32')
        if np.linalg.norm(direction_vector) != 1.0:
            self.direction_vector = direction_vector / (np.linalg.norm(direction_vector) + 1e-15)
        else:
            self.direction_vector = direction_vector
        print("Wind direction:", self.direction_vector)
        self.strength = strength

    def ask_for_disturbance(self, time):
        wind_force = self.direction_vector * self.strength
        wind_force = wind_force.reshape((3, 1))
        return wind_force

class RandomAdditiveNoiseWind(WindModel):
    def __init__(self, direction_vector=np.array([0.0, 1.0, 0.0]), strength=1.0, scale=1.0):
        super().__init__(direction_vector=direction_vector, strength=strength)
        self.scale = scale
    def ask_for_disturbance(self, time):
        wind_force = self.strength * self.direction_vector + np.random.normal(loc=0.0, scale=self.scale, size=(3))
        wind_force = wind_force.reshape((3, 1))
        return wind_force

class RandomWalkWind(WindModel):
    def __init__(self, direction_vector=np.array([0.0, 1.0, 0.0]), strength=1.0, dir_vec_scale=0.1, strength_scale=0.1, weight=0.5):
        super().__init__(direction_vector=direction_vector, strength=strength)
        self.dir_vec_scale = dir_vec_scale
        self.strength_scale = strength_scale
        self.weight = weight

    def ask_for_disturbance(self, time):
        if self.time_prev == time:
            pass
        else:
            self.direction_vector = self.direction_vector * (1-self.weight) + self.weight * np.random.normal(loc=0.0, scale=self.dir_vec_scale, size=3)
            self.direction_vector = self.direction_vector / (np.linalg.norm(self.direction_vector) + 1e-15)
            self.strength = self.strength + np.random.normal(loc=0.0, scale=self.strength_scale, size=1)
        wind_force = self.strength * self.direction_vector
        wind_force = wind_force.reshape((3, 1))
        return wind_force
class SinusoidalWind(WindModel):
    def __init__(self, sin_f, fs, direction_vector=np.array([0.0, 1.0, 0.0]), max_strength=1.0):
        #doesnt work ATM
        super().__init__(direction_vector=direction_vector, strength=max_strength)
        self.fs = fs
        self.Ts = 1 / fs
        self.sin_f = sin_f
        self.sin_t = 1 / sin_f
    def ask_for_disturbance(self, time):
        wind_force = self.strength * np.sin(2 * np.pi * self.sin_f * time) * self.direction_vector
        wind_force = wind_force.reshape((3, 1))
        return wind_force


if __name__ == "__main__":
    wind = WindModel([0, 0, 1])
    print(wind.ask_for_disturbance())
