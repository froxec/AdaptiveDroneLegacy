import numpy as np

class ExternalDisturbance:
    def __init__(self):
        self.time_prev = 0.0
    def __call__(self, time):
        disturbance = self.ask_for_disturbance(time)
        self.time_prev = time
        return disturbance

    def ask_for_disturbance(self):
        raise NotImplementedError("Method ask_for_disturbance() should be implemented in child class..")
class WindModel(ExternalDisturbance):
    def __init__(self, direction_vector=np.array([0.0, 1.0, 0.0]), strength=1.0):
        super().__init__()
        if isinstance(direction_vector, list):
            direction_vector = np.array(direction_vector)
        direction_vector = direction_vector.astype('float32')
        if np.linalg.norm(direction_vector) != 1.0:
            self.direction_vector = direction_vector / np.linalg.norm(direction_vector)
        else:
            self.direction_vector = direction_vector
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
    def __init__(self, direction_vector=np.array([0.0, 1.0, 0.0]), strength=1.0, dir_vec_scale=0.1, strength_scale=0.1):
        super().__init__(direction_vector=direction_vector, strength=strength)
        self.dir_vec_scale = dir_vec_scale
        self.strength_scale = strength_scale

    def ask_for_disturbance(self, time):
        if self.time_prev == time:
            pass
        else:
            self.direction_vector = self.direction_vector + np.random.normal(loc=0.0, scale=self.dir_vec_scale, size=3)
            self.direction_vector = self.direction_vector / np.linalg.norm(self.direction_vector)
            self.strength = self.strength + np.random.normal(loc=0.0, scale=self.strength_scale, size=1)
        wind_force = self.strength * self.direction_vector
        wind_force = wind_force.reshape((3, 1))
        print("Wind force:", wind_force.flatten())
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
