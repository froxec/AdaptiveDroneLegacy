import numpy as np

class ExternalDisturbance:
    def __call__(self):
        return self.ask_for_disturbance()

    def ask_for_disturbance(self):
        raise NotImplementedError("Method ask_for_disturbance() should be implemented in child class..")
class WindModel(ExternalDisturbance):
    def __init__(self, direction_vector=np.array([1.0, 1.0, 0.0]), strength=1.0):
        if isinstance(direction_vector, list):
            direction_vector = np.array(direction_vector)
        direction_vector = direction_vector.astype('float32')
        if np.linalg.norm(direction_vector) != 1.0:
            self.direction_vector = direction_vector / np.linalg.norm(direction_vector)
        else:
            self.direction_vector = direction_vector
        self.strength = strength

    def ask_for_disturbance(self):
        wind_force = self.direction_vector * self.strength
        wind_force = wind_force.reshape((3, 1))
        return wind_force

if __name__ == "__main__":
    wind = WindModel([0, 0, 1])
    print(wind.ask_for_disturbance())