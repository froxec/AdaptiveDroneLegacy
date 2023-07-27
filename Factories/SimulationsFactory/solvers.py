import copy
import numpy as np

class ODE_Solver:
    def __init__(self, parent=None):
        self.parent_simulation = parent
        pass
    def __call__(self, **kwargs):
        return self._step(**kwargs)

    def _step(self, **kwargs):
        raise NotImplementedError("_step method not implemented in child class..")


class RungeKutta4(ODE_Solver):
    def __init__(self, model_object, parent=None):
        super().__init__(parent=parent)
        self.model = copy.deepcopy(model_object)

    def _step(self, delta_t, model_object, u=np.array([None])):
        self.model.__dict__.update(model_object.__dict__)
        state0 = self.model.state
        if u.any() == None:
            k1 = delta_t * self.model.updateStateOde(state0)
            k2 = delta_t * self.model.updateStateOde(state0 + 0.5 * k1)
            k3 = delta_t * self.model.updateStateOde(state0 + 0.5 * k2)
            k4 = delta_t * self.model.updateStateOde(state0 + k3)
        else:
            k1 = delta_t * self.model.updateStateOde(state0, u)
            k2 = delta_t * self.model.updateStateOde(state0 + 0.5 * k1, u)
            k3 = delta_t * self.model.updateStateOde(state0 + 0.5 * k2, u)
            k4 = delta_t * self.model.updateStateOde(state0 + k3, u)
        model_object.state = model_object.state + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return model_object.state

solvers = {'RK': RungeKutta4}