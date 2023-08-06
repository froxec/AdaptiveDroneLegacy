import threading

import numpy as np

from Factories.ConfigurationsFactory.configurations import *
from Factories.ModelsFactory.linear_models import *
from Factories.GaussianProcessFactory.gaussian_process import *
from Factories.ToolsFactory.GeneralTools import RollBuffers
from Factories.ToolsFactory.GeneralTools import manhattan_distance, sigmoid_function
from Factories.DataManagementFactory.data_managers import ParametersManager
from typing import Type
import plotly.graph_objects as go
from threading import Thread
import time
from collections import Counter, deque
class BanditEstimatorAgent():
    def __init__(self,
                 parameters_manager: Type[ParametersManager],
                 predictive_model: Type[LinearizedQuad],
                 gp: Type[EfficientGaussianProcess],
                 deltaT,
                 atomic_traj_samples_num: int):
        self.parameters_manager = parameters_manager
        self.predictive_model = deepcopy(predictive_model)
        self.gp = gp
        self.nominal_parameters_holder = DataHolder(self.predictive_model.parameters_holder.get_data())
        self.estimated_parameters_holder = DataHolder(self.predictive_model.parameters_holder.get_data())
        self.predictive_model.parameters_holder = self.estimated_parameters_holder
        states_num = self.predictive_model.Ad.shape[0]
        controls_num = self.predictive_model.Bd.shape[1]
        self.trajectory_buffer = RollBuffers(['state', 'state_prediction', 'control_input'],
                                         [(states_num,), (states_num,), (controls_num,)],
                                         buffer_size=atomic_traj_samples_num)
        self.actions_buffer = RollBuffers(['actions'], [(1,)], buffer_size=3)
        self.penalties_buffer = RollBuffers(['penalties'], [(1,)], buffer_size=3)
        self.penalty_min = None
        self.deltaT = deltaT
        self.samples_num = atomic_traj_samples_num
        self.u_prev = None
        self.prediction_prev = None
        self.converged = False
        self.penalty_history = []
    def __call__(self, x, u):
        if self.prediction_prev is None:
            self.prediction_prev = x
        if self.trajectory_buffer.full['state'] == True:
            # equivalent to environment.step()
            self.prediction_prev = x[:6]
            penalty = self.calculate_penalty()
            penalty = self.postprocess_penalty(penalty)
            self.penalties_buffer.add_sample(['penalties'], [penalty])
            self.penalty_history.append(penalty)
            if not self.converged:
                self.update_gp(self.estimated_parameters_holder.m, penalty)
                self.gp.plot('./images/gp/')
                action = self.take_action()
                print("Action taken {}".format(action))
                self.actions_buffer.add_sample(['actions'], [action])
                self.estimated_parameters_holder.m = action
                self.predictive_model.update_parameters()
            else:
                # conditions_changed = self.check_for_conditions_changes()
                conditions_changed=False
                if conditions_changed:
                    self.converged = False
                    self.penalty_min = np.mean(self.penalties_buffer['penalties'])
                    self.gp.reset()
            self.trajectory_buffer.flush()
        if self.u_prev is not None:
            x_predicted = self.predictive_model.discrete_prediction(self.prediction_prev, self.u_prev)
            self.add_sample_to_buffer(x[:6], x_predicted, self.u_prev)
            self.prediction_prev = x_predicted
        if self.check_for_convergence() and not self.converged:
            self.update_parameters()
            self.converged = True
            self.penalty_min = np.mean(self.penalties_buffer['penalties'])
            self.actions_buffer.flush()
        self.u_prev = u
    def update_gp(self, action, reward):
        self.gp(np.array(action).reshape(-1, 1), [reward])
    def take_action(self):
        best = self.gp.Thompson_sampling(mode='min', number_of_samples=1)
        action = best['best_action']
        return action
    def update_parameters(self):
        parameters = self.predictive_model.parameters
        self.parameters_manager.update_parameters(parameters)

    def add_sample_to_buffer(self, state, state_prediction, control_input):
        self.trajectory_buffer.add_sample(['state', 'state_prediction', 'control_input'], [state, state_prediction, control_input])
    def convert_trajectory(self, velocity_trajectory, input_trajectory, deltaT):
        ## pitch, roll, yaw are ommited (not required)
        velocity_trajectory = np.flip(velocity_trajectory, axis=0)
        input_trajectory = np.flip(input_trajectory)
        average_input = np.mean(input_trajectory)
        V_norm = np.linalg.norm(velocity_trajectory, axis=1)
        acceleration = (V_norm[1:] - V_norm[:-1]) / deltaT
        average_acceleration = np.mean(acceleration)
        return average_input, average_acceleration
        '''
        :param trajectory: subsequent state x and input u samples 
        :return: trajectory converted to average throttle input and average acceleration over time horizon
        '''
    def calculate_penalty(self):
        reward = 0
        state, state_prediction = self.trajectory_buffer['state'][:, 3:], self.trajectory_buffer['state_prediction'][:, 3:]
        # normalized_state = self.normalize_trajectory(state)
        # normalized_prediction = self.normalize_trajectory(state_prediction)
        for i in range(len(self.trajectory_buffer)):
            reward += manhattan_distance(state[i], state_prediction[i])*self.deltaT
        return reward
    def postprocess_penalty(self, penalty):
        penalty = self.normalize_penalty(penalty)
        return (sigmoid_function(0.2*penalty**2 , 10, 0))

    def normalize_penalty(self, penalty):
        return penalty/(self.samples_num*self.deltaT)

    def check_for_convergence(self):
        if len(set(self.actions_buffer['actions'].flatten())) == 1 and self.actions_buffer.full['actions']:
            #print("Estimator converged. Mass equal {}".format(self.actions_buffer['actions'][0]))
            return True
        else:
            return False

    def check_for_conditions_changes(self):
        if not self.penalties_buffer.full['penalties']:
            return False
        penalties = self.penalties_buffer['penalties']
        mean_penalty = np.mean(penalties)
        #print(penalties)
        if mean_penalty > 1.2*self.penalty_min:
            return True
        else:
            return False

    def plot(self):
        fig = go.Figure()

        fig.add_trace(go.Scatter(x=list(range(len(self.penalty_history))), y=self.penalty_history, name='penalty_history'))


class BanditEstimatorThread(BanditEstimatorAgent, Thread):
    def __init__(self,
                 parameters_manager: Type[ParametersManager],
                 predictive_model: Type[LinearizedQuad],
                 gp: Type[EfficientGaussianProcess],
                 atomic_traj_samples_num: int):
        BanditEstimatorAgent.__init__(self,
                                      parameters_manager=parameters_manager,
                                      predictive_model=predictive_model,
                                      gp=gp,
                                      deltaT=None,
                                      atomic_traj_samples_num=atomic_traj_samples_num)
        Thread.__init__(self)
        self.data_set_event = threading.Event()
        self.data = {'x': None, 'u': None}
        self.start()

    def __call__(self, x, u):
        if self.prediction_prev is None:
            self.prediction_prev = x
            self.start = time.time()
        if self.trajectory_buffer.full['state'] == True:
            self.deltaT = time.time() - self.start
            self.start = time.time()
            # equivalent to environment.step()
            self.prediction_prev = x[:6]
            penalty = self.calculate_penalty()
            penalty = self.postprocess_penalty(penalty)
            self.penalties_buffer.add_sample(['penalties'], [penalty])
            self.penalty_history.append(penalty)
            if not self.converged:
                self.update_gp(self.estimated_parameters_holder.m, penalty)
                self.gp.plot('./images/gp/')
                action = self.take_action()
                print("Action taken {}".format(action))
                self.actions_buffer.add_sample(['actions'], [action])
                self.estimated_parameters_holder.m = action
                self.predictive_model.update_parameters()
            else:
                # conditions_changed = self.check_for_conditions_changes()
                conditions_changed=False
                if conditions_changed:
                    self.converged = False
                    self.penalty_min = np.mean(self.penalties_buffer['penalties'])
                    self.gp.reset()
            self.trajectory_buffer.flush()
        if self.u_prev is not None:
            x_predicted = self.predictive_model.discrete_prediction(self.prediction_prev, self.u_prev)
            self.add_sample_to_buffer(x[:6], x_predicted, self.u_prev)
            self.prediction_prev = x_predicted
        if self.check_for_convergence() and not self.converged:
            self.update_parameters()
            self.converged = True
            self.penalty_min = np.mean(self.penalties_buffer['penalties'])
            self.actions_buffer.flush()
        self.u_prev = u
    def run(self):
        while True:
            self._control_execution()
            self.__call__(self.data['x'], self.data['u'])


    def _control_execution(self):
        self.data_set_event.wait()
        self.data_set_event.clear()

class BanditEstimatorAcceleration:
    def __init__(self,
                 parameters_manager: Type[ParametersManager],
                 prediction_model,
                 gp: Type[EfficientGaussianProcess],
                 convergence_checker,
                 sleeptime=1):
        self.parameters_manager = parameters_manager
        self.prediction_model = prediction_model
        self.gp = gp
        self.convergence_checker = convergence_checker

        # parameters holders
        self.nominal_parameters_holder = DataHolder(self.prediction_model.parameters_holder.get_data())
        self.estimated_parameters_holder = DataHolder(self.prediction_model.parameters_holder.get_data())
        self.prediction_model.parameters_holder = self.estimated_parameters_holder

        # flags
        self.converged = False
        self.parameters_changed = False
        self.sleeptime = sleeptime
    def __call__(self, acceleration, force_norm, angles):
        if not self.converged:
            a_hat = self.prediction_model(force_norm=force_norm, angles=angles)
            penalty = self._calculate_penalty(a_hat, acceleration)
            penalty = self._normalize_penalty(penalty, acceleration)
            self.update_gp(self.estimated_parameters_holder.m, penalty)
            #self.gp.plot('./images/gp/')
            action = self.take_action()
            self.estimated_parameters_holder.m = action
            self.converged = self.convergence_checker(action)
        elif not self.parameters_changed:
            parameters = self.get_parameters()
            print("Converged to {}".format(parameters))
            self.update_parameters(parameters)
            self.parameters_changed = True
        else:
            pass

    def update_gp(self, action, reward):
        self.gp(np.array(action).reshape(-1, 1), [reward])
    def _calculate_penalty(self, a_hat, a):
        reward = manhattan_distance(a_hat, a)
        return reward

    def take_action(self):
        best = self.gp.Thompson_sampling(mode='min', number_of_samples=1)
        action = best['best_action']
        return action

    def _normalize_penalty(self, penalty, a):
        penalty = penalty / (np.linalg.norm(a) + 1)
        return penalty

    def update_parameters(self, parameters):
        self.parameters_manager.update_parameters(parameters)

    def get_parameters(self):
        parameters = self.estimated_parameters_holder.get_data()
        parameters['m'] = self.convergence_checker.average
        return parameters

if __name__ == "__main__":
    import plotly.express as px

    n = 100
    data = np.linspace(0, 10, 11)
    p = list(range(1, 12))
    p = np.array(p) / sum(p)
    print(p)
    x = deque(maxlen=n)
    for i in range(n):
        element = np.random.choice(data, 1, p=p).item()
        x.appendleft(element)
    cnt = Counter(x)
    data_to_plot = {'keys': list(cnt.keys()), 'values': list(cnt.values())}
    fig = px.bar(data_to_plot, x="keys", y="values")
    fig.show()
    avg = np.average(np.array(list(cnt.keys())),weights=np.array(list(cnt.values())))
    print("Average",  avg)