import threading
import csv
from datetime import datetime
import numpy as np

from Factories.ConfigurationsFactory.configurations import *
from Factories.ModelsFactory.linear_models import *
from Factories.GaussianProcessFactory.gaussian_process import *
from Factories.ToolsFactory.GeneralTools import RollBuffers
from Factories.ToolsFactory.GeneralTools import manhattan_distance, sigmoid_function, euclidean_distance
from Factories.DataManagementFactory.data_managers import ParametersManager
from Factories.DataManagementFactory.data_holders import DataHolder
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


# class BanditEstimatorThread(BanditEstimatorAgent, Thread):
#     def __init__(self,
#                  parameters_manager: Type[ParametersManager],
#                  predictive_model: Type[LinearizedQuad],
#                  gp: Type[EfficientGaussianProcess],
#                  atomic_traj_samples_num: int):
#         BanditEstimatorAgent.__init__(self,
#                                       parameters_manager=parameters_manager,
#                                       predictive_model=predictive_model,
#                                       gp=gp,
#                                       deltaT=None,
#                                       atomic_traj_samples_num=atomic_traj_samples_num)
#         Thread.__init__(self)
#         self.data_set_event = threading.Event()
#         self.data = {'x': None, 'u': None}
#         self.start()
#
#     def __call__(self, x, u):
#         if self.prediction_prev is None:
#             self.prediction_prev = x
#             self.start = time.time()
#         if self.trajectory_buffer.full['state'] == True:
#             self.deltaT = time.time() - self.start
#             self.start = time.time()
#             # equivalent to environment.step()
#             self.prediction_prev = x[:6]
#             penalty = self.calculate_penalty()
#             penalty = self.postprocess_penalty(penalty)
#             self.penalties_buffer.add_sample(['penalties'], [penalty])
#             self.penalty_history.append(penalty)
#             if not self.converged:
#                 self.update_gp(self.estimated_parameters_holder.m, penalty)
#                 self.gp.plot('./images/gp/')
#                 action = self.take_action()
#                 print("Action taken {}".format(action))
#                 self.actions_buffer.add_sample(['actions'], [action])
#                 self.estimated_parameters_holder.m = action
#                 self.predictive_model.update_parameters()
#             else:
#                 # conditions_changed = self.check_for_conditions_changes()
#                 conditions_changed=False
#                 if conditions_changed:
#                     self.converged = False
#                     self.penalty_min = np.mean(self.penalties_buffer['penalties'])
#                     self.gp.reset()
#             self.trajectory_buffer.flush()
#         if self.u_prev is not None:
#             x_predicted = self.predictive_model.discrete_prediction(self.prediction_prev, self.u_prev)
#             self.add_sample_to_buffer(x[:6], x_predicted, self.u_prev)
#             self.prediction_prev = x_predicted
#         if self.check_for_convergence() and not self.converged:
#             self.update_parameters()
#             self.converged = True
#             self.penalty_min = np.mean(self.penalties_buffer['penalties'])
#             self.actions_buffer.flush()
#         self.u_prev = u
#     def run(self):
#         while True:
#             self._control_execution()
#             self.__call__(self.data['x'], self.data['u'])
#
#
#     def _control_execution(self):
#         self.data_set_event.wait()
#         self.data_set_event.clear()

class BanditEstimatorAcceleration:
    def __init__(self,
                 parameters_manager: Type[ParametersManager],
                 prediction_model,
                 gp: Type[EfficientGaussianProcess],
                 convergence_checker,
                 sleeptime=1,
                 mode = 'ACCELERATION',
                 pen_moving_window=None,
                 actions_moving_window=None,
                 variance_threshold=0.2,
                 epsilon_episode_steps=0,
                 max_steps=np.Inf,
                 testing_mode=False,
                 save_images=False,
                 logs_path='./logs/'):
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
        self.save_images = save_images
        self.pen_moving_window = pen_moving_window
        self.actions_moving_window = actions_moving_window
        self.variance_threshold = variance_threshold
        if epsilon_episode_steps == 0:
            self.epsilon_episode = False
        else:
            self.epsilon_episode = True
        self.epsilon_episode_steps = epsilon_episode_steps
        self.i = 0
        self.current_step = 0
        self.max_steps = max_steps
        self.process_finished = False

        #mode
        self.available_modes = ['ACCELERATION_MEASUREMENT', 'ACCELERATION_CONTROL', 'VELOCITY_MEASUREMENT', 'VELOCITY_CONTROL']
        if mode not in self.available_modes:
            raise ValueError("Estimator mode not correct. Should be {}".format(self.available_modes))
        self.mode = mode
        self.testing_mode = testing_mode

        self.velocity_prev = None
        self.velocity_timer = time.time()

        #memory
        self.memory = {'penalty': [],
                       'normalized_penalty': [],
                       'actions': deque(maxlen=self.actions_moving_window)}
        #action init
        values_pool = self.gp.X.flatten()
        self.action = np.random.choice(values_pool, 1)[0]
        self.estimated_parameters_holder.m = self.action

        self.history = {'acceleration': [],
                        'a_hat': [],
                        'action': [],
                        'penalty': [],
                        'force_norm': [],
                        'angles': []}
        self.logs_path = logs_path

    def __call__(self, measurement, force_norm, angles, deltaT=None):
        mode = self.mode.split('_')
        if mode[0] == 'VELOCITY':
            if self.velocity_prev is None:
                self.velocity_prev = measurement
                return
            acceleration = self._convert_velocity_to_acceleration(measurement, deltaT)
        else:
            acceleration = measurement
        if not self.converged:
            self.memory['actions'].append(self.estimated_parameters_holder.m)
            print("Action:", self.estimated_parameters_holder.m)
            a_hat = self.prediction_model(force_norm=force_norm, angles=angles)
            penalty = self._calculate_penalty(a_hat, acceleration)
            self.memory['penalty'].append(penalty)
            # penalty = self._normalize_penalty_max_a(penalty, acceleration)
            # self.memory['normalized_penalty'].append(penalty)
            self.update_gp(self.estimated_parameters_holder.m, penalty)
            if self.save_images:
                self.gp.plot('./images/gp/')
            self.action = self.take_action()
            self.estimated_parameters_holder.m = self.action
            self.converged = self.convergence_checker(self.action)
            # append data
            self.history['acceleration'].append(list(acceleration))
            self.history['a_hat'].append(list(a_hat))
            self.history['action'].append(self.action)
            self.history['penalty'].append(penalty)
            self.history['force_norm'].append(force_norm)
            self.history['angles'].append(angles)
        if self.converged and not self.parameters_changed:
            parameters = self.get_parameters()
            print("Converged to {}".format(parameters))
            self.update_parameters(parameters)
            self.parameters_changed = True
        else:
            pass
        self.current_step += 1
        if self.current_step > self.max_steps or self.converged:
            self.process_finished = True
    def update_gp(self, action, reward):
        self.gp(np.array(action).reshape(-1, 1), [reward])
    def _calculate_penalty(self, a_hat, a):
        penalty = manhattan_distance(a, a_hat)
        return penalty

    def take_action(self):
        best = self.gp.Thompson_sampling(mode='min', number_of_samples=1)
        action = best['best_action']
        return action

    def _normalize_penalty_max_a(self, penalty, a):
        penalty = penalty / (np.linalg.norm(a) + 1)
        return penalty

    def _convert_velocity_to_acceleration(self, velocity, deltaT):
        if deltaT is None:
            raise ValueError("deltaT in velocity mode should be passed to estimator..")
        # current_time = time.time()
        # velocity_dt = current_time - self.velocity_timer
        acceleration = (velocity - self.velocity_prev) / deltaT
        self.velocity_prev = velocity
        # self.velocity_timer = current_time
        return acceleration

    def update_parameters(self, parameters):
        self.parameters_manager.update_parameters(parameters)

    def get_parameters(self):
        parameters = self.estimated_parameters_holder.get_data()
        parameters['m'] = self.convergence_checker.average
        return parameters

    def reset(self):
        # reset gp
        self.gp.reset()
        # reset convergence checker
        self.convergence_checker.reset()
        # reset parameters holders
        self.nominal_parameters_holder = DataHolder(self.prediction_model.parameters_holder.get_data())
        self.estimated_parameters_holder = DataHolder(self.prediction_model.parameters_holder.get_data())
        self.prediction_model.parameters_holder = self.estimated_parameters_holder
        #flags reset
        self.converged = False
        self.parameters_changed = False
        self.i = 0
        self.current_step = 0
        self.process_finished = False
        # reset history
        self.reset_history()

    def reset_history(self):
        self.history = {'acceleration': [],
                        'a_hat': [],
                        'action': [],
                        'penalty': [],
                        'force_norm': [],
                        'angles': []}

    def plot_history(self, signal_name):
        fig = go.Figure()
        data = self.memory[signal_name]
        fig.add_trace(go.Scatter(x=list(range(len(data))), y=data, name=signal_name))
        fig.show()

    def save_history_to_file(self):
        fieldnames = self.history.keys()
        filename = self.logs_path + datetime.now().strftime("%m-%d-%Y-%H:%M:%S") + '_estimation' + '.csv'
        file = open(filename, 'w')
        writer = csv.writer(file)
        data = []
        for key in self.history.keys():
            data.append(self.history[key])
        rows = zip(*data)
        writer.writerow(fieldnames)
        writer.writerows(rows)
        file.close()




class BanditEstimatorThread(BanditEstimatorAcceleration, Thread):
    def __init__(self,
                 parameters_manager: Type[ParametersManager],
                 prediction_model,
                 gp: Type[EfficientGaussianProcess],
                 convergence_checker,
                 sleeptime=1,
                 mode='ACCELERATION',
                 pen_moving_window=None,
                 actions_moving_window=None,
                 variance_threshold=0.2,
                 epsilon_episode_steps=0):
        super().__init__(parameters_manager=parameters_manager,
                 prediction_model=prediction_model,
                 gp=gp,
                 convergence_checker=convergence_checker,
                 sleeptime=sleeptime,
                 mode=mode,
                 pen_moving_window=pen_moving_window,
                 actions_moving_window=actions_moving_window,
                 variance_threshold=variance_threshold,
                 epsilon_episode_steps=epsilon_episode_steps)
        Thread.__init__(self)
        self.data_set_event = threading.Event()
        self.procedure_finished = threading.Event()
        # data
        self.data = {'measurement': None,
                     'force': None,
                     'angles': None}

        #time_control
        self.time = time.time()
        self.start()
    def run(self):
        while True:
            measurement, force_norm, angles = self.get_data()
            self.estimate_parameters(measurement, force_norm, angles)

    def estimate_parameters(self, measurement, force_norm, angles):
        mode = self.mode.split('_')
        if mode[0] == 'VELOCITY':
            if self.velocity_prev is None:
                self.velocity_prev = measurement
                return
            current_time = time.time()
            deltaT = current_time - self.time
            self.time = current_time
            acceleration = self._convert_velocity_to_acceleration(measurement, deltaT)
        else:
            acceleration = measurement
        if self.i < self.epsilon_episode_steps:
            print("Epsilon episode.. Calibration..")
            self._epsilon_episode_step(acceleration, force_norm, angles)
            self.i += 1
            return
        if not self.converged:
            self.memory['actions'].append(self.estimated_parameters_holder.m)
            print("Action:", self.estimated_parameters_holder.m)
            a_hat = self.prediction_model(force_norm=force_norm, angles=angles)
            penalty = self._calculate_penalty(a_hat, acceleration)
            self.memory['penalty'].append(penalty)
            # penalty = self._normalize_penalty_max_a(penalty, acceleration)
            # self.memory['normalized_penalty'].append(penalty)
            self.update_gp(self.estimated_parameters_holder.m, penalty)
            self.gp.plot('./images/gp_rt/')
            self.action = self.take_action()
            self.estimated_parameters_holder.m = self.action
            self.converged = self.convergence_checker(self.action)
        elif not self.parameters_changed:
            parameters = self.get_parameters()
            print("Converged to {}".format(parameters))
            self.update_parameters(parameters)
            self.parameters_changed = True
        else:
            self.save_history_to_file()
            self.procedure_finished.set()
            time.sleep(1)

    def get_data(self):
        self.data_set_event.wait()
        return self.data['measurement'], self.data['force'], self.data['angles']

class BanditEstimatorAccelerationProcess(BanditEstimatorAcceleration):
    def __init__(self,
                 db_interface,
                 prediction_model,
                 gp: Type[EfficientGaussianProcess],
                 convergence_checker,
                 sleeptime=1,
                 mode = 'ACCELERATION',
                 pen_moving_window=None,
                 actions_moving_window=None,
                 variance_threshold=0.2,
                 epsilon_episode_steps=0,
                 max_steps=np.Inf,
                 testing_mode=False,
                 save_images=False):
        BanditEstimatorAcceleration.__init__(self,
                                             None,
                                             prediction_model,
                                             gp,
                                             convergence_checker,
                                             sleeptime,
                                             mode,
                                             pen_moving_window,
                                             actions_moving_window,
                                             variance_threshold,
                                             epsilon_episode_steps,
                                             max_steps,
                                             testing_mode,
                                             save_images)
        self.db_interface = db_interface
    def update_parameters(self, parameters):
        self.db_interface.update_parameters(parameters)

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