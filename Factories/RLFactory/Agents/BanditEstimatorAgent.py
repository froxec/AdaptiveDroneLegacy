from Factories.ConfigurationsFactory.configurations import *
from Factories.ModelsFactory.linear_models import *
from Factories.GaussianProcessFactory.gaussian_process import *
from Factories.ToolsFactory.GeneralTools import RollBuffers
from Factories.ToolsFactory.GeneralTools import manhattan_distance
from typing import Type
class BanditEstimatorAgent():
    def __init__(self,
                 position_controller_conf: Type[PositionControllerConfiguration],
                 predictive_model: Type[LinearizedQuad],
                 gp: Type[EfficientGaussianProcess],
                 atomic_traj_samples_num: int,
                 deltaT: int):
        self.position_controller_conf = position_controller_conf
        self.predictive_model = predictive_model
        self.gp = gp
        states_num = self.position_controller_conf.position_controller.x_num
        controls_num = self.position_controller_conf.position_controller.u_num
        self.trajectory_buffer = RollBuffers(['state', 'state_prediction', 'control_input'],
                                         [(states_num,), (states_num,), (controls_num,)],
                                         buffer_size=atomic_traj_samples_num)
        self.actions_buffer = RollBuffers(['actions'], [(1,)], buffer_size=3)
        self.estimated_parameters = self.predictive_model.parameters
        self.deltaT = deltaT
        self.u_prev = None
        self.prediction_prev = None
        self.converged = False
    def __call__(self, x, u):
        if self.prediction_prev is None:
            self.prediction_prev = x
        if self.trajectory_buffer.full['state'] == True:
            # equivalent to environment.step()
            penalty = self.calculate_penalty()
            if not self.converged:
                self.update_gp(self.estimated_parameters['m'], penalty)
                self.gp.plot('../images/gp/')
                action = self.take_action()
                print("Action taken {}".format(action))
                self.actions_buffer.add_sample(['actions'], [action])
                self.estimated_parameters['m'] = action
                self.predictive_model.update_parameters(self.estimated_parameters)
            self.prediction_prev = x[:6]
            self.trajectory_buffer.flush()
        if self.u_prev is not None:
            x_predicted = self.predictive_model.discrete_prediction(self.prediction_prev, self.u_prev, self.deltaT)
            self.add_sample_to_buffer(x[:6], x_predicted, self.u_prev)
            self.prediction_prev = x_predicted
        if self.check_for_convergence():
            self.update_parameters()
            self.converged = True
        self.u_prev = u
    def update_gp(self, action, reward):
        self.gp(np.array(action).reshape(-1, 1), [reward])
    def take_action(self):
        best = self.gp.Thompson_sampling(mode='min', number_of_samples=1)
        action = best['best_action']
        return action
    def update_parameters(self):
        parameters = self.predictive_model.parameters
        u_ss = self.predictive_model.U_OP[:3]
        self.position_controller_conf.position_controller.update_model_parameters(parameters)
        self.position_controller_conf.input_converter.update(u_ss)
        self.position_controller_conf.output_converter.update(u_ss, parameters['Kt'])

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

    def check_for_convergence(self):
        if len(set(self.actions_buffer['actions'].flatten())) == 1 and self.actions_buffer.full['actions']:
            #print("Estimator converged. Mass equal {}".format(self.actions_buffer['actions'][0]))
            return True
        else:
            return False
