import numpy as np
from quad_model import QuadAccelerationModel
from Simulation.model_parameters import quad_parameters
import copy
class Environment:
    def __init__(self, quad_parameters):
        self.ground_truth_quad = QuadAccelerationModel(quad_parameters)
        quad_parameters_random_mass = quad_parameters
        quad_parameters_random_mass['m'] = -np.random.random()*10
        self.estimated_quad = QuadAccelerationModel(quad_parameters_random_mass)
        self.state_ranges = [100, 100, 100, 10, 10, 10, np.pi/6, np.pi/6, np.pi/6, 10, 10, 10]
    def __call__(self, mass_perturbation):
        x0 = np.array([np.random.random()*var_range for var_range in self.state_ranges])
        self.update_mass(mass_perturbation)
        rotors_speed = np.random.random(4)*1500
        accelerations = np.array(self.estimated_quad(x0, rotors_speed))
        reward = self.reward(x0, rotors_speed)
        return reward, np.concatenate((accelerations, rotors_speed, np.array([self.estimated_quad.mass])))
    def reward(self, x0, rotors_speed):
        if self.estimated_quad.mass < 0.1:
            return -1000
        state_truth = np.array([self.ground_truth_quad(x0, rotors_speed)])
        state_estimated = np.array([self.estimated_quad(x0, rotors_speed)])
        delta = abs(state_truth - state_estimated)
        return -(np.linalg.norm(delta)/np.linalg.norm(state_truth))**2
    def update_mass(self, delta_m):
        self.estimated_quad.mass += delta_m
        if self.estimated_quad.mass < 0.05:
            self.estimated_quad.mass = 0.05

state_action_limits = {
    'accelerations': {'lower': -20, 'upper': 20},
    'rotor_speeds': {'lower': 0, 'upper': 1500},
    'delta_m': {'lower': -0.1, 'upper': 0.1},
    'a_omega_proportion':  {'lower': -0.1, 'upper':0.1},
    'mass_estimation': {'lower': 0.05, 'upper': 10}
}

def generate_points(points_number, number_of_dimensions, var):
    center = np.zeros(number_of_dimensions)
    #covariance = np.diag((state_action_std['theta'], state_action_std['omega'], state_action_std['x'], state_action_std['V']**2, state_action_std['F']))
    covariance = np.diag(var)
    prototype_points = np.random.multivariate_normal(center, covariance, size=(points_number))
    return prototype_points

def normalize_variables(variables, var_symbols, limits):
    if not isinstance(variables, np.ndarray):
        variables = np.array([variables])
    variables_copy = copy.deepcopy(variables)
    for i, var_symbol in enumerate(var_symbols):
        var_range = limits[var_symbol]['upper'] - limits[var_symbol]['lower']
        variables_copy[i] = (variables[i] - limits[var_symbol]['lower'])/var_range
    return variables_copy

def code_state(prototype_points, states, actions, threshold):
    states_num = states.shape[0]
    actions_num = actions.shape[0]
    state_action = np.concatenate((states, actions)).reshape((1, states_num + actions_num))
    #distance = np.linalg.norm(prototype_points - state_action, axis = 1)
    distance = np.sum(np.abs(prototype_points - state_action), axis = 1) #Manhattan distance
    return (distance < threshold).astype('int')

def greedy_epsilon(Q, actions, eps=0.1):
    prob = np.random.random()
    if prob < (1-eps):
        return actions[np.argmax(Q)]
    else:
        return np.random.random()*(abs(state_action_limits['delta_m']['upper'] - state_action_limits['delta_m']['lower'])) + state_action_limits['delta_m']['lower']

def calculate_q(weights, features):
    q = np.matmul(weights, features)
    return q
def a_omega_proportion(state):
    acceleration_norm = np.linalg.norm(state[:3])
    omega_norm = np.linalg.norm(state[3:])
    if omega_norm < 10:
        omega_norm = 10
    return acceleration_norm/omega_norm

def main():
    gamma = 0.98
    alpha = 0.001
    lamb  = 0.95
    actions_num = 3
    prototype_num = 100
    distance_threshold = 3
    dimensionality = 3
    variance = np.ones(dimensionality)
    eps_coef = 1000
    steps_num = 100000

    env = Environment(quad_parameters)
    prototype_points = generate_points(prototype_num, dimensionality, variance)
    actions = np.linspace(state_action_limits['delta_m']['lower'], state_action_limits['delta_m']['upper'], actions_num)
    actions_normalized = []
    for action in actions:
        normalized_action = normalize_variables(action, ['delta_m'], state_action_limits)
        actions_normalized.append(normalized_action)
    liczba_wag = prototype_num
    w = np.ones(liczba_wag)
    z = np.zeros(liczba_wag)
    a_omega_prop = a_omega_proportion(np.zeros(7))
    state = np.array([env.estimated_quad.mass, a_omega_prop])
    step = 0
    while step < steps_num:
        epsilon = np.exp(-eps_coef*step/steps_num)
        step = step + 1
        if step == 1:
            q = []
            state_normalized = normalize_variables(state, ['a_omega_proportion', 'mass_estimation'], state_action_limits)
            for action_normalized in actions_normalized:
                features = code_state(prototype_points, state_normalized, action_normalized, distance_threshold)
                q.append(calculate_q(w, features))
        else:
            q = qnew
            state_normalized = new_state_normalized
        action = greedy_epsilon(q, actions, epsilon)
        action_ind = np.digitize(action, actions[1:-1])
        action_normalized = normalize_variables(action, ['delta_m'], state_action_limits)
        gradient = code_state(prototype_points, state_normalized, action_normalized, distance_threshold)
        reward, new_state = env(action)
        a_omega_prop = a_omega_proportion(new_state[:7])
        new_state = np.array([a_omega_prop, new_state[-1]])
        new_state_normalized = normalize_variables(new_state, ['a_omega_proportion', 'mass_estimation'], state_action_limits)
        qnew = []
        for action_normalized in actions_normalized:
            features_new = code_state(prototype_points, new_state_normalized, action_normalized, distance_threshold)
            qnew.append(calculate_q(w, features_new))
        action_new = greedy_epsilon(qnew, actions, epsilon)
        action_new_ind = np.digitize(action_new, actions[1:-1])

        delta = reward + gamma*qnew[action_new_ind] + q[action_ind]
        z = lamb*gamma*z + gradient
        w = w + alpha*delta*z
        print("Reward", reward)
        #print(env.estimated_quad.mass)
        #print(epsilon)
        #print(sum(gradient))
if __name__ == '__main__':
    main()