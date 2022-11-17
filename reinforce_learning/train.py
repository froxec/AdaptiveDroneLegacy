import numpy as np
from quad_model import QuadAccelerationModel
from Simulation.model_parameters import quad_parameters
class Environment:
    def __init__(self, quad_parameters):
        self.ground_truth_quad = QuadAccelerationModel(quad_parameters)
        quad_parameters_random_mass = quad_parameters
        quad_parameters_random_mass['m'] = np.random.random()*10 + 10
        self.estimated_quad = QuadAccelerationModel(quad_parameters_random_mass)
        self.state_ranges = [100, 100, 100, 10, 10, 10, np.pi/6, np.pi/6, np.pi/6, 10, 10, 10]
    def __call__(self, mass_perturbation):
        x0 = np.array([np.random.random()*var_range for var_range in self.state_ranges])
        self.estimated_quad.mass += mass_perturbation
        rotors_speed = np.random.random(4)*1500
        accelerations = np.array(self.estimated_quad(x0, rotors_speed))
        reward = self.reward(x0, rotors_speed)
        return reward, np.concatenate((accelerations, rotors_speed))
    def reward(self, x0, rotors_speed):
        state_truth = np.array([self.ground_truth_quad(x0, rotors_speed)])
        state_estimated = np.array([self.estimated_quad(x0, rotors_speed)])
        delta = abs(state_truth - state_estimated)
        return -(np.sum(delta))**2

state_action_limits = {
    'accelerations': {'lower': -20, 'upper': 20},
    'rotor_speeds': {'lower': 0, 'upper': 1500},
    'delta_m': {'lower': -0.1, 'upper': 0.1}
}

def generate_grids(grids_num, intervals_num, states_names, limits):
    states_num = len(states_names)
    grids = np.zeros((grids_num, states_num, intervals_num-1)) #(grids_num, states_num, intervals_num)
    for i in range(grids_num):
        discrete_space = np.array([np.linspace(limits[state]['lower'], limits[state]['upper'], intervals_num - 1) for num, state in enumerate(states_names)])
        for j, state_space_bounds in enumerate(discrete_space):
            for k, bound in enumerate(state_space_bounds[1:], start = 1):
                if k >= state_space_bounds.shape[0] - 1:
                    break
                polarity = np.random.choice([-1, 1])
                interval = abs(discrete_space[j, k + 1] - discrete_space[j, k - 1])
                shift = polarity*(np.random.random()*interval)/2
                bound = discrete_space[j, k - 1] + interval/2 + shift
                discrete_space[j, k] = bound
        grids[i, :, :] = discrete_space
    return grids

def code_state(state, grids):
    state_indices = np.zeros((grids.shape[0], state.shape[0]))
    state_len = state.shape[0]
    for i, grid in enumerate(grids):
        for j in range(state_len):
            index = np.digitize(state[j], grid[j])
            state_indices[i, j] = index
    return state_indices.astype('int')

def convert_to_features(coded_state, coded_action, intervals_num):
    grids_num = coded_state.shape[0]
    features = np.zeros((grids_num, intervals_num, intervals_num, intervals_num, intervals_num, intervals_num))
    for i in range(grids_num):
        features[i, coded_state[i, 0], coded_state[i, 1], coded_state[i, 2], coded_state[i, 3], coded_action[i]] = 1
    return features.flatten()

def greedy_epsilon(Q, actions, eps=0.1):
    prob = np.random.random()
    if prob < (1-eps):
        return actions[np.argmax(Q)]
    else:
        return np.random.random()*2000 - 1000

def calculate_q(weights, features):
    q = np.matmul(weights, features)
    return q

def main():
    gamma = 0.98
    alpha = 0.001
    lamb  = 0.95
    actions_num = 3
    grids_num = 15
    intervals_num = 10
    state_grids = generate_grids(grids_num, intervals_num, ['accelerations', 'accelerations', 'accelerations', 'rotor_speeds', 'rotor_speeds', 'rotor_speeds', 'rotor_speeds'], state_action_limits)
    action_grids = generate_grids(grids_num, intervals_num, ['delta_m'], state_action_limits)
    env = Environment(quad_parameters)

    reward = -1000
    actions = np.linspace(state_action_limits['delta_m']['lower'], state_action_limits['delta_m']['upper'], actions_num)
    actions_coded = []
    for action in actions:
        actions_coded.append(code_state(np.array([action]), action_grids))

    liczba_wag = pow(intervals_num, 5) * grids_num
    w = np.ones(liczba_wag)
    z = np.zeros(liczba_wag)
    state = np.zeros(7)
    step = 0
    while step < 10000:
        step = step + 1
        coded_state = code_state(state, state_grids)
        if step == 1:
            q = []
            for coded_action in actions_coded:
                features = convert_to_features(coded_state, coded_action, intervals_num)
                q.append(calculate_q(w, features))
                action = actions[np.argmax(q)]
        else:
            q = qnew
            action = action_new
        #action = greedy_epsilon(q, action)
        reward, new_state = env(action)
        coded_new_state = code_state(new_state, state_grids)
        qnew = []
        for coded_action in actions_coded:
            features_new = convert_to_features(coded_new_state, coded_action, intervals_num)
            qnew.append(calculate_q(w, features_new))
            action_new = actions[np.argmax(qnew)]
        gradient = features
        delta = reward + gamma*max(qnew) + max(q)
        z = lamb*gamma*z + gradient
        w = w + alpha*delta*z
        #print("Reward", reward)
        print(env.estimated_quad.mass)
if __name__ == '__main__':
    main()