from Factories.RLFactory.Agents.quad_mass_estimation_agent import QuadMassEstimator, PolicyGradientLearning, ReplayBuffer

SAMPLING_FREQ = 10 #Hz
STEP_TIME = 1 #s
SIGNALS_NUMBER = 3 #control input, predicted trajectory, real trajectory
MAX_EPISODE_TIME = 15 #s
EPISODES_NUM = 1000
ESTIMATOR_INPUT_SHAPE = (SIGNALS_NUMBER, STEP_TIME*SAMPLING_FREQ)
ESTIMATOR_OUTPUT_SHAPE = 2 #mean and variance
GAMMA = 0.0
ALPHA = 1e-5

if __name__ == "__main__":
    mass_estimator = QuadMassEstimator(ESTIMATOR_INPUT_SHAPE, ESTIMATOR_OUTPUT_SHAPE)
    learning_algorithm = PolicyGradientLearning(mass_estimator.policy_network, ALPHA, GAMMA)
    replay_buffer = ReplayBuffer(ESTIMATOR_INPUT_SHAPE)

    for i in range(1000):
