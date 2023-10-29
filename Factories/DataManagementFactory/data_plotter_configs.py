SIM_ESTIM_TESTS_CONF = {
    'base_path': '/home/pete/PycharmProjects/AdaptiveDrone/logs/sim_official_new/',
    'file_names': ['load0_1762.csv', 'load300_2066.csv', 'load500_2268.csv'],
    'cuts': [(0, 1), (0, 1), (0, 1)],
    'height_shifts': [0.0, 0.0, 0.0],
    'reference_points': [[-10, -10, 6],
                        [-10, 10, 6],
                        [10, 10, 6],
                        [-10, 10, 6],
                        [-10, -10, 6]],
    'reference_shift': 220,
    'legend': [r'm_{load} = 0.0 kg', r'm_{load} = 0.2 kg', r'm_{load} = 0.4 kg']
}

SIM_MPC_TESTS_CONF2 = {
    'base_path': '/home/pete/PycharmProjects/AdaptiveDrone/logs/sim_official_new2/',
    'file_names': ['load0g.csv', 'load250.csv', 'load500.csv'],
    'cuts': [(0.01, 1), (0, 1), (0.01, 1)],
    'height_shifts': [0.0, 0.0, 0.0],
    'reference_points': [[-10, -10, 6],
                         [-10, 10, 6],
                         [10, 10, 6],
                         [-10, 10, 6],
                         [-10, -10, 6]],
    'reference_shift': 220,
    'legend': [r'm_{load} = 0.0 kg', r'm_{load} = 0.25 kg', r'm_{load} = 0.50 kg']
}

SIM_MPC_ADA_TESTS_CONF2 = {
    'base_path': '/home/pete/PycharmProjects/AdaptiveDrone/logs/sim_official_new2/',
    'file_names': ['load0g_adaON.csv', 'load250_adaON.csv', 'load500_adaON.csv'],
    'cuts': [(0, 1), (0.01, 1), (0, 1)],
    'height_shifts': [0.0, 0.0, 0.0],
    'reference_points': [[-10, -10, 6],
                         [-10, 10, 6],
                         [10, 10, 6],
                         [-10, 10, 6],
                         [-10, -10, 6]],
    'reference_shift': 220,
    'legend': [r'm_{load} = 0.0 kg', r'm_{load} = 0.25 kg', r'm_{load} = 0.50 kg']
}

SIM_ESTIM_TESTS_CONF2 = {
    'base_path': '/home/pete/PycharmProjects/AdaptiveDrone/logs/sim_official_new2/',
    'file_names': ['load0g_1749.csv', 'load250_2026.csv', 'load500_2259.csv'],
    'cuts': [(0, 1), (0, 1), (0, 1)],
    'height_shifts': [0.0, 0.0, 0.0],
    'reference_points': [[-10, -10, 6],
                         [-10, 10, 6],
                         [10, 10, 6],
                         [-10, 10, 6],
                         [-10, -10, 6]],
    'reference_shift': 220,
    'legend': [r'm_{load} = 0.0 kg', r'm_{load} = 0.25 kg', r'm_{load} = 0.50 kg']
}

SIM_ESTIM_TESTS_ADA_CONF2 = {
    'base_path': '/home/pete/PycharmProjects/AdaptiveDrone/logs/sim_official_new2/',
    'file_names': ['load0g_1749_adaON.csv', 'load250_2026_adaON.csv', 'load500_2259_adaON.csv'],
    'cuts': [(0, 1), (0, 1), (0, 1)],
    'height_shifts': [0.0, 0.0, 0.0],
    'reference_points': [[-10, -10, 6],
                         [-10, 10, 6],
                         [10, 10, 6],
                         [-10, 10, 6],
                         [-10, -10, 6]],
    'reference_shift': 220,
    'legend': [r'm_{load} = 0.0 kg', r'm_{load} = 0.25 kg', r'm_{load} = 0.50 kg']
}

ONE_FILE_CONF = {
    'base_path': '/home/pete/PycharmProjects/AdaptiveDrone/logs/18_10/',
    'file_names': ['10-17-2023-16:19:27_estim400gRPI.csv'],
    'cuts': [(0, 1)],
    'height_shifts': [0.0],
    'reference_points': [[-10, -10, 6],
                        [-10, 10, 6],
                        [10, 10, 6],
                        [-10, 10, 6],
                        [-10, -10, 6]],
    'reference_shift': 220,
    'legend': [r'none']
}

FIELD_ESTIM_TESTS_CONF = {
    'base_path': '/home/pete/PycharmProjects/AdaptiveDrone/logs/field_tests_official/estim_tests2/',
    'file_names': ['estim0g.csv', 'estim200g.csv', 'estim400g3.csv'],
    'cuts': [(0.42, 0.7), (0.45, 0.62), (0.82, 0.93)],
    'height_shifts': [0.0, 0.0, 0.0],
    'reference_points': [[-10, -10, 6],
                        [-10, 10, 6],
                        [10, 10, 6],
                        [-10, 10, 6],
                        [-10, -10, 6]],
    'reference_shift': 220,
    'legend': [r'm_{load} = 0.0 kg', r'm_{load} = 0.2 kg', r'm_{load} = 0.4 kg']
}

MPC_FIELD_TESTS_CONF = {
    'base_path': '/home/pete/PycharmProjects/AdaptiveDrone/logs/field_tests_official/mpc_tests/',
    'file_names': ['mpc0g.csv', 'mpc200g.csv', 'mpc400g.csv'],
    'cuts': [(0.5, 0.7), (0.55, 0.79), (0.555, 0.73)],
    'height_shifts': [0.0, 0.0, 0.0],
    'reference_points': [[-10, -10, 8],
                        [-10, 10, 8],
                        [10, 10, 8],
                        [-10, 10, 8],
                        [-10, -10, 8]],
    'reference_shift': 220,
    'legend': [r'm_{load} = 0.0 kg', r'm_{load} = 0.2 kg', r'm_{load} = 0.4 kg']
}


ADAPTIVE_FIELD_TESTS_CONF = {
    'base_path': '/home/pete/PycharmProjects/AdaptiveDrone/logs/field_tests_official/adaptive_tests/',
    'file_names': ['adaptive_0g.csv', 'adaptive_200g.csv', 'adaptive_400g.csv'],
    'cuts': [(0.19, 0.61), (0.33, 0.65), (0.072, 0.3)],
    'height_shifts': [0.0, 0.0, 0.0],
    'reference_points': [[-10, -10, 6],
                        [-10, 10, 6],
                        [10, 10, 6],
                        [-10, 10, 6],
                        [-10, -10, 6]],
    'reference_shift': 220,
    'legend': [r'm_{load} = 0.0 kg', r'm_{load} = 0.2 kg', r'm_{load} = 0.4 kg']
}

