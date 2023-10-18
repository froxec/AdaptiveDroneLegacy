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
    'file_names': ['estim0g.csv', 'estim200g.csv', 'estim400g2.csv'],
    'cuts': [(0.42, 0.7), (0.45, 0.62), (0.0, 1)],
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