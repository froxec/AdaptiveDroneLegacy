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
    'base_path': '/home/pete/PycharmProjects/AdaptiveDrone/logs/16_10/',
    'file_names': ['10-16-2023-18:41:44_estim200g3RPI.csv'],
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