## mode to parameters mapping
MPC_PARAMETERS_MAPPING = {
    "LINEARIZED": {'Q_base': [0.5, 0.5, 0.5, 1, 1, 1], 'P_base': [1, 200, 200]},
    "LINEARIZED_NORMALIZED": {'Q_base': [5, 5, 1, 0.8, 0.8, 0.8], 'P_base': [0.01, 0.2, 0.2]},
    "TRANSLATIONAL_DYNAMICS": {'Q_base': [5, 5, 5, 1, 1, 1], 'P_base': [1, 1, 1]},
    "TRANSLATIONAL_DYNAMICS_NORMALIZED": {'Q_base': [1, 1, 1, 0.01, 0.01, 0.01], 'P_base':[1, 1, 0.01]}
}