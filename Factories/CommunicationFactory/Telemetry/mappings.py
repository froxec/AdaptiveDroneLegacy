from Factories.ToolsFactory.GeneralTools import BidirectionalDict

COMMAND_NAMES = [
    'ARM_DISARM',
    'SET_SPIRAL_SETPOINT:X', 'SET_SPIRAL_SETPOINT:Y', 'SET_SPIRAL_SETPOINT:Z',
    'AUXILIARY_COMMAND',
    'HEARTBEAT',
    'DATA_WRITE',
    'POSITION_CONTROLLER_ON_OFF',
    'ADAPTIVE_CONTROLLER_ON_OFF',
    'ESTIMATOR_ON_OFF',
    'TELEMETRY_ESTIMATED_MASS',
    'TELEMETRY_HEADING',
    'TELEMETRY_FLIGHT_MODE',
    'TELEMETRY_THROTTLE_REF',
    'TELEMETRY_BATTERY_VOLTAGE',
    'TELEMETRY_BATTERY_CURRENT',
    'TELEMETRY_WRITING_OK',
    'TELEMETRY_MPC_RUNNING',
    'TELEMETRY_ADAPTIVE_RUNNING',
    'TELEMETRY_POSITION_LOC:X', 'TELEMETRY_POSITION_LOC:Y', 'TELEMETRY_POSITION_LOC:Z',
    'TELEMETRY_POSITION_GLOB:X', 'TELEMETRY_POSITION_GLOB:Y', 'TELEMETRY_POSITION_GLOB:Z',
    'TELEMETRY_VELOCITY:X', 'TELEMETRY_VELOCITY:Y', 'TELEMETRY_VELOCITY:Z',
    'TELEMETRY_ATTITUDE:X', 'TELEMETRY_ATTITUDE:Y' , 'TELEMETRY_ATTITUDE:Z',
    'TELEMETRY_CONTROL:X', 'TELEMETRY_CONTROL:Y', 'TELEMETRY_CONTROL:Z',
    'TELEMETRY_ADAPTIVE_CONTROL:X', 'TELEMETRY_ADAPTIVE_CONTROL:Y', 'TELEMETRY_ADAPTIVE_CONTROL:Z',
    'TELEMETRY_UNC_ESTIMATION:X', 'TELEMETRY_UNC_ESTIMATION:Y', 'TELEMETRY_UNC_ESTIMATION:Z',
    'TELEMETRY_CONTROL_OUTPUT:X', 'TELEMETRY_CONTROL_OUTPUT:Y', 'TELEMETRY_CONTROL_OUTPUT:Z'
]

# COMMANDS TO ASCII BIDIRECTIONAL MAPPING
COMMANDS_ASCII_MAPPING = BidirectionalDict()
id = 10
for comm_name in COMMAND_NAMES:
    COMMANDS_ASCII_MAPPING[comm_name] = "0,868," + chr(id)
    id += 1
del id

# COMMANDS to TelemetryManager methods mapping (subscriptions mapping)
SUBSCRIPTIONS_MAPPING = {
    'ARM_DISARM': ['arm_disarm_callback'],
    'AUXILIARY_COMMAND': ['auxiliary_command_callback'],
    'SET_SPIRAL_SETPOINT': ['change_setpoint_callback'],
    'HEARTBEAT': ['printer_callback'],
    'DATA_WRITE': ['data_write_callback'],
    'POSITION_CONTROLLER_ON_OFF': ['update_controllers_callback'],
    'ADAPTIVE_CONTROLLER_ON_OFF': ['update_controllers_callback'],
    'ESTIMATOR_ON_OFF': ['update_controllers_callback'],
    'TELEMETRY_ESTIMATED_MASS': ['update_telemetry_callback'],
    'TELEMETRY_POSITION_LOC': ['update_telemetry_callback'],
    'TELEMETRY_POSITION_GLOB': ['update_telemetry_callback'],
    'TELEMETRY_VELOCITY': ['update_telemetry_callback'],
    'TELEMETRY_ATTITUDE': ['update_telemetry_callback'],
    'TELEMETRY_CONTROL': ['update_telemetry_callback'],
    'TELEMETRY_ADAPTIVE_CONTROL': ['update_telemetry_callback'],
    'TELEMETRY_UNC_ESTIMATION': ['update_telemetry_callback'],
    'TELEMETRY_CONTROL_OUTPUT': ['update_telemetry_callback'],
    'TELEMETRY_HEADING': ['update_telemetry_callback'],
    'TELEMETRY_FLIGHT_MODE': ['update_telemetry_callback'],
    'TELEMETRY_THROTTLE_REF': ['update_telemetry_callback'],
    'TELEMETRY_BATTERY_VOLTAGE': ['update_telemetry_callback'],
    'TELEMETRY_BATTERY_CURRENT': ['update_telemetry_callback'],
    'TELEMETRY_WRITING_OK': ['update_telemetry_callback'],
    'TELEMETRY_MPC_RUNNING': ['update_telemetry_callback'],
    'TELEMETRY_ADAPTIVE_RUNNING': ['update_telemetry_callback']
}

# COMMANDS DATATYPES MAPPING
COMMANDS_DATATYPES_MAPPING = {
    'ARM_DISARM': 'int8',
    'AUXILIARY_COMMAND': 'int8',
    'SET_SPIRAL_SETPOINT': 'float32',
    'HEARTBEAT': 'int8',
    'DATA_WRITE': 'string',
    'POSITION_CONTROLLER_ON_OFF': 'int8',
    'ADAPTIVE_CONTROLLER_ON_OFF': 'int8',
    'ESTIMATOR_ON_OFF': 'int8',
    'TELEMETRY_ESTIMATED_MASS': 'float32',
    'TELEMETRY_POSITION_LOC': 'float32',
    'TELEMETRY_POSITION_GLOB': 'float32',
    'TELEMETRY_VELOCITY': 'float32',
    'TELEMETRY_ATTITUDE': 'float32',
    'TELEMETRY_CONTROL': 'float32',
    'TELEMETRY_ADAPTIVE_CONTROL': 'float32',
    'TELEMETRY_UNC_ESTIMATION': 'float32',
    'TELEMETRY_CONTROL_OUTPUT': 'float32',
    'TELEMETRY_HEADING': 'float32',
    'TELEMETRY_FLIGHT_MODE': 'int8',
    'TELEMETRY_THROTTLE_REF': 'float32',
    'TELEMETRY_BATTERY_VOLTAGE': 'float32',
    'TELEMETRY_BATTERY_CURRENT': 'float32',
    'TELEMETRY_WRITING_OK': 'int8',
    'TELEMETRY_MPC_RUNNING': 'int8',
    'TELEMETRY_ADAPTIVE_RUNNING': 'int8',
}

# COMMANDS TO TELEMETRY NAMES MAPPING
COMMANDS_TO_TELEMETRY_INDICES = {
    'TELEMETRY_POSITION_LOC:X': ('position_local', 0),
    'TELEMETRY_POSITION_LOC:Y': ('position_local', 1),
    'TELEMETRY_POSITION_LOC:Z': ('position_local', 2),
    'TELEMETRY_POSITION_GLOB:X': ('position_global', 0),
    'TELEMETRY_POSITION_GLOB:Y': ('position_global', 1),
    'TELEMETRY_POSITION_GLOB:Z': ('position_global', 2),
    'TELEMETRY_VELOCITY:X': ('velocity', 0),
    'TELEMETRY_VELOCITY:Y': ('velocity', 1),
    'TELEMETRY_VELOCITY:Z': ('velocity', 2),
    'TELEMETRY_ATTITUDE:X': ('attitude', 0),
    'TELEMETRY_ATTITUDE:Y': ('attitude', 1),
    'TELEMETRY_ATTITUDE:Z': ('attitude', 2),
    'TELEMETRY_CONTROL:X': ('u', 0),
    'TELEMETRY_CONTROL:Y': ('u', 1),
    'TELEMETRY_CONTROL:Z': ('u', 2),
    'TELEMETRY_ADAPTIVE_CONTROL:X': ('u_l1', 0),
    'TELEMETRY_ADAPTIVE_CONTROL:Y': ('u_l1', 1),
    'TELEMETRY_ADAPTIVE_CONTROL:Z': ('u_l1', 2),
    'TELEMETRY_UNC_ESTIMATION:X': ('sigma_hat', 0),
    'TELEMETRY_UNC_ESTIMATION:Y': ('sigma_hat', 1),
    'TELEMETRY_UNC_ESTIMATION:Z': ('sigma_hat', 2),
    'TELEMETRY_CONTROL_OUTPUT:X': ('u_output', 0),
    'TELEMETRY_CONTROL_OUTPUT:Y': ('u_output', 1),
    'TELEMETRY_CONTROL_OUTPUT:Z': ('u_output', 2),
    'TELEMETRY_ESTIMATED_MASS': 'estimated_mass',
    'TELEMETRY_HEADING': 'heading',
    'TELEMETRY_FLIGHT_MODE': 'flight_mode',
    'TELEMETRY_THROTTLE_REF': 'throttle',
    'TELEMETRY_BATTERY_VOLTAGE': 'bat_voltage',
    'TELEMETRY_BATTERY_CURRENT': 'bat_current',
    'TELEMETRY_WRITING_OK': 'telem_writing_ok',
    'TELEMETRY_MPC_RUNNING': 'telem_mpc_running',
    'TELEMETRY_ADAPTIVE_RUNNING': 'telem_adaptive_running'
}

# MAPPING FLIGHT_MODES TO VALUES INT8
FLIGHT_MODES_MAPPING = BidirectionalDict()
MODES = ['STABILIZE', 'ACRO', 'GUIDED', 'RTL', 'LAND', 'POSHOLD', 'LOITER', 'ALT_HOLD']
for i, mode in enumerate(MODES):
    FLIGHT_MODES_MAPPING[mode] = i

# TRAJECTORY TYPE MAPPING
TRAJECTORY_TYPE_MAPPING = BidirectionalDict()
TRAJECTORY_TYPE_MAPPING['SINGLE_POINT_TRAJECTORY'] = 1
TRAJECTORY_TYPE_MAPPING['SPIRAL_TRAJECTORY'] = 2
TRAJECTORY_TYPE_MAPPING['EIGHT_TRAJECTORY'] = 3

# SUFFIX_INDICES_MAPPING
SUFFIX_INDICES_MAPPING = BidirectionalDict()
SUFFIX_INDICES_MAPPING['X'] = 0
SUFFIX_INDICES_MAPPING['Y'] = 1
SUFFIX_INDICES_MAPPING['Z'] = 2

# AUXILIARY COMMANDS MAPPING
AUXILIARY_COMMANDS_MAPPING = BidirectionalDict()
AUXILIARY_COMMANDS = ['RETURN_TO_LAUNCH', 'LAND', 'TAKEOFF', 'ACCEPT_ESTIMATION',
                      'START_IDENTIFICATION_PROCEDURE', 'STOP_IDENTIFICATION_PROCEDURE']
for i, mode in enumerate(AUXILIARY_COMMANDS):
    AUXILIARY_COMMANDS_MAPPING[mode] = i