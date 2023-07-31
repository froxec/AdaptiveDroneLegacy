from Factories.ToolsFactory.GeneralTools import BidirectionalDict

COMMAND_NAMES = [
    'ARM_DISARM',
    'TELEMETRY_HEADING',
    'TELEMETRY_FLIGHT_MODE',
    'TELEMETRY_POSITION_LOC:X', 'TELEMETRY_POSITION_LOC:Y', 'TELEMETRY_POSITION_LOC:Z',
    'TELEMETRY_POSITION_GLOB:X', 'TELEMETRY_POSITION_GLOB:Y', 'TELEMETRY_POSITION_GLOB:Z',
    'TELEMETRY_VELOCITY:X', 'TELEMETRY_VELOCITY:Y', 'TELEMETRY_VELOCITY:Z',
    'TELEMETRY_ATTITUDE:X', 'TELEMETRY_ATTITUDE:Y' , 'TELEMETRY_ATTITUDE:Z'
]


# COMMANDS TO ASCII BIDIRECTIONAL MAPPING
COMMANDS_ASCII_MAPPING = BidirectionalDict()
id = 10
for comm_name in COMMAND_NAMES:
    COMMANDS_ASCII_MAPPING[comm_name] = chr(id)
    id += 1
del id

# COMMANDS to TelemetryManager methods mapping (subscriptions mapping)
SUBSCRIPTIONS_MAPPING = {
    'ARM_DISARM': ['arm_disarm_callback'],
    'TELEMETRY_POSITION_LOC': ['update_telemetry_callback', 'printer_callback'],
    'TELEMETRY_POSITION_GLOB': ['update_telemetry_callback', 'printer_callback'],
    'TELEMETRY_VELOCITY': ['update_telemetry_callback', 'printer_callback'],
    'TELEMETRY_ATTITUDE': ['update_telemetry_callback', 'printer_callback'],
    'TELEMETRY_HEADING': ['update_telemetry_callback', 'printer_callback'],
    'TELEMETRY_FLIGHT_MODE': ['update_telemetry_callback', 'printer_callback'],
}

# COMMANDS DATATYPES MAPPING
COMMANDS_DATATYPES_MAPPING = {
    'ARM_DISARM': 'int8',
    'TELEMETRY_POSITION_LOC': 'float32',
    'TELEMETRY_POSITION_GLOB': 'float32',
    'TELEMETRY_VELOCITY': 'float32',
    'TELEMETRY_ATTITUDE': 'float32',
    'TELEMETRY_HEADING': 'float32',
    'TELEMETRY_FLIGHT_MODE': 'int8',
}

# COMMANDS TO TELEMETRY NAMES MAPPING
COMMANDS_TO_TELEMETRY_NAMES = {
    'TELEMETRY_POSITION_LOC': 'position_local',
    'TELEMETRY_POSITION_GLOB': 'position_global',
    'TELEMETRY_VELOCITY': 'velocity',
    'TELEMETRY_ATTITUDE': 'attitude',
    'TELEMETRY_HEADING': 'heading',
    'TELEMETRY_FLIGHT_MODE': 'flight_mode'
}

FLIGHT_MODES_MAPPING = {
    '1': 'STABILIZE',
    '2': 'ACRO',
    '3': 'GUIDED'
}