global telemetry
telemetry = {
    'position_local': None,
    'position_global': None,
    'velocity': None,
    'armed': None,
    'attitude': None,
    'heading': None,
    'flight_mode': None,
}

commands = {
    'ARM': 'arm\n'.encode(),
    'DISARM': 'disarm\n'.encode(),
    'STABILIZE': 'mode:STABILIZE\n'.encode(),
    'GUIDED': 'mode:GUIDED\n'.encode(),
    'ACRO': 'mode:ACRO\n'.encode()

}