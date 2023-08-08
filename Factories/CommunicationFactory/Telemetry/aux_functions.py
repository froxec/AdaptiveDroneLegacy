from QuadcopterIntegration.Utilities.dronekit_commands import get_state
from Factories.CommunicationFactory.Telemetry.mappings import FLIGHT_MODES_MAPPING
def update_telemetry(telemetry, vehicle):
   state = get_state(vehicle)
   telemetry['position_local'] = state[:3]
   telemetry['velocity'] = state[3:]
   telemetry['armed'] = vehicle.armed
   telemetry['attitude'] = [vehicle.attitude.pitch,
                            vehicle.attitude.roll,
                            vehicle.attitude.yaw]
   telemetry['heading'] = vehicle.heading
   mode_name = vehicle.mode.name
   telemetry['flight_mode'] = FLIGHT_MODES_MAPPING[mode_name]
   telemetry['bat_voltage'] = vehicle.battery.voltage
   telemetry['bat_current'] = vehicle.battery.current