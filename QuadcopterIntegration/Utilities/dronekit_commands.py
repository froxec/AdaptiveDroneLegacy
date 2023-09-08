from QuadcopterIntegration.Utilities.converters import euler_to_quaternion
from dronekit import VehicleMode
from pymavlink import mavutil
import time
def initialize_drone(vehicle):
    vehicle.parameters['GUID_OPTIONS'] = int(8)
    vehicle.parameters['FENCE_ENABLE'] = 1
    vehicle.parameters['FENCE_ALT_MAX'] = 10
    vehicle.parameters['FENCE_ALT_MIN'] = 0
    vehicle.parameters['FENCE_RADIUS'] = 30
    vehicle.parameters['RTL_ALT'] = 5
    vehicle.parameters['WP_YAW_BEHAVIOR'] = 0 #never change yaw
    vehicle.parameters['ATC_RAT_YAW_I'] = 0.18
    vehicle.parameters['ATC_ANG_YAW_P'] = 6.5

    # commands below dont affect the behaviour..
    # message_rates = [
    #     (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 100),
    #     (mavutil.mavlink.MAV_DATA_STREAM_ALL, 50),
    # ]
    # messages = [vehicle.message_factory.command_long_encode(
    #     0, # Target system ID
    #     0, # Target component ID
    #     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    #     0, #Confirmation
    #     msg, # message ID
    #     10, # interval in microseconds
    #     0, 0, 0, 0, 0) for msg, rate in message_rates]
    # for message in messages:
    #     vehicle.send_mavlink(message)
    #     time.sleep(0.1)
    # vehicle.parameters['SR0_ADSB'] = 1
    # vehicle.parameters['SR0_EXT_STAT'] = 1
    # vehicle.parameters['SR0_EXTRA1'] = 1
    # vehicle.parameters['SR0_EXTRA2'] = 1
    # vehicle.parameters['SR0_EXTRA3'] = 1
    # vehicle.parameters['SR0_POSITION'] = 10
    # vehicle.parameters['SR0_RAW_CTRL'] = 1
    # vehicle.parameters['SR0_RAW_SENS'] = 1
    # vehicle.parameters['SR0_RC_CHAN'] = 1

def set_attitude(vehicle, roll, pitch, yaw, thrust):
    quaternion = euler_to_quaternion(roll, pitch, yaw)
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,
        0, 0,
        7,
        quaternion,
        0, 0, 0,
        thrust
    )
    # msg = mavutil.mavlink.MAVLink_set_attitude_target_message(1, 0, 0, 7, quaternion, 0, 0, 0, thrust)
    vehicle.send_mavlink(msg)

def set_position_local(vehicle, x, y, z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        1,
        3576,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
def get_state(vehicle):
    position = vehicle.location.local_frame
    velocity = vehicle.velocity
    if None not in velocity:
        velocity[2] = -velocity[2]
    state = [position.north, position.east, position.down]
    if None not in state:
        state[2] = -state[2]
    state.extend(velocity)
    return state

def get_attitude(vehicle):
    attitude = vehicle.attitude
    attitude = [attitude.roll, attitude.pitch, attitude.yaw]
    return attitude

def update_telemetry(telemetry, vehicle):
   state = get_state(vehicle)
   telemetry['position_local'] = state[:3]
   telemetry['velocity'] = state[3:]
   telemetry['armed'] = vehicle.armed
   telemetry['attitude'] = [vehicle.attitude.pitch, 
                            vehicle.attitude.roll,
                            vehicle.attitude.yaw]
   telemetry['position_global'] = [vehicle.location.global_frame.lat, vehicle.location.global_frame.lon] 
   telemetry['heading'] = vehicle.heading
   telemetry['flight_mode'] = vehicle.mode.name
   telemetry['bat_voltage'] = vehicle.battery.voltage
   telemetry['bat_current'] = vehicle.battery.current
  

class GCSCommInterpreter:
   
    def __init__(self, vehicle):
      self.vehicle = vehicle
      self.commands_map = {
          'arm': lambda: self.arm_disarm('arm'),
          'disarm': lambda: self.arm_disarm('disarm'),
          'change_mode': lambda name: self.change_flight_mode(name)
      }
    def __call__(self, command, value=None):
      func = self.commands_map[command]
      if value == None:
          return func()
      else:
          return func(value)
    def change_flight_mode(self, mode_name):
       print("Changing flight mode:", mode_name)
       self.vehicle.mode = VehicleMode(mode_name)
    def arm_disarm(self, mode):
        if mode == 'arm':
            print("Arming...")
            self.vehicle.arm()
        elif mode == 'disarm':
            print("Disarming...")
            self.vehicle.disarm()
# Function to arm and then takeoff to a user specified altitude
# def arm_and_takeoff(vehicle, aTargetAltitude):
#
#   print("Basic pre-arm checks")
#   # Don't let the user try to arm until autopilot is ready
#   while not vehicle.is_armable:
#     print(" Waiting for vehicle to initialise...")
#     time.sleep(1)
#
#   print("Arming motors")
#   # Copter should arm in GUIDED mode
#   vehicle.mode    = VehicleMode("GUIDED")
#   vehicle.armed   = True
#
#   while not vehicle.armed:
#     print(" Waiting for arming...")
#     time.sleep(1)
#
#   print("Taking off!")
#   vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
#
#   # Check that vehicle has reached takeoff altitude
#   while True:
#     print(" Altitude: ", vehicle.location.global_relative_frame.alt)
#     #Break and return from function just below target altitude.
#     if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
#       print("Reached target altitude")
#       break
#     time.sleep(1)