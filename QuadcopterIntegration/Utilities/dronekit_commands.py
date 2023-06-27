from QuadcopterIntegration.Utilities.converters import euler_to_quaternion
from dronekit import VehicleMode
from pymavlink import mavutil
import time

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
    state = [position.north, position.east, -position.down]
    state.extend(velocity)
    return state

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(vehicle, aTargetAltitude):

  print("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

  print("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print("Reached target altitude")
      break
    time.sleep(1)