from QuadcopterIntegration.Utilities.converters import euler_to_quaternion
from pymavlink import mavutil

def set_attitude(vehicle, roll, pitch, yaw, thrust):
    quaternion = euler_to_quaternion(roll, pitch, yaw)
    msg = vehicle.send_mavlink(mavutil.mavlink.MAVLink_set_attitude_target_message(1, 0, 0, 7, quaternion, 0, 0, 0, thrust))
    
def get_state(vehicle):
    position = vehicle.location.local_frame
    velocity = vehicle.velocity
    state = [position.north, position.east, -position.down]
    state.extend(velocity)
    return state
