from QuadcopterIntegration.Utilities.converters import euler_to_quaternion
from pymavlink import mavutil

def set_attitude(vehicle, roll, pitch, yaw, thrust):
    quaternion = euler_to_quaternion(roll, pitch, yaw)
    msg = vehicle.send_mavlink(mavutil.mavlink.MAVLink_set_attitude_target_message(1, 0, 0, 7, quaternion, 0, 0, 0, thrust))
    
