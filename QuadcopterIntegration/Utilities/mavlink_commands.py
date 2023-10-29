from pymavlink import mavutil
from QuadcopterIntegration.Utilities.converters import euler_to_quaternion

def send_attitude(conn, roll, pitch, yaw, thrust):
    quaternion = euler_to_quaternion(roll, pitch, yaw)
    conn.mav.send(mavutil.mavlink.MAVLink_set_attitude_target_message(1, conn.target_system, conn.target_component, 7, quaternion, 0, 0, 0, thrust))
    #conn.mav.command_long_send(conn.target_system, conn.target_component, 82, 0, 10, conn.target_system, conn.target_component, 7, quaternion, 0, 0, 0, thrust)
    msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

def send_arm(conn):
    conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
def send_disarm(conn):
    conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    msg = conn.recv_match(blocking=True)
    print(msg)

def send_takeoff(conn, altitude):
    conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
    msg = conn.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

def check_if_armed(conn):
    conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 1, 0, 0, 0, 0, 0, 0, 0)
    msg = conn.recv_match(type='SYS_STATUS', blocking=True)
    pre_arm_check_code = 268435456
    print(msg.onboard_control_sensors_enabled)
    armed = pre_arm_check_code & msg.onboard_control_sensors_enabled
    return armed

def wait_until_armed(conn):
    heartbeat = conn.wait_heartbeat()
    print(heartbeat)
    