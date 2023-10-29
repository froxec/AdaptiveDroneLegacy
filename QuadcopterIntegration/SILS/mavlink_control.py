from pymavlink import mavutil
from QuadcopterIntegration.Utilities.mavlink_commands import *
conn = mavutil.mavlink_connection('udpin:localhost:8000')

conn.wait_heartbeat()
print("Heartbeat from system (system {}, component {})".format(conn.target_system, conn.target_component))

#arm drone
send_arm(conn)

while True:
    armed = check_if_armed(conn)
    print(armed)
    
#takeoff
send_takeoff(conn, 20)
#conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
send_attitude(conn, 0.0, 0.0, 0.0, 1)

