from pymavlink import mavutil
import dronekit
import argparse
import time


parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='localhost:8000')
args = parser.parse_args()
print('Connecting to vehicle on: %s' % args.connect)
vehicle = dronekit.connect(args.connect, baud=921600, wait_ready=True, rate=100)
print(vehicle)
@vehicle.on_message('GLOBAL_POSITION_INT_COV')
def my_method(self, name, msg):
        print(name)
        print(msg)
while True:
        time.sleep(10)


# # Start a connection listening on a UDP port
# connection = mavutil.mavlink_connection('udpin:localhost:8500')
#
# # Wait for the first heartbeat to set the system and component ID of remote system for the link
# connection.wait_heartbeat()
# print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
#
# # Define command_long_encode message to send MAV_CMD_SET_MESSAGE_INTERVAL command
# # param1: MAVLINK_MSG_ID_BATTERY_STATUS (message to stream)
# # param2: 1000000 (Stream interval in microseconds)
# message = connection.mav.command_long_encode(
#         connection.target_system,  # Target system ID
#         connection.target_component,  # Target component ID
#         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
#         0,  # Confirmation
#         mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,  # param1: Message ID to be streamed
#         1000000, # param2: Interval in microseconds
#         0,       # param3 (unused)
#         0,       # param4 (unused)
#         0,       # param5 (unused)
#         0,       # param5 (unused)
#         0        # param6 (unused)
#         )
#
# connection.mav.send(message)
#
# # Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
# response = connection.recv_match(type='COMMAND_ACK', blocking=True)
# if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
#     print("Command accepted")
# else:
#     print("Command failed")