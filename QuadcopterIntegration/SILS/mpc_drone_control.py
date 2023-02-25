from dronekit import connect, VehicleMode, LocationGlobalRelative
from QuadcopterIntegration.Utilities.dronekit_commands import *
from pymavlink import mavutil
import time
from ModelsFactory.model_parameters import arducopter_parameters
from Simulation.MPC.configure_mpc_controler import MPC_configurator
import argparse  
import numpy as np

def mpc_command_convert(delta_u, u_ss, u_min, u_max):
    # mapping to <0, 1> range with 0.5 equal to equilibrium thrust
    thrust = delta_u[0]
    thrust_converted = thrust/(u_max[0] - u_min[0]) + 0.5
    u = delta_u + u_ss
    u = -u
    u[0] = thrust_converted
    return u


parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='192.168.0.27:8000')
args = parser.parse_args()

# Create MPC configurator
mpc = MPC_configurator(arducopter_parameters, np.array([0, 0, 20, 0, 0, 0]), np.array([1300, 500, 100, 0, 0, 0]))
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

arm_and_takeoff(vehicle, 20)

print("Take off complete")

u = mpc.uminus1
while True:
    x = np.array(get_state(vehicle))
    delta_u = mpc.update_state_control(x)
    u = mpc_command_convert(delta_u, mpc.quad.U_OP, mpc.umin, mpc.umax)
    print("State", x)
    print("Updating control:", u)
    u[3] = 0
    set_attitude(vehicle, u[1], u[2], u[3], u[0])
    time.sleep(2)