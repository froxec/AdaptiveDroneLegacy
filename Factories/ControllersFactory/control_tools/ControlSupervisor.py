from Factories.ControllersFactory.adaptive_augmentation.l1_augmentation import L1_AugmentationThread
from Factories.ControllersFactory.position_controllers.position_controller import PositionControllerThread
from QuadcopterIntegration.Utilities import dronekit_commands
from dronekit import *
from typing import Type
import numpy as np
import time
class ControlSupervisor:
    def __init__(self,
                 position_controller: Type[PositionControllerThread],
                 adaptive_controller: Type[L1_AugmentationThread],
                 vehicle: Type[Vehicle]):
        self.position_controller = position_controller
        self.adaptive_controller = adaptive_controller
        self.vehicle = vehicle
        self.mpc_ref = None
        self.z_prev = np.zeros(3)
        self.u_prev = np.zeros(3)

    def __call__(self):
        return self.supervise()

    def supervise(self):
        x = self.get_state()
        self.position_controller.x = x
        if self.position_controller.control_set.is_set():
            self.mpc_ref = self.position_controller.u_ref
            self.position_controller.control_set.clear()
            print("Got MPC output {}".format(self.mpc_ref))
        if self.mpc_ref is not None:
            z = x[3:6]
            self.adaptive_controller.set_data([z, self.z_prev, self.mpc_ref, self.u_prev, None])
            self.z_prev = z
            self.u_prev = self.mpc_ref
        if self.adaptive_controller.control_set.is_set():
            u_composite = self.adaptive_controller.u_composite
            print("Got adaptive output {}".format(u_composite))
            u_composite = self.position_controller.output_converter.convert_throttle(u_composite)
            u_composite_converted = self.command_convert(u_composite, 0,
                                                         2*self.position_controller.controller.model.parameters['m']*self.position_controller.controller.model.parameters['g'])
            self.adaptive_controller.control_set.clear()
            self.set_attitude(u_composite_converted)

    def get_state(self):
        x = np.array(dronekit_commands.get_state(self.vehicle))
        return x

    def set_attitude(self, u):
        dronekit_commands.set_attitude(self.vehicle, u[1], u[2], 0, u[0])
    def command_convert(self, u, thrust_min, thrust_max):
        thrust = u[0]
        u = -u
        if thrust > 1:
            thrust_converted = 1
        else:
            thrust_converted = thrust
        u[0] = thrust_converted
        return u

