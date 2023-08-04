from Factories.ControllersFactory.adaptive_augmentation.l1_augmentation import L1_AugmentationThread
from Factories.ControllersFactory.position_controllers.position_controller import PositionControllerThread
from QuadcopterIntegration.Utilities import dronekit_commands
from Factories.ModelsFactory.uncertain_models import LinearQuadUncertain
from dronekit import *
from typing import Type
import numpy as np
import time
class ControlSupervisor:
    def __init__(self,
                 vehicle: Type[Vehicle],
                 position_controller: Type[PositionControllerThread],
                 adaptive_controller: Type[L1_AugmentationThread]=None):
        self.position_controller = position_controller
        self.adaptive_controller = adaptive_controller
        self.vehicle = vehicle
        self.mpc_ref = None
        self.z_prev = np.zeros(3)
        self.u_prev = np.zeros(3)

    def __call__(self):
        return self.supervise()

    def supervise(self):
        u_composite = None
        x = self.get_state()
        #print("State", x)
        if self.position_controller.ready_event.is_set():
            self.position_controller.x = x
            self.position_controller.data_set.set()
            self.position_controller.ready_event.clear()
        if self.position_controller.control_set.is_set():
            self.mpc_ref = self.position_controller.u_ref
            self.position_controller.control_set.clear()
            #print("Got MPC reference {}".format(self.mpc_ref))
            u_composite = self.position_controller.output_converter.convert_throttle(self.mpc_ref)
        if self.adaptive_controller is not None and self.mpc_ref is not None and self.adaptive_controller.ready_event.is_set():
            z = x[3:6]
            if isinstance(self.adaptive_controller.predictor.ref_model, LinearQuadUncertain):
                _, delta_u = self.position_controller.input_converter(x, self.mpc_ref)
                u = delta_u
            else:
                u = self.mpc_ref
            self.adaptive_controller.set_data([z, self.z_prev, u, self.u_prev, None])
            self.adaptive_controller.ready_event.clear()
            self.z_prev = z
            self.u_prev = u
        if self.adaptive_controller is not None and self.adaptive_controller.control_set.is_set():
            u_composite = self.adaptive_controller.u_composite
            u_composite = self.position_controller.output_converter.convert_throttle(u_composite)
            self.adaptive_controller.control_set.clear()
        if u_composite is not None:
            u_composite_converted = self.command_convert(u_composite, 0,
                                                         2*self.position_controller.controller.model.parameters['m']*self.position_controller.controller.model.parameters['g'])
            #print("Control", u_composite_converted)
            self.set_attitude(u_composite_converted)

    def get_state(self):
        x = np.array(dronekit_commands.get_state(self.vehicle))
        return x

    def set_attitude(self, u):
        dronekit_commands.set_attitude(self.vehicle, u[1], u[2], 0, u[0])
    def command_convert(self, u, thrust_min, thrust_max):
        thrust = u[0]
        # for i, angle in enumerate(u[1:], 1):
        #     if angle > np.pi/4:
        #         u[i] = np.pi/4
        #     elif angle < -np.pi/4:
        #         u[i] = -np.pi/4
        u = -u
        if thrust > 1:
            thrust_converted = 1
        else:
            thrust_converted = thrust
        u[0] = thrust_converted
        return u

