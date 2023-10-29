from Factories.ControllersFactory.adaptive_augmentation.l1_augmentation import L1_AugmentationThread
from Factories.ControllersFactory.position_controllers.position_controller import PositionControllerThread
from QuadcopterIntegration.Utilities import dronekit_commands
from Factories.ModelsFactory.uncertain_models import LinearQuadUncertain, QuadTranslationalDynamicsUncertain
from Factories.RLFactory.Agents.BanditEstimatorAgent import BanditEstimatorThread
from Factories.ToolsFactory.Converters import RampSaturation
from Factories.ToolsFactory.GeneralTools import LowPassLiveFilter
from dronekit import *
from typing import Type
import numpy as np
import time
class ControlSupervisor:
    def __init__(self,
                 vehicle: Type[Vehicle],
                 position_controller: Type[PositionControllerThread],
                 adaptive_controller: Type[L1_AugmentationThread]=None,
                 estimator_agent: Type[BanditEstimatorThread]=None,
                 min_altitude=2.5):
        self.position_controller = position_controller
        self.adaptive_controller = adaptive_controller
        self.estimator_agent = estimator_agent
        self.vehicle = vehicle
        if self.adaptive_controller is not None:
            self.Ts = self.adaptive_controller.predictor.Ts
        else:
            self.Ts = 1/self.position_controller.controller.freq
        self.mpc_ref = None
        self.z_prev = np.zeros(3)
        self.u_prev = np.zeros(3)
        self.mpc_ref_prev = self.position_controller.input_converter.u_ss
        self.telemetry_to_read = None
        self.min_altitude = min_altitude
        self.position_controller_on = False
        self.adaptive_controller_on = False
        self.estimation_on = False
        self.velocity_filter = LowPassLiveFilter([15, 15, 15], fs=100, signals_num=3)

    def __call__(self):
        return self.supervise()

    def supervise(self):
        #print("State", x)
        if not self._check_for_min_altitude():
            self.position_controller_on = False
        if self.position_controller_on and self.vehicle.mode.name == VehicleMode("GUIDED").name:
            self.run_controllers()
        else:
            if self.vehicle.mode.name != VehicleMode("RTL").name:
                print("CONTROL_SUPERVISOR: CONTROLLER TURNED OFF: MODE RTL")
                self.vehicle.mode = VehicleMode("RTL")

    def get_state(self):
        x = np.array(dronekit_commands.get_state(self.vehicle))
        return x

    def run_controllers(self):
        u_composite = None
        x = self.get_state()
        x[3:6] = self.velocity_filter(x[3:6])
        if self.position_controller.ready_event.is_set():
            self.position_controller.x = x
            self.position_controller.u = self.mpc_ref_prev
            self.position_controller.data_set.set()
            self.position_controller.ready_event.clear()
        if self.position_controller.control_set.is_set():
            self.mpc_ref = self.position_controller.u_ref # [F, phi, theta]
            self.position_controller.control_set.clear()
            #print("Got MPC reference {}".format(self.mpc_ref))
            u_composite = self.mpc_ref
            self.mpc_ref_prev = self.mpc_ref
        if (self.adaptive_controller is not None
                and self.adaptive_controller_on
                and self.mpc_ref is not None
                and self.adaptive_controller.ready_event.is_set()):
            if isinstance(self.adaptive_controller.predictor.ref_model, LinearQuadUncertain):
                delta_x, delta_u = self.position_controller.input_converter(x, self.mpc_ref)
                u = delta_u
                z = delta_x[3:6]
            else:
                z = x[3:6]
                u = self.mpc_ref
            self.adaptive_controller.set_data([z, self.z_prev, u, self.u_prev, None])
            self.adaptive_controller.ready_event.clear()
            self.z_prev = z
            self.u_prev = u
        if self.adaptive_controller is not None and self.adaptive_controller.control_set.is_set():
            u_composite = self.adaptive_controller.u_composite
            if isinstance(self.adaptive_controller.predictor.ref_model, LinearQuadUncertain):
                u_composite = self.position_controller.output_converter(u_composite, throttle=False)
            self.adaptive_controller.control_set.clear()
        if u_composite is not None:
            u_composite_throttle_converted = self.position_controller.output_converter.convert_throttle(u_composite)
            u_composite_converted = self.command_convert(u_composite_throttle_converted)
            #print("Control", u_composite_converted)
            self.set_attitude(u_composite_converted)
            self._set_telemetry(u_composite, u_composite_converted[0])
            if (self.estimator_agent is not None
                    and self.estimation_on):
                if self.estimator_agent.procedure_finished.is_set():
                    self.estimation_on = False
                    self.estimator_agent.procedure_finished.clear()
                    print("CONTROL_SUPERVISOR: Estimation procedure turned off..")
                measurement = x[3:6]
                force = u_composite[0]
                angles = np.concatenate([u_composite[1:], np.array([0.0])])
                self.estimator_agent.data = {'measurement': measurement, 'force': force, 'angles': angles}
                self.estimator_agent.data_set_event.set()
        #time.sleep(self.Ts)
    def set_attitude(self, u):
        dronekit_commands.set_attitude(self.vehicle, u[1], u[2], 0, u[0])

    def _check_for_min_altitude(self):
        alt = self.get_state()[2]
        if alt < self.min_altitude * 0.9:
            print("Current alt {}. Supervisor waiting to reach minimum altitude...".format(self.vehicle.location.global_relative_frame.alt))
            return False
        else:
            return True
    def command_convert(self, u):
        thrust = u[0]
        # for i, angle in enumerate(u[1:], 1):
        #     if angle > np.pi/4:
        #         u[i] = np.pi/4
        #     elif angle < -np.pi/4:
        #         u[i] = -np.pi/4
        u = -u
        if thrust > 1:
            thrust_converted = 1
        elif thrust < 0:
            thrust_converted = 0
        else:
            thrust_converted = thrust
        u[0] = thrust_converted
        return u

    def _set_telemetry(self, u_output, throttle):
        self.telemetry_to_read = {'u_output': u_output,
                                  'throttle': throttle}
        return self.telemetry_to_read

