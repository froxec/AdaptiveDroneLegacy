import threading

from Factories.ControllersFactory.position_controllers.mpc import ModelPredictiveControl
from Factories.ToolsFactory.Converters import MPC_input_converter, MPC_output_converter
from Factories.CommunicationFactory.interfaces import ControllerInterface
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import Trajectory, SinglePoint
from Factories.ToolsFactory.GeneralTools import euclidean_distance
from typing import Type
import numpy as np
from threading import Thread

class PositionController():
    '''
    Wrapper for all position controller modules.
    '''
    def __init__(self,
                 controller: Type[ModelPredictiveControl],
                 input_converter: Type[MPC_input_converter],
                 output_converter: Type[MPC_output_converter], 
                 trajectory: Type[Trajectory], 
                 interface: Type[ControllerInterface]=None) -> None:
        self.input_converter = input_converter
        self.controller = controller
        self.output_converter = output_converter
        self.trajectory = trajectory
        self.interface = interface

        #trajectory tracking
        self.current_waypoint_id = 0
        self.setpoint = None

    def __call__(self, x=None, convert_throttle=True):
        return self.get_control(x, convert_throttle=convert_throttle)

    def get_control(self, x=None, convert_throttle=True):
        if self.interface is not None:
            data = self.interface('recv')
            if data is not None:
                xnum = self.controller.model.x_num
                self.x = np.array(data[:xnum])
                self.xref = np.array(data[xnum:])
                print(self.xref)
                trajectory = SinglePoint(self.xref)
                self.change_trajectory(trajectory)
        else:
            self.x = x
        self.set_reference(self.trajectory, x)
        delta_x, _ = self.input_converter(self.x, None)
        delta_u_next = self.controller.predict(delta_x)
        if convert_throttle:
            u_next = self.output_converter(delta_u_next)
        else:
            u_next = self.output_converter(delta_u_next, throttle=False)
        if self.interface is not None:
            self.interface('send', u_next)
        return u_next

    def change_trajectory(self, trajectory):
        self.trajectory = trajectory

    def set_reference(self, ref, x):
        if self.setpoint is not None and self.check_if_reached_waypoint(x) and self.current_waypoint_id < len(
                self.trajectory) - 1:
            self.current_waypoint_id += 1
        if isinstance(ref, Trajectory):
            self.trajectory = ref
            self.setpoint = self.trajectory.generated_trajectory[self.current_waypoint_id]
        else:
            self.setpoint = ref
        if isinstance(self.setpoint, np.ndarray):
            self.setpoint = self.setpoint.reshape(-1, 1)
        else:
            self.setpoint = np.array(self.setpoint).reshape(-1, 1)
        if self.setpoint.shape[0] == 3:
            self.setpoint = np.concatenate([self.setpoint, np.zeros(3).reshape((-1, 1))], axis=0)

        #update_other_objects
        self.controller.change_setpoint(self.setpoint)
        self.input_converter.update(x_ss=self.setpoint.flatten())

    def check_if_reached_waypoint(self, x):
        if self.setpoint.shape != x.shape:
            distance = euclidean_distance(self.setpoint.flatten(), x)
        else:
            euclidean_distance(self.setpoint, x)
        if distance < 0.1:
            return True
        else:
            return False

class PositionControllerThread(PositionController, Thread):
    def __init__(self,
                 controller: Type[ModelPredictiveControl],
                 input_converter: Type[MPC_input_converter],
                 output_converter: Type[MPC_output_converter],
                 trajectory: Type[Trajectory]) -> None:
        PositionController.__init__(self, controller,
                                         input_converter,
                                         output_converter,
                                         trajectory)
        Thread.__init__(self)
        self.x = None
        self.u_ref = None
        self.data_set = threading.Event()
        self.control_set = threading.Event()
        self.ready_event = threading.Event()
        self.ready_event.set()
        self._watchdog_active = threading.Event()
        self._watchdog = threading.Timer(1 / self.controller.freq, self._watchdog_activation)
        self.start()
    def run(self):
        while True:
            self._restart_watchdog()
            x = self._get_data()
            self.u_ref = self.get_control(x)
            self.control_set.set()
            self._control_execution()

    def get_control(self, x=None):
        if self.interface is not None:
            data = self.interface('recv')
            if data is not None:
                xnum = self.controller.model.x_num
                self.x = np.array(data[:xnum])
                self.xref = np.array(data[xnum:])
                print(self.xref)
                trajectory = SinglePoint(self.xref)
                self.change_trajectory(trajectory)
        else:
            self.x = x
        self.set_reference(self.trajectory, x)
        delta_x, _ = self.input_converter(self.x, None)
        delta_u_next = self.controller.predict(delta_x)
        u_next = self.output_converter(delta_u_next, throttle=False)
        if self.interface is not None:
            self.interface('send', u_next)
        return u_next

    def _control_execution(self):
        self._watchdog_active.wait()
        self.ready_event.set()
        self._watchdog_active.clear()
    def _watchdog_activation(self):
        self._watchdog_active.set()

    def _restart_watchdog(self):
        self._watchdog.cancel()
        self._watchdog = threading.Timer(1 / self.controller.freq, self._watchdog_activation)
        self._watchdog.start()

    def set_data(self, data):
        self.data = data
        self.data_set.set()

    def _get_data(self):
        self.data_set.wait()
        x = self.x
        self.data_set.clear()
        return x