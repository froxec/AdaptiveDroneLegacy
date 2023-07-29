import threading

from Factories.ControllersFactory.position_controllers.mpc import ModelPredictiveControl
from Factories.ToolsFactory.Converters import MPC_input_converter, MPC_output_converter
from Factories.CommunicationFactory.interfaces import ControllerInterface
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import Trajectory, SinglePoint
from typing import Type
import numpy as np
import time
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

    def __call__(self, x=None):
        return self.get_control(x)

    def get_control(self, x=None):
        if self.interface is not None:
            data = self.interface('recv')
            if data is not None:
                xnum = self.controller.model.x_num
                self.x = np.array(data[:xnum])
                self.xref = np.array(data[xnum:])
                print(self.xref)
                trajectory = SinglePoint(self.xref)
                self.change_setpoint(trajectory)
        else:
            self.x = x
        delta_x, _ = self.input_converter(self.x, None)
        delta_u_next = self.controller.predict(delta_x, self.trajectory)
        u_next = self.output_converter(delta_u_next)
        if self.interface is not None:
            self.interface('send', u_next)
        return u_next

    def change_setpoint(self, trajectory):
        self.trajectory = trajectory

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
        self.control_set = threading.Event()
        self.ready_event = threading.Event()
        self.Ts = 1 / self.controller.freq
        self.start()

    def run(self):
        start = time.monotonic()
        while True:
            if self.x is not None:
                x = self.x
            else:
                continue
            self.u_ref = self.get_control(x)
            self.control_set.set()
            self._control_execution(start)

    def get_control(self, x=None):
        if self.interface is not None:
            data = self.interface('recv')
            if data is not None:
                xnum = self.controller.model.x_num
                self.x = np.array(data[:xnum])
                self.xref = np.array(data[xnum:])
                print(self.xref)
                trajectory = SinglePoint(self.xref)
                self.change_setpoint(trajectory)
        else:
            self.x = x
        delta_x, _ = self.input_converter(self.x, None)
        delta_u_next = self.controller.predict(delta_x, self.trajectory)
        u_next = self.output_converter(delta_u_next, throttle=False)
        if self.interface is not None:
            self.interface('send', u_next)
        return u_next

    def _control_execution(self, start):
        time.sleep(self.Ts - ((time.monotonic() - start) % self.Ts))



    def set_data(self, data):
        self.data = data
        self.data_set.set()
