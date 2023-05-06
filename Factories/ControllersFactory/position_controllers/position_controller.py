from Factories.ControllersFactory.position_controllers.mpc import ModelPredictiveControl
from Factories.ToolsFactory.Converters import MPC_input_converter, MPC_output_converter
from Factories.CommunicationFactory.interfaces import ControllerInterface
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import Trajectory
from typing import Type
import numpy as np

class PositionController():
    '''
    Wrapper for all position controller modules.
    '''
    def __init__(self,
                 controller: Type[ModelPredictiveControl],
                 input_converter: Type[MPC_input_converter],
                 output_converter: Type[MPC_output_converter], 
                 trajectory: Type[Trajectory], 
                 interface: Type[ControllerInterface] == None) -> None:
        self.input_converter = input_converter
        self.controller = controller
        self.output_converter = output_converter
        self.trajectory = trajectory
        self.interface = interface

    def __call__(self, x=None):
        if self.interface is not None:
            x = self.interface('recv')
        if x is not None:
            self.x = np.array(x)
        delta_x, _ = self.input_converter(self.x, None)
        delta_u_next = self.controller.predict(delta_x, self.trajectory)
        u_next = self.output_converter(delta_u_next)
        if self.interface is not None:
            self.interface('send', u_next)
        return u_next

    def change_setpoint(self, trajectory):
        self.trajectory = trajectory
        