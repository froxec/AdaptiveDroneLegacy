import threading

from Factories.ControllersFactory.position_controllers.mpc import ModelPredictiveControl
from Factories.ToolsFactory.Converters import MPC_input_converter, MPC_output_converter
from Factories.CommunicationFactory.interfaces import ControllerInterface
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import Trajectory, SinglePoint
from Factories.ToolsFactory.GeneralTools import euclidean_distance
from typing import Type
import numpy as np
from threading import Thread
import plotly.graph_objects as go
from plotly.subplots import make_subplots

class PositionController():
    '''
    Wrapper for all position controller modules.
    '''
    def __init__(self,
                 controller: Type[ModelPredictiveControl],
                 input_converter: Type[MPC_input_converter],
                 output_converter: Type[MPC_output_converter], 
                 trajectory: Type[Trajectory], 
                 interface: Type[ControllerInterface]=None,
                 ramp_saturation=None) -> None:
        self.input_converter = input_converter
        self.controller = controller
        self.output_converter = output_converter
        self.trajectory = trajectory
        self.interface = interface
        self.ramp_saturation = ramp_saturation

        #trajectory tracking
        self.current_waypoint_id = 0
        self.setpoint = None
        self.history = {'u': []}

    def __call__(self, x=None, u_prev=None, convert_throttle=True):
        return self.get_control(x, u_prev,convert_throttle=convert_throttle)

    def get_control(self, x=None, u_prev=None, convert_throttle=True):
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
            self.u_prev = u_prev
        self.set_reference(self.trajectory, x)
        delta_x, delta_u = self.input_converter(self.x, self.u_prev)
        delta_u_next = self.controller.predict(delta_x, delta_u)
        if convert_throttle:
            u_next = self.output_converter(delta_u_next)
        else:
            u_next = self.output_converter(delta_u_next, throttle=False)
        if self.interface is not None:
            self.interface('send', u_next)
        if self.ramp_saturation is not None:
            u_next = self.ramp_saturation(u_next)
        self.history['u'].append(u_next)
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

    def plot_history(self, signal_name):
        if signal_name not in self.history.keys():
            raise ValueError("{} not tracked.. signal_name should be one of {}".format(signal_name,
                                                                                       self.history.keys()))
        fig = make_subplots(rows=3, cols=1, x_title='Czas [s]',
                            subplot_titles=('{}[0]'.format(signal_name), '{}[1]'.format(signal_name),
                                            '{}[2]'.format(signal_name)))
        data = np.array(self.history[signal_name])
        x = list(range(data.shape[0]))

        fig.add_trace(go.Scatter(x=x, y=data[:, 0]), row=1, col=1)
        fig.add_trace(go.Scatter(x=x, y=data[:, 1]), row=2, col=1)
        fig.add_trace(go.Scatter(x=x, y=data[:, 2]), row=3, col=1)
        fig.show()

class PositionControllerThread(PositionController, Thread):
    def __init__(self,
                 controller: Type[ModelPredictiveControl],
                 input_converter: Type[MPC_input_converter],
                 output_converter: Type[MPC_output_converter],
                 trajectory: Type[Trajectory],
                 ramp_saturation=None) -> None:
        PositionController.__init__(self,
                                    controller,
                                    input_converter,
                                    output_converter,
                                    trajectory,
                                    ramp_saturation=ramp_saturation)
        Thread.__init__(self)
        self.x = None
        self.u = None
        self.u_ref = None
        self.data_set = threading.Event()
        self.control_set = threading.Event()
        self.ready_event = threading.Event()
        self.ready_event.set()
        self._watchdog_active = threading.Event()
        self._watchdog = threading.Timer(1 / self.controller.freq, self._watchdog_activation)
        self.start()

        # telemetry
        self.telemetry_to_read = None

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
        delta_x, delta_u = self.input_converter(self.x, self.u)
        delta_u_next = self.controller.predict(delta_x, delta_u)
        u_next = self.output_converter(delta_u_next, throttle=False)
        if self.interface is not None:
            self.interface('send', u_next)
        if self.ramp_saturation is not None:
            u_next = self.ramp_saturation(u_next)
        self._set_telemetry(u_next)
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

    def _set_telemetry(self, u):
        self.telemetry_to_read = {'u': u}
        return self.telemetry_to_read