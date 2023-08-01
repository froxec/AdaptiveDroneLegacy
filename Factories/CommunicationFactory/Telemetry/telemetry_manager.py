import threading
from typing import Type

from dronekit import Vehicle, VehicleMode
from pytelemetry import Pytelemetry
from pytelemetry.transports.serialtransport import SerialTransport

from Factories.CommunicationFactory.Telemetry.mappings import *
from Factories.CommunicationFactory.Telemetry.aux_functions import update_telemetry
from Factories.ControllersFactory.position_controllers.position_controller import PositionController
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import *
from threading import Thread
import time
class TelemetryManager:
    def __init__(self,
                 serialport: str,
                 baudrate: int,
                 vehicle: Type[Vehicle] = None,
                 subscribed_comms='ALL'):
        self.serialport = serialport
        self.baudrate = baudrate
        self.transport = SerialTransport()
        self.tlm = Pytelemetry(self.transport)
        self.transport.connect({'port': serialport, 'baudrate': self.baudrate})
        self.COM_ASCII_MAP = COMMANDS_ASCII_MAPPING
        self.subscriptions_mapping = SUBSCRIPTIONS_MAPPING
        self._subscription_mapping_isvalid(self.subscriptions_mapping)
        if subscribed_comms == 'ALL':
            self.subscribed_comms = self.COM_ASCII_MAP.nominal_keys
        else:
            self.subscribed_comms = subscribed_comms
        self.subscribe(self.subscribed_comms)
        self._init_telemetry()
        self.vehicle = vehicle
    def subscribe(self, comms):
        for comm in comms:
            comm_decoupled = comm.split(':')
            comm_stem = comm_decoupled[0]
            comm_asci = self.COM_ASCII_MAP[comm]
            for function in self.subscriptions_mapping[comm_stem]:
                method = getattr(self, function)
                self.tlm.subscribe(comm_asci, method)
        print("TELEM_MANAGER: Subscriptions attached.")

    def _subscription_mapping_isvalid(self, subscription_mapping):
        for key in subscription_mapping.keys():
            methods = subscription_mapping[key]
            for method_name in methods:
                if not hasattr(self, method_name):
                    raise ValueError("Subscription to method {} not valid. TelemetryManager doesn't support this method.".format(method_name))

    def printer_callback(self, topic, data, opts):
        print(self.COM_ASCII_MAP[topic] + " " + str(data))

    def update_telemetry_callback(self, topic, data, opts):
        comm = self.COM_ASCII_MAP[topic]
        indices = COMMANDS_TO_TELEMETRY_INDICES[comm]
        if len(indices) == 1:
            key = indices[0]
            self.telemetry[key] = data
        elif len(indices) == 2:
            key, id = indices
            self.telemetry[key][id] = data
        if hasattr(self, 'telemetry_set_event'):
            self.telemetry_set_event.set()

    def arm_disarm_callback(self, topic, data, opts):
        comm = self.COM_ASCII_MAP[topic]
        if self.vehicle is None:
            print("Cannot arm/disarm - no vehicle attached to manager")
            return
        if comm != 'ARM_DISARM':
            print("Command type {} not valid for arm/disarm.. Command should be 'ARM_DISARM'".format(comm))
            return
        if data==0:
            self.vehicle.disarm()
            print("TELEM_MANAGER: Disarming!")
        if data==1:
            self.vehicle.arm()
            print("TELEM_MANAGER: Arming!")

    def change_setpoint_callback(self, topic, data, opts):
        comm = COMMANDS_ASCII_MAPPING[topic]
        comm_decoupled = comm.split(":")
        if len(comm_decoupled) == 1:
            comm = comm_decoupled[0]
        elif len(comm_decoupled) == 2:
            comm, suffix = comm_decoupled
        if comm == 'SET_SPIRAL_SETPOINT':
            if not isinstance(self.position_controller.trajectory, SinglePoint):
                print("SET_SPIRAL_SETPOINT COMMAND REJECTED - Change trajectory type to single point first!")
                return
            setpoint = self.position_controller.trajectory.setpoint
            setpoint[SUFFIX_INDICES_MAPPING[suffix]] = data
            self.position_controller.change_setpoint(SinglePoint(setpoint))

    def auxiliary_command_callback(self, topic, data, opts):
        command = AUXILIARY_COMMANDS_MAPPING[data]
        if command == 'RETURN_TO_LAUNCH':
            self.vehicle.mode = VehicleMode('RTL')
        if command == 'LAND':
            self.vehicle.mode = VehicleMode('LAND')
        if command == 'TAKEOFF':
            target_attitude = 5
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True
            while not self.vehicle.armed:
                print(" Waiting for arming...")
                time.sleep(0.5)
            self.vehicle.simple_takeoff(target_attitude)
            while True:
                print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
                # Break and return from function just below target altitude.
                if self.vehicle.location.global_relative_frame.alt >= target_attitude * 0.95:
                    print("Reached target altitude")
                    break
                time.sleep(0.5)

    def publish(self, comm, value):
        if value is None:
            return
        comm_decoupled = comm.split(":")
        if len(comm_decoupled) > 1:
            comm_no_suffix, suffix = comm_decoupled
        elif len(comm_decoupled) == 1:
            comm_no_suffix = comm_decoupled[0]
        data_type = COMMANDS_DATATYPES_MAPPING[comm_no_suffix]
        comm_asci = self.COM_ASCII_MAP[comm]
        self.tlm.publish(comm_asci, value, data_type)
    def update(self):
        self.tlm.update()

    def disconnect(self):
        self.transport.disconnect()

    def _init_telemetry(self):
        self.telemetry = {}
        for indices in COMMANDS_TO_TELEMETRY_INDICES.values():
            if not isinstance(indices, tuple):
                key = indices
                self.telemetry[key] = None
            elif len(indices) == 2:
                key, id = indices
                if key not in self.telemetry:
                    self.telemetry[key] = []
                self.telemetry[key].append(None)
class TelemetryManagerThread(TelemetryManager, Thread):
    def __init__(self,
                 serialport: str,
                 baudrate: int,
                 update_freq: int,
                 vehicle: Type[Vehicle] = None,
                 subscribed_comms='ALL'):
        TelemetryManager.__init__(self,
                                  serialport=serialport,
                                  baudrate=baudrate,
                                  vehicle=vehicle,
                                  subscribed_comms=subscribed_comms)
        Thread.__init__(self)
        self.update_freq = update_freq
        self.start()

    def run(self):
        while True:
            self.update()
            time.sleep(1 / self.update_freq)

class TelemetryManagerThreadUAV(TelemetryManagerThread):
    def __init__(self,
                 serialport: str,
                 baudrate: int,
                 update_freq: int,
                 vehicle: Type[Vehicle],
                 position_controller: Type[PositionController],
                 subscribed_comms='ALL'):
        TelemetryManagerThread.__init__(self,
                                  serialport=serialport,
                                  baudrate=baudrate,
                                  vehicle=vehicle,
                                  update_freq=update_freq,
                                  subscribed_comms=subscribed_comms)
        self.position_controller = position_controller

    def publish_telemetry(self):
        update_telemetry(self.telemetry, self.vehicle)
        for command, indices in zip(COMMANDS_TO_TELEMETRY_INDICES.keys(), COMMANDS_TO_TELEMETRY_INDICES.values()):
            if not isinstance(indices, tuple):
                id = indices
                value = self.telemetry[id]
            else:
                key, id = indices
                value = self.telemetry[key][id]
            self.publish(command, value)

    def run(self):
        while True:
            self.publish_telemetry()
            self.update()
            time.sleep(1 / self.update_freq)

class TelemetryManagerThreadGCS(TelemetryManagerThread):
    def __init__(self,
                 serialport: str,
                 baudrate: int,
                 update_freq: int,
                 vehicle: Type[Vehicle] = None,
                 subscribed_comms='ALL'):
        TelemetryManagerThread.__init__(self,
                                  serialport=serialport,
                                  baudrate=baudrate,
                                  vehicle=vehicle,
                                  update_freq=update_freq,
                                  subscribed_comms=subscribed_comms)
        self.telemetry_set_event = threading.Event()
    def run(self):
        while True:
            self.update()
            time.sleep(1 / self.update_freq)