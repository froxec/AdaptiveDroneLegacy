from typing import Type

from dronekit import Vehicle
from pytelemetry import Pytelemetry
from pytelemetry.transports.serialtransport import SerialTransport

from Factories.CommunicationFactory.Telemetry.mappings import *
from Factories.ToolsFactory.GeneralTools import BidirectionalDict
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
        self.commands_to_telemetry_names = COMMANDS_TO_TELEMETRY_NAMES
        self.telemetry = {}
        for telemetry_name in self.commands_to_telemetry_names.values():
            self.telemetry[telemetry_name] = None
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
        telem_name = self.commands_to_telemetry_names[comm]
        self.telemetry[telem_name] = data

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

    def publish(self, comm, value):
        data_type = COMMANDS_DATATYPES_MAPPING[comm]
        comm_asci = self.COM_ASCII_MAP[comm]
        self.tlm.publish(comm_asci, value, data_type)
    def update(self):
        self.tlm.update()

    def disconnect(self):
        self.transport.disconnect()

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