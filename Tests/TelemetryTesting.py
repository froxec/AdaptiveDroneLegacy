from pytelemetry import Pytelemetry
from pytelemetry.transports.serialtransport import SerialTransport
from Factories.CommunicationFactory.Telemetry.mappings import COMMANDS_ASCII_MAPPING, SUBSCRIPTIONS_MAPPING,\
COMMANDS_TO_TELEMETRY_NAMES, COMMANDS_DATATYPES_MAPPING
import time
from typing import Type
from Factories.ToolsFactory.GeneralTools import BidirectionalDict
from dronekit import Vehicle, connect
import argparse
class TelemetryManager:
    def __init__(self,
                 serialport: str,
                 baudrate: int,
                 comm_ascii_map: Type[BidirectionalDict],
                 subscriptions_mapping: dict,
                 commands_to_telemetry_names: dict,
                 subscribed_comms='ALL',
                 vehicle: Type[Vehicle]=None):
        self.serialport = serialport
        self.baudrate = baudrate
        self.transport = SerialTransport()
        self.tlm = Pytelemetry(self.transport)
        self.transport.connect({'port': serialport, 'baudrate': self.baudrate})
        self.COM_ASCII_MAP = comm_ascii_map
        self.subsctiptions_mapping = subscriptions_mapping
        self._subscription_mapping_isvalid(subscriptions_mapping)
        if subscribed_comms == 'ALL':
            self.subscribed_comms = self.COM_ASCII_MAP.nominal_values
        else:
            self.subscribed_comms = subscribed_comms
        self.subscribe(self.subscribed_comms)
        self.commands_to_telemetry_names = commands_to_telemetry_names
        self.telemetry = {}
        for telemetry_name in self.commands_to_telemetry_names.values():
            self.telemetry[telemetry_name] = None
        self.vehicle = vehicle
    def subscribe(self, comms):
        for comm in comms:
            for function in self.subsctiptions_mapping[comm]:
                method = getattr(self, function)
                self.tlm.subscribe(comm, method)
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
        self.tlm.publish(comm, value, data_type)
    def update(self):
        self.tlm.update()

    def disconnect(self):
        self.transport.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='localhost:8000')
    args = parser.parse_args()
    vehicle = connect(args.connect, baud=921600, wait_ready=True, rate=100)
    tm = TelemetryManager(serialport='/dev/ttyUSB0',
                     baudrate=115200,
                     comm_ascii_map=COMMANDS_ASCII_MAPPING,
                     subscriptions_mapping=SUBSCRIPTIONS_MAPPING,
                     commands_to_telemetry_names=COMMANDS_TO_TELEMETRY_NAMES,
                     vehicle=vehicle)


    while True:
        tm.update()
        time.sleep(0.01)


