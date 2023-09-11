import threading
from typing import Type

from dronekit import Vehicle, VehicleMode
from pytelemetry import Pytelemetry
from pytelemetry.transports.serialtransport import SerialTransport
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
from Factories.CommunicationFactory.Telemetry.mappings import *
from Factories.CommunicationFactory.Telemetry.aux_functions import update_telemetry
from Factories.ControllersFactory.position_controllers.position_controller import PositionController
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import *
from Factories.ControllersFactory.adaptive_augmentation.l1_augmentation import L1_Augmentation
from Factories.ControllersFactory.control_tools.ControlSupervisor import ControlSupervisor
from threading import Thread
import warnings
import time
class TelemetryManager:
    def __init__(self,
                 serialport: str,
                 baudrate: int,
                 vehicle: Type[Vehicle] = None,
                 subscribed_comms='ALL',
                 lora_address = None,
                 lora_freq = None,
                 remote_lora_address=None,
                 remote_lora_freq=None):
        if serialport is not None:
            self.serialport = serialport
            self.baudrate = baudrate
            self.transport = SerialTransport()
            self.tlm = Pytelemetry(self.transport, lora_address, lora_freq)
            self.transport.connect({'port': serialport, 'baudrate': self.baudrate})
        self.lora_info = {'lora_address': lora_address,
                          'lora_freq': lora_freq,
                          'remote_lora_address': remote_lora_address,
                          'remote_lora_freq': remote_lora_freq}
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
        if isinstance(indices, tuple):
            key, id = indices
            self.telemetry[key][id] = data
        else:
            key = indices
            self.telemetry[key] = data
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
        print("Change setpoint_callback", topic, data)
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
            self.position_controller.change_trajectory(SinglePoint(setpoint))

    def auxiliary_command_callback(self, topic, data, opts):
        command = AUXILIARY_COMMANDS_MAPPING[data]
        if command == 'RETURN_TO_LAUNCH':
            self.vehicle.mode = VehicleMode('RTL')
        if command == 'LAND':
            self.vehicle.mode = VehicleMode('LAND')
        if command == 'TAKEOFF':
            if self.vehicle.armed == False or self.vehicle.mode != VehicleMode("GUIDED"):
                print("TAKEOFF_COMM: COMMAND REJECTED: PLEASE CHECK VEHICLE IS ARMED AND IN GUIDED MODE!")
                return
            target_attitude = 3
            self.vehicle.simple_takeoff(target_attitude)
            while True:
                print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
                # Break and return from function just below target altitude.
                if self.vehicle.location.global_relative_frame.alt >= target_attitude * 0.95:
                    print("Reached target altitude")
                    break
                time.sleep(0.1)

    def identification_procedure_callback(self, topic, data, opts):
        raise NotImplementedError
    def update_controllers_callback(self, topic, data, opts):
        raise NotImplementedError

    def data_write_callback(self, topic, data, opts):
        raise NotImplementedError

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
        self.tlm.publish(comm_asci, value, data_type, self.lora_info['remote_lora_address'], self.lora_info['remote_lora_freq'])
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
                 subscribed_comms='ALL',
                 lora_address=None,
                 lora_freq=None,
                 remote_lora_address=None,
                 remote_lora_freq=None):
        TelemetryManager.__init__(self,
                                  serialport=serialport,
                                  baudrate=baudrate,
                                  vehicle=vehicle,
                                  subscribed_comms=subscribed_comms,
                                  lora_address=lora_address,
                                  lora_freq=lora_freq,
                                  remote_lora_address=remote_lora_address,
                                  remote_lora_freq=remote_lora_freq)
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
                 control_supervisor: Type[ControlSupervisor],
                 adaptive_augmentation: Type[L1_Augmentation]=None,
                 data_writer=None,
                 subscribed_comms='ALL',
                 additional_telemetry=['reference', 'estimation_and_ref', 'output_and_throttle'],
                 send_telemetry=True,
                 lora_address=None,
                 lora_freq=None,
                 remote_lora_address=None,
                 remote_lora_freq=None):

        self.position_controller = position_controller
        self.control_supervisor = control_supervisor
        self.adaptive_augmentation = adaptive_augmentation
        self.data_writer = data_writer
        self.additional_telemetry = additional_telemetry
        self.send_telemetry = send_telemetry
        if 'estimation_and_ref' in additional_telemetry and self.adaptive_augmentation is None:
            warnings.warn("'Estimation and control' telemetry requested, but it requires adaptive controller, which"
                          "is None. Requested telemetry won't be added to a stream.")
            self.additional_telemetry.remove('estimation_and_ref')
        if ('reference' in additional_telemetry and 'estimation_and_control' in additional_telemetry):
            warnings.warn("'Control' telemetry requested, but 'estimation and control' already satisfies the needs."
                          " Requested telemetry won't be added to a stream.")
            self.additional_telemetry.remove('reference')

        TelemetryManagerThread.__init__(self,
                                  serialport=serialport,
                                  baudrate=baudrate,
                                  vehicle=vehicle,
                                  update_freq=update_freq,
                                  subscribed_comms=subscribed_comms,
                                  lora_address=lora_address,
                                  lora_freq=lora_freq,
                                  remote_lora_address=remote_lora_address,
                                  remote_lora_freq=remote_lora_freq)

    def publish_telemetry(self):
        update_telemetry(self.telemetry, self.vehicle)
        if hasattr(self, 'additional_telemetry') and 'estimation_and_ref' in self.additional_telemetry:
            self._add_estimation_to_telemetry(self.telemetry)
        if hasattr(self, 'additional_telemetry') and 'reference' in self.additional_telemetry:
            self._add_control_to_telemetry(self.telemetry)
        if hasattr(self, 'additional_telemetry') and 'output_and_throttle' in self.additional_telemetry:
            self._add_output_to_telemetry(self.telemetry)
        if self.data_writer is not None:
            self.telemetry['telem_writing_ok'] = self.data_writer.writing_ok
        available_telemetry = self.telemetry.keys()
        for command, indices in zip(COMMANDS_TO_TELEMETRY_INDICES.keys(), COMMANDS_TO_TELEMETRY_INDICES.values()):
            if not isinstance(indices, tuple):
                key = indices
                if key not in available_telemetry:
                    continue
                value = self.telemetry[key]
            else:
                key, id = indices
                if key not in available_telemetry:
                    continue
                value = self.telemetry[key][id]
            self.publish(command, value)

    def _add_estimation_to_telemetry(self, telemetry):
        estimation_telemetry = self.adaptive_augmentation.telemetry_to_read
        if estimation_telemetry is not None:
            for key in estimation_telemetry.keys():
                telemetry[key] = estimation_telemetry[key]
        self.telemetry = telemetry
        return self.telemetry

    def _add_control_to_telemetry(self, telemetry):
        control_telemetry = self.position_controller.telemetry_to_read
        if control_telemetry is not None:
            for key in control_telemetry.keys():
                telemetry[key] = control_telemetry[key]
        self.telemetry = telemetry
        return self.telemetry

    def _add_output_to_telemetry(self, telemetry):
        output_and_throttle = self.control_supervisor.telemetry_to_read
        if output_and_throttle is not None:
            for key in output_and_throttle.keys():
                telemetry[key] = output_and_throttle[key]
        self.telemetry = telemetry
        return self.telemetry


    def auxiliary_command_callback(self, topic, data, opts):
        super().auxiliary_command_callback(topic, data, opts)
        command = AUXILIARY_COMMANDS_MAPPING[data]
        if command == 'START_ESTIMATION_PROCEDURE':
            print("TELEM_MANAGER: Turning on estimation procedure..")
            self.control_supervisor.estimation_on = True

    def update_controllers_callback(self, topic, data, opts):
        comm = COMMANDS_ASCII_MAPPING[topic]
        comm_split = comm.split("_")
        if comm_split[0] == 'POSITION':
            if data == 1:
                print("TELEM_MANAGER: POSITION_CONTROLLER ON")
                self.control_supervisor.position_controller_on = True
            elif data == 0:
                print("TELEM_MANAGER: POSITION_CONTROLLER OFF")
                self.control_supervisor.position_controller_on = False
        if comm_split[0] == 'ADAPTIVE':
            if data == 1:
                print("TELEM_MANAGER: ADAPTIVE_CONTROLLER ON")
                self.control_supervisor.adaptive_controller_on = True
            elif data == 0:
                print("TELEM_MANAGER: ADAPTIVE_CONTROLLER OFF")
                self.control_supervisor.adaptive_controller_on = False
        if comm_split[0] == 'ESTIMATOR':
            if data == 1:
                print("TELEM_MANAGER: ESTIMATOR ON")
                self.control_supervisor.estimation_on = True
            elif data == 0:
                print("TELEM_MANAGER: ESTIMATOR OFF")
                self.control_supervisor.estimation_on = False

    def data_write_callback(self, topic, data, opts):
        if self.data_writer is None:
            raise "TELEM_MANAGER: write request, data_writer is None"
        commands = data.split("_")
        if len(commands) == 1 and commands[0] == "R":
            self.data_writer.writing_event.clear()
            print("TELEM_MANAGER: REQUESTED TO STOP WRITING DATA")
        elif commands[0] == "S":
            filename = commands[1]
            self.data_writer.filename = filename
            self.data_writer.writing_event.set()
            print("TELEM_MANAGER: REQUESTED WRITING DATA TO FILE {}".format(filename))
    def run(self):
        while True:
            if self.send_telemetry:
                self.publish_telemetry()
            self.update()
            time.sleep(1 / self.update_freq)

class TelemetryManagerThreadGCS(TelemetryManagerThread):
    def __init__(self,
                 serialport: str,
                 baudrate: int,
                 update_freq: int,
                 vehicle: Type[Vehicle] = None,
                 subscribed_comms='ALL',
                 lora_address=None,
                 lora_freq=None,
                 remote_lora_address=None,
                 remote_lora_freq=None):
        TelemetryManagerThread.__init__(self,
                                  serialport=serialport,
                                  baudrate=baudrate,
                                  vehicle=vehicle,
                                  update_freq=update_freq,
                                  subscribed_comms=subscribed_comms,
                                  lora_address=lora_address,
                                  lora_freq=lora_freq,
                                  remote_lora_address=remote_lora_address,
                                  remote_lora_freq=remote_lora_freq)
        self.telemetry_set_event = threading.Event()
    def run(self):
        while True:
            self.update()
            time.sleep(1 / self.update_freq)

class TelemetryManagerUAV(TelemetryManager):
    def __init__(self,
                 serialport: str,
                 baudrate: int,
                 vehicle,
                 db_interface,
                 data_writer=None,
                 subscribed_comms='ALL',
                 additional_telemetry=['reference', 'estimation_and_ref', 'output_and_throttle'],
                 send_telemetry=True,
                 lora_address=None,
                 lora_freq=None,
                 remote_lora_address=None,
                 remote_lora_freq=None):
        TelemetryManager.__init__(self,
                                  serialport=serialport,
                                  baudrate=baudrate,
                                  vehicle=vehicle,
                                  subscribed_comms=subscribed_comms,
                                  lora_address=lora_address,
                                  lora_freq=lora_freq,
                                  remote_lora_address=remote_lora_address,
                                  remote_lora_freq=remote_lora_freq)

        self.data_writer = data_writer
        self.additional_telemetry = additional_telemetry
        self.send_telemetry = send_telemetry
        self.db_interface = db_interface

    def update_controllers_callback(self, topic, data, opts):
        comm = COMMANDS_ASCII_MAPPING[topic]
        comm_split = comm.split("_")

        # update flags
        if comm_split[0] == 'POSITION':
            if data == 1:
                print("TELEM_MANAGER: POSITION_CONTROLLER ON")
                self.db_interface.telemetry_manager_state['mpc_running'] = True
            elif data == 0:
                print("TELEM_MANAGER: POSITION_CONTROLLER OFF")
                self.db_interface.telemetry_manager_state['mpc_running'] = False
        if comm_split[0] == 'ADAPTIVE':
            if data == 1:
                print("TELEM_MANAGER: ADAPTIVE_CONTROLLER ON")
                self.db_interface.telemetry_manager_state['adaptive_running'] = True
            elif data == 0:
                print("TELEM_MANAGER: ADAPTIVE_CONTROLLER OFF")
                self.db_interface.telemetry_manager_state['adaptive_running'] = False
        if comm_split[0] == 'ESTIMATOR':
            if data == 1:
                print("TELEM_MANAGER: ESTIMATOR ON")
                self.db_interface.telemetry_manager_state['estimator_running'] = True
            elif data == 0:
                print("TELEM_MANAGER: ESTIMATOR OFF")
                self.db_interface.telemetry_manager_state['estimator_running'] = False

        # update db
        self.db_interface.update_telemetry_manager_db()

    def publish_telemetry(self):
        update_telemetry(self.telemetry, self.vehicle)
        if hasattr(self, 'additional_telemetry') and 'estimation_and_ref' in self.additional_telemetry:
            self._add_estimation_to_telemetry(self.telemetry)
        if self.data_writer is not None:
            self.telemetry['telem_writing_ok'] = self.data_writer.writing_ok
        self._add_mass_estimation_to_telemetry(self.telemetry)
        available_telemetry = self.telemetry.keys()
        for command, indices in zip(COMMANDS_TO_TELEMETRY_INDICES.keys(), COMMANDS_TO_TELEMETRY_INDICES.values()):
            if not isinstance(indices, tuple):
                key = indices
                if key not in available_telemetry:
                    continue
                value = self.telemetry[key]
            else:
                key, id = indices
                if key not in available_telemetry:
                    continue
                value = self.telemetry[key][id]
            self.publish(command, value)

    def change_setpoint_callback(self, topic, data, opts):
        comm = COMMANDS_ASCII_MAPPING[topic]
        comm_decoupled = comm.split(":")
        if len(comm_decoupled) == 1:
            comm = comm_decoupled[0]
        elif len(comm_decoupled) == 2:
            comm, suffix = comm_decoupled
            current_setpoint = self.db_interface.telemetry_manager_state['setpoint']
            current_setpoint[SUFFIX_INDICES_MAPPING[suffix]] = data
            self.db_interface.telemetry_manager_state['setpoint'] = current_setpoint
        self.db_interface.update_telemetry_manager_db()
        #update db
        if None not in self.db_interface.telemetry_manager_state['setpoint']:
            self.db_interface.publish_setpoint(self.db_interface.telemetry_manager_state['setpoint'])
            self.db_interface.telemetry_manager_state['setpoint'] = [None, None, None]

    def data_write_callback(self, topic, data, opts):
        if self.data_writer is None:
            raise ValueError("TELEM_MANAGER: write request, data_writer is None")
        commands = data.split("_")
        if len(commands) == 1 and commands[0] == "R":
            self.data_writer.writing_event.clear()
            print("TELEM_MANAGER: REQUESTED TO STOP WRITING DATA")
        elif commands[0] == "S":
            filename = commands[1]
            self.data_writer.filename = filename
            self.data_writer.writing_event.set()
            print("TELEM_MANAGER: REQUESTED WRITING DATA TO FILE {}".format(filename))

    def _add_estimation_to_telemetry(self, telemetry):
        sigma_hat = self.db_interface.adaptive_interface_state['sigma_hat']
        u_l1 = self.db_interface.adaptive_interface_state['u_l1']
        ref = self.db_interface.adaptive_interface_state['ref']
        u_output = self.db_interface.adaptive_interface_state['u_output']
        if sigma_hat is not None:
            telemetry['sigma_hat'] = sigma_hat
        if u_l1 is not None:
            telemetry['u_l1'] = u_l1
        if ref is not None:
            telemetry['u'] = ref
        if u_output is not None:
            telemetry['u_output'] = u_output
        self.telemetry = telemetry
        return self.telemetry

    def _add_mass_estimation_to_telemetry(self, telemetry):
        mass = self.db_interface.get_estimated_mass()
        telemetry['estimated_mass'] = mass
        self.telemetry = telemetry
        return self.telemetry

    def auxiliary_command_callback(self, topic, data, opts):
        super().auxiliary_command_callback(topic, data, opts)
        command = AUXILIARY_COMMANDS_MAPPING[data]
        if command == 'ACCEPT_ESTIMATION':
            print("TELEMETRY MANAGER: ESTIMATION ACCEPTED -> PUBLISHING PARAMETERS")
            self.db_interface.publish_parameters()

    def identification_procedure_callback(self, topic, data, opts):
        throttle = data
        if data >= 0.0:
            self.db_interface.start_identification_procedure(throttle)
        else:
            self.db_interface.stop_identification_procedure()



    def run(self):
        if self.send_telemetry:
            self.publish_telemetry()
        self.update()


class TelemetryManagerUAVMultiprocessingThread(TelemetryManagerUAV, Thread):
    def __init__(self,
                 serialport: str,
                 baudrate: int,
                 update_freq,
                 vehicle,
                 db_interface,
                 data_writer=None,
                 subscribed_comms='ALL',
                 additional_telemetry=['reference', 'estimation_and_ref', 'output_and_throttle'],
                 send_telemetry=True,
                 lora_address=None,
                 lora_freq=None,
                 remote_lora_address=None,
                 remote_lora_freq=None):
        TelemetryManagerUAV.__init__(self,
                                  serialport=serialport,
                                  baudrate=baudrate,
                                  vehicle=vehicle,
                                  db_interface=db_interface,
                                  data_writer=data_writer,
                                  subscribed_comms=subscribed_comms,
                                  additional_telemetry=additional_telemetry,
                                  send_telemetry=send_telemetry,
                                  lora_address=lora_address,
                                  lora_freq=lora_freq,
                                  remote_lora_address=remote_lora_address,
                                  remote_lora_freq=remote_lora_freq)
        Thread.__init__(self)
        self.update_freq = update_freq
        self.start()

    def run(self):
        while True:
            if self.send_telemetry:
                self.publish_telemetry()
            time.sleep(1 / self.update_freq)

class MQTT_TelemetryManager(TelemetryManagerUAVMultiprocessingThread):
    def __init__(self,
                 mqtt_host,
                 mqtt_port,
                 vehicle,
                 update_freq,
                 db_interface=None,
                 data_writer=None,
                 send_telemetry=False,
                 subscribed_comms='ALL',
                 additional_telemetry=['reference', 'estimation_and_ref', 'output_and_throttle']):
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(mqtt_host, mqtt_port, keepalive=0)
        TelemetryManagerUAVMultiprocessingThread.__init__(self,
                                                          serialport=None,
                                                          baudrate=None,
                                                          update_freq=update_freq,
                                                          vehicle=vehicle,
                                                          db_interface=db_interface,
                                                          data_writer=data_writer,
                                                          subscribed_comms=subscribed_comms,
                                                          additional_telemetry=additional_telemetry,
                                                          send_telemetry=send_telemetry)
    def run(self):
        while True:
            if self.send_telemetry:
                self.publish_telemetry()
            self.mqtt_client.loop()
            time.sleep(1 / self.update_freq)

    def subscribe(self, comms):
        for comm in comms:
            comm_decoupled = comm.split(':')
            comm_stem = comm_decoupled[0]
            comm_asci = self.COM_ASCII_MAP[comm]
            for function in self.subscriptions_mapping[comm_stem]:
                method = getattr(self, function)
                self.mqtt_client.subscribe(comm_asci)
                self.mqtt_client.message_callback_add(comm_asci, callback=method)
        print("TELEM_MANAGER: Subscriptions attached.")
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
        self.mqtt_client.publish(comm_asci, str(value))

    def printer_callback(self, client, userdata, message):
        topic = message.topic
        payload = message.payload
        print("Topic {} Payload {}", topic, payload)

    def update_telemetry_callback(self, client, userdata, message):
        topic = message.topic
        payload = message.payload
        comm = self.COM_ASCII_MAP[topic]
        data = mqtt_map_datatype(payload, comm)
        indices = COMMANDS_TO_TELEMETRY_INDICES[comm]
        if isinstance(indices, tuple):
            key, id = indices
            self.telemetry[key][id] = data
        else:
            key = indices
            self.telemetry[key] = data
        if hasattr(self, 'telemetry_set_event'):
            self.telemetry_set_event.set()

    def arm_disarm_callback(self, client, userdata, message):
        topic = message.topic
        payload = message.payload
        comm = self.COM_ASCII_MAP[topic]
        data = mqtt_map_datatype(payload, comm)
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

    def change_setpoint_callback(self, client, userdata, message):
        topic = message.topic
        payload = message.payload
        comm = self.COM_ASCII_MAP[topic]
        data = mqtt_map_datatype(payload, comm)
        comm_decoupled = comm.split(":")
        if len(comm_decoupled) == 1:
            comm = comm_decoupled[0]
        elif len(comm_decoupled) == 2:
            comm, suffix = comm_decoupled
            current_setpoint = self.db_interface.telemetry_manager_state['setpoint']
            current_setpoint[SUFFIX_INDICES_MAPPING[suffix]] = data
            self.db_interface.telemetry_manager_state['setpoint'] = current_setpoint
        self.db_interface.update_telemetry_manager_db()
        # update db
        if None not in self.db_interface.telemetry_manager_state['setpoint']:
            self.db_interface.publish_setpoint(self.db_interface.telemetry_manager_state['setpoint'])
            self.db_interface.telemetry_manager_state['setpoint'] = [None, None, None]

    def auxiliary_command_callback(self, client, userdata, message):
        topic = message.topic
        payload = message.payload
        comm = self.COM_ASCII_MAP[topic]
        data = mqtt_map_datatype(payload, comm)
        command = AUXILIARY_COMMANDS_MAPPING[data]
        if command == 'RETURN_TO_LAUNCH':
            self.vehicle.mode = VehicleMode('RTL')
        if command == 'LAND':
            self.vehicle.mode = VehicleMode('LAND')
        if command == 'TAKEOFF':
            if self.vehicle.armed == False or self.vehicle.mode != VehicleMode("GUIDED"):
                print("TAKEOFF_COMM: COMMAND REJECTED: PLEASE CHECK VEHICLE IS ARMED AND IN GUIDED MODE!")
                return
            target_attitude = 3
            self.vehicle.simple_takeoff(target_attitude)
            while True:
                print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
                # Break and return from function just below target altitude.
                if self.vehicle.location.global_relative_frame.alt >= target_attitude * 0.95:
                    print("Reached target altitude")
                    break
                time.sleep(0.1)

    def update_controllers_callback(self, client, userdata, message):
        topic = message.topic
        payload = message.payload
        comm = self.COM_ASCII_MAP[topic]
        data = mqtt_map_datatype(payload, comm)
        comm_split = comm.split("_")

        # update flags
        if comm_split[0] == 'POSITION':
            if data == 1:
                print("TELEM_MANAGER: POSITION_CONTROLLER ON")
                self.db_interface.telemetry_manager_state['mpc_running'] = True
            elif data == 0:
                print("TELEM_MANAGER: POSITION_CONTROLLER OFF")
                self.db_interface.telemetry_manager_state['mpc_running'] = False
        if comm_split[0] == 'ADAPTIVE':
            if data == 1:
                print("TELEM_MANAGER: ADAPTIVE_CONTROLLER ON")
                self.db_interface.telemetry_manager_state['adaptive_running'] = True
            elif data == 0:
                print("TELEM_MANAGER: ADAPTIVE_CONTROLLER OFF")
                self.db_interface.telemetry_manager_state['adaptive_running'] = False
        if comm_split[0] == 'ESTIMATOR':
            if data == 1:
                print("TELEM_MANAGER: ESTIMATOR ON")
                self.db_interface.telemetry_manager_state['estimator_running'] = True
            elif data == 0:
                print("TELEM_MANAGER: ESTIMATOR OFF")
                self.db_interface.telemetry_manager_state['estimator_running'] = False

        # update db
        self.db_interface.update_telemetry_manager_db()

    def data_write_callback(self, client, userdata, message):
        topic = message.topic
        payload = message.payload
        comm = self.COM_ASCII_MAP[topic]
        data = mqtt_map_datatype(payload, comm)
        if self.data_writer is None:
            raise "TELEM_MANAGER: write request, data_writer is None"
        print(type(data))
        commands = data.split("_")
        print(commands[0])
        if len(commands) == 1 and commands[0] == "R":
            self.data_writer.writing_event.clear()
            print("TELEM_MANAGER: REQUESTED TO STOP WRITING DATA")
        elif commands[0] == "S":
            filename = commands[1]
            self.data_writer.filename = filename
            self.data_writer.writing_event.set()
            print("TELEM_MANAGER: REQUESTED WRITING DATA TO FILE {}".format(filename))

    def identification_procedure_callback(self, client, userdata, message):
        topic = message.topic
        payload = message.payload
        comm = self.COM_ASCII_MAP[topic]
        data = mqtt_map_datatype(payload, comm)
        throttle = data
        if data >= 0.0:
            self.db_interface.start_identification_procedure(throttle)
        else:
            self.db_interface.stop_identification_procedure()

class MQTT_TelemetryManagerGCS(MQTT_TelemetryManager):
    def __init__(self,
                 mqtt_host,
                 mqtt_port,
                 update_freq,
                 subscribed_comms):
        MQTT_TelemetryManager.__init__(self,mqtt_host=mqtt_host,
                                            mqtt_port=mqtt_port,
                                            vehicle=None,
                                            update_freq=update_freq,
                                            subscribed_comms=subscribed_comms)
        self.telemetry_set_event = threading.Event()
    def run(self):
        while True:
            self.mqtt_client.loop()
            time.sleep(1 / self.update_freq)

if __name__ == "__main__":
    from dronekit import connect
    host = "192.168.0.27"
    port = 8955
    print("Connecting to drone {}".format("udp:localhost:8000"))
    vehicle = connect("udp:localhost:8000", baud=921600, wait_ready=True, rate=100)
    print("Connection established!")
    tm = MQTT_TelemetryManager(host, port, vehicle)
    while True:
        tm.publish_telemetry()
        time.sleep(0.01)