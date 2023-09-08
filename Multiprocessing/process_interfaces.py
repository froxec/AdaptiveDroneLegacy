import json
import redis
from Multiprocessing.PARAMS import REDIS_HOST, REDIS_PORT, DB_NUM
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SinglePoint
from QuadcopterIntegration.Utilities import dronekit_commands
import numpy as np
from copy import deepcopy
import pickle

drone_proxy_definition = {
    'x': None,
    'armed': False,
    'vehicle_mode': None
}

mpc_proxy_defintion = {
    'ref': None,
    'current_setpoint': [None, None, None],
}

adaptive_proxy_definition = {
    'u': None,
    'u_l1': None,
    'sigma_hat': None,
    'ref': None,
    'u_output': None
}

telemetry_manager_proxy_definition = {
    'adaptive_running': False,
    'mpc_running': False,
    'estimator_running': False,
    'identification_procedure_running': False,
    'identification_procedure_throttle': None,
    'setpoint':  [None, None, None],
}

estimator_proxy_definition = {
    'parameters': None
}

class Interface:
    def __init__(self):
        self.mpc_interface_state = deepcopy(mpc_proxy_defintion)
        self.drone_state = deepcopy(drone_proxy_definition)
        self.telemetry_manager_state = deepcopy(telemetry_manager_proxy_definition)
        self.adaptive_interface_state = adaptive_proxy_definition
        self.estimator_interface_state = estimator_proxy_definition
        self.redis_database = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=DB_NUM)
        self.pubsub = None

    def update_db(self):
        raise NotImplementedError

    def fetch_db(self):
        # get mpc state
        mpc_state_packed = self.redis_database.get("mpc_state")
        if mpc_state_packed is not None:
            self.mpc_interface_state = json.loads(mpc_state_packed.decode("utf-8"))
        # get drone state
        drone_state_packed = self.redis_database.get("drone_state")
        if drone_state_packed is not None:
            self.drone_state = json.loads(drone_state_packed.decode("utf-8"))
        # get telemetry manager state
        telemetry_manager_state_packed = self.redis_database.get("telemetry_manager_state")
        if telemetry_manager_state_packed is not None:
            self.telemetry_manager_state = json.loads(telemetry_manager_state_packed.decode("utf-8"))
        # get adaptive state
        adaptive_state_packed = self.redis_database.get("adaptive_state")
        if adaptive_state_packed is not None:
            self.adaptive_interface_state = json.loads(adaptive_state_packed.decode("utf-8"))
        # get estimator state
        estimator_state_packed = self.redis_database.get("estimator_state")
        if estimator_state_packed is not None:
            self.estimator_interface_state = json.loads(estimator_state_packed.decode("utf-8"))
        if self.pubsub is not None:
            self.pubsub.get_message()

    def get_drone_state(self):
        x = self.drone_state['x']
        return np.array(x)

    def is_mpc_running(self):
        return self.telemetry_manager_state['mpc_running']

    def is_adaptive_running(self):
        return self.telemetry_manager_state['adaptive_running']

    def is_estimator_running(self):
        return self.telemetry_manager_state['estimator_running']

    def is_vehicle_armed(self):
        return self.drone_state['armed']

    def get_vehicle_mode(self):
        return self.drone_state['vehicle_mode']



class MPC_Interface(Interface):
    def __init__(self, position_controller):
        Interface.__init__(self)
        self.pubsub = self.redis_database.pubsub()
        self.pubsub.subscribe(**{"setpoint_change": self.setpoint_change_callback,
                                 'parameters_change': self.parameters_change_callback})

        # get access to position controller for setpoint callback
        self.position_controller = position_controller
    def update_db(self):
        self.redis_database.set("mpc_state", json.dumps(self.mpc_interface_state))


    def get_previous_control(self):
        u_prev = self.mpc_interface_state['ref']
        return np.array(u_prev)

    def set_control(self, u):
        u = list(u)
        self.mpc_interface_state['ref'] = u

    def update_setpoint(self, setpoint):
        if not None in setpoint:
            setpoint = list(setpoint.flatten())
            self.mpc_interface_state['current_setpoint'] = setpoint
    def reset_state(self):
        self.mpc_interface_state = deepcopy(mpc_proxy_defintion)

    def setpoint_change_callback(self, message):
        data = message['data']
        setpoint_dict = json.loads(data.decode("utf-8"))
        setpoint = setpoint_dict['setpoint']
        if None not in setpoint:
            print("MPC interface: setpoint change", setpoint)
            self.position_controller.change_trajectory(SinglePoint(setpoint))
            self.mpc_interface_state['current_setpoint'] = setpoint

    def parameters_change_callback(self, message):
        data = message['data']
        parameters_dict = json.loads(data.decode("utf-8"))
        parameters = parameters_dict['parameters']
        if None not in parameters:
            print("MPC interface: parameters change", parameters)
            parameters_holder = self.position_controller.input_converter.parameters_holder
            for key in parameters.keys():
                parameters_holder.__setattr__(key, parameters[key])
            self.position_controller.controller.update_parameters()
            self.position_controller.output_converter.update()
            self.position_controller.input_converter.update(update_u_ss=True)

class Adaptive_Interface(Interface):
    def __init__(self,
                 input_converter,
                 output_converter,
                 prediction_model,
                 adaptive_controller):
        Interface.__init__(self)
        self.pubsub = self.redis_database.pubsub()
        self.pubsub.subscribe(**{'setpoint_change': self.setpoint_change_callback,
                                 'parameters_change': self.parameters_change_callback})
        #get converters representations
        self.input_converter = input_converter
        self.output_converter = output_converter
        self.prediction_model = prediction_model
        self.adaptive_controller = adaptive_controller

    def update_db(self):
        self.redis_database.set("adaptive_state", json.dumps(self.adaptive_interface_state))

    def set_control(self, u):
        u = list(u)
        self.adaptive_interface_state['u'] = u

    def get_ref(self):
        ref = np.array(self.mpc_interface_state['ref'])
        return ref

    def get_current_setpoint(self):
        current_setpoint = np.array(self.mpc_interface_state['current_setpoint'])
        return np.array(current_setpoint)

    def set_sigma_hat(self, sigma_hat):
        sigma_hat = list(sigma_hat)
        self.adaptive_interface_state['sigma_hat'] = sigma_hat

    def set_u_l1(self, u_l1):
        u_l1 = list(u_l1)
        self.adaptive_interface_state['u_l1'] = u_l1

    def set_ref(self, ref):
        ref = list(ref)
        self.adaptive_interface_state['ref'] = ref

    def set_u_output(self, u_output):
        u_output = list(u_output)
        self.adaptive_interface_state['u_output'] = u_output
    def reset_state(self):
        self.adaptive_interface_state = deepcopy(adaptive_proxy_definition)

    def setpoint_change_callback(self, message):
        data = message['data']
        setpoint_dict = json.loads(data.decode("utf-8"))
        setpoint = setpoint_dict['setpoint']
        if None not in setpoint:
            print("Adaptive interface: setpoint change", setpoint)
            self.input_converter.update(x_ss=np.array(setpoint))
            
    def parameters_change_callback(self, message):
        data = message['data']
        parameters_dict = json.loads(data.decode("utf-8"))
        parameters = parameters_dict['parameters']
        if None not in parameters:
            print("Adaptive interface: parameters change", parameters)
            parameters_holder = self.input_converter.parameters_holder
            for key in parameters.keys():
                parameters_holder.__setattr__(key, parameters[key])
            self.prediction_model.update_parameters()
            self.output_converter.update()
            self.input_converter.update(update_u_ss=True)
            self.adaptive_controller.reset()


class Supervisor_Interface(Interface):
    def __init__(self, vehicle):
        Interface.__init__(self)
        self.redis_database.flushdb()
        self._init_db()
        self.vehicle = vehicle

    def _init_db(self):
        self.redis_database.set("drone_state", json.dumps(self.drone_state))
        self.redis_database.set("mpc_state", json.dumps(self.mpc_interface_state))
        self.redis_database.set("adaptive_state", json.dumps(self.adaptive_interface_state))
        self.redis_database.set("telemetry_manager_state", json.dumps(self.telemetry_manager_state))
        self.redis_database.set("estimator_state", json.dumps(self.estimator_interface_state))
    def update_db(self):
        self.set_vehicle_mode()
        self.drone_state['armed'] = self.vehicle.armed
        self.redis_database.set("drone_state", json.dumps(self.drone_state))

    def update_telemetry_manager_db(self):
        self.redis_database.set("telemetry_manager_state", json.dumps(self.telemetry_manager_state))

    def publish_setpoint(self, setpoint):
        setpoint = json.dumps({"setpoint": setpoint})
        self.redis_database.publish('setpoint_change', setpoint)

    def set_drone_state(self, x):
        self.drone_state['x'] = x

    def get_control(self):
        u = self.adaptive_interface_state['u']
        return u

    def set_vehicle_mode(self):
        self.drone_state['vehicle_mode'] = self.vehicle.mode.name

    def publish_parameters(self):
        parameters = self.get_model_parameters()
        if parameters is not None:
            parameters = json.dumps({'parameters': parameters})
            self.redis_database.publish('parameters_change', parameters)

    def get_estimated_mass(self):
        parameters = self.estimator_interface_state['parameters']
        if parameters is not None:
            mass = parameters['m']
        else:
            mass = 0.0
        return mass

    def get_model_parameters(self):
        parameters = self.estimator_interface_state['parameters']
        return parameters

    def start_identification_procedure(self, throttle):
        if throttle is not None and throttle <= 1.0 and throttle >= 0.0:
            self.telemetry_manager_state['identification_procedure_running'] = True
            self.telemetry_manager_state['identification_procedure_throttle'] = throttle
            self.update_telemetry_manager_db()
    def stop_identification_procedure(self):
        self.telemetry_manager_state['identification_procedure_running'] = False
        self.telemetry_manager_state['identification_procedure_throttle'] = None
        self.update_telemetry_manager_db()

    def is_identification_running(self):
        return self.telemetry_manager_state['identification_procedure_running']

    def get_throttle(self):
        throttle = self.telemetry_manager_state['identification_procedure_throttle']
        return throttle
    def reset(self):
        self.drone_state = deepcopy(drone_proxy_definition)

class Estimator_Interface(Interface):
    def __init__(self):
        Interface.__init__(self)
    def update_db(self):
        self.redis_database.set("estimator_state", json.dumps(self.estimator_interface_state))
    def update_parameters(self, parameters):
        if 'I' in parameters.keys():
            parameters['I'] = list(parameters['I'])
        self.estimator_interface_state['parameters'] = parameters

    def get_u_output(self):
        u_output = self.adaptive_interface_state['u_output']
        return np.array(u_output)
