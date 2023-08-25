import json
import redis
from Multiprocessing.PARAMS import REDIS_HOST, REDIS_PORT, DB_NUM
from Factories.SimulationsFactory.TrajectoriesDepartment.trajectories import SinglePoint
from QuadcopterIntegration.Utilities import dronekit_commands
import numpy as np
from copy import deepcopy

drone_proxy_definition = {
    'x': None,
    'armed': False,
}

mpc_proxy_defintion = {
    'ref': None,
    'current_setpoint': [None, None, None],
}

adaptive_proxy_definition = {
    'u': None,
}

telemetry_manager_proxy_definition = {
    'adaptive_running': False,
    'mpc_running': False,
    'setpoint': None,
}
class MPC_Interface:
    def __init__(self, position_controller):
        self.mpc_interface_state = deepcopy(mpc_proxy_defintion)
        self.drone_state = deepcopy(drone_proxy_definition)
        self.telemetry_manager_state = deepcopy(telemetry_manager_proxy_definition)
        self.redis_database = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=DB_NUM)
        self.pubsub = self.redis_database.pubsub()
        self.pubsub.subscribe(**{'setpoint_change': self.setpoint_change_callback})

        # get access to position controller for setpoint callback
        self.position_controller = position_controller
    def update_db(self):
        self.redis_database.set("mpc_state", json.dumps(self.mpc_interface_state))

    def fetch_db(self):
        drone_state_packed = self.redis_database.get("drone_state")
        if drone_state_packed is not None:
            self.drone_state = json.loads(drone_state_packed.decode("utf-8"))
        telemetry_manager_state_packed = self.redis_database.get("telemetry_manager_state")
        if telemetry_manager_state_packed is not None:
            self.telemetry_manager_state = json.loads(telemetry_manager_state_packed.decode("utf-8"))
        self.pubsub.get_message()

    def get_drone_state(self):
        x = self.drone_state['x']
        return np.array(x)

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

    def is_mpc_running(self):
        return self.telemetry_manager_state['mpc_running']

    def is_vehicle_armed(self):
        return self.drone_state['armed']

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
class Adaptive_Interface:
    def __init__(self,
                 input_converter,
                 output_converter):
        self.mpc_interface_state = deepcopy(mpc_proxy_defintion)
        self.drone_state = deepcopy(drone_proxy_definition)
        self.adaptive_interface_state = deepcopy(adaptive_proxy_definition)
        self.telemetry_manager_state = deepcopy(telemetry_manager_proxy_definition)
        self.redis_database = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=DB_NUM)
        self.pubsub = self.redis_database.pubsub()
        self.pubsub.subscribe(**{'setpoint_change': self.setpoint_change_callback})
        #get converters representations
        self.input_converter = input_converter
        self.output_converter = output_converter

    def update_db(self):
        self.redis_database.set("adaptive_state", json.dumps(self.adaptive_interface_state))

    def fetch_db(self):
        drone_state_packed = self.redis_database.get("drone_state")
        if drone_state_packed is not None:
            self.drone_state = json.loads(drone_state_packed.decode("utf-8"))
        mpc_state_packed = self.redis_database.get("mpc_state")
        if mpc_state_packed is not None:
            self.mpc_interface_state = json.loads(mpc_state_packed.decode("utf-8"))
        telemetry_manager_state_packed = self.redis_database.get("telemetry_manager_state")
        if telemetry_manager_state_packed is not None:
            self.telemetry_manager_state = json.loads(telemetry_manager_state_packed.decode("utf-8"))
        self.pubsub.get_message()

    def get_drone_state(self):
        x = self.drone_state['x']
        return np.array(x)

    def set_control(self, u):
        u = list(u)
        self.adaptive_interface_state['u'] = u

    def get_ref(self):
        ref = np.array(self.mpc_interface_state['ref'])
        return ref

    def is_adaptive_running(self):
        return self.telemetry_manager_state['adaptive_running']

    def is_vehicle_armed(self):
        return self.drone_state['armed']

    def get_current_setpoint(self):
        current_setpoint = np.array(self.mpc_interface_state['current_setpoint'])
        return np.array(current_setpoint)

    def reset_state(self):
        self.adaptive_interface_state = deepcopy(adaptive_proxy_definition)


    def setpoint_change_callback(self, message):
        data = message['data']
        setpoint_dict = json.loads(data.decode("utf-8"))
        setpoint = setpoint_dict['setpoint']
        if None not in setpoint:
            print("Adaptive interface: setpoint change", setpoint)
            self.input_converter.update(x_ss=np.array(setpoint))

class Supervisor_Interface:
    def __init__(self, vehicle):
        self.mpc_interface_state = deepcopy(mpc_proxy_defintion)
        self.drone_state = deepcopy(drone_proxy_definition)
        self.adaptive_interface_state = deepcopy(adaptive_proxy_definition)
        self.telemetry_manager_state = deepcopy(telemetry_manager_proxy_definition)
        self.redis_database = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=DB_NUM)
        self.redis_database.flushdb()
        self._init_db()
        self.vehicle = vehicle

    def _init_db(self):
        self.redis_database.set("drone_state", json.dumps(self.drone_state))
        self.redis_database.set("mpc_interface_state", json.dumps(self.mpc_interface_state))
        self.redis_database.set("adaptive_interface_state", json.dumps(self.adaptive_interface_state))
        self.redis_database.set("telemetry_manager_state", json.dumps(self.telemetry_manager_state))
    def update_db(self):
        self.drone_state['armed'] = self.vehicle.armed
        self.redis_database.set("drone_state", json.dumps(self.drone_state))

    def update_telemetry_manager_db(self):
        self.redis_database.set("telemetry_manager_state", json.dumps(self.telemetry_manager_state))

    def publish_setpoint(self, setpoint):
        setpoint = json.dumps({"setpoint": setpoint})
        self.redis_database.publish('setpoint_change', setpoint)

    def fetch_db(self):
        adaptive_state_packed = self.redis_database.get("adaptive_state")
        if adaptive_state_packed is not None:
            self.adaptive_interface_state = json.loads(adaptive_state_packed.decode("utf-8"))
        mpc_state_packed = self.redis_database.get("mpc_state")
        if mpc_state_packed is not None:
            self.mpc_interface_state = json.loads(mpc_state_packed.decode("utf-8"))
        telemetry_manager_state_packed = self.redis_database.get("telemetry_manager_state")
        if telemetry_manager_state_packed is not None:
            self.telemetry_manager_state = json.loads(telemetry_manager_state_packed.decode("utf-8"))

    def set_drone_state(self):
        x = dronekit_commands.get_state(self.vehicle)
        self.drone_state['x'] = x

    def get_control(self):
        u = self.adaptive_interface_state['u']
        return u

    def reset(self):
        self.drone_state = deepcopy(drone_proxy_definition)