import json
import redis
from Multiprocessing.PARAMS import REDIS_HOST, REDIS_PORT, DB_NUM
from QuadcopterIntegration.Utilities import dronekit_commands
import numpy as np

drone_proxy_definition = {
    'x': None
}

mpc_proxy_defintion = {
    'ref': None,
}

adaptive_proxy_definition = {
    'u': None,
}

telemetry_manager_proxy_definition = {
    'adaptive_running': False,
    'mpc_running': False
}
class MPC_Interface:
    def __init__(self):
        self.mpc_interface_state = mpc_proxy_defintion
        self.drone_state = drone_proxy_definition
        self.telemetry_manager_state = telemetry_manager_proxy_definition
        self.redis_database = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=DB_NUM)
    def update_db(self):
        self.redis_database.set("mpc_state", json.dumps(self.mpc_interface_state))

    def fetch_db(self):
        drone_state_packed = self.redis_database.get("drone_state")
        if drone_state_packed is not None:
            self.drone_state = json.loads(drone_state_packed.decode("utf-8"))
        telemetry_manager_state_packed = self.redis_database.get("telemetry_manager_state")
        if telemetry_manager_state_packed is not None:
            self.telemetry_manager_state = json.loads(telemetry_manager_state_packed.decode("utf-8"))

    def get_drone_state(self):
        x = self.drone_state['x']
        return np.array(x)

    def get_previous_control(self):
        u_prev = self.mpc_interface_state['ref']
        return np.array(u_prev)

    def set_control(self, u):
        u = list(u)
        self.mpc_interface_state['ref'] = u

    def is_mpc_running(self):
        return self.telemetry_manager_state['mpc_running']

class Adaptive_Interface:
    def __init__(self):
        self.mpc_interface_state = mpc_proxy_defintion
        self.drone_state = drone_proxy_definition
        self.adaptive_interface_state = adaptive_proxy_definition
        self.telemetry_manager_state = telemetry_manager_proxy_definition
        self.redis_database = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=DB_NUM)

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

class Supervisor_Interface:
    def __init__(self, vehicle):
        self.mpc_interface_state = mpc_proxy_defintion
        self.drone_state = drone_proxy_definition
        self.adaptive_interface_state = adaptive_proxy_definition
        self.telemetry_manager_state = telemetry_manager_proxy_definition
        self.redis_database = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=DB_NUM)
        self.vehicle = vehicle

    def update_db(self):
        self.redis_database.set("drone_state", json.dumps(self.drone_state))

    def update_telemetry_manager_db(self):
        self.redis_database.set("telemetry_manager_state", json.dumps(self.telemetry_manager_state))

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