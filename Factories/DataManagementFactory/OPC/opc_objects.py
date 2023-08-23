from opcua import ua, Server, Client
from QuadcopterIntegration.Utilities import dronekit_commands
import numpy as np
import time

class ControlHandler(object):
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.t1 = None
    def datachange_notification(self, node, val, data):
        if self.t1 is not None:
            print("Control handler dt", time.time() - self.t1)
        self.t1 = time.time()
        print("Control handler: output", val)
        if val is not None:
            dronekit_commands.set_attitude(self.vehicle, val[1], val[2], 0, val[0])

class AlgorithmSwitchingHandler(object):
    def datachange_notification(self, node, val, data):
        print(node, val, data)

class DroneDataServer:
    def __init__(self, endpoint_address):
        self.server = Server()
        self.server.set_endpoint(endpoint_address)

        self.server.import_xml("./Multiprocessing/server_model.xml")

        self.server.start()

class DataAcquisition:
    def __init__(self, vehicle, endpoint_address):
        self.vehicle = vehicle
        self.client = Client(endpoint_address)

        try:
            self.client.connect()
        except:
            raise "acquisition client could not connect to opc server on {}".format(endpoint_address)

        # get required nodes
        objects = self.client.get_objects_node()
        self.drone_node = objects.get_child(["2:Drone"])
        self.state_node = self.drone_node.get_child(["2:state"])
        self.mpc_node = objects.get_child(["2:MPC"])
        self.mpc_output_node = self.mpc_node.get_child(["2:output"])
        self.adaptive_node = objects.get_child(["2:AdaptiveController"])
        self.adpative_output_node = self.adaptive_node.get_child(["2:output"])

        #control handler
        self.control_handler = ControlHandler(self.vehicle)
        sub = self.client.create_subscription(500, self.control_handler)
        self.handle = sub.subscribe_data_change(self.adpative_output_node)

    def update_state(self):
        x = dronekit_commands.get_state(self.vehicle)
        self.state_node.set_value(x)
class MPCClient:
    def __init__(self, endpoint_address):
        self.client = Client(endpoint_address)
        try:
            self.client.connect()
        except:
            raise "MPC client could not connect to opc server on {}". format(endpoint_address)

        objects = self.client.get_objects_node()
        self.mpc_node = objects.get_child(["2:MPC"])
        self.drone_node = objects.get_child(["2:Drone"])
        self.u_node = self.mpc_node.get_child(["2:output"])
        self.x_node = self.drone_node.get_child(["2:state"])
        self.mpc_running_node = self.mpc_node.get_child(["2:running"])
        self.current_state = None
    def set_control(self, u):
        u = list(u)
        self.current_state = u
        self.u_node.set_value(u)

    def get_current_state(self):
        x = self.x_node.get_value()
        return np.array(x)

    def get_previous_control(self):
        u_prev = self.u_node.get_value()
        return np.array(u_prev)

class AdaptiveClient:
    def __init__(self, endpoint_address):
        self.client = Client(endpoint_address)
        try:
            self.client.connect()
        except:
            raise "Adaptive client could not connect to opc server on {}". format(endpoint_address)

        # get required nodes
        objects = self.client.get_objects_node()
        self.adaptive_node = objects.get_child(["2:AdaptiveController"])
        self.mpc_node = objects.get_child(["2:MPC"])
        self.drone_node = objects.get_child(["2:Drone"])
        self.u_node = self.adaptive_node.get_child(["2:output"])
        self.ref_node = self.mpc_node.get_child(["2:output"])
        self.x_node = self.drone_node.get_child(["2:state"])
        self.adaptive_running_node = self.adaptive_node.get_child(["2:running"])
        self.mpc_running_node = self.mpc_node.get_child(["2:running"])

        self.current_state = None
    def set_control(self, u):
        u = list(u)
        self.current_state = u
        self.u_node.set_value(u)

    def get_current_state(self):
        x = self.x_node.get_value()
        return np.array(x)

    def get_previous_control(self):
        u_prev = self.ref_node.get_value()
        return np.array(u_prev)

class Supervisor:
    def __init__(self, endpoint_address, vehicle):
        self.client = Client(endpoint_address)
        try:
            self.client.connect()
        except:
            raise "Supervisor client could not connect to opc server on {}".format(endpoint_address)

        self.vehicle = vehicle
