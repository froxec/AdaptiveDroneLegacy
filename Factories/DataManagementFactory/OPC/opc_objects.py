from opcua import ua, Server, Client
from QuadcopterIntegration.Utilities import dronekit_commands
import numpy as np
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
        objects = self.client.get_objects_node()
        self.drone_node = objects.get_child(["2:Drone"])
        self.state_node = self.drone_node.get_child(["2:state"])
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