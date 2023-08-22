from opcua import ua, Server
class DroneDataServer:
    def __init__(self, endpoint_address):
        server = Server()
        server.set_endpoint(endpoint_address)

        # setup our own namespace, not really necessary but should as spec
        uri = "OPC_SERVER"
        ns = server.register_namespace(uri)

        # get Objects node, this is where we should put our nodes
        objects = server.get_objects_node()

        # populating our address space
        vehicleState = objects.add_object(ns, "VehicleState")
        x = vehicleState.add_variable(ns, "x", 0.0)
        y = vehicleState.add_variable(ns, "y", 0.0)
        z = vehicleState.add_variable(ns, "z", 0.0)
        Vx = vehicleState.add_variable(ns, "Vx", 0.0)
        Vy = vehicleState.add_variable(ns, "Vy", 0.0)
        Vz = vehicleState.add_variable(ns, "Vz", 0.0)

        self.state_vars = [x, y, z, Vx, Vy, Vz]
        for state_var in self.state_vars:
            state_var.set_writable()
        # starting!
        server.start()