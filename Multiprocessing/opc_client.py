from Multiprocessing.PARAMS import OPC_SERVER_ADDRESS
from opcua import Client

if __name__ == "__main__":

    client = Client(OPC_SERVER_ADDRESS)

    try:
        client.connect()
        root = client.get_root_node()

        # Now getting a variable node using its browse path
        x = root.get_child(["0:Objects", "2:VehicleState", "2:x"])
        while True:
            print(x.get_value())
    finally:
        client.disconnect()