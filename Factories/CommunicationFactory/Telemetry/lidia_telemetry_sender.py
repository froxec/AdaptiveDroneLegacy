import numpy as np

from Factories.ToolsFactory.GeneralTools import BidirectionalDict
import socket
import json
# TELEMETRY MAPPING
TELEMETRY_MAPPING = BidirectionalDict()
TELEMETRY_MAPPING['ned'] = 'position_local'
TELEMETRY_MAPPING['att'] = 'attitude'
TELEMETRY_MAPPING['v_ned'] = 'velocity'
class LidiaTelemetrySender:
    def __init__(self, address_port=("localhost", 5000)):
        self.telemetry_lidia = telemetry_default
        self.telemetry_to_update_names = ['ned', 'att', 'v_ned']
        self.address_port = address_port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def __call__(self, telemetry):
        self.update_telemetry(telemetry)
        return
    def update_telemetry(self, telemetry):
        print(self.telemetry_lidia)
        for tel in self.telemetry_to_update_names:
            data = telemetry[TELEMETRY_MAPPING[tel]]
            if isinstance(data, (list, np.ndarray)):
                for i in range(len(list(data))):
                    data[i] = 0.0 if data[i] is None else data[i]
                self.telemetry_lidia[tel] = data
            else:
                self.telemetry_lidia[tel] = data
        telem_to_send = json.dumps(self.telemetry_lidia).encode('utf-8')
        self.socket.sendto(telem_to_send, self.address_port)

telemetry_default = {
   "ned":[
      0,
      0,
      -1.51536243315903
    ]
}