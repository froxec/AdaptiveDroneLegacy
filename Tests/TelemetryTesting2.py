from pytelemetry import Pytelemetry
from pytelemetry.transports.serialtransport import SerialTransport
from Factories.CommunicationFactory.Telemetry.mappings import *
import struct
import time

if __name__ == "__main__":
    transport = SerialTransport()
    tlm = Pytelemetry(transport, addr=40, freq=868)
    transport.connect({'port':'/dev/ttyUSB0', 'baudrate':115200})

    tlm.publish(chr(10), 20, 'int8', 2, 868)
    
    def printer(topic, data, opts):
        print(topic, " : ", data)

    tlm.subscribe(None, printer)

    timeout = time.time() + 20
    start = time.time()
    while True:
        tlm.update()
        time.sleep(0.05)


    transport.disconnect()