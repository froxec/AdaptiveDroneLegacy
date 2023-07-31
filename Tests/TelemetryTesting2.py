from pytelemetry import Pytelemetry
from pytelemetry.transports.serialtransport import SerialTransport
import struct
import time

if __name__ == "__main__":
    transport = SerialTransport()
    tlm = Pytelemetry(transport)
    transport.connect({'port':'/dev/pts/4', 'baudrate':9600})
    tlm.publish(chr(2), 0.9, 'float32')

    def printer(topic, data, opts):
        print(topic, " : ", data)

    tlm.subscribe(None, printer)

    timeout = time.time() + 20
    while True:
        tlm.update()
        if time.time() > timeout:
            break


    transport.disconnect()