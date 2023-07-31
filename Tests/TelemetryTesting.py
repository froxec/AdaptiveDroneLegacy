from pytelemetry import Pytelemetry
from pytelemetry.transports.serialtransport import SerialTransport
import time

if __name__ == "__main__":
    transport = SerialTransport()
    tlm = Pytelemetry(transport)
    transport.connect({'port':'/dev/pts/5', 'baudrate':9600})
    tlm.publish(chr(1) +":1", 0.9, 'float32')
    tlm.publish(chr(1) + ":2", 0.5, 'float32')
    def printer(topic, data, opts):
        print(topic, " : ", data)

    tlm.subscribe(None, printer)

    timeout = time.time() + 20
    while True:
        tlm.update()
        if time.time() > timeout:
            break


    transport.disconnect()