from Factories.DataManagementFactory.OPC.opc_objects import AdaptiveClient
from Multiprocessing.PARAMS import OPC_SERVER_ADDRESS
from oclock import Timer
import time

if __name__ == "__main__":
    # parameters
    FREQ = 50
    DELTA_T = 1 / FREQ

    # init database client
    adaptive_client = AdaptiveClient(OPC_SERVER_ADDRESS)

    # init adaptive controller

    # init Timer
    timer = Timer(interval=DELTA_T)

    while True:
        t1 = time.time()
        # check if adaptive controller is running
        if adaptive_client.adaptive_running_node.get_value():
            # calculate adaptive control
            ref = adaptive_client.ref_node.get_value()

        else:
            # pass mpc control
            u = adaptive_client.ref_node.get_value()

        # process thrust to throttle

        # set output
        adaptive_client.set_control(u)
        timer.checkpt()
        print(time.time() - t1)