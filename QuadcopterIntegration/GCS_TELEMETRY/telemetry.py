import serial

from QuadcopterIntegration.Utilities.gcs_comm_functions import readThread, sendThread

uav = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)

if __name__ == "__main__":
    read = readThread(uav)
    send = sendThread(uav)
    while True:
        pass