import socket
import struct

class Interface:
    def __init__(self) -> None:
        self.IP = None
        self.PORT = None
        self.REMOTE_IP = None
        self.REMOTE_PORT = None
        self.socket = None
    def READ_MESSAGE(self):
        raise NotImplementedError
    
    def SEND_MESSAGE(self):
        raise NotImplementedError

class ControllerInterface(Interface):
    def __init__(self, IP, PORT, REMOTE_IP, REMOTE_PORT) -> None:
        super().__init__()
        self.IP = IP
        self.PORT = PORT
        self.REMOTE_IP = REMOTE_IP
        self.REMOTE_PORT = REMOTE_PORT
        self.socket = socket.socket(socket.AF_INET,
                                    socket.SOCK_DGRAM)
        self.socket.bind((self.IP, self.PORT))

    def __call__(self, mode, *args):
        if mode == 'recv':
            x = self.READ_MESSAGE()
            return x
        if mode == 'send':
            u = args
            message = self.construct_message(u)
            self.SEND_MESSAGE(message)
        
    def READ_MESSAGE(self):
        data, addr = self.socket.recvfrom(1024)
        return data
    
    def SEND_MESSAGE(self, MESSAGE):
        self.socket.sendto(MESSAGE, (self.REMOTE_IP, self.REMOTE_PORT))
    
    def construct_message(self, u):
        u_struct = struct.pack('fff', u[0], u[1], u[2])
        return u_struct
    
class PCInterface(Interface):
    def __init__(self, IP, PORT, REMOTE_IP, REMOTE_PORT) -> None:
        super().__init__()
        self.IP = IP
        self.PORT = PORT
        self.REMOTE_IP = REMOTE_IP
        self.REMOTE_PORT = REMOTE_PORT
        self.socket = socket.socket(socket.AF_INET,
                                    socket.SOCK_DGRAM)
        self.socket.bind((self.IP, self.PORT))

    def __call__(self, mode, *args):
        if mode == 'recv':
            u = self.READ_MESSAGE()
            return u
        if mode == 'send':
            x = args
            message = self.construct_message(x)
            self.SEND_MESSAGE(message)

    def READ_MESSAGE(self):
        data, addr = self.socket.recvfrom(1024)
        return data
    
    def SEND_MESSAGE(self, MESSAGE):
        self.socket.sendto(MESSAGE, (self.REMOTE_IP, self.REMOTE_PORT))
    
    def construct_message(self, x):
        u_struct = struct.pack('ffffff', x[0], x[1], x[2], x[3], x[4], x[5])
        return u_struct