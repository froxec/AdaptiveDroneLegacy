import socket
import struct
import select
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
            message = self.READ_MESSAGE()
            if message is not None:
                decoded_message = self.decode_message(message)
                return decoded_message
            else:
                return None
        if mode == 'send':
            u = args[0]
            message = self.construct_message(u)
            self.SEND_MESSAGE(message)
        
    def READ_MESSAGE(self):
        print("Waiting for message...")
        data, addr = self.socket.recvfrom(1024)
        print("Data {} received from {}".format(data, addr))
        return data
    
    def SEND_MESSAGE(self, MESSAGE):
        print("Sending message...")
        self.socket.sendto(MESSAGE, (self.REMOTE_IP, self.REMOTE_PORT))
        print("Message {} sent".format(MESSAGE))
    
    def construct_message(self, u):
        u_struct = struct.pack('fff', u[0], u[1], u[2])
        return u_struct
    def decode_message(self, message):
        unpacked = struct.unpack('fffffffff', message)
        return unpacked
    
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
        self.timeout_in_seconds = 0.0

    def __call__(self, mode, *args):
        if mode == 'recv':
            message = self.READ_MESSAGE()
            if message is not None:
                u = self.decode_message(message)
                return u
            else:
                return None
        if mode == 'send':
            x = args[0]
            ref = args[1]
            message = self.construct_message(x, ref)
            self.SEND_MESSAGE(message)

    def READ_MESSAGE(self):
        print("Waiting for message...")
        ready = select.select([self.socket], [], [], self.timeout_in_seconds)
        if ready[0]:
            data, addr = self.socket.recvfrom(1024)
            print("Data {} received from {}".format(data, addr))
            return data
        else:
            return None
    def SEND_MESSAGE(self, MESSAGE):
        print("Sending message...")
        self.socket.sendto(MESSAGE, (self.REMOTE_IP, self.REMOTE_PORT))
        print("Message {} sent".format(MESSAGE))
    
    def construct_message(self, x, ref):
        u_struct = struct.pack('fffffffff', x[0], x[1], x[2], x[3], x[4], x[5], ref[0], ref[1], ref[2])
        return u_struct

    def decode_message(self, message):
        unpacked = struct.unpack('fff', message)
        return unpacked