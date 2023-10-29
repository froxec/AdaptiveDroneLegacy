import struct
import serial

MESSAGES_MAPPING = {
    'ARM': (5, '')
}
class SerialFramer:
    def __init__(self):
        self.start_byte = struct.pack('!B', int('0x02', 16))
        self.stop_byte = struct.pack('!B', int('0x03', 16))
        self.esc_byte = struct.pack('!B', int('0x04', 16))

    def create_message(self, message_type, data=None):
        message_id, message_encoding = MESSAGES_MAPPING[message_type]
        if message_encoding != '' and data == None:
            raise ValueError("Data should be of type {}".format(message_encoding))
        if data is not None:
            bin_data = struct.pack(message_encoding, data)
        else:
            bin_data = struct.pack('x')
        header = self.create_header(message_id, bin_data)
        message = self.start_byte + header + bin_data + self.stop_byte
        return message

    def create_header(self, message_id, bin_data):
        length = len(bin_data)
        header = struct.pack('<BB', message_id, length)
        return header

    def read_header(self, header):
        message_id, length = struct.unpack('<BB', header)
        print("Message id:", message_id)
        print("Length:", length)



if __name__ == "__main__":
    master = serial.Serial('/dev/pts/5', baudrate=115200, timeout=0.05)
    client = serial.Serial('/dev/pts/5', baudrate=115200, timeout=0.05)
    framer = SerialFramer()
    header = framer.create_header(5, struct.pack('<f', 5.0))
    msg = framer.create_message('ARM')
    serial.write(msg)
