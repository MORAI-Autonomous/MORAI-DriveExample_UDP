from .sender import Sender
import struct


class TrafficLightSender(Sender):
    def __init__(self, ip, port):
        super().__init__(ip, port)
        message_name = '#TrafficLight$'.encode()
        data_length = struct.pack('i', 14)
        aux_data = struct.pack('iii', 0, 0, 0)
        self.header = message_name + data_length + aux_data
        self.tail = '\r\n'.encode()

    def _format_data(self, data):
        traffic_index = data[0].encode()
        traffic_status = struct.pack('h', data[1])
        message = traffic_index + traffic_status
        formatted_data = self.header + message + self.tail

        return formatted_data
