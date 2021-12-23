from .receiver import Receiver
import struct


class TrafficLightReceiver(Receiver):
    def __init__(self, ip, port, callback):
        super().__init__(ip, port, callback)
        self.header = '#TrafficLight$'
        self.data_length = 16

    def _parse_data(self, raw_data):
        if self.header == raw_data[0:14].decode() and self.data_length == struct.unpack('i', raw_data[14:18])[0]:
            traffic_index = raw_data[30:42].decode()
            traffic_type, traffic_status = struct.unpack('2h', raw_data[42:46])
            self._parsed_data = [traffic_index, traffic_type, traffic_status]
        else:
            self._parsed_data = []
