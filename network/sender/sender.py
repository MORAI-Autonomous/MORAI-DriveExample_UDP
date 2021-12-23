from abc import ABCMeta, abstractmethod
import socket


class Sender(metaclass=ABCMeta):
    def __init__(self, ip, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = (ip, port)

    def send_data(self, data):
        formatted_data = self._format_data(data)
        self.sock.sendto(formatted_data, self.address)

    @abstractmethod
    def _format_data(self, data):
        pass
