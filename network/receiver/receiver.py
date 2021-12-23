from abc import ABCMeta, abstractmethod
import socket
import threading


class Receiver(metaclass=ABCMeta):
    def __init__(self, ip, port, callback):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))

        self._callback = callback

        self._parsed_data = []

        threading.Thread(target=self._receive_data, daemon=True).start()

    def __del__(self):
        self.sock.close()

    @property
    def parsed_data(self):
        return self._parsed_data

    def _receive_data(self):
        while True:
            data_size = 65535
            raw_data, _ = self.sock.recvfrom(data_size)
            self._parse_data(raw_data)
            self._callback(self._parsed_data)

    @abstractmethod
    def _parse_data(self, raw_data):
        pass
