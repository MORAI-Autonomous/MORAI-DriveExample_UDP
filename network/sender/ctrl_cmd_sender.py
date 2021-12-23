from .sender import Sender
import struct


class CtrlCmdSender(Sender):
    def __init__(self, ip, port):
        super().__init__(ip, port)
        message_name = '#MoraiCtrlCmd$'.encode()
        data_length = struct.pack('i', 23)
        aux_data = struct.pack('iii', 0, 0, 0)
        self.header = message_name + data_length + aux_data
        self.tail = '\r\n'.encode()

    def _format_data(self, data):
        mode = struct.pack('b', 2)  # 1: KeyBoard / 2: AutoMode
        gear = struct.pack('b', 4)  # 1: Parking / 2: Reverse / 3: Neutral / 4: Drive
        cmd_type = struct.pack('b', 1)  # 1: Throttle  /  2: Velocity  /  3: Acceleration
        velocity = struct.pack('f', 0)  # cmd_type이 2일때 원하는 속도를 넣어준다.
        acceleration = struct.pack('f', 0)  # cmd_type이 3일때 원하는 가속도를 넣어준다.

        accel = struct.pack('f', data[0])
        brake = struct.pack('f', data[1])
        steering = struct.pack('f', data[2])
        message = mode + gear + cmd_type + velocity + acceleration + accel + brake + steering
        formatted_data = self.header + message + self.tail

        return formatted_data
