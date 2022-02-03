from .receiver import Receiver
import struct


class EgoInfoReceiver(Receiver):
    def __init__(self, ip, port, callback):
        super().__init__(ip, port, callback)
        self.header = '#MoraiInfo$'
        self.data_length = 132

    def _parse_data(self, raw_data):
        if self.header == raw_data[0:11].decode() and self.data_length == struct.unpack('i', raw_data[11:15])[0]:
            ctrl_mode = struct.unpack('b', raw_data[27:28])[0]
            gear = struct.unpack('b', raw_data[28:29])[0]
            signed_vel = struct.unpack('f', raw_data[29:33])[0]   # km/h
            map_id = struct.unpack('i', raw_data[33:37])[0]
            accel = struct.unpack('f', raw_data[37:41])[0]
            brake = struct.unpack('f', raw_data[41:45])[0]
            size_x, size_y, size_z = struct.unpack('fff', raw_data[45:57])
            overhang, wheelbase, rear_overhang = struct.unpack('fff', raw_data[57:69])
            pos_x, pos_y, pos_z = struct.unpack('fff', raw_data[69:81])
            roll, pitch, yaw = struct.unpack('fff', raw_data[81:93])
            vel_x, vel_y, vel_z = struct.unpack('fff', raw_data[93:105])
            acc_x, acc_y, acc_z = struct.unpack('fff', raw_data[105:117])
            steer = struct.unpack('f', raw_data[117:121])[0]
            self._parsed_data = [
                ctrl_mode, gear, signed_vel, map_id, accel, brake, size_x, size_y, size_z, overhang, wheelbase,
                rear_overhang, pos_x, pos_y, pos_z, roll, pitch, yaw, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, steer
            ]
        else:
            self._parsed_data = []
