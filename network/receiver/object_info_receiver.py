from .receiver import Receiver
import struct


class ObjectInfoReceiver(Receiver):
    def __init__(self, ip, port, callback):
        super().__init__(ip, port, callback)
        self.header = '#MoraiObjInfo$'

    def _parse_data(self, raw_data):
        if self.header == raw_data[0:14].decode():
            object_info_list = []
            offset_byte = 30
            data_size = 68
            for i in range(20):
                start_byte = i*data_size + offset_byte
                object_id, object_type = struct.unpack('hh', raw_data[start_byte:start_byte+4])
                pos_x, pos_y, pos_z = struct.unpack('fff', raw_data[start_byte+4:start_byte+16])
                heading = struct.unpack('f', raw_data[start_byte+16:start_byte+20])[0]
                size_x, size_y, size_z = struct.unpack('fff', raw_data[start_byte+20:start_byte+32])
                overhang, wheelbase, rear_overhang = struct.unpack('fff', raw_data[start_byte+32:start_byte+44])
                vel_x, vel_y, vel_z = struct.unpack('fff', raw_data[start_byte+44:start_byte+56])
                acc_x, acc_y, acc_z = struct.unpack('fff', raw_data[start_byte+56:start_byte+68])
                object_info = [
                    object_id, object_type, pos_x, pos_y, pos_z, heading, size_x, size_y, size_z,
                    overhang, wheelbase, rear_overhang, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z
                ]
                if object_info[0] != 0:
                    object_info_list.append(object_info)

            self._parsed_data = object_info_list
        else:
            self._parsed_data = []
