from .receiver import Receiver
import struct


class EgoInfoReceiver(Receiver):
    def __init__(self, ip, port, callback):
        super().__init__(ip, port, callback)
        self.header = '#MoraiInfo$'
        self.data_length = 216

    def _parse_data(self, raw_data):
        if self.header == raw_data[0:11].decode() and self.data_length == struct.unpack('i', raw_data[11:15])[0]:
            secs = struct.unpack('f', raw_data[27:31])[0]
            nsecs = struct.unpack('f', raw_data[31:35])[0]
            ctrl_mode = struct.unpack('b', raw_data[35:36])[0]
            gear = struct.unpack('b', raw_data[36:37])[0]
            signed_vel = struct.unpack('f', raw_data[37:41])[0]   # km/h
            map_id = struct.unpack('i', raw_data[41:45])[0]
            accel = struct.unpack('f', raw_data[45:49])[0]
            brake = struct.unpack('f', raw_data[49:53])[0]
            size_x, size_y, size_z = struct.unpack('fff', raw_data[53:65])
            overhang, wheelbase, rear_overhang = struct.unpack('fff', raw_data[65:77])
            pos_x, pos_y, pos_z = struct.unpack('fff', raw_data[77:89])
            roll, pitch, yaw = struct.unpack('fff', raw_data[89:101])
            vel_x, vel_y, vel_z = struct.unpack('fff', raw_data[101:113])
            ang_vel_x, ang_vel_y, ang_vel_z = struct.unpack('fff', raw_data[113:125])
            acc_x, acc_y, acc_z = struct.unpack('fff', raw_data[125:137])
            front_steer, rear_steer = struct.unpack('ff', raw_data[137:145])
            link_id = raw_data[145:183].decode()
            tire_lateral_force_fl, tire_lateral_force_fr, tire_lateral_force_rl, tire_lateral_force_rr = struct.unpack('ffff', raw_data[183:199])
            side_slip_angle_fl, side_slip_angle_fr, side_slip_angle_rl, side_slip_angle_rr = struct.unpack('ffff', raw_data[199:215])
            tire_cornering_stiffness_fl, tire_cornering_stiffness_fr, tire_cornering_stiffness_rl, tire_cornering_stiffness_rr = struct.unpack('ffff', raw_data[215:231])
            distance_left_lane_boundary, distance_right_lane_boundary = struct.unpack('ff', raw_data[231:239])
            cross_track_error = struct.unpack('f', raw_data[239:243])[0]
            self._parsed_data = [
                ctrl_mode, gear, signed_vel, map_id, accel, brake, size_x, size_y, size_z, overhang, wheelbase,
                rear_overhang, pos_x, pos_y, pos_z, roll, pitch, yaw, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, front_steer
            ]
        else:
            self._parsed_data = []
