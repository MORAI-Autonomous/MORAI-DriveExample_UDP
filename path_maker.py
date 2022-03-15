# from lib.morai_udp_parser import udp_parser

import numpy as np

from autonomous_driving.vehicle_state import VehicleState
from autonomous_driving.config.config import Config
from network.receiver import EgoInfoReceiver

import time
import csv
import threading
from math import cos,sin,sqrt,pow,atan2,pi

import os,json
import sys


path = os.path.dirname(os.path.abspath( __file__ ))

user_ip = '127.0.0.1'
status_port = 9090
path_folder_name = 'path'
path_file_name = 'path.csv'

class path_maker :

    def __init__(self):
       
        self.file_path=os.path.dirname( os.path.abspath( __file__ ) )
        full_path = self.file_path+'/'+path_folder_name+'/'+path_file_name
        self.f = open(full_path, 'w')
        self.wr = csv.writer(self.f)
        self.wr.writerow(['x','y','z'])

        self.vehicle_state = None
        self.prev_x = 0
        self.prev_y = 0
        
        self._is_status=False
        self._set_protocol()

        self.main_loop()

    def _set_protocol(self):
        # receiver
        self.ego_info_receiver = EgoInfoReceiver(
            user_ip,status_port, self._ego_info_callback
        )

    
    def main_loop(self):

        while True:
            try:
                if self.vehicle_state:
                
                    x=self.vehicle_state.position.x
                    y=self.vehicle_state.position.y
                    z=0
                    
                    distance = sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
                    if distance > 0.5 :
                        self.wr.writerow([x,y,z])
                        self.prev_x=x
                        self.prev_y=y
                        print(x,y)
            except KeyboardInterrupt :
                self.f.close()

    def _ego_info_callback(self, data):
        if data:
            self.vehicle_state = VehicleState(data[12], data[13], np.deg2rad(data[17]), data[18]/3.6)
            self.vehicle_currenty_steer = data[-1]
        else:
            self.vehicle_state = None


if __name__ == "__main__":
    path=path_maker()
