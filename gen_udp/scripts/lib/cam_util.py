import cv2
import numpy as np
import os
import socket
import struct
import threading
import os,time

clear = lambda: os.system('cls') 

class UDP_CAM_Parser:
    
    def __init__(self, ip, port, params_cam=None):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (ip,port)
        self.sock.bind(recv_address)

        print("connected")

        self.data_size=int(65000)
        
        self.max_len = 10 #640X480
        self.raw_img=None
        self.is_img=False
        thread = threading.Thread(target=self.loop)
        thread.daemon = True 
        thread.start() 

    def loop(self):
        while True:
            self.raw_img=self.recv_udp_data()
            self.is_img=True

    def check_max_len(self):

        idx_list=[]

        for _ in range(self.ready_step):

            UnitBlock, sender = self.sock.recvfrom(self.data_size)

            print("check the size .. ")
            
            idx_list.append(np.fromstring(UnitBlock[3:7], dtype = "int"))

        self.max_len = np.max(idx_list)+1

    def recv_udp_data(self):

        TotalBuffer = b''
        num_block = 0

        while True:

            UnitBlock, sender = self.sock.recvfrom(self.data_size)

            # traffic light
            if np.frombuffer(UnitBlock[0:3], dtype = "S1")[0] == b'T':  
                UnitIdx = np.frombuffer(UnitBlock[3:7], dtype = "int")[0]
                UnitSize = np.frombuffer(UnitBlock[7:11], dtype = "int")[0]
                UnitTail = UnitBlock[-2:]

                if UnitTail==b'ET':
                    for i in range(int(UnitSize/19)):
                        print("------------------------------")
                        print("{}th Traffic Light".format(i+1))

                        print("min_x {}".format(np.frombuffer(UnitBlock[19*i+11:19*i+15], dtype = "float32")))
                        print("min_y {}".format(np.frombuffer(UnitBlock[19*i+15:19*i+19], dtype = "float32")))
                        print("max_x {}".format(np.frombuffer(UnitBlock[19*i+19:19*i+23], dtype = "float32")))
                        print("max_y {}".format(np.frombuffer(UnitBlock[19*i+23:19*i+27], dtype = "float32")))

                        print("group data {}".format(hex(np.frombuffer(UnitBlock[19*i+27:19*i+28], dtype = "uint8")[0])))
                        print("class data {}".format(hex(np.frombuffer(UnitBlock[19*i+28:19*i+29], dtype = "uint8")[0])))
                        print("subclass data {}".format(hex(np.frombuffer(UnitBlock[19*i+29:19*i+30], dtype = "uint8")[0])))
                        print("------------------------------")

                    

            # Camera Image
            if np.frombuffer(UnitBlock[0:3], dtype = "S1")[0] == b'M':
                UnitIdx = np.frombuffer(UnitBlock[3:7], dtype = "int")[0]
                UnitSize = np.frombuffer(UnitBlock[7:11], dtype = "int")[0]
                UnitTail = UnitBlock[-2:]
                UnitBody = UnitBlock[11:(11 + UnitSize)]

                TotalBuffer+=UnitBody

                if UnitTail==b'EI':
                
                    TotalIMG = cv2.imdecode(np.fromstring(TotalBuffer[-64987*self.max_len-UnitSize:], np.uint8), 1)

                    TotalBuffer = b''

                    break

            # 2D/3D Bounding Box
            if np.frombuffer(UnitBlock[0:3], dtype = "S1")[0] == b'B':

                UnitIdx = np.frombuffer(UnitBlock[3:7], dtype = "int")[0]
                UnitSize = np.frombuffer(UnitBlock[7:11], dtype = "int")[0]
                UnitTail = UnitBlock[-2:]

                if UnitTail==b'EO':
                    for i in range(int(UnitSize/179)):
                        print("------------------------------")
                        print("{}th Object".format(i+1))
                        for j in range(8):
                            print("{}th world x position {}".format(j+1, np.frombuffer(UnitBlock[179*i+11+12*j:179*i+15+12*j], dtype = "float32")))
                            print("{}th world y position {}".format(j+1, np.frombuffer(UnitBlock[179*i+15+12*j:179*i+19+12*j], dtype = "float32")))
                            print("{}th world z position {}".format(j+1, np.frombuffer(UnitBlock[179*i+19+12*j:179*i+23+12*j], dtype = "float32")))

                        for k in range(8):
                            print("{}th image_coner_x {}".format(k+1, np.frombuffer(UnitBlock[179*i+23+84+8*k:179*i+27+84+8*k], dtype = "float32")))
                            print("{}th image_coner_y {}".format(k+1, np.frombuffer(UnitBlock[179*i+27+84+8*k:179*i+31+84+8*k], dtype = "float32")))

                        print("min_x {}".format( np.frombuffer(UnitBlock[179*i+31+84+56:179*i+35+84+56], dtype = "float32")))
                        print("min_y {}".format( np.frombuffer(UnitBlock[179*i+35+84+56:179*i+39+84+56], dtype = "float32")))
                        print("max_x {}".format( np.frombuffer(UnitBlock[179*i+39+84+56:179*i+43+84+56], dtype = "float32")))
                        print("max_y {}".format( np.frombuffer(UnitBlock[179*i+43+84+56:179*i+47+84+56], dtype = "float32")))

                        print("group data {}".format(hex(np.frombuffer(UnitBlock[179*i+47+84+56:179*i+48+84+56], dtype = "uint8")[0])))
                        print("class data {}".format(hex(np.frombuffer(UnitBlock[179*i+48+84+56:179*i+49+84+56], dtype = "uint8")[0])))
                        print("subclass data {}".format(hex(np.frombuffer(UnitBlock[179*i+49+84+56:179*i+50+84+56], dtype = "uint8")[0]))) 

                        print("------------------------------")

                
        return TotalIMG
        
    def __del__(self):
        self.sock.close()
        print('del')
