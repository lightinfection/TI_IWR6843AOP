import os
import sys
import time
import serial
import struct
import numpy as np


class get_data:
    def __init__(self, command_port="", data_port="", command_rate="115200", data_rate="921600", cfg_path="", connection=True, timeout=5):
        self.cfg_path = cfg_path
        self.connected = False
        self.config_params = {}
        self.ms_per_frame = 1000
        self.__found_rate = 0
        self.__timeout = timeout
        self.__buffer_temp = b''
        self.__start_time = time.time()
        self.__magicWord = b'\x02\x01\x04\x03\x06\x05\x08\x07'
        if connection:
            try:
                self.command_port = serial.Serial(command_port, command_rate)
                self.data_port = serial.Serial(data_port, data_rate)
            except:
                auth1 = os.system("sudo chmod +777 " + command_port)
                if(auth1==256 or auth1==1):
                    sys.stderr.write("Authorization of cli_port failed... \n")
                    os._exit(1)
                auth2 = os.system("sudo chmod +777 " + data_port)
                if(auth2==256 or auth2==1):
                    sys.stderr.write("Authorization of data_port failed... \n")
                    os._exit(1)
                self.command_port = serial.Serial(command_port, command_rate)
                self.data_port = serial.Serial(data_port, data_rate)
            self.connected = True
        if self.connected:
            self.__load_cfgs()
        
    def __load_cfgs(self):
        try:
            with open(self.cfg_path) as cfg:
                ti_config = [line.rstrip("\r\n") for line in cfg if line[0][0]!="%"]
            # print(ti_config)
            for i in ti_config:
                self.command_port.write((i+"\n").encode())
                time.sleep(0.01)
            print("Configuration Done!")
        except:
            print("cfg file wrong...")
            self.close()
            return None
        for i in ti_config:
            split_words = i.split(" ")
            if not self.__found_rate and "frameCfg" in split_words[0]:
                self._ms_per_frame = float(split_words[5])
                print("Found frameCfg, milliseconds per frame is ", self._ms_per_frame)
                self.__found_rate = 1
                break
        if not self.__found_rate:
            print("cfg parameters wrong")
            self.close()
            return None
    
    # https://dev.ti.com/tirex/explore/node?node=A__ADnbI7zK9bSRgZqeAxprvQ__radar_toolbox__1AslXXD__LATEST#:~:text=Understanding%20the%20UART%20Data%20Output%20Format,-To%20understand%20the&text=When%20the%20chirp%20returns%20after,on%20the%20demo%20being%20run.
    def __parse_data(self, buffer):
            start = buffer.index(self.__magicWord)+28
            # print("start")
            (num_points, num_tlvs, num_subframes), i = self.unpack(buffer, i=start, amount=3, data_type='I')
            print(num_points, num_tlvs)
            ####  tvl1  ####
            (tlv_type, tlv_length), i = self.unpack(buffer, i, amount=2, data_type='I')
            # print(tlv_type)
            if num_points!=int(tlv_length/16):
                print("NUMBER OF DETECTED POINTS IS NOT CORRECT")
                return None    
            data=np.zeros((num_points,5),dtype=float)
            if(int(tlv_type)==1):
                for j in range(num_points):
                    try:
                        ( x, y, z, vel), i = self.unpack(buffer, i, amount=4, data_type='f')
                        data[j][0]=y
                        data[j][1]=-x
                        data[j][2]=z
                        data[j][4]=vel
                    except:
                        print("xyz fails")
            else:
                print("OUTPUT_MSG_DETECTED_POINTS WRONG")
                return None
            ####  tvl2  ####
            ## For SDK 3.x, intensity is replaced by snr in sideInfo and is parsed in the READ_SIDE_INFO code
            (tlv_type, tlv_length), i = self.unpack(buffer, i, amount=2, data_type='I')
            if(int(tlv_type)==7):
                for j in range(num_points):
                    try:
                        ( snr, noise ), i = self.unpack(buffer, i, amount=2, data_type='h')
                        data[j][3]=float(snr/10.0)
                    except:
                        print("snr fais")
            else:
                print("OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO WRONG")
                return None
            return data

    def read(self):
        while time.time()<(self.__start_time+self.__timeout):
            self.__buffer_temp += self.data_port.read_all()
            if(len(self.__buffer_temp)!=0):
                try:
                    idx_start = self.__buffer_temp.index(self.__magicWord)
                    idx_end = self.__buffer_temp.index(self.__magicWord, idx_start+1)
                except:
                    continue
                print(idx_start, idx_end)
                # print(self.__buffer_temp[idx_start:idx_end])
                msg = self.__parse_data(self.__buffer_temp[idx_start:idx_end])
                # print(msg.shape)
                self.reset_timer()
                self.clear_buffer()
                yield msg
            else:
                time.sleep(self._ms_per_frame/1000)  
        self.close()
        
    @staticmethod
    def unpack(buffer, i, order="<", amount=1, data_type="I"):
        data_size = {'h': 2, 'I': 4, 'f': 4}
        try:
            parsed_data = struct.unpack(order+str(amount)+data_type, buffer[i:(i+data_size[data_type]*amount)])
            if(len(parsed_data)==1):
                parsed_data = parsed_data[0]
            return parsed_data, (i+data_size[data_type]*amount)
        except:
            print("Unpacking " + data_type + " data failed")
            return None

    def reset_timer(self):
        self.__start_time = time.time()
    
    def clear_buffer(self):
        self.__buffer_temp = b''
        self.is_clearing = True

    def close(self):
        self.command_port.write('sensorStop\n'.encode())
        self.command_port.close()
        self.data_port.close()
        print("sensor shut down")   