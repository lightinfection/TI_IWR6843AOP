import os
import sys
import time
import serial
import struct
import math
import warnings
import numpy as np

LIGHT_SPEED = 299792458
RX = 4
TX = 3

class get_data:
    def __init__(self, command_port="", data_port="", command_rate="115200", data_rate="921600", cfg_path="", connection=True, timeout=5):
        self.cfg_path = cfg_path
        self.connected = False
        self.config_params = {}
        self.ms_per_frame = 1000
        self.output_heat_map = False
        self._numVirtualAnt = TX*RX
        self.__found_key = -1
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
            if self.__found_key > 0: break
            if "profileCfg" in split_words[0]:
                startFreq = float(split_words[2])
                idleTime = float(split_words[3])
                adcStartTime = float(split_words[4])
                rampEndTime = float(split_words[5])
                freqSlopeConst = float(split_words[8])
                numADCsample = int(split_words[10])
                digOutSampleRate = float(split_words[11])
                self.__found_key+=1
            if "frameCfg" in split_words[0]:
                ntx = int(split_words[2]) - int(split_words[1]) + 1
                numChirploop = int(split_words[3])
                self._ms_per_frame = float(split_words[5])
                self.__found_key+=1
        if self.__found_key <= 0:
            print("cfg parameters wrong")
            self.close()
            return None
        ADCDuration = numADCsample / (digOutSampleRate * 1e3)
        BW = ADCDuration * freqSlopeConst * 1e12
        PRI = (idleTime + rampEndTime) * 1e-6
        self._range_resolution = LIGHT_SPEED / (2*BW)
        self._range_max = self._range_resolution * numADCsample
        self._vel_max = LIGHT_SPEED / (2 * (startFreq * 1e9 + (freqSlopeConst * 1e12) * (adcStartTime * 1e-6 + ADCDuration / 2)) * PRI) / ntx
        self._vel_abs_max = self._vel_max / 2
        self._vel_resolution = self._vel_max / numChirploop
        # print(self._range_max, self._range_resolution, self._vel_abs_max, self._vel_resolution)
        self._rangeFFTSize = pow(2, math.ceil(math.log2(numADCsample)))
        self._rangeDopplerSize = pow(2, math.ceil(math.log2(numChirploop)))
    
    # https://dev.ti.com/tirex/explore/content/radar_toolbox_2_00_00_06/docs/software_guides/Understanding_UART_Data_Output_Format.html
    def __parse_data(self, buffer):
        start = buffer.index(self.__magicWord)+28  # not suited for official people counting binary
        (num_points, num_tlvs, num_subframes), i = self.unpack(buffer, i=start, amount=3, data_type='I')
        # print(len(buffer), num_points, num_tlvs)
        ####  tlv1  ####
        (tlv_type, tlv_length), i = self.unpack(buffer, i, amount=2, data_type='I')
        if num_points!=int(tlv_length/16):
            print("NUMBER OF DETECTED POINTS IS NOT CORRECT")
            return None    
        data=np.zeros((num_points,5),dtype=float)
        if tlv_type == 1:
            for j in range(num_points):
                try:
                    ( x, y, z, vel), i = self.unpack(buffer, i, amount=4, data_type='f')
                    data[j][0]=y
                    data[j][1]=-x
                    data[j][2]=z
                    data[j][4]=vel
                except:
                    print("xyz failed")
        else:
            print("OUTPUT_MSG_DETECTED_POINTS WRONG")
            return None
        ####  tlv7  ####
        ## For SDK 3.x, intensity is replaced by snr in sideInfo and is parsed in the READ_SIDE_INFO code
        (tlv_type, tlv_length), i = self.unpack(buffer, i, amount=2, data_type='I')
        if tlv_type == 7:
            for j in range(num_points):
                try:
                    ( snr, noise ), i = self.unpack(buffer, i, amount=2, data_type='H')
                    data[j][3]=float(snr/10.0)
                except:
                    print("snr failed")
        else:
            print("OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO WRONG")
            return None
        # print(i)
        if num_tlvs > 2:
            remnant = len(buffer) - 40 - num_points*20
            # print("remnant: ", remnant)
            self.output_heat_map = True
            RA_Mat_F = np.zeros(self._rangeFFTSize*self._numVirtualAnt)
            RA_Mat_R = np.zeros(self._rangeFFTSize*self._numVirtualAnt)
            RD_Mat = np.zeros(self._rangeFFTSize*self._rangeDopplerSize)
            for extra_tlv in range(num_tlvs-2):
                if extra_tlv == 0:
                    (tlv_type, tlv_length), i = self.unpack(buffer, i, amount=2, data_type='I')
                    remnant -= 8
                else:  ##### total length of a frame must be multiple of 32 bytes, so that the lenght of extra tlv data debiates from calculation
                    explore_times = 0
                    explore_success = False
                    while not explore_success:
                        (tlv_type, tlv_length), i = self.unpack(buffer, i, amount=2, data_type='I')
                        explore_success = True if tlv_type == 5 else False
                        remnant -= 8
                        explore_times += 1
                        if explore_times > 16:
                            print("Extra TLV DATA FORMAT WRONG")
                            break
                # print(tlv_type, tlv_length, remnant)
                try:
                    ####  tlv8  ####
                    if tlv_type == 8:
                        if remnant >= tlv_length + (num_tlvs - 3)*(8 + self._rangeDopplerSize * self._rangeFFTSize):
                            for j in range(self._rangeFFTSize*self._numVirtualAnt):
                                (imag, real), i = self.unpack(buffer, i, amount=2, data_type='h')
                                RA_Mat_F[j] = imag
                                RA_Mat_R[j] = real
                                remnant -= 4
                        else: warnings.warn("OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP WRONG")
                    ###  tlv5  ####
                    elif tlv_type == 5:
                        if remnant >= tlv_length:
                            for j in range(self._rangeFFTSize*self._rangeDopplerSize):
                                X, i = self.unpack(buffer, i, amount=1, data_type='h')
                                RD_Mat[j] = X
                                remnant -= 2
                        else: warnings.warn("OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP WRONG")
                    else: warnings.warn('TLV {} not supported yet, waiting one more time...'.format(tlv_type))       
                except Exception as e:
                    print("Parsing TLV" + str(tlv_type) + " failed.\n" + e)
            return data, RA_Mat_F, RA_Mat_R, RD_Mat
        else:
            return data

    def read(self):
        while time.time()<(self.__start_time+self.__timeout):
            self.__buffer_temp += self.data_port.read_all()
            if(len(self.__buffer_temp)!=0):
                try:
                    idx_start = self.__buffer_temp.index(self.__magicWord)
                    idx_end = self.__buffer_temp.index(self.__magicWord, idx_start+1)
                    # print(idx_start, idx_end)
                except:
                    continue
                msg = self.__parse_data(self.__buffer_temp[idx_start:idx_end])
                self.reset_timer()
                self.clear_buffer()
                yield msg
            else:
                time.sleep(self._ms_per_frame/1000)
        self.close()
        
    @staticmethod
    def unpack(buffer, i, order="<", amount=1, data_type="I"):
        data_size = {'h': 2, 'H': 2, 'I': 4, 'f': 4}
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
        if self.command_port.is_open:
            self.command_port.write('sensorStop\n'.encode())
            self.command_port.close()
        if self.data_port.is_open: self.data_port.close()
        print("sensor shut down")   