#!/usr/bin/python
import sys
sys.path.append('/root/test/')
import math
import fpga_map as mmap
import time
from options import *

fpga = ipc_helper.FPGAClientInterface()
power = mmap.Power(fpga)
edfa = mmap.EDFA(fpga)
seed_setting = 1 #0 for flat_sat 1 for payload

class txPacket():

    def __init__(self, ppm_order, data, ascii=True):
        self.data = data
        self.ppm_order = ppm_order
        self.ascii = ascii
        self.symbols = []

    """
    Data to send to transmitter
    Provide the PPM ppm_order to use 4-128 and the data to be sent
    Resets the PPM order to 4 after each transmit

    ppm_order: Int
    Data: string or binary

    Returns: list of int's prepared for transmitter
    """
    def pack(self):
        setter = 6
        if(self.ascii): binary_data = ascii2bits(self.data)
        else: binary_data = self.data
        symbols = []
        symbols += [int(0b1 << 7 | int("0b" + "1"*(int(math.log(self.ppm_order)/math.log(2))),2))]

        if(self.ppm_order == 4 | self.ppm_order == 8 | self.ppm_order == 64): setter = 6
        elif(self.ppm_order == 16):  setter = 4
        elif(self.ppm_order == 32):  setter = 5
        elif(self.ppm_order == 128): setter = 7

        symbols += [int('0b'+ binary_data[x:x+setter].zfill(8),2) for x in range(0,len(binary_data),setter)]
        symbols += [int(0b1 << 7 | int("0b" + "1"*(int(math.log(self.ppm_order)/math.log(2))),2))]

        self.symbols = symbols

    def set_PPM(self, fpga):
        ppm_code = [int(0b1 << 7 | int("0b" + "1"*(int(math.log(self.ppm_order)/math.log(2))),2))]
        fpga.write_reg(mmap.DATA, ppm_code)
    """
    Takes FPGA Client object and sleep time
    Sleep is configurable for EDFA to settle to new PPM order
    Writes to FPGA FIFO 
    """
    def transmit(self, fpga, sleep_time=1):
        for i in xrange(len(self.symbols[:])):
            fpga.write_reg(mmap.DATA, self.symbols[i])
        time.sleep(sleep_time)


    def empty(self):
        self.data = []
        self.symbols = []

def ascii2bits(s):
    return(''.join([bin(ord(x))[2:].zfill(8) for x in s]))

def bits2ascii(s):
    return(''.join([chr(int(s[x:x+8],2)) for x in range(0,len(s),8)]))

def seed_align(default_settings, cw = False):

    power.edfa_on()
    power.bias_on()
    power.tec_on()

    tec_msb, tec_lsb, ld_msb, ld_lsb = default_settings
    total_tec = tec_msb*256 + tec_lsb

    for i in range(1,5):
        fpga.write_reg(i, default_settings[i-1])
    if(not cw):
        fpga.write_reg(mmap.DATA, 131)
    
    time.sleep(2)
    power_inputs = []
    window = 6
    avg = 3
    for i in range(total_tec-window, total_tec+window):
        tec_msb = i//256
        tec_lsb = i%256
        fpga.write_reg(mmap.LTSa, tec_msb)
        fpga.write_reg(mmap.LTSb, tec_lsb)
        time.sleep(.1)
        avg_input_power = sum([fpga.read_reg(mmap.EDFA_POWER_IN) for x in range(avg)])/avg
        power_inputs.append(avg_input_power)

    pwr_index = 256*tec_msb + tec_lsb
    for pwr in power_inputs:
        if PPM4_THRESHOLDS[0] < pwr < PPM4_THRESHOLDS[1] and not cw:
            pwr_index = pwr
        elif CW_THRESHOLDS[0] < pwr < CW_THRESHOLDS[1] and cw:
            pwr_index = pwr
    
    new_tec = total_tec+power_inputs.index(pwr_index)-window
    tec_msb = new_tec//256
    tec_lsb = new_tec%256
    fpga.write_reg(mmap.LTSa, tec_msb)
    fpga.write_reg(mmap.LTSb, tec_lsb)
    time.sleep(1)

    return tec_msb, tec_lsb


