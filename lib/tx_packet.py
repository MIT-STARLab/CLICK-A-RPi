#!/usr/bin/python
import sys
sys.path.append('/root/test/')
import math
import fpga_map as mmap
import time

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
        if(self.ppm_order == 4 | self.ppm_order == 8 | self.ppm_order == 64): setter = 6
        elif(self.ppm_order == 16):  setter = 4
        elif(self.ppm_order == 32):  setter = 5
        elif(self.ppm_order == 128): setter = 7

        symbols += [int('0b'+ binary_data[x:x+setter].zfill(8),2) for x in range(0,len(binary_data),setter)]
        # symbols += [int(0b1 << 7 | int("0b" + "1"*(int(math.log(4)/math.log(2))),2))]

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


