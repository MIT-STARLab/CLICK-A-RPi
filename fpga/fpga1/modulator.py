# Joe Kusters
# modulator.py
# helper functions for reading in a file and packing the bits to be sent to the modulator

import math
import fl
import memorymap as mem

# the downlink can be broken up into parts
# first, determine ppm order and create first byte
# second, read in binary data into an array
# third, pack this data based on the ppm order into an array
# then the downlink loop is two parts, first fill up the fifo, then do the handshake, fill up fifo, do handshake, etc. until the whole data set has been sent

class Modulator(object): 
    def __init__(self, ppm_order, handle):
        self.ppm_order = ppm_order
        self.memory = mem.MemoryMap()
        self.handle = handle
        self.data = []
        self.bytes = []
        self.ones = 0
        self.errors = 0

    def set_ppm_order(self, ppm_order):
        self.ppm_order = ppm_order

    def get_ppm_order(self):
        return self.ppm_order

    # method for performing a downlink data transfer to the modulator
    def downlink(self, FILE):
        M = int(self.ppm_order)>>2
        f = open(FILE, 'rb')
        self.bytes += [128+self.ppm_order] # create first byte (sets ppm order in FPGA), MSB tells FPGA whether this byte sets the ppm order (1) or not (0), other 7 bits are data

        # read file data into an array
        done = False
        while not done:
            char = f.read(1)
            if(char == ''):
                f.close()
                done = True
            else:
                self.data += [ord(char)]

        self.pack_data()
        self.fill_fifo()
        
        ones = 0
        errors = 0
        while(fl.flReadChannel(self.handle, self.memory.get_register('efp')) or fl.flReadChannel(self.handle, self.memory.get_register('fff'))&56):
            errors += fl.flReadChannel(self.handle, self.memory.get_register('err'))
        while(fl.flReadChannel(self.handle, self.memory.get_register('ofp')) or fl.flReadChannel(self.handle, self.memory.get_register('fff'))&196):
            ones += fl.flReadChannel(self.handle, self.memory.get_register('one'))
        #    print('Ones FIFO Pointer = '+str(fl.flReadChannel(handle, memMap.get_register('ofp'))))
        print('errors = '+str(errors))

    # method for packing the data bytes into the appropriate sizes for the modulator
    def pack_data(self):
        i = 0
        while(i<len(self.data)):
            byte = 0
            if(self.ppm_order < 4):
                for j in range(0, 3):
                    byte += self.data[i+j]<<(8*j)
                for j in range(0, 4):
                    self.bytes += [(byte&(0x1F<<(6*j)))>>6*j]
                i+=3
            else:
                for j in range(0, self.ppm_order):
                    byte += self.data[i+j]<<(8*j)
                for j in range(0,8):
                    self.bytes += [(byte&(0x1F<<(self.ppm_order*j)))>>self.ppm_order*j]
                i+=self.ppm_order

    # method for filling the modulator fifo until the all of the data bytes have been sent to it
    def fill_fifo(self):
        count = 0
        end = 2048
        while(count != len(self.bytes)):
            fifo_space = 2048-((0x07&fl.flReadChannel(self.handle, self.memory.get_register('fff'))<<8)+fl.flReadChannel(self.handle, self.memory.get_register('dfp')))
            if(fifo_space>0):
                end+=fifo_space
                if(end>len(self.bytes)):
                    end = len(self.bytes)
                fl.flWriteChannel(self.handle, memMap.get_register('dat'), bytearray(self.bytes[count:end]))
                print('Data FIFO Pointer = '+str(fl.flReadChannel(self.handle, self.memory.get_register('dfp'))))
                count = end

