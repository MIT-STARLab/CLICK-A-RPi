#!/usr/bin/env python
import sys
sys.path.append('/root/lib/')
import zmq
import struct
import options
import ipc_helper
import fpga_map


class FPGABusBase:
    def __init__(self): pass
    
    def read_register(self, addr):
        return self.transfer( (addr,), (0,), (0,) )[0][0]
        
    def write_register(self, addr, value):
        return self.transfer( (addr,), (1,), (value,) )[0][0]

class SPIBus(FPGABusBase):
    def __init__(self):
        FPGABus.__init__(self)
        import spidev
        
        self.spi = spidev.SpiDev()
           
    def transfer(self, addr_in, is_write, values):
    
        # Prepare data
        iter_input = zip(addr_in, is_write, values)
        def enc(addr, w_f, val): return [(addr & 0x7F) | ((w_f & 1) << 7), (val & 0xFF)]
        data_in = sum([enc(addr, w_f, val) for addr, w_f, val in iter_input])
        
        # Run transfer
        self.spi.open(1, 0)
        data_out = self.spi.xfer(data_in+[0,0], options.SPI_FREQ)
        self.spi.close()
        
        # Check return addresses
        errors = [ain == aout for ain, aout in zip(data_in[0::2], data_out[0::2])]
        
        values_out = data_out[1::2]
        return errors,values_out
        
    def verify_boot():
        addr = 0
        value = self.transfer( (addr,), (0,), (0,) )[0][0]
        addr_error = self.transfer( (addr,), (0,), (0,) )[1][0]
        dcm_locked = value & 1
        crc_err = (value >> 4) & 1
        if addr_error == 0 and dcm_locked == 1 and crc_err == 0: return True
        return False
        
class USBBus(FPGABusBase):
    def __init__(self):
        FPGABus.__init__(self)
        import fl  

        fl.flInitialise(0)
        self.handle = fl.flOpen(options.USB_DEVICE_ID)
        fl.flSelectConduit(self.handle, 1)
        
    def transfer(self, addr_in, is_write, values):
        
        # Prepare data
        iter_input = zip(addr_in, is_write, values)
        
        # Run transfer
        values_out = []
        for addr, w_f, val in iter_input:
            if w_f:
                fl.flWriteChannel(self.handle, addr, byte[num])
                values_out.append(None)
            else:
                values_out.append(fl.flReadChannel(self.handle, addr))
        
        #no error check for USB
        errors = [0]*len(addr_in)
        
        return errors,values_out
        
    def verify_boot():
        # LOL nope
        return True

def loop():

    #ZMQ i/f. Keep timout low to update ZMQ internal logic before clients time-out.
    ipc_server = ipc_helper.FPGAServerInterface(timeout=100)

    #FPGA i/f
    if options.USE_SPI: fpga_bus = SPIBus()
    else: fpga_bus = USBBus()
    
    # Check if the FPGA is alive
    assert fpga_bus.verify_boot()

    while 1:
    
        try:
            req = ipc_server.get_request()
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN: continue
            
        register_number = req.size//4
        addresses = range(req.start_addr, req.start_addr+register_number)
        
        if req.start_addr < 128:
            # Raw registers
            write_flag = [req.rw_flag]*register_number
            if req.rw_flag:
                values_in = struct.unpack('%dI' % register_number, req.write_data)
            else:
                values_in = [0]*register_number
                
            errors,values_out = fpga_bus.transfer(addresses, write_flag, values)
            
            error_flag = sum(errors)
            data_out = struct.pack('%dI' % register_number, *values_out)
            
            req.answer(data_out,error_flag)
            
        else:
            # Virtual registers
            # TO BE IMPLEMENTED
            
            values_out = []
            for addr in addresses:
            
                if addr in fpga_map.TEMPERATURE_BLOCK:
                    msb = fpga_map.BYTE_1[addr]
                    lsb = fpga_map.BYTE_2[addr]
                    temp = fpga_map.decode_temperature(msb.lsb)
                    values_out.append(struct.pack('f',temp))
                
                elif addr in fpga_map.CURRENT_BLOCK:
                    pass
               
                else:
                    req.answer(''.join(values_out),error_flag=1)
            
            
        
loop()
        
'''


import binascii
import math
import argparse
import struct
import csv
from datetime import datetime


from options import FPGA_MAP_ANSWER_PORT, FPGA_MAP_REQUEST_PORT
from fpga_map import REGISTERS
from ipc_packets import FPGAMapRequestPacket, FPGAMapAnswerPacket
from zmqTxRx import recv_zmq, send_zmq

import fl
from node import NodeFPGA
import memorymap
import fsm
import edfa
import alignment

DEBUG = False
memMap = memorymap.MemoryMap()

""" FPGA initialization """
conduit = 1
vp = "1d50:602b:0002"

try:
    fl.flInitialise(0)
    handle = fl.flOpen(vp)
    fl.flSelectConduit(handle, conduit)

    """ ZMQ inter process communication initialization """

    context = zmq.Context()

    socket_FPGA_map_request = context.socket(zmq.SUB)
    socket_FPGA_map_request.bind("tcp://*:%s" % FPGA_MAP_REQUEST_PORT)

    socket_FPGA_map_answer = context.socket(zmq.PUB)
    socket_FPGA_map_answer.bind("tcp://*:%s" % FPGA_MAP_ANSWER_PORT)

    # socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
    # subscribe to ALL incoming FPGA_map_requests
    socket_FPGA_map_request.setsockopt(zmq.SUBSCRIBE, b'')

    # socket needs some time to set up. give it a second - else the first message will be lost
    time.sleep(1)

    print ("\n")

    while True:

        # wait for a package to arrive
        print ('RECEIVING on %s with TIMEOUT %d' % (socket_FPGA_map_request.get_string(zmq.LAST_ENDPOINT), socket_FPGA_map_request.get(zmq.RCVTIMEO)))
        message = recv_zmq(socket_FPGA_map_request)

        # decode the package
        ipc_fpgarqpacket = FPGAMapRequestPacket()
        ipc_fpgarqpacket.decode(message)
        print (ipc_fpgarqpacket)


        if ipc_fpgarqpacket.rw_flag == 1:
            print ('| got FPGA_MAP_REQUEST_PACKET with WRITE in ENVELOPE %d' % (ipc_fpgarqpacket.return_addr))

            print(ipc_fpgarqpacket.decoded_data)

            """ write FPGA memory map """
            if(isCommCapable):
                count = -1
                for reg in range(ipc_fpgarqpacket.start_addr, ipc_fpgarqpacket.start_addr + ipc_fpgarqpacket.size): #from start_addr to start_addr+size

                    count += 1
                    print('got %d for register %d' % (ipc_fpgarqpacket.decoded_data[count], reg))

                    if REGISTERS[reg] is not None and REGISTERS[reg][2] is True: #if the requested register is defined and writeable
                        bytes = struct.pack(REGISTERS[reg][0], ipc_fpgarqpacket.decoded_data[count])
                        byte = [int(ord(c)) for c in bytes]
                        num = -1
                        for register in REGISTERS[reg][1]: #for each physical register in register definition
                            num += 1
                            if REGISTERS[reg][0] == '?': #if data is bool type
                                if byte[num] == 1: #if True, set to 0x55
                                    byte[num] = 85
                                else:
                                    byte[num] = 15
                            print('writing %d to channel %d' % (byte[num], register))
                            fl.flWriteChannel(handle, register, byte[num])
                    else:
                        print('Register undefined or read-only')

            else:
                print('!isCommCapable')
            """                    """

            # send the FPGA_map_answer packet (write)
            ipc_fpgaaswpacket_write = FPGAMapAnswerPacket()
            raw = ipc_fpgaaswpacket_write.encode(return_addr=ipc_fpgarqpacket.return_addr, rq_number=ipc_fpgarqpacket.rq_number, rw_flag=1, error_flag=0, start_addr=ipc_fpgarqpacket.start_addr, size=ipc_fpgarqpacket.size)
            ipc_fpgaaswpacket_write.decode(raw)
            print ('SENDING to %s with ENVELOPE %d' % (socket_FPGA_map_answer.get_string(zmq.LAST_ENDPOINT), ipc_fpgaaswpacket_write.return_addr))
            #print(b'| ' + raw)
            print(ipc_fpgaaswpacket_write)
            socket_FPGA_map_answer.send(raw)

        else:
            print ('| got FPGA_MAP_REQUEST_PACKET with READ in ENVELOPE %d' % (ipc_fpgarqpacket.return_addr))

            mapData = b''

            """ read FPGA memory map """
            if(isCommCapable):
                for reg in range(ipc_fpgarqpacket.start_addr, ipc_fpgarqpacket.start_addr + ipc_fpgarqpacket.size): #from start_addr to start_addr+size

                    if REGISTERS[reg] is not None: #if the requested register is defined
                        bytes = ''
                        count = 0
                        for register in REGISTERS[reg][1]: #for each physical register in register definition
                            count += 1
                            num = fl.flReadChannel(handle, register)
                            byte = struct.pack('B', num) #convert int to byte (unsigned char)
                            derp = 'Register '+str(register)+' = '+str(num)+' (raw='+byte+', hex='+binascii.hexlify(byte)+')'
                            print(derp)
                            bytes = bytes + byte
                        if REGISTERS[reg][0] == 'I': #check if requested datatype is Integer
                            for x in range(4-count): #Integer has 4 bytes - add zeros if less than 4 bytes retrieved
                                bytes = bytes + struct.pack('B', 0)
                        if REGISTERS[reg][0] == '?': #if data is bool type
                            if bytes == b'\x55': #this is the FPGA map value for true, so encode a 01
                                bytes = b'\x01'
                            else: #if not true, encode a 00
                                bytes = b'\x00'
                        #print(binascii.hexlify(bytes))
                        mapData = mapData + bytes
                    else:
                        print('Register undefined')
                        mapData = mapData + struct.pack('B', 0)

            else:
                print('!isCommCapable')
            """                    """


            # send the FPGA_map_answer packet (read)
            ipc_fpgaaswpacket_read = FPGAMapAnswerPacket()
            raw = ipc_fpgaaswpacket_read.encode(return_addr=ipc_fpgarqpacket.return_addr, rq_number=ipc_fpgarqpacket.rq_number, rw_flag=0, error_flag=0, start_addr=ipc_fpgarqpacket.start_addr, size=ipc_fpgarqpacket.size, read_data=mapData)
            ipc_fpgaaswpacket_read.decode(raw)
            print ('SENDING to %s with ENVELOPE %d' % (socket_FPGA_map_answer.get_string(zmq.LAST_ENDPOINT), ipc_fpgaaswpacket_read.return_addr))
            print(b'| ' + raw)
            print(ipc_fpgaaswpacket_read)
            socket_FPGA_map_answer.send(raw)



except fl.FLException as ex:
    print(ex)
finally:
    fl.flClose(handle)
'''