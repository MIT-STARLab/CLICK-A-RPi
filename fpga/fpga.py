#!/usr/bin/env python
import sys
sys.path.append('/root/lib/')
import zmq
import struct
import options
import ipc_helper
import fpga_map as mmap
import fpga_bus
import edfa
import dac
import time

def loop():

    #ZMQ i/f. Keep timout low to update ZMQ internal logic before clients time-out.
    ipc_server = ipc_helper.FPGAServerInterface(timeout=100)

    #FPGA i/f
    if options.IPC_USES_SPI: fpgabus = fpga_bus.SPIBus()
    else: fpgabus = fpga_bus.USBBus()
    
    # Check if the FPGA is alive
    if options.CHECK_ASSERTS: assert fpgabus.verify_boot()
    
    # Buffer for virtual regs
    reg_buffer = {}
    last_edfa_update = 0

    while 1:
    
        try:
            req = ipc_server.get_request()
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN: continue
            
        register_number = req.size//4
        addresses = range(req.start_addr, req.start_addr+register_number)
        write_flag = [req.rw_flag]*register_number
        if req.rw_flag and register_number:
            values_in = []
            for addr,idx in zip(addresses,xrange(0,req.size,4)):
                try: frmt = mmap.REGISTER_TYPE[addr]
                except: frmt = 'xxxB'
                pl = req.write_data[idx,idx+4]
                values_in.append(struct.unpack(frmt,pl)[0])
        else:
            values_in = [0]*register_number
        
        
        if req.start_addr < 128:
                
            errors,values_out = fpgabus.transfer(addresses, write_flag, values_in)
            
            error_flag = sum(errors)
            req.answer(values_out,error_flag)
            
        elif req.start_addr == mmap.EDFA_IN_STR:
        
            edfa.reset_fifo(fpgabus)
            
            if options.CHECK_ASSERTS: assert req.size > 3
            
            len_str = struct.unpack('I', req.write_data[0:4])[0]
            
            if options.CHECK_ASSERTS: assert req.size >= (len_str+4)
            
            values_in = req.write_data[4:4+len_str]
            
            error_flag = edfa.write_string(fpgabus, values_in)
            req.answer('',error_flag)
            
        elif req.start_addr == mmap.EDFA_OUT_STR:
            
            error,edfa_out_str = edfa.read_string(fpgabus)
            req.answer(edfa_out_str,error)
            
        else:
            # Virtual registers
            # TO BE IMPLEMENTED
            
            # if any in the EDFA block, get FLINE
            if any([x in mmap.EDFA_PARSED_BLOCK for x in addresses]):
                if time.time() - last_edfa_update > options.EDFA_VIRTUAL_REGS_GOOD_FOR:
                    fline = edfa.fline(fpgabus)
                    flist = fline.split()          
                    reg_buffer = edfa.parse(reg_buffer, flist)
            
            values_out = []
            for addr,value in zip(addresses,values_in):
            
                if addr in mmap.TEMPERATURE_BLOCK:
                    msb = mmap.BYTE_1[addr]
                    lsb = mmap.BYTE_2[addr]
                    temp = mmap.decode_temperature(msb.lsb)
                    values_out.append(struct.pack('f',temp))
                
                elif addr in mmap.CURRENT_BLOCK:
                    pass
                    
                elif addr in mmap.EDFA_PARSED_BLOCK:
                    values_out.append(reg_buffer[addr])
                    
                elif addr ==  mmap.DAC_ENABLE:
                    dac.set_ouput_mode(fpgabus, value)
               
                elif addr in mmap.DAC_BLOCK:
                    if req.rw_flag:
                        target_chan = addr - mmap.DAC_BLOCK[0]
                        dac.write_and_update(fpgabus,target_chan,value)
                    values_out.append(value)
               
                else: req.answer('',error_flag=1)
                
                
            req.answer(values_out,error_flag)
        
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