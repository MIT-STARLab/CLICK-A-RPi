#!/usr/bin/env python
import sys
import zmq
import time
import binascii
import math
import argparse
import struct
import csv
from datetime import datetime

sys.path.append('/root/lib/')
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

try:
    conduit = 1
    vp = "1d50:602b:0002"
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
