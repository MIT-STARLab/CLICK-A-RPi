#!/usr/bin/env python

import sys
import zmq
import time
import binascii

import math
import argparse
import time
import struct
import csv
from datetime import datetime

import sys #importing options and functions
sys.path.append('../lib/')
sys.path.append('/root/lib/')
from options import FPGA_MAP_ANSWER_PORT, FPGA_MAP_REQUEST_PORT
from fpga_map import REGISTERS
from ipc_packets import FPGAMapRequestPacket, FPGAMapAnswerPacket
from zmqTxRx import recv_zmq, send_zmq

sys.path.append('driver/')
import fl
from node import NodeFPGA
import memorymap
import fsm
import edfa
import alignment


DEBUG = False
memMap = memorymap.MemoryMap()



""" FPGA initialization """

handle = fl.FLHandle()

try:
    fl.flInitialise(0)
    print("here")
    vp = "1d50:602b:0002"
    print("Attempting to open connection to FPGALink device {}...".format(vp))
    try:
        handle = fl.flOpen(vp)
	print("Made it")
    except fl.FLException as ex:
        ivp = "04b4:8613" #TODO: needs root?
        #ivp = '0424:2422' #TODO: why is this different? (USB ADDRESS)
        print("Loading firmware into {}...".format(ivp))
        fl.flLoadStandardFirmware(ivp, vp)
        print type(ivp)
        print type(vp)
        # Long delay for renumeration
        # TODO: fix this hack.  The timeout value specified in flAwaitDevice() below doesn't seem to work
        time.sleep(3)

        print("Awaiting renumeration...")
        if ( not fl.flAwaitDevice(vp, 10000) ):
            raise fl.FLException("FPGALink device did not renumerate properly as {}".format(vp))

        print("Attempting to open connection to FPGALink device {} again...".format(vp))
        handle = fl.flOpen(vp)

    time.sleep(2)
    conduit = 1

    isNeroCapable = fl.flIsNeroCapable(handle)
    isCommCapable = fl.flIsCommCapable(handle, conduit)
    fl.flSelectConduit(handle, conduit)
    #time.sleep(2)
    progConfig = "J:A7A0A3A1:/root/bin/protected_output_2.xsvf"
    print("Programming device with config {}...".format(progConfig))
    if ( isNeroCapable ):
        fl.flProgram(handle, progConfig)
    else:
        raise fl.FLException("Device program requested but device at {} does not support NeroProg".format(vp))

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

            time.sleep(1)

            # send the FPGA_map_answer packet (write)
            ipc_fpgaaswpacket_write = FPGAMapAnswerPacket()
            raw = ipc_fpgaaswpacket_write.encode(return_addr=ipc_fpgarqpacket.return_addr, rq_number=0, rw_flag=1, error_flag=0, start_addr=0xDEF0, size=0)
            ipc_fpgaaswpacket_write.decode(raw)
            print ('SENDING to %s with ENVELOPE %d' % (socket_FPGA_map_answer.get_string(zmq.LAST_ENDPOINT), ipc_fpgaaswpacket_write.return_addr))
            #print(b'| ' + raw)
            print(ipc_fpgaaswpacket_write)
            send_zmq(socket_FPGA_map_answer, raw, ipc_fpgaaswpacket_write.return_addr)

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
            raw = ipc_fpgaaswpacket_read.encode(return_addr=ipc_fpgarqpacket.return_addr, rq_number=0, rw_flag=0, error_flag=0, start_addr=ipc_fpgarqpacket.start_addr, size=ipc_fpgarqpacket.size, read_data=mapData)
            ipc_fpgaaswpacket_read.decode(raw)
            print ('SENDING to %s with ENVELOPE %d' % (socket_FPGA_map_answer.get_string(zmq.LAST_ENDPOINT), ipc_fpgaaswpacket_read.return_addr))
            print(b'| ' + raw)
            print(ipc_fpgaaswpacket_read)
            send_zmq(socket_FPGA_map_answer, raw, ipc_fpgaaswpacket_read.return_addr)






except fl.FLException as ex:
    print(ex)
finally:
    fl.flClose(handle)






