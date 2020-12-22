#!/usr/bin/env python

import sys
import zmq
import json
import time

import sys #importing options and functions
sys.path.append('../lib/')
import fpga_map.h
import fl

from options import FPGA_MAP_ANSWER_PORT, FPGA_MAP_REQUEST_PORT
from ipc_packets import FPGAMapRequestPacket, FPGAMapAnswerPacket
from zmqTxRx import recv_zmq, send_zmq
from node import NodeFPGA


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

handle = fl.FLHandle()
isNeroCapable = fl.flIsNeroCapable(handle)
isCommCapable = fl.flIsCommCapable(handle, conduit)



# https://stackoverflow.com/questions/25188792/how-can-i-use-send-json-with-pyzmq-pub-sub
# How can I use send_json with pyzmq PUB SUB

while True:

    # wait for a package to arrive
    print ('RECEIVING on %s with TIMEOUT %d' % (socket_FPGA_map_request.get_string(zmq.LAST_ENDPOINT), socket_FPGA_map_request.get(zmq.RCVTIMEO)))
    message = recv_zmq(socket_FPGA_map_request)

    # decode the package
    ipc_fpgarqpacket = FPGAMapRequestPacket()
    return_addr, rq_number, rw_flag, start_addr, size, write_data,len(write_data) = ipc_fpgarqpacket.decode(message)
    #print (ipc_fpgarqpacket)



    if ipc_fpgarqpacket.rw_flag == 1:
        print ('| got FPGA_MAP_REQUEST_PACKET with WRITE in ENVELOPE %d' % (ipc_fpgarqpacket.return_addr))
        time.sleep(1)
        # send the FPGA_map_answer packet (write)
        data = []

        if(star_addr == TEMPERATURE_BLOCK and isCommCapable):
            for i in range(size):
                data.append(fl.flReadChannel(handle,start_addr + i))
            

        ipc_fpgaaswpacket_write = FPGAMapAnswerPacket()
        raw = ipc_fpgaaswpacket_write.encode(return_addr=ipc_fpgarqpacket.return_addr, rq_number=123, rw_flag=1, error_flag=0, start_addr=start_addr, size=size, data)
        ipc_fpgaaswpacket_write.decode(raw)
        print ('SENDING to %s with ENVELOPE %d' % (socket_FPGA_map_answer.get_string(zmq.LAST_ENDPOINT), ipc_fpgaaswpacket_write.return_addr))
        print(b'| ' + raw)
        print(ipc_fpgaaswpacket_write)
        send_zmq(socket_FPGA_map_answer, raw, ipc_fpgaaswpacket_write.return_addr)

    else:
        print ('| got FPGA_MAP_REQUEST_PACKET with READ in ENVELOPE %d' % (ipc_fpgarqpacket.return_addr))
        time.sleep(1)
        # send the FPGA_map_answer packet (read)
        ipc_fpgaaswpacket_read = FPGAMapAnswerPacket()
        raw = ipc_fpgaaswpacket_read.encode(return_addr=ipc_fpgarqpacket.return_addr, rq_number=123, rw_flag=0, error_flag=0, start_addr=0x9ABC, size=16, read_data=b"I'm Mr. Meeseeks")
        ipc_fpgaaswpacket_read.decode(raw)
        print ('SENDING to %s with ENVELOPE %d' % (socket_FPGA_map_answer.get_string(zmq.LAST_ENDPOINT), ipc_fpgaaswpacket_read.return_addr))
        print(b'| ' + raw)
        print(ipc_fpgaaswpacket_read)
        send_zmq(socket_FPGA_map_answer, raw, ipc_fpgaaswpacket_read.return_addr)



def mogrify(topic, msg):
    """ json encode the message and prepend the topic """
    return (topic + ' ' + json.dumps(msg)).encode('ascii')

def demogrify(topicmsg):
    """ Inverse of mogrify() """
    topicmsg = topicmsg.decode('ascii')
    json0 = topicmsg.find('{')
    topic = topicmsg[0:json0].strip()
    msg = json.loads(topicmsg[json0:])
    return topic, msg