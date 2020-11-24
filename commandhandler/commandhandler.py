#!/usr/bin/env python3

import zmq
import random
import sys
import os
import time
import json
import pickle
import sys

#importing options and functions
sys.path.append('../lib/')
sys.path.append('/home/pi/CLICK-A/github/lib/')
from options import FPGA_MAP_ANSWER_PORT, FPGA_MAP_REQUEST_PORT, TX_PACKETS_PORT, RX_CMD_PACKETS_PORT, MESSAGE_TIMEOUT, PAT_CONTROL_PORT, TEST_RESPONSE_PORT
from ipc_packets import FPGAMapRequestPacket, FPGAMapAnswerPacket, TxPacket, RxCommandPacket
from zmqTxRx import recv_zmq, separate

# use PID as unique identifier for this progress
topic = str(os.getpid())
pid = os.getpid()

# ZeroMQ inter process communication
context = zmq.Context()

socket_FPGA_map_request = context.socket(zmq.PUB) #send messages on this port
socket_FPGA_map_request.connect("tcp://localhost:%s" % FPGA_MAP_REQUEST_PORT) #connect to specific address (localhost)

socket_test_response_packets = context.socket(zmq.PUB) #send messages on this port
socket_test_response_packets.connect("tcp://localhost:%s" % TEST_RESPONSE_PORT) #connect to specific address (localhost)

socket_pat_control = context.socket(zmq.PUB) #send messages on this port
socket_pat_control.connect("tcp://localhost:%s" % PAT_CONTROL_PORT) #connect to specific address (localhost)

print ("Subscribing to FPGA_MAP_ANSWER topic {}".format(topic))
print ("on port {}".format(FPGA_MAP_ANSWER_PORT))
socket_FPGA_map_answer = context.socket(zmq.SUB)
socket_FPGA_map_answer.setsockopt(zmq.SUBSCRIBE, topic.encode('ascii'))
socket_FPGA_map_answer.setsockopt(zmq.RCVTIMEO, MESSAGE_TIMEOUT) # 5 second timeout on receive
socket_FPGA_map_answer.connect ("tcp://localhost:%s" % FPGA_MAP_ANSWER_PORT)

socket_tx_packets = context.socket(zmq.PUB)
socket_tx_packets.connect ("tcp://localhost:%s" % TX_PACKETS_PORT)

print ("Pulling RX_CMD_PACKETS")
print ("on port {}".format(RX_CMD_PACKETS_PORT))
socket_rx_command_packets = context.socket(zmq.SUB)
socket_rx_command_packets.setsockopt(zmq.SUBSCRIBE, b'')
socket_rx_command_packets.connect ("tcp://127.0.0.1:%s" % RX_CMD_PACKETS_PORT)

# socket needs some time to set up. give it a second - else the first message will be lost
time.sleep(1)

def getFPGAmap():
    ipc_fpgarqpacket_read = FPGAMapRequestPacket()

    #create bytes object (struct) for reading
    '''do we need error handling for struct generation?'''
    raw = ipc_fpgarqpacket_read.encode(return_addr=pid, rq_number=123, rw_flag=0, start_addr=0x5678, size=48)
    ipc_fpgarqpacket_read.decode(raw)

    print ('SENDING to %s with ENVELOPE %d' % (socket_rx_command_packets.get_string(zmq.LAST_ENDPOINT), ipc_fpgarqpacket_read.return_addr))
    print(raw)
    print(ipc_fpgarqpacket_read)

    #send message
    socket_FPGA_map_request.send(raw)

    print ('RECEIVING on %s with TIMEOUT %d for ENVELOPE %d' % (socket_FPGA_map_answer.get_string(zmq.LAST_ENDPOINT), socket_FPGA_map_answer.get(zmq.RCVTIMEO), ipc_fpgarqpacket_read.return_addr))
    message, envelope = separate(recv_zmq(socket_FPGA_map_answer))

    print (b'| found ENVELOPE: ' + envelope)
    print (b'| ' + message)

    # decode the package
    ipc_fpgaaswpacket = FPGAMapAnswerPacket()
    ipc_fpgaaswpacket.decode(message)
    print (ipc_fpgaaswpacket)
    print ('| got PAYLOAD %s' % (ipc_fpgaaswpacket.read_data))

    # sending read_data back as TX packet
    if ipc_fpgaaswpacket.read_data:
        not_empty_ipc_txpacket = TxPacket()
        raw = not_empty_ipc_txpacket.encode(APID=0x02,payload=ipc_fpgaaswpacket.read_data)
        not_empty_ipc_txpacket.decode(raw)
        print(not_empty_ipc_txpacket)

        print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT)))
        print(raw)
        print(ipc_fpgarqpacket_read)

        socket_tx_packets.send(raw)

while True:

    # wait for a package to arrive
    print ('RECEIVING on %s with TIMEOUT %d' % (socket_rx_command_packets.get_string(zmq.LAST_ENDPOINT), socket_rx_command_packets.get(zmq.RCVTIMEO)))
    message = recv_zmq(socket_rx_command_packets)

    # decode the package
    ipc_rxcompacket = RxCommandPacket()
    ipc_rxcompacket.decode(message)
    print (ipc_rxcompacket)
    print ('| got PAYLOAD %s' % (ipc_rxcompacket.payload))

    # do something with the package
    if ipc_rxcompacket.payload == b'block':
        print ("SLEEPING FOR 60 SECONDS")
        time.sleep(30)
    elif ipc_rxcompacket.payload == b'getMap':
        print ("REQUESTING FPGA MAP")
        getFPGAmap()




