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
from ipc_packets import FPGAMapRequestPacket, FPGAMapAnswerPacket, TxPacket
from zmqTxRx import recv_zmq, send_zmq

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
socket_rx_command_packets = context.socket(zmq.PULL)
socket_rx_command_packets.connect ("tcp://127.0.0.1:%s" % RX_CMD_PACKETS_PORT)


# socket needs some time to set up. give it a second - else the first message will be lost
time.sleep(1)

# https://stackoverflow.com/questions/25188792/how-can-i-use-send-json-with-pyzmq-pub-sub
# How can I use send_json with pyzmq PUB SUB
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

def sendPatControl(): #this should be the Bus
    pat_control  = dict()
    pat_control ['timestamp'] = time.time()
    pat_control ['command'] = 'pat'
    print ("Sending socket_pat_control from process {}".format(topic))
    print (pat_control)
    socket_pat_control.send(mogrify(topic, pat_control ))

def returnToTest(): #this should be the Bus
    test_response_packets  = dict()
    test_response_packets ['timestamp'] = time.time()
    test_response_packets ['command'] = 'pong'
    print ("Sending test_response_packets from process {}".format(topic))
    print (test_response_packets)
    socket_test_response_packets.send(mogrify(topic, test_response_packets ))



def getFPGAmap():
    ipc_fpgarqpacket_write = FPGAMapRequestPacket()

    '''
    #create bytes object (struct) for writing (we are not doing this [yet])
    raw = ipc_fpgarqpacket_write.encode(return_addr=pid, rq_number=123, rw_flag=1, start_addr=0x5678, size=48, write_data=b"number of characters must match 'size' attribute")
    '''
    #create bytes object (struct) for reading
    '''do we need error handling for struct generation?'''
    raw = ipc_fpgarqpacket_write.encode(return_addr=pid, rq_number=123, rw_flag=0, start_addr=0x5678, size=48)

    #what's going on
    print('')
    ipc_fpgarqpacket_write.decode(raw)
    print('PAYLOAD')
    print(ipc_fpgarqpacket_write)
    print('')

    #send message
    '''do we need the return_addr in the struct?? we DO need it here'''
    send_zmq(socket_FPGA_map_request, raw, pid)

    #what's going on
    print('')
    print('SENDING')
    print(raw)
    print('TO')
    print(socket_FPGA_map_request.get_string(zmq.LAST_ENDPOINT))
    print('WITH ENVELOPE')
    print(pid)


    #socket_FPGA_map_request.send(b'123' + b' ' + ipc_fpgarqpacket_write)

    #send_pickle(socket_FPGA_map_request, ipc_fpgarqpacket_write, '123')

    print("")
    print("Waiting for FPGA_map_answer with topic {} ...".format(topic))
    print("")

    '''TODO: send to FPGA, wait for response, on timeout, send error'''

    not_empty_ipc_txpacket = TxPacket()
    raw = not_empty_ipc_txpacket.encode(APID=0x02,payload=b"timeout")
    not_empty_ipc_txpacket.decode(raw)
    print(not_empty_ipc_txpacket)

    send_zmq(socket_tx_packets, raw, pid)

    #what's going on
    print('')
    print('SENDING')
    print(raw)
    print('TO')
    print(socket_tx_packets.get_string(zmq.LAST_ENDPOINT))
    print('WITH ENVELOPE')
    print(pid)

    '''
    FPGA_map_request = dict()
    FPGA_map_request ['timestamp'] = time.time()
    FPGA_map_request ['command'] = 'gib'
    print ("Sending FPGA_map_request from process {}".format(topic))
    print (FPGA_map_request)
    socket_FPGA_map_request.send(mogrify(topic, FPGA_map_request ))
    '''



    '''
    try:
       recv_topic, FPGA_map_answer = demogrify(socket_FPGA_map_answer.recv())
       print (recv_topic, FPGA_map_answer)
       print ("\n=======\n")
    except zmq.ZMQError as e:
       if e.errno == zmq.EAGAIN:
           print ("TIMEOUT!!!")
           print ("\n=======\n")
           pass
       else:
           raise
    '''

print ("\n")

while True:
    '''
    try:
       print("Waiting for socket_rx_command_packets ...")
       recv_topic, rx_command_packets = demogrify(socket_rx_command_packets.recv())
       print (recv_topic, rx_command_packets)
       print ("\n======= sending response ========\n")
       sendPatControl()
    except zmq.ZMQError as e: #socket_rx_command_packets should never timeout - instead it just waits forever for a packet to come in
       if e.errno == zmq.EAGAIN:
           print ("TIMEOUT!!!")
           print ("\n=======\n")
           pass
       else:
           raise
    '''

    #what's going on
    print('')
    print('RECEIVING ON')
    print(socket_rx_command_packets.get_string(zmq.LAST_ENDPOINT))
    print('WITH TIMEOUT')
    print(socket_rx_command_packets.get(zmq.RCVTIMEO))

    message = recv_zmq(socket_rx_command_packets)

    #what's going on
    print('')
    print('GOT MESSAGE')
    print(message)
    print('WITH ENVELOPE')

    getFPGAmap()
    time.sleep(10)


