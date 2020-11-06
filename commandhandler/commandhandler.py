#!/usr/bin/env python3

import zmq
import random
import sys
import os
import time
import json

import sys #importing options and functions
sys.path.append('../lib/')
from options import FPGA_MAP_ANSWER_PORT, FPGA_MAP_REQUEST_PORT, TX_PACKETS_PORT, RX_CMD_PACKETS_PORT, MESSAGE_TIMEOUT


## JSON alternative
# print struct.unpack("i", struct.pack("i",4))[0] ?


# ZeroMQ inter process communication
context = zmq.Context()

socket_FPGA_map_request = context.socket(zmq.PUB) #send messages on this port
socket_FPGA_map_request.connect("tcp://localhost:%s" % FPGA_MAP_REQUEST_PORT) #connect to specific address (localhost)

socket_FPGA_map_answer = context.socket(zmq.SUB)
socket_FPGA_map_answer.connect ("tcp://localhost:%s" % FPGA_MAP_ANSWER_PORT)
# use PID as unique identifier for this progress
topic = str(os.getpid())
print ("Subscribing to topic {}".format(topic))
print ("on port {}".format(FPGA_MAP_ANSWER_PORT))
socket_FPGA_map_answer.setsockopt(zmq.SUBSCRIBE, topic.encode('ascii'))
socket_FPGA_map_answer.setsockopt(zmq.RCVTIMEO, MESSAGE_TIMEOUT) # 5 second timeout on receive

socket_tx_packets = context.socket(zmq.PUB)
socket_tx_packets.connect ("tcp://localhost:%s" % TX_PACKETS_PORT)

socket_rx_command_packets = context.socket(zmq.SUB)
socket_rx_command_packets.connect ("tcp://localhost:%s" % RX_CMD_PACKETS_PORT)
# socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
# subscribe to ALL incoming RX_CMD_PACKETS
socket_rx_command_packets.setsockopt(zmq.SUBSCRIBE, b'')

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

def getFPGAmap():
    FPGA_map_request  = dict()
    FPGA_map_request ['timestamp'] = time.time()
    FPGA_map_request ['command'] = 'gib'
    print ("Sending FPGA_map_request from process {}".format(topic))
    print (FPGA_map_request)
    socket_FPGA_map_request.send(mogrify(topic, FPGA_map_request ))

    print ("")

    print ("Waiting for FPGA_map_answer with topic {} ...".format(topic))

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

print ("\n")

while True:
    getFPGAmap()
    time.sleep(10)


