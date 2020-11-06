#!/usr/bin/env python3

import sys
import zmq
import json
import time

import sys #importing options and functions
sys.path.append('../lib/')
from options import FPGA_MAP_ANSWER_PORT, FPGA_MAP_REQUEST_PORT

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


print ("\n")

while True:

    print ("Waiting for FPGA_map_request ...")
    topic, FPGA_map_request = demogrify(socket_FPGA_map_request.recv())
    print (topic, FPGA_map_request)

    print ("")
    time.sleep(1)

    print ("Sending FPGA_map_answer to process {}".format(topic))
    FPGA_map_answer  = dict()
    FPGA_map_answer ['timestamp'] = time.time()
    FPGA_map_answer ['somedata'] = 'I am Groot'
    print (FPGA_map_answer)
    socket_FPGA_map_answer.send(mogrify(topic, FPGA_map_answer ))
    print ("\n=======\n")

