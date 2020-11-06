#!/usr/bin/env python3

import sys
import zmq
import json
import time

context = zmq.Context()

port_FPGA_map_request = "5557" #receive FPGA_map_request
socket_FPGA_map_request = context.socket(zmq.SUB)
socket_FPGA_map_request.bind("tcp://*:%s" % port_FPGA_map_request)

port_FPGA_map_answer = "5558" #send FPGA_map_answer
socket_FPGA_map_answer = context.socket(zmq.PUB)
socket_FPGA_map_answer.bind ("tcp://*:%s" % port_FPGA_map_answer)

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
