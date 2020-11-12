#!/usr/bin/env python3

#
#   Inter-process Communication Demo
#   Talks to click-demo service to execute specific commands and returns response
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Ping" to server, expects current timestamp back
#

import zmq
import time
import os
import json

import sys #importing options and functions
sys.path.append('../lib/')
sys.path.append('/home/pi/CLICK-A/github/lib/')
from options import FPGA_MAP_ANSWER_PORT, FPGA_MAP_REQUEST_PORT, TX_PACKETS_PORT, RX_CMD_PACKETS_PORT, MESSAGE_TIMEOUT, TEST_RESPONSE_PORT

# use PID as unique identifier for this progress
topic = str(os.getpid())

# ZeroMQ inter process communication
context = zmq.Context()

socket_rx_command_packets = context.socket(zmq.PUB) #send messages on this port
socket_rx_command_packets.bind("tcp://*:%s" % RX_CMD_PACKETS_PORT) #connect to specific address (localhost)

print ("Subscribing to all TEST_RESPONSE topics")
print ("on port {}".format(TEST_RESPONSE_PORT))
socket_test_response_packets = context.socket(zmq.SUB)
socket_test_response_packets.bind("tcp://*:%s" % TEST_RESPONSE_PORT)
# socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
# subscribe to ALL incoming TEST_RESPONSE
socket_test_response_packets.setsockopt(zmq.SUBSCRIBE, b'')
socket_test_response_packets.setsockopt(zmq.RCVTIMEO, MESSAGE_TIMEOUT) # 5 second timeout on receive

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

def evaluate_command(command):
    global topic, socket_rx_command_packets, test_response_packets
    if command == "ping":
        # send rx_command_packets with ping command
        rx_command_packets  = dict()
        rx_command_packets ['timestamp'] = time.time()
        rx_command_packets ['command'] = 'ping'
        print ("Sending rx_command_packets from process {}".format(topic))
        print (rx_command_packets)
        socket_rx_command_packets.send(mogrify(topic, rx_command_packets ))
        # waiting
        try:
           print("Waiting for test_response_packets ...")
           recv_topic, test_response_packets = demogrify(socket_test_response_packets.recv())
           print (recv_topic, test_response_packets)
        except zmq.ZMQError as e: #socket_rx_command_packets should never timeout - instead it just waits forever for a packet to come in
           if e.errno == zmq.EAGAIN:
               print ("TIMEOUT!!!")
               pass
           else:
               raise

    elif command == "start":
        print ("Starting services")
        time.sleep(1)
        print ("Camera (camera)")
        os.system("systemctl --user start camera")
        time.sleep(1)
        print ("FPGA (fpga)")
        os.system("systemctl --user start fpga")
        time.sleep(1)
        print ("Command Handler (commandhandler)")
        os.system("systemctl --user start commandhandler")
        print ("")

    elif command == "stop":
        print ("Stopping services")
        time.sleep(1)
        print ("Camera (camera)")
        os.system("systemctl --user stop camera")
        time.sleep(1)
        print ("FPGA (fpga)")
        os.system("systemctl --user stop fpga")
        time.sleep(1)
        print ("Command Handler (commandhandler)")
        os.system("systemctl --user stop commandhandler")
        print ("")

    elif command == "exit":
        print ("Okay Bye")
        print ("")
        exit()

    else:
        print ("ERROR: I have no idea what your monkey brain wants from me...")
        print ("")

context = zmq.Context()


print ("Hello hairless monkey, how can my awesomeness assist you today?")
print ("")
print ("Available commands:")
print ("- ping: ping the command handler")
print ("- start: start all services")
print ("- stop: stop all services")
print ("- exit: terminate this script")
print ("")

while True:
    command = input ("Command: ")
    print ("")
    evaluate_command(command)

