#!/usr/bin/env python

#
#   Inter-process Communication Demo
#   Talks to click-demo service to execute specific commands and returns response
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Ping" to server, expects current timestamp back
#

import thread
import zmq
import time
import os
import json
import struct

import sys #importing options and functions
sys.path.append('../lib/')
sys.path.append('/root/lib/')
from options import FPGA_MAP_ANSWER_PORT, FPGA_MAP_REQUEST_PORT, TX_PACKETS_PORT, RX_CMD_PACKETS_PORT, MESSAGE_TIMEOUT, TEST_RESPONSE_PORT
from ipc_packets import RxCommandPacket, FPGAMapRequestPacket, FPGAMapAnswerPacket
from zmqTxRx import recv_zmq

# use PID as unique identifier for this progress
topic = str(os.getpid())
pid = os.getpid()

# ZeroMQ inter process communication
context = zmq.Context()

socket_rx_command_packets = context.socket(zmq.PUB) #send messages on this port (PUSH/PULL for load balancing!)
socket_rx_command_packets.bind("tcp://*:%s" % RX_CMD_PACKETS_PORT) #connect to specific address (localhost)

socket_FPGA_map_request = context.socket(zmq.PUB) #send messages on this port
socket_FPGA_map_request.connect("tcp://localhost:%s" % FPGA_MAP_REQUEST_PORT) #connect to specific address (localhost)

print ("Subscribing to FPGA_MAP_ANSWER topic {}".format(topic))
print ("on port {}".format(FPGA_MAP_ANSWER_PORT))
socket_FPGA_map_answer = context.socket(zmq.SUB)
#socket_FPGA_map_answer.setsockopt(zmq.SUBSCRIBE, topic.encode('ascii'))
socket_FPGA_map_answer.setsockopt(zmq.SUBSCRIBE, struct.pack('I',pid))
socket_FPGA_map_answer.setsockopt(zmq.RCVTIMEO, MESSAGE_TIMEOUT) # 5 second timeout on receive
socket_FPGA_map_answer.connect ("tcp://localhost:%s" % FPGA_MAP_ANSWER_PORT)

print ("Subscribing to all TEST_RESPONSE topics")
print ("on port {}".format(TEST_RESPONSE_PORT))
socket_test_response_packets = context.socket(zmq.SUB)
socket_test_response_packets.bind("tcp://*:%s" % TEST_RESPONSE_PORT)
# socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
# subscribe to ALL incoming TEST_RESPONSE
socket_test_response_packets.setsockopt(zmq.SUBSCRIBE, b'')
socket_test_response_packets.setsockopt(zmq.RCVTIMEO, MESSAGE_TIMEOUT) # 5 second timeout on receive

print ("Subscribing to all TX_PACKETS topics")
print ("on port {}".format(TX_PACKETS_PORT))
socket_tx_packets = context.socket(zmq.SUB)
socket_tx_packets.bind("tcp://*:%s" % TX_PACKETS_PORT)
socket_tx_packets.setsockopt(zmq.SUBSCRIBE, b'')

# Listen for incoming TX_PACKETS in a parallel thread
def listen_for_TX_packets( socket ):
   print('Starting listen_for_TX_packets thread ...')
   while 1:
        message = recv_zmq(socket)
        print(' ')
        print(' ')
        print('===')
        print('INCOMING TX_PACKET:')
        print(message)
        print('===')
        print(' ')
        print(' ')


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
    global topic, socket_rx_command_packets, socket_FPGA_map_request, socket_test_response_packets
    if command == "fpga":
        print ("Options:")
        print ("- read start_addr size")
        print ("- write start_addr size data")
        print ("")

        fpga = raw_input ("FPGA: ")
        options = fpga.split( )

        if options[0] == 'read':

            ipc_fpgarqpacket_read = FPGAMapRequestPacket()
            raw = ipc_fpgarqpacket_read.encode(return_addr=pid, rq_number=0, rw_flag=0, start_addr=int(options[1]), size=int(options[2]))

            ipc_fpgarqpacket_read.decode(raw)
            print(' ')
            print ('SENDING to %s' % (socket_FPGA_map_request.get_string(zmq.LAST_ENDPOINT)))
            print(ipc_fpgarqpacket_read)
            print(' ')

            socket_FPGA_map_request.send(raw)

            message = recv_zmq(socket_FPGA_map_answer)

            print('===')
            print('INCOMING FPGA_MAP_ANSWER_PACKET:')
            ipc_fpgarqpacket_answer = FPGAMapAnswerPacket()
            ipc_fpgarqpacket_answer.decode(message)
            print(ipc_fpgarqpacket_answer)
            print('===')

        elif options[0] == 'write':

            message = recv_zmq(socket_FPGA_map_answer)

            print('===')
            print('INCOMING FPGA_MAP_ANSWER_PACKET:')
            print(message)
            print('===')

        else:
            print ("ERROR: Not a valid option")
            print ("")


    elif command == "rx":
        # send rx_command_packets with payload

        print ("Payloads:")
        print ("- getMap: tell Command Handler to tell FPGA Driver to send an FPGA_MAP")
        print ("- block: tell Command Handler to block the process for 1 minute")
        print ("")

        payload = raw_input ("Payload: ")

        ipc_rxcompacket = RxCommandPacket()
        raw = ipc_rxcompacket.encode(APID=0x04,ts_txed_s=000,ts_txed_ms=0,payload=payload.encode('ascii'))

        ipc_rxcompacket.decode(raw)
        print(' ')
        print ('SENDING to %s' % (socket_rx_command_packets.get_string(zmq.LAST_ENDPOINT)))
        print(ipc_rxcompacket)
        print(' ')

        socket_rx_command_packets.send(raw)

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

thread.start_new_thread( listen_for_TX_packets, (socket_tx_packets, ) )

# socket needs some time to set up. give it a second - else the first message will be lost
time.sleep(1)

print('')
print('')

print ("Hello hairless monkey, how can my awesomeness assist you today?")
print ("")
print ("Available commands:")
print ("- fpga: send FPGA_MAP_REQUEST_PACKETS (read or write)")
print ("- rx: send RX_CMD_PACKETS")
print ("- start: start all services")
print ("- stop: stop all services")
print ("- exit: terminate this script")
print ("")

while True:
    command = raw_input ("Command: ")
    print ("")
    evaluate_command(command)

