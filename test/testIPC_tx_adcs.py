#!/usr/bin/env python

import sys
import zmq
import json
import time
import struct
import sys #importing options and functions
sys.path.append('/root/lib/')
from options import TX_PACKETS_PORT
from ipc_packets import TxPacket
#from zmqTxRx import recv_zmq #send_zmq
        
context = zmq.Context()

socket_tx_packets = context.socket(zmq.SUB)
socket_tx_packets.bind("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)
socket_tx_packets.subscribe("")
# socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
# subscribe to ALL incoming FPGA_map_requests

# ~ socket_PAT_control = context.socket(zmq.PUB)
# ~ socket_PAT_control.bind("tcp://*:%s" % PAT_CONTROL_PORT)

# socket needs some time to set up. give it a second - else the first message will be lost
time.sleep(1)

print("\n")
    
while True:
        #Continuously read telemetry
        print('RECEIVING on %s with TIMEOUT %d' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT), socket_tx_packets.get(zmq.RCVTIMEO)))
        message = socket_tx_packets.recv()
        ipc_txPacket = TxPacket()
        apid, payload = ipc_txPacket.decode(message) #decode the package
        print(apid)
	#print(payload)
	data = struct.unpack('!3f', payload)
	print(data)
        time.sleep(1)

