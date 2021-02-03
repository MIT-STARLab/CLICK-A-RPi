#!/usr/bin/env python
import sys #importing options and functions
import os
import zmq
import json
import time
sys.path.append('../lib/')
sys.path.append('/root/lib/')
from options import *
from ipc_packets import PATControlPacket, PATHealthPacket, PATStatusPacket
from zmqTxRx import recv_zmq, send_zmq
import struct

status_list = [PAT_STATUS_CAMERA_INIT, PAT_STATUS_STANDBY, PAT_STATUS_MAIN]

context = zmq.Context()

socket_PAT_status = context.socket(zmq.SUB)
socket_PAT_status.bind("tcp://*:%s" % PAT_STATUS_PORT)
socket_PAT_status.setsockopt(zmq.SUBSCRIBE, b'')
poller = zmq.Poller()
poller.register(socket_PAT_status, zmq.POLLIN)

# socket needs some time to set up. give it a second - else the first message will be lost
time.sleep(1)

print("\n")
print('RECEIVING on %s' % socket_PAT_status.get_string(zmq.LAST_ENDPOINT))
poll_timeout_msec = 1000
while True:    
	socks = dict(poller.poll(poll_timeout_msec))
	if socket_PAT_status in socks and socks[socket_PAT_status] == zmq.POLLIN:
		#Read telemetry if available
		print('RECEIVING on %s' % socket_PAT_status.get_string(zmq.LAST_ENDPOINT))
		message = recv_zmq(socket_PAT_status)
		ipc_patStatusPacket = PATStatusPacket()
		return_addr, status_flag = ipc_patStatusPacket.decode(message) #decode the package
		if(status_flag in status_list):
			if(status_flag == PAT_STATUS_CAMERA_INIT):
				print ('PAT Process (' + str(return_addr) + ') Status: In Camera Initialization Loop')
			elif(status_flag == PAT_STATUS_STANDBY):
				print ('PAT Process (' + str(return_addr) + ') Status: In Standby Loop')
			elif(status_flag == PAT_STATUS_MAIN):
				print ('PAT Process (' + str(return_addr) + ') Status: In Main PAT Loop')
		else:
			print ('Unrecognized PAT Status Flag: ' + status_flag)
