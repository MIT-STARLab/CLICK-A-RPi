#!/usr/bin/env python
import sys #importing options and functions
import os
import zmq
import json
import time
sys.path.append('../lib/')
sys.path.append('/root/lib/')
from options import *
from ipc_packets import PATStatusPacket
from zmqTxRx import recv_zmq, send_zmq
import struct

#PAT Status Flag List
pat_status_list = [PAT_STATUS_CAMERA_INIT, PAT_STATUS_STANDBY, PAT_STATUS_STANDBY_CALIBRATED, PAT_STATUS_STANDBY_SELF_TEST_PASSED, PAT_STATUS_STANDBY_SELF_TEST_FAILED, PAT_STATUS_MAIN]
pat_status_names = ['CAMERA INIT', 'STANDBY', 'STANDBY_CALIBRATED', 'STANDBY_SELF_TEST_PASSED', 'STANDBY_SELF_TEST_FAILED', 'MAIN']

context = zmq.Context()

print ("Pulling PAT Status Packets")
print ("on port {}".format(PAT_STATUS_PORT))
socket_PAT_status = context.socket(zmq.SUB)
socket_PAT_status.bind("tcp://127.0.0.1:%s" % PAT_STATUS_PORT)
socket_PAT_status.setsockopt(zmq.SUBSCRIBE, b'')
poller_PAT_status = zmq.Poller()
poller_PAT_status.register(socket_PAT_status, zmq.POLLIN)

# socket needs some time to set up. give it a second - else the first message will be lost
time.sleep(1)
poll_timeout_msec = 250
def get_pat_status():
    #get pat status
    socks_status = dict(poller_PAT_status.poll(poll_timeout_msec)) #poll for 250 ms
    if socket_PAT_status in socks_status and socks_status[socket_PAT_status] == zmq.POLLIN:
        #print('RECEIVING on %s' % socket_PAT_status.get_string(zmq.LAST_ENDPOINT))
        message = recv_zmq(socket_PAT_status)
        ipc_patStatusPacket = PATStatusPacket()
        pat_return_addr, pat_status_flag = ipc_patStatusPacket.decode(message) #decode the package
        received_status = True
    else:
        pat_return_addr = -1
        pat_status_flag = -1
        received_status = False

    return received_status, pat_status_flag, pat_return_addr

def update_pat_status(pat_status_flag):
    #get pat status
    received_status, new_status_flag, _ = get_pat_status()
    if(received_status):
        pat_status_flag = new_status_flag

    return pat_status_flag

#intialize PAT status
for i in range(10):
   pat_received_status, pat_status_flag, pat_return_addr = get_pat_status()
   if(pat_received_status):
       print('Connected to PAT process at PID = ' + str(pat_return_addr))
       break
if(not pat_received_status):
   print('WARNING: PAT process unresponsive at test script (PID = ' + str(pat_return_addr) + ') startup.')

def pat_status_is(pat_status_check):
    if(pat_status_flag in pat_status_list):
        print('PAT Process Running (PID: ' + str(pat_return_addr) + '). Status: ' + pat_status_names[pat_status_list.index(pat_status_flag)])
        #print('pat_status_flag: ', pat_status_flag)
        #print('pat_status_check: ', pat_status_check)
        #print('bool: ', (pat_status_flag == pat_status_check))
        return (pat_status_flag == pat_status_check)
    else:
        print('PAT Process Running (PID: ' + str(pat_return_addr) + '). Status: Unrecognized')
        return False

display_period_sec = 1
counter = 0
while True:    
	pat_status_flag = update_pat_status(pat_status_flag)
	for status_flag_check in pat_status_list:
		pat_status_is(status_flag_check)
	time.sleep(1)
