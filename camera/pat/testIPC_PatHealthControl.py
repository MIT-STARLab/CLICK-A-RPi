#!/usr/bin/env python3

import sys
import zmq
import json
import time

import sys #importing options and functions
sys.path.append('../../lib/')
from options import PAT_HEALTH_PORT, PAT_CONTROL_PORT
from ipc_packets import PATControlPacket
from zmqTxRx import recv_zmq, send_zmq

#define pat health packet (can copy this over to ipc_packets.py after tested)
import struct

class IpcPacket:
    def __init__(self): pass
    
class PATHealthPacket(IpcPacket):
    def __init__(self): IpcPacket.__init__(self)

    def encode(self, return_addr, payload=''):
        '''Encode a packet to be transmited to the bus:
        return_addr: return address of sender
        size: size of telemetry contents in bytes
        payload: raw telemetry contents, bytes
        returns
        message bytes'''

        self.return_addr = return_addr
        self.size = len(payload)
        self.payload = payload
        
        self.raw = struct.pack('II%ds'%self.size,return_addr,self.size,payload)

        return self.raw

    def decode(self, raw):
        '''Decode a packet to be transmited to the bus:
        raw: message bytes to decode
        returns
        return_addr: return address of sender
        size: size of telemetry contents in bytes
        payload: raw command contents, bytes'''

        self.raw = raw
        raw_size = len(raw)-8

        self.return_addr, self.size, self.payload = struct.unpack('II%ds'%raw_size,raw)
        
        assert self.size == raw_size
        
        telemetry_bytes, padding_bytes = self.payload.split(b'\n')
        
        telemetry_string = telemetry_bytes.decode('utf-8')
        
        return telemetry_string, self.return_addr, self.size, self.payload


context = zmq.Context()

socket_PAT_health = context.socket(zmq.SUB)
socket_PAT_health.bind("tcp://*:%s" % PAT_HEALTH_PORT)
socket_PAT_health.setsockopt(zmq.SUBSCRIBE, b'')
# socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
# subscribe to ALL incoming FPGA_map_requests

socket_PAT_control = context.socket(zmq.PUB)
socket_PAT_control.bind("tcp://*:%s" % PAT_CONTROL_PORT)

# socket needs some time to set up. give it a second - else the first message will be lost
time.sleep(1)

print("\n")

while True:

    # wait for a package to arrive
    print('RECEIVING on %s with TIMEOUT %d' % (socket_PAT_health.get_string(zmq.LAST_ENDPOINT), socket_PAT_health.get(zmq.RCVTIMEO)))
    message = recv_zmq(socket_PAT_health)

    # decode the package
    ipc_patHealthPacket = PATHealthPacket()
    telemetry_string, return_addr, size, payload = ipc_patHealthPacket.decode(message)
    print(ipc_patHealthPacket)
    print('telemetry_string: ',telemetry_string)
    print('return_addr: ',return_addr)
    print('size: ',size)
    print('payload: ',payload)
    

    # ~ if ipc_fpgarqpacket.rw_flag == 1:
        # ~ print ('| got FPGA_MAP_REQUEST_PACKET with WRITE in ENVELOPE %d' % (ipc_fpgarqpacket.return_addr))
        # ~ time.sleep(1)
        # ~ # send the FPGA_map_answer packet (write)
        # ~ ipc_fpgaaswpacket_write = FPGAMapAnswerPacket()
        # ~ raw = ipc_fpgaaswpacket_write.encode(return_addr=ipc_fpgarqpacket.return_addr, rq_number=123, rw_flag=1, error_flag=0, start_addr=0xDEF0, size=0)
        # ~ ipc_fpgaaswpacket_write.decode(raw)
        # ~ print ('SENDING to %s with ENVELOPE %d' % (socket_FPGA_map_answer.get_string(zmq.LAST_ENDPOINT), ipc_fpgaaswpacket_write.return_addr))
        # ~ print(b'| ' + raw)
        # ~ print(ipc_fpgaaswpacket_write)
        # ~ send_zmq(socket_FPGA_map_answer, raw, ipc_fpgaaswpacket_write.return_addr)

    # ~ else:
        # ~ print ('| got FPGA_MAP_REQUEST_PACKET with READ in ENVELOPE %d' % (ipc_fpgarqpacket.return_addr))
        # ~ time.sleep(1)
        # ~ # send the FPGA_map_answer packet (read)
        # ~ ipc_fpgaaswpacket_read = FPGAMapAnswerPacket()
        # ~ raw = ipc_fpgaaswpacket_read.encode(return_addr=ipc_fpgarqpacket.return_addr, rq_number=123, rw_flag=0, error_flag=0, start_addr=0x9ABC, size=16, read_data=b"I'm Mr. Meeseeks")
        # ~ ipc_fpgaaswpacket_read.decode(raw)
        # ~ print ('SENDING to %s with ENVELOPE %d' % (socket_FPGA_map_answer.get_string(zmq.LAST_ENDPOINT), ipc_fpgaaswpacket_read.return_addr))
        # ~ print(b'| ' + raw)
        # ~ print(ipc_fpgaaswpacket_read)
        # ~ send_zmq(socket_FPGA_map_answer, raw, ipc_fpgaaswpacket_read.return_addr)



