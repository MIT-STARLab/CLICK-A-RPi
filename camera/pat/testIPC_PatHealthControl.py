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

#Shared Command Parameters with c-code (packetdef.h)
CMD_START_PAT = 0x00
CMD_END_PAT = 0x01
CMD_PAYLOAD_SIZE = 256 #Only a fixed size is allowed in the C++ code (packetdef.h): add padding if necessary
CMD_HEADER_SIZE = 5 #Only a fixed size is allowed in the C++ code (packetdef.h): add padding if necessary

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
        
        payload_list = self.payload.split(b'\n')
        
        telemetry_string = payload_list[0].decode('utf-8')
        
        return telemetry_string, self.return_addr, self.size, self.payload

def send_pat_command(socket_PAT_control, return_address, command, payload = ''):
        #Define Command Header
        CMD_HEADER = return_address + '\0' #PID Header
        #Header format checks and padding
        assert len(CMD_HEADER) <= CMD_HEADER_SIZE #Ensure CMD_HEADER is the right length
        if(len(CMD_HEADER) < CMD_HEADER_SIZE):
                for i in range(CMD_HEADER_SIZE - len(CMD_HEADER)):
                        CMD_HEADER = CMD_HEADER + '\0' #append null padding
        assert CMD_HEADER[len(CMD_HEADER)-1] == '\0' #always terminate strings with null character ('\0') for c-code

        #Define Command Payload
        CMD_PAYLOAD = payload + '\0'
        assert len(CMD_PAYLOAD) <= CMD_PAYLOAD_SIZE #Ensure CMD_PAYLOAD is the right length
        if(len(CMD_PAYLOAD) < CMD_PAYLOAD_SIZE):
                for i in range(CMD_PAYLOAD_SIZE - len(CMD_PAYLOAD)):
                        CMD_PAYLOAD = CMD_PAYLOAD + '\0' #append null padding
        assert CMD_HEADER[len(CMD_HEADER)-1] == '\0' #always terminate strings with null character ('\0') for c-code
        CMD_PAYLOAD = str(CMD_PAYLOAD).encode('ascii') #format to ascii

        ipc_patControlPacket = PATControlPacket()
        raw_patControlPacket = ipc_patControlPacket.encode(command,CMD_PAYLOAD) 
        send_zmq(socket_PAT_control, raw_patControlPacket, CMD_HEADER) 
        
        return ipc_patControlPacket
        
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

return_address = '9999' #Placeholder PID

# Wait for a ping from the PAT process
print('RECEIVING on %s with TIMEOUT %d' % (socket_PAT_health.get_string(zmq.LAST_ENDPOINT), socket_PAT_health.get(zmq.RCVTIMEO)))
message = recv_zmq(socket_PAT_health)
ipc_patHealthPacket = PATHealthPacket()
telemetry_string, _, _, _ = ipc_patHealthPacket.decode(message) #decode the package
print(telemetry_string)
time.sleep(1)

# Send the CMD_START_PAT Command
print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, CMD_START_PAT)    
print(ipc_patControlPacket)
time.sleep(1)
    
while True:
        #Continuously read telemetry
        print('RECEIVING on %s with TIMEOUT %d' % (socket_PAT_health.get_string(zmq.LAST_ENDPOINT), socket_PAT_health.get(zmq.RCVTIMEO)))
        message = recv_zmq(socket_PAT_health)
        ipc_patHealthPacket = PATHealthPacket()
        telemetry_string, _, _, _ = ipc_patHealthPacket.decode(message) #decode the package
        print(telemetry_string)
        time.sleep(1)
