#!/usr/bin/env python


#needs to poll from a queue (that's filled by the depacketizer)
#and relay the message to the next available command handler (ipc_client.send_request(message))

import zmq
import time
import os
import sys
sys.path.append('/root/lib/')
sys.path.append('../lib/')
from options import *
from ipc_packets import RxCommandPacket, HeartbeatPacket
import ipc_loadbalancer

# use PID as unique identifier for this progress
pid = os.getpid()

context = zmq.Context()

socket_housekeeping = context.socket(zmq.PUB) #send messages on this port
socket_housekeeping.bind("tcp://127.0.0.1:%s" % LB_HEARTBEAT_PORT) #connect to specific address (localhost)

print ("Pulling Rx Cmd Packets")
print ("on port {}".format(RX_CMD_PACKETS_PORT))
socket_rx_command_packets = context.socket(zmq.SUB)
socket_rx_command_packets.setsockopt(zmq.SUBSCRIBE, b'')
socket_rx_command_packets.connect("tcp://127.0.0.1:%s" % RX_CMD_PACKETS_PORT)
poller_rx_command_packets = zmq.Poller() #poll rx commands
poller_rx_command_packets.register(socket_rx_command_packets, zmq.POLLIN)

# socket needs some time to set up. give it a second - else the first message will be lost
time.sleep(1)

#ZMQ REQ worker socket for load balancing
ipc_client = ipc_loadbalancer.ClientInterface(context)


#check if RX packet is available (polling timeout 1s)
#relay packet (only works if a client is idle - will block and not send heartbeat if all clients are busy)
#send heartbeat
#loop

#initialization
start_time = time.time() #default start_time is the execution time (debug or downlink mode commands overwrite this)
counter_heartbeat = 0 #used to count the number of repetitive process tasks

while True:
    curr_time = time.time()
    elapsed_time = curr_time - start_time

    #poll for received commands
    sockets = dict(poller_rx_command_packets.poll(1000)) #poll for 1000 milliseconds
    if socket_rx_command_packets in sockets and sockets[socket_rx_command_packets] == zmq.POLLIN:
        # get commands
        # print ('RECEIVING on %s with TIMEOUT %d' % (socket_rx_command_packets.get_string(zmq.LAST_ENDPOINT), socket_rx_command_packets.get(zmq.RCVTIMEO)))
        message = recv_zmq(socket_rx_command_packets)

        #relay message to idle worker (this will wait until a worker becomes idle and block heartbeats from being sent out if all workers are blocked)

        ipc_rxcompacket = RxCommandPacket() #Debug printing
        ipc_rxcompacket.decode(message) #Debug printing
        print(ipc_rxcompacket) #Debug printing

        ipc_client.send_request(message)

    else:
        print('no RxCommandPacket received for the last 1000 ms') #Debug printing

    #send heartbeat to housekeeping (HK_LB_HEARTBEAT_PD = every 10 seconds)
    if(elapsed_time >= HK_LB_HEARTBEAT_PD*counter_heartbeat):
        ipc_heartbeatPacket = HeartbeatPacket()
        raw_ipc_heartbeatPacket = ipc_heartbeatPacket.encode(pid, curr_time)
        print(ipc_heartbeatPacket) #Debug printing
        socket_housekeeping.send(raw_ipc_heartbeatPacket)
        counter_heartbeat += 1
