#!/usr/bin/env python
import zmq
import time
import os
import sys
import struct
sys.path.append('/root/lib/')
from options import *
from ipc_packets import RxCommandPacket
from zmqTxRx import recv_zmq

tot_rx = 0
context = zmq.Context()
socket_rx_command_packets = context.socket(zmq.SUB)
socket_rx_command_packets.setsockopt(zmq.SUBSCRIBE, b'')
socket_rx_command_packets.connect("tcp://127.0.0.1:%s" % LOAD_BALANCER_PORT)
poller_rx_command_packets = zmq.Poller()
poller_rx_command_packets.register(socket_rx_command_packets, zmq.POLLIN)


while True:
    sockets = dict(poller_rx_command_packets.poll(100))
    if socket_rx_command_packets in sockets and sockets[socket_rx_command_packets] == zmq.POLLIN:
        message = recv_zmq(socket_rx_command_packets)
        ipc_rxcompacket = RxCommandPacket()
        ipc_rxcompacket.decode(message)
        if ipc_rxcompacket.APID == APID_TIME_AT_TONE:
            tot_rx += 1
            if tot_rx >= 2:
                tai_secs,_,_,_,_,_,_,_,_,_,_,_,_ = struct.unpack('!L6QB4QB', ipc_rxcompacket.payload)
                # Epoch starts Jan 1 2000 (or 946684800 s)
                set_time = time.gmtime(946684800+tai_secs)
                os.system("timedatectl set-time '%04d-%02d-%02d %02d:%02d:%02d'" % (set_time.tm_year,
                                                                                    set_time.tm_mon,
                                                                                    set_time.tm_mday,
                                                                                    set_time.tm_hour,
                                                                                    set_time.tm_min,
                                                                                    set_time.tm_sec))
                break
