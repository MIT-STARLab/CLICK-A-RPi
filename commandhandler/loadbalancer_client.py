#!/usr/bin/env python

import zmq

import sys
sys.path.append('/root/lib/')
sys.path.append('../lib/')
from options import *
from ipc_packets import RxCommandPacket
import ipc_loadbalancer

context = zmq.Context()

#ZMQ REQ worker socket for load balancing
ipc_client = ipc_loadbalancer.ClientInterface(context)

for _ in range(10):

    ipc_rxcompacket = RxCommandPacket()
    raw = ipc_rxcompacket.encode(APID=0x03,ts_txed_s=123,ts_txed_ms=2)

    #send payload to idle worker (this will wait until a worker becomes idle)
    ipc_client.send_request(raw)


