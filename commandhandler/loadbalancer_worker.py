#!/usr/bin/env python

import zmq

import time
import random

import sys
sys.path.append('/root/lib/')
sys.path.append('../lib/')
import ipc_loadbalancer
from ipc_packets import RxCommandPacket

context = zmq.Context()

#ZMQ REQ worker socket for load balancing
ipc_worker = ipc_loadbalancer.WorkerInterface(context)

total = 0
while True:
    # Tell the router we're ready for work
    ipc_worker.send_ready()

    # Get RxCommandPacket() workload from router
    workload = ipc_worker.get_request()

    # interpret workload as RxCommandPacket
    ipc_rxcompacket = RxCommandPacket()
    ipc_rxcompacket.decode(workload)

    print(ipc_rxcompacket)

    # Do some random work
    time.sleep(0.1 * random.random())
