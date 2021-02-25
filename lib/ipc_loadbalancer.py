import zmq
import options
import os
import time

import sys
#importing options and functions
sys.path.append('/root/lib/')
sys.path.append('../lib/')
from options import *


class WorkerInterface:
    def __init__(self,context=None):

        #use PID as unique identifier for this process
        self.pid = unicode(str(os.getpid()), "utf-8")

        # ZeroMQ context setup
        if not context: context = zmq.Context()

        # Worker REQ socket
        self.socket_worker = context.socket(zmq.REQ)
        self.socket_worker.setsockopt_string(zmq.IDENTITY, self.pid)
        self.socket_worker.connect("tcp://127.0.0.1:%s" % options.RX_CMD_PACKETS_PORT)

        # Worker REQ Poller
        self.worker_poller = zmq.Poller()
        self.worker_poller.register(self.socket_worker, zmq.POLLIN)

        # socket needs some time to set up. give it a second - else the first message will be lost
        time.sleep(1)

    def get_request(self): #blocking

        # Get workload from router
        workload = self.socket_worker.recv()
        return workload

    def poll_request(self,timeout): #polling (10 ms)

        # Check if payload is available from router
        if self.worker_poller.poll(timeout):
            # get payload
            workload = self.socket_worker.recv()
        else:
            # set payload to NULL
            workload = None

        return workload

    def send_ready(self):

        #indicate idle (ready to receive work)
        self.socket_worker.send(b"ready")


class ClientInterface:
    def __init__(self,context=None):

        #use PID as unique identifier for this process
        self.pid = unicode(str(os.getpid()), "utf-8")

        # ZeroMQ context setup
        if not context: context = zmq.Context()

        # Client ROUTER socket
        self.socket_client = context.socket(zmq.ROUTER)
        self.socket_client.bind("tcp://127.0.0.1:%s" % RX_CMD_PACKETS_PORT)

        # socket needs some time to set up. give it a second - else the first message will be lost
        time.sleep(1)

    def send_request(self,payload):

        # worker next waiting in the queue
        address, empty, ready = self.socket_client.recv_multipart()

        self.socket_client.send_multipart([
            address,
            b'',
            payload,
        ])
