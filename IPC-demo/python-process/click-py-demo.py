#!/usr/bin/env python3
#
#   Hello World server in Python
#   Binds REP socket to tcp://*:5555
#   Expects b"Hello" from client, replies with b"World"
#

import time
import zmq

PY_ENDPOINT = "tcp://*:5556"

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind(PY_ENDPOINT)

print("Starting ipc listener")
context = zmq.Context()
socket = context.socket(zmq.REP)
print("")
print("Listening to incoming commands…")
print(PY_ENDPOINT)
socket.bind(PY_ENDPOINT)
print("")

while True:
    #  Wait for next request from client
    message = socket.recv()
    print("Received request: %s" % message)

    #  Send timestamp back to client
    socket.send_string(str(round(time.time())))

