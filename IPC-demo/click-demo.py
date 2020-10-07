#!/usr/bin/env python3

#
#   Hello World server in Python
#   Binds REP socket to tcp://*:5555
#   Expects b"Hello" from client, replies with b"World"
#

import time
import zmq
from multiprocessing import Process


context = zmq.Context()
print("")
print("Connecting to python process server…")
print("tcp://localhost:5556")
socket2 = context.socket(zmq.REQ)
socket2.connect("tcp://localhost:5556")
print("")
print("Connecting to cpp process server…")
print("tcp://localhost:5557")
socket3 = context.socket(zmq.REQ)
socket3.connect("tcp://localhost:5557")
print("")


def Listener():
    print("Starting command listener")
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    print("")
    print("Listening to incoming commands…")
    print("tcp://*:5555")
    socket.bind("tcp://*:5555")

    while True:
        #  Wait for next request from client
        print("listening")
        command = socket.recv()
        print("Received command: %s" % command)

        if command == b"status":
            socket.send(b"status command")
        else:
            socket.send(b"unknown command")


if __name__ == '__main__':
    starttime = time.time()
    print (str(round(starttime)))
    listener = Process(target=Listener, args=())
    listener.start()
    listener.join() #Block the main thread until the process whose join() method is called terminates or until the optional timeout occurs.


