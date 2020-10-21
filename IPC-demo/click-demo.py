#!/usr/bin/env python3

#
#   Hello World server in Python
#   Binds REP socket to tcp://*:5555
#   Expects b"Hello" from client, replies with b"World"
#

import time
import zmq
from multiprocessing import Process, Array

REQUEST_TIMEOUT = 1000
SEND_TIMEOUT = 1000
SERVER_ENDPOINT = "tcp://*:5555"
PY_ENDPOINT = "tcp://localhost:5556"
CPP_ENDPOINT = "tcp://localhost:5557"

context = zmq.Context()


def Watcher(status):
    print("Starting process watcher")
    context = zmq.Context()
    print("")
    print("Connecting to python process server…")
    print(PY_ENDPOINT)
    socketPY = context.socket(zmq.REQ)
    socketPY.RCVTIMEO = REQUEST_TIMEOUT # receiver timeout in milliseconds
    socketPY.SNDTIMEO = SEND_TIMEOUT # sender timeout in milliseconds
    socketPY.LINGER = 0 # pending messages shall be discarded immediately when the socket is closed with zmq_close()
    socketPY.connect(PY_ENDPOINT)
    print("")
    print("Connecting to cpp process server…")
    print(CPP_ENDPOINT)
    socketCPP = context.socket(zmq.REQ)
    socketCPP.RCVTIMEO = REQUEST_TIMEOUT # receiver timeout in milliseconds
    socketCPP.SNDTIMEO = SEND_TIMEOUT # sender timeout in milliseconds
    socketCPP.LINGER = 0 # pending messages shall be discarded immediately when the socket is closed with zmq_close()
    socketCPP.connect(CPP_ENDPOINT)
    print("")

    while True:

        time.sleep(5)

        try:
           socketPY.send(b"ping")
           message = socketPY.recv()
           print("Received response: %s" % (message))
           status[0] = int(message)
        except zmq.ZMQError as e:
           if e.errno == zmq.EAGAIN:
               print ("ERROR: PY resource temporarily unavailable (zmq.EAGAIN)")
               socketPY.close()
               print ("Reconnecting")
               socketPY = context.socket(zmq.REQ)
               socketPY.RCVTIMEO = REQUEST_TIMEOUT # receiver timeout in milliseconds
               socketPY.SNDTIMEO = SEND_TIMEOUT # sender timeout in milliseconds
               socketPY.LINGER = 0 # pending messages shall be discarded immediately when the socket is closed with zmq_close()
               socketPY.connect(PY_ENDPOINT)
               pass
           else:
               raise

        try:
           socketCPP.send(b"ping")
           message = socketCPP.recv()
           print("Received response: %s" % (message))
           status[1] = int(message)
        except zmq.ZMQError as e:
           if e.errno == zmq.EAGAIN:
               print ("ERROR: CPP resource temporarily unavailable (zmq.EAGAIN)")
               socketCPP.close()
               print ("Reconnecting")
               socketCPP = context.socket(zmq.REQ)
               socketCPP.RCVTIMEO = REQUEST_TIMEOUT # receiver timeout in milliseconds
               socketCPP.SNDTIMEO = SEND_TIMEOUT # sender timeout in milliseconds
               socketCPP.LINGER = 0 # pending messages shall be discarded immediately when the socket is closed with zmq_close()
               socketCPP.connect(CPP_ENDPOINT)
               pass
           else:
               raise

def Listener(status):
    print("Starting command listener")
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    print("")
    print("Listening to incoming commands…")
    print(SERVER_ENDPOINT)
    socket.bind(SERVER_ENDPOINT)
    print("")

    while True:
        #  Wait for next request from client
        print("listening…")
        print("")
        command = socket.recv()
        print("Received command: %s" % command)

        if command == b"status":
            print("STATUS")
            socket.send_string(str(status[:]))
        else:
            socket.send(b"unknown command")


if __name__ == '__main__':
    status = Array('i', range(2)) # 'i' indicates a signed integer

    starttime = time.time()
    print (str(round(starttime)))

    watcher = Process(target=Watcher, args=(status,))
    watcher.start()

    listener = Process(target=Listener, args=(status,))
    listener.start()
    listener.join() #Block the main thread until the process whose join() method is called terminates or until the optional timeout occurs.



