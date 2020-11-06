#!/usr/bin/env python3

#
#   Inter-process Communication Demo
#   Talks to click-demo service to execute specific commands and returns response
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Ping" to server, expects current timestamp back
#

import zmq
import time
import os

REQUEST_TIMEOUT = 1000
SEND_TIMEOUT = 1000
SERVER_ENDPOINT = "tcp://localhost:5555"

def evaluate_command(command):
    global socket
    if command == "status":
        print ("Sending 'status' request")
        try:
           socket.send(b"status")
           message = socket.recv()
           print("Received response: %s" % (message))
        except zmq.ZMQError as e:
           if e.errno == zmq.EAGAIN:
               print ("ERROR: Resource temporarily unavailable (zmq.EAGAIN)")
               socket.close()
               print ("Reconnecting")
               socket = context.socket(zmq.REQ)
               socket.RCVTIMEO = REQUEST_TIMEOUT # receiver timeout in milliseconds
               socket.SNDTIMEO = SEND_TIMEOUT # sender timeout in milliseconds
               socket.LINGER = 0 # pending messages shall be discarded immediately when the socket is closed with zmq_close()
               socket.connect(SERVER_ENDPOINT)
               pass
           else:
               raise
        print ("")
    elif command == "start":
        print ("Starting services")
        time.sleep(1)
        print ("click-demo")
        os.system("systemctl --user start click-demo")
        time.sleep(1)
        print ("click-py-demo")
        os.system("systemctl --user start click-py-demo")
        time.sleep(1)
        print ("click-cpp-demo")
        os.system("systemctl --user start click-cpp-demo")
        print ("")
    elif command == "stop":
        print ("Stopping services")
        time.sleep(1)
        print ("click-demo")
        os.system("systemctl --user stop click-demo")
        time.sleep(1)
        print ("click-py-demo")
        os.system("systemctl --user stop click-py-demo")
        time.sleep(1)
        print ("click-cpp-demo")
        os.system("systemctl --user stop click-cpp-demo")
        print ("")
    elif command == "exit":
        print ("Okay Bye")
        print ("")
        exit()
    else:
        print ("ERROR: I have no idea what your monkey brain wants from me...")
        print ("")

context = zmq.Context()

#  Socket to talk to server
print ("")
print("Connecting to click-demo server…")
print(SERVER_ENDPOINT)
print("")
socket = context.socket(zmq.REQ)
socket.RCVTIMEO = REQUEST_TIMEOUT # receiver timeout in milliseconds
socket.SNDTIMEO = SEND_TIMEOUT # sender timeout in milliseconds
socket.LINGER = 0 # pending messages shall be discarded immediately when the socket is closed with zmq_close()
socket.connect(SERVER_ENDPOINT)

print ("Hello hairless monkey, how can my awesomeness assist you today?")
print ("")
print ("Available commands:")
print ("- status: get service status")
print ("- start: start all services")
print ("- stop: stop all services")
print ("- exit: terminate this script")
print ("")

while True:
    command = input ("Command: ")
    print ("")
    evaluate_command(command)

