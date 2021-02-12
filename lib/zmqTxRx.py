#!/usr/bin/env python
import zmq

def push_zmq(socket, payload):
    '''push zmq message without envelope
    socket: zmq socket
    payload: raw contents, bytes
    returns
    error'''
    return socket.send(payload)

def send_zmq(socket, payload):
    '''send zmq message with envelope
    socket: zmq socket
    payload: raw contents, bytes
    envelope: string (usually unique identifier for return message, can be PID)
    returns
    error'''
    return socket.send(payload)

def recv_zmq(socket):
    '''receive zmq message with envelope
    returns
    message: raw contents, bytes
    envelope: string'''
    try:
       message = socket.recv()
       return (message)
    except zmq.ZMQError as e: #socket_rx_command_packets should never timeout - instead it just waits forever for a packet to come in
       if e.errno == zmq.EAGAIN:
           print ("\n=======\n")
           print ("TIMEOUT")
           print ("\n=======\n")
           pass
       else:
           raise

def separate(payload):
    array = payload.split(b' ', 1)
    return array[1], array[0]
