#/usr/bin/env python3

import sys
import zmq
import json
import time
import argparse
import fl
from node import NodeFPGA
import sys
import memorymap
import fsm
import edfa
import alignment

import sys #importing options and functions
sys.path.append('../../lib/')
#sys.path.append('/home/pi/CLICK-A/github/lib/')
from options import FPGA_MAP_ANSWER_PORT, FPGA_MAP_REQUEST_PORT
from ipc_packets import FPGAMapRequestPacket, FPGAMapAnswerPacket
from zmqTxRx import recv_zmq, send_zmq

context = zmq.Context()

socket_FPGA_map_request = context.socket(zmq.SUB)
socket_FPGA_map_request.bind("tcp://*:%s" % FPGA_MAP_REQUEST_PORT)

socket_FPGA_map_answer = context.socket(zmq.PUB)
socket_FPGA_map_answer.bind("tcp://*:%s" % FPGA_MAP_ANSWER_PORT)

# socket.setsockopt(zmq.SUBSCRIBE, topicfilter)
# subscribe to ALL incoming FPGA_map_requests
socket_FPGA_map_request.setsockopt(zmq.SUBSCRIBE, b'')

# socket needs some time to set up. give it a second - else the first message will be lost
time.sleep(1)

# https://stackoverflow.com/questions/25188792/how-can-i-use-send-json-with-pyzmq-pub-sub
# How can I use send_json with pyzmq PUB SUB
def mogrify(topic, msg):
    """ json encode the message and prepend the topic """
    return (topic + ' ' + json.dumps(msg)).encode('ascii')

def demogrify(topicmsg):
    """ Inverse of mogrify() """
    topicmsg = topicmsg.decode('ascii')
    json0 = topicmsg.find('{')
    topic = topicmsg[0:json0].strip()
    msg = json.loads(topicmsg[json0:])
    return topic, msg

DEBUG = False
memMap = memorymap.MemoryMap()
# FPGA image
FPGA_IMAGE = "/home/kingryan/Dropbox/grad_school/fpga/makestuff/hdlmake/apps/roamingryan/swled/bist/vhdl/top_level.xsvf"

# File-reader which yields chunks of data
def readFile(fileName):
    with open(fileName, "rb") as f:
        while True:
            chunk = f.read(32768)
            if chunk:
                yield chunk
            else:
                break

print("NODE FPGA Commander Copyright (C) 2015 Ryan Kingsbury\n")
parser = argparse.ArgumentParser(description='Load FX2LP firmware, load the FPGA, interact with the FPGA.')
parser.add_argument('-i', action="store", nargs=1, metavar="<VID:PID>", help="vendor ID and product ID (e.g 1443:0007)")
parser.add_argument('-v', action="store", nargs=1, required=True, metavar="<VID:PID>", help="VID, PID and opt. dev ID (e.g 1D50:602B:0001)")
#parser.add_argument('-d', action="store", nargs=1, metavar="<port+>", help="read/write digital ports (e.g B13+,C1-,B2?)")
parser.add_argument('-q', action="store", nargs=1, metavar="<jtagPorts>", help="query the JTAG chain")
parser.add_argument('-p', action="store", nargs=1, metavar="<config>", help="program a device")
parser.add_argument('-c', action="store", nargs=1, metavar="<conduit>", help="which comm conduit to choose (default 0x01)")
parser.add_argument('-f', action="store", nargs=1, metavar="<dataFile>", help="binary data to write to channel 0")
parser.add_argument('-N', action="store", nargs=1, metavar="numwrites", help="Number of times to write file to FPGA")
parser.add_argument('--ppm', action="store", nargs=1, metavar="<M>", help="PPM order")
parser.add_argument('--txdel', action="store", nargs=1, metavar="<delay>", help="TX loopback delay (in clock cycles)")
parser.add_argument('--dac', action="store", nargs=1, metavar="<counts>", help="DAC setting")
parser.add_argument('--prbs', action="store", nargs='?',const=True, metavar="<prbs>", help="Use PRBS")
parser.add_argument('--ser', action="store", nargs='?',const='1', metavar="<dwell>", help="perform slot error rate measurement")
parser.add_argument('--serCont', action="store", nargs='?',const='1', metavar="<dwell>", help="perform continuous slot error rate measurement")
parser.add_argument('--peak', action="store", nargs='?',const='0.1', metavar="<dwell>", help="peak power measurement, binary search for DAC value")
parser.add_argument('--read', action="store", nargs=1, metavar="<regnum>", help="Reads from the FPGA memory map, given a channel number (0-127)", type=int)
parser.add_argument('--write', action="store", nargs=2, metavar="<regnum>", help="Writes to the FPGA memory map, given a channel number (0-127) and value to be written", type=int)
parser.add_argument('--edfa', action="store", nargs=1, metavar="<edfacmd>", help="Writes either edfa on, edfa off, or flash to the FPGA register controlling the edfa")
parser.add_argument('--send', action="store", nargs=2, metavar="<sendfile>", help="Sends a binary file to the FPGA Modulator along with desired PPM order")
parser.add_argument('--power', action="store", nargs=1, metavar="<power>", help="Turns power on/off on the FPGA board")
parser.add_argument('--fsmWrite', action="store", nargs=2, metavar="<fsmWrite>", help="Send FSM commands", type=int)
parser.add_argument('--fsmInit', action="store", nargs="?", metavar="<fsm>", help="Initialize FSM DAC")
parser.add_argument('--fsmUpdate', action="store", nargs="?", metavar="<fsm>", help="Update FSM DAC Values")
parser.add_argument('--update', nargs = '?', help="Starts printing the values in registers 96-117 every 0.1 seconds until told to stop")
parser.add_argument('--interval', action="store", nargs=1, metavar="<interval>", help="Sets the printing interval", type=int)
parser.add_argument('--findCenter', action="store", nargs=1, metavar="<start>", help="Matches the laser wavelength to the center of the FBG")


argList = parser.parse_args()

print argList

handle = fl.FLHandle()
try:
    fl.flInitialise(0)

    vp = argList.v[0]
    print("Attempting to open connection to FPGALink device {}...".format(vp))
    try:
        handle = fl.flOpen(vp)
    except fl.FLException as ex:
        if ( argList.i ):
            ivp = argList.i[0]
            print("Loading firmware into {}...".format(ivp))
            fl.flLoadStandardFirmware(ivp, vp);
	    print type(ivp)
	    print type(vp)
            # Long delay for renumeration
            # TODO: fix this hack.  The timeout value specified in flAwaitDevice() below doesn't seem to work
            time.sleep(3)
            
            print("Awaiting renumeration...")
            if ( not fl.flAwaitDevice(vp, 10000) ):
                raise fl.FLException("FPGALink device did not renumerate properly as {}".format(vp))

            print("Attempting to open connection to FPGALink device {} again...".format(vp))
            handle = fl.flOpen(vp)
        else:
            raise fl.FLException("Could not open FPGALink device at {} and no initial VID:PID was supplied".format(vp))
    
    # if ( argList.d ):
    #     print("Configuring ports...")
    #     rb = "{:0{}b}".format(fl.flMultiBitPortAccess(handle, argList.d[0]), 32)
    #     print("Readback:   28   24   20   16    12    8    4    0\n          {} {} {} {}  {} {} {} {}".format(
    #         rb[0:4], rb[4:8], rb[8:12], rb[12:16], rb[16:20], rb[20:24], rb[24:28], rb[28:32]))
    #     fl.flSleep(100)

    conduit = 1
    if ( argList.c ):
        conduit = int(argList.c[0])

    isNeroCapable = fl.flIsNeroCapable(handle)
    isCommCapable = fl.flIsCommCapable(handle, conduit)
    fl.flSelectConduit(handle, conduit)

    #Execute CODE:
    print ("\n")

    while True:

        # wait for a package to arrive
        print ('RECEIVING on %s with TIMEOUT %d' % (socket_FPGA_map_request.get_string(zmq.LAST_ENDPOINT), socket_FPGA_map_request.get(zmq.RCVTIMEO)))
        message = recv_zmq(socket_FPGA_map_request)

        # decode the package
        ipc_fpgarqpacket = FPGAMapRequestPacket()
        return_addr, rq_number, rw_flag, start_addr, size, write_data_raw = ipc_fpgarqpacket.decode(message) #
        write_data = int(write_data_raw.encode('hex'), 16) 
        print (ipc_fpgarqpacket)
        print "return_addr: ", return_addr
        print "rq_number: ", rq_number
        print "rw_flag: ", rw_flag
        print "start_addr: ", start_addr
        print "size: ", size
        print "write_data (X): 0x%X" % write_data
        print "write_data (d): %d" % write_data

        if(start_addr == 0x20):
            if(write_data == 0x55):
                print "Prior to Cal Laser ON - Writing 0x55 to channel 35"
                if(isCommCapable):
                    fl.flWriteChannel(handle, 35, 0x55)
                    time.sleep(0.001)
                    ch35_val = fl.flReadChannel(handle, 35)
                    print "Channel 35 Value: 0x%X" % ch35_val
                else:
                    print "Error: FPGA not Comm Capable"
                #print "Prior to Cal Laser ON - Writing 0x55 to channel 32"
                #fl.flWriteChannel(handle, 32, 0x55) 
                #time.sleep(0.001)
            #if(write_data == 0x0F):
            #    print "Prior to Cal Laser OFF - Writing 0x0F to channel 32"
            #    fl.flWriteChannel(handle, 32, 0x0F) 
            #    time.sleep(0.001)
        
        print "Writing to FPGA..."
        if(isCommCapable):
            print "Writing to Channel: 0x%X" % start_addr
            print "Data to Write: 0x%X" % write_data
            fl.flWriteChannel(handle, start_addr, write_data)
            ch_val = fl.flReadChannel(handle, start_addr) 
            print "New Channel Value: 0x%X" % ch_val
        else:
            print "Error: FPGA not Comm Capable"

        # ~ if ipc_fpgarqpacket.rw_flag == 1:
            # ~ print ('| got FPGA_MAP_REQUEST_PACKET with WRITE in ENVELOPE %d' % (ipc_fpgarqpacket.return_addr))
            # ~ time.sleep(1)
            # ~ # send the FPGA_map_answer packet (write)
            # ~ ipc_fpgaaswpacket_write = FPGAMapAnswerPacket()
            # ~ raw = ipc_fpgaaswpacket_write.encode(return_addr=ipc_fpgarqpacket.return_addr, rq_number=123, rw_flag=1, error_flag=0, start_addr=0xDEF0, size=0)
            # ~ ipc_fpgaaswpacket_write.decode(raw)
            # ~ print ('SENDING to %s with ENVELOPE %d' % (socket_FPGA_map_answer.get_string(zmq.LAST_ENDPOINT), ipc_fpgaaswpacket_write.return_addr))
            # ~ print(b'| ' + raw)
            # ~ print(ipc_fpgaaswpacket_write)
            # ~ send_zmq(socket_FPGA_map_answer, raw, ipc_fpgaaswpacket_write.return_addr)

        # ~ else:
            # ~ print ('| got FPGA_MAP_REQUEST_PACKET with READ in ENVELOPE %d' % (ipc_fpgarqpacket.return_addr))
            # ~ time.sleep(1)
            # ~ # send the FPGA_map_answer packet (read)
            # ~ ipc_fpgaaswpacket_read = FPGAMapAnswerPacket()
            # ~ raw = ipc_fpgaaswpacket_read.encode(return_addr=ipc_fpgarqpacket.return_addr, rq_number=123, rw_flag=0, error_flag=0, start_addr=0x9ABC, size=16, read_data=b"I'm Mr. Meeseeks")
            # ~ ipc_fpgaaswpacket_read.decode(raw)
            # ~ print ('SENDING to %s with ENVELOPE %d' % (socket_FPGA_map_answer.get_string(zmq.LAST_ENDPOINT), ipc_fpgaaswpacket_read.return_addr))
            # ~ print(b'| ' + raw)
            # ~ print(ipc_fpgaaswpacket_read)
            # ~ send_zmq(socket_FPGA_map_answer, raw, ipc_fpgaaswpacket_read.return_addr)

except fl.FLException as ex:
    print(ex)
finally:
    fl.flClose(handle)
