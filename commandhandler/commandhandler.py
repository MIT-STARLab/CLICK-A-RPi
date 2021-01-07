#!/usr/bin/env python3

import zmq
import random
import sys
import os
import time
import json
import pickle
import sys
import struct
import math
import hashlib
#importing options and functions
sys.path.append('../lib/')
sys.path.append('/home/pi/CLICK-A/github/lib/')
from options import FPGA_MAP_ANSWER_PORT, FPGA_MAP_REQUEST_PORT, TX_PACKETS_PORT, RX_CMD_PACKETS_PORT, MESSAGE_TIMEOUT, PAT_CONTROL_PORT, TEST_RESPONSE_PORT
from ipc_packets import FPGAMapRequestPacket, FPGAMapAnswerPacket, TxPacket, RxCommandPacket
from zmqTxRx import recv_zmq, separate

# use PID as unique identifier for this progress
topic = str(os.getpid())
pid = os.getpid()

# ZeroMQ inter process communication
context = zmq.Context()

socket_FPGA_map_request = context.socket(zmq.PUB) #send messages on this port
socket_FPGA_map_request.connect("tcp://localhost:%s" % FPGA_MAP_REQUEST_PORT) #connect to specific address (localhost)

socket_test_response_packets = context.socket(zmq.PUB) #send messages on this port
socket_test_response_packets.connect("tcp://localhost:%s" % TEST_RESPONSE_PORT) #connect to specific address (localhost)

socket_pat_control = context.socket(zmq.PUB) #send messages on this port
socket_pat_control.connect("tcp://localhost:%s" % PAT_CONTROL_PORT) #connect to specific address (localhost)

print ("Subscribing to FPGA_MAP_ANSWER topic {}".format(topic))
print ("on port {}".format(FPGA_MAP_ANSWER_PORT))
socket_FPGA_map_answer = context.socket(zmq.SUB)
#socket_FPGA_map_answer.setsockopt(zmq.SUBSCRIBE, topic.encode('ascii'))
socket_FPGA_map_answer.setsockopt(zmq.SUBSCRIBE, struct.pack('I',pid))
socket_FPGA_map_answer.setsockopt(zmq.RCVTIMEO, MESSAGE_TIMEOUT) # 5 second timeout on receive
socket_FPGA_map_answer.connect ("tcp://localhost:%s" % FPGA_MAP_ANSWER_PORT)

socket_tx_packets = context.socket(zmq.PUB)
socket_tx_packets.connect ("tcp://localhost:%s" % TX_PACKETS_PORT)

print ("Pulling RX_CMD_PACKETS")
print ("on port {}".format(RX_CMD_PACKETS_PORT))
socket_rx_command_packets = context.socket(zmq.SUB)
socket_rx_command_packets.setsockopt(zmq.SUBSCRIBE, b'')
socket_rx_command_packets.connect ("tcp://127.0.0.1:%s" % RX_CMD_PACKETS_PORT)

# socket needs some time to set up. give it a second - else the first message will be lost
time.sleep(1)

def getFPGAmap(request_number, num_registers, start_address):
    ipc_fpgarqpacket_read = FPGAMapRequestPacket()

    #create bytes object (struct) for reading
    '''do we need error handling for struct generation?'''
    raw = ipc_fpgarqpacket_read.encode(return_addr=pid, rq_number=request_number, rw_flag=0, start_addr=start_address, size=num_registers)
    ipc_fpgarqpacket_read.decode(raw)

    print ('SENDING to %s with ENVELOPE %d' % (socket_rx_command_packets.get_string(zmq.LAST_ENDPOINT), ipc_fpgarqpacket_read.return_addr))
    print(raw)
    print(ipc_fpgarqpacket_read)

    #send message
    socket_FPGA_map_request.send(raw)

    print ('RECEIVING on %s with TIMEOUT %d for ENVELOPE %d' % (socket_FPGA_map_answer.get_string(zmq.LAST_ENDPOINT), socket_FPGA_map_answer.get(zmq.RCVTIMEO), ipc_fpgarqpacket_read.return_addr))
    # message, envelope = separate(recv_zmq(socket_FPGA_map_answer))
    message = recv_zmq(socket_FPGA_map_answer)

    print (message)

    # decode the package
    ipc_fpgaaswpacket = FPGAMapAnswerPacket()
    ipc_fpgaaswpacket.decode(message)
    print (ipc_fpgaaswpacket)
    print ('| got PAYLOAD %s' % (ipc_fpgaaswpacket.read_data))

    envelope = ipc_fpgaaswpacket.return_addr
    print(envelope)


    # sending read_data back as TX packet
    if ipc_fpgaaswpacket.read_data:
        not_empty_ipc_txpacket = TxPacket()
        raw = not_empty_ipc_txpacket.encode(APID=0x02,payload=ipc_fpgaaswpacket.read_data)
        not_empty_ipc_txpacket.decode(raw)
        print(not_empty_ipc_txpacket)

        print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT)))
        print(raw)
        print(ipc_fpgarqpacket_read)

        socket_tx_packets.send(raw)

while True:

    # wait for a package to arrive
    print ('RECEIVING on %s with TIMEOUT %d' % (socket_rx_command_packets.get_string(zmq.LAST_ENDPOINT), socket_rx_command_packets.get(zmq.RCVTIMEO)))
    message = recv_zmq(socket_rx_command_packets)

    # decode the package
    ipc_rxcompacket = RxCommandPacket()
    ipc_rxcompacket.decode(message)
    print (ipc_rxcompacket)
    print ('| got PAYLOAD %s' % (ipc_rxcompacket.payload))

    # Parse received command payload as per control packet format (TBR)
    generic_control_packet = GenericControlPacket()
    generic_control_packet.decode(ipc_rxcompacket.payload)
    CMD_ID = generic_control_packet.command

    if(CMD_ID == CMD_PL_REBOOT):
        #TODO: Acknowledge command receipt via telemetry
        os.system("sudo shutdown -r now") #reboot the RPi (TODO: Add other shutdown actions if needed)

    elif(CMD_ID == CMD_PL_ENABLE_TIME):
        #TODO
        pass

    elif(CMD_ID == CMD_PL_DISABLE_TIME):
        #TODO
        pass

    elif(CMD_ID == CMD_PL_EXEC_FILE):
        #TODO: Acknowledge command receipt via telemetry
        raw_size = generic_control_packet.size - 3 #TBR
        output_to_file, file_out_num, file_path_len, raw_file_path = struct.unpack('BBB%ds'%raw_size,generic_control_packet.payload) #TBR
        file_path = raw_file_path.decode('utf-8') #TBR
        file_path = file_path[0:(file_path_len-1)] #Strip padding
        try:
            os.system(file_path) #TBR
            #if(output_to_file):
                #TODO - have the script write to file with file_out_num (= file output number)

            #TODO: Send success telemetry
        except:
            #TODO: Send error telemetry
            pass

    elif(CMD_ID == CMD_PL_LIST_FILE):
        #TODO: Acknowledge command receipt via telemetry
        raw_size = generic_control_packet.size - 1 #TBR
        directory_path_len, raw_directory_path = struct.unpack('B%ds'%raw_size,generic_control_packet.payload) #TBR
        directory_path = raw_directory_path.decode('utf-8') #TBR
        directory_path = directory_path[0:(directory_path_len-1)] #Strip padding
        try:
            directory_listing = os.listdir(directory_path) #Get directory list
            #Convert list to single string for telemetry:
            return_data = ''
            for list_item in directory_listing:
                return_data = return_data + list_item + '\n' #TBR separator type

            list_file_txpacket = TxPacket()
            raw = list_file_txpacket.encode(payload=return_data) #TBR

            list_file_txpacket.decode(raw) #for Debug printing
            print(list_file_txpacket) #Debug printing
            print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT))) #Debug printing
            print(raw) #Debug printing

            socket_tx_packets.send(raw) #send packet

            #TODO: Send success telemetry
        except:
            #TODO: Send error telemetry
            pass

    elif(CMD_ID == CMD_PL_REQUEST_FILE):
        # simple dumb file chunking
        max_chunk_size = 4068
        file_name = generic_control_packet.payload #TBR
        with open(file_name, "rb") as source_file:
        	file_id = hashlib.md5(file_name).digest()
        	file_len = os.stat(file_name).st_size
        	seq_len = math.ceil(float(file_len)/max_chunk_size)
        	seq_cnt = 0
        	while ((seq_cnt + 1)*max_chunk_size) < file_len:
        		packet_payload = source_file.read(max_chunk_size)
        		packet = struct.pack('%dsHH%ds' % (16, max_chunk_size), file_id, seq_cnt, seq_len, packet_payload)

        		txpacket = TxPacket()
        		raw_packet = txpacket.encode(payload=packet)
        		socket_tx_packets.send(raw_packet)

        		seq_cnt += 1

        	if (seq_cnt*max_chunk_size < file_len):
        		packet_data_len = file_len - (seq_cnt * max_chunk_size)
        		packet_payload = source_file.read(packet_data_len)
        		packet = struct.pack('%dsHH%ds' % (16, packet_data_len), file_id, seq_cnt, seq_len, packet_payload)

        		# print(len(packet))
        		# print(binascii.hexlify(packet))
        		txpacket = TxPacket()
        		raw_packet = txpacket.encode(payload=packet)
        		socket_tx_packets.send(raw_packet)

    elif(CMD_ID == CMD_PL_UPLOAD_FILE):
        #TODO
        pass

    elif(CMD_ID == CMD_PL_SINGLE_CAPTURE):
        #TODO
        pass

    elif(CMD_ID == CMD_PL_RUN_CALIBRATION):
        #TODO
        pass

    elif(CMD_ID == CMD_PL_SET_FPGA):
        #TODO
        pass

    elif(CMD_ID == CMD_PL_GET_FPGA):
        #TODO
        pass

    elif(CMD_ID == CMD_PL_SET_HK):
        #TODO
        pass

    elif(CMD_ID == CMD_PL_ECHO):
        echo_txpacket = TxPacket()
        raw = echo_txpacket.encode(payload=generic_control_packet.payload) #TBR

        echo_txpacket.decode(raw) #for Debug printing
        print(echo_txpacket) #Debug printing
        print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT))) #Debug printing
        print(raw) #Debug printing

        socket_tx_packets.send(raw) #send packet

    elif(CMD_ID == CMD_PL_NOOP):
        #TODO
        pass

    elif(CMD_ID == CMD_PL_DWNLINK_MODE):
        #TODO
        pass

    elif(CMD_ID == CMD_PL_DEBUG_MODE):
        #TODO
        pass

    else: #default
        #TODO
        pass

    # ~ if ipc_rxcompacket.payload == b'block':
        # ~ print ("SLEEPING FOR 60 SECONDS")
        # ~ time.sleep(30)
    # ~ elif ipc_rxcompacket.payload == b'getMap':
        # ~ print ("REQUESTING FPGA MAP")
        # ~ getFPGAmap()
