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
sys.path.append('/root/lib/')
sys.path.append('../lib/')
from options import *
#from options import FPGA_MAP_ANSWER_PORT, FPGA_MAP_REQUEST_PORT, TX_PACKETS_PORT, RX_CMD_PACKETS_PORT, MESSAGE_TIMEOUT, PAT_CONTROL_PORT, TEST_RESPONSE_PORT
from ipc_packets import FPGAMapRequestPacket, FPGAMapAnswerPacket, TxPacket, RxCommandPacket, PATControlPacket, HandlerHeartbeatPacket, CHHealthPacket
from zmqTxRx import recv_zmq, send_zmq, separate

# use PID as unique identifier for this progress
topic = str(os.getpid())
pid = os.getpid()

# parameters
PAT_MODE_ID = PAT_CMD_START_PAT #set PAT mode to default for execution w/o ground specified mode change

# ZeroMQ inter process communication
context = zmq.Context()

socket_FPGA_map_request = context.socket(zmq.PUB) #send messages on this port
socket_FPGA_map_request.connect("tcp://localhost:%s" % FPGA_MAP_REQUEST_PORT) #connect to specific address (localhost)

socket_test_response_packets = context.socket(zmq.PUB) #send messages on this port
socket_test_response_packets.connect("tcp://localhost:%s" % TEST_RESPONSE_PORT) #connect to specific address (localhost)

socket_pat_control = context.socket(zmq.PUB) #send messages on this port
socket_pat_control.connect("tcp://localhost:%s" % PAT_CONTROL_PORT) #connect to specific address (localhost)

socket_housekeeping = context.socket(zmq.PUB) #send messages on this port
socket_housekeeping.connect("tcp://localhost:%s" % CH_HEARTBEAT_PORT) #connect to specific address (localhost)

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

poller = zmq.Poller() #poll rx commands
poller.register(socket_rx_command_packets, zmq.POLLIN)

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
        raw = not_empty_ipc_txpacket.encode(APID = TLM_GET_FPGA, payload=ipc_fpgaaswpacket.read_data)
        not_empty_ipc_txpacket.decode(raw)
        print(not_empty_ipc_txpacket)

        print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT)))
        print(raw)
        print(ipc_fpgarqpacket_read)

        socket_tx_packets.send(raw)

def send_pat_command(socket_PAT_control, return_address, command, payload = ''):
    #Define Command Payload
    CMD_PAYLOAD = payload + '\0'
    assert len(CMD_PAYLOAD) <= PAT_CMD_PAYLOAD_SIZE #Ensure CMD_PAYLOAD is the right length
    if(len(CMD_PAYLOAD) < PAT_CMD_PAYLOAD_SIZE):
            for i in range(PAT_CMD_PAYLOAD_SIZE - len(CMD_PAYLOAD)):
                    CMD_PAYLOAD = CMD_PAYLOAD + '\0'   #append null padding
    assert CMD_PAYLOAD[len(CMD_PAYLOAD)-1] == '\0' #always terminate strings with null character ('\0') for c-code
    CMD_PAYLOAD = str(CMD_PAYLOAD).encode('ascii') #format to ascii

    ipc_patControlPacket = PATControlPacket()
    raw_patControlPacket = ipc_patControlPacket.encode(command,CMD_PAYLOAD) 
    send_zmq(socket_PAT_control, raw_patControlPacket)    
    return ipc_patControlPacket

def log_to_hk(payload):
    ipc_healthPacket = CHHealthPacket()
    raw = ipc_healthPacket.encode(pid, payload)
    send_zmq(socket_housekeeping, raw) 

while True:
    #send heartbeat to housekeeping
    ipc_heartbeatPacket = HandlerHeartbeatPacket()
    curr_time = time.time()
    raw_ipc_heartbeatPacket = ipc_heartbeatPacket.encode(pid, curr_time)
    print(ipc_heartbeatPacket) #Debug printing
    print ('SENDING to %s' % (socket_housekeeping.get_string(zmq.LAST_ENDPOINT))) #Debug printing
    send_zmq(socket_housekeeping, raw_ipc_heartbeatPacket)  

    #poll for received commands
    sockets = dict(poller.poll(CH_HEARTBEAT_PD - 1000)) #wait for 5 seconds in between heartbeats (CH_HEARTBEAT_PD = 6000)
    if socket_rx_command_packets in sockets and sockets[socket_rx_command_packets] == zmq.POLLIN:
        # get commands
        print ('RECEIVING on %s with TIMEOUT %d' % (socket_rx_command_packets.get_string(zmq.LAST_ENDPOINT), socket_rx_command_packets.get(zmq.RCVTIMEO)))
        message = recv_zmq(socket_rx_command_packets)

        # decode the package
        ipc_rxcompacket = RxCommandPacket()
        ipc_rxcompacket.decode(message)
        print (ipc_rxcompacket)
        print ('| got PAYLOAD %s' % (ipc_rxcompacket.payload))
        CMD_ID = ipc_rxcompacket.APID

        if(CMD_ID == CMD_PL_REBOOT):
            log_to_hk('ACK CMD PL_REBOOT')
            time.sleep(1) 
            os.system("sudo shutdown -r now") #reboot the RPi (TODO: Add other shutdown actions if needed)

        elif(CMD_ID == CMD_PL_ENABLE_TIME):
            #TODO: set mode to sync CPU clock with time at tone packet
            log_to_hk('ACK CMD PL_ENABLE_TIME')

        elif(CMD_ID == CMD_PL_DISABLE_TIME):
            #TODO: set mode to NOT sync CPU clock with time at tone packet
            log_to_hk('ACK CMD PL_DISABLE_TIME')

        elif(CMD_ID == CMD_PL_EXEC_FILE):
            raw_size = generic_control_packet.size - 3 #TBR
            output_to_file, file_out_num, file_path_len, raw_file_path = struct.unpack('BBB%ds'%raw_size,generic_control_packet.payload) #TBR
            file_path = raw_file_path.decode('utf-8') #TBR
            file_path = file_path[0:(file_path_len-1)] #Strip padding
            try:                
                if(output_to_file):
                    os.system(file_path + ' > ' + string(file_out_num) + '.log')
                    #send file...
                else:
                    os.system(file_path)

                log_to_hk('ACK CMD PL_EXEC_FILE')
            except:
                log_to_hk('ERROR CMD PL_EXEC_FILE: ' + traceback.format_exc()) 

        elif(CMD_ID == CMD_PL_LIST_FILE):
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
                raw = list_file_txpacket.encode(APID = TLM_LIST_FILE, payload = return_data) #TBR
                print(list_file_txpacket) #Debug printing
                print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT))) #Debug printing
                socket_tx_packets.send(raw) #send packet
                log_to_hk('ACK CMD PL_LIST_FILE')
            except:
                log_to_hk('ERROR CMD PL_LIST_FILE: ' + traceback.format_exc()) 

        elif(CMD_ID == CMD_PL_REQUEST_FILE):
            # simple dumb file chunking
            max_chunk_size = 4068
            file_path_len, raw_file_path = struct.unpack('B%ds'%raw_size,generic_control_packet.payload) #TBR
            file_name = raw_file_path.decode('utf-8') #TBR             
            with open(file_name, "rb") as source_file:
                file_id = hashlib.md5(file_name).digest()
                file_len = os.stat(file_name).st_size
                seq_len = math.ceil(float(file_len)/max_chunk_size)
                seq_cnt = 0
                while ((seq_cnt + 1)*max_chunk_size) < file_len:
                    packet_payload = source_file.read(max_chunk_size)
                    packet = struct.pack('%dsHH%ds' % (16, max_chunk_size), file_id, seq_cnt, seq_len, packet_payload)

                    txpacket = TxPacket()
                    raw_packet = txpacket.encode(APID = TLM_DL_FILE, payload = packet)
                    socket_tx_packets.send(raw_packet)

                    seq_cnt += 1

                if (seq_cnt*max_chunk_size < file_len):
                    packet_data_len = file_len - (seq_cnt * max_chunk_size)
                    packet_payload = source_file.read(packet_data_len)
                    packet = struct.pack('%dsHH%ds' % (16, packet_data_len), file_id, seq_cnt, seq_len, packet_payload)

                    # print(len(packet))
                    # print(binascii.hexlify(packet))
                    txpacket = TxPacket()
                    raw_packet = txpacket.encode(APID = TLM_DL_FILE, payload = packet)
                    socket_tx_packets.send(raw_packet)

            log_to_hk('ACK CMD PL_REQUEST_FILE')

        elif(CMD_ID == CMD_PL_UPLOAD_FILE):
            #TODO: receive file chunks and re-assemble file
            log_to_hk('ACK CMD PL_UPLOAD_FILE_TOSTAGING')

        elif(CMD_ID == CMD_PL_SET_PAT_MODE):
            pat_mode_cmd = struct.unpack('B', generic_control_packet.payload) #maps to PAT_CMD_START_PAT_OPEN_LOOP, PAT_CMD_START_PAT_STATIC_POINT, 
            if(pat_mode_cmd not in [PAT_CMD_START_PAT_OPEN_LOOP, PAT_CMD_START_PAT_STATIC_POINT, PAT_CMD_START_PAT_BUS_FEEDBACK]):
                log_to_hk('ERROR CMD PL_SET_PAT_MODE: Unrecognized PAT_MODE_ID = ' + string(pat_mode_cmd)) 
                PAT_MODE_ID = PAT_CMD_START_PAT #default PAT mode
            else:
                PAT_MODE_ID = pat_mode_cmd #execute with commanded PAT mode
                log_to_hk('ACK CMD PL_SET_PAT_MODE')

        elif(CMD_ID == CMD_PL_SINGLE_CAPTURE):
            exp_cmd = struct.unpack('I', generic_control_packet.payload) #TBR
            if(exp_cmd < 10):
                    print ('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = 10
            elif(exp_cmd > 10000000):
                    print ('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = 10000000

            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
            ipc_patControlPacket = send_pat_command(socket_PAT_control, string(pid), PAT_CMD_GET_IMAGE, str(exp_cmd))
            print(ipc_patControlPacket) #debug print
            log_to_hk('ACK CMD PL_SINGLE_CAPTURE') 
            #send image telemetry file...

        elif(CMD_ID == CMD_PL_CALIB_LASER_TEST):
            exp_cmd = struct.unpack('I', generic_control_packet.payload) #TBR
            if(exp_cmd < 10):
                    print ('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = 10
            elif(exp_cmd > 10000000):
                    print ('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = 10000000

            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
            ipc_patControlPacket = send_pat_command(socket_PAT_control, string(pid), PAT_CMD_CALIB_LASER_TEST, str(exp_cmd))
            print(ipc_patControlPacket) #debug print
            log_to_hk('ACK CMD PL_CALIB_LASER_TEST') 
            #send image telemetry file... 

        elif(CMD_ID == CMD_PL_FSM_TEST):
            exp_cmd = struct.unpack('I', generic_control_packet.payload) #TBR
            if(exp_cmd < 10):
                    print ('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = 10
            elif(exp_cmd > 10000000):
                    print ('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = 10000000

            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
            ipc_patControlPacket = send_pat_command(socket_PAT_control, string(pid), PAT_CMD_FSM_TEST, str(exp_cmd))
            print(ipc_patControlPacket) #debug print
            log_to_hk('ACK CMD PL_FSM_TEST') 
            #send image telemetry files...   

        elif(CMD_ID == CMD_PL_RUN_CALIBRATION):
            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
            ipc_patControlPacket = send_pat_command(socket_PAT_control, string(pid), PAT_CMD_CALIB_TEST)
            print(ipc_patControlPacket) #debug print
            log_to_hk('ACK CMD PL_RUN_CALIBRATION') 
            #send image telemetry files...    

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
            echo_raw_size = ipc_rxcompacket.size
            echo_raw = ipc_rxcompacket.payload
            echo_payload_tuple = struct.unpack('%ds'%echo_raw_size,echo_raw)
            echo_payload = echo_payload_tuple[0] #python returns a tuple by default
            raw = echo_txpacket.encode(APID = TLM_ECHO, payload = echo_payload) #TBR
            print(echo_txpacket) #Debug printing
            print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT))) #Debug printing
            socket_tx_packets.send(raw) #send packet
            log_to_hk('ACK CMD PL_ECHO') 

        elif(CMD_ID == CMD_PL_NOOP):
            log_to_hk('ACK CMD PL_NOOP') 

        elif(CMD_ID == CMD_PL_DWNLINK_MODE):
            #Start Main PAT Loop:
            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
            ipc_patControlPacket = send_pat_command(socket_PAT_control, string(pid), PAT_MODE_ID)
            print(ipc_patControlPacket) #debug print

            ###TODO: Lasercom experiment...

            #End PAT Process:
            # print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
            # ipc_patControlPacket = send_pat_command(socket_PAT_control, string(pid), PAT_CMD_END_PAT)
            # print(ipc_patControlPacket) #debug print
            #TODO: acknowledgement telemetry 
            log_to_hk('ACK CMD PL_DWNLINK_MODE') 

        elif(CMD_ID == CMD_PL_DEBUG_MODE):
            log_to_hk('ACK CMD PL_DEBUG_MODE') 
            #TODO...

        else: #default
            log_to_hk('ERROR: Unrecognized CMD_ID = ' + string(CMD_ID))  
