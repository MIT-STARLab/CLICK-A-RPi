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
from ipc_packets import FPGAMapRequestPacket, FPGAMapAnswerPacket, TxPacket, RxCommandPacket, PATControlPacket, HandlerHeartbeatPacket, CHHealthPacket
from zmqTxRx import recv_zmq, separate

from filehandling import *

# use PID as unique identifier for this progress
topic = str(os.getpid())
pid = os.getpid()

# mode parameters
CH_MODE_GROUND_TEST = 0
CH_MODE_DEBUG = 1
CH_MODE_DOWNLINK = 2
CH_MODE_ID = CH_MODE_GROUND_TEST #default for ground testing (can replace with debug for flight)
PAT_MODE_ID = PAT_CMD_START_PAT #set PAT mode to default for execution w/o ground specified mode change

# timing parameters
#ground test mode doesn't timeout for convenience during testing (can remove this mode for flight)
TIMEOUT_PD_DEBUG = 1800 #seconds, timeout period for debug mode (TBR)
TIMEOUT_PD_DOWNLINK = 900 #seconds, timeout period for debug mode (TBR)
UPDATE_PD_GROUND_TEST = 10 #seconds, time period for any repeated process commands needed during this mode
UPDATE_PD_DEBUG = 10 #seconds, time period for any repeated process commands needed during this mode
UPDATE_PD_DOWNLINK = 10 #seconds, time period for any repeated process commands needed during this mode

#other parameters
DEFAULT_CALIB_EXP = 25 #microseconds, default calibration laser exposure time for self-test

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

def send_pat_command(socket_PAT_control, command, payload = ''):
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
    socket_PAT_control.send(raw_patControlPacket)
    return ipc_patControlPacket

def log_to_hk(payload):
    print (payload) #debug printing
    ipc_healthPacket = CHHealthPacket()
    raw = ipc_healthPacket.encode(pid, payload)
    socket_housekeeping.send(raw)

start_time = time.time() #default start_time is the execution time (debug or downlink mode commands overwrite this)
counter_ground_test = 0 #used to count the number of repetitive process tasks
counter_debug = 0 #used to count the number of repetitive process tasks
counter_downlink = 0 #used to count the number of repetitive process tasks
counter_heartbeat = 0 #used to count the number of repetitive process tasks
while True:
    curr_time = time.time()
    elapsed_time = curr_time - start_time

    # check for timeouts and do any repetitive process tasks
    if((CH_MODE_ID == CH_MODE_GROUND_TEST) and (elapsed_time >= UPDATE_PD_GROUND_TEST*counter_ground_test)): #no timeout for ground testing
        log_to_hk('CH_MODE_ID = CH_MODE_GROUND_TEST. Start Time: ' + str(start_time))
        counter_ground_test += 1

    if(CH_MODE_ID == CH_MODE_DEBUG):
        time_remaining = TIMEOUT_PD_DEBUG - elapsed_time
        if(time_remaining <= 0):
            log_to_hk('CH_MODE_ID = CH_MODE_DEBUG. Timeout Reached.')
            #TODO: do any pre-shutdown tasks
            break #exit main loop

        elif(elapsed_time >= UPDATE_PD_DEBUG*counter_debug):
            log_to_hk('CH_MODE_ID = CH_MODE_DEBUG. Time Remaining (sec): ' + str(time_remaining) + '. Start Time: ' + str(start_time))
            #TODO: do any repetitive process tasks
            counter_debug += 1

    if(CH_MODE_ID == CH_MODE_DOWNLINK):
        time_remaining = TIMEOUT_PD_DOWNLINK - elapsed_time
        if(time_remaining <= 0):
            log_to_hk('CH_MODE_ID = CH_MODE_DOWNLINK. Timeout Reached.')
            #TODO: do any pre-shutdown tasks
            break #exit main loop

        elif(elapsed_time >= UPDATE_PD_DOWNLINK*counter_downlink):
            log_to_hk('CH_MODE_ID = CH_MODE_DOWNLINK. Time Remaining (sec): ' + str(time_remaining) + '. Start Time: ' + str(start_time))
            #TODO: do any repetitive process tasks
            counter_downlink += 1

    #send heartbeat to housekeeping
    if(elapsed_time >= HK_CH_HEARTBEAT_PD*counter_heartbeat):
        ipc_heartbeatPacket = HandlerHeartbeatPacket()
        raw_ipc_heartbeatPacket = ipc_heartbeatPacket.encode(pid, curr_time)
        print(ipc_heartbeatPacket) #Debug printing
        socket_housekeeping.send(raw_ipc_heartbeatPacket)
        counter_heartbeat += 1

    #poll for received commands
    sockets = dict(poller.poll(10)) #poll for 10 milliseconds
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
            ex_raw_size = ipc_rxcompacket.size - 4
            output_to_file, file_out_num, file_path_len, file_path_payload = struct.unpack('!BBH%ds'%ex_raw_size, ipc_rxcompacket.payload)
            file_path = file_path_payload[0:(file_path_len-1)] #Strip padding
            try:
                if(output_to_file):
                    os.system(file_path + ' > /root/log/' + string(file_out_num) + '.log')
                    #send file...
                else:
                    os.system(file_path)

                log_to_hk('ACK CMD PL_EXEC_FILE')
            except:
                log_to_hk('ERROR CMD PL_EXEC_FILE: ' + traceback.format_exc())

        elif(CMD_ID == CMD_PL_LIST_FILE):
            list_raw_size = ipc_rxcompacket.size - 2
            directory_path_len, directory_path_payload = struct.unpack('!H%ds'%list_raw_size, ipc_rxcompacket.payload)
            directory_path = directory_path_payload[0:(directory_path_len-1)] #Strip padding
            try:
                directory_listing = os.listdir(directory_path) #Get directory list
                #Convert list to single string for telemetry:
                return_data = ''
                for list_item in directory_listing:
                    return_data = return_data + list_item + '\n' #TBR separator type

                list_file_txpacket = TxPacket()
                raw = list_file_txpacket.encode(APID = TLM_LIST_FILE, payload = return_data) #TBR
                print (list_file_txpacket) #Debug printing
                print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT))) #Debug printing
                socket_tx_packets.send(raw) #send packet
                log_to_hk('ACK CMD PL_LIST_FILE')
            except:
                log_to_hk('ERROR CMD PL_LIST_FILE: ' + traceback.format_exc())

        elif(CMD_ID == CMD_PL_REQUEST_FILE):
            send_file_chunks(ipc_rxcompacket, socket_tx_packets)
            log_to_hk('ACK CMD PL_REQUEST_FILE')

        elif(CMD_ID == CMD_PL_UPLOAD_FILE):
            receive_file_chunk(ipc_rxcompacket)
            log_to_hk('ACK CMD PL_UPLOAD_FILE')

        elif(CMD_ID == CMD_PL_ASSEMBLE_FILE):
            assemble_file(ipc_rxcompacket, socket_tx_packets)
            log_to_hk('ACK CMD PL_ASSEMBLE_FILE')

        elif(CMD_ID == CMD_PL_VALIDATE_FILE):
            validate_file(ipc_rxcompacket)
            log_to_hk('ACK CMD PL_VALIDATE_FILE')

        elif(CMD_ID == CMD_PL_MOVE_FILE):
            move_file(ipc_rxcompacket)
            log_to_hk('ACK CMD PL_MOVE_FILE')

        elif(CMD_ID == CMD_PL_DELETE_FILE):
            del_file(ipc_rxcompacket)
            log_to_hk('ACK CMD PL_DELETE_FILE')

        elif(CMD_ID == CMD_PL_SET_PAT_MODE):
            pat_mode_cmd = struct.unpack('B', ipc_rxcompacket.payload) #maps to PAT_CMD_START_PAT_OPEN_LOOP, PAT_CMD_START_PAT_STATIC_POINT,
            pat_mode_list = [PAT_CMD_START_PAT, PAT_CMD_START_PAT_OPEN_LOOP, PAT_CMD_START_PAT_STATIC_POINT, PAT_CMD_START_PAT_BUS_FEEDBACK]
            pat_mode_names = ['Default', 'Open-Loop', 'Static Pointing', 'Bus Feedback']
            if(pat_mode_cmd not in pat_mode_list):
                log_to_hk('ERROR CMD PL_SET_PAT_MODE: Unrecognized PAT mode command: ' + string(pat_mode_cmd) + '. PAT mode is ' + pat_mode_names[pat_mode_list == PAT_MODE_ID])
            else:
                PAT_MODE_ID = pat_mode_cmd #execute with commanded PAT mode
                log_to_hk('ACK CMD PL_SET_PAT_MODE: PAT mode is ' + pat_mode_names[pat_mode_list == PAT_MODE_ID])

        elif(CMD_ID == CMD_PL_SINGLE_CAPTURE):
            exp_cmd = struct.unpack('I', ipc_rxcompacket.payload) #TBR
            if(exp_cmd < 10):
                    log_to_hk('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = 10
            elif(exp_cmd > 10000000):
                    log_to_hk('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = 10000000

            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
            ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_GET_IMAGE, str(exp_cmd))
            print(ipc_patControlPacket) #debug print
            log_to_hk('ACK CMD PL_SINGLE_CAPTURE')
            #send image telemetry file...

        elif(CMD_ID == CMD_PL_CALIB_LASER_TEST):
            exp_cmd = struct.unpack('I', ipc_rxcompacket.payload) #TBR
            if(exp_cmd < 10):
                    log_to_hk('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = 10
            elif(exp_cmd > 10000000):
                    log_to_hk('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = 10000000

            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
            ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_CALIB_LASER_TEST, str(exp_cmd))
            print(ipc_patControlPacket) #debug print
            log_to_hk('ACK CMD PL_CALIB_LASER_TEST')
            #send image telemetry files...

        elif(CMD_ID == CMD_PL_FSM_TEST):
            exp_cmd = struct.unpack('I', ipc_rxcompacket.payload) #TBR
            if(exp_cmd < 10):
                    log_to_hk('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = 10
            elif(exp_cmd > 10000000):
                    log_to_hk('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = 10000000

            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
            ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_FSM_TEST, str(exp_cmd))
            print(ipc_patControlPacket) #debug print
            log_to_hk('ACK CMD PL_FSM_TEST')
            #send image telemetry files...

        elif(CMD_ID == CMD_PL_RUN_CALIBRATION):
            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
            ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_CALIB_TEST)
            print(ipc_patControlPacket) #debug print
            log_to_hk('ACK CMD PL_RUN_CALIBRATION')
            #send image telemetry files...

        elif(CMD_ID == CMD_PL_SET_FPGA):
            sf_raw_size = ipc_rxcompacket.size - 4
            rq_number, start_addr, num_registers, write_data = struct.unpack('!BHB%dI'%sf_raw_size, ipc_rxcompacket.payload)
            rw_flag = 1

            #send fpga request
            fpga_req_pkt = FPGAMapRequestPacket()
            raw_fpga_req_pkt = fpga_req_pkt.encode(pid, rq_number, rw_flag, start_addr, num_registers, write_data)
            socket_FPGA_map_request.send(raw_fpga_req_pkt)

            #get fpga answer
            fpga_answer_message = recv_zmq(socket_FPGA_map_answer)
            ipc_fpgaaswpacket = FPGAMapAnswerPacket()
            ipc_fpgaaswpacket.decode(fpga_answer_message)
            print (ipc_fpgaaswpacket) #debug print
            if(ipc_fpgaaswpacket.error):
                log_to_hk('ACK CMD CMD_PL_SET_FPGA. WRITE FAILURE. Request Number: ' + str(ipc_fpgaaswpacket.rq_number) + '. Start Address: ' + str(ipc_fpgaaswpacket.start_addr))
            else:
                log_to_hk('ACK CMD CMD_PL_SET_FPGA. WRITE SUCCESS. Request Number: ' + str(ipc_fpgaaswpacket.rq_number) + '. Start Address: ' + str(ipc_fpgaaswpacket.start_addr))


        elif(CMD_ID == CMD_PL_GET_FPGA):
            rq_number, start_addr, num_registers = struct.unpack('!BHB', ipc_rxcompacket.payload)
            rw_flag = 0

            #send fpga request
            fpga_req_pkt = FPGAMapRequestPacket()
            raw_fpga_req_pkt = fpga_req_pkt.encode(pid, rq_number, rw_flag, start_addr, num_registers)
            socket_FPGA_map_request.send(raw_fpga_req_pkt)

            #get fpga answer
            fpga_answer_message = recv_zmq(socket_FPGA_map_answer)
            ipc_fpgaaswpacket = FPGAMapAnswerPacket()
            ipc_fpgaaswpacket.decode(fpga_answer_message)
            print (ipc_fpgaaswpacket) #debug print
            if(ipc_fpgaaswpacket.error):
                log_to_hk('ACK CMD CMD_PL_GET_FPGA. READ FAILURE. Request Number: ' + str(ipc_fpgaaswpacket.rq_number) + '. Start Address: ' + str(ipc_fpgaaswpacket.start_addr))
            else:
                log_to_hk('ACK CMD CMD_PL_GET_FPGA. READ SUCCESS. Request Number: ' + str(ipc_fpgaaswpacket.rq_number) + '. Start Address: ' + str(ipc_fpgaaswpacket.start_addr))
                #send on tx port
                fpga_read_payload = struct.pack('!BHB%ds'%ipc_fpgaaswpacket.size, ipc_fpgaaswpacket.rq_number, ipc_fpgaaswpacket.start_addr, ipc_fpgaaswpacket.size, ipc_fpgaaswpacket.read_data)
                print (fpga_read_payload)
                fpga_read_txpacket = TxPacket()
                raw_fpga_read_txpacket = fpga_read_txpacket.encode(APID = TLM_GET_FPGA, payload = fpga_read_payload)
                socket_tx_packets.send(raw_fpga_read_txpacket) #send packet

        elif(CMD_ID == CMD_PL_SET_HK):
            log_to_hk('ACK CMD PL_SET_HK')
            #TODO

        elif(CMD_ID == CMD_PL_ECHO):
            echo_raw_size = ipc_rxcompacket.size
            echo_payload = struct.unpack('!%ds'%echo_raw_size, ipc_rxcompacket.payload)[0] #decode the raw payload bytes; since there's only one return type, take the first element of the return tuple
            echo_txpacket = TxPacket()
            raw_echo_txpacket = echo_txpacket.encode(APID = TLM_ECHO, payload = echo_payload)
            print (echo_payload) #debug printing
            print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT))) #Debug printing
            socket_tx_packets.send(raw_echo_txpacket) #send packet
            log_to_hk('ACK CMD PL_ECHO')

        elif(CMD_ID == CMD_PL_NOOP):
            log_to_hk('ACK CMD PL_NOOP')

        elif(CMD_ID == CMD_PL_SELF_TEST):
            test_id = struct.unpack('B', ipc_rxcompacket.payload)
            test_list = [TEST_PAT_HW]
            test_names = ['PAT HW']
            if(test_id not in test_list):
                log_to_hk('ERROR CMD PL_SELF_TEST: Unrecognized test ID: ' + string(test_id))
            else:
                log_to_hk('ACK CMD PL_SELF_TEST: Test is ' + test_names[test_list == test_id])
                #execute test
                if(test_id == TEST_PAT_HW):
                    #execute calib laser test
                    print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
                    ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_CALIB_LASER_TEST, str(DEFAULT_CALIB_EXP))
                    print(ipc_patControlPacket) #debug print
                    time.sleep(10) #give test some time to complete
                    #send image telemetry files...

                    #execute fsm test
                    print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
                    ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_FSM_TEST, str(DEFAULT_CALIB_EXP))
                    print(ipc_patControlPacket) #debug print
                    time.sleep(10) #give test some time to complete
                    #send image telemetry files...

                    #execute stand alone calibration test
                    print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
                    ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_CALIB_TEST)
                    print(ipc_patControlPacket) #debug print
                    time.sleep(10) #give test some time to complete
                    #send image telemetry files...

                #elif(test_id == ...):

        elif(CMD_ID == CMD_PL_DWNLINK_MODE):
            start_time = time.time()
            CH_MODE_ID = CH_MODE_DOWNLINK
            log_to_hk('ACK CMD PL_DWNLINK_MODE with start time: ' + start_time)

            #Start Main PAT Loop:
            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
            ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_MODE_ID)
            print(ipc_patControlPacket) #debug print

            ###TODO: add any other lasercom experiment process start-up tasks

        elif(CMD_ID == CMD_PL_DEBUG_MODE):
            start_time = time.time()
            CH_MODE_ID = CH_MODE_DEBUG
            log_to_hk('ACK CMD PL_DEBUG_MODE with start time: ' + start_time)

            ###TODO: add any other debug process start-up tasks

        else: #default
            log_to_hk('ERROR: Unrecognized CMD_ID = ' + string(CMD_ID))
