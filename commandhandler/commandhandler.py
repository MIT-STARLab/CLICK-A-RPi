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
import traceback
#importing options and functions
sys.path.append('/root/lib/')
sys.path.append('../lib/')
from options import *
from ipc_packets import FPGAMapRequestPacket, FPGAMapAnswerPacket, TxPacket, RxCommandPacket, PATControlPacket, HandlerHeartbeatPacket, CHHealthPacket, PATStatusPacket
from zmqTxRx import recv_zmq, separate
import ipc_helper
import fpga_map as mmap
from filehandling import *

# define fpga interface
fpga = ipc_helper.FPGAClientInterface()
power = mmap.Power(fpga) # power sub-interface

# use PID as unique identifier for this progress
topic = str(os.getpid())
pid = os.getpid()

# mode parameters
CH_MODE_GROUND_TEST = 0
CH_MODE_DEBUG = 1
CH_MODE_DOWNLINK = 2
CH_MODE_ID = CH_MODE_GROUND_TEST #default for ground testing (can replace with debug for flight)
PAT_MODE_ID = PAT_CMD_START_PAT #set PAT mode to default for execution w/o ground specified mode change
pat_mode_list = [PAT_CMD_START_PAT, PAT_CMD_START_PAT_OPEN_LOOP, PAT_CMD_START_PAT_STATIC_POINT, PAT_CMD_START_PAT_BUS_FEEDBACK, PAT_CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK, PAT_CMD_BCN_ALIGN]
pat_mode_names = ['Default', 'Open-Loop', 'Static Pointing', 'Default w/ Bus Feedback', 'Open-Loop w/ Bus Feedback', 'Beacon Alignment Mode']

# timing parameters
#ground test mode doesn't timeout for convenience during testing (can remove this mode for flight)
TIMEOUT_PD_DEBUG = 1800 #seconds, timeout period for debug mode (TBR)
TIMEOUT_PD_DOWNLINK = 900 #seconds, timeout period for debug mode (TBR)
UPDATE_PD_GROUND_TEST = 10 #seconds, time period for any repeated process commands needed during this mode
UPDATE_PD_DEBUG = 10 #seconds, time period for any repeated process commands needed during this mode
UPDATE_PD_DOWNLINK = 10 #seconds, time period for any repeated process commands needed during this mode

#PAT Status Flag List
pat_status_list = [PAT_STATUS_CAMERA_INIT, PAT_STATUS_STANDBY, PAT_STATUS_MAIN]

# ZeroMQ inter process communication
context = zmq.Context()

socket_pat_control = context.socket(zmq.PUB) #send messages on this port
socket_pat_control.bind("tcp://127.0.0.1:%s" % PAT_CONTROL_PORT) #connect to specific address (localhost)

socket_housekeeping = context.socket(zmq.PUB) #send messages on this port
socket_housekeeping.bind("tcp://127.0.0.1:%s" % CH_HEARTBEAT_PORT) #connect to specific address (localhost)

socket_tx_packets = context.socket(zmq.PUB)
socket_tx_packets.connect("tcp://localhost:%s" % TX_PACKETS_PORT)

print ("Pulling Rx Cmd Packets")
print ("on port {}".format(RX_CMD_PACKETS_PORT))
socket_rx_command_packets = context.socket(zmq.SUB)
socket_rx_command_packets.setsockopt(zmq.SUBSCRIBE, b'')
socket_rx_command_packets.connect("tcp://127.0.0.1:%s" % RX_CMD_PACKETS_PORT)
poller_rx_command_packets = zmq.Poller() #poll rx commands
poller_rx_command_packets.register(socket_rx_command_packets, zmq.POLLIN)

print ("Pulling PAT Status Packets")
print ("on port {}".format(PAT_STATUS_PORT))
socket_PAT_status = context.socket(zmq.SUB)
socket_PAT_status.bind("tcp://127.0.0.1:%s" % PAT_STATUS_PORT)
socket_PAT_status.setsockopt(zmq.SUBSCRIBE, b'')
poller_PAT_status = zmq.Poller()
poller_PAT_status.register(socket_PAT_status, zmq.POLLIN)

# socket_test_response_packets = context.socket(zmq.PUB) #send messages on this port
# socket_test_response_packets.connect("tcp://localhost:%s" % TEST_RESPONSE_PORT) #connect to specific address (localhost)

# socket_FPGA_map_request = context.socket(zmq.PUB) #send messages on this port
# socket_FPGA_map_request.connect("tcp://localhost:%s" % FPGA_MAP_REQUEST_PORT) #connect to specific address (localhost)

# print ("Subscribing to FPGA_MAP_ANSWER topic {}".format(topic))
# print ("on port {}".format(FPGA_MAP_ANSWER_PORT))
# socket_FPGA_map_answer = context.socket(zmq.SUB)
# #socket_FPGA_map_answer.setsockopt(zmq.SUBSCRIBE, topic.encode('ascii'))
# socket_FPGA_map_answer.setsockopt(zmq.SUBSCRIBE, struct.pack('I',pid))
# socket_FPGA_map_answer.setsockopt(zmq.RCVTIMEO, MESSAGE_TIMEOUT) # 5 second timeout on receive
# socket_FPGA_map_answer.connect ("tcp://localhost:%s" % FPGA_MAP_ANSWER_PORT)

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

def initialize_cal_laser():
        #Make sure heaters are on
        if(fpga.read_reg(mmap.PO3) != 85):
                power.heaters_on()
        #Make sure cal laser diode is on
        if(fpga.read_reg(mmap.CAL) != 85):
                power.calib_diode_on()
        #Set DAC
        fpga.write_reg(mmap.DAC_SETUP,1)
        fpga.write_reg(mmap.DAC_1_D, CAL_LASER_DAC_SETTING)
cal_laser_init = False #identifies if cal laser has been initialized yet or not

def cal_laser_on():
    if(cal_laser_init):
            power.calib_diode_on()
    else:
            initialize_cal_laser()
            cal_laser_init = True
    log_to_hk('CALIBRATION LASER ON')

def cal_laser_off():
    power.calib_diode_off()
    log_to_hk('CALIBRATION LASER OFF')

def start_camera():
    os.system('start camera') #calls camera.service to turn on camera
    log_to_hk('CAMERA ON')

def stop_camera():
    os.system('stop camera') #calls camera.service to turn off camera
    log_to_hk('CAMERA OFF')

def get_pat_status():
    #get pat status
    socks_status = dict(poller_PAT_status.poll(250)) #poll for 250 ms
    if socket_PAT_status in socks_status and socks_status[socket_PAT_status] == zmq.POLLIN:
        print('RECEIVING on %s' % socket_PAT_status.get_string(zmq.LAST_ENDPOINT))
        message = recv_zmq(socket_PAT_status)
        ipc_patStatusPacket = PATStatusPacket()
        return_addr, status_flag = ipc_patStatusPacket.decode(message) #decode the package
        received_status = True
    else:
        return_addr = -1
        status_flag = -1
        received_status = False

    return received_status, status_flag, return_addr

def stop_pat():
    print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
    ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_END_PROCESS)
    print(ipc_patControlPacket) #debug print
    cal_laser_off()
    stop_camera()

def start_pat(allow_camera_init = False):
    start_camera() #start camera
    os.system('pat') #run the /root/bin/pat executable
    for i in range(10): #try to get the status of pat
        received_status, status_flag, return_addr = get_pat_status()
        if(received_status):
            if(status_flag in pat_status_list):
                if(status_flag == PAT_STATUS_CAMERA_INIT):
                    log_to_hk('=PAT Process Started (PID: ' + str(return_addr) + '). Status: In Camera Initialization Loop') #start camera command failed
                    if(allow_camera_init):
                        return True
                    else:
                        stop_pat()
                        return False
                elif(status_flag == PAT_STATUS_STANDBY):
                    log_to_hk('PAT Process Started (PID: ' + str(return_addr) + '). Status: In Standby Loop')
                    return True
                elif(status_flag == PAT_STATUS_MAIN):
                    log_to_hk('PAT Process Started (PID: ' + str(return_addr) + '). Status: In Main Loop - skipped standby loop anomalously') #this should not happen
                    stop_pat()
                    return False
            else:
                log_to_hk('PAT Process Started (PID: ' + str(return_addr) + '). Status: Unrecognized')
                stop_pat()
                return False
    log_to_hk('PAT Process Unresponsive.')
    stop_pat()
    return False

#initialization
start_time = time.time() #default start_time is the execution time (debug or downlink mode commands overwrite this)
counter_ground_test = 0 #used to count the number of repetitive process tasks
counter_debug = 0 #used to count the number of repetitive process tasks
counter_downlink = 0 #used to count the number of repetitive process tasks
counter_heartbeat = 0 #used to count the number of repetitive process tasks

# #start PAT process
# pat_process_running = start_pat()
pat_process_running = False #test housekeeping output to COSMOS before trying PAT tests

#start command handling
while True:
    curr_time = time.time()
    elapsed_time = curr_time - start_time

    # check for timeouts and do any repetitive process tasks
    if((CH_MODE_ID == CH_MODE_GROUND_TEST) and (elapsed_time >= UPDATE_PD_GROUND_TEST*counter_ground_test)): #no timeout for ground testing
        #log_to_hk('CH_MODE_ID = CH_MODE_GROUND_TEST. Start Time: ' + str(start_time))
        counter_ground_test += 1

    if(CH_MODE_ID == CH_MODE_DEBUG):
        time_remaining = TIMEOUT_PD_DEBUG - elapsed_time
        if(time_remaining <= 0):
            log_to_hk('CH_MODE_ID = CH_MODE_DEBUG. Timeout Reached.')
            #Do pre-shutdown tasks
            stop_pat()
            break #exit main loop

        elif(elapsed_time >= UPDATE_PD_DEBUG*counter_debug):
            log_to_hk('CH_MODE_ID = CH_MODE_DEBUG. Time Remaining (sec): ' + str(time_remaining) + '. Start Time: ' + str(start_time))
            #TODO: do any repetitive process tasks
            counter_debug += 1

    if(CH_MODE_ID == CH_MODE_DOWNLINK):
        time_remaining = TIMEOUT_PD_DOWNLINK - elapsed_time
        if(time_remaining <= 0):
            log_to_hk('CH_MODE_ID = CH_MODE_DOWNLINK. Timeout Reached.')
            #Do pre-shutdown tasks
            stop_pat()
            break #exit main loop

        elif(elapsed_time >= UPDATE_PD_DOWNLINK*counter_downlink):
            log_to_hk('CH_MODE_ID = CH_MODE_DOWNLINK. Time Remaining (sec): ' + str(time_remaining) + '. Start Time: ' + str(start_time))
            #TODO: do any repetitive process tasks
            counter_downlink += 1

    #send heartbeat to housekeeping
    if(elapsed_time >= HK_CH_HEARTBEAT_PD*counter_heartbeat):
        ipc_heartbeatPacket = HandlerHeartbeatPacket()
        raw_ipc_heartbeatPacket = ipc_heartbeatPacket.encode(pid, curr_time)
        #print(ipc_heartbeatPacket) #Debug printing
        socket_housekeeping.send(raw_ipc_heartbeatPacket)
        counter_heartbeat += 1

    #poll for received commands
    sockets = dict(poller_rx_command_packets.poll(10)) #poll for 10 milliseconds
    if socket_rx_command_packets in sockets and sockets[socket_rx_command_packets] == zmq.POLLIN:
        # get commands
        # print ('RECEIVING on %s with TIMEOUT %d' % (socket_rx_command_packets.get_string(zmq.LAST_ENDPOINT), socket_rx_command_packets.get(zmq.RCVTIMEO)))
        message = recv_zmq(socket_rx_command_packets)

        # decode the package
        ipc_rxcompacket = RxCommandPacket()
        ipc_rxcompacket.decode(message)
        CMD_ID = ipc_rxcompacket.APID

        if(CMD_ID != APID_TIME_AT_TONE):
            #don't print the time at tone receives
            print (ipc_rxcompacket)
            print ('| got PAYLOAD %s' % (ipc_rxcompacket.payload))

        if(CMD_ID == APID_TIME_AT_TONE):
            if (TIME_SET_ENABLE > 0):
                tai_secs,_,_,_,_,_,_,_,_,_,_,_,_ = struct.unpack('!7LB4LB', ipc_rxcompacket.payload)
                set_time = time.gmtime(tai_secs)
                os.system("timedatectl set-time '%04d-%02d-%02d %02d:%02d:%02d'" % (set_time.tm_year,
                                                                                    set_time.tm_mon,
                                                                                    set_time.tm_mday,
                                                                                    set_time.tm_hour,
                                                                                    set_time.tm_min,
                                                                                    set_time.tm_sec))
                TIME_SET_ENABLE -= 1
            else:
                pass

        elif(CMD_ID == CMD_PL_REBOOT):
            log_to_hk('ACK CMD PL_REBOOT')
            time.sleep(1)
            os.system("sudo shutdown -r now") #reboot the RPi (TODO: Add other shutdown actions if needed)

        elif(CMD_ID == CMD_PL_ENABLE_TIME):
            TIME_SET_ENABLE = 1
            log_to_hk('ACK CMD PL_ENABLE_TIME')

        elif(CMD_ID == CMD_PL_EXEC_FILE):
            ex_raw_size = ipc_rxcompacket.size - 4
            output_to_file, file_out_num, file_path_len, file_path_payload = struct.unpack('!BBH%ds'%ex_raw_size, ipc_rxcompacket.payload)
            file_path = file_path_payload[0:(file_path_len-1)] #Strip padding
            try:
                if(output_to_file):
                    os.system(file_path + ' > /root/log/' + str(file_out_num) + '.log')
                    #send file...
                else:
                    os.system(file_path)

                log_to_hk('ACK CMD PL_EXEC_FILE')
            except:
                log_to_hk('ERROR CMD PL_EXEC_FILE: ' + traceback.format_exc())

        elif(CMD_ID == CMD_PL_LIST_FILE):
            list_raw_size = ipc_rxcompacket.size - 2
            directory_path_len, directory_path_payload = struct.unpack('!H%ds'%list_raw_size, ipc_rxcompacket.payload)
            directory_path = directory_path_payload[0:directory_path_len] #Strip any padding

            try:
                directory_listing = os.listdir(directory_path) #Get directory list
                #Convert list to single string for telemetry:
                return_data = ""
                for list_item in directory_listing:
                    return_data += (list_item + "\n")
                log_to_hk('ACK CMD PL_LIST_FILE')
            except:
                return_data = ("ERROR CMD PL_LIST_FILE: " + traceback.format_exc())
                log_to_hk(return_data)
            
            list_file_txpacket = TxPacket()
            raw = list_file_txpacket.encode(APID = TLM_LIST_FILE, payload = return_data) #TBR
            print (list_file_txpacket) #Debug printing
            print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT))) #Debug printing
            socket_tx_packets.send(raw) #send packet

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
            pat_mode_cmd = struct.unpack('B', ipc_rxcompacket.payload)
            if(pat_process_running):
                if(pat_mode_cmd in pat_mode_list):
                    PAT_MODE_ID = pat_mode_cmd #execute with commanded PAT mode
                    log_to_hk('ACK CMD PL_SET_PAT_MODE: PAT mode is ' + pat_mode_names[pat_mode_list == PAT_MODE_ID])
                else:
                    log_to_hk('ERROR CMD PL_SET_PAT_MODE: Unrecognized PAT mode command: ' + str(pat_mode_cmd) + '. PAT mode is ' + pat_mode_names[pat_mode_list == PAT_MODE_ID])
            else:
                log_to_hk('ERROR CMD PL_SET_PAT_MODE: PAT process is not running.')

        elif(CMD_ID == CMD_PL_SINGLE_CAPTURE):
            exp_cmd = struct.unpack('I', ipc_rxcompacket.payload) #TBR
            if(exp_cmd < 10):
                    log_to_hk('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = 10
            elif(exp_cmd > 10000000):
                    log_to_hk('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = 10000000
            if(pat_process_running):
                print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
                ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_GET_IMAGE, str(exp_cmd))
                print(ipc_patControlPacket) #debug print
                log_to_hk('ACK CMD PL_SINGLE_CAPTURE')
                #manage image telemetry file...
            else:
                log_to_hk('ERROR CMD PL_SINGLE_CAPTURE: PAT process is not running.')

        elif(CMD_ID == CMD_PL_CALIB_LASER_TEST):
            exp_cmd = struct.unpack('I', ipc_rxcompacket.payload) #TBR
            if(exp_cmd < 10):
                    log_to_hk('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = 10
            elif(exp_cmd > 10000000):
                    log_to_hk('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = 10000000
            if(pat_process_running):
                print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
                ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_CALIB_LASER_TEST, str(exp_cmd))
                print(ipc_patControlPacket) #debug print
                log_to_hk('ACK CMD PL_CALIB_LASER_TEST')
                #Manage image telemetry files...
            else:
                log_to_hk('ERROR CMD PL_CALIB_LASER_TEST: PAT process is not running.')

        elif(CMD_ID == CMD_PL_FSM_TEST):
            exp_cmd = struct.unpack('I', ipc_rxcompacket.payload) #TBR
            if(exp_cmd < 10):
                    log_to_hk('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = 10
            elif(exp_cmd > 10000000):
                    log_to_hk('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = 10000000
            if(pat_process_running):
                print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
                ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_FSM_TEST, str(exp_cmd))
                print(ipc_patControlPacket) #debug print
                log_to_hk('ACK CMD PL_FSM_TEST')
                #Manage image telemetry files...
            else:
                log_to_hk('ERROR CMD PL_FSM_TEST: PAT process is not running.')

        elif(CMD_ID == CMD_PL_RUN_CALIBRATION):
            if(pat_process_running):
                print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
                ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_CALIB_TEST)
                print(ipc_patControlPacket) #debug print
                log_to_hk('ACK CMD PL_RUN_CALIBRATION')
                #Manage image telemetry files...
            else:
                log_to_hk('ERROR CMD PL_RUN_CALIBRATION: PAT process is not running.')

        elif(CMD_ID == CMD_PL_PAT_TEST):
            if(pat_process_running):
                print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
                ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_MODE_ID, str(PAT_TEST_FLAG))
                print(ipc_patControlPacket) #debug print
                log_to_hk('ACK CMD PL_PAT_TEST')
                #Manage image telemetry files...
            else:
                log_to_hk('ERROR CMD PL_PAT_TEST: PAT process is not running.')

        elif(CMD_ID == CMD_PL_END_PAT_PROCESS):
            stop_pat()
            log_to_hk('ACK CMD PL_END_PAT_PROCESS')

        elif(CMD_ID == CMD_PL_RESTART_PAT_PROCESS):
            pat_process_running = start_pat()
            if(pat_process_running):
                log_to_hk('ACK CMD PL_RESTART_PAT_PROCESS')
            else:
                log_to_hk('ERROR CMD PL_RESTART_PAT_PROCESS')

        elif(CMD_ID == CMD_PL_SET_FPGA):
            set_fpga_raw_size = ipc_rxcompacket.size - 4
            rq_number, start_addr, num_registers, write_data = struct.unpack('!BHB%dI'%set_fpga_raw_size, ipc_rxcompacket.payload)
            print ('Request Number = ' + str(rq_number) + ', Start Address = ' + str(start_addr) + ', Num Registers = ' + str(num_registers) + ', Write Data = ' + str(write_data)) #debug print
            if(num_registers != len(write_data)):
                log_to_hk('ERROR CMD PL_SET_FPGA - Packet Error: expected number of registers (= ' + str(num_registers) +  ' not equal to data length (= ' + str(len(write_data)))
            else:
                fpga.write_reg(start_addr, write_data)
                check_write_data = fpga.read_reg(start_addr, num_registers)
                addresses = range(start_addr, start_addr+num_registers)
                return_message = ""
                num_errors = 0
                for i in range(num_registers):
                    if check_write_data[i] != write_data[i]:
                        return_message += ("REG: " + str(addresses[i]) + ", VAL = " + str(check_write_data[i]) + " != " + str(write_data[i]) + "\n")
                        num_errors += 1
                    else:
                        return_message += ("REG: " + str(addresses[i]) + ", VAL = " + str(check_write_data[i]) + "\n")
                if(num_errors == 0):
                    log_to_hk('ACK CMD PL_SET_FPGA. Request Number = ' + str(rq_number) + "\n" + return_message)
                else:
                    log_to_hk('ERROR CMD PL_SET_FPGA. Request Number = ' + str(rq_number) + "\n" + return_message)
            #TODO: add direct telemetry message instead of just housekeeping

            ###OLD FPGA Interface
            # #send fpga request
            # fpga_req_pkt = FPGAMapRequestPacket()
            # raw_fpga_req_pkt = fpga_req_pkt.encode(pid, rq_number, rw_flag, start_addr, num_registers, write_data)
            # socket_FPGA_map_request.send(raw_fpga_req_pkt)

            # #get fpga answer
            # fpga_answer_message = recv_zmq(socket_FPGA_map_answer)
            # ipc_fpgaaswpacket = FPGAMapAnswerPacket()
            # ipc_fpgaaswpacket.decode(fpga_answer_message)
            # print (ipc_fpgaaswpacket) #debug print
            # if(ipc_fpgaaswpacket.error):
            #     log_to_hk('ACK CMD CMD_PL_SET_FPGA. WRITE FAILURE. Request Number: ' + str(ipc_fpgaaswpacket.rq_number) + '. Start Address: ' + str(ipc_fpgaaswpacket.start_addr))
            # else:
            #     log_to_hk('ACK CMD CMD_PL_SET_FPGA. WRITE SUCCESS. Request Number: ' + str(ipc_fpgaaswpacket.rq_number) + '. Start Address: ' + str(ipc_fpgaaswpacket.start_addr))

        elif(CMD_ID == CMD_PL_GET_FPGA):
            rq_number, start_addr, num_registers = struct.unpack('!BHB', ipc_rxcompacket.payload)
            read_data = fpga.read_reg(start_addr, num_registers)
            read_data_len = len(read_data)
            if(num_registers != read_data_len):
                log_to_hk('ERROR CMD PL_GET_FPGA - Expected number of registers (= ' + str(num_registers) +  ' not equal to read data length (= ' + str(len(read_data)))
            #send on tx port
            fpga_read_payload = struct.pack('!BHB%dI'%read_data_len, rq_number, start_addr, read_data_len, *read_data)
            print (fpga_read_payload) #debug print
            fpga_read_txpacket = TxPacket()
            raw_fpga_read_txpacket = fpga_read_txpacket.encode(APID = TLM_GET_FPGA, payload = fpga_read_payload)
            socket_tx_packets.send(raw_fpga_read_txpacket) #send packet

            ###OLD FPGA Interface
            # #send fpga request
            # fpga_req_pkt = FPGAMapRequestPacket()
            # raw_fpga_req_pkt = fpga_req_pkt.encode(pid, rq_number, rw_flag, start_addr, num_registers)
            # socket_FPGA_map_request.send(raw_fpga_req_pkt)

            # #get fpga answer
            # fpga_answer_message = recv_zmq(socket_FPGA_map_answer)
            # ipc_fpgaaswpacket = FPGAMapAnswerPacket()
            # ipc_fpgaaswpacket.decode(fpga_answer_message)
            # print (ipc_fpgaaswpacket) #debug print
            # if(ipc_fpgaaswpacket.error):
            #     log_to_hk('ACK CMD CMD_PL_GET_FPGA. READ FAILURE. Request Number: ' + str(ipc_fpgaaswpacket.rq_number) + '. Start Address: ' + str(ipc_fpgaaswpacket.start_addr))
            # else:
            #     log_to_hk('ACK CMD CMD_PL_GET_FPGA. READ SUCCESS. Request Number: ' + str(ipc_fpgaaswpacket.rq_number) + '. Start Address: ' + str(ipc_fpgaaswpacket.start_addr))
            #     #send on tx port
            #     fpga_read_payload = struct.pack('!BHB%ds'%ipc_fpgaaswpacket.size, ipc_fpgaaswpacket.rq_number, ipc_fpgaaswpacket.start_addr, ipc_fpgaaswpacket.size, ipc_fpgaaswpacket.read_data)
            #     print (fpga_read_payload)
            #     fpga_read_txpacket = TxPacket()
            #     raw_fpga_read_txpacket = fpga_read_txpacket.encode(APID = TLM_GET_FPGA, payload = fpga_read_payload)
            #     socket_tx_packets.send(raw_fpga_read_txpacket) #send packet

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
            test_list = [GENERAL_SELF_TEST, LASER_SELF_TEST, PAT_SELF_TEST]
            test_names = ['GENERAL_SELF_TEST', 'LASER_SELF_TEST', 'PAT_SELF_TEST']
            if(test_id not in test_list):
                log_to_hk('ERROR CMD PL_SELF_TEST: Unrecognized test ID: ' + str(test_id))
            else:
                log_to_hk('ACK CMD PL_SELF_TEST: Test is ' + test_names[test_list == test_id])
                #execute test
                if(test_id == GENERAL_SELF_TEST):
                    #Execute general self test script
                    run_test_script = 'python /root/test/general_functionality_test.py'
                    try:
                        os.system(run_test_script + ' > /root/log/' + str(file_out_num) + '.log') #TBR output file
                        #file management...
                    except:
                        log_to_hk('ERROR CMD PL_SELF_TEST - GENERAL_SELF_TEST: ' + traceback.format_exc())

                elif(test_id == LASER_SELF_TEST):
                    #Execute laser self test script
                    run_test_script = 'python /root/test/automated_laser_checks.py'
                    try:
                        os.system(run_test_script + ' > /root/log/' + str(file_out_num) + '.log') #TBR output file
                        #file management...
                    except:
                        log_to_hk('ERROR CMD PL_SELF_TEST - LASER_SELF_TEST: ' + traceback.format_exc())

                elif(test_id == PAT_SELF_TEST):
                    if(pat_process_running):
                        #execute PAT self test
                        print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
                        ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_SELF_TEST)
                        print(ipc_patControlPacket) #debug print
                    elif(start_pat(allow_camera_init = True)):
                        #Check if camera failure is to blame and run self test in camera initialization loop
                        print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
                        ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_CMD_SELF_TEST)
                        print(ipc_patControlPacket) #debug print
                        time.sleep(5) #TBR
                        stop_pat()
                    else:
                        log_to_hk('PAT process initialization not working.')

        elif(CMD_ID == CMD_PL_DWNLINK_MODE):
            start_time = time.time()
            CH_MODE_ID = CH_MODE_DOWNLINK
            log_to_hk('ACK CMD PL_DWNLINK_MODE with start time: ' + start_time)

            if(pat_process_running):
                #Start Main PAT Loop:
                print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
                ipc_patControlPacket = send_pat_command(socket_PAT_control, PAT_MODE_ID, str(PAT_FLIGHT_FLAG))
                print(ipc_patControlPacket) #debug print
            else:
                log_to_hk('ERROR CMD PL_DWNLINK_MODE: PAT process is not running.')

            ###TODO: add any other lasercom experiment process start-up tasks

        elif(CMD_ID == CMD_PL_DEBUG_MODE):
            start_time = time.time()
            CH_MODE_ID = CH_MODE_DEBUG
            log_to_hk('ACK CMD PL_DEBUG_MODE with start time: ' + start_time)

            if(not pat_process_running):
                log_to_hk('ERROR CMD PL_DEBUG_MODE: PAT process is not running.')

            ###TODO: add any other debug process start-up tasks

        else: #default
            log_to_hk('ERROR: Unrecognized CMD_ID = ' + str(CMD_ID))
