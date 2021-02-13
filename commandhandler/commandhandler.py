#!/usr/bin/env python

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
from ipc_packets import FPGAMapRequestPacket, FPGAMapAnswerPacket, TxPacket, RxCommandPacket, PATControlPacket, CHHeartbeatPacket, HKControlPacket, PATStatusPacket
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
pat_status_names = ['CAMERA INIT', 'STANDBY', 'MAIN']

# ZeroMQ inter process communication
context = zmq.Context()

socket_PAT_control = context.socket(zmq.PUB) #send messages on this port
socket_PAT_control.bind("tcp://127.0.0.1:%s" % PAT_CONTROL_PORT) #connect to specific address (localhost)

socket_hk_heartbeat = context.socket(zmq.PUB) #send messages on this port
socket_hk_heartbeat.bind("tcp://127.0.0.1:%s" % CH_HEARTBEAT_PORT) #connect to specific address (localhost)

socket_hk_control = context.socket(zmq.PUB) #send messages on this port
socket_hk_control.bind("tcp://127.0.0.1:%s" % HK_CONTROL_PORT) #connect to specific address (localhost)

socket_tx_packets = context.socket(zmq.PUB)
socket_tx_packets.connect("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)

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
    print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
    print(ipc_patControlPacket) #debug print
    socket_PAT_control.send(raw_patControlPacket)
    return ipc_patControlPacket

def log_to_hk(payload):
    print(payload) #debug printing
    ipc_HKPacket = HKControlPacket()
    raw = ipc_HKPacket.encode(pid, HK_CONTROL_LOG, payload)
    socket_hk_control.send(raw)

def ack_to_hk(cmd_id, status):
    ipc_HKPacket = HKControlPacket()
    payload = struct.pack('HH', cmd_id, status)
    raw = ipc_HKPacket.encode(pid, HK_CONTROL_ACK, payload)
    socket_hk_control.send(raw)

def initialize_cal_laser():
    #Make sure heaters are on
    if(fpga.read_reg(mmap.PO3) != 85):
        power.heaters_on()
    #Make sure cal laser diode is on
    if(fpga.read_reg(mmap.CAL) != 85):
        power.calib_diode_on()   
    log_to_hk('CALIBRATION LASER ON')    
    #Set DAC
    fpga.write_reg(mmap.DAC_SETUP,1)
    fpga.write_reg(mmap.DAC_1_D, CAL_LASER_DAC_SETTING)
    log_to_hk('CALIBRATION LASER DAC INITIALIZED')
    #Make sure cal laser is off
    power.calib_diode_off()
    log_to_hk('CALIBRATION LASER OFF')

def start_camera():
    os.system('systemctl --user start camera') #calls camera.service to turn on camera
    log_to_hk('CAMERA ON')

def stop_camera():
    os.system('systemctl --user stop camera') #calls camera.service to turn off camera
    log_to_hk('CAMERA OFF')

def stop_pat():
    send_pat_command(socket_PAT_control, PAT_CMD_END_PROCESS) #this isn't necessary if PAT is running as a service
    os.system("systemctl --user stop pat") #stop the pat service
    log_to_hk('PAT STOPPED')

def restart_pat():
    stop_pat()
    time.sleep(1)
    os.system("systemctl --user restart pat") #restart the pat service
    log_to_hk('PAT RESTARTED')

def get_pat_status():
    #get pat status
    socks_status = dict(poller_PAT_status.poll(250)) #poll for 250 ms
    if socket_PAT_status in socks_status and socks_status[socket_PAT_status] == zmq.POLLIN:
        #print('RECEIVING on %s' % socket_PAT_status.get_string(zmq.LAST_ENDPOINT))
        message = recv_zmq(socket_PAT_status)
        ipc_patStatusPacket = PATStatusPacket()
        return_addr, status_flag = ipc_patStatusPacket.decode(message) #decode the package
        received_status = True
    else:
        return_addr = -1
        status_flag = -1
        received_status = False

    return received_status, status_flag, return_addr

def update_pat_status(status_flag):
    #get pat status
    received_status, new_status_flag, _ = get_pat_status()
    if(received_status):
        status_flag = new_status_flag

    return status_flag

#intialize PAT status
for i in range(10):
    pat_received_status, pat_status_flag, pat_return_addr = get_pat_status()
    if(pat_received_status):
        log_to_hk('Connected to PAT process at PID = ' + str(pid))
        break
if(not pat_received_status):
    log_to_hk('WARNING: PAT process unresponsive at CH (PID = ' + str(pid) + ') startup.')

def pat_status_is(pat_status_check):
    if(pat_status_flag in pat_status_list):
        log_to_hk('PAT Process Running (PID: ' + str(pat_return_addr) + '). Status: ' + pat_status_names[pat_status_list.index(pat_status_flag)])
        print('status_flag: ', pat_status_flag)
        print('pat_status_check: ', pat_status_check)
        print('bool: ', (pat_status_flag == pat_status_check))
        return (pat_status_flag == pat_status_check)
    else:
        log_to_hk('PAT Process Running (PID: ' + str(pat_return_addr) + '). Status: Unrecognized')
        return False

#initialization
start_time = time.time() #default start_time is the execution time (debug or downlink mode commands overwrite this)
counter_ground_test = 0 #used to count the number of repetitive process tasks
counter_debug = 0 #used to count the number of repetitive process tasks
counter_downlink = 0 #used to count the number of repetitive process tasks
counter_heartbeat = 0 #used to count the number of repetitive process tasks

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
        ipc_heartbeatPacket = CHHeartbeatPacket()
        raw_ipc_heartbeatPacket = ipc_heartbeatPacket.encode(pid, curr_time)
        #print(ipc_heartbeatPacket) #Debug printing
        socket_hk_heartbeat.send(raw_ipc_heartbeatPacket)
        counter_heartbeat += 1

    #update PAT status
    pat_status_flag = update_pat_status(pat_status_flag)

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
                #print('len(ipc_rxcompacket.payload): ', len(ipc_rxcompacket.payload))
                #print('ipc_rxcompacket.payload: ', ipc_rxcompacket.payload)
                tai_secs,_,_,_,_,_,_,_,_,_,_,_,_ = struct.unpack('!L6QB4QB', ipc_rxcompacket.payload)
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
            ack_to_hk(CMD_PL_REBOOT, CMD_ACK)
            time.sleep(1)
            os.system("shutdown -r now") #reboot the RPi (TODO: Add other shutdown actions if needed)

        elif(CMD_ID == CMD_PL_ENABLE_TIME):
            TIME_SET_ENABLE = 1
            log_to_hk('ACK CMD PL_ENABLE_TIME')
            ack_to_hk(CMD_PL_ENABLE_TIME, CMD_ACK)

        elif(CMD_ID == CMD_PL_EXEC_FILE):
            ex_raw_size = ipc_rxcompacket.size - 4
            output_to_file, file_out_num, file_path_len, file_path_payload = struct.unpack('!BBH%ds'%ex_raw_size, ipc_rxcompacket.payload)
            file_path = file_path_payload[0:file_path_len] #Strip padding
            try:
                if(output_to_file):
                    os.system(file_path + ' > /root/log/' + str(file_out_num) + '.log')
                    #send file...
                else:
                    os.system(file_path)

                log_to_hk('ACK CMD PL_EXEC_FILE')
                ack_to_hk(CMD_PL_EXEC_FILE, CMD_ACK)
            except:
                log_to_hk('ERROR CMD PL_EXEC_FILE: ' + traceback.format_exc())
                ack_to_hk(CMD_PL_EXEC_FILE, CMD_ERR)

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
                ack_to_hk(CMD_PL_LIST_FILE, CMD_ACK)
            except:
                return_data = ("ERROR CMD PL_LIST_FILE: " + traceback.format_exc())
                log_to_hk(return_data)
                ack_to_hk(CMD_PL_LIST_FILE, CMD_ERR)

            list_file_txpacket = TxPacket()
            raw = list_file_txpacket.encode(APID = TLM_LIST_FILE, payload = return_data) #TBR
            print (list_file_txpacket) #Debug printing
            print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT))) #Debug printing
            socket_tx_packets.send(raw) #send packet

        elif(CMD_ID == CMD_PL_AUTO_DOWNLINK_FILE):
            auto_downlink_file(ipc_rxcompacket, socket_tx_packets)
            log_to_hk('ACK CMD PL_AUTO_DOWNLINK_FILE')
            ack_to_hk(CMD_PL_AUTO_DOWNLINK_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_DISASSEMBLE_FILE):
            disassemble_file(ipc_rxcompacket, socket_tx_packets)
            log_to_hk('ACK CMD PL_DISASSEMBLE_FILE')
            ack_to_hk(CMD_PL_DISASSEMBLE_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_REQUEST_FILE):
            request_file(ipc_rxcompacket, socket_tx_packets)
            log_to_hk('ACK CMD PL_REQUEST_FILE')
            ack_to_hk(CMD_PL_REQUEST_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_UPLINK_FILE):
            uplink_file(ipc_rxcompacket, socket_tx_packets)
            log_to_hk('ACK CMD PL_UPLINK_FILE')
            ack_to_hk(CMD_PL_UPLINK_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_ASSEMBLE_FILE):
            assemble_file(ipc_rxcompacket, socket_tx_packets)
            log_to_hk('ACK CMD PL_ASSEMBLE_FILE')
            ack_to_hk(CMD_PL_ASSEMBLE_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_VALIDATE_FILE):
            validate_file(ipc_rxcompacket, socket_tx_packets)
            log_to_hk('ACK CMD PL_VALIDATE_FILE')
            ack_to_hk(CMD_PL_VALIDATE_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_MOVE_FILE):
            move_file(ipc_rxcompacket, socket_tx_packets)
            log_to_hk('ACK CMD PL_MOVE_FILE')
            ack_to_hk(CMD_PL_MOVE_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_DELETE_FILE):
            del_file(ipc_rxcompacket, socket_tx_packets)
            log_to_hk('ACK CMD PL_DELETE_FILE')
            ack_to_hk(CMD_PL_DELETE_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_AUTO_ASSEMBLE_FILE):
            auto_assemble_file(ipc_rxcompacket, socket_tx_packets)
            log_to_hk('ACK CMD PL_AUTO_ASSEMBLE_FILE')
            ack_to_hk(CMD_PL_AUTO_ASSEMBLE_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_SET_PAT_MODE):
            pat_mode_cmd_tuple = struct.unpack('!B', ipc_rxcompacket.payload)
            pat_mode_cmd = pat_mode_cmd_tuple[0]
            if(pat_status_is(PAT_STATUS_STANDBY)):
                if(pat_mode_cmd in pat_mode_list):
                    PAT_MODE_ID = pat_mode_cmd #execute with commanded PAT mode
                    log_to_hk('ACK CMD PL_SET_PAT_MODE: PAT mode is ' + pat_mode_names[pat_mode_list.index(PAT_MODE_ID)])
                    ack_to_hk(CMD_PL_SET_PAT_MODE, CMD_ACK)
                else:
                    log_to_hk('ERROR CMD PL_SET_PAT_MODE: Unrecognized PAT mode command: ' + str(pat_mode_cmd) + '. PAT mode is ' + pat_mode_names[pat_mode_list.index(PAT_MODE_ID)])
                    ack_to_hk(CMD_PL_SET_PAT_MODE, CMD_ERR)
            else:
                log_to_hk('ERROR CMD PL_SET_PAT_MODE: PAT process not in STANDBY.')
                ack_to_hk(CMD_PL_SET_PAT_MODE, CMD_ERR)

        elif(CMD_ID == CMD_PL_SINGLE_CAPTURE):
            exp_cmd_tuple = struct.unpack('!I', ipc_rxcompacket.payload) #TBR
            exp_cmd = exp_cmd_tuple[0]
            if(exp_cmd < 10):
                    log_to_hk('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = 10
            elif(exp_cmd > 10000000):
                    log_to_hk('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = 10000000
            if(pat_status_is(PAT_STATUS_STANDBY)):
                send_pat_command(socket_PAT_control, PAT_CMD_GET_IMAGE, str(exp_cmd))
                log_to_hk('ACK CMD PL_SINGLE_CAPTURE')
                ack_to_hk(CMD_PL_SINGLE_CAPTURE, CMD_ACK)
                #manage image telemetry file...
            else:
                log_to_hk('ERROR CMD PL_SINGLE_CAPTURE: PAT process not in STANDBY.')
                ack_to_hk(CMD_PL_SINGLE_CAPTURE, CMD_ERR)

        elif(CMD_ID == CMD_PL_CALIB_LASER_TEST):
            exp_cmd_tuple = struct.unpack('!I', ipc_rxcompacket.payload) #TBR
            exp_cmd = exp_cmd_tuple[0]
            if(exp_cmd < 10):
                    log_to_hk('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = 10
            elif(exp_cmd > 10000000):
                    log_to_hk('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = 10000000
            if(pat_status_is(PAT_STATUS_STANDBY)):
                initialize_cal_laser() #make sure cal laser dac settings are initialized for PAT
                send_pat_command(socket_PAT_control, PAT_CMD_CALIB_LASER_TEST, str(exp_cmd))
                log_to_hk('ACK CMD PL_CALIB_LASER_TEST')
                ack_to_hk(CMD_PL_CALIB_LASER_TEST, CMD_ACK)
                #Manage image telemetry files...
            else:
                log_to_hk('ERROR CMD PL_CALIB_LASER_TEST: PAT process not in STANDBY.')
                ack_to_hk(CMD_PL_SINGLE_CAPTURE, CMD_ERR)

        elif(CMD_ID == CMD_PL_FSM_TEST):
            exp_cmd_tuple = struct.unpack('!I', ipc_rxcompacket.payload) #TBR
            exp_cmd = exp_cmd_tuple[0]
            if(exp_cmd < 10):
                    log_to_hk('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = 10
            elif(exp_cmd > 10000000):
                    log_to_hk('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = 10000000
            if(pat_status_is(PAT_STATUS_STANDBY)):
                initialize_cal_laser() #make sure cal laser dac settings are initialized for PAT
                send_pat_command(socket_PAT_control, PAT_CMD_FSM_TEST, str(exp_cmd))
                log_to_hk('ACK CMD PL_FSM_TEST')
                ack_to_hk(CMD_PL_FSM_TEST, CMD_ACK)
                #Manage image telemetry files...
            else:
                log_to_hk('ERROR CMD PL_FSM_TEST: PAT process not in STANDBY.')
                ack_to_hk(CMD_PL_FSM_TEST, CMD_ERR)

        elif(CMD_ID == CMD_PL_RUN_CALIBRATION):
            if(pat_status_is(PAT_STATUS_STANDBY)):
                initialize_cal_laser() #make sure cal laser dac settings are initialized for PAT
                send_pat_command(socket_PAT_control, PAT_CMD_CALIB_TEST)
                log_to_hk('ACK CMD PL_RUN_CALIBRATION')
                ack_to_hk(CMD_PL_RUN_CALIBRATION, CMD_ACK)
                #Manage image telemetry files...
            else:
                log_to_hk('ERROR CMD PL_RUN_CALIBRATION: PAT process not in STANDBY.')
                ack_to_hk(CMD_PL_RUN_CALIBRATION, CMD_ERR)

        elif(CMD_ID == CMD_PL_TX_ALIGN):
            if(pat_status_is(PAT_STATUS_STANDBY)):
                send_pat_command(socket_PAT_control, PAT_CMD_TX_ALIGN)
                log_to_hk('ACK CMD PL_TX_ALIGN')
                ack_to_hk(CMD_PL_TX_ALIGN, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_TX_ALIGN: PAT process not in STANDBY.')
                ack_to_hk(CMD_PL_TX_ALIGN, CMD_ERR)

        elif(CMD_ID == CMD_PL_UPDATE_TX_OFFSETS):
            tx_update_x, tx_update_y = struct.unpack('!hh', ipc_rxcompacket.payload)
            if(pat_status_is(PAT_STATUS_STANDBY) or pat_status_is(PAT_STATUS_MAIN)):
                if(abs(tx_update_x) < 1000):
                    send_pat_command(socket_PAT_control, PAT_CMD_UPDATE_TX_OFFSET_X, str(tx_update_x))
                if(abs(tx_update_y) < 1000):
                    send_pat_command(socket_PAT_control, PAT_CMD_UPDATE_TX_OFFSET_Y, str(tx_update_y))
                log_to_hk('ACK CMD PL_UPDATE_TX_OFFSETS')
                ack_to_hk(CMD_PL_UPDATE_TX_OFFSETS, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_UPDATE_TX_OFFSETS: PAT process not in MAIN.')
                ack_to_hk(CMD_PL_UPDATE_TX_OFFSETS, CMD_ERR)

        elif(CMD_ID == CMD_PL_UPDATE_FSM_ANGLES):
            fsm_update_x, fsm_update_y = struct.unpack('!hh', ipc_rxcompacket.payload)
            if(pat_status_is(PAT_STATUS_STANDBY)):
                if(abs(fsm_update_x) < 1000):
                    send_pat_command(socket_PAT_control, PAT_CMD_UPDATE_FSM_X, str(fsm_update_x))
                if(abs(fsm_update_y) < 1000):
                    send_pat_command(socket_PAT_control, PAT_CMD_UPDATE_FSM_Y, str(fsm_update_y))
                log_to_hk('ACK CMD PL_UPDATE_FSM_ANGLES')
                ack_to_hk(CMD_PL_UPDATE_FSM_ANGLES, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_UPDATE_FSM_ANGLES: PAT process not in STANDBY.')
                ack_to_hk(CMD_PL_UPDATE_FSM_ANGLES, CMD_ERR)

        elif(CMD_ID == CMD_PL_ENTER_PAT_MAIN):
            if(pat_status_is(PAT_STATUS_STANDBY)):
                send_pat_command(socket_PAT_control, PAT_MODE_ID, str(PAT_TEST_FLAG))
                log_to_hk('ACK CMD PL_ENTER_PAT_MAIN')
                ack_to_hk(CMD_PL_ENTER_PAT_MAIN, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_ENTER_PAT_MAIN: PAT process not in STANDBY.')
                ack_to_hk(CMD_PL_ENTER_PAT_MAIN, CMD_ERR)

        elif(CMD_ID == CMD_PL_EXIT_PAT_MAIN):
            if(pat_status_is(PAT_STATUS_MAIN)):
                send_pat_command(socket_PAT_control, PAT_CMD_END_PAT)
                log_to_hk('ACK CMD PL_EXIT_PAT_MAIN')
                ack_to_hk(CMD_PL_EXIT_PAT_MAIN, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_EXIT_PAT_MAIN: PAT process not in MAIN')
                ack_to_hk(CMD_PL_EXIT_PAT_MAIN, CMD_ERR)

        elif(CMD_ID == CMD_PL_END_PAT_PROCESS):
            stop_pat()
            log_to_hk('ACK CMD PL_END_PAT_PROCESS')
            ack_to_hk(CMD_PL_END_PAT_PROCESS, CMD_ACK)

        elif(CMD_ID == CMD_PL_SET_FPGA):
            set_fpga_num_reg = (ipc_rxcompacket.size - 4)//4
            set_fpga_data = struct.unpack('!BHB%dI'%set_fpga_num_reg, ipc_rxcompacket.payload)
            rq_number = set_fpga_data[0]
            start_addr = set_fpga_data[1]
            num_registers = set_fpga_data[2]
            write_data = list(set_fpga_data[3:])
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
                    ack_to_hk(CMD_PL_SET_FPGA, CMD_ACK)
                else:
                    log_to_hk('ERROR CMD PL_SET_FPGA. Request Number = ' + str(rq_number) + "\n" + return_message)
                    ack_to_hk(CMD_PL_SET_FPGA, CMD_ERR)

        elif(CMD_ID == CMD_PL_GET_FPGA):
            rq_number, start_addr, num_registers = struct.unpack('!BHB', ipc_rxcompacket.payload)
            read_data = fpga.read_reg(start_addr, num_registers)
            read_data_len = len(read_data)
            if(num_registers != read_data_len):
                log_to_hk('ERROR CMD PL_GET_FPGA - Expected number of registers (= ' + str(num_registers) +  ' not equal to read data length (= ' + str(len(read_data)))
                ack_to_hk(CMD_PL_GET_FPGA, CMD_ERR)
            else:
                #send on tx port
                fpga_read_payload = struct.pack('!BHB%dI'%read_data_len, rq_number, start_addr, read_data_len, *read_data)
                print (fpga_read_payload) #debug print
                fpga_read_txpacket = TxPacket()
                raw_fpga_read_txpacket = fpga_read_txpacket.encode(APID = TLM_GET_FPGA, payload = fpga_read_payload)
                socket_tx_packets.send(raw_fpga_read_txpacket) #send packet
                ack_to_hk(CMD_PL_GET_FPGA, CMD_ACK)

        elif(CMD_ID == CMD_PL_SET_HK):
            ipc_HKControlPacket = HKControlPacket()
            raw_HKControlPacket = ipc_HKControlPacket.encode(pid, CMD_ID, ipc_rxcompacket.payload)
            socket_hk_control.send(raw_HKControlPacket)
            log_to_hk('ACK CMD PL_SET_HK')
            ack_to_hk(CMD_PL_SET_HK, CMD_ACK)

        elif(CMD_ID == CMD_PL_ECHO):
            echo_raw_size = ipc_rxcompacket.size
            echo_payload = struct.unpack('!%ds'%echo_raw_size, ipc_rxcompacket.payload)[0] #decode the raw payload bytes; since there's only one return type, take the first element of the return tuple
            echo_txpacket = TxPacket()
            raw_echo_txpacket = echo_txpacket.encode(APID = TLM_ECHO, payload = echo_payload)
            print (echo_payload) #debug printing
            print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT))) #Debug printing
            socket_tx_packets.send(raw_echo_txpacket) #send packet
            log_to_hk('ACK CMD PL_ECHO')
            ack_to_hk(CMD_PL_ECHO, CMD_ACK)

        elif(CMD_ID == CMD_PL_NOOP):
            log_to_hk('ACK CMD PL_NOOP')
            ack_to_hk(CMD_PL_NOOP, CMD_ACK)

        elif(CMD_ID == CMD_PL_SELF_TEST):
            test_id_tuple = struct.unpack('!B', ipc_rxcompacket.payload)
            test_id = test_id_tuple[0]
            test_list = [GENERAL_SELF_TEST, LASER_SELF_TEST, PAT_SELF_TEST]
            test_names = ['GENERAL_SELF_TEST', 'LASER_SELF_TEST', 'PAT_SELF_TEST']
            if(test_id not in test_list):
                log_to_hk('ERROR CMD PL_SELF_TEST: Unrecognized test ID: ' + str(test_id))
            else:
                #execute test
                if(test_id == GENERAL_SELF_TEST):
                    log_to_hk('ACK CMD PL_SELF_TEST: Test is GENERAL_SELF_TEST')

                    #Execute general self test script
                    run_test_script = 'python /root/test/general_functionality_test.py'
                    try:
                        os.system(run_test_script + ' > /root/log/' + str(file_out_num) + '.log') #TBR output file
                        #file management...
                    except:
                        log_to_hk('ERROR CMD PL_SELF_TEST - GENERAL_SELF_TEST: ' + traceback.format_exc())

                elif(test_id == LASER_SELF_TEST):
                    log_to_hk('ACK CMD PL_SELF_TEST: Test is LASER_SELF_TEST')
                    #Execute laser self test script
                    run_test_script = 'python /root/test/automated_laser_checks.py'
                    try:
                        os.system(run_test_script + ' > /root/log/' + str(file_out_num) + '.log') #TBR output file
                        #file management...
                    except:
                        log_to_hk('ERROR CMD PL_SELF_TEST - LASER_SELF_TEST: ' + traceback.format_exc())

                elif(test_id == PAT_SELF_TEST):
                    log_to_hk('ACK CMD PL_SELF_TEST: Test is PAT_SELF_TEST')
                    if(pat_status_is(PAT_STATUS_STANDBY)):
                        initialize_cal_laser() #make sure cal laser dac settings are initialized for PAT
                        #execute PAT self test
                        send_pat_command(socket_PAT_control, PAT_CMD_SELF_TEST)
                    elif(pat_status_is(PAT_STATUS_CAMERA_INIT)):
                        #Check if camera failure is to blame and run self test in camera initialization loop
                        send_pat_command(socket_PAT_control, PAT_CMD_SELF_TEST)
                    else:
                        log_to_hk('ERROR CMD PL_SELF_TEST: PAT process not in CAMERA INIT or STANDBY.')

        elif(CMD_ID == CMD_PL_DWNLINK_MODE):
            start_time = time.time()
            CH_MODE_ID = CH_MODE_DOWNLINK
            log_to_hk('ACK CMD PL_DWNLINK_MODE with start time: ' + start_time)
            if(pat_status_is(PAT_STATUS_STANDBY)):
                ack_to_hk(CMD_PL_DWNLINK_MODE, CMD_ACK)
                initialize_cal_laser() #make sure cal laser dac settings are initialized for PAT
                #Start Main PAT Loop:
                send_pat_command(socket_PAT_control, PAT_MODE_ID, str(PAT_FLIGHT_FLAG))
            else:
                log_to_hk('ERROR CMD PL_DWNLINK_MODE: PAT process not in STANDBY.')
                ack_to_hk(CMD_PL_DWNLINK_MOD, CMD_ERR)

            ###TODO: variable delay until lasercom tx script execution

        elif(CMD_ID == CMD_PL_DEBUG_MODE):
            start_time = time.time()
            CH_MODE_ID = CH_MODE_DEBUG
            log_to_hk('ACK CMD PL_DEBUG_MODE with start time: ' + start_time)

            if(not pat_status_is(PAT_STATUS_STANDBY)):
                log_to_hk('ERROR CMD PL_DEBUG_MODE: PAT process not in STANDBY.')
                ack_to_hk(CMD_PL_DEBUG_MODE, CMD_ERR)
            else:
                ack_to_hk(CMD_PL_DEBUG_MODE, CMD_ACK)

            ###TODO: add any other debug process start-up tasks

        elif(CMD_ID == CMD_PL_UPDATE_SEED_PARAMS):
            set_fpga_num_reg = (ipc_rxcompacket.size - 4)//4
            set_fpga_data = struct.unpack('!%dI'%(set_fpga_num_reg+3), ipc_rxcompacket.payload)
            rq_number = set_fpga_data[0]
            start_addr = set_fpga_data[1]
            num_registers = set_fpga_data[2]
            write_data = list(set_fpga_data[3:])
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
                    ack_to_hk(CMD_PL_SET_FPGA, CMD_ACK)
                else:
                    log_to_hk('ERROR CMD PL_SET_FPGA. Request Number = ' + str(rq_number) + "\n" + return_message)
                    ack_to_hk(CMD_PL_SET_FPGA, CMD_ERR)


        else: #default
            log_to_hk('ERROR: Unrecognized CMD_ID = ' + str(CMD_ID))
            # TODO: Send Error Packet
