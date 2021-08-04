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
import csv
#importing options and functions
sys.path.append('/root/lib/')
sys.path.append('/root/test/')
import ipc_loadbalancer
from options import *
from ipc_packets import FPGAMapRequestPacket, FPGAMapAnswerPacket, TxPacket, RxCommandPacket, PATControlPacket, HeartbeatPacket, HKControlPacket, PATStatusPacket
from zmqTxRx import recv_zmq, separate
import ipc_helper
import fpga_map as mmap
from filehandling import *
import tx_packet
from os import path
import general_functionality_test
import automated_laser_checks

# define fpga interface
fpga = ipc_helper.FPGAClientInterface()
power = mmap.Power(fpga) # power sub-interface

# use PID as unique identifier for this progress
topic = str(os.getpid())
pid = os.getpid()

# mode parameters
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
pat_status_list = [PAT_STATUS_CAMERA_INIT, PAT_STATUS_STANDBY, PAT_STATUS_STANDBY_CALIBRATED, PAT_STATUS_STANDBY_SELF_TEST_PASSED, PAT_STATUS_STANDBY_SELF_TEST_FAILED, PAT_STATUS_MAIN]
pat_status_names = ['CAMERA INIT', 'STANDBY', 'STANDBY_CALIBRATED', 'STANDBY_SELF_TEST_PASSED', 'STANDBY_SELF_TEST_FAILED', 'MAIN']

# ZeroMQ inter process communication
context = zmq.Context()

#ZMQ REQ worker socket for load balancing
ipc_worker = ipc_loadbalancer.WorkerInterface(context)
# Tell the router we're ready for work
ipc_worker.send_ready()

#print ("Pulling Rx Cmd Packets")
#print ("on port {}".format(RX_CMD_PACKETS_PORT))
#socket_rx_command_packets = context.socket(zmq.SUB)
#socket_rx_command_packets.setsockopt(zmq.SUBSCRIBE, b'')
#socket_rx_command_packets.connect("tcp://127.0.0.1:%s" % RX_CMD_PACKETS_PORT)
#poller_rx_command_packets = zmq.Poller() #poll rx commands
#poller_rx_command_packets.register(socket_rx_command_packets, zmq.POLLIN)

socket_hk_heartbeat = context.socket(zmq.PUB) #send messages on this port
socket_hk_heartbeat.connect("tcp://127.0.0.1:%s" % CH_HEARTBEAT_PORT) #connect to specific address (localhost)

socket_hk_control = context.socket(zmq.PUB) #send messages on this port
socket_hk_control.connect("tcp://127.0.0.1:%s" % HK_CONTROL_PORT) #connect to specific address (localhost)

socket_tx_packets = context.socket(zmq.PUB)
socket_tx_packets.connect("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)

socket_PAT_control = context.socket(zmq.PUB) #send messages on this port
socket_PAT_control.connect("tcp://127.0.0.1:%s" % PAT_CONTROL_PORT) #connect to specific address (localhost)

socket_PAT_status = context.socket(zmq.SUB)
socket_PAT_status.connect("tcp://127.0.0.1:%s" % PAT_STATUS_PORT)
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
    #print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT))) #debug print
    #print(ipc_patControlPacket) #debug print
    socket_PAT_control.send(raw_patControlPacket)
    return ipc_patControlPacket

def send_heartbeat(current_time,counter_hb):
    ipc_heartbeatPacket = HeartbeatPacket()
    raw_ipc_heartbeatPacket = ipc_heartbeatPacket.encode(pid, current_time)
    #print(ipc_heartbeatPacket) #Debug printing
    socket_hk_heartbeat.send(raw_ipc_heartbeatPacket)
    return (counter_hb + 1)

def log_to_hk(payload):
    #print(payload) #debug printing
    ipc_HKPacket = HKControlPacket()
    raw = ipc_HKPacket.encode(pid, HK_CONTROL_LOG, payload)
    socket_hk_control.send(raw)

def ack_to_hk(cmd_id, status):
    ipc_HKPacket = HKControlPacket()
    payload = struct.pack('HH', cmd_id, status)
    raw = ipc_HKPacket.encode(pid, HK_CONTROL_ACK, payload)
    socket_hk_control.send(raw)

def set_hk_ch_period(new_period):
    ipc_HKPacket = HKControlPacket()
    payload = struct.pack('I', new_period)
    raw = ipc_HKPacket.encode(pid, HK_CONTROL_CH, payload)
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

# def start_camera():
#     os.system('systemctl --user start camera') #calls camera.service to turn on camera
#     log_to_hk('CAMERA ON')

# def stop_camera():
#     os.system('systemctl --user stop camera') #calls camera.service to turn off camera
#     log_to_hk('CAMERA OFF')

def set_pat_mode(mode_id):
    if(mode_id in pat_mode_list):
        log_to_hk("In set_pat_mode - Setting PAT mode to: " + pat_mode_names[pat_mode_list.index(mode_id)])
        pat_mode_file = open(PAT_MODE_FILENAME,"w")
        pat_mode_file.write(str(mode_id))
        pat_mode_file.close()
    else:
        log_to_hk("In set_pat_mode - Ignoring unrecognized PAT mode input: " + str(mode_id))

#for get_pat_mode checking
def string_is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def get_pat_mode():
    if(path.exists(PAT_MODE_FILENAME)):
        pat_mode_file = open(PAT_MODE_FILENAME,"r")
        mode_id_str = pat_mode_file.read()
        if(string_is_int(mode_id_str)):
            mode_id = int(mode_id_str)
            if(mode_id in pat_mode_list):
                log_to_hk("In get_pat_mode - PAT mode is: " + pat_mode_names[pat_mode_list.index(mode_id)])
            else:
                log_to_hk("In get_pat_mode - Unrecognized PAT mode: " + str(mode_id))
                pat_mode_file.close()
                mode_id = PAT_CMD_START_PAT
                set_pat_mode(mode_id)
        else:
            log_to_hk("In get_pat_mode - Unrecognized PAT mode: " + mode_id_str)
            pat_mode_file.close()
            mode_id = PAT_CMD_START_PAT
            set_pat_mode(mode_id)
    else:
        log_to_hk("In get_pat_mode - Creating " + PAT_MODE_FILENAME)
        mode_id = PAT_CMD_START_PAT
        set_pat_mode(mode_id)
    return mode_id

def stop_pat():
    send_pat_command(socket_PAT_control, PAT_CMD_END_PROCESS) #this isn't necessary if PAT is running as a service
    os.system("systemctl --user stop pat") #stop the pat service
    log_to_hk('PAT STOPPED')

# def restart_pat():
#     stop_pat()
#     time.sleep(1)
#     os.system("systemctl --user restart pat") #restart the pat service
#     log_to_hk('PAT RESTARTED')

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

def init_pat_status():
    #intialize PAT status
    for i in range(10):
        pat_received_status, status_flag, pat_return_addr = get_pat_status()
        if(pat_received_status):
            log_to_hk('CH (PID ' + str(pid) + ') connected to PAT process at PID = ' + str(pat_return_addr))
            break
    if(not pat_received_status):
        log_to_hk('WARNING: PAT process unresponsive at CH (PID = ' + str(pid) + ') startup.')
    
    return status_flag

pat_status_flag = -1 #initialize to null
def pat_status_is(pat_status_check):
    if(pat_status_flag in pat_status_list):
        return (pat_status_flag == pat_status_check)
    else:
        return False

def log_pat_status():
    if(pat_status_flag in pat_status_list):
        log_to_hk('PAT Process Running. Status: ' + pat_status_names[pat_status_list.index(pat_status_flag)])
    else:
        log_to_hk('PAT Process Running. Status: Unrecognized')

def heat_to_0C(counter_heartbeat):
    start_time = time.time()
    counter_heartbeat = send_heartbeat(start_time, counter_heartbeat)    
    #Heat Payload to 0C (if not already there)
    temp_block = [fpga.read_reg(reg) for reg in mmap.TEMPERATURE_BLOCK]
    temps = sum(temp_block)/6
    log_to_hk('mmap.TEMPERATURE_BLOCK = ' + str(temp_block))
    success = True #initialize return value
    if (temps<0):
        log_to_hk('Heat to 0C with start time: ' + str(start_time))
        fpga.write_reg(mmap.PO3, 85)
        fpga.write_reg(mmap.HE1, 85)
        fpga.write_reg(mmap.HE2, 85)
        #Poll temps once per 15 seconds, hang until the average is above 0C stop
        temp_sleep_time = 15 #seconds
        set_hk_ch_period(2*temp_sleep_time) #delay heartbeat period from default 10 sec to twice the sleep time
        counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
        begin_time = time.time()
        while(temps < 0):
            temps = sum([fpga.read_reg(reg) for reg in mmap.TEMPERATURE_BLOCK])/6
            counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
            time.sleep(temp_sleep_time)
            log_to_hk("Elapsed Time = %s, Temp = %s" % (time.time() - begin_time, temps))
            if ((time.time() - begin_time) > TIMEOUT_HEAT_TO_0C):
                log_to_hk("Heater time reached 15 minutes and avg temps: %s" % sum([fpga.read_reg(reg) for reg in mmap.TEMPERATURE_BLOCK])/6)
                success = False
                break 
        fpga.write_reg(mmap.PO3, 15)
        fpga.write_reg(mmap.HE1, 15)
        fpga.write_reg(mmap.HE2, 15)
        set_hk_ch_period(HK_CH_CHECK_PD) #reset housekeeping heartbeat checking to default
    else:
        log_to_hk("Avg payload temperature is above 0C")
    return success, counter_heartbeat

#initialization
start_time_ch = time.time() #default start_time is the execution time (debug or downlink mode commands overwrite this)
counter_ground_test = 0 #used to count the number of repetitive process tasks
counter_debug = 0 #used to count the number of repetitive process tasks
counter_downlink = 0 #used to count the number of repetitive process tasks
counter_heartbeat = 0 #used to count the number of repetitive process tasks
pat_init = False 

#start command handling
while True:
    curr_time = time.time()
    elapsed_time = curr_time - start_time_ch

    #send heartbeat to housekeeping
    if(elapsed_time >= HK_CH_HEARTBEAT_PD*counter_heartbeat):
        counter_heartbeat = send_heartbeat(curr_time, counter_heartbeat)

    if((elapsed_time >= 10) and (not pat_init)):
        #initialize PAT status
        pat_status_flag = init_pat_status()
        if(pat_status_flag in pat_status_list):
            pat_init = True
            log_pat_status()

    if(pat_init):
        #update PAT status
        pat_status_flag = update_pat_status(pat_status_flag)

    # # Check if new RxCommandPacket() workload is available from router
    workload = ipc_worker.poll_request(500) #500 ms timeout
    if workload:
        # # interpret workload as RxCommandPacket
        ipc_rxcompacket = RxCommandPacket()
        ipc_rxcompacket.decode(workload)
        #poll for received commands
        #sockets = dict(poller_rx_command_packets.poll(10)) #poll for 10 milliseconds
        #if socket_rx_command_packets in sockets and sockets[socket_rx_command_packets] == zmq.POLLIN:
        # get commands
        # print ('RECEIVING on %s with TIMEOUT %d' % (socket_rx_command_packets.get_string(zmq.LAST_ENDPOINT), socket_rx_command_packets.get(zmq.RCVTIMEO)))
        #message = recv_zmq(socket_rx_command_packets)

        # decode the package
        #ipc_rxcompacket = RxCommandPacket()
        #ipc_rxcompacket.decode(message)
        CMD_ID = ipc_rxcompacket.APID

        if(CMD_ID != APID_TIME_AT_TONE):
            #don't print the time at tone receives
            print (ipc_rxcompacket)
            # print ('| got PAYLOAD %s' % (ipc_rxcompacket.payload))

        if(CMD_ID == APID_TIME_AT_TONE):
            if (TIME_SET_ENABLE > 0):
                #print('len(ipc_rxcompacket.payload): ', len(ipc_rxcompacket.payload))
                #print('ipc_rxcompacket.payload: ', ipc_rxcompacket.payload)
                tai_secs,_,_,_,_,_,_,_,_,_,_,_,_ = struct.unpack('!L6QB4QB', ipc_rxcompacket.payload)
                # tai_secs is TAI time since 1970, convert to UTC here
                set_time = time.gmtime(tai_secs-37)
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
            os.system("shutdown -r now") #reboot the RPi

        elif(CMD_ID == CMD_PL_SHUTDOWN):
            log_to_hk('ACK CMD PL_SHUTDOWN')
            ack_to_hk(CMD_PL_SHUTDOWN, CMD_ACK)
            time.sleep(1)
            os.system("shutdown now") #reboot the RPi

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
            # print (list_file_txpacket) #Debug printing
            # print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT))) #Debug printing
            socket_tx_packets.send(raw) #send packet

        elif(CMD_ID == CMD_PL_AUTO_DOWNLINK_FILE):
            auto_downlink_file(ipc_rxcompacket.payload, socket_tx_packets)
            log_to_hk('ACK CMD PL_AUTO_DOWNLINK_FILE')
            ack_to_hk(CMD_PL_AUTO_DOWNLINK_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_ZIP_DOWNLINK_FILE):
            zip_downlink_file(ipc_rxcompacket.payload, socket_tx_packets)
            log_to_hk('ACK CMD PL_ZIP_DOWNLINK_FILE')
            ack_to_hk(CMD_PL_ZIP_DOWNLINK_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_ZIP_DOWNLINK_PAT_DATA):
            zip_downlink_pat_data(ipc_rxcompacket.payload, socket_tx_packets)
            log_to_hk('ACK CMD PL_ZIP_DOWNLINK_PAT_DATA')
            ack_to_hk(CMD_PL_ZIP_DOWNLINK_PAT_DATA, CMD_ACK)

        elif(CMD_ID == CMD_PL_DISASSEMBLE_FILE):
            disassemble_file(ipc_rxcompacket.payload, socket_tx_packets)
            log_to_hk('ACK CMD PL_DISASSEMBLE_FILE')
            ack_to_hk(CMD_PL_DISASSEMBLE_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_REQUEST_FILE):
            request_file(ipc_rxcompacket.payload, socket_tx_packets)
            log_to_hk('ACK CMD PL_REQUEST_FILE')
            ack_to_hk(CMD_PL_REQUEST_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_UPLINK_FILE):
            uplink_file(ipc_rxcompacket.payload, socket_tx_packets)
            log_to_hk('ACK CMD PL_UPLINK_FILE')
            ack_to_hk(CMD_PL_UPLINK_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_ASSEMBLE_FILE):
            assemble_file(ipc_rxcompacket.payload, socket_tx_packets)
            log_to_hk('ACK CMD PL_ASSEMBLE_FILE')
            ack_to_hk(CMD_PL_ASSEMBLE_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_VALIDATE_FILE):
            validate_file(ipc_rxcompacket.payload, socket_tx_packets)
            log_to_hk('ACK CMD PL_VALIDATE_FILE')
            ack_to_hk(CMD_PL_VALIDATE_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_MOVE_FILE):
            move_file(ipc_rxcompacket.payload, socket_tx_packets)
            log_to_hk('ACK CMD PL_MOVE_FILE')
            ack_to_hk(CMD_PL_MOVE_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_DELETE_FILE):
            del_file(ipc_rxcompacket.payload, socket_tx_packets)
            log_to_hk('ACK CMD PL_DELETE_FILE')
            ack_to_hk(CMD_PL_DELETE_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_AUTO_ASSEMBLE_FILE):
            auto_assemble_file(ipc_rxcompacket.payload, socket_tx_packets)
            log_to_hk('ACK CMD PL_AUTO_ASSEMBLE_FILE')
            ack_to_hk(CMD_PL_AUTO_ASSEMBLE_FILE, CMD_ACK)

        elif(CMD_ID == CMD_PL_SET_PAT_MODE):
            pat_mode_cmd_tuple = struct.unpack('!B', ipc_rxcompacket.payload)
            pat_mode_cmd = pat_mode_cmd_tuple[0]
            if(pat_mode_cmd in pat_mode_list):
                set_pat_mode(pat_mode_cmd)
                ack_to_hk(CMD_PL_SET_PAT_MODE, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_SET_PAT_MODE: Unrecognized PAT mode command: ' + str(pat_mode_cmd))
                ack_to_hk(CMD_PL_SET_PAT_MODE, CMD_ERR)
            get_pat_mode() #print current pat mode to hk telemetry

        elif(CMD_ID == CMD_PL_UPDATE_PAT_OFFSET_PARAMS):
            len_new_parameter_data = (ipc_rxcompacket.size - 2)//4
            packet_data = struct.unpack('!H%df' % len_new_parameter_data, ipc_rxcompacket.payload)
            flag = packet_data[0]
            new_parameter_data = packet_data[1:]
            num_offset_params = len(NAMES_OFFSET_PARAMS)
            bool_update_row = [0]*num_offset_params
            for i in range(0,num_offset_params):
                bool_update_row[i] = (flag >> (15-i)) & 1

            if(len_new_parameter_data == sum(bool_update_row)): 
                #update parameter values
                with open('offsetParams.csv', mode = 'w') as csvfile:
                    csv_writer = csv.writer(csvfile, delimiter=',')
                    j = 0
                    for i in range(0,num_offset_params):
                        if(bool_update_row[i]):
                            csv_writer.writerow([NAMES_OFFSET_PARAMS[i], " %f" % new_parameter_data[j]])
                            j += 1
                
                log_to_hk('ACK CMD PL_UPDATE_PAT_OFFSET_PARAMS')
                ack_to_hk(CMD_PL_UPDATE_PAT_OFFSET_PARAMS, CMD_ACK)
            else:
                log_to_hk("ERROR CMD PL_UPDATE_PAT_OFFSET_PARAMS: Data Size Mismatch. Float Len (%d) != Flag Sum (%d)" % (len_new_parameter_data, sum(bool_update_row)))
                ack_to_hk(CMD_PL_UPDATE_PAT_OFFSET_PARAMS, CMD_ERR)

        elif(CMD_ID == CMD_PL_SINGLE_CAPTURE):
            window_ctr_rel_x, window_ctr_rel_y, window_width, window_height, exp_cmd = struct.unpack('!hhHHI', ipc_rxcompacket.payload)
            if(pat_status_is(PAT_STATUS_STANDBY) or pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_FAILED)):
                if((abs(window_ctr_rel_x) <= CAMERA_WIDTH/2 - window_width/2) and (abs(window_ctr_rel_y) < CAMERA_HEIGHT/2 - window_height/2) and (window_width <= CAMERA_WIDTH) and (window_height <= CAMERA_HEIGHT) and (exp_cmd >= CAMERA_MIN_EXP) and (exp_cmd <= CAMERA_MAX_EXP)):
                    send_pat_command(socket_PAT_control, PAT_CMD_SET_GET_IMAGE_WINDOW_WIDTH, str(window_width))
                    send_pat_command(socket_PAT_control, PAT_CMD_SET_GET_IMAGE_WINDOW_HEIGHT, str(window_height))
                    send_pat_command(socket_PAT_control, PAT_CMD_SET_GET_IMAGE_CENTER_X, str(window_ctr_rel_x))
                    send_pat_command(socket_PAT_control, PAT_CMD_SET_GET_IMAGE_CENTER_Y, str(window_ctr_rel_y))
                    send_pat_command(socket_PAT_control, PAT_CMD_GET_IMAGE, str(exp_cmd))
                    log_to_hk('ACK CMD PL_SINGLE_CAPTURE')
                    ack_to_hk(CMD_PL_SINGLE_CAPTURE, CMD_ACK)
                else:
                    log_to_hk("ERROR CMD PL_SINGLE_CAPTURE: Parameters out of bounds. [" + str(window_ctr_rel_x) + ", " + str(window_ctr_rel_y) + ", " + str(window_width) + ", " + str(window_height) + ", " + str(exp_cmd) + "]")
                    ack_to_hk(CMD_PL_SINGLE_CAPTURE, CMD_ERR)
            else:
                log_to_hk('ERROR CMD PL_SINGLE_CAPTURE: PAT process not in STANDBY.')
                log_pat_status()
                ack_to_hk(CMD_PL_SINGLE_CAPTURE, CMD_ERR)

        elif(CMD_ID == CMD_PL_CALIB_LASER_TEST):
            exp_cmd_tuple = struct.unpack('!I', ipc_rxcompacket.payload) #TBR
            exp_cmd = exp_cmd_tuple[0]
            if(exp_cmd < CAMERA_MIN_EXP):
                    log_to_hk('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = CAMERA_MIN_EXP
            elif(exp_cmd > CAMERA_MAX_EXP):
                    log_to_hk('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = CAMERA_MAX_EXP
            if(pat_status_is(PAT_STATUS_STANDBY) or pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_FAILED)):
                initialize_cal_laser() #make sure cal laser dac settings are initialized for PAT
                send_pat_command(socket_PAT_control, PAT_CMD_CALIB_LASER_TEST, str(exp_cmd))
                log_to_hk('ACK CMD PL_CALIB_LASER_TEST')
                ack_to_hk(CMD_PL_CALIB_LASER_TEST, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_CALIB_LASER_TEST: PAT process not in STANDBY.')
                log_pat_status()
                ack_to_hk(CMD_PL_SINGLE_CAPTURE, CMD_ERR)

        elif(CMD_ID == CMD_PL_FSM_TEST):
            exp_cmd_tuple = struct.unpack('!I', ipc_rxcompacket.payload) #TBR
            exp_cmd = exp_cmd_tuple[0]
            if(exp_cmd < CAMERA_MIN_EXP):
                    log_to_hk('Exposure below minimum of 10 us entered. Using 10 us.')
                    exp_cmd = CAMERA_MIN_EXP
            elif(exp_cmd > CAMERA_MAX_EXP):
                    log_to_hk('Exposure above maximum of 10000000 us entered. Using 10000000 us.')
                    exp_cmd = CAMERA_MAX_EXP
            if(pat_status_is(PAT_STATUS_STANDBY) or pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_FAILED)):
                initialize_cal_laser() #make sure cal laser dac settings are initialized for PAT
                send_pat_command(socket_PAT_control, PAT_CMD_FSM_TEST, str(exp_cmd))
                log_to_hk('ACK CMD PL_FSM_TEST')
                ack_to_hk(CMD_PL_FSM_TEST, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_FSM_TEST: PAT process not in STANDBY.')
                log_pat_status()
                ack_to_hk(CMD_PL_FSM_TEST, CMD_ERR)

        elif(CMD_ID == CMD_PL_RUN_CALIBRATION):
            if(pat_status_is(PAT_STATUS_STANDBY) or pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_FAILED)):
                initialize_cal_laser() #make sure cal laser dac settings are initialized for PAT
                send_pat_command(socket_PAT_control, PAT_CMD_CALIB_TEST)
                log_to_hk('ACK CMD PL_RUN_CALIBRATION')
                ack_to_hk(CMD_PL_RUN_CALIBRATION, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_RUN_CALIBRATION: PAT process not in STANDBY.')
                log_pat_status()
                ack_to_hk(CMD_PL_RUN_CALIBRATION, CMD_ERR)

        elif(CMD_ID == CMD_PL_TEST_ADCS_FEEDBACK):
            if(pat_status_is(PAT_STATUS_STANDBY) or pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_FAILED)):
                send_pat_command(socket_PAT_control, PAT_CMD_TEST_BUS_FEEDBACK)
                log_to_hk('ACK CMD PL_TEST_ADCS_FEEDBACK')
                ack_to_hk(CMD_PL_TEST_ADCS_FEEDBACK, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_TEST_ADCS_FEEDBACK: PAT process not in STANDBY.')
                log_pat_status()
                ack_to_hk(CMD_PL_TEST_ADCS_FEEDBACK, CMD_ERR)

        elif(CMD_ID == CMD_PL_UPDATE_ACQUISITION_PARAMS):
            bcn_rel_x, bcn_rel_y, bcn_window_size, bcn_max_exp = struct.unpack('!hhHI', ipc_rxcompacket.payload)
            if(pat_status_is(PAT_STATUS_STANDBY) or pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_FAILED)):
                if((abs(bcn_rel_x) < CAMERA_WIDTH/2) and (abs(bcn_rel_y) < CAMERA_HEIGHT/2) and (bcn_window_size <= CAMERA_HEIGHT) and (bcn_max_exp > CAMERA_MIN_EXP) and (bcn_max_exp < CAMERA_MAX_EXP)):
                    send_pat_command(socket_PAT_control, PAT_CMD_SET_BEACON_X, str(bcn_rel_x))
                    send_pat_command(socket_PAT_control, PAT_CMD_SET_BEACON_Y, str(bcn_rel_y))
                    send_pat_command(socket_PAT_control, PAT_CMD_SET_BEACON_WINDOW_SIZE, str(bcn_window_size))
                    send_pat_command(socket_PAT_control, PAT_CMD_SET_BEACON_MAX_EXP, str(bcn_max_exp))
                    log_to_hk('ACK CMD PL_UPDATE_ACQUISITION_PARAMS')
                    ack_to_hk(CMD_PL_UPDATE_ACQUISITION_PARAMS, CMD_ACK)
                else:
                    log_to_hk('ERROR CMD PL_UPDATE_ACQUISITION_PARAMS: Parameters out of bounds')
            else:
                log_to_hk('ERROR CMD PL_TX_ALIGN: PAT process not in STANDBY.')
                ack_to_hk(CMD_PL_TX_ALIGN, CMD_ERR)

        elif(CMD_ID == CMD_PL_TX_ALIGN):
            if(pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED)):
                send_pat_command(socket_PAT_control, PAT_CMD_TX_ALIGN)
                log_to_hk('ACK CMD PL_TX_ALIGN')
                ack_to_hk(CMD_PL_TX_ALIGN, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_TX_ALIGN: PAT process not in STANDBY.')
                log_pat_status()
                ack_to_hk(CMD_PL_TX_ALIGN, CMD_ERR)

        elif(CMD_ID == CMD_PL_UPDATE_TX_OFFSETS):
            tx_update_x, tx_update_y, tx_offset_calc_pd, enable_dither, dither_pd = struct.unpack('!hhHBH', ipc_rxcompacket.payload)
            if(pat_status_is(PAT_STATUS_STANDBY) or pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_FAILED) or pat_status_is(PAT_STATUS_MAIN)):
                if(abs(tx_update_x) < CAMERA_WIDTH/2):
                    send_pat_command(socket_PAT_control, PAT_CMD_UPDATE_TX_OFFSET_X, str(tx_update_x))
                if(abs(tx_update_y) < CAMERA_HEIGHT/2):
                    send_pat_command(socket_PAT_control, PAT_CMD_UPDATE_TX_OFFSET_Y, str(tx_update_y))
                if(pat_status_is(PAT_STATUS_STANDBY) or pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_FAILED)):
                    send_pat_command(socket_PAT_control, PAT_CMD_UPDATE_PERIOD_CALCULATE_TX_OFFSET, str(tx_offset_calc_pd))
                    if(enable_dither == PAT_ENABLE_DITHER):
                        send_pat_command(socket_PAT_control, PAT_CMD_ENABLE_DITHER_TX_OFFSET, str(enable_dither))
                        send_pat_command(socket_PAT_control, PAT_CMD_UPDATE_PERIOD_DITHER_TX_OFFSET, str(dither_pd))
                log_to_hk('ACK CMD PL_UPDATE_TX_OFFSETS')
                ack_to_hk(CMD_PL_UPDATE_TX_OFFSETS, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_UPDATE_TX_OFFSETS: PAT process not in STANDBY or MAIN.')
                log_pat_status()
                ack_to_hk(CMD_PL_UPDATE_TX_OFFSETS, CMD_ERR)

        elif(CMD_ID == CMD_PL_UPDATE_FSM_ANGLES):
            fsm_update_x, fsm_update_y = struct.unpack('!hh', ipc_rxcompacket.payload)
            if(pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED)):
                if(abs(fsm_update_x) < CAMERA_HEIGHT/2):
                    send_pat_command(socket_PAT_control, PAT_CMD_UPDATE_FSM_X, str(fsm_update_x))
                if(abs(fsm_update_y) < CAMERA_HEIGHT/2):
                    send_pat_command(socket_PAT_control, PAT_CMD_UPDATE_FSM_Y, str(fsm_update_y))
                log_to_hk('ACK CMD PL_UPDATE_FSM_ANGLES')
                ack_to_hk(CMD_PL_UPDATE_FSM_ANGLES, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_UPDATE_FSM_ANGLES: PAT process not in STANDBY.')
                log_pat_status()
                ack_to_hk(CMD_PL_UPDATE_FSM_ANGLES, CMD_ERR)

        elif(CMD_ID == CMD_PL_ENTER_PAT_MAIN):
            if(pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED)):
                pat_mode_id = get_pat_mode()
                send_pat_command(socket_PAT_control, pat_mode_id, str(PAT_SKIP_CALIB_FLAG))
                log_to_hk('ACK CMD PL_ENTER_PAT_MAIN')
                ack_to_hk(CMD_PL_ENTER_PAT_MAIN, CMD_ACK)
            elif(pat_status_is(PAT_STATUS_STANDBY) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_FAILED)):
                pat_mode_id = get_pat_mode()
                send_pat_command(socket_PAT_control, pat_mode_id, str(PAT_DO_CALIB_FLAG)) #bcn align skips calib regardless
                log_to_hk('ACK CMD PL_ENTER_PAT_MAIN')
                ack_to_hk(CMD_PL_ENTER_PAT_MAIN, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_ENTER_PAT_MAIN: PAT process not in STANDBY.')
                log_pat_status()
                ack_to_hk(CMD_PL_ENTER_PAT_MAIN, CMD_ERR)

        elif(CMD_ID == CMD_PL_EXIT_PAT_MAIN):
            if(pat_status_is(PAT_STATUS_MAIN)):
                send_pat_command(socket_PAT_control, PAT_CMD_END_PAT)
                log_to_hk('ACK CMD PL_EXIT_PAT_MAIN')
                ack_to_hk(CMD_PL_EXIT_PAT_MAIN, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_EXIT_PAT_MAIN: PAT process not in MAIN')
                log_pat_status()
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
            #print ('Request Number = ' + str(rq_number) + ', Start Address = ' + str(start_addr) + ', Num Registers = ' + str(num_registers) + ', Write Data = ' + str(write_data)) #debug print
            if(num_registers != len(write_data)):
                log_to_hk('ERROR CMD PL_SET_FPGA - Packet Error: expected number of registers (= ' + str(num_registers) +  ' not equal to data length (= ' + str(len(write_data)))
            else:
                fpga.write_reg(start_addr, write_data)
                check_write_data = fpga.read_reg(start_addr, num_registers)
                if(type(check_write_data) == int):
                    check_write_data = [check_write_data]
                if(type(check_write_data) == list):
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
                else:
                    log_to_hk('ERROR CMD PL_SET_FPGA - Type error, expected list for check_write_data, got: ' + str(type(check_write_data)))
                    ack_to_hk(CMD_PL_GET_FPGA, CMD_ERR)

        elif(CMD_ID == CMD_PL_GET_FPGA):
            rq_number, start_addr, num_registers = struct.unpack('!BHB', ipc_rxcompacket.payload)
            read_data = fpga.read_reg(start_addr, num_registers)
            if(type(read_data) == int):
                read_data = [read_data]
            if(type(read_data) == list):
                read_data_len = len(read_data)
                if(num_registers != read_data_len):
                    log_to_hk('ERROR CMD PL_GET_FPGA - Expected number of registers (= ' + str(num_registers) +  ' not equal to read data length (= ' + str(len(read_data)))
                    ack_to_hk(CMD_PL_GET_FPGA, CMD_ERR)
                else:
                    #send on tx port
                    fpga_read_payload = struct.pack('!BHB%dI'%read_data_len, rq_number, start_addr, read_data_len, *read_data)
                    #print (fpga_read_payload) #debug print
                    fpga_read_txpacket = TxPacket()
                    raw_fpga_read_txpacket = fpga_read_txpacket.encode(APID = TLM_GET_FPGA, payload = fpga_read_payload)
                    socket_tx_packets.send(raw_fpga_read_txpacket) #send packet
                    log_to_hk('ACK CMD PL_GET_FPGA. Request Number = ' + str(rq_number) + '. Start Addr: ' + str(start_addr) + '. Num Reg: ' + str(num_registers))
                    ack_to_hk(CMD_PL_GET_FPGA, CMD_ACK)
            else:
                log_to_hk('ERROR CMD PL_GET_FPGA - Type error, expected list for read_data, got: ' + str(type(read_data)))
                ack_to_hk(CMD_PL_GET_FPGA, CMD_ERR)

        elif(CMD_ID == CMD_PL_SET_HK):
            set_hk_payload = struct.unpack('!6s', ipc_rxcompacket.payload)[0]
            ipc_HKControlPacket = HKControlPacket()
            raw_HKControlPacket = ipc_HKControlPacket.encode(pid, CMD_ID, set_hk_payload)
            socket_hk_control.send(raw_HKControlPacket)
            log_to_hk('ACK CMD PL_SET_HK')
            ack_to_hk(CMD_PL_SET_HK, CMD_ACK)

        elif(CMD_ID == CMD_PL_ECHO):
            echo_raw_size = ipc_rxcompacket.size
            echo_payload = struct.unpack('!%ds'%echo_raw_size, ipc_rxcompacket.payload)[0] #decode the raw payload bytes; since there's only one return type, take the first element of the return tuple
            echo_txpacket = TxPacket()
            raw_echo_txpacket = echo_txpacket.encode(APID = TLM_ECHO, payload = echo_payload)
            # print (echo_payload) #debug printing
            # print ('SENDING to %s' % (socket_tx_packets.get_string(zmq.LAST_ENDPOINT))) #Debug printing
            socket_tx_packets.send(raw_echo_txpacket) #send packet
            log_to_hk('ACK CMD PL_ECHO')
            ack_to_hk(CMD_PL_ECHO, CMD_ACK)

        elif(CMD_ID == CMD_PL_NOOP):
            log_to_hk('ACK CMD PL_NOOP')
            ack_to_hk(CMD_PL_NOOP, CMD_ACK)

        elif(CMD_ID == CMD_PL_SELF_TEST):
            test_id_tuple = struct.unpack('!B', ipc_rxcompacket.payload)
            test_id = test_id_tuple[0]
            test_list = [GENERAL_SELF_TEST, LASER_SELF_TEST, PAT_SELF_TEST, THERMAL_SELF_TEST, ALL_SELF_TEST]
            test_names = ['GENERAL_SELF_TEST', 'LASER_SELF_TEST', 'PAT_SELF_TEST', 'THERMAL_SELF_TEST', 'ALL_SELF_TEST']
            if(test_id not in test_list):
                log_to_hk('ERROR CMD PL_SELF_TEST: Unrecognized test ID: ' + str(test_id))
            else:
                #execute test
                if(test_id == GENERAL_SELF_TEST):
                    log_to_hk('ACK CMD PL_SELF_TEST: Test is GENERAL_SELF_TEST')
                    success_heating, counter_heartbeat = heat_to_0C(counter_heartbeat) #heat to 0C (if not already there)
                    if(success_heating or OVERRIDE_HEAT_TO_0C):
                        set_hk_ch_period(150) #delay housekeeping heartbeat checking for 2 min 30 sec (test is ~ 2 min)
                        counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                        #Execute general self test script
                        try:
                            results_summary = general_functionality_test.run_all("CH (%s)"%(pid))
                            log_to_hk(results_summary)
                            ack_to_hk(CMD_PL_SELF_TEST, CMD_ACK)
                        except:
                            log_to_hk('ERROR CMD PL_SELF_TEST - GENERAL_SELF_TEST: ' + traceback.format_exc())
                            ack_to_hk(CMD_PL_SELF_TEST, CMD_ERR)
                        set_hk_ch_period(HK_CH_CHECK_PD) #reset housekeeping heartbeat checking to default
                        counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                    else:
                        ack_to_hk(CMD_PL_SELF_TEST, CMD_ERR)

                elif(test_id == LASER_SELF_TEST):
                    log_to_hk('ACK CMD PL_SELF_TEST: Test is LASER_SELF_TEST')
                    success_heating, counter_heartbeat = heat_to_0C(counter_heartbeat) #heat to 0C (if not already there)
                    if(success_heating or OVERRIDE_HEAT_TO_0C):
                        set_hk_ch_period(30) #delay housekeeping heartbeat checking for 30 sec
                        counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                        #Execute laser self test script 
                        try:
                            results_summary = automated_laser_checks.run_all("CH (%s)"%(pid))
                            log_to_hk(results_summary)
                            ack_to_hk(CMD_PL_SELF_TEST, CMD_ACK)
                        except:
                            log_to_hk('ERROR CMD PL_SELF_TEST - LASER_SELF_TEST: ' + traceback.format_exc())
                            ack_to_hk(CMD_PL_SELF_TEST, CMD_ERR)
                        set_hk_ch_period(HK_CH_CHECK_PD) #reset housekeeping heartbeat checking to default
                        counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                    else:
                        ack_to_hk(CMD_PL_SELF_TEST, CMD_ERR)

                elif(test_id == PAT_SELF_TEST):
                    log_to_hk('ACK CMD PL_SELF_TEST: Test is PAT_SELF_TEST')
                    success_heating, counter_heartbeat = heat_to_0C(counter_heartbeat) #heat to 0C (if not already there)
                    if(success_heating or OVERRIDE_HEAT_TO_0C):
                        if(pat_status_is(PAT_STATUS_STANDBY) or pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_FAILED)):
                            initialize_cal_laser() #make sure cal laser dac settings are initialized for PAT
                            #execute PAT self test
                            send_pat_command(socket_PAT_control, PAT_CMD_SELF_TEST) #Output data (text-only) is automatically sent to bus for downlink
                            ack_to_hk(CMD_PL_SELF_TEST, CMD_ACK)
                        elif(pat_status_is(PAT_STATUS_CAMERA_INIT)):
                            #Check if camera failure is to blame and run self test in camera initialization loop
                            log_pat_status()
                            send_pat_command(socket_PAT_control, PAT_CMD_SELF_TEST)
                            ack_to_hk(CMD_PL_SELF_TEST, CMD_ACK)
                        else:
                            log_to_hk('ERROR CMD PL_SELF_TEST: PAT process not in CAMERA INIT or STANDBY.')
                            log_pat_status()
                            ack_to_hk(CMD_PL_SELF_TEST, CMD_ERR)
                    else:
                        ack_to_hk(CMD_PL_SELF_TEST, CMD_ERR)                    

                elif(test_id == THERMAL_SELF_TEST):
                    start_time = time.time()
                    counter_heartbeat = send_heartbeat(start_time, counter_heartbeat)
                    log_to_hk('ACK CMD PL_SELF_TEST: THERMAL_SELF_TEST')    
                    #Heat Payload for up to 3 min or up to a change in temp of 10C
                    temp_block = [fpga.read_reg(reg) for reg in mmap.TEMPERATURE_BLOCK]
                    temps = sum(temp_block)/6
                    log_to_hk('mmap.TEMPERATURE_BLOCK = ' + str(temp_block))
                    log_to_hk("Start Time = %s, Temp = %s" % (start_time, temps))
                    fpga.write_reg(mmap.PO3, 85)
                    fpga.write_reg(mmap.HE1, 85)
                    fpga.write_reg(mmap.HE2, 85)
                    #Poll temps once per 15 seconds, hang until the average is above 0C stop
                    temp_sleep_time = 15 #seconds
                    set_hk_ch_period(2*temp_sleep_time) #delay heartbeat period from default 10 sec to twice the sleep time
                    counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                    begin_time = time.time()
                    temps_init = temps
                    while((abs(temps - temps_init) < 10) and ((time.time() - begin_time) < 180)):
                        temps = sum([fpga.read_reg(reg) for reg in mmap.TEMPERATURE_BLOCK])/6
                        counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                        time.sleep(temp_sleep_time)
                        log_to_hk("Elapsed Time = %s, Temp = %s" % (time.time() - begin_time, temps))
                    fpga.write_reg(mmap.PO3, 15)
                    fpga.write_reg(mmap.HE1, 15)
                    fpga.write_reg(mmap.HE2, 15)
                    set_hk_ch_period(HK_CH_CHECK_PD) #reset housekeeping heartbeat checking to default
                    log_to_hk("Thermal Self Test Complete")
                    ack_to_hk(CMD_PL_SELF_TEST, CMD_ACK)

                elif(test_id == ALL_SELF_TEST):
                    log_to_hk('ACK CMD PL_SELF_TEST: Test is ALL_SELF_TEST')
                    success_heating, counter_heartbeat = heat_to_0C(counter_heartbeat) #heat to 0C (if not already there)
                    if(success_heating or OVERRIDE_HEAT_TO_0C):
                        set_hk_ch_period(150) #delay housekeeping heartbeat checking for 2 min 30 sec (test is ~ 2 min)
                        counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                        no_test_error = True 
                        #Execute general self test script
                        try:
                            log_to_hk("Running General Self Test...")
                            results_summary = general_functionality_test.run_all("CH (%s)"%(pid))
                            log_to_hk(results_summary)
                        except:
                            log_to_hk('ERROR CMD PL_SELF_TEST - ALL_SELF_TEST - GENERAL_SELF_TEST: ' + traceback.format_exc())
                            ack_to_hk(CMD_PL_SELF_TEST, CMD_ERR)
                            no_test_error = False
                        counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)

                        #Execute laser self test script
                        set_hk_ch_period(30) #delay housekeeping heartbeat checking for 30 sec
                        counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                        try:
                            log_to_hk("Running Laser Self Test...")
                            results_summary = automated_laser_checks.run_all("CH (%s)"%(pid))
                            log_to_hk(results_summary)
                        except:
                            log_to_hk('ERROR CMD PL_SELF_TEST - ALL_SELF_TEST - LASER_SELF_TEST: ' + traceback.format_exc())
                            ack_to_hk(CMD_PL_SELF_TEST, CMD_ERR)
                            no_test_error = False
                        counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)

                        #Execute PAT self test
                        set_hk_ch_period(HK_CH_CHECK_PD) #reset housekeeping heartbeat checking to default
                        log_to_hk("Running PAT Self Test...")
                        if(pat_status_is(PAT_STATUS_STANDBY) or pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_FAILED)):
                            initialize_cal_laser() #make sure cal laser dac settings are initialized for PAT
                            #execute PAT self test
                            send_pat_command(socket_PAT_control, PAT_CMD_SELF_TEST) #Output data (text-only) is automatically sent to bus for downlink
                        elif(pat_status_is(PAT_STATUS_CAMERA_INIT)):
                            #Check if camera failure is to blame and run self test in camera initialization loop
                            log_pat_status()
                            send_pat_command(socket_PAT_control, PAT_CMD_SELF_TEST)
                        else:
                            log_to_hk('ERROR CMD PL_SELF_TEST - ALL_SELF_TEST: PAT process not in CAMERA INIT or STANDBY.')
                            log_pat_status()
                            ack_to_hk(CMD_PL_SELF_TEST, CMD_ERR)
                            no_test_error = False
                    else:
                        no_test_error = False 

                    if(no_test_error):
                        ack_to_hk(CMD_PL_SELF_TEST, CMD_ACK)

        elif(CMD_ID == CMD_PL_DWNLINK_MODE):
            start_time = time.time()
            counter_heartbeat = send_heartbeat(start_time, counter_heartbeat)
            log_to_hk('ACK CMD PL_DWNLINK_MODE with start time: ' + str(start_time))

            #heat to 0C (if not already there)
            success_heating, counter_heartbeat = heat_to_0C(counter_heartbeat) 
            if(success_heating or OVERRIDE_HEAT_TO_0C):
                #General self test:
                log_to_hk("Running General Self Test...")
                set_hk_ch_period(150) #delay housekeeping heartbeat checking for 2 min 30 sec (test is ~ 2 min)
                counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                results_summary = general_functionality_test.run_all("CH (%s)"%(pid))
                log_to_hk(results_summary)
                counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)

                #Laser self test
                log_to_hk("Running Laser Self Test...")
                set_hk_ch_period(30) #delay housekeeping heartbeat checking for 2 min 30 sec (test is ~ 2 min) [TBR]
                counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                results_summary = automated_laser_checks.run_all("CH (%s)"%(pid))
                log_to_hk(results_summary)
                counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)

                #PAT self test
                log_to_hk("Running PAT Self Test...")
                set_hk_ch_period(HK_CH_CHECK_PD) #reset housekeeping heartbeat checking to default
                counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                if(pat_status_is(PAT_STATUS_STANDBY) or pat_status_is(PAT_STATUS_STANDBY_CALIBRATED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_PASSED) or pat_status_is(PAT_STATUS_STANDBY_SELF_TEST_FAILED)):
                    initialize_cal_laser() #make sure cal laser dac settings are initialized for PAT
                    #execute PAT self test
                    send_pat_command(socket_PAT_control, PAT_CMD_SELF_TEST)
                    for i in range(60): #max test time is about 60 sec
                        #log_to_hk("Waiting for pat self test to finish")
                        counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                        time.sleep(1)
                    log_to_hk('PAT self test wait complete. Commanding PAT to enter MAIN mode.')
                    pat_mode_id = get_pat_mode()
                    send_pat_command(socket_PAT_control, pat_mode_id, str(PAT_SKIP_CALIB_FLAG))
                elif(pat_status_is(PAT_STATUS_CAMERA_INIT)):
                    log_pat_status()
                    log_to_hk("Camera is off - pat self test failed.")
                    ack_to_hk(CMD_PL_DWNLINK_MODE, CMD_ERR)
                else:
                    log_pat_status()
                    log_to_hk("Pat was not in standby mode, pat self test will not run")
                    ack_to_hk(CMD_PL_DWNLINK_MODE, CMD_ERR)

                #proceed to transmit
                end_time = time.time()
                counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                log_to_hk("Pretransmit Time: %s" %(end_time - start_time))

                #log transmit start time
                start_time = time.time()

                tx_pkt = tx_packet.txPacket(TRANSMIT_PPM, TRANSMIT_MESSAGE)
                tx_pkt.pack()

                control = fpga.read_reg(mmap.CTL)
                if(control & 0x8): fpga.write_reg(mmap.DATA, 0x7) #Turn stall off

                payload_seed = [DEFAULT_TEC_MSB, DEFAULT_TEC_LSB, DEFAULT_LD_MSB, DEFAULT_LD_LSB]
                flat_sat_seed = [DEFAULT_FTEC_MSB, DEFAULT_FTEC_LSB, DEFAULT_FLD_MSB, DEFAULT_FLD_LSB]

                # seed = payload_seed
                if(SEED_SETTING):
                    seed = payload_seed
                    ppm_input = [PPM4_THRESHOLDS[0], PPM4_THRESHOLDS[1]]
                else:
                    seed = flat_sat_seed
                    ppm_input = [PPM4_THRESHOLDS[2], PPM4_THRESHOLDS[3]]

                #Align seed to FGBG
                counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                tx_packet.seed_align(seed)
                log_to_hk("Turn EDFA On")            

                fpga.write_reg(mmap.EDFA_IN_STR ,'mode acc\r')
                time.sleep(0.1)
                fpga.write_reg(mmap.EDFA_IN_STR ,'ldc ba 2200\r')
                time.sleep(0.1)
                fpga.write_reg(mmap.EDFA_IN_STR ,'edfa on\r')
                time.sleep(2)

                #set points are dependent on temperature
                counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                ppm_order = (128 + (255 >>(8-int(math.log(TRANSMIT_PPM)/math.log(2)))))
                log_to_hk("PPM: "+str(ppm_order) +', EDFA Power: '+str(fpga.read_reg(34)))
                while(abs(end_time - start_time) < TRANSMIT_TIME):

                    #Stall Fifo
                    fpga.write_reg(mmap.CTL, control | 0x8)

                    # # #Write to FIFO
                    tx_pkt.transmit(fpga, .1)

                    fifo_len = fpga.read_reg(47)*256+fpga.read_reg(48)
                    if(len(tx_pkt.symbols) != fifo_len): #Why is the empty fifo length 2
                        # success = False
                        log_to_hk("Fifo length %s does not match packet symbol length %s " % (fifo_len, len(tx_pkt.symbols)))
                        # fo.write("Fifo length %s does not match packet symbol legnth %s " % (fifo_len, tx_pkt1.symbols))
                        # fo.write("Packet PPM: %s and Data: %s " % (tx_pkt1.ppm_order, tx_pkt1.data))

                    if(fifo_len < 100): time.sleep(.005)

                    # #Release FIFO
                    fpga.write_reg(mmap.CTL, 0x7)
                    end_time = time.time()
                    if((end_time - start_time) >= HK_CH_HEARTBEAT_PD*counter_heartbeat): 
                        counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)
                
                counter_heartbeat = send_heartbeat(time.time(), counter_heartbeat)

                power.edfa_off()
                power.bias_off()
                power.tec_off()

                #reset PAT:
                send_pat_command(socket_PAT_control, PAT_CMD_END_PAT)

                log_to_hk("END PL_DWNLINK_MODE - Transmit Session Complete")
                ack_to_hk(CMD_PL_DWNLINK_MODE, CMD_ACK)
            else:
                ack_to_hk(CMD_PL_DWNLINK_MODE, CMD_ERR)

        elif(CMD_ID == CMD_PL_DEBUG_MODE):
            start_time = time.time()
            log_to_hk("ACK CMD PL_DEBUG_MODE with start time: %s" % (start_time))

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

        # # Tell the router we're ready for work
        ipc_worker.send_ready()
