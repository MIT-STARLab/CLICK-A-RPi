#!/usr/bin/env python

import sys
import os
import zmq
import json
import time
import struct
sys.path.append('../lib/')
sys.path.append('/root/lib/')
from options import *
from ipc_packets import PATControlPacket, PATStatusPacket
from zmqTxRx import recv_zmq, send_zmq
import ipc_helper
import fpga_map as mmap
fpga = ipc_helper.FPGAClientInterface()
power = mmap.Power(fpga)

#PAT Status Flag List
# pat_status_list = [PAT_STATUS_CAMERA_INIT, PAT_STATUS_STANDBY, PAT_STATUS_STANDBY_CALIBRATED, PAT_STATUS_STANDBY_SELF_TEST_PASSED, PAT_STATUS_STANDBY_SELF_TEST_FAILED, PAT_STATUS_MAIN]
# pat_status_names = ['CAMERA INIT', 'STANDBY', 'STANDBY_CALIBRATED', 'STANDBY_SELF_TEST_PASSED', 'STANDBY_SELF_TEST_FAILED', 'MAIN'] 

#PAT Command List
cmd_list = [PAT_CMD_START_PAT, PAT_CMD_START_PAT_OPEN_LOOP, PAT_CMD_START_PAT_STATIC_POINT, PAT_CMD_START_PAT_BUS_FEEDBACK, PAT_CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK,  PAT_CMD_UPDATE_TX_OFFSET_X, PAT_CMD_UPDATE_TX_OFFSET_Y, PAT_CMD_END_PAT, PAT_CMD_GET_IMAGE, PAT_CMD_CALIB_TEST, PAT_CMD_CALIB_LASER_TEST, PAT_CMD_FSM_TEST, PAT_CMD_BCN_ALIGN, PAT_CMD_TX_ALIGN, PAT_CMD_UPDATE_FSM_X, PAT_CMD_UPDATE_FSM_Y, PAT_CMD_SELF_TEST, PAT_CMD_END_PROCESS, PAT_CMD_SET_BEACON_X, PAT_CMD_SET_BEACON_Y, PAT_CMD_SET_BEACON_WINDOW_SIZE, PAT_CMD_SET_BEACON_MAX_EXP]
TURN_ON_CAL_LASER = cmd_list[len(cmd_list)-1] + 1
TURN_OFF_CAL_LASER = TURN_ON_CAL_LASER + 1

# use PID as unique identifier for this progress
topic = str(os.getpid())
pid = os.getpid()
return_address = str(pid)

context = zmq.Context()

# socket_PAT_status = context.socket(zmq.SUB)
# socket_PAT_status.bind("tcp://127.0.0.1:%s" % PAT_STATUS_PORT)
# socket_PAT_status.setsockopt(zmq.SUBSCRIBE, b'')
# poller_PAT_status = zmq.Poller()
# poller_PAT_status.register(socket_PAT_status, zmq.POLLIN)

socket_PAT_control = context.socket(zmq.PUB)
socket_PAT_control.connect("tcp://127.0.0.1:%s" % PAT_CONTROL_PORT)

# socket needs some time to set up. give it a second - else the first message will be lost
time.sleep(1)

#print("\n")

def send_pat_command(socket_PAT_control, return_address, command, payload = ''):
        # ~ #Define Command Header
        # ~ PAT_CMD_HEADER = return_address + '\0' #PID Header
        # ~ #Header format checks and padding
        # ~ assert len(PAT_CMD_HEADER) <= PAT_CMD_HEADER_SIZE #Ensure PAT_CMD_HEADER is the right length
        # ~ if(len(PAT_CMD_HEADER) < PAT_CMD_HEADER_SIZE):
                # ~ for i in range(PAT_CMD_HEADER_SIZE - len(PAT_CMD_HEADER)):
                        # ~ PAT_CMD_HEADER = PAT_CMD_HEADER + '\0' #append null padding
        # ~ assert PAT_CMD_HEADER[len(PAT_CMD_HEADER)-1] == '\0' #always terminate strings with null character ('\0') for c-code

        #Define Command Payload
        PAT_CMD_PAYLOAD = payload + '\0'
        assert len(PAT_CMD_PAYLOAD) <= PAT_CMD_PAYLOAD_SIZE #Ensure PAT_CMD_PAYLOAD is the right length
        if(len(PAT_CMD_PAYLOAD) < PAT_CMD_PAYLOAD_SIZE):
                for i in range(PAT_CMD_PAYLOAD_SIZE - len(PAT_CMD_PAYLOAD)):
                        PAT_CMD_PAYLOAD = PAT_CMD_PAYLOAD + '\0'   #append null padding
        assert PAT_CMD_PAYLOAD[len(PAT_CMD_PAYLOAD)-1] == '\0' #always terminate strings with null character ('\0') for c-code
        PAT_CMD_PAYLOAD = str(PAT_CMD_PAYLOAD).encode('ascii') #format to ascii

        ipc_patControlPacket = PATControlPacket()
        raw_patControlPacket = ipc_patControlPacket.encode(command,PAT_CMD_PAYLOAD) 
        send_zmq(socket_PAT_control, raw_patControlPacket) #, PAT_CMD_HEADER)        
        return ipc_patControlPacket

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

# def get_pat_status():
#     #get pat status
#     socks_status = dict(poller_PAT_status.poll(250)) #poll for 250 ms
#     if socket_PAT_status in socks_status and socks_status[socket_PAT_status] == zmq.POLLIN:
#         #print('RECEIVING on %s' % socket_PAT_status.get_string(zmq.LAST_ENDPOINT))
#         message = recv_zmq(socket_PAT_status)
#         ipc_patStatusPacket = PATStatusPacket()
#         pat_return_addr, pat_status_flag = ipc_patStatusPacket.decode(message) #decode the package
#         received_status = True
#     else:
#         pat_return_addr = -1
#         pat_status_flag = -1
#         received_status = False

#     return received_status, pat_status_flag, pat_return_addr

# def update_pat_status(pat_status_flag):
#     #get pat status
#     received_status, new_status_flag, _ = get_pat_status()
#     if(received_status):
#         pat_status_flag = new_status_flag

#     return pat_status_flag

# #intialize PAT status
# for i in range(100):
#    pat_received_status, pat_status_flag, pat_return_addr = get_pat_status()
#    if(pat_received_status):
#        print('Connected to PAT process at PID = ' + str(pid))
#        break
# if(not pat_received_status):
#    print('WARNING: PAT process unresponsive at test script (PID = ' + str(pid) + ') startup.')

# def pat_status_is(pat_status_check):
#     if(pat_status_flag in pat_status_list):
#         print('PAT Process Running (PID: ' + str(pat_return_addr) + '). Status: ' + pat_status_names[pat_status_list.index(pat_status_flag)])
#         #print('pat_status_flag: ', pat_status_flag)
#         #print('pat_status_check: ', pat_status_check)
#         #print('bool: ', (pat_status_flag == pat_status_check))
#         return (pat_status_flag == pat_status_check)
#     else:
#         print('PAT Process Running (PID: ' + str(pat_return_addr) + '). Status: Unrecognized')
#         return False

# Wait for a ping from the PAT process
#Read telemetry if available
# print('RECEIVING on %s' % socket_PAT_status.get_string(zmq.LAST_ENDPOINT))
# message = recv_zmq(socket_PAT_status)
# ipc_patStatusPacket = PATStatusPacket()
# pat_return_addr, pat_status_flag = ipc_patStatusPacket.decode(message) #decode the package
# #print(telemetry_string)
# time.sleep(1)

counter = 0
command_period_sec = 10
poll_timeout_msec = 25
cal_laser_init = False
while True:    
        # pat_status_flag = update_pat_status(pat_status_flag)
        # socks_status = dict(poller_PAT_status.poll(poll_timeout_msec))
        # if socket_PAT_status in socks_status and socks_status[socket_PAT_status] == zmq.POLLIN:
        #         print('RECEIVING on %s' % socket_PAT_status.get_string(zmq.LAST_ENDPOINT))
        #         message = recv_zmq(socket_PAT_status)
        #         ipc_patStatusPacket = PATStatusPacket()
        #         pat_return_addr, pat_status_flag = ipc_patStatusPacket.decode(message) #decode the package

        #Send commands if no incoming telemetry (standby) or after command_period timeout (allow exiting main pat loop while running)
        if((counter*poll_timeout_msec/1000) % command_period_sec == 0):
            #if(pat_status_flag in pat_status_list):
            #print ('PAT Process (PID: ' + str(pat_return_addr) + ') Status: ' + pat_status_names[pat_status_list.index(pat_status_flag)])
            print "Commands are: "
            print "TURN_ON_CAL_LASER = ", TURN_ON_CAL_LASER
            print "TURN_OFF_CAL_LASER = ", TURN_OFF_CAL_LASER 
            print "CMD_START_PAT = ", PAT_CMD_START_PAT
            print "CMD_START_PAT_OPEN_LOOP = ", PAT_CMD_START_PAT_OPEN_LOOP
            print "CMD_START_PAT_STATIC_POINT = ", PAT_CMD_START_PAT_STATIC_POINT
            print "CMD_START_PAT_BUS_FEEDBACK = ", PAT_CMD_START_PAT_BUS_FEEDBACK
            print "CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK = ", PAT_CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK
            print "CMD_END_PAT (Return to Standby) = ", PAT_CMD_END_PAT
            print "CMD_GET_IMAGE = ", PAT_CMD_GET_IMAGE
            print "CMD_CALIB_TEST = ", PAT_CMD_CALIB_TEST
            print "CMD_CALIB_LASER_TEST = ", PAT_CMD_CALIB_LASER_TEST
            print "CMD_FSM_TEST = ", PAT_CMD_FSM_TEST
            print "CMD_BCN_ALIGN = ", PAT_CMD_BCN_ALIGN
            print "CMD_SET_BEACON_X = ", PAT_CMD_SET_BEACON_X
            print "CMD_SET_BEACON_Y = ", PAT_CMD_SET_BEACON_Y
            print "CMD_SET_BEACON_WINDOW_SIZE = ", PAT_CMD_SET_BEACON_WINDOW_SIZE
            print "CMD_SET_BEACON_MAX_EXP = ", PAT_CMD_SET_BEACON_MAX_EXP
            print "CMD_TX_ALIGN = ", PAT_CMD_TX_ALIGN
            print "CMD_UPDATE_TX_OFFSET_X = ", PAT_CMD_UPDATE_TX_OFFSET_X
            print "CMD_UPDATE_TX_OFFSET_Y = ", PAT_CMD_UPDATE_TX_OFFSET_Y
            print "CMD_UPDATE_FSM_X = ", PAT_CMD_UPDATE_FSM_X
            print "CMD_UPDATE_FSM_Y = ", PAT_CMD_UPDATE_FSM_Y
            print "CMD_SELF_TEST = ", PAT_CMD_SELF_TEST 
            print "CMD_END_PROCESS (End PAT Binary Execution) = ", PAT_CMD_END_PROCESS                      
            #else:
            #    print ('Unrecognized PAT Status Flag: ' + pat_status_flag)

            user_cmd = int(input("Please enter a command number (enter -1 to skip command entry): ")) 
            if(user_cmd in cmd_list):
                    if(user_cmd in [PAT_CMD_CALIB_LASER_TEST, PAT_CMD_FSM_TEST]):
                            exp_cmd = int(input("Please enter an exposure in us (10 to 10000000): "))
                            if(exp_cmd < 10):
                                    print "Exposure below minimum of 10 us entered. Using 10 us."
                                    exp_cmd = 10
                            elif(exp_cmd > 10000000):
                                    print "Exposure above maximum of 10000000 us entered. Using 10000000 us."
                                    exp_cmd = 10000000
                            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
                            ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, user_cmd, str(exp_cmd))  
                    elif(user_cmd in [PAT_CMD_START_PAT, PAT_CMD_START_PAT_OPEN_LOOP, PAT_CMD_START_PAT_BUS_FEEDBACK, PAT_CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK]):
                            if(int(input('Enter 1 to skip calibration (0 otherwise): ')) == 1):
                                    print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
                                    ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, user_cmd, str(PAT_SKIP_CALIB_FLAG)) 
                            else:
                                    print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
                                    ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, user_cmd, str(PAT_DO_CALIB_FLAG)) 

                    elif(user_cmd == PAT_CMD_GET_IMAGE):
                                window_ctr_rel_x = int(input("Please enter window center X relative to center: "))
                                window_ctr_rel_y = int(input("Please enter window center Y relative to center: "))
                                window_width = int(input("Please enter window width: "))
                                window_height = int(input("Please enter window height: "))
                                exp_cmd = int(input("Please enter an exposure in us (10 to 10000000): "))
                                if((abs(window_ctr_rel_x) <= CAMERA_WIDTH/2 - window_width/2) and (abs(window_ctr_rel_y) < CAMERA_HEIGHT/2 - window_height/2) and (window_width <= CAMERA_WIDTH) and (window_height <= CAMERA_HEIGHT) and (exp_cmd >= CAMERA_MIN_EXP) and (exp_cmd <= CAMERA_MAX_EXP)):
                                        send_pat_command(socket_PAT_control, return_address, PAT_CMD_SET_GET_IMAGE_WINDOW_WIDTH, str(window_width))
                                        time.sleep(0.25)
                                        send_pat_command(socket_PAT_control, return_address, PAT_CMD_SET_GET_IMAGE_WINDOW_HEIGHT, str(window_height))
                                        time.sleep(0.25)
                                        send_pat_command(socket_PAT_control, return_address, PAT_CMD_SET_GET_IMAGE_CENTER_X, str(window_ctr_rel_x))
                                        time.sleep(0.25)
                                        send_pat_command(socket_PAT_control, return_address, PAT_CMD_SET_GET_IMAGE_CENTER_Y, str(window_ctr_rel_y))
                                        time.sleep(0.25)
                                        send_pat_command(socket_PAT_control, return_address, PAT_CMD_GET_IMAGE, str(exp_cmd))
                                else:
                                        print('Error: image parameters out of bounds')

                    elif(user_cmd == PAT_CMD_UPDATE_TX_OFFSET_X):
                            tx_offset_x = int(input("Enter new Tx Offset in X (pixels): "))
                            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
                            ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, user_cmd, str(tx_offset_x))
                    elif(user_cmd == PAT_CMD_UPDATE_TX_OFFSET_Y):
                            tx_offset_y = int(input("Enter new Tx Offset in Y (pixels): "))
                            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
                            ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, user_cmd, str(tx_offset_y))
                    elif(user_cmd == PAT_CMD_UPDATE_FSM_X):
                            fsm_update_x = int(input("Enter FSM X displacement (pixels): "))
                            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
                            ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, user_cmd, str(fsm_update_x))
                    elif(user_cmd == PAT_CMD_UPDATE_FSM_Y):
                            fsm_update_y = int(input("Enter FSM Y displacement (pixels): "))
                            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
                            ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, user_cmd, str(fsm_update_y))
                    elif(user_cmd == PAT_CMD_SET_BEACON_X):
                            beacon_x = int(input("Enter beacon X rel to center (pixels): "))
                            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
                            ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, user_cmd, str(beacon_x))
                    elif(user_cmd == PAT_CMD_SET_BEACON_Y):
                            beacon_y = int(input("Enter beacon Y rel to center (pixels): "))
                            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
                            ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, user_cmd, str(beacon_y))
                    elif(user_cmd == PAT_CMD_SET_BEACON_WINDOW_SIZE):
                            beacon_window = int(input("Enter beacon window width = height (pixels): "))
                            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
                            ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, user_cmd, str(beacon_window))
                    elif(user_cmd == PAT_CMD_SET_BEACON_MAX_EXP):
                            beacon_max_exp = int(input("Enter beacon max exp (us): "))
                            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
                            ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, user_cmd, str(beacon_max_exp))

                    else:
                            print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
                            ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, user_cmd)  
            if(user_cmd == PAT_CMD_END_PROCESS):
                    print "Exiting..."
                    break 

            elif(user_cmd == TURN_ON_CAL_LASER):
                    if(cal_laser_init):
                            power.calib_diode_on()
                    else:
                            initialize_cal_laser()
                            cal_laser_init = True
                    print('CALIBRATION LASER ON')

            elif(user_cmd == TURN_OFF_CAL_LASER):
                    power.calib_diode_off()
                    print('CALIBRATION LASER OFF')

            elif(user_cmd == 0):
                    print "Skipping command entry."

            else:
                    print "Unrecognized command number entered: ", user_cmd, ". Skipping command entry."              
                
        counter = counter + 1        
