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
from ipc_packets import PATControlPacket, PATHealthPacket, PATStatusPacket
from zmqTxRx import recv_zmq, send_zmq
import ipc_helper
import fpga_map as mmap
fpga = ipc_helper.FPGAClientInterface()
power = mmap.Power(fpga)

#PAT Status Flag List
status_list = [PAT_STATUS_CAMERA_INIT, PAT_STATUS_STANDBY, PAT_STATUS_MAIN]

#PAT Command List
cmd_list = [PAT_CMD_START_PAT, PAT_CMD_START_PAT_OPEN_LOOP, PAT_CMD_START_PAT_STATIC_POINT, PAT_CMD_START_PAT_BUS_FEEDBACK, PAT_CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK,  PAT_CMD_UPDATE_TX_OFFSET_X, PAT_CMD_UPDATE_TX_OFFSET_Y, PAT_CMD_END_PAT, PAT_CMD_GET_IMAGE, PAT_CMD_CALIB_TEST, PAT_CMD_CALIB_LASER_TEST, PAT_CMD_FSM_TEST, PAT_CMD_BCN_ALIGN, PAT_CMD_TX_ALIGN, PAT_CMD_UPDATE_FSM_X, PAT_CMD_UPDATE_FSM_Y, PAT_CMD_SELF_TEST, PAT_CMD_END_PROCESS]
TURN_ON_CAL_LASER = cmd_list[len(cmd_list)-1] + 1
TURN_OFF_CAL_LASER = TURN_ON_CAL_LASER + 1

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
        
context = zmq.Context()

socket_PAT_health = context.socket(zmq.SUB)
socket_PAT_health.bind("tcp://127.0.0.1:%s" % PAT_HEALTH_PORT)
socket_PAT_health.setsockopt(zmq.SUBSCRIBE, b'')
poller_PAT_health = zmq.Poller()
poller_PAT_health.register(socket_PAT_health, zmq.POLLIN)

socket_PAT_status = context.socket(zmq.SUB)
socket_PAT_status.bind("tcp://127.0.0.1:%s" % PAT_STATUS_PORT)
socket_PAT_status.setsockopt(zmq.SUBSCRIBE, b'')
poller_PAT_status = zmq.Poller()
poller_PAT_status.register(socket_PAT_status, zmq.POLLIN)

socket_PAT_control = context.socket(zmq.PUB)
socket_PAT_control.bind("tcp://127.0.0.1:%s" % PAT_CONTROL_PORT)

# socket needs some time to set up. give it a second - else the first message will be lost
time.sleep(1)

print("\n")

# use PID as unique identifier for this progress
topic = str(os.getpid())
pid = os.getpid()
return_address = str(pid)

# Wait for a ping from the PAT process
#Read telemetry if available
print('RECEIVING on %s' % socket_PAT_health.get_string(zmq.LAST_ENDPOINT))
message = recv_zmq(socket_PAT_health)
ipc_patHealthPacket = PATHealthPacket()
telemetry_string, _, _, _ = ipc_patHealthPacket.decode(message) #decode the package
print(telemetry_string)
time.sleep(1)

counter = 0
command_period_sec = 10
poll_timeout_msec = 250
cal_laser_init = False
while True:    
        socks_health = dict(poller_PAT_health.poll(poll_timeout_msec))
        if socket_PAT_health in socks_health and socks_health[socket_PAT_health] == zmq.POLLIN:
                #Read telemetry if available
                print('RECEIVING on %s' % socket_PAT_health.get_string(zmq.LAST_ENDPOINT))
                message = recv_zmq(socket_PAT_health)
                ipc_patHealthPacket = PATHealthPacket()
                telemetry_string, _, _, _ = ipc_patHealthPacket.decode(message) #decode the package
                print(telemetry_string) 

        socks_status = dict(poller_PAT_status.poll(poll_timeout_msec))
        if socket_PAT_status in socks_status and socks_status[socket_PAT_status] == zmq.POLLIN:
                print('RECEIVING on %s' % socket_PAT_status.get_string(zmq.LAST_ENDPOINT))
		message = recv_zmq(socket_PAT_status)
		ipc_patStatusPacket = PATStatusPacket()
		return_addr, status_flag = ipc_patStatusPacket.decode(message) #decode the package

        #Send commands if no incoming telemetry (standby) or after command_period timeout (allow exiting main pat loop while running)
        if((counter*poll_timeout_msec/1000) % command_period_sec == 0):
		if(status_flag in status_list):
			if(status_flag == PAT_STATUS_CAMERA_INIT):
				print ('PAT Process (PID: ' + str(return_addr) + ') Status: In Camera Initialization Loop')
                                print "Available Commands are: "
                                print "TURN_ON_CAL_LASER = ", TURN_ON_CAL_LASER
                                print "TURN_OFF_CAL_LASER = ", TURN_OFF_CAL_LASER                              
                                print "CMD_START_PAT_STATIC_POINT = ", PAT_CMD_START_PAT_STATIC_POINT
                                print "CMD_SELF_TEST = ", PAT_CMD_SELF_TEST
                                print "CMD_END_PROCESS (End PAT Binary Execution) = ", PAT_CMD_END_PROCESS

			elif(status_flag == PAT_STATUS_STANDBY):
				print ('PAT Process (PID: ' + str(return_addr) + ') Status: In Standby Loop')
                                print "Available Commands are: "
                                print "TURN_ON_CAL_LASER = ", TURN_ON_CAL_LASER
                                print "TURN_OFF_CAL_LASER = ", TURN_OFF_CAL_LASER 
                                print "CMD_START_PAT = ", PAT_CMD_START_PAT
                                print "CMD_START_PAT_OPEN_LOOP = ", PAT_CMD_START_PAT_OPEN_LOOP
                                print "CMD_START_PAT_STATIC_POINT = ", PAT_CMD_START_PAT_STATIC_POINT
                                print "CMD_START_PAT_BUS_FEEDBACK = ", PAT_CMD_START_PAT_BUS_FEEDBACK
                                print "CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK = ", PAT_CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK
                                print "CMD_GET_IMAGE = ", PAT_CMD_GET_IMAGE
                                print "CMD_CALIB_TEST = ", PAT_CMD_CALIB_TEST
                                print "CMD_CALIB_LASER_TEST = ", PAT_CMD_CALIB_LASER_TEST
                                print "CMD_FSM_TEST = ", PAT_CMD_FSM_TEST
                                print "CMD_BCN_ALIGN = ", PAT_CMD_BCN_ALIGN
                                print "CMD_TX_ALIGN = ", PAT_CMD_TX_ALIGN
                                print "CMD_UPDATE_FSM_X = ", PAT_CMD_UPDATE_FSM_X
                                print "CMD_UPDATE_FSM_Y = ", PAT_CMD_UPDATE_FSM_Y
                                print "CMD_SELF_TEST = ", PAT_CMD_SELF_TEST 
                                print "CMD_END_PROCESS (End PAT Binary Execution) = ", PAT_CMD_END_PROCESS

			elif(status_flag == PAT_STATUS_MAIN):
				print ('PAT Process (PID: ' + str(return_addr) + ') Status: In Main Loop')
                                print "Available Commands are: "
                                print "TURN_ON_CAL_LASER = ", TURN_ON_CAL_LASER
                                print "TURN_OFF_CAL_LASER = ", TURN_OFF_CAL_LASER    
                                print "CMD_UPDATE_TX_OFFSET_X = ", PAT_CMD_UPDATE_TX_OFFSET_X
                                print "CMD_UPDATE_TX_OFFSET_Y = ", PAT_CMD_UPDATE_TX_OFFSET_Y
                                print "CMD_END_PAT (Return to Standby) = ", PAT_CMD_END_PAT
                                print "CMD_END_PROCESS (End PAT Binary Execution) = ", PAT_CMD_END_PROCESS                           
		else:
			print ('Unrecognized PAT Status Flag: ' + status_flag)

                user_cmd = int(input("Please enter a command number (enter 0 to skip command entry): ")) 
                if(user_cmd in cmd_list):
                        if(user_cmd in [PAT_CMD_GET_IMAGE, PAT_CMD_CALIB_LASER_TEST, PAT_CMD_FSM_TEST]):
                                exp_cmd = int(input("Please enter an exposure in us (10 to 10000000): "))
                                if(exp_cmd < 10):
                                        print "Exposure below minimum of 10 us entered. Using 10 us."
                                        exp_cmd = 10
                                elif(exp_cmd > 10000000):
                                        print "Exposure above maximum of 10000000 us entered. Using 10000000 us."
                                        exp_cmd = 10000000
                                print('SENDING on %s' % (socket_PAT_control.get_string(zmq.LAST_ENDPOINT)))
                                ipc_patControlPacket = send_pat_command(socket_PAT_control, return_address, user_cmd, str(exp_cmd))     
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
