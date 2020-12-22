# fsm.py
# Joe Kusters
# helper functions for writing to FSM DAC (AD5664R)
import fl
from time import sleep

#PD DAC = 1
#CALIBRATION laser channel 4 on PD DAC
#FSM DAC = 2

def init(handle, dac):
	#pd_dac
	if(dac == 1):
		reg = [5,6,7]
	#fsm_dac	
	elif (dac == 2):
		reg = [8,9,10]
	#none
	else:
		reg = [0,0,0]	

	reset(handle,reg)
	sleep(0.001)
	enable_int_ref(handle,reg)
	sleep(0.001)
	enable_all_dacs(handle,reg)
	sleep(0.001)
	enable_sw_ldac(handle,reg)
	sleep(0.001)
	return

#Resets registers to 0: 
#register 7 value 1: DAC, Input, LDAC, Power-down, internal reference setup
#register 7 value 0: DAC register & Input Shift register only
def reset(handle,dac):
	fl.flWriteChannel(handle, dac[0], 0x28)
	sleep(0.001)
	fl.flWriteChannel(handle, dac[1], 0)
	sleep(0.001)
	fl.flWriteChannel(handle, dac[2], 0x01)
	return
#internal reference on register 7 value 1 = on, 0 = off
def enable_int_ref(handle,dac):
	fl.flWriteChannel(handle, dac[0], 0x38)
	sleep(0.001)
	fl.flWriteChannel(handle, dac[1], 0)
	sleep(0.001)
	fl.flWriteChannel(handle, dac[2], 0x01)
	return

#Powers up all DAC's to normal operation mode
#Setting DB4&5 can fix the output impedance of the DAC
def enable_all_dacs(handle,dac):
	fl.flWriteChannel(handle, dac[0], 0x20)
	sleep(0.001)
	fl.flWriteChannel(handle, dac[1], 0)
	sleep(0.001)
	fl.flWriteChannel(handle, dac[2], 0x0F)
	return
#latch outputs to current register value 
#input register will not affect the output
#changing DB3-0 will update dac registers when new data is read in
def enable_sw_ldac(handle,dac):
	fl.flWriteChannel(handle, dac[0], 0x30)
	sleep(0.001)
	fl.flWriteChannel(handle, dac[1], 0)
	sleep(0.001)
	fl.flWriteChannel(handle, dac[2], 0)
	return
#write to single register N
def write_dac(handle, dac, addr, value):
	#pd_dac
	if(dac == 1):
		reg = [5,6,7]
	#fsm_dac	
	elif (dac == 2):
		reg = [8,9,10]
	#none
	else:
		reg = [0,0,0]	

	fl.flWriteChannel(handle, reg[0], addr&0x7)
	sleep(0.001)
	fl.flWriteChannel(handle, reg[1], (value&0xFF00)>>8)
	sleep(0.001)
	fl.flWriteChannel(handle, reg[2], value&0xFF)
	return
#updates all DAC's to current input values
def update_dac(handle, dac):
	#pd_dac
	if(dac == 1):
		reg = [5,6,7]
	#fsm_dac	
	elif (dac == 2):
		reg = [8,9,10]
	#none
	else:
		reg = [0,0,0]	

	fl.flWriteChannel(handle, reg[0], 0x0F)
	sleep(0.001)
	fl.flWriteChannel(handle, reg[1], 0)
	sleep(0.001)
	fl.flWriteChannel(handle, reg[2], 0)
	return