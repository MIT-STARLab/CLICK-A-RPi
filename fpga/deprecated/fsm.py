#!/usr/bin/env python
# Joe Kusters
# helper functions for writing to FSM DAC (AD5664R)
import fl
from time import sleep

def init(handle):
	reset(handle)
	sleep(0.001)
	enable_int_ref(handle)
	sleep(0.001)
	enable_all_dacs(handle)
	sleep(0.001)
	enable_sw_ldac(handle)
	sleep(0.001)
	return

#Resets registers to 0: 
#register 7 value 1: DAC, Input, LDAC, Power-down, internal reference setup
#register 7 value 0: DAC register & Input Shift register only
def reset(handle):
	fl.flWriteChannel(handle, 8, 0x28)
	sleep(0.001)
	fl.flWriteChannel(handle, 9, 0)
	sleep(0.001)
	fl.flWriteChannel(handle, 10, 0x01)
	return
#internal reference on register 7 value 1 = on, 0 = off
def enable_int_ref(handle):
	fl.flWriteChannel(handle, 8, 0x38)
	sleep(0.001)
	fl.flWriteChannel(handle, 9, 0)
	sleep(0.001)
	fl.flWriteChannel(handle, 10, 0x01)
	return

#Powers up all DAC's to normal operation mode
#Setting DB4&5 can fix the output impedance of the DAC
def enable_all_dacs(handle):
	fl.flWriteChannel(handle, 8, 0x20)
	sleep(0.001)
	fl.flWriteChannel(handle, 9, 0)
	sleep(0.001)
	fl.flWriteChannel(handle, 10, 0x0F)
	return
#latch outputs to current register value 
#input register will not affect the output
#changing DB3-0 will update dac registers when new data is read in
def enable_sw_ldac(handle):
	fl.flWriteChannel(handle, 8, 0x30)
	sleep(0.001)
	fl.flWriteChannel(handle, 9, 0)
	sleep(0.001)
	fl.flWriteChannel(handle, 10, 0)
	return
#write to single register N
def write_dac(handle, addr, value):
	fl.flWriteChannel(handle, 8, addr&0x7)
	sleep(0.001)
	fl.flWriteChannel(handle, 9, (value&0xFF00)>>8)
	sleep(0.001)
	fl.flWriteChannel(handle, 10, value&0xFF)
	return
#updates all DAC's to current input values
def update_dac(handle):
	fl.flWriteChannel(handle, 8, 0x0F)
	sleep(0.001)
	fl.flWriteChannel(handle, 9, 0)
	sleep(0.001)
	fl.flWriteChannel(handle, 10, 0)
	return

