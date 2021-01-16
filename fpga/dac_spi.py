import sys
sys.path.append('/root/fpga/')
import spi
from time import sleep

#PD DAC = "PD"
#CALIBRATION laser channel 4 on PD DAC
#FSM DAC = FSM

def init(dac):
    reg = []
    #pd_dac
    if(dac == "PD"):
        reg = [5,6,7]
    #fsm_dac	
    elif (dac == "FSM"):
        reg = [8,9,10]
    #none
    else:
        print("Invalid DAC, use PD or FSM")	
        reg = [0,0,0]

    reset(reg)
    sleep(0.1)
    enable_int_ref(reg)
    sleep(0.1)
    enable_all_dacs(reg)
    sleep(0.1)
    enable_sw_ldac(reg)
    sleep(0.1)
    return

#Resets registers to 0: 
#register 7 value 1: DAC, Input, LDAC, Power-down, internal reference setup
#register 7 value 0: DAC register & Input Shift register only
def reset(dac):
    spi.write_register(dac[0], 0x28)
    sleep(0.1)
    spi.write_register(dac[1], 0)
    sleep(0.1)
    spi.write_register(dac[2], 0x01)
    return
#internal reference on register 7 value 1 = on, 0 = off
def enable_int_ref(dac):
    spi.write_register(dac[0], 0x38)
    sleep(0.1)
    spi.write_register(dac[1], 0)
    sleep(0.1)
    spi.write_register(dac[2], 0x01)
    return

#Powers up all DAC's to normal operation mode
#Setting DB4&5 can fix the output impedance of the DAC
def enable_all_dacs(dac):
    spi.write_register(dac[0], 0x20)
    sleep(0.1)
    spi.write_register(dac[1], 0)
    sleep(0.1)
    spi.write_register(dac[2], 0x0F)
    return
#latch outputs to current register value 
#input register will not affect the output
#changing DB3-0 will update dac registers when new data is read in
def enable_sw_ldac(dac):
    spi.write_register(dac[0], 0x30)
    sleep(0.1)
    spi.write_register(dac[1], 0)
    sleep(0.1)
    spi.write_register(dac[2], 0)
    return
#write to single register N
def write_dac(dac, addr, value):
    reg = []
    #pd_dac
    if(dac == "PD"):
    	reg = [5,6,7]
    #fsm_dac	
    elif (dac == "FSM"):
	    reg = [8,9,10]
    #none
    else:
        print("Invalid DAC, use PD or FSM")
        reg = [0,0,0]

    spi.write_register(reg[0], addr&0x7)
    sleep(0.1)
    spi.write_register(reg[1], (value&0xFF00)>>8)
    sleep(0.1)
    spi.write_register(reg[2], value&0xFF)
    return

#updates all DAC's to current input values
def update_dac(dac):
    reg = []
    #pd_dac
    if(dac == "PD"):
    	reg = [5,6,7]
    #fsm_dac	
    elif (dac == "FSM"):
    	reg = [8,9,10]
    #none
    else:
        print("Invalid DAC, use PD or FSM")	
        reg =[0,0,0]

    spi.write_register(reg[0], 0x0F)
    sleep(0.1)
    spi.write_register(reg[1], 0)
    sleep(0.1)
    spi.write_register(reg[2], 0)
    return
