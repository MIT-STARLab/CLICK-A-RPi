#!/usr/bin/env python
import fl
from time import sleep
import memorymap
#Register numbers instead of names are used to the descrepancy between the names and the numbers as seen on the FPGA

memMap = memorymap.MemoryMap()
# Initialize laser settings
def laserInit(handle):
    #if(checkTECPowerOn(handle) == False):
    #Set DAC for TEC Controller before turning on power. Otherwise it will burn out the laser
    #Setting temp to 4,20 should be 25C Use below table as a guide for temps
    #Temps should be set between 35-45C
    # 25C = 8,52
    # 30C = 7,78
    # 35C = 6,114
    # 40C = 5,170
    # 45C = 4,246
    fl.flWriteChannel(handle,1,4)
    fl.flWriteChannel(handle,2,100) 
    print("Wrote to TEC registers")
    sleep(.10)
    #Turn on TEC Power/LD Controller Power AND make sure TEC set voltage is non zero
    if(fl.flReadChannel(handle,1) > 0):
        fl.flWriteChannel(handle,36,85)
        print("TEC Power on")

    #Turn on Current Driver Power to drive current make sure initial values are zero
    fl.flWriteChannel(handle,3,0)
    fl.flWriteChannel(handle,4,0)
    #fl.flWriteChannel(handle,33,85)
    print("LD Bias Power On")
   
    #Set Start current at minimum threshold of 30mA
    #Equation for current: 4.096 *1.1(1/6.81+1/16500)*CODE/4096
    #Code = MSB + LSB
    fl.flWriteChannel(handle,33,85)
    sleep(.25) 
    fl.flWriteChannel(handle,3,15)
    fl.flWriteChannel(handle,4,100)
    print("Laser Current Code set to 60mA")

# Set the seed laser current 
# Input units are in uA
#
def setCurrent(handle,code):
    
    MSB_channel = 3 
    LSB_channel = 4

    #Convert uA to byte assignments
    code = setpoint #finish equation
    first_byte = code/256
    second_byte = code%256

    fl.flWriteChannel(handle,MSB_channel,first_byte)      #writes bytes to channel
    fl.flWriteChannel(handle,LSB_channel,second_byte)
    print("Current set to: " + str(code))

def readCurrent(handle):

    MSB = fl.flReadChannel(handle,memMap.get_register('LTSa'))
    LSB = fl.flReadChannel(handle,memMap.get_register('LTSb'))
    return MSB*256+LSB


def setTemp(handle,code):
    
    MSB_channel = 1 
    LSB_channel = 2

    #Convert uA to byte assignments
    code = setpoint #finish equation
    first_byte = code/256
    second_byte = code%256

    fl.flWriteChannel(handle,MSB_channel,first_byte)      #writes bytes to channel
    fl.flWriteChannel(handle,LSB_channel,second_byte)
    print("Temp set to : " + str(code))

#Checks to see if the power switch for the TEC Circuit is on
def checkTECPowerOn(handle):

    status = fl.flReadChannel(handle, memMap.get_register('PO4'))
    if(status != 85):
        return False
    else:
        return True

#Checks to see if the power switch for the laser bias Circuit is on
def checkLDPowerOn(handle):
    
    if(fl.flReadChannel(handle,33) != 85):
        return False
    else:
        return True

#turns on DAC and sends minimum values to the photodiode comparators
def thresholdInit(handle,reg,value):
    initDAC(handle)
    #print("PD DAC Initialized")
    if((reg < 4 or reg == 7) and (value <65000 or value > 0)):
        voltage = int(value*(2.5/3.3)) 
        write_PD_dac(handle, reg, voltage)
        #print("PD DAC written to")
    else:
        print("FSM DAC reg or value are invalid")
    update_PD_dac(handle)
    print("PD DAC Line " + str(reg)+ " updated to "+str(round(value/65000.0*3.3,3))+" volts")


def debounce(handle,length,reg,pin,val):
    pos = 0.0
    mask = 0b00000001 << pin
    for i in range(length):
        if (fl.flReadChannel(handle, reg & mask) == val):
            pos+=1
    return pos/length


#his will find the center of the fbg
def findCenter(handle, init_only):

    laserInit(handle)
    thresholdInit(handle, 0, 15000) #CHECK THE STARTING POINT HERE
    thresholdInit(handle, 1, 15000)
    thresholdInit(handle, 2, 15000)
    sleep(5)
    print("Initialization Complete")
    if(init_only):
        return
    while(fl.flReadChannel(handle,memMap.get_register('PDI')) < 1):
            
            #check starting temp
        MSB = fl.flReadChannel(handle,memMap.get_register('LTSa'))
        LSB = fl.flReadChannel(handle,memMap.get_register('LTSb'))

        value = MSB*256 + LSB
        new_value = value +1
        new_MSB = new_value/256
        new_LSB = new_value%256
        fl.flWriteChannel(handle,memMap.get_register('LTSa'),new_MSB)
        fl.flWriteChannel(handle,memMap.get_register('LTSb'),new_LSB)
        sleep(.15)
        if(new_value < 1100 or new_value >1800):
            print("Center frequency was not matched")
            return
        reg64 = fl.flReadChannel(handle,memMap.get_register('PDI'))
        print(new_value, MSB, new_MSB, LSB, new_LSB,"Power level: ", reg64)
    print("matched center frequency")

# Author Joe Kusters
# DAC helper functions
# Set up for AD5664R on the Photodiode board

def initDAC(handle):
    reset(handle)
    sleep(0.001)
    enable_int_ref(handle)
    sleep(0.001)
    enable_all_outputs(handle)
    sleep(0.001)
    enable_sw_ldac(handle)
    sleep(0.001)
    return

#Resets registers to 0: 
#register 7 value 1: DAC, Input, LDAC, Power-down, internal reference setup
#register 7 value 0: DAC register & Input Shift register only
def reset(handle):
    sleep(0.001)
    fl.flWriteChannel(handle, 7, 0x01)
    return
#internal :reference on register 7 value 1 = on, 0 = off
def enable_int_ref(handle):
    fl.flWriteChannel(handle, 5, 0x38)
    sleep(0.001)
    fl.flWriteChannel(handle, 6, 0)
    sleep(0.001)
    fl.flWriteChannel(handle, 7, 0x01)
    return

#Powers up all outputs to normal operation mode
#Setting DB4&5 can fix the output impedance of the DAC
def enable_all_outputs(handle):
    fl.flWriteChannel(handle, 5, 0x20)
    sleep(0.001)
    fl.flWriteChannel(handle, 6, 0)
    sleep(0.001)
    fl.flWriteChannel(handle, 7, 0x0F)
    return
#latch outputs to current register value 
#input register will not affect the output
#changing DB3-0 will update dac registers when new data is read in
def enable_sw_ldac(handle):
    fl.flWriteChannel(handle, 5, 0x30)
    return

#write to single register N
def write_PD_dac(handle, addr, value):
    fl.flWriteChannel(handle, 5, addr&0x7)
    sleep(0.001)
    fl.flWriteChannel(handle, 6, (value&0xFF00)>>8)
    sleep(0.001)
    fl.flWriteChannel(handle, 7, value&0xFF)
    return
#updates all DAC's to current input values
def update_PD_dac(handle):
    fl.flWriteChannel(handle, 5, 0x0F)
    sleep(0.001)
    fl.flWriteChannel(handle, 6, 0)
    sleep(0.001)
    fl.flWriteChannel(handle, 7, 0)
    return
