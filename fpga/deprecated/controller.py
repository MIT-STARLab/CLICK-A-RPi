#!/usr/bin/env python
import os
from time import sleep
import subprocess
import memorymap
#Register numbers instead of names are used to the descrepancy between the names and the numbers as seen on the FPGA

memMap = memorymap.MemoryMap()  

def write(reg, value, verbose=False):
    output = subprocess.call(["sudo", "python", "fpgadriver.py", "-i", "04b4:8613", "-v", "1d50:602b:0002", "--write",str(reg), str(value)])
    if(verbose):
        return 'None'
def read(reg, verbose=True):
    output = subprocess.check_output(["sudo","python", "fpgadriver.py", "-i", "04b4:8613", "-v", "1d50:602b:0002", "--read",str(reg)])
    if(verbose):
        return int(output.decode("ascii"))
    # print(output.decode("ascii"))

#values must be a list
def cmd(cmd_str, values, verbose=True):
    cmd_str_temp = ["sudo", "python", "fpgadriver.py", "-i", "04b4:8613", "-v", "1d50:602b:0002", "--"+str(cmd_str)]
    for val in values:
        cmd_str_temp.append(str(val))
    output = subprocess.check_output(cmd_str_temp)
    if(verbose):
        return output.decode("ascii")

# Initialize laser settings
def laserInit():
    #if(checkTECPowerOn(handle) == False):
    #Set DAC for TEC Controller before turning on power. Otherwise it will burn out the laser
    #Setting temp to 4,20 should be 25C Use below table as a guide for temps
    #Temps should be set between 35-45C
    # 25C = 8,52
    # 30C = 7,78
    # 35C = 6,114
    # 40C = 5,170
    # 45C = 4,246
    write(1,3,True)
    write(2,200) 
    print("Wrote to TEC registers")
    sleep(.10)
    #Turn on TEC Power/LD Controller Power AND make sure TEC set voltage is non zero
    if(read(1) > 0):
        write(36,85)
        print("TEC Power on")

    #Turn on Current Driver Power to drive current make sure initial values are zero
    write(3,0)
    write(4,0)
    #fl.flWriteChannel(handle,33,85)
    print("LD Bias Power On")
   
    #Set Start current at minimum threshold of 30mA
    #Equation for current: 4.096 *1.1(1/6.81+1/16500)*CODE/4096
    #Code = MSB + LSB
    write(33,85)
    sleep(.25) 
    write(3,15)
    write(4,100)
    print("Laser Current Code set to 60mA")

# Set the seed laser current 
# Input units are in uA
#
def setCurrent(code):
    
    MSB_channel = 3 
    LSB_channel = 4

    #Convert uA to byte assignments
    code = setpoint #finish equation
    first_byte = code/256
    second_byte = code%256

    write(MSB_channel,first_byte)      #writes bytes to channel
    write(LSB_channel,second_byte)
    print("Current set to: " + str(code))

def readCurrent():

    MSB = read(memMap.get_register('LTSa'))
    LSB = read(memMap.get_register('LTSb'))
    return MSB*256+LSB


def setTemp(code):
    
    MSB_channel = 1 
    LSB_channel = 2

    #Convert uA to byte assignments
    code = setpoint #finish equation
    first_byte = code/256
    second_byte = code%256

    write(MSB_channel,first_byte)      #writes bytes to channel
    write(LSB_channel,second_byte)
    print("Temp set to : " + str(code))

#Checks to see if the power switch for the TEC Circuit is on
def checkTECPowerOn():

    status = read(memMap.get_register('PO4'))
    if(status != 85):
        return False
    else:
        return True

#Checks to see if the power switch for the laser bias Circuit is on
def checkLDPowerOn():
    
    if(read(33) != 85):
        return False
    else:
        return True

#turns on DAC and sends minimum values to the photodiode comparators
def thresholdInit(reg,value):
    cmd('dacInit', [1])
    if((reg < 4 or reg == 7) and (value <65000 or value > 0)):
        voltage = int(value*(2.5/3.3)) 
        cmd('dacWrite', [1, reg, value])
    else:
        print("FSM DAC reg or value are invalid")
    cmd('dacUpdate', [1])
    print("PD DAC Line " + str(reg)+ " updated to "+str(round(value/65000.0*3.3,3))+" volts")


def debounce(length,reg,pin,val):
    pos = 0.0
    mask = 0b00000001 << pin
    for i in range(length):
        if (read(reg & mask) == val):
            pos+=1
    return pos/length


#This will find the center of the fbg
def findCenter():

    laserInit()
    thresholdInit(0,20000) #CHECK THE STARTING POINT HERE
    thresholdInit(1,45000)
    thresholdInit(2,10000)
    sleep(2)
    # print("Initialization Complete")
    while(read(memMap.get_register('PDI')) < 3):
            
        #check starting temp
        MSB = read(memMap.get_register('LTSa'))
        LSB = read(memMap.get_register('LTSb'))

        value = MSB*256 + LSB
        new_value = value +1
        new_MSB = new_value/256
        new_LSB = new_value%256
        reg64 = read(memMap.get_register('PDI'))
        print(new_value, MSB, new_MSB, LSB, new_LSB, "Power: ", reg64)
        write(memMap.get_register('LTSa'),new_MSB)
        write(memMap.get_register('LTSb'),new_LSB)
        if(new_value < (900) or new_value>1800):
            print("Center frequency was not matched")
            return
        
    print("matched center frequency")

if __name__ == '__main__':
    findCenter()
