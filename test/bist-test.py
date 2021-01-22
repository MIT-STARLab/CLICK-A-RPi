#!/usr/bin/env python
import sys
sys.path.append('/root/fpga/')
import spi
from time import sleep   
import dac_spi     
import math
import csv

#PPM Order
PPM = int(sys.argv[1])
#DAC Resolution used (step size)
RES = int(sys.argv[2])
MAX = 0xFFFF


"""
Initialize the laser to be aligned with the FBG using the specified PPM order
Sets up DAC to maximum values to start depending on the resolution asked for.

"""
def initialize(PPM, dac_threshold):
    #Initialize PD DAC
    print("Starting Laser initialization")
    dac_spi.init("PD")
    

    # start TOSA voltage rails
    spi.write_register(36,85)
    spi.write_register(33,85)

    sleep(.5)
    #Start Laser

    # To put chirp on FBG use
    #Reg 1 = 5
    #Reg 2 = 107
    #Reg 3 = 14
    #Reg 4 = 33

    #TEC
    spi.write_register(1,5)
    spi.write_register(2,107)

    #Bias current
    spi.write_register(3,14)
    spi.write_register(4,33)

    #start PPM Modulation at PPM4
    ppm_order = (128 + (255 >>(8-int(math.log(PPM)/math.log(2)))))
    spi.write_register(13,ppm_order)

    print("Laser Initialized at PPM"+str(PPM))

def scan_reply():
    spi.write_register(24,0)
    spi.write_register(24, 9)
    while((spi.read_register(15) & 0b1) != 1):
        pass
    pass_data = []
    for reg in range(16,24,2):
            pass_data.append(round((256*spi.read_register(reg)+spi.read_register(reg+1))/65536.0,3))
    return pass_data

def step(res, input):
    res = input - (0b1 << (16-res))
    return res & 0xFFFF


def write_dac_thresholds(dac, num, threshold):
    for i in num:
        dac_spi.write_dac(dac, i, threshold)
    #print("Wrote Threshold: "+str(threshold))
    dac_spi.update_dac(dac)

if __name__ == '__main__':
    start_dac_threshold = (MAX << (16-RES)) & 0xFFFF
    initialize(PPM, start_dac_threshold)
    dac = start_dac_threshold
    
    run_data = [['Threshold','Counter','Seed','FBG','Output']]
    steps = 0
    while(dac >= 0):
        steps +=1
        data = scan_reply()
        threshold_data = [dac] + data[:]
        run_data.append(threshold_data)
        if(dac == 0):
            break
        dac = step(RES, dac)
        write_dac_thresholds("PD", [0,1,2], dac)
    print(str(steps) + " thresholds taken for " +str(RES) + "-bit DAC resolution with PPM order " +str(PPM))
    #print("Data: ", run_data)
    with open("bist_data.csv", "wb") as f:
        writer = csv.writer(f)
        for row in run_data:
            writer.writerow(row)





