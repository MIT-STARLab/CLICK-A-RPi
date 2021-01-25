#!/usr/bin/env python
import sys
sys.path.append('/root/fpga/')
import spi
from time import sleep   
import math
import csv
import datetime


if __name__ == '__main__':
    curr_time = datetime.time
    wait_time = 2
    print(curr_time)
    spi.write_register(35,85)
    spi.write_register(37,85)
    spi.write_register(38,85)


    log = [["time", "PD", "EDFA", "Camera", "TOSA", "Lens", "Raceway"]]
    for step in range(10):
        entry = [step*wait_time]
        for reg in range(98,110,2):
            val = ((spi.read_register(reg)&0x0F)*256) + spi.read_register(reg+1)
            if(val <=4095):
                dec_val = round((-920.0/1000/(((2.5*val/(2**12*8.5)) - 1.65)/3.3) -920.0/1000 -1)/(3.81*10**-3),2) 
            else:
                print("invalid value: ", val)
            entry.append(val)
        log.append(entry)
        sleep(wait_time)


    spi.write_register(38,15)
    spi.write_register(37,15)
    spi.write_register(35,15)
    print("Complete")


    with open("temp_data.csv", "wb") as f:
        writer = csv.writer(f)
        for row in log:
            writer.writerow(row)





