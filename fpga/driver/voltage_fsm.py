#!/usr/bin/python
import fl
import time
import memorymap
import dac
#FSM PINS = DAC Channel
#1 = x+ Channel 3
#3 = X- Channel 2
#5 = Y- Channel 1
#7 = Y+ Channel 0

if __name__ == "__main__":
    bias = 80
    throw = 50
    max_voltage = 130.0

    bias_bits = round(bias/max_voltage*65536,0)
    throw_bits = round(throw/max_voltage*65536,0)
        
    print("Bias: ", bias_bits, "Throw: ", throw_bits)

    time.sleep(2)
    #dac_initialization
    dac.init(2)
    for i in range(4):
        dac.write_dac(2,i,bias_bits)    
    dac.update_dac(2)

    time.sleep(2)

    while(1):
        #Towards X+ Channel
        dacWrite(2,2,bias_bits-throw_bits)
        dacWrite(2,3,bias_bits+throw_bits)
        dacUpdate(2)
        time.sleep(1)
        #Towards X- Channel
        dacWrite(2,2,bias_bits+throw_bits)
        dacWrite(2,3,bias_bits-throw_bits)
        dacUpdate(2)
        time.sleep(1)
        #Back to Center
        dacWrite(2,2,bias_bits)
        dacWrite(2,3,bias_bits)
        dacUpdate(2)
        time.sleep(1)
        #Toward Y+ Channel
        dacWrite(2,1,bias_bits-throw_bits)
        dacWrite(2,0,bias_bits+throw_bits)
        dacUpdate(2)
        time.sleep(1) 
        #Toward Y- Channel
        dacWrite(2,1,bias_bits+throw_bits)
        dacWrite(2,0,bias_bits-throw_bits)
        dacUpdate(2)
        time.sleep(1)
        #Toward Center
        dacWrite(2,1,bias_bits)
        dacWrite(2,0,bias_bits)
        dacUpdate(2)
        time.sleep(1) 
 

        
        



