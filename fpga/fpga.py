# Joe Kusters
# fpga.py
# main program for handling all fpga processes (downlink, optimizer, interface, fault detection)

import fl
import sys
import time
import zmq
import memorymap as mem
import modulator as mod
import optimizer as opt
import fdir
import encoder
import interleaver

memory = mem.MemoryMap()
handle = fl.FLHandle()

fl.flInitialise(0)
vp = "1d50:602b:0002"
# check lsusb to see what the current prodcut ID is for the FPGA/Cypress chip, and uncomment accordingly
ip = '04b4:8613' 
#ip = '1d50:602b'
try: 
    handle = fl.flOpen(vp)
except fl.FLException as ex:
    if(ip):
        fl.flLoadStandardFirmware(ip, vp)
        time.sleep(3)
        if(not fl.flAwaitDevice(vp, 10000)):
            raise fl.FLException("Device did not renumerate properly")
        handle = fl.flOpen(vp)
    else:
        raise fl.FLException("Couldn't open connection to FPGA".format(vp))

conduit = 1
isNeroCapable = fl.flIsNeroCapable(handle)
isCommCapable = fl.flIsCommCapable(handle, conduit)
fl.flSelectConduit(handle, conduit)

if(isCommCapable):
    modulator = mod.Modulator(4, handle)
    optimizer = opt.Optimizer(handle)
    walle = fdir.RepairBot(handle)
    
    # configure fpga
    if(isNeroCapable):
        print('Reprogramming FPGA...')
        fl.flProgram(handle, 'J:A7A0A3A1:/home/pi/TopLevel_FPGAboard_v9.xsvf')
        print('Done!')


context = zmq.Context()

receiver = context.socket(zmq.PULL)
receiver.connect("tcp://localhost:5556")

sender = context.socket(zmq.PUSH)
sender.bind("tcp://*:5557")

while(1):
    # poll command queue to see if any commands for FPGA (downlink, FSMs, new config, etc.)
    msg = receiver.recv()
    #simply print to console for now, write implemenetation later
    print(msg)
    sender.send("received message " + msg)
    msg = msg.split()
    # execute any commands
    if(msg[0] == 'downlink'):
        encoded = 'encoded_'+msg[1]
        interleaved = 'interleaved_'+msg[1]
        encoder.encodeFile(msg[1], 255, 223, True, encoded)
        #interleaver.intrlvFile(encoded, modulator.get_ppm_order(), 255, 223, True, interleaved)
        modulator.downlink(encoded)

    # run optimizer loop, generates telemetry
    optimizer.dither()

    # run fault detection, generates telemetry
    walle.detect()
    if(walle.problem()):
        walle.respond()

    # send any telemetry to bus

    time.sleep(1)

#def startup():
    # turn shit on, check power levels, run initial optimization loop, then enter main loop of checking command fifo, optimizer run through, fdir run through, repeat
