#!/usr/bin/env python
import fl
import time
import os

ivp = "04b4:8613"
vp = "1d50:602b:0002"
progConfig = "J:A7A0A3A1:/root/bin/fpga.xsvf"
usbDevPath = "/dev/bus/usb/001/"
timeoutCounter = 0
numDev = 0

# Wait up to 5 seconds for USB enumerations to finish
while timeoutCounter < 20:
    try:
        numDev = len(os.listdir(usbDevPath))
        if numDev >= 4:
            break
    except:
        pass
    time.sleep(0.25)
    timeoutCounter += 1

print("Found %d USB devices" % numDev)

try:
    print("Initializing FPGALink library...")
    fl.flInitialise(3)
    print("Attempting to open connection to FPGALink device {}...".format(vp))
    try:
        handle = fl.flOpen(vp)
    except fl.FLException as ex:
        print(ex)
        print("Loading standard firmware into RAM {}...".format(ivp))
        fl.flLoadStandardFirmware(ivp, vp)
        time.sleep(3)
        fl.flAwaitDevice(vp, 10000)
        print("Attempting to open connection to FPGALink device {} again...".format(vp))
        handle = fl.flOpen(vp)

    conduit = 1
    isNeroCapable = fl.flIsNeroCapable(handle)
    isCommCapable = fl.flIsCommCapable(handle, conduit)
    fl.flSelectConduit(handle, conduit)
    
    if ( isNeroCapable ):
        print("Programming FPGA with {}...".format(progConfig))
        fl.flProgram(handle, progConfig)
        print("Programming successful")
    else:
        raise fl.FLException("Device does not support NeroProg")

except fl.FLException as ex:
    print(ex)
finally:
    fl.flClose(handle)
