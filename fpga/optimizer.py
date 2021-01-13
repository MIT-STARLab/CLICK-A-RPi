# Joe Kusters
# optimizer.py
# optimizer class for running the Laser Calibration optimization control loop based on Myron Lee's SM Thesis (2017)

import fl;
import math as m

class Optimizer():
    def __init__(self, handle, temp=0, current=0):
        self.handle = handle
        self.temp = temp
        self.measTemp = 0
        self.current = current

    def dither(self):

        # TODO
        # need to turn on temp & current control or nah?
        # units for temp and current control
        # wait time needed for temperature measurement? (lag in controller)

        obslength = float(argList.ser)

        # turn Laser Diode Temp and Current Controllers on
        # (needed with this code? if so, fill in below)

        #optimization code
        current = 125
        self.setCurrent(current)
            
        temp = 38
        self.setTemp(temp)

        #get temp/current again since commanded current/temp may not be 100% accurate
        self.getTemp()

        # measure SER
        ser = self.measureSER()
        print('Begin Algorithm')
        ntemp = self.temp
        ncurrent = self.current
        retries = 0
        down = False
        tser = SER_VAL
        #Vary temperature/current by smallest resolution zzz
        zzz = 0.1
	    
        while tser > THtemp:
           #safety check
            if (ntemp >= 50):
                print("temp exceeded")
                break
            if down:
                ntemp = ntemp - zzz
            else:
                ntemp = ntemp + zzz

            self.setTemp(ntemp)
#necessary?
            time.sleep(2)
            nser = self.measureSER()

            if nser > tser*10:
                if retries > MAX_RETRIES:
                    raise ValueError('Too many retries, possible infinite loop')
                else:
                    retries += 1
            print("retry++")
            down = not down
            tser = nser  #keeps track of old SER each loop

        #optimize for current until below threshold
        cser = tser
        while cser > THcurrent:
            #safety check
            if (ncurrent >= 138):
                print ("curr exceeded")
                break
            if down:
                ncurrent = ncurrent - zzz
            else:
                ncurrent = ncurrent + zzz
            self.setCurrent(ncurrent)
#necessary?
            time.sleep(2)
            nser = self.measureSER()

            if nser > cser*10:
                if retries > MAX_RETRIES:
                    raise ValueError('Too many retries, possible infinite loop')
                else:
                    retries += 1
            print("retry++")
            down = not down
            cser = nser #keeps track of old ser each loop

        print("Minimum SER reached, algorithm ending; ntemp: %f; ncurrent: %f"%(ntemp,ncurrent))

    def measureSER(self):
        ser = 0
        return ser

    def setCurrent(self, current):
        self.current = current
        currentMSB = 0xf00&self.current
        currentLSB = 0xff&self.current
        fl.flWriteChannel(self.handle, memMap.get_register('LBCa'), currentMSB)
        fl.flWriteChannel(self.handle, memMap.get_register('LBCb'), currentLSB)

    def setTemp(self, temp):
        self.temp = temp
        tempMSB = 0xf00&self.temp
        tempLSB = 0xff&self.current
        fl.flWriteChannel(self.handle, memMap.get_register('LTSa'), tempMSB)
        fl.flWriteChannel(self.handle, memMap.get_register('LTSb'), tempLSB)

    def getTemp(self):
        self.measTemp = fl.flReadChannel(self.handle, memMap.get_register('LTMa'))<<8
        self.measTemp += fl.flReadChannel(self.handle, memMap.get_register('LTMb'))
