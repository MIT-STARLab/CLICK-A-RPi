#Wavelength Alignment Functions

import fl
import time
import math as m
from datetime import datetime
from mmap import Tester
#from bench import LaserController

#*assuming fl,handle and fpga objects already created
class BOptimizer(object):

    """ Class Initiation """
    def __init__(self, handle, fpga, config):
        self.handle = handle
        self.fpga = fpga
        self.mem_map = Tester(config, handle) 

        #Initiate TEC seed laser operating point parameters
        self.temp = 0
        self.current = 0

        #Track adjustments made to previous setpoint and present setpoint
        self.delta_T = 0
        self.delta_C = 0
    
    def getMemMap(self):
        return self.mem_map
        
    """ get functions (check present TEC seed laser operating point) """

    def getTemp(self):
        return self.temp

    def getCurrent(self):
        return self.current

    def getMemMap(self):
        return self.mem_map

    """ set functions (update present TEC seed laser operating point) """

    def setTemp(self,new_temp):
        self.delta_T = new_temp - self.getTemp()
        self.temp = new_temp
        

    def setCurrent(self,new_current):
        self.delta_C = new_current - self.getCurrent()
        self.current = new_current

    """ LaserController functions """
     #TODO: May need to update the SPI channels

    def setLaserCurrent(self, comm_current):
        
        #Current Consumption
        #MSB_channel = 3    #LBCa
        MSB_channel = self.getMemMap().getAddress('LBCa')
        #LSB_channel = 4    #LBCb
        LSB_channel = self.getMemMap().getAddress('LBCb')

        #Convert commanded current to bytes
	#What are the units of this calculation??	
        code = comm_current/(4.096*1.1*((1/6.81)+(1/16500)))*4096
        first_byte, second_byte = self.code2bytes(code)

        fl.flWriteChannel(self.handle,MSB_channel,first_byte)      #writes bytes to channel
        fl.flWriteChannel(self.handle,LSB_channel,second_byte)

    def setLaserTemp(self, comm_temp):
        
        #Temp Set Point
        #MSB_channel = 23    #LTSa
        MSB_channel = self.getMemMap().getAddress('LTSa')  
        #LSB_channel = 24    #LTSb
        LSB_channel = self.getMemMap().getAddress('LTSb')

        #TODO Constants are estimated; may need to verify with vendor
        R_known = 10000
        Vcc = 0.8
        B = 3900
        R_0 = 10000
        T_0 = 25 
        
        #converts input/commanded temp (comm_temp) to voltage
        V_set = Vcc/(((m.exp(B/comm_temp)*(R_0 * m.exp(-B/T_0)))/R_known)+1)
        V_code = self.voltage2code(V_set) #convert voltage to code
        fb, sb = self.code2byte(V_code) #convert code to bytes
        
        fl.flWriteChannel(self.handle,MSB_channel, fb)
        fl.flWriteChannel(self.handle,LSB_channel, sb)


    def getLaserCurrent(self):

        #Current Consumption 3
        #MSB_channel = 100   #CC3a
        MSB_channel = self.getMemMap().getAddress('CC3a')
        #LSB_channel = 101   #CC3b
        LSB_channel = self.getMemMap().getAddress('CC3b')

        rxm = fl.flReadChannel(self.handle, MSB_channel)
        rxl = fl.flReadChannel(self.handle, LSB_channel)

        #converts the bytes read to a current value
        return (rxm*256 + rxl)/4096 * (4.096*1.1*((1/6.81)+(1/16500)))


    def getLaserTemp(self):

        #Measured Temp
        #MSB_channel = 116   #LTMa
        MSB_channel = self.getMemMap().getAddress('LTMa')

        #LSB_channel = 117   #LTMb
        LSB_channel = self.getMemMap().getAddress('LTMb')
        
        rxm = fl.flReadChannel(self.handle,MSB_channel)
        rxl = fl.flReadChannel(self.handle,LSB_channel)

        code_meas = rxm*256 + rxl           #byte to code
        V_meas = self.code2voltage(code_meas)

        #Converts voltage to temperature
        R_t = R_known * (Vcc/V_meas - 1)
        T = B/m.log(R_t/R_0 * m.exp(-B/T_0))

        return T
    
    def code2byte(self, code):
        fb = code/256   #first byte, second byte
        sb = code%256
        return fb, sb

    def voltage2code(self, v):
        max_code = 2**12 #assuming 12-bit ADC
        V_cc = 3.3  #assuming 3.3V source
        return v*(max_code/3.3)

    def code2voltage(self, c):
        max_code = 2**12 #assuming 12-bit ADC
        V_cc = 3.3 #assuming 3.3V source
        return c*(V_cc/max_code)


    """ Functions for alignment algorithm """

    # Scan mode can either assume no known operating setpoint or start at the current operating point
    # and outputs an operating point using hill-climbing search for temp and bias_current
    # #this is where we might need reset condition
    def scan_mode(self, max_retries, THtemp, THcurrent, M, reset= True):
        """
        Args:
        max_retries: max number of times algorithm should switch from inc to dec
        THtemp: threshold to reach before optimizing current
        THcurrent: threshold to reach before completing algorithm
	M: ppm order
        """

        gpib_port = "/dev/ttyUSB0"
        controller = LaserController(gpib_port,20) #object used to adjust temp and current
        #enable output modes
        controller.setLaserOutput(1);
        time.sleep(5)
        controller.setTECOutput(1);
        time.sleep(5)

        obslength = 1 #constant for waiting after resetting counters when measuring SER
        zzz = 0.1 #amount to alter temp/current every step
        start = datetime.now()

        #optimization code
        if (reset): #reset condition goes here    
            current = 127
            controller.setLaserCurrent(current)
            time.sleep(2)
            while not (current - 0.1 <= round(controller.getLaserCurrent(),1) <= current + 0.1):
                controller.setLaserCurrent(current)
                time.sleep(1)
            
            temp = 39
            controller.setLaserTemp(temp)
            time.sleep(2)
            while not(m.floor(controller.getLaserTemp()) == round(temp,1)):
                time.sleep(1)

        #get temp/current again since commanded current/temp may not be 100% accurate
        temp = controller.getLaserTemp()
        current = controller.getLaserCurrent()
        print("New temperature: %f, new current: %f"%(temp, current))
        print("Measuring slot error rate...")
        cycles,errors,ones,ser = self.fpga.measureSER(obslength=obslength)
        print(" cycles = 0x%-12X"%(cycles))
        print(" errors = 0x%-12X"%(errors))
        print(" ones   = 0x%-12X target=0x%-12X"%(ones,cycles/M))
        print(" SlotER = %e"%(ser))

        print('Begin Algorithm')
        ntemp = temp #keeps track of initial temp and current
        ncurrent = current
        retries = 0 #counter for when algo switches from inc to dec
        down = False #whether or not the algo increases or decreases
        tser = ser #keeps track of original SER
    
        while tser > THtemp:
            #safety check
            if (ntemp >= 50):
                print("temp exceeded")
                break
            if down:
                ntemp = ntemp - zzz
            else:
                ntemp = ntemp + zzz

            controller.setLaserTemp(ntemp)
            time.sleep(2)
            ncycles, nerrors, nones, nser = self.fpga.measureSER(obslength=obslength)
            print("New temperature: %f, nser: %e" %(ntemp, nser))

            if nser > tser*2:
                if retries > max_retries:
                    raise ValueError('Too many retries, possible infinite loop')
                else:
                    retries += 1
                    down = not down
            tser = nser  #keeps track of old SER each loop

        #optimize for current until below threshold
        cser = tser
        while cser > THcurrent:
            #safety check
            if (ncurrent >= 138):
                print("current exceeded")
                break
            if down:
                ncurrent = ncurrent - zzz
            else:
                ncurrent = ncurrent + zzz
            controller.setLaserCurrent(ncurrent)
            time.sleep(2)
            ncycles, nerrors, nones, nser = self.fpga.measureSER(obslength=obslength)
            print("New current: %f, nser: %e" %(ncurrent,nser))

            if nser > cser*2:
                if retries > max_retries:
                    raise ValueError('Too many retries, possible infinite loop')
                else:
                    retries += 1
                    down = not down
            cser = nser #keeps track of old ser each loop
        now = datetime.now()
        total_min = (now-start).total_seconds()/60
	print 'Time elsapsed: ', total_min
	print 'current time: ', now	
	print 'Minimum SER reached, algorithm ending; end temp: ', ntemp, ' end current: ', ncurrent
        return (ntemp, ncurrent, retries, now, total_min)
        
    

    # TEC power consumption optimization feature (require ambient temperature reading from TOSA RTD)
    def power_opt(self,T_amb):
        
        #Seed laser model (output wavelength dependencies)
        dw_dt = 0.088 #change_in_ wavelength(nm)/change_in_temp(C)
        dw_dc = 0.0036 #change_in_wavelength(nm)/change_in_current(mA)

        
        #Seed laser model (power consumption)
        
        """ P_temp """
        k = 0
        dT = T_amb - self.getTemp() #T_amb-T_set
        if dT > 0: #heating
            k = 0.2/(15**2)
        elif dT < 0: #cooling
            k = 0.4/((-20)**2)
        P_temp = k*(dT)**2 #model derived from Ryan Kingsbury's thesis pg.79
        
        """ Normalized power consumption for bias consumption (per degree C) """
        #We calculate the number of mA needed to produce the same wavelength shift for 1 degree C
        P_curr_norm = 3.3 * ((0.088/0.0036)/1000) * dT #P=VI with V from ADN8810 datasheet


        #Calculate total change in wavelength of new set point
        dw = abs(self.delta_T)*dw_dt + abs(self.delta_C)*dw_dc

        #Optimize
        if P_temp > P_curr_norm: #Adjustments in mA will be more power efficient
            ntemp = T_amb - (P_curr_norm/k)**(0.5) #Calculate T_set at intersection (from eq k*T_amb-T_set)^2 = P_curr_norm)
            dw_remaining = (ntemp - (self.getTemp()-self.delta_T))*dw_dt #Calculate wavelength shift from new temp
            ncurr = self.getCurrent() + round(dw_remaining/dw_dc,2) #Finish remaining wavelength shift using bias current (since it is more efficient)
            #Update operating points
            self.setTemp(ntemp)
            self.setCurrent(ncurr)
