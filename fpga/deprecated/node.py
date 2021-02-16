#!/usr/bin/env python
# NODE FPGA class
import fl
import time
import struct
import math

class NodeFPGA(object):

    def __init__(self, handle, debug=False):
        self.DEBUG = debug
        self.handle = handle

        # Chip-specific defaults
        self.FIFO_ASYNC_WR = 0x01
        self.FIFO_SPI_WR   = 0x03
        self.FIFO_GPIO_WR  = 0x04
        self.FIFO_ASYNC_RD = 0x01

        # DAC commands
        self.DAC_CMD_WRITE_AND_UPDATE = 0x30
        self.DAC_CMD_SEL_INTERNAL_REF = 0x60

        self.FPGA_init()

    def FPGA_init(self):
        # SPI CS default to high
        fl.flWriteChannel(self.handle,self.FIFO_GPIO_WR,bytearray([0xFF]))
        time.sleep(0.1)

        self.initDAC()

        time.sleep(0.1)
        
    def SPI_write(self,byte_array):
        # assert chipselect
        fl.flWriteChannel(self.handle,self.FIFO_GPIO_WR,bytearray([0x00]))

        # Write to SPI master
        #TODO:type "bytes" is alias for type "str" in Python2.7, not sure if this line will work
        #fl.flWriteChannel(self.handle,self.FIFO_SPI_WR,bytes)
        fl.flWriteChannel(self.handle,self.FIFO_SPI_WR,bytearray(byte_array))
        # deassert chipselect
        fl.flWriteChannel(self.handle,self.FIFO_GPIO_WR,bytearray([0xFF]))

    def initDAC(self):
        spi_bytes = struct.pack(">BH",self.DAC_CMD_SEL_INTERNAL_REF,0)
        self.SPI_write(spi_bytes)

    def writeDAC(self,value):
        dacval = (value<<4)
        dacval &= 0xFFFF
        CMD_WRITE_AND_UPDATE = 0x30
        spi_bytes = struct.pack(">BH",self.DAC_CMD_WRITE_AND_UPDATE,dacval)
        self.SPI_write(spi_bytes)

    def resetSLERcounts(self):
        # Reset cycle and error counters
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,0x00)

    def readSLERcounts(self):
        # Latch cycle and error counters
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,0x01)

        # Request readback of cycle counter and error counter
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,0x03) # cycle counter
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,0x04) # error counter
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,0x05) # ones counter

        # Readback counters (two counters, each 8 bytes)
        resp = fl.flReadChannel(self.handle,self.FIFO_ASYNC_RD,24)
        if self.DEBUG: print "Length of response:",len(resp)
        if self.DEBUG: print [hex(a) for a in resp]
        (cycles,errors,ones) = struct.unpack(">QQQ",resp)
        ser = float(errors)/float(cycles)

        if self.DEBUG: print "0x%016X 0x%016X 0x%016X SER = %e"%(cycles,errors,ones,ser)

        return cycles,errors,ones,ser

    def measureSER(self,obslength=1.0):
        # Wait for DAC to settle
        time.sleep(0.1)

        # Reset cycle and error counters
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,0x00)

        # Wait for the observation time
        time.sleep(obslength)

        # TODO: combine commands below together
        # Latch cycle and error counters
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,0x01)

        # Request readback of cycle counter and error counter
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,0x03) # cycle counter
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,0x04) # error counter
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,0x05) # ones counter

        # Readback counters (two counters, each 8 bytes)
        resp = fl.flReadChannel(self.handle,self.FIFO_ASYNC_RD,24)
        if self.DEBUG: print "Length of response:",len(resp)
        if self.DEBUG: print [hex(a) for a in resp]
        (cycles,errors,ones) = struct.unpack(">QQQ",resp)
        ser = float(errors)/float(cycles)

        if self.DEBUG: print "0x%016X 0x%016X 0x%016X SER = %e"%(cycles,errors,ones,ser)

        return cycles,errors,ones,ser

    def binSearchPeak(self,M,obslength=1.0,target=None,debug=True):
        """ Perform binary search to find DAC value where occurance rate of mark slots (i.e. 1's)
        is closest to 1/M.
        """

        if target==None:
            targetP1 = 0.9/M
        else:
            targetP1 = target

        DAC_VALS = range(0,2**12)

        first = 0
        last = len(DAC_VALS)-1

        i = 0
        while first<=last:
            midpoint = (first+last)//2

            # Set DAC to next value
            self.writeDAC(DAC_VALS[midpoint])

            # Wait for DAC output to settle
            time.sleep(0.01)

            # Count 1's
            cycles,errors,ones,ser = self.measureSER(obslength)

            P1 = float(ones)/float(cycles)

            if debug: print "  %2i %4i"%(i,midpoint),

            if P1 < targetP1:
                last = midpoint-1
                if debug: print "<",
            else:
                first = midpoint+1
                if debug: print ">",

            if debug: print P1

            i += 1

        return DAC_VALS[midpoint]

    def setTXdelay(self,delay):
        if delay>255 or delay<0:
            raise "Delay invalid, must be in range 0-255"
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,bytearray([0x80,0x10,delay]))

    def setPPM_M(self,M):
        #Depreciated command! Only use if using old FPGA image!
        self.M = M

        binval = struct.pack(">H",M)
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,bytearray([0x80,0x00,binval[0]]))
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,bytearray([0x80,0x01,binval[1]]))

        # Configure the PRBS bits-per-symbol, regardless of whether PRBS is active
        self.setBitsPerSymbol()

    def setBitsPerSymbol(self,bps=None):
        """ Sets the number of PRBS bits per symbol. If not specified, calculate from M."""
        if bps==None:
            # Not specified, so calculate from M
            try:
                # Calculate number of bits per symbol, based on setting for M
                # ***floor*** is used to guarantee that there is an active slot in each symbol,
                # for non-power of 2 settings for M, this is desirable as it prevents empty slots
                bps = int(math.floor(math.log(self.M,2)))
            except NameError:
                # If we get here, it means that M has not yet been configured
                raise "Configure M before enabling PRBS"
        if bps>16 or bps<2:
            raise "Bits per symbol invalue, must be in range 2-16"
        fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,bytearray([0x80,0x12,bps]))

        
    def usePRBS(self,turn_on_prbs=True):

        if turn_on_prbs:
            # First, we need to set the bits-per-symbol, this will be calculated from existing M
            self.setBitsPerSymbol()

            # Switch data source to PRBS
            fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,bytearray([0x80,0x11,0x01]))

        else:
            # Disable PRBS, causes modulator to send all "zero" symbols
            fl.flWriteChannel(self.handle,self.FIFO_ASYNC_WR,bytearray([0x80,0x11,0x00]))

    def setModulatorParams(self,ppm_order):
	# inputs: ppm order for modulation
	# outputs: delay between writes, number of bytes per packet written to FPGA, hex
	# value for tracking byte (commands modulator to enter tracking mode)
	#
        # sets delays and num_bytes and trackingbyte for specified ppm order
        # tried to set delay and num_bytes to maximize length of delay and number of delays
        # (FSM needs to update often and the longer the delay the better chance we have
        # making SPI successful)
        if ppm_order == 4:
            delay = 1e-5
            num_bytes = 60963
            trackingbyte = 0x83
        elif ppm_order == 8:
            delay = 1e-5
            num_bytes = 48292
            trackingbyte = 0x87
        elif ppm_order == 16:
            delay = 1e-5
            num_bytes = 38292
            trackingbyte = 0x8F
        elif ppm_order == 32:
            delay = 1e-4
            num_bytes = 9952 
            trackingbyte = 0x9F
        elif ppm_order == 64:
            delay = 2e-4
            num_bytes = 8832 
            trackingbyte = 0xBF
        elif ppm_order == 128:
            delay = 1e-3
            num_bytes = 8553
            trackingbyte = 0xFF
        else:
            raise ValueError('PPM order not supported. Please try again')

        return (delay,num_bytes,trackingbyte)

    def setTrackingMode(self,writechannel,trackingbyte,ppm_order):
	#inputs: channel to write data to, trackingbyte (byte that commands 
	#modulator to enter tracking mode), and ppm order
	#outputs: terminal string
	print("Putting modulator into tracking mode for ppm order %d." % ppm_order)
	fl.flWriteChannel(self.handle,writechannel,bytearray([trackingbyte]))

    def loadDataFile(self,dataFile,num_bytes):
	#inputs: name of file to load, number of bytes in each packet written to FPGA
	#outputs: list of byte arrays containing data from file
	
	# format data in list of bytearrays (where each bytearray contains a packet of data) for
	# writing to FPGA
        with open(dataFile,'rb') as f:
            data = f.read()
        data = [ord(byte) for byte in data]
        num_packets = int(math.ceil(len(data)/num_bytes))
        data_packets = [bytearray(data[i*num_bytes:(i+1)*num_bytes]) for i in range(num_packets)]  
	return data_packets

    def writeFile(self,writechannel,resetchannel,statuschannel,data_packets,delay,vp):
        # inputs: channel to write data to, channel to reset (writes to this
	# channel reset modulator), channel to read back status flags, data_packets as a list of
	# bytearrays, delay between writing packets,FPGA identifier (vp)
	# outputs: writes to FPGA and writes to terminal

	# reset channel is 0x08, read channel is 0x05

        # initiate list for saving write times
        wholetime = []
        print("Writing to FPGAlink device {}...".format(vp))
        start_reset = time.time() # start timer for reset
        fl.flWriteChannel(self.handle,resetchannel,0x01) # reset flags in FPGA
        time.sleep(0.02)
        flag = fl.flReadChannel(self.handle, statuschannel) # read out flag in FPGA
        print flag  # Print out value for flag - value of 0 shows that flags are cleared
        end_reset = time.time() # determine time to perform reset
                # write data to virtual channel in FPGA
        for packet in data_packets:
            start = time.time()
            fl.flWriteChannel(self.handle, writechannel, packet)
            end = time.time()
            time.sleep(delay)
            wholetime.append((end-start)) # add write time to list for assessing time to write each packet
        start = time.time()   # initialize measuring time to read channel
        flag = fl.flReadChannel(self.handle, statuschannel) # read flag for modulator status
        end = time.time()   # determine time to read channel
                # print information about time to read/write
        print("read time is: %f" % (end-start))
        maxtime = max(wholetime)
        mintime = min(wholetime)
        avgtime = sum(wholetime)/len(wholetime)
        print("Write time is %f" % avgtime)
        print("max time is %f" % maxtime)
        print("min time is %f" % mintime)
        print("Reset time is %f" % (end_reset-start_reset))
                # Print status of FPGA/modulator to terminal and reset flags
        if flag == 0x1E:
            fl.flWriteChannel(self.handle, resetchannel, 0x01)
            time.sleep(0.01)
            raise ValueError('data error and empty')
        elif flag == 0xFE:
            fl.flWriteChannel(self.handle, resetchannel, 0x01)
            time.sleep(0.01)
            print('data error and full')
            #raise ValueError('data error and full')
        elif flag == 0xDE:
            fl.flWriteChannel(self.handle, resetchannel, 0x01)
            time.sleep(0.01)
            print('data error')
            #raise ValueError('data error')
        elif flag == 0xFF:
            fl.flWriteChannel(self.handle, resetchannel, 0x01)
            time.sleep(0.01)
            raise ValueError('full and empty asserted')
        elif flag == 0xCF:
            fl.flWriteChannel(self.handle, resetchannel, 0x01)
            time.sleep(0.01)
            print('full asserted')
        elif flag == 0x11:
            fl.flWriteChannel(self.handle, resetchannel, 0x01)
            time.sleep(0.01)
            raise ValueError('empty asserted')
        elif flag == 0xAF:
            fl.flWriteChannel(self.handle, resetchannel, 0x01)
            time.sleep(0.01)
            print('prog_full asserted')
        elif flag == 0xBF:
            fl.flWriteChannel(self.handle, resetchannel, 0x01)
            time.sleep(0.01)
            print('all prog_fulls asserted')
        else:
            print('Good fifo')
        print('file sent')
        
    def writeFileNTimes(self,writechannel,resetchannel,statuschannel,data_packets,delay,vp,N):
	count = 0
	while count < N:
	    self.writeFile(writechannel,resetchannel,statuschannel,data_packets,delay,vp)
	    count += 1

