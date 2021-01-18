import sys
import itertools
sys.path.append('/root/lib/')
import options

class FPGABusBase:
    def __init__(self): pass
    
    def read_reg(self, addr):
        return self.transfer( (addr,), (0,), (0,) )[1][0]
        
    def write_reg(self, addr, value):
        return self.transfer( (addr,), (1,), (value,) )[0][0]

class SPIBus(FPGABusBase):
    def __init__(self):
        FPGABusBase.__init__(self)
        import spidev
        
        self.spi = spidev.SpiDev()
           
    def transfer(self, addr_in, is_write, values):
    
        # Prepare data
        iter_input = zip(addr_in, is_write, values)
        def enc(addr, w_f, val): return (addr & 0x7F) | ((w_f & 1) << 7), (val & 0xFF)
        data_in = list(itertools.chain.from_iterable([enc(addr, w_f, val) for addr, w_f, val in iter_input]))
        
        # Run transfer
        self.spi.open(1, 0)
        data_out = self.spi.xfer(data_in+[0,0], options.SPI_FREQ)
        self.spi.close()
        
        # Check return addresses
        errors = [ain != aout for ain, aout in zip(data_in[0::2], data_out[2::2])]
        
        values_out = data_out[3::2]
        return errors,values_out
        
    def verify_boot(self):
        addr = 63
        trf = self.transfer( (addr,), (0,), (0,) )
        value = trf[1][0]
        addr_error = trf[0][0]
        dcm_locked = value & 1
        crc_err = (value >> 4) & 1
        if addr_error == 0 and dcm_locked == 1 and crc_err == 0: return True
        else: return False
        
class USBBus(FPGABusBase):
    def __init__(self):
        FPGABusBase.__init__(self)
        import fl  

        fl.flInitialise(0)
        self.handle = fl.flOpen(options.USB_DEVICE_ID)
        fl.flSelectConduit(self.handle, 1)
        
    def transfer(self, addr_in, is_write, values):
        
        # Prepare data
        iter_input = zip(addr_in, is_write, values)
        
        # Run transfer
        values_out = []
        for addr, w_f, val in iter_input:
            if w_f:
                fl.flWriteChannel(self.handle, addr, byte[num])
                values_out.append(None)
            else:
                values_out.append(fl.flReadChannel(self.handle, addr))
        
        #no error check for USB
        errors = [0]*len(addr_in)
        
        return errors,values_out
        
    def verify_boot(self):
        # LOL nope
        return True