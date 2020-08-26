import pigpio
from time import sleep


def init():
	reset()
	sleep(0.001)
	enable_int_ref()
	sleep(0.001)
	enable_all_dacs()
	sleep(0.001)
	enable_sw_ldac()
	sleep(0.001)
	return

def reset():
	pi = pigpio.pi()
	h = pi.spi_open(0, 12000000, 1)
	buf = [0x28, 0, 0x01]
        print('sending buffer '+str(buf))
	pi.spi_write(h, buf)
	pi.spi_close(h)
	return

def enable_int_ref():
	pi = pigpio.pi()
	h = pi.spi_open(0, 12000000, 1)
	buf = [0x38, 0, 0x01]
        print('sending buffer '+str(buf))
	pi.spi_write(h, buf)
	pi.spi_close(h)
	return

def enable_all_dacs():
	pi = pigpio.pi()
	h = pi.spi_open(0, 12000000, 1)
	buf = [0x20, 0, 0x0F]
        print('sending buffer '+str(buf))
	pi.spi_write(h, buf)
	pi.spi_close(h)
	return

def enable_sw_ldac():
	pi = pigpio.pi()
	h = pi.spi_open(0, 12000000, 1)
	buf = [0x30, 0, 0]
        print('sending buffer '+str(buf))
	pi.spi_write(h, buf)
	pi.spi_close(h)
	return

def update_dac(addr):
	pi = pigpio.pi()
	h = pi.spi_open(0, 12000000, 1)
	buf = [8+(addr&0x7), 0, 0]
        print('sending buffer '+str(buf))
	pi.spi_write(h, buf)
	pi.spi_close(h)
	return	

def write_dac(addr, value):
	pi = pigpio.pi()
	h = pi.spi_open(0, 12000000, 1)
	buf = [addr&0x7, (value&0xFF00)>>8, value&0xFF]
        print('sending buffer '+str(buf))
	pi.spi_write(h, buf)
	pi.spi_close(h)
	return

def send_command(cmd, addr, value):
	pi = pigpio.pi()
	h = pi.spi_open(0, 12000000, 1)
        buf = [((cmd&0x7)<<3)+(addr&0x7), (value&0xFF00)>>8, value&0xFF]
        print('sending buffer '+str(buf))
	print("done!")
	pi.spi_write(h, buf)
	pi.spi_close(h)
        return

