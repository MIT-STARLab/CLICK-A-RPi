import wiringpi
import zmq

wiringpi.wiringPiSPISetup(0, 12000000)
buf = bytes([121])
retlen, retdata = wiringpi.wiringPiSPIDataRW(0, buf)


