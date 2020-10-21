from multiprocessing import Process
from multiprocessing import Pipe

from fpga import fpga

def Camera(camera2fpga, camera2bus):
    print("Hi, I'm the camera")

def Fpga(fpga2camera, fpga2bus):
    print("Hi, I'm the fpga")

def Bus(bus2camera, bus2fpga):
    print("Hi, I'm the bus")

if __name__ == '__main__':
    camera2fpga, fpga2camera = Pipe()
    camera2bus, bus2camera = Pipe()
    fpga2bus, bus2fpga = Pipe()

    camera = Process(target=Camera, args=(camera2fpga, camera2bus))
    fpga = Process(target=Fpga, args=(fpga2camera, fpga2bus))
    bus = Process(target=Bus, args=(bus2camera, bus2fpga))
    
    camera.start()
    fpga.start()
    bus.start()


