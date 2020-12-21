#!/usr/bin/env python
import os
import time

delay = [16, 17, 18, 19, 20, 21, 22]
tec_settings = [180,185,190,195,200,205,210,215,220]

for d in delay:
    os.system("./fpga.py '--write 15 '"+str(d))
    for t in tec_settings:
        print("Delay: ", d, "TEC: ", t)
        os.system("./fpga.py '--write 2 '"+str(t))
        os.system("./fpga.py '--send ../poem_zone.txt 4'")
        time.sleep(.1)
