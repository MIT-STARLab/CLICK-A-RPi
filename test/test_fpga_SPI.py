#!/usr/bin/env python
import sys
sys.path.append('/root/fpga/')
import spi

if len(sys.argv) == 2:
    if sys.argv[1].isdigit():
        print(spi.read_register(int(sys.argv[1])))
    else:
        spi.edfa(sys.argv[1])
elif len(sys.argv) == 3:
    if sys.argv[1].isdigit() and sys.argv[2].isdigit():
        print(spi.write_register(int(sys.argv[1]), int(sys.argv[2])))
