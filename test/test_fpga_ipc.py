#!/usr/bin/env python
import sys
sys.path.append('/root/lib/')
import ipc_helper
import fpga_map as mmap
import time

fpga = ipc_helper.FPGAClientInterface()

if len(sys.argv) == 2 and sys.argv[1].isdigit():
    print(fpga.read_reg(int(sys.argv[1])))

elif len(sys.argv) == 3 and sys.argv[1].isdigit() and sys.argv[2].isdigit():
    print(fpga.write_reg(int(sys.argv[1]), int(sys.argv[2])))

elif len(sys.argv) > 1:
    fpga.write_reg(mmap.EDFA_IN_STR ,' '.join(sys.argv[1:])+'\r')
    time.sleep(0.1)
    reply = fpga.read_reg(mmap.EDFA_OUT_STR)
    reply = reply.split('\n')
    if len(reply) > 1:
        reply = reply[1]
        if len(reply) > 0:
            print(reply)