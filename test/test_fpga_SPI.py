#!/usr/bin/env python
import sys
sys.path.append('/root/fpga/')
import spi

if len(sys.argv) == 2 and sys.argv[1].isdigit():
    print(spi.read_register(int(sys.argv[1])))

elif len(sys.argv) == 3 and sys.argv[1].isdigit() and sys.argv[2].isdigit():
    print(spi.write_register(int(sys.argv[1]), int(sys.argv[2])))

elif len(sys.argv) > 1:
    reply = spi.edfa(' '.join(sys.argv[1:]))
    if len(reply) > 0:
        print(reply)
