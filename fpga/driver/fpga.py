#!/usr/bin/env bash
NAME=${1?Error: no name given}

sudo python fpgadriver.py -i 04b4:8613 -v 1d50:602b:0002 $NAME
