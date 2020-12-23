/*

This is a alias map for fpga map requests. 
Please use all human readable names in the code and no individual register values.

All blocks are specified as (starting reg address, length of block)

Human readable value block starts at reg 200;

*/
import ctypes


I_VID_PID = ctypes.c_char_p("04b4:8613") //"vendor ID and product ID
V_VID_PID = ctypes.c_char_p("1d50:602b:0002")
CONFIG_IMAGE_PATH = ctypes.c_char_p("J:A7A0A3A1:/root/bin/fpga.xsvf")
//Aliases for raw data

TEMPERATURE_BLOCK = (98,12)
CURRENT_BLOCK = (110,8)
LD_TEMP_CONTROL = (1,2)
LD_BIAS_CONTROL = (3,2)


//Human Readable Blocks
TEMPERATURE_BLOCK_C = (200,12)
EDFA_FLINE = 201