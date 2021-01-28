import sys
sys.path.append('/root/lib/')
import options
import fpga_map as mmap

DAC_WRITE            = 0b000 << 19
DAC_UPDATE           = 0b001 << 19
DAC_WRITE_UPDATE_ALL = 0b010 << 19
DAC_WRITE_UPDATE_ONE = 0b011 << 19
DAC_POWER            = 0b100 << 19
DAC_RESET            = 0b101 << 19
DAC_LDAC_SETTING     = 0b110 << 19
DAC_REF              = 0b111 << 19

DAC_A   = 0b000 << 16
DAC_B   = 0b001 << 16
DAC_C   = 0b010 << 16
DAC_D   = 0b011 << 16
DAC_ALL = 0b111 << 16

DAC_OEN   = 0b00 << 4
DAC_1K    = 0b01 << 4
DAC_100K  = 0b10 << 4
DAC_HIGHZ = 0b11 << 4

"""
Initialize DAC to default settings
is_por: 1
ref_enable: 1
transparent_mode: 0
"""
def init(fpgabus, target_dac):
    reset(fpgabus, target_dac,1)
    set_reference(fpgabus,target_dac,1)
    set_output_mode(fpgabus, 0)

"""
Write to one of the dac's 
Target 1: Photodiode Board DAC
Target 2: FSM DAC
Command: 24 bit number for 3 8 bit commands to send to DAC
"""
def write_command(fpgabus, target_dac, command):
    if   target_dac == 1: addr = (mmap.THRa, mmap.THRb, mmap.THRc)
    elif target_dac == 2: addr = (mmap.FSMa, mmap.FSMb, mmap.FSMc)
    else: return
    data = ( (command & 0x3F0000) >> 16,
             (command & 0x00FF00) >> 8,
             (command & 0x0000FF) )
    rw_flag = [1]*3
    fpgabus.transfer(addr, rw_flag, data)

"""
Resets registers to 0: 
is_por 1: DAC, Input, LDAC, Power-down, internal reference setup
is_por 0: DAC register & Input Shift register only
"""
def reset(fpgabus, target_dac, is_por):
    if is_por: write_command(fpgabus, target_dac, DAC_RESET | 1)
    else:      write_command(fpgabus, target_dac, DAC_RESET | 0)

"""
ref_enable: 1 reference on
ref_enable: 0 reference off
"""
def set_reference(fpgabus, target_dac, ref_enabled):
    if ref_enabled: write_command(fpgabus, target_dac, DAC_REF | 1)
    else:           write_command(fpgabus, target_dac, DAC_REF | 0)


"""
Mask: 4 bit code word 1 bit for each channel DB3:DB0
1: Transparent mode, inputs become outputs on falling edge of 24th clock cycle
0: Inputs do not affect outputs until an update command is sent.
"""
def set_output_mode(fpgabus, mask):
    mode = (mask & 0x300) >> 4
    if mask & 0x00FF:
        target_dac = 1
        mask_low = mask & 0x00FF
        write_command(fpgabus, target_dac, DAC_POWER | mode | mask_low)
    if mask & 0xFF00:
        target_dac = 2
        mask_high = (mask & 0xFF00) >> 8
        write_command(fpgabus, target_dac, DAC_POWER | mode | mask_high)


"""
Write and update a single channel
target_chan: 0-7 ; 0-3 Photodiode DAC 4-7 FSM DAC 
"""
def write_and_update(fpgabus, target_chan, value):
    if options.CHECK_ASSERTS:
        assert target_chan  < 8
        assert target_chan >= 0
    if target_chan < 4: target_dac = 1
    else:               target_dac = 2
    addr = (target_chan%4 & 0b11) << 16
    write_command(fpgabus, target_dac, DAC_WRITE_UPDATE_ONE | addr | value)
    