import sys
sys.path.append('/root/lib/')
import options
import fpga_map as mmap
import time

def reset_fifo(fpgabus):
    read_string(fpgabus)
    '''
    # Reset the EDFA UART FIFO
    ctl = fpgabus.read_reg(mmap.CTL)
    ctl |= mmap.CTL_ERX_FIFO
    fpgabus.write_reg(mmap.CTL,ctl)
    ctl &= ~mmap.CTL_ERX_FIFO & 0xFF
    fpgabus.write_reg(mmap.CTL,ctl)
    '''

def read_string(fpgabus):
    read_max = 1200
    empty = fpgabus.read_reg(mmap.EFL) & mmap.EFL_EMPTY
    chars = []
    error = 0
    
    while (not empty) and read_max:
        errors, values = fpgabus.transfer((mmap.ERF,mmap.EFL), (0,0), (0,0))
        chars.append(chr(values[0]))
        empty = values[1] & mmap.EFL_EMPTY
        error += sum(errors)
        read_max -= 1
        
    return error,''.join(chars)
    
def write_string(fpgabus, tx_str):
    
    start = time.time()
    
    for char in tx_str:
        while not (fpgabus.read_reg(mmap.FLG) & mmap.FLG_TX_READY):
            if time.time() > start + options.EDFA_TIMOUT: return True
        fpgabus.write_reg(mmap.ETX, ord(char))
    return False
    
def fline(fpgabus):
    tx_str = 'fline\r'
    reset_fifo(fpgabus)
    write_string(fpgabus,tx_str)
    time.sleep(options.EDFA_READ_WRITE_DELAY) 
    reply = read_string(fpgabus)[1]
    reply = reply.split('\n')
    if len(reply) > 1: 
        reply = reply[1]
        return reply
    else:
        return '0 '*len(mmap.EDFA_PARSED_BLOCK)
        
def parse(regs,flist):
    if   flist[0] == 'EL': regs[mmap.EDFA_EN_PIN] = 0
    elif flist[0] == 'HL': regs[mmap.EDFA_EN_PIN] = 1
    else:                  regs[mmap.EDFA_EN_PIN] = 2
    
    if   flist[1] ==  'ACC': regs[mmap.EDFA_MODE] = mmap.EDFA_MODE_ACC
    elif flist[1] == 'AOPC': regs[mmap.EDFA_MODE] = mmap.EDFA_MODE_AOPC
    elif flist[1] ==  'APC': regs[mmap.EDFA_MODE] = mmap.EDFA_MODE_APC
    else:                    regs[mmap.EDFA_MODE] = 0
    
    if   flist[2] == 'LDOFF': regs[mmap.EDFA_DIODE_ON] = 0
    elif flist[2] ==  'LDON': regs[mmap.EDFA_DIODE_ON] = 1
    else:                     regs[mmap.EDFA_DIODE_ON] = 2
    
    try:    regs[mmap.EDFA_MYSTERY_TEMP] = float(flist[3])
    except: regs[mmap.EDFA_MYSTERY_TEMP] = -1000.0
    
    if flist[3] == 'IPA': regs[ mmap.EDFA_POWER_IN] = -100.0
    else:
        try:              regs[ mmap.EDFA_POWER_IN] = float(flist[4])
        except:           regs[ mmap.EDFA_POWER_IN] = -1000.0
        
    if flist[5] == 'IPA': regs[mmap.EDFA_POWER_OUT] = -100.0
    else:
        try:              regs[mmap.EDFA_POWER_OUT] = float(flist[5])
        except:           regs[mmap.EDFA_POWER_OUT] = -1000.0
        
    try:    regs[mmap.EDFA_PRE_CURRENT] = int(flist[6])
    except: regs[mmap.EDFA_PRE_CURRENT] = 0xFFFFFFFF
    
    try:    regs[mmap.EDFA_PRE_POWER] = int(flist[7])
    except: regs[mmap.EDFA_PRE_POWER] = 0xFFFFFFFF
    
    if flist[8] == 'LOW': regs[mmap.EDFA_PUMP_CURRENT] = 0
    else:
        try:    regs[mmap.EDFA_PUMP_CURRENT] = int(flist[8])
        except: regs[mmap.EDFA_PUMP_CURRENT] = 0xFFFFFFFF
    
    if flist[9] == 'CTA': regs[mmap.EDFA_CASE_TEMP] = 60.0
    else:
        try:    regs[mmap.EDFA_CASE_TEMP] = float(flist[9])
        except: regs[mmap.EDFA_CASE_TEMP] = -1000.0
    
    return regs
