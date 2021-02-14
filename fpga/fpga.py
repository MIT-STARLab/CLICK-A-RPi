#!/usr/bin/env python
import sys
sys.path.append('/root/lib/')
import zmq
import struct
import options
import ipc_helper
import fpga_map as mmap
import fpga_bus
import edfa
import dac
import time

def loop():

    #ZMQ i/f. Keep timout low to update ZMQ internal logic before clients time-out.
    ipc_server = ipc_helper.FPGAServerInterface(timeout=100)

    #FPGA i/f
    if options.IPC_USES_SPI: fpgabus = fpga_bus.SPIBus()
    else: fpgabus = fpga_bus.USBBus()
    
    # Check if the FPGA is alive
    if options.CHECK_ASSERTS: assert fpgabus.verify_boot()
    
    # Buffer for virtual regs
    reg_buffer = {}
    last_edfa_update = 0

    while 1:
    
        try:
            req = ipc_server.get_request()
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN: continue
            
        # EDFA in and out string have a special packet, containing a string for a single address
        if req.start_addr == mmap.EDFA_IN_STR:
        
            edfa.reset_fifo(fpgabus)
            
            if options.CHECK_ASSERTS: assert req.size > 3
            
            len_str = struct.unpack('I', req.write_data[0:4])[0] 
            
            if options.CHECK_ASSERTS: assert req.size >= (len_str+4)
            
            values_in = req.write_data[4:4+len_str]
            error_flag = edfa.write_string(fpgabus, values_in)
            req.answer('',error_flag)
            continue
            
        elif req.start_addr == mmap.EDFA_OUT_STR:
            
            error,edfa_out_str = edfa.read_string(fpgabus)
            req.answer(edfa_out_str,error)
            continue
            
        # Decoding the incoming packet
        register_number = req.size//4
        addresses = range(req.start_addr, req.start_addr+register_number)
        if req.rw_flag and register_number:
            values_in = []
            for addr,idx in zip(addresses,xrange(0,req.size,4)):
                frmt = mmap.REGISTER_TYPE[addr]
                pl = req.write_data[idx:idx+4]
                decode = struct.unpack(frmt,pl)
                if len(decode): values_in.append(decode[0])
                else: values_in.append(0)
        else:
            values_in = [0]*register_number
        
        if req.start_addr < 128:
            # Raw registers
            write_flag = [req.rw_flag]*register_number 
            errors,values_out = fpgabus.transfer(addresses, write_flag, values_in)
            
            error_flag = sum(errors)
            req.answer(values_out,error_flag)
            
        else:
            # Virtual registers
            
            # ----------------- EDFA -----------------
            # if any in the EDFA block, get FLINE
            if any([x in mmap.EDFA_PARSED_BLOCK for x in addresses]):
                if time.time() - last_edfa_update > options.EDFA_VIRTUAL_REGS_GOOD_FOR:
                    fline = edfa.fline(fpgabus)
                    flist = fline.split()
                    reg_buffer = edfa.parse(reg_buffer, fline)

            # -----------------------------------------
            
            values_out = []
            for addr,value in zip(addresses,values_in):
            
                if addr in mmap.TEMPERATURE_BLOCK:
                    msb = fpgabus.read_reg(mmap.BYTE_1[addr])
                    lsb = fpgabus.read_reg(mmap.BYTE_2[addr])
                    temp = mmap.decode_temperature(msb,lsb)
                    values_out.append(temp)
                
                #converts adc value to current in amps
                elif addr in mmap.CURRENT_BLOCK:
                    msb = fpgabus.read_reg(mmap.BYTE_3[addr])
                    lsb = fpgabus.read_reg(mmap.BYTE_4[addr])
                    temp = mmap.decode_current(msb,lsb)
                    values_out.append(temp)
                    
                # ----------------- EDFA -----------------
                elif addr in mmap.EDFA_PARSED_BLOCK:
                    values_out.append(reg_buffer[addr])
                    
                    
                # ----------------- DACs ----------------- 
                elif addr == mmap.DAC_SETUP:
                    dac.init(fpgabus, value)

                elif addr == mmap.DAC_ENABLE:
                    dac.set_output_mode(fpgabus, value)
                    
                elif addr == mmap.DAC_RESET:
                    is_por = (value & 0b0100) >> 2
                    ref_en = (value & 0b1000) >> 3
                    if value & 0b01:
                        dac.reset(fpgabus, 1, is_por)
                        dac.set_reference(fpgabus, 1, is_por)
                    if value & 0b10: dac.reset(fpgabus, 2, ref_en)
               
                elif addr in mmap.DAC_BLOCK:
                    if req.rw_flag:
                        target_chan = addr - mmap.DAC_BLOCK[0]
                        dac.write_and_update(fpgabus,target_chan,value)
                    values_out.append(value)
                # ----------Telemetry---------------------- 

                elif addr == mmap.FPGA_TELEM:
                    fline = edfa.fline(fpgabus)
                    flist = fline.split()
                    reg_buffer = edfa.parse(reg_buffer, flist)
                    time.sleep(.5)
                    addresses = options.FPGA_TELEM_REGS
                    values_out = []
                    for addr in addresses:
                        if addr not in mmap.EDFA_PARSED_BLOCK:
                            values_out.append(fpgabus.transfer([addr], [req.rw_flag], [0])[1][0])
                        else:
                            values_out.append(reg_buffer[addr])
                    # print(values_out)   

                else: req.answer('',error_flag=1)

                
                
            req.answer(values_out)
        
loop()