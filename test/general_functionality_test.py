#!/usr/bin/python
from __future__ import print_function
import sys
sys.path.append('/root/lib/')
import ipc_helper
import fpga_map as mmap
import time
import file_manager
import traceback



fpga = ipc_helper.FPGAClientInterface()

len_pass_string = 100
def print_test(fo,name): 
    print(name + ' ' + '.'*(len_pass_string - len(name)) + ' ', end='')
    fo.write('--- Starting %s ---\n' % name)
def pass_test(fo):
    print('Pass')
    fo.write('--- Pass ---\n')
def fail_test(fo):
    print('Fail')
    fo.write('--- Fail ---\n')

def error_to_file(func):
    def e_to_f(fo):
        try: return func(fo)
        except Exception as e:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            fo.write(repr(e)+'\n')
            print(repr(e))
            traceback.print_tb(exc_traceback,file=fo)
            traceback.print_tb(exc_traceback)
            fail_test(fo)
            return 0
    return e_to_f

@error_to_file
def test_basic_fpga_if(fo):
    
    print_test(fo,'Basic FPGA read/write')

    v1 = 0
    v2 = 100
    fo.write('Writing %s, with %s then %s\n' % (mmap.SCP,v1,v2))
    fpga.write_reg(mmap.SCP, v1)
    fpga.write_reg(mmap.SCP, v2)
    vo = fpga.read_reg(mmap.SCP)
    fo.write('Reading %s, got %s\n' % (mmap.SCP,vo))
    assert vo == v2
    
    v1 = [0,0,0,0]
    v2 = [10,20,30,40]
    fo.write('Writing %s, with %s then %s\n' % (mmap.SCP,v1,v2))
    fpga.write_reg(mmap.SCP, v1)
    fpga.write_reg(mmap.SCP, v2)
    vo = fpga.read_reg(mmap.SCP,len(v2))
    fo.write('Reading %s, got %s\n' % (mmap.SCP,vo))
    assert vo == v2
    
    pass_test(fo)
    return 1

@error_to_file        
def test_fpga_if_performance(fo):
    
    print_test(fo,'Benchmark FPGA read/write')
    
    success = True
    
    # single reads
    v1 = 100
    fo.write('Writing %s, with %s\n' % (mmap.SCP,v1))
    fpga.write_reg(mmap.SCP, v1)
    fo.write('Reading %s 1000 times\n' % (mmap.SCP))
    start_single = time.time()
    for i in xrange(1000):
        try:
            vo = fpga.read_reg(mmap.SCP)
        except:
            success = False
            fo.write('IO Error at itteration %d\n' % (i))
            continue
        if vo != v1:
            success = False
            fo.write('Value Error at itteration %d, got %d\n' % (i, vo))
    time_single = time.time() - start_single
    fo.write('Completed in %f sec\n' % (time_single))
    
    #block reads
    v1 = [1,2,3,4]
    fo.write('Writing %s, with %s\n' % (mmap.SCP,v1))
    fpga.write_reg(mmap.SCP, v1)
    fo.write('Reading %s 1000 times\n' % (mmap.SCP))
    start_block = time.time()
    for i in xrange(1000):
        try:
            vo = fpga.read_reg(mmap.SCP,len(v1))
        except:
            success = False
            fo.write('IO Error at itteration %d\n' % (i))
            continue
        if vo != v1:
            success = False
            fo.write('Value Error at itteration %d, got %d\n' % (i, vo))
    time_block = time.time() - start_block
    fo.write('Completed in %f sec\n' % (time_block))
    
    #async reads
    v1 = 10
    fo.write('Writing %s, with %s\n' % (mmap.SCP,v1))
    fpga.write_reg(mmap.SCP, v1)
    fo.write('Reading %s 1000 times\n' % (mmap.SCP))
    start_async = time.time()
    vol = []
    for i in xrange(1000):
        try:
            vol.append([fpga.read_reg_async(mmap.SCP),i])
        except:
            success = False
            fo.write('IO Error at itteration %d\n' % (i))
            continue
    for vocb,i in vol:
        try:
            vo = vocb.wait()
        except:
            success = False
            fo.write('IO Error at itteration %d\n' % (i))
            continue
        if vo != v1:
            success = False
            fo.write('Value Error at itteration %d, got %d\n' % (i, vo))
    time_async = time.time() - start_async
    fo.write('Completed in %f sec\n' % (time_async))

    if time_single > 3: success = False
    if time_block  > 3: success = False
    if time_async  > 3: success = False

    if success:
        pass_test(fo)
    else: 
        fail_test(fo)

    print('1000 single reads: %f sec' % time_single)
    print('1000 4-block reads: %f sec' % time_block)
    print('1000 single async reads: %f sec' % time_async)
    
    return success
    
@error_to_file        
def test_BIST(fo):
    
    print_test(fo,'BIST circuit')
    
    fo.write('Reset BIST threshold DAC, set reference on\n')
    fpga.dac.reset_bist()
    mask = mmap.DAC_BIST_A + mmap.DAC_BIST_B + mmap.DAC_BIST_C
    fo.write('Enable thresholdhold output\n')
    fpga.dac.enable_output(mask)
    
    fo.write('Set all thresholds to 0\n')
    fpga.write_reg(mmap.DAC_1_A,[0,0,0])
    
    fo.write('Reset and start bist capture\n')
    fpga.write_reg(mmap.SCN, 0)
    fpga.write_reg(mmap.SCN, mmap.SCN_RUN_CAPTURE)
    fo.write('Read BIST results\n')
    BIST_all_high = fpga.read_reg(mmap.SCTa, 8)
    
    fo.write('Set all thresholds to max (0xFFF)\n')
    fpga.write_reg(mmap.DAC_1_A,[0xFFFF,0xFFFF,0xFFFF])
    
    fpga.write_reg(mmap.SCN, 0)
    fpga.write_reg(mmap.SCN, mmap.SCN_RUN_CAPTURE)
    
    fo.write('Read BIST results\n')
    BIST_all_low = fpga.read_reg(mmap.SCTa, 8)
    
    expected_high = [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF]
    expected_low  =  [0xFF,0xFF,   0,   0,   0,   0,   0,   0]
    
    success = True 
    if BIST_all_high != expected_high:
        success = False
        fo.write('Expected %s,' % str(expected_high))
        fo.write('Got %s\n' % str(BIST_all_high))
    if BIST_all_low  != expected_low:
        success = False
        fo.write('Expected %s,' % str(expected_low))
        fo.write('Got %s\n' % str(BIST_all_high))
    if success:
        pass_test(fo)
        return 1
    else:
        fail_test(fo)
        
        if BIST_all_high != expected_high:
            print('Expected %s,' % str(expected_high))
            print('Got %s' % str(BIST_all_high))
        if BIST_all_low  != expected_low:
            print('Expected %s,' % str(expected_low))
            print('Got %s' % str(BIST_all_low))
        return 0
        
@error_to_file        
def test_EDFA_IF(fo):
    
    #Turn on EDFA power switch
    #Check current
    #Run fline
    #check values
    print_test(fo,'EDFA UART Interface')
    
    raise NotImplementedError
    
@error_to_file        
def test_mod_FIFO(fo):
    
    print_test(fo,'Modulator data FIFO')
    
    raise NotImplementedError

@error_to_file 
def test_camera(fo):

    print_test(fo, 'Camera connect test')

    raise NotImplementedError

@error_to_file 
def test_heaters(fo):

    print_test(fo, "Heater Accuation test")

    raise NotImplementedError

"""
This test checks the temperature sensors are within
an operating range of 20C-30C (room_temp)
"""
@error_to_file        
def test_roomtemp_temperatures(fo):

    print_test(fo,'Room Temp Temperature Sensor Test')

    temp_name_list = ["PD: ", "EDFA: ", "Camera: ", "TOSA: ","Lens: ", "Raceway: "]
    temps = []
    for temp in mmap.TEMPERATURE_BLOCK:
        temps.append(round(fpga.read_reg(temp),2))
        if (temp < 20 or temp > 30):
            success = False

    fo.write("Temperature Read out\n")
    fo.write(str(list(zip(temp_name_list,temps)))+'\n')

    if success:
        pass_test(fo)
    else: 
        fail_test(fo)
        print('Expected temps between 20 & 30C')
        print('Received temps: ' + str(list(zip(temp_name_list,temps))))

        
if __name__ == '__main__':
    
    t_str = time.strftime("%d.%b.%Y %H.%M.%S", time.gmtime())
    with file_manager.ManagedFileOpen('/root/data/test/general/%s.gz' % t_str,'w') as (f, tags):
    
        tags['origin'] = 'command line'
        
        # Flash the FPGA
        
        test_basic_fpga_if(f)
        # Test switches value, should be 0
        # Check Tx FIFO is empty
        # Check CRC ok
        test_fpga_if_performance(f)
        test_roomtemp_temperatures(f) #Update to check slow change, some noise, grouped toogether
        test_BIST(f)
        #test_mod_FIFO(f)
        
        # check EDFA current
        # Turn on EDFA switch
        # check EDFA current
        
        # test_EDFA_IF(f)
        
        # check TEC current
        # Turn TEC on
        # check TEC current
        
        # check TEC readback is linear and within ~20%.
        # set TEC to nominal
        
        # check Bias rail current
        # Turn Bias on
        # check Bias rail current
        
        # Set seed current to nominal, check rail increase
        
        # PPM scan, record EDFA input power, check range and -3dB decrease
        # CW, find with BIST threshold
        
        # Turn off EDFA, TEC, Bias
        # Check currents EDFA, TEC, Bias rail
        
        # Check heaters current
        # Turn heaters
        # Check heaters current (still 0?)
        # Heater 1 on, check current
        # Heater 2 on, check current
        # Heater 1 off, check current
        # Heater 2 off, check current. 
        
        
        
        
        
        