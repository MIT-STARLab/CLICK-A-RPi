#!/usr/bin/env python
from __future__ import print_function
import sys
sys.path.append('/root/lib/')
import ipc_helper
import fpga_map as mmap
import time
import file_manager
import traceback
import tx_packet
import options
sys.path.append('/root/fpga/')
import dac
import math


fpga = ipc_helper.FPGAClientInterface()
power = mmap.Power(fpga)
edfa = mmap.EDFA(fpga)
seed_setting = 1 #0 for flat_sat 1 for payload


len_pass_string = 100
def print_test(fo,name): 
    print(name + ' ' + '.'*(len_pass_string - len(name)) + ' \n', end='')
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

    fo.write('1000 single reads: %f sec' % time_single)
    fo.write('1000 4-block reads: %f sec' % time_block)
    fo.write('1000 single async reads: %f sec' % time_async)
    if success:
        pass_test(fo)
    else: 
        fail_test(fo)
        print("FPGA read/writes Failed")

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
    
    fo.write('Set all thresholds to max (0xFFFF)\n')
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

    return success

"""
Check temperatures during initialization
Verifies temps within range of 0-40C and that all temperatures are within 5C of each other
Checks to make sure there is some amoutn of noise on the signal 

"""
@error_to_file
def check_temperature_init(fo):

    print_test(fo, "Temperature Sensor Init Sequence Check")
    temp_name_list = ["PD: ", "EDFA: ", "Camera: ", "TOSA: ","Lens: ", "Raceway: "]
    temps = []
    sample_num = 10
    success = True
    for temp in mmap.TEMPERATURE_BLOCK:
        samples = []
        for sample in range(sample_num):
            samples.append(fpga.read_reg(temp))

        if((max(samples) - min(samples)) == 0):
            success = False
            print('Expected temperatures are not time varying.')
            break
        
        temps.append(sum(samples)/sample_num)

    if((max(temps) - min(temps)) > 5 or min(temps) < 0 or max(temps) > 40 ):
        success = False
        print('Expected Max vs. Min < 5C and 0C<Temp<40C')
        print('Max: %s, Min: %s' % (max(temps), min(temps)))
    
    if success:
        pass_test(fo)
    else:
        fail_test(fo)
        fo.write("Temperature Read out\n")
        fo.write(str(list(zip(temp_name_list,temps)))+'\n')
        # print('Received temps: ' + str(list(zip(temp_name_list,temps))))

    return success


"""
Reprogram the FPGA and check to verify the all power switches are 0

"""
@error_to_file
def reflash_fpga(fo):
    import os
    print_test(fo, "Reflashing FPGA to start self test sequence")

    power.edfa_off()
    power.calib_diode_off()

    os.system("systemctl start --user flash-fpga.service")
    time.sleep(5)

    power_switches = [mmap.CAL, mmap.PO1, mmap.PO2, mmap.PO3, mmap.PO4, mmap.HE1, mmap.HE2]

    success = True
    for sw in power_switches:
        if fpga.read_reg(sw) != 0:
            success = False

    if success:
        pass_test(fo)
    else:
        fail_test(fo)
        fo.write('Expected edfa and calibration laser registers to be zero after reprogramming \n')
        fo.write('Edfa power: ' + str(edfa_power) +  ' Calibration power: ' + str(calib_power) +"\n")
        fo.write('FPGA was not reprogrammed \n')
        print('FPGA was not reprogrammed')

    return success
"""
Test EDFA current before/after current switch
Read fline response to verify operation
"""
@error_to_file
def test_EDFA_IF(fo):

    print_test(fo, "Testing EDFA IF & current")

    if(fpga.read_reg(mmap.PO2) == 85):
        power.edfa_off()
        time.sleep(.5)
    off_current = fpga.read_reg(mmap.EDFA_CURRENT)
    
    power.edfa_on()
    time.sleep(5)
    
    edfa_temp = fpga.read_reg(mmap.EDFA_CASE_TEMP)
    power_in = fpga.read_reg(mmap.EDFA_POWER_IN)
    power_out = fpga.read_reg(mmap.EDFA_POWER_OUT)
    on_current = sum([fpga.read_reg(mmap.EDFA_CURRENT) for x in range(10)])/10
    time.sleep(.2)
    power.edfa_off()

    success = True
    if(off_current > 10e-3 or on_current < 10e-3 or on_current > .2):
        success = False
        fo.write('EDFA current is more than 10mA when off: %s or less than 100mA or greater than 600mA when on: %s \n' % (off_current, on_current))

    if(edfa_temp < 0 and edfa_temp >40):
        success = False
        fo.write('EDFA Case temp is outside range 0-40C: %s \n' % edfa_temp)


    if(power_in != -100.0):
        success = False
        fo.write("Seed power is nonzero and shouldn't be: %s \n" % power_in)

    if(power_out != -100.0):
        success = False
        fo.write("EDFA output power is nonzero and shouldn't be: %s \n" % power_out) 

    if success:
        pass_test(fo)
    else:
        fail_test(fo)
        print('EDFA Case temp: %s Off Current: %s On Current: %s Power In: %s Power Out: %s \n' % (edfa_temp, off_current, on_current, power_in, power_out))

    return success 

@error_to_file 
def test_heaters(fo):

    print_test(fo, "Heater Accuation test")

    #Make sure heaters are off
    power.heaters_off()
    power.heater_1_off()
    power.heater_2_off
    time.sleep(.1)
    heater_off_curr = fpga.read_reg(mmap.HEATER_CURRENT)
    
    power.heaters_on()
    time.sleep(.1)
    power.heater_1_on()
    time.sleep(.1)
    heater_one_curr = fpga.read_reg(mmap.HEATER_CURRENT)
    power.heater_1_off()
    power.heater_2_on()
    time.sleep(.1)
    heater_two_curr = fpga.read_reg(mmap.HEATER_CURRENT)

    power.heaters_off()
    power.heater_2_off()

    success = True
    if(heater_off_curr > 10e-2):
        success = False
        fo.write("Heater off current is larger than 10mA: " + str(heater_off_curr) + "\n")
    
    if(heater_one_curr > 100):
        success = False
        fo.write("Heater one current is larger than normal: " + str(heater_one_curr) +"\n")

    if(heater_two_curr > 100):
        success = False
        fo.write("Heater two current is larger than normal: " + str(heater_two_curr) +"\n")

    if success:
        pass_test(fo)
    else:
        fail_test(fo)
        print("Heater off: "+str(heater_off_curr) + " Heater 1: " + str(heater_one_curr) + " Heater 2: " + str(heater_two_curr))


"""
Verify TEC driver linearity, readback matches commanded value and current does not exceed 100mA
"""    
@error_to_file 
def test_tec_driver(fo):

    print_test(fo, "TEC driver test")
    power.edfa_off()

    if(fpga.read_reg(mmap.PO4) != 85):
        power.tec_on()

    time.sleep(.5)

    success = True
    #standard room temperature values
    test_msb = [4,5,6]
    test_lsb = [0,100,200]
    for i in range(3):
        fpga.write_reg(mmap.LTSa, test_msb[i])
        fpga.write_reg(mmap.LTSb, test_lsb[i])
        
        #Wait for TEC to settle
        time.sleep(1.5)

        for y in range(10):
            avg_len = 10
            on_curr = sum([fpga.read_reg(mmap.TEC_CURRENT) for x in range(avg_len)])/avg_len

        if(on_curr > 250e-3):
            success = False
            fo.write("TEC on current is higher than 200mA: %s" % on_curr )
            print("TEC on current is higher than 200mA: %s" % on_curr )

        #check TEC linearity
        tec_readback = []
        val = test_msb[i]*256 + test_lsb[i]
        tec_readback = fpga.read_reg(mmap.LTRa)*256 + fpga.read_reg(mmap.LTRb)
        error = .05
        if (abs(val -tec_readback) > val*error):
            success = False
            fo.write("TEC Readback is more than 5" + '%'+" from TEC value: %s Readback: %s \n" %(val, tec_readback))
            print("TEC Readback is more than 5"+'%'+"from TEC value: %s Readback: %s \n" %(val, tec_readback))

    power.tec_off()
    fpga.write_reg(mmap.LTSa, 0)
    fpga.write_reg(mmap.LTSb, 0)
    
    if success:
        pass_test(fo)
    else:
        fail_test(fo)

    return success


@error_to_file 
def test_bias_driver(fo):

    print_test(fo, "LD Bias test")
    power.edfa_off()
    power.bias_off()
    
    if(fpga.read_reg(mmap.PO1) == 85):
        power.bias_off()
        fpga.write_reg(mmap.LBCa, 0)
        fpga.write_reg(mmap.LBCb, 0)

    time.sleep(.1)
    off_curr = fpga.read_reg(mmap.LD_CURRENT)

    power.bias_on()
    time.sleep(.2)

    fpga.write_reg(mmap.DAC_SETUP,2)
    for i in range(506,510):
        fpga.write_reg(i,0)

    fpga.write_reg(mmap.LBCa, 14)
    fpga.write_reg(mmap.LBCb, 33)
    
    time.sleep(2)

    success = True
    for i in range(10):
        avg_len = 10
        avg_on_curr = sum([fpga.read_reg(mmap.LD_CURRENT) for i in range(avg_len)])/avg_len
        time.sleep(1)

    
        if(off_curr > 100e-2):
            success = False
            fo.write("LD off current is greater than %sA: %s" % (str(100e-2), str(off_curr)))
            break

        if(avg_on_curr > 500e-3 or avg_on_curr < 100e-3):
            success = False
            fo.write("LD on current is outside of normal bounds (%s, %s)A: %s A" % (str(500e-3), str(100e-3), str(round(avg_on_curr,3))))
            break

    power.bias_off()

    if success:
        pass_test(fo)
    else:
        fail_test(fo)
        print("LD current is out of nominal bounds: %s" % round(avg_on_curr,3))

    return success

"""
Seed setting 0 for flatsat and 1 for payload
"""
@error_to_file
def test_scan_PPM(fo):

    print_test(fo, "PPM Power test")

    power.edfa_on()
    power.bias_on()
    power.tec_on()
    time.sleep(2)

    #set points are dependent on temperature
    payload_seed = [options.DEFAULT_TEC_MSB, options.DEFAULT_TEC_LSB, options.DEFAULT_LD_MSB, options.DEFAULT_LD_LSB]
    flat_sat_seed = [options.DEFAULT_FTEC_MSB, options.DEFAULT_FTEC_LSB, options.DEFAULT_FLD_MSB, options.DEFAULT_FLD_LSB]
    # seed = payload_seed
    ppm_codes = [4,8,16,32,64,128]
    ppm4_input = options.PPM4_THRESHOLDS

    if(seed_setting): 
        seed = payload_seed 
        ppm_input = [ppm4_input[0], ppm4_input[1]]
    else: 
        seed = flat_sat_seed
        ppm_input = [ppm4_input[2], ppm4_input[3]]

    for x in range(1,5):
        fpga.write_reg(x, seed[x-1])
        time.sleep(.1)

    time.sleep(3)
    success = True
    baseline_input = 0
    edfa_inputs =[]        
    for ppm in ppm_codes:
        
        ppm_order = (128 + (255 >>(8-int(math.log(ppm)/math.log(2)))))
        fpga.write_reg(mmap.DATA, ppm_order) 
        time.sleep(.5)

        avg_len = 3
        edfa_input = sum([fpga.read_reg(mmap.EDFA_POWER_IN) for i in range(avg_len)])/avg_len
        edfa_inputs.append(edfa_input)
        if(ppm == 4):
            if(edfa_input > ppm_input[1] and edfa < ppm_input[0]):
                baseline_input = edfa_input
            else:
                success = False
                fo.write("PPM%s input power to the edfa is outside of nominal range: %s \n" % (ppm, edfa_input))
                fo.write("Bias Curr: %s TEC Curr: %s TEC_ReadBack: %s REG 1-4 %s, %s, %s, %s, PPM_ORDER: %s" % \
                (fpga.read_reg(mmap.LD_CURRENT), fpga.read_reg(mmap.TEC_CURRENT), fpga.read_reg(mmap.LTRa)*256 + fpga.read_reg(mmap.LTRb), \
                fpga.read_reg(1),fpga.read_reg(2), fpga.read_reg(3), fpga.read_reg(4), ppm_order))
        else:
            expected_input = baseline_input - 3.0
            if (abs(edfa_input-expected_input)) > .5:
                success = False
                fo.write("PPM%s was outside of the acceptable range of input power to the EDFA: %s \n" % (ppm, edfa_input))
                fo.write("Bias Curr: %s TEC Curr: %s TEC_ReadBack: %s REG 1-4 %s, %s, %s, %s" % \
                (fpga.read_reg(mmap.LD_CURRENT), fpga.read_reg(mmap.TEC_CURRENT), fpga.read_reg(mmap.LTRa)*256 + fpga.read_reg(mmap.LTRb), \
                fpga.read_reg(1),fpga.read_reg(2), fpga.read_reg(3), fpga.read_reg(4)))
            baseline_input = edfa_input

    power.edfa_off()
    power.bias_off()
    power.tec_off()


    if success:
        pass_test(fo)
    else:
        fail_test(fo)
        fo.write("EDFA input not nominal across PPM orders. Powers: %s \n" % str(edfa_inputs))
        print("EDFA input not nominal across PPM orders. Powers: ", edfa_inputs)

    return success

"""
Seed setting 0 for flatsat and 1 for payload
"""
@error_to_file
def check_CW_power(fo):
    print_test(fo, "CW Power test")

    power.edfa_on()
    power.bias_on()
    power.tec_on()
    time.sleep(2)

    payload_seed = [options.DEFAULT_CW_TEC_MSB, options.DEFAULT_CW_TEC_LSB, options.DEFAULT_LD_MSB, options.DEFAULT_LD_LSB]
    flat_sat_seed = [options.DEFAULT_CW_FTEC_MSB, options.DEFAULT_CW_FTEC_LSB, options.DEFAULT_FLD_MSB, options.DEFAULT_FLD_LSB]
    # seed = payload_seed
    ppm4_input = options.CW_THRESHOLDS

    if(seed_setting): 
        seed = payload_seed 
        ppm_input = [ppm4_input[0], ppm4_input[1]]
    else: 
        seed = flat_sat_seed
        ppm_input = [ppm4_input[2], ppm4_input[3]]

    for x in range(1,5):
        fpga.write_reg(x, seed[x-1])

    fpga.write_reg(mmap.DATA, 0)
    #wait for TEC to tune
    time.sleep(5)

    input_power = fpga.read_reg(mmap.EDFA_POWER_IN)
    success = True
    if (input_power < ppm_input[1] or input_power > ppm_input[0]):
        success = False

    power.edfa_off()
    power.bias_off()
    power.tec_off()

    if success:
        pass_test(fo)
    else:
        fail_test(fo)
        fo.write("EDFA Input Power outside of expected range: "+str(input_power)+" dbm \n")
        fo.write("Bias Curr: %s TEC Curr: %s TEC_ReadBack: %s REG 1-4 %s, %s, %s, %s" % \
        (fpga.read_reg(mmap.LD_CURRENT), fpga.read_reg(mmap.TEC_CURRENT), fpga.read_reg(mmap.LTRa)*256 + fpga.read_reg(mmap.LTRb), \
        fpga.read_reg(1),fpga.read_reg(2), fpga.read_reg(3), fpga.read_reg(4)))
        print("EDFA Input Power outside of expected range: "+str(input_power)+" dbm \n")

    return success

    #SCN Registers are quite noisy we need to take a look at this.
    # success = True
    # fpga.write_reg(mmap.DAC_SETUP, 1)
    # fpga.write_reg(mmap.DAC_1_A, 45500) #Post EDFA
    # fpga.write_reg(mmap.DAC_1_B, 45500) #Pre EDFA
    # fpga.write_reg(mmap.DAC_1_C, 45500) #Seed
    # fo.write("Post EDFA Pre EDFA SEED DAC EDFA POWER IN \n")
    # for i in range(65):
    #     for y in range(5):
    #         fpga.write_reg(mmap.SCN, 0)
    #         fpga.write_reg(mmap.SCN, mmap.SCN_RUN_CAPTURE)
    #         if not(fpga.read_reg(mmap.SCF) & mmap.SCF_CAPTURE_DONE):
    #             time.sleep(.5)
    #             print("Waiting for FPGA to capture")
    #         seed = fpga.read_reg(mmap.SCAa)*256+fpga.read_reg(mmap.SCAb)
    #         seed1 = fpga.read_reg(mmap.SCBa)*256+fpga.read_reg(mmap.SCBb)
    #         seed2 = fpga.read_reg(mmap.SCCa)*256+fpga.read_reg(mmap.SCCb)
    #         fo.write(str(seed) + " "+ str(seed1) +" "+ str(seed2) + " " +str(65500-1000*i) + " "+ str(fpga.read_reg(mmap.EDFA_POWER_IN))+ '\n')
    #         print(seed, seed1, seed2, 65500-1000*i, fpga.read_reg(mmap.EDFA_POWER_IN))

    #     fpga.write_reg(mmap.DAC_1_A, 65500-1000*i)
    #     fpga.write_reg(mmap.DAC_1_B, 65500-1000*i)
    #     fpga.write_reg(mmap.DAC_1_C, 65500-1000*i)

    
@error_to_file        
def test_mod_FIFO(fo):
    
    print_test(fo,'Modulator data FIFO')

    #SHORT FIFO
    #with open("transmit.txt") as file:
    data = "Hi I'm Mr. Meseeks!"

    ppm_order = 4
    tx_pkt = tx_packet.txPacket(ppm_order, data)
    tx_pkt.pack()

    control = fpga.read_reg(mmap.CTL)
    if(control & 0x8): fpga.write_reg(mmap.DATA, 0x7) #Turn stall off
    tx_pkt.set_PPM(fpga)
    
    #Stall Fifo
    fpga.write_reg(mmap.CTL, control | 0x8)

    #Write to FIFO
    tx_pkt.transmit(fpga)

    fifo_len = fpga.read_reg(47)*256+fpga.read_reg(48)
    success = True
    if(len(tx_pkt.symbols) != fifo_len):
        success = False
        print("Fifo length %s does not match packet symbol length %s \n" % (fifo_len, len(tx_pkt.symbols)))
        fo.write("Fifo length %s does not match packet symbol legnth %s \n" % (fifo_len, tx_pkt.symbols)) 
        fo.write("Packet PPM: %s and Data: %s \n" % (tx_pkt.ppm_order, tx_pkt.data))   
    
    if(fifo_len < 100): time.sleep(.005)
    
    # #Release FIFO
    fpga.write_reg(mmap.CTL, 0x7) 

    #EMPTY FIFO
    ppm_order = 16
    data = ""
    tx_pkt1 = tx_packet.txPacket(ppm_order, data)
    tx_pkt1.pack()

    control = fpga.read_reg(mmap.CTL)
    if(control & 0x8): fpga.write_reg(mmap.DATA, 0x7) #Turn stall off
    tx_pkt1.set_PPM(fpga)
    
    #Stall Fifo
    fpga.write_reg(mmap.CTL, control | 0x8)

    #Write to FIFO
    tx_pkt1.transmit(fpga)

    fifo_len = fpga.read_reg(47)*256+fpga.read_reg(48)
    if(len(tx_pkt1.symbols)+2 != fifo_len): #Why is the empty fifo length 2
        success = False
        print("Fifo length %s does not match packet symbol length %s " % (fifo_len, len(tx_pkt1.symbols)))
        fo.write("Fifo length %s does not match packet symbol legnth %s \n" % (fifo_len, tx_pkt1.symbols)) 
        fo.write("Packet PPM: %s and Data: %s \n" % (tx_pkt1.ppm_order, tx_pkt1.data))   
    
    if(fifo_len < 100): time.sleep(.005)

    #Release FIFO
    fpga.write_reg(mmap.CTL, 0x7) 

    #LONG FIFO
    data = '1'*1000
    ppm_order = 128
    tx_pkt = tx_packet.txPacket(ppm_order, data)
    tx_pkt.pack()

    control = fpga.read_reg(mmap.CTL)

    if(control & 0x8): fpga.write_reg(mmap.DATA, 0x7) #Turn stall off
    tx_pkt.set_PPM(fpga)
    
    #Stall Fifo
    fpga.write_reg(mmap.CTL, control | 0x8)

    #Write to FIFO
    tx_pkt.transmit(fpga)

    fifo_len = fpga.read_reg(47)*256+fpga.read_reg(48)
    if(len(tx_pkt.symbols) != fifo_len):
        success = False
        print("Fifo length %s does not match packet symbol length %s " % (fifo_len, len(tx_pkt.symbols)))
        fo.write("Fifo length %s does not match packet symbol legnth %s " % (fifo_len, tx_pkt.symbols)) 
        fo.write("Packet PPM: %s and Data: %s " % (tx_pkt.ppm_order, tx_pkt.data))   
    
    if(fifo_len < 100): time.sleep(.005)

    #Release FIFO
    fpga.write_reg(mmap.CTL, 0x7) 

    if success:
        pass_test(fo)
    else:
        fail_test(fo)

    return success


def seed_align(default_settings):

    tec_msb, tec_lsb, ld_msb, ld_lsb = default_settings
    total_tec = tec_msb*256 + tec_lsb

    power.edfa_on()
    power.tec_on()
    power.bias_on()

    time.sleep(2)

    for i in range(1,5):
        fpga.write_reg(i, default_settings[i-1])
    
    fpga.write_reg(mmap.DATA, 131)
    
    time.sleep(.1)
    power_inputs = []
    window = 15
    for i in range(total_tec-window, total_tec+window):
        tec_msb = i//256
        tec_lsb = i%256
        fpga.write_reg(mmap.LTSa, tec_msb)
        fpga.write_reg(mmap.LTSb, tec_lsb)
        time.sleep(.1)
        avg_input_power = sum([fpga.read_reg(mmap.EDFA_POWER_IN) for x in range(5)])/5
        power_inputs.append(avg_input_power)

    new_tec = total_tec+power_inputs.index(max(power_inputs))-window
    tec_msb = new_tec//256
    tec_lsb = new_tec%256
    fpga.write_reg(mmap.LTSa, tec_msb)
    fpga.write_reg(mmap.LTSb, tec_lsb)

    # print(new_tec, power_inputs, power_inputs[power_inputs.index(max(power_inputs))], fpga.read_reg(mmap.TOSA_TEMP))

    power.edfa_off()
    power.tec_off()
    power.bias_on()



if __name__ == '__main__':
    
    t_str = time.strftime("%d.%b.%Y %H.%M.%S", time.gmtime())
    with file_manager.ManagedFileOpen('/root/log/self_test_data/%s.gz' % t_str,'w') as (f, tags):
    
        tags['origin'] = 'command line'

        reflash_fpga(f)
        test_basic_fpga_if(f)
        test_fpga_if_performance(f)
        test_mod_FIFO(f)
        check_temperature_init(f)
        test_BIST(f)
        test_EDFA_IF(f)
        test_tec_driver(f)
        test_bias_driver(f)
        seed_align([options.DEFAULT_TEC_MSB, options.DEFAULT_TEC_LSB, options.DEFAULT_LD_MSB, options.DEFAULT_LD_LSB])
        test_scan_PPM(f)
        check_CW_power(f)
        test_heaters(f)
        
        
        
        
        
        
