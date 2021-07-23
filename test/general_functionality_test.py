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
import hashlib
from tx_packet import seed_align


fpga = ipc_helper.FPGAClientInterface()
power = mmap.Power(fpga)
edfa = mmap.EDFA(fpga)
seed_setting = 1 #0 for flat_sat 1 for payload


len_pass_string = 100
def print_test(fo,name): 
    print(name + ' ' + '.'*(len_pass_string - len(name)) + ' ', end='')
    fo.write('\n--- Starting %s ---\n' % name)
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
            traceback.print_tb(exc_traceback,file=fo)
            fail_test(fo)
            traceback.print_tb(exc_traceback)
            print(repr(e))
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

    if time_single > 3.5: success = False #housekeeping causes delay, adding .5 second
    if time_block  > 3.5: success = False #housekeeping causes delay, adding .5 second
    if time_async  > 3: success = False

    if success:
        pass_test(fo)
    else: 
        fail_test(fo)
        print("FPGA read/writes Failed")
        
    print('1000 single reads: %f sec' % time_single)
    print('1000 4-block reads: %f sec' % time_block)
    print('1000 single async reads: %f sec' % time_async)

    return success
    
@error_to_file        
def test_BIST(fo):
    
    print_test(fo,'BIST Circuit Test')
    
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
    fail_text = []
    sample_num = 10
    success = True
    for temp in mmap.TEMPERATURE_BLOCK:
        samples = []
        for sample in range(sample_num):
            samples.append(fpga.read_reg(temp))

        if((max(samples) - min(samples)) == 0):
            success = False
            fail_text.append('Expected temperatures are not time varying.')
            break
        
        temps.append(sum(samples)/sample_num)

    if((max(temps) - min(temps)) > 15 or min(temps) < -20 or max(temps) > 60 ):
        success = False
        fail_text.append('Expected Max vs. Min < 5C and 0C<Temp<40C')
        fail_text.append('Max: %s, Min: %s' % (max(temps), min(temps)))
    
    for nm,tp in zip(temp_name_list,temps): fo.write('%s%f\n' % (nm,tp))
    
    if success:
        pass_test(fo)
    else:
        fo.write("Temperature Read out\n")
        fo.write(str(list(zip(temp_name_list,temps)))+'\n')
        fail_test(fo)
        for t in fail_text: print(t)

    return success

"""
Reprogram the FPGA and check to verify the all power switches are 0

"""
@error_to_file
def reflash_fpga(fo):
    import os
    print_test(fo, "Reflashing FPGA To Start Self Test Sequence")

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
        fo.write('Expected edfa and calibration laser registers to be zero after reprogramming \n')
        fo.write('Edfa power: ' + str(edfa_power) +  ' Calibration power: ' + str(calib_power) +"\n")
        fo.write('FPGA was not reprogrammed \n')
        fail_test(fo)
        print('FPGA was not reprogrammed')

    return success

"""
Test EDFA current before/after current switch
Read fline response to verify operation
"""
@error_to_file
def test_EDFA_IF(fo):

    print_test(fo, "Testing EDFA IF & Current")

    if(fpga.read_reg(mmap.PO2) != 85):  
        power.edfa_on()

    time.sleep(5)
    
    edfa_temp = fpga.read_reg(mmap.EDFA_CASE_TEMP)
    power_in = fpga.read_reg(mmap.EDFA_POWER_IN)
    power_out = fpga.read_reg(mmap.EDFA_POWER_OUT)
    time.sleep(.2)

    success = True
    if(edfa_temp < -20 and edfa_temp > 60):
        success = False
        fo.write('EDFA Case temp is outside range -20-60C: %s \n' % edfa_temp)

    if(power_in != -100.0):
        success = False
        fo.write("Seed power is nonzero and shouldn't be: %s \n" % power_in)

    if(power_out != -100.0):
        success = False
        fo.write("EDFA output power is nonzero and shouldn't be: %s \n" % power_out) 

    power.edfa_off()
    if success:
        pass_test(fo)
    else:
        fail_test(fo)
        print('EDFA Case temp: %s Power In: %s Power Out: %s' % (edfa_temp, power_in, power_out))

    return success 

@error_to_file 
def test_heaters(fo):

    print_test(fo, "Heater Actuation Test")

    #Make sure heaters are off
    power.heaters_off()
    power.heater_1_off()
    power.heater_2_off()
    time.sleep(.25)
    heater_off_curr = fpga.read_reg(mmap.HEATER_CURRENT)
    fo.write('OFF: %f A\n' % heater_off_curr)
    
    power.heaters_on()
    time.sleep(.1)
    
    power.heater_1_on()
    time.sleep(.25)
    heater_one_curr = fpga.read_reg(mmap.HEATER_CURRENT)
    fo.write('Heater 1: %f A\n' % heater_one_curr)
    power.heater_2_on()
    time.sleep(.1)
    heater_both_curr = fpga.read_reg(mmap.HEATER_CURRENT)
    fo.write('Both: %f A\n' % heater_both_curr)
    power.heater_1_off()
    time.sleep(.1)
    heater_two_curr = fpga.read_reg(mmap.HEATER_CURRENT)
    fo.write('Heater 2: %f A\n' % heater_two_curr)
    power.heater_2_off()
        
    power.heaters_off()


    success = True
    #can be 110mA while off at 0C
    if(heater_off_curr > 0.2):
        success = False
        fo.write("Heater off current is larger than 200mA\n")
        print(1)
    add_temp = .007*fpga.read_reg(mmap.TOSA_TEMP)
    if(not (0.7 < heater_one_curr < 0.9+add_temp)):
        success = False
        fo.write("Heater one current is outside of bounds: %f A to %f A: %f\n" % (.7, .9, heater_one_curr))
        print(2)

    if(not (0.4 < heater_two_curr < 0.6+add_temp)):
        success = False
        fo.write("Heater two current is outside of bounds: %f A to %f A: %f\n" % (.4, .6, heater_two_curr))
        print(3)
    if(not (1.1 <  heater_both_curr < 1.5+add_temp)):
        success = False
        print(4)
        fo.write("Both heater current is outside of bounds: %f A to %f A: %f\n" % (1.1, 1.5, heater_both_curr))

    if success:
        pass_test(fo)
    else:
        fail_test(fo)
    print("OFF: %.03f A, Heater 1: %.03f A, Heater 2: %.03f A, Both: %.03f A" % (heater_off_curr, heater_one_curr, heater_two_curr, heater_both_curr))

    return success

"""
Verify TEC driver linearity, readback matches commanded value and current does not exceed 100mA
"""    
@error_to_file 
def test_tec_driver(fo):

    print_test(fo, "TEC Driver Test")
    # power.edfa_off()

    if(fpga.read_reg(mmap.PO4) != 85):
        power.tec_on()

    time.sleep(.5)
    to_print = []

    success = True
    #standard room temperature values
    test_msb = [4,5,5,6,6]
    test_lsb = [128,0,128,0,128]
    for i in range(len(test_msb)):
        fpga.write_reg(mmap.LTSa, test_msb[i])
        fpga.write_reg(mmap.LTSb, test_lsb[i])
        
        #Wait for TEC to settle
        time.sleep(3)

        for y in range(10):
            avg_len = 10
            on_curr = sum([fpga.read_reg(mmap.TEC_CURRENT) for x in range(avg_len)])/avg_len

            limit = .300 + .007*fpga.read_reg(mmap.TOSA_TEMP)

            if(on_curr > limit):
                success = False
                fo.write("TEC on current is higher than %smA: %s\n" % (round(limit,3), on_curr))
                to_print.append("TEC on current is higher than %s mA: %s" % (round(limit,3), on_curr))
        
            fo.write("Reg 1 at %d, reg 2 at %d: readback %f\n" % (test_msb[i], test_lsb[i], on_curr))

        time.sleep(5)
        #check TEC linearity
        error = .05
        tec_readback = []
        if(fpga.read_reg(mmap.TOSA_TEMP) < 10):
            continue

        val = test_msb[i]*256 + test_lsb[i]
        tec_readback = fpga.read_reg(mmap.LTRa)*256 + fpga.read_reg(mmap.LTRb)
        if (abs(val -tec_readback) > val*error):
            success = False
            if(fpga.read_reg(mmap.TOSA_TEMP)< 10):
                fo.write("TEC Readback is more than 10" + '%'+" from TEC value: %s Readback: %s \n" %(val, tec_readback))
                to_print.append("TEC Readback is more than 10"+'%'+"from TEC value: %s Readback: %s" %(val, tec_readback))
            else:
                fo.write("TEC Readback is more than 5" + '%'+" from TEC value: %s Readback: %s \n" %(val, tec_readback))
                to_print.append("TEC Readback is more than 5"+'%'+"from TEC value: %s Readback: %s" %(val, tec_readback))

    fpga.write_reg(mmap.LTSa, 0)
    fpga.write_reg(mmap.LTSb, 0)
    
    if success:
        pass_test(fo)
    else:
        fail_test(fo)
    for t in to_print: print(t)

    return success


@error_to_file 
def test_bias_driver(fo):

    print_test(fo, "LD Bias Test")
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

    #fpga.write_reg(mmap.DAC_SETUP,2)
    #for i in range(506,510):
    #    fpga.write_reg(i,0)

    fpga.write_reg(mmap.LBCa, 14)
    fpga.write_reg(mmap.LBCb, 33)
    
    time.sleep(2)

    success = True
    for i in range(10):
        avg_len = 10
        avg_on_curr = sum([fpga.read_reg(mmap.LD_CURRENT) for i in range(avg_len)])/avg_len
        time.sleep(.1)

        limit = .600 + .01*fpga.read_reg(mmap.TOSA_TEMP)
        if(avg_on_curr > limit or avg_on_curr < 100e-3):
            success = False
            fo.write("LD on current is outside of normal bounds (%s, %s)A: %s A\n" % (str(round(limit,3)), str(100e-3), str(round(avg_on_curr,3))))
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

    print_test(fo, "PPM Power Test")

    ppm_codes = [4,8,16,32,64,128]

    if(seed_setting):
        ppm_input = [options.PPM4_THRESHOLDS[0], options.PPM4_THRESHOLDS[1]]
        seed_align([options.DEFAULT_TEC_MSB, options.DEFAULT_TEC_LSB, options.DEFAULT_LD_MSB, options.DEFAULT_LD_LSB])
    else:
        ppm_input = [options.PPM4_THRESHOLDS[2], options.PPM4_THRESHOLDS[3]]
        seed_align([options.DEFAULT_FTEC_MSB, options.DEFAULT_FTEC_LSB, options.DEFAULT_FLD_MSB, options.DEFAULT_FLD_LSB])


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
                fo.write("Bias Curr: %s TEC Curr: %s TEC_ReadBack: %s REG 1-4 %s, %s, %s, %s, PPM_ORDER: %s\n" % \
                (fpga.read_reg(mmap.LD_CURRENT), fpga.read_reg(mmap.TEC_CURRENT), fpga.read_reg(mmap.LTRa)*256 + fpga.read_reg(mmap.LTRb), \
                fpga.read_reg(1),fpga.read_reg(2), fpga.read_reg(3), fpga.read_reg(4), ppm_order))
        else:
            expected_input = baseline_input - 3.0
            if (abs(edfa_input-expected_input)) > .5:
                success = False
                fo.write("PPM%s was outside of the acceptable range of input power to the EDFA: %s \n" % (ppm, edfa_input))
                fo.write("Bias Curr: %s TEC Curr: %s TEC_ReadBack: %s REG 1-4 %s, %s, %s, %s\n" % \
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
        print("EDFA input not nominal across PPM orders. Powers: %s \n" % str(edfa_inputs))
        print("EDFA input not nominal across PPM orders.")
    for ppm_set, power_v in zip(ppm_codes,edfa_inputs): print('PPM%03d: %.02f dBm' % (ppm_set, power_v))
    for ppm_set, power_v in zip(ppm_codes,edfa_inputs): fo.write('PPM%03d: %f dBm\n' % (ppm_set, power_v))

    return success

"""
Seed setting 0 for flatsat and 1 for payload
"""
@error_to_file
def check_CW_power(fo):
    print_test(fo, "CW Power Test")

    fpga.write_reg(mmap.DATA,0)
    if(seed_setting):
        ppm_input = [options.CW_THRESHOLDS[0], options.CW_THRESHOLDS[1]]
        seed_align([options.DEFAULT_CW_TEC_MSB, options.DEFAULT_CW_TEC_LSB, options.DEFAULT_LD_MSB, options.DEFAULT_LD_LSB], True)

    else:
        ppm_input = [options.CW_THRESHOLDS[2], options.CW_THRESHOLDS[3]]
        seed_align([options.DEFAULT_CW_FTEC_MSB, options.DEFAULT_CW_FTEC_LSB, options.DEFAULT_FLD_MSB, options.DEFAULT_FLD_LSB], True)


    input_power = fpga.read_reg(mmap.EDFA_POWER_IN)
    success = True
    if (input_power < ppm_input[1] or input_power > ppm_input[0]):
        success = False

    if success:
        pass_test(fo)
    else:
        fo.write("EDFA Input Power outside of expected range: "+str(input_power)+" dbm \n")
        fo.write("Bias Curr: %s TEC Curr: %s TEC_ReadBack: %s REG 1-4 %s, %s, %s, %s\n" % \
        (fpga.read_reg(mmap.LD_CURRENT), fpga.read_reg(mmap.TEC_CURRENT), fpga.read_reg(mmap.LTRa)*256 + fpga.read_reg(mmap.LTRb), \
        fpga.read_reg(1),fpga.read_reg(2), fpga.read_reg(3), fpga.read_reg(4)))
        fail_test(fo)
        print("EDFA Input Power outside of expected range: "+str(input_power)+" dbm")

    power.edfa_off()
    power.bias_off()
    power.tec_off()

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
    
    print_test(fo,'Modulator Data FIFO Test')

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
        print("Fifo length %s does not match packet symbol length %s" % (fifo_len, len(tx_pkt.symbols)))
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
    if(len(tx_pkt1.symbols) != fifo_len): #Why is the empty fifo length 2
        success = False
        print("Fifo length %s does not match packet symbol length %s" % (fifo_len, len(tx_pkt1.symbols)))
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
        print("Fifo length %s does not match packet symbol length %s" % (fifo_len, len(tx_pkt.symbols)))
        fo.write("Fifo length %s does not match packet symbol legnth %s \n" % (fifo_len, tx_pkt.symbols)) 
        fo.write("Packet PPM: %s and Data: %s \n" % (tx_pkt.ppm_order, tx_pkt.data))   
    
    if(fifo_len < 100): time.sleep(.005)

    #Release FIFO
    fpga.write_reg(mmap.CTL, 0x7) 

    if success:
        pass_test(fo)
    else:
        fail_test(fo)

    return success


def run_all(origin):
    test_summary = "" #initialize return val
    t_str = time.strftime("%d.%b.%Y %H.%M.%S", time.gmtime())
    
    with file_manager.ManagedFileOpen('/root/log/self_test_data/%s.gz' % t_str,'w') as (f, tags):
    
        tags['origin'] = origin
        f.write('Self-test, %s\n' % t_str)
        print('   CLICK-A Self-Test Script')
        try:
            def file_as_bytes(file):
                with file:
                    return file.read()

            hash_v =  hashlib.md5(file_as_bytes(open('/root/test/general_functionality_test.py', 'rb'))).hexdigest()
            f.write('MD5: %s\n' % str(hash_v))
            print('MD5: %s' % str(hash_v))
        except:
            f.write('Hash failure, check script path\n')
            print('Hash failure, check script path')
        
        results = ["","","","","","","","","","","",""]
        results[0] = "PASS" if reflash_fpga(f) else "FAIL" #"Reflashing FPGA to start self test sequence"
        results[1] = "PASS" if test_basic_fpga_if(f) else "FAIL" #'Basic FPGA read/write'
        results[2] = "PASS" if test_fpga_if_performance(f) else "FAIL" #'Benchmark FPGA read/write'
        results[3] = "PASS" if test_mod_FIFO(f) else "FAIL" #'Modulator Data FIFO Test'
        results[4] = "PASS" if check_temperature_init(f) else "FAIL" #"Temperature Sensor Init Sequence Check"
        results[5] = "PASS" if test_BIST(f) else "FAIL" #BIST Circuit Test
        results[6] = "PASS" if test_EDFA_IF(f) else "FAIL" # "Testing EDFA IF & Current"
        results[7] = "PASS" if test_tec_driver(f) else "FAIL" #"TEC Driver Test"
        results[8] = "PASS" if test_bias_driver(f) else "FAIL" #"LD Bias Test"
        results[9] = "PASS" if test_scan_PPM(f) else "FAIL" #"PPM Power Test"
        results[10] = "PASS" if check_CW_power(f) else "FAIL" #"CW Power Test"
        results[11] = "PASS" if test_heaters(f) else "FAIL" #"Heater Actuation Test"
        test_summary = "Reflash FPGA: %s\nBasic FPGA R/W: %s\nBenchmark FPGA R/W: %s\nFIFO: %s\nTemp Sensor Init: %s\nBIST Circuit: %s\nEDFA IF & Current: %s\nTEC Driver: %s\nLD Bias: %s\nPPM Pwr: %s\nCW Pwr: %s\nHeater Actuation: %s" % tuple(results)
    
    return test_summary

if __name__ == '__main__':
    run_all(origin='command line')

        
        
        
        
        
        
