#!/usr/bin/env python
from __future__ import print_function
import sys
sys.path.append('/root/lib/')
sys.path.append('/root/fpga/')
import ipc_helper
import fpga_map as mmap
import time
import file_manager
import traceback
import options
import dac
import math
import hashlib

#For basic testing of calibration laser and seed laser aliveness. Can be upgraded as necessary (e.g. expected parameter values)

# define fpga interface
fpga = ipc_helper.FPGAClientInterface()
power = mmap.Power(fpga) # power sub-interface

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
def test_calib_laser(fo):
    #Note: cal laser circuit is on heater circuit current monitor
    print_test(fo, "Test Heater + Calibration Laser Actuation")
    avg_len = 10

    #Make sure heaters & cal laser are off
    power.calib_diode_off()
    power.heaters_off()
    power.heater_1_off()
    power.heater_2_off()
    time.sleep(.25)
    heater_off_curr = sum([fpga.read_reg(mmap.HEATER_CURRENT) for i in range(avg_len)])/avg_len
    fo.write('OFF: %f A\n' % heater_off_curr)
    
    power.heaters_on()
    time.sleep(.1)
    heater_only_curr = sum([fpga.read_reg(mmap.HEATER_CURRENT) for i in range(avg_len)])/avg_len
    fo.write('Heater Circuit: %f A\n' % heater_only_curr)

    power.calib_diode_on()
    fpga.write_reg(mmap.DAC_SETUP,1)
    fpga.write_reg(mmap.DAC_1_D, options.CAL_LASER_DAC_SETTING)
    time.sleep(.1)
    calib_curr = sum([fpga.read_reg(mmap.HEATER_CURRENT) for i in range(avg_len)])/avg_len
    fo.write('Heater Circuit + Cal Laser: %f A\n' % calib_curr)
        
    tosa_temp = fpga.read_reg(mmap.TOSA_TEMP)  
    power.calib_diode_off()
    power.heaters_off()

    success = True
    # Bound Checks (update as needed during ground testing)
    HEATER_OFF_UB = 0.2 #Reference temp is 26.5C, can be 110mA at 0C
    HEATER_ONLY_LB = 0.007 #Reference temp is 26.5C
    HEATER_ONLY_UB = 0.117 #Reference temp is 26.5C
    CALIB_LB = 0.022 #Reference temp is 26.5C
    CALB_UB = 0.134 #Reference temp is 26.5C
    add_temp = .007*fpga.read_reg(mmap.TOSA_TEMP) #Temperature variation TBR    
    if(heater_off_curr > HEATER_OFF_UB):
        success = False
        fo.write("Heater off current is larger than %f A\n" % (HEATER_OFF_LB))
        print(1)
    if(not (HEATER_ONLY_LB < heater_only_curr < HEATER_ONLY_UB+add_temp)):
        success = False
        fo.write("Heater Circuit current is outside of bounds: %f A to %f A: %f\n" % (HEATER_ONLY_LB, HEATER_ONLY_UB+add_temp, heater_only_curr))
        print(2)
    if(not (CALIB_LB < calib_curr < CALB_UB+add_temp)):
        success = False
        fo.write("Heater Circuit + Cal Laser current is outside of bounds: %f A to %f A: %f\n" % (CALIB_LB, CALB_UB+add_temp, calib_curr))
        print(3)

    if success:
        pass_test(fo)
    else:
        fail_test(fo)
    print("TOSA Temp: %0.03f, OFF: %.03f A, Heater Circuit Only: %.03f A, Heater Circuit + Cal Laser: %.03f A" % (tosa_temp, heater_off_curr, heater_only_curr, calib_curr))    

@error_to_file
def test_seed(fo):
    #Note: seed laser and FSM circuit is on LD bias circuit current monitor
    print_test(fo, "Test Seed Laser Actuation")
    avg_len = 10

    power.edfa_off()
    power.bias_off()

    time.sleep(.1)
    off_curr = sum([fpga.read_reg(mmap.LD_CURRENT) for i in range(avg_len)])/avg_len
    fo.write('OFF: %f A\n' % off_curr)

    power.edfa_on()
    power.bias_on()
    power.tec_on()
    time.sleep(1)
    standby_curr = sum([fpga.read_reg(mmap.LD_CURRENT) for i in range(avg_len)])/avg_len
    fo.write('Standby: %f A\n' % standby_curr)

    fpga.write_reg(mmap.EDFA_IN_STR ,'mode acc\r')
    time.sleep(0.1)
    fpga.write_reg(mmap.EDFA_IN_STR ,'ldc ba 2200\r')
    time.sleep(0.1)
    fpga.write_reg(mmap.EDFA_IN_STR ,'edfa on\r')
    time.sleep(2)
    on_curr = sum([fpga.read_reg(mmap.LD_CURRENT) for i in range(avg_len)])/avg_len
    fo.write('ON: %f A\n' % on_curr)

    tosa_temp = fpga.read_reg(mmap.TOSA_TEMP)  
    success = True
    # Bound Checks (update as needed during ground testing)
    for i in range(10):
        avg_on_curr = sum([fpga.read_reg(mmap.LD_CURRENT) for i in range(avg_len)])/avg_len
        time.sleep(.1)
        limit = .600 + .01*fpga.read_reg(mmap.TOSA_TEMP) #TBR Temperature Variation (Agrees w/ 26.5C data)
        if(avg_on_curr > limit or avg_on_curr < 100e-3):
            success = False
            fo.write("LD on current is outside of normal bounds (%s, %s)A: %s A\n" % (str(round(limit,3)), str(100e-3), str(round(avg_on_curr,3))))
            break

    power.edfa_off()
    power.bias_off()
    power.tec_off()

    if success:
        pass_test(fo)
    else:
        fail_test(fo)
        print("LD current is out of nominal bounds: %s" % round(avg_on_curr,3))
    print("TOSA Temp: %0.03f, OFF: %.03f A, Standby: %.03f A, ON: %.03f A" % (tosa_temp, off_curr, standby_curr, on_curr))    

def run_all(origin):

    t_str = time.strftime("%d.%b.%Y %H.%M.%S", time.gmtime())
    
    with file_manager.ManagedFileOpen('/root/log/laser_self_test_data/%s.gz' % t_str,'w') as (f, tags):
    
        tags['origin'] = origin
        f.write('Laser Self-test, %s\n' % t_str)
        print('   CLICK-A Laser Self-Test Script')
        try:
            def file_as_bytes(file):
                with file:
                    return file.read()

            hash_v =  hashlib.md5(file_as_bytes(open('/root/test/automated_laser_checks.py', 'rb'))).hexdigest()
            f.write('MD5: %s\n' % str(hash_v))
            print('MD5: %s' % str(hash_v))
        except:
            f.write('Hash failure, check script path\n')
            print('Hash failure, check script path')
        
        test_calib_laser(f)
        test_seed(f)


if __name__ == '__main__':
    run_all(origin='command line')