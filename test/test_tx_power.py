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

# define fpga interface
fpga = ipc_helper.FPGAClientInterface()
power = mmap.Power(fpga) # power sub-interface
edfa = mmap.EDFA(fpga)
seed_setting = 1 #0 for flat_sat 1 for payload

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
def test_tx(fo):
    success = True
    avg_len = 10
    avg = 3
    print_test(fo, "Power EDFA ON and Align Seed Laser")

    seed_align([options.DEFAULT_TEC_MSB, options.DEFAULT_TEC_LSB, options.DEFAULT_LD_MSB, options.DEFAULT_LD_LSB])
    time.sleep(10)
    avg_input_power = sum([fpga.read_reg(mmap.EDFA_POWER_IN) for x in range(avg)])/avg
    fo.write('EDFA Power In: %f dBm\n' % avg_input_power)

    fpga.write_reg(mmap.EDFA_IN_STR ,'mode acc\r')
    time.sleep(0.1)
    fpga.write_reg(mmap.EDFA_IN_STR ,'ldc ba 2200\r')
    time.sleep(0.1)
    fpga.write_reg(mmap.EDFA_IN_STR ,'edfa on\r')
    time.sleep(4)
    on_curr = sum([fpga.read_reg(mmap.LD_CURRENT) for i in range(avg_len)])/avg_len
    fo.write('EDFA ON Current: %f A\n' % on_curr)

    time.sleep(120)
    power.edfa_off()
    power.bias_off()
    power.tec_off()

    if success:
        pass_test(fo)
    else:
        fail_test(fo)

    return success   

def run_all(origin):
    test_summary = "" #initialize return value

    t_str = time.strftime("%d.%b.%Y %H.%M.%S", time.gmtime())
    
    with file_manager.ManagedFileOpen('/root/log/%s.gz' % t_str,'w') as (f, tags):
    
        tags['origin'] = origin
        f.write('Tx Power Test Script, %s\n' % t_str)
        print('   CLICK-A Tx Power Test Script')
        try:
            def file_as_bytes(file):
                with file:
                    return file.read()

            hash_v =  hashlib.md5(file_as_bytes(open('/root/test/test_tx_power.py', 'rb'))).hexdigest()
            f.write('MD5: %s\n' % str(hash_v))
            print('MD5: %s' % str(hash_v))
        except:
            f.write('Hash failure, check script path\n')
            print('Hash failure, check script path')
        
        results = "PASS" if test_tx(f) else "FAIL"
        test_summary = "Tx Power Test: %s" % results
        f.write(test_summary)
        print(test_summary)
    
    return test_summary


if __name__ == '__main__':
    run_all(origin='command line')