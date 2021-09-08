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
def pwr_edfa_off(fo):
    #Note: seed laser and FSM circuit is on LD bias circuit current monitor
    success = True
    avg_len = 10
    print_test(fo, "Power EDFA OFF")

    fpga.write_reg(mmap.EDFA_IN_STR ,'edfa off\r')
    time.sleep(4)
    off_curr = sum([fpga.read_reg(mmap.LD_CURRENT) for i in range(avg_len)])/avg_len
    fo.write('OFF Current: %f A\n' % on_curr)

    power.edfa_off() #write 15 to reg 34

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
        f.write('EDFA OFF Script, %s\n' % t_str)
        print('   CLICK-A EDFA OFF Script')
        try:
            def file_as_bytes(file):
                with file:
                    return file.read()

            hash_v =  hashlib.md5(file_as_bytes(open('/root/test/edfa_off.py', 'rb'))).hexdigest()
            f.write('MD5: %s\n' % str(hash_v))
            print('MD5: %s' % str(hash_v))
        except:
            f.write('Hash failure, check script path\n')
            print('Hash failure, check script path')
        
        results = "SUCCESS" if pwr_edfa_off(f) else "FAIL"
        test_summary = "EDFA OFF: %s" % results
        f.write(test_summary)
        print(test_summary)
    
    return test_summary


if __name__ == '__main__':
    run_all(origin='command line')