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


def test_calib_laser(fo):

    print_test(fo, "Test Calibration laser Actuation")

    raise NotImplementedError

def test_seed(fo):

    print_test(fo, "Test Seed laser Actuation")

    raise NotImplementedError

def test_fsm(fo):

    print_test(fo, 'FSM test')

    raise NotImplementedError

def test_seed_PPM(fo):

    print_test(fo, 'Test PPM orders')

    raise NotImplementedError

def test_bist_pulse_shape(fo):

    print_test(fo, 'Reconstruction pulse shape')

    raise NotImplementedError
