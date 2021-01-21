#!/usr/bin/env python
import os
import sys
import itertools
import heapq
import time
sys.path.append('/root/fpga/')
sys.path.append('/root/lib/')
import ipc_helper
import options
import fpga_map as mmap
import file_manager

DUTY_PULSE = 0
DUTY_HALF  = 1

SCAN_MONOTONIC  = 0
SCAN_DIVIDE     = 1
SCAN_ADAPTATIVE = 2

class Scope:
    def __init__(self,handler):
        self.handler = handler

    def set_frequency(self,slots=1,dutty=DUTY_PULSE):
        
        if dutty == DUTY_PULSE:
        
            if options.CHECK_ASSERTS:
                assert slots > 0
                assert slots < 17
            
            self.high_time = 1
            self.low_time  = slots*4 - 3
            
        elif dutty == DUTY_HALF:
        
            if options.CHECK_ASSERTS:
                assert slots > 0
                assert slots < 33
            
            self.high_time = slots*2 - 1
            self.low_time  = slots*2 - 1

    def set_timebase(self,phase=0):
    
        if options.CHECK_ASSERTS:
            assert self.counter_phase >= 0
    
        self.vco_phase = phase & 0b111
        self.counter_phase = (phase & 0b11111000) >> 3
        
        start_addr = mmap.SPP
        data = [
            self.vco_phase.
            self.counter_phase,
            self.low_time,
            self.high_time]
        
        self.handler.write_reg(start_addr, data)
        
        self.handler.write_reg(mmap.SCN, mmap.SCN_PROG_PLL)
        self.handler.write_reg(mmap.SCN,                 0)
        
    def init_thresholds(self):
        mask = mmap.DAC_BIST_A + mmap.DAC_BIST_B + mmap.DAC_BIST_C 
        self.handler.dac.enable_output(mask)
        
    def stop_thresholds(self):
        mask = mmap.DAC_BIST_A + mmap.DAC_BIST_B + mmap.DAC_BIST_C 
        self.handler.dac.disable_output(mask)
        
    def set_thresholds(self, thA, thB, thC):
        self.handler.write_reg(mmap.DAC_BIST_A, thA)
        self.handler.write_reg(mmap.DAC_BIST_B, thB)
        self.handler.write_reg(mmap.DAC_BIST_C, thC)
        
    def acquire_once(self, record):
    
        record.file_object.write('Low,high,vco,delay\n')
        record.file_object.write('%d,%d,%d,%d\n' % (self.low_time,self.high_time,self.vco_phase,self.counter_phase))
        
        for thA, thB, thC in record:
        
            self.set_thresholds(thA, thB, thC)
            
            self.handler.write_reg(mmap.SCN, mmap.SCN_RUN_CAPTURE)
            
            n_tries = 100
            while self.handler.read_reg(mmap.SCF) & mmap.SCF_CAPTURE_DONE:
                n_tries -= 1
                if not n_tries: raise
            
            lsbT, msbT, lsbA, msbB, lsbB, msbC, lsbC = self.handler.read_reg(mmap.SCTa, 8)
            
            probT = lsbT + msbT*256
            probA = lsbA + msbA*256
            probB = lsbB + msbB*256
            probC = lsbC + msbC*256
            
            record.save(probT, probA, probB, probC)
            
    def acquire_scan_phase(self,record,phases):
        
        for phase in phases:
            
            record = record.renew()
            
            self.set_timebase(phase)
            
            self.acquire_once(record)
    
class VerticalRecord:
    def __init__(self,file_object,compression,th_range=(0,0xFFFF)):
        self.file_object = file_object
        self.compression = compression
        self.th_range = th_range
        self.write_hearer()
        
    def write_hearer(self):
        raise NotImplementedError
        
    def __next__(self):
        raise NotImplementedError
    
    def save_point(self):
        raise NotImplementedError
        
    def renew(self):
        raise NotImplementedError
            
    next = __next__
        
        
class VerticalRecordMonotonic(VerticalRecord):
    def __init__(self,file_object,th_range=(0,0xFFFF),step=256):
        self.step = step
        self.th_range = th_range
        VerticalRecord.__init__(self,file_object,th_range)
        self.thresholds_container = xrange(th_range[0],th_range[1],step)
        
    def write_hearer(self):
        self.file_object.write('%d,%d,%d,%d\n' % (SCAN_MONOTONIC, self.th_range[0], self.th_range[1], self.step))
        self.file_object.write('A,B,C,total\n')

    def __next__(self):
        return [next(self.thresholds_container)]*3
        
    def save_point(self, total, probA, probB, probC):
        self.file_object.write('%d,%d,%d' % (probA, probB, probC))
        if total != 0xFFFF: self.file_object.write('%d\n' % total)
        else: self.file_object.write('\n')
        
    def renew(self): return VerticalRecordMonotonic(self.file_object,self.th_range,self.step)
            
    next = __next__
            
class VerticalRecordDivide(VerticalRecord):
    def __init__(self,file_object,th_range=(0,0xFFFF),max_pts=256):
        self.max_pts = max_pts
        VerticalRecord.__init__(self,file_object,th_range)
        self.thresholds_container = [th_range]
        self.next_thresholds_container = []
        
    def write_hearer(self):
        self.file_object.write('%d,%d,%d,%d\n' % (SCAN_DIVIDE, self.th_range[0], self.th_range[1], self.max_pts))
        self.file_object.write('A,B,C,total\n')
        
    def __next__(self):
        if not self.pts_to_go: raise StopIteration
        self.pts_to_go -= 1
        
        try: 
            low,high = next(self.thresholds_container)
            mid = (low + high)//2
            self.next_thresholds_container.append((low,mid))
            self.next_thresholds_container.append((mid,high))
            return [mid]*3
        except StopIteration:
            self.thresholds_container = iter(self.next_thresholds_container)
            self.next_thresholds_container = []
            return self.__next__()
            
    def save_point(self, total, probA, probB, probC):
        self.file_object.write('%d,%d,%d' % (probA, probB, probC))
        if total != 0xFFFF: self.file_object.write('%d\n' % total)
        else: self.file_object.write('\n')
        
    def renew(self): return VerticalRecordMonotonic(self.file_object,self.th_range,self.max_pts)
             
    next = __next__
    
    
class VerticalRecordAdaptative(VerticalRecord):
    def __init__(self,file_object,th_range=(0,0xFFFF),max_pts=256):
        self.max_pts = max_pts
        VerticalRecord.__init__(self,file_object,th_range)
        
        # thresholds containers
        class heap_list(list):
            # get next test threshold
            def next(self):
                weight, p_low, p_high, low, high = self[0]
                self.current_test = (low + high)//2
                return self.current_test
            # update test threshold container, with two new candidates
            def update(self,prob):
                weight, p_low, p_high, low, high = heapq.heappop(self)
                elem_low  = ( -abs(p_low-prob), p_low,   prob, low,  self.current_test)
                elem_high = (-abs(prob-p_high),  prob, p_high, self.current_test, high)
                heapq.heappush(self, elem_low)
                heapq.heappush(self, elem_high)
                
            
        self.thcA = heap_list()
        self.thcB = heap_list()
        self.thcC = heap_list()
        
        low = th_range[0]
        high = th_range[1]
        p_low  = 0xFFFF
        p_high = 0x0000
        weight = -abs(p_low-p_high)
        
        head = (weight, p_low, p_high, low, high)
        
        heapq.heappush(self.thcA, head)
        heapq.heappush(self.thcB, head)
        heapq.heappush(self.thcC, head)
        
    def write_hearer(self):
        self.file_object.write('%d,%d,%d,%d\n' % (SCAN_DIVIDE, self.th_range[0], self.th_range[1], self.max_pts))
        self.file_object.write('thA,thB,thC,A,B,C,total\n')
        
    def __next__(self):
        if not self.pts_to_go: raise StopIteration
        self.pts_to_go -= 1
        
        return (self.thcA.next(), self.thcB.next(), self.thcC.next())
        
        
    def save_point(self, total, probA, probB, probC):
    
        self.thcA.update(probA)
        self.thcA.update(probB)
        self.thcA.update(probC)
        
        to_save = (self.thcA.current_test, self.thcB.current_test, self.thcB.current_test,
            probA, probB, probB) 
        
        self.file_object.write('%d,%d,%d,%d,%d,%d' % to_save)
        if total != 0xFFFF: self.file_object.write('%d\n' % total)
        else: self.file_object.write('\n')
        
    def renew(self): return VerticalRecordMonotonic(self.file_object,self.th_range,self.max_pts)
    
    next = __next__
            
if __name__ == '__maint__':

    import file_manager

    fpga = ipc_helper.FPGAClientInterface()
    
    fpga.dac.reset_bist()
    
    #PPM Order
    PPM = int(sys.argv[1])
    
    assert sys.argv[2] in ('m','d','a')
    
    end = sys.argv[3]

    #time window size, in symbols
    SYMB = int(sys.argv[4])
    #phase, 32 by symbol
    PHA = int(sys.argv[5])
    
    MAX = 0xCCCC
    
    t_str = time.strftime("%d.%b.%Y %H.%M.%S", time.gmtime())
    with file_manager.ManagedFileOpen('/root/data/bist/%s.gz' % t_str,'w') as f, tags:
        tags['PPM']=sys.argv[1]
        tags['mode']=sys.argv[2]
        tages['mode param']=sys.argv[3]
        tags['Symbols']=sys.argv[4]
        tags['Phase']=sys.argv[5]
        
        sc = Scope(fpga)
        sc.init_thresholds()
        sc.set_frequency(slots=SYMB)
        sc.set_timebase(phase=PHA)
        
        if sys.argv[2] == 'm':
            rc = VerticalRecordMonotonic(f,th_range=(0,MAX),step=end)
        elif sys.argv[2] == 'd':
            rc = VerticalRecordDivide(f,th_range=(0,MAX),max_pts=end)
        elif sys.argv[2] == 'a':
            rc = VerticalRecordAdaptative(f,th_range=(0,MAX),max_pts=end)
            
        sc.acquire_once()