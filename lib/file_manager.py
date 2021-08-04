#!/usr/bin/env python
import os
import csv
import gzip
import sys
import time
sys.path.append('/root/lib/')
import options

class ManagedFileOpen:

    def __init__(self, name, mode):
        self.f = gzip.open(name, mode)
        self.tags = {'_name':name, '_time':str(int(time.time()))}
        
        self.name = name
        self.path = os.path.dirname(self.name)
        
        self.update_symlink()
        
    def update_symlink(self):
        sym_path_current = os.path.join(self.path,str(options.SYMLINK_MAX))
        if os.path.exists(sym_path_current): os.remove(sym_path_current)
        
        for idx in range(options.SYMLINK_MAX,0,-1):
            sym_path_past = sym_path_current
            sym_path_current = os.path.join(self.path,str(idx-1))
            if os.path.exists(sym_path_current):
                os.rename(sym_path_current, sym_path_past)
        
        sym_path_current = os.path.join(self.path,'0')
        os.symlink(self.name, sym_path_current)
        
    def update_index(self):
    
        idx_path      = os.path.join(self.path,'index.gz')
        idx_path_temp = os.path.join(self.path,'index.temp.gz')
        new_keys = self.tags.keys()
        
        if os.path.exists(idx_path):
        
            with gzip.open(idx_path,'r') as index_in, gzip.open(idx_path_temp,'w') as index_out:

                key_reader = csv.DictReader(index_in,delimiter=',')

                old_keys = key_reader.fieldnames
                if old_keys: keys = old_keys + [k for k in new_keys if k not in old_keys]
                else: keys = new_keys
                
                key_writer = csv.DictWriter(index_out,fieldnames=keys,delimiter=',')
                
                key_writer.writeheader()
                #key_writer.writerows(key_reader)
                file_data = list(key_reader)
                file_count = len(file_data)
                excess_file_count = file_count - options.SYMLINK_MAX
                for i in range(0,file_count):
                    if(excess_file_count > 0):
                        os.remove(file_data[i]['_name']) #clean up old files and stop indexing them
                        excess_file_count -= 1
                    else:
                        key_writer.writerow(file_data[i])
                key_writer.writerow(self.tags)
                
            os.remove(idx_path)
            os.rename(idx_path_temp, idx_path)
                
        else: 
            old_keys = ['_name','_time','_size']
            keys = old_keys + [k for k in new_keys if k not in old_keys]
            
            with gzip.open(idx_path,'w') as index_out:
                key_writer = csv.DictWriter(index_out,fieldnames=keys,delimiter=',')
                key_writer.writeheader()
                key_writer.writerow(self.tags)
        
    def __enter__(self):
        return self.f, self.tags
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.f.close()
        self.tags['_size'] = str(os.stat(self.name).st_size)
        self.update_index()
        
if __name__ == "__main__":
    
    with ManagedFileOpen('texta.gz','w') as (fl,tags):
        fl.write('Hello,world\n')
        fl.write('I am groot.\n')
        tags['tree']='groot'
        tags['raccoon']='yes'
        
    with ManagedFileOpen('textb.gz','w') as (fl,tags):
        fl.write('1,2,3\n')
        fl.write('4,5,6\n')
        tags['raccoon']='no'
        tags['count']='no?'
        tags['count']='yes!'
        
    with ManagedFileOpen('texta.gz','a') as (fl,tags):
        fl.write('1,2,3\n')
        fl.write('I am groot.\n')
        tags['tree']='groot!'
        tags['raccoon']='yes'