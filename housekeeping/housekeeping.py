from multiprocessing import Process
from time import sleep

import sys
import os
sys.path.append(os.path.abspath('../fpga'))
import fpga

def Camera():
    print("Test camera process")
    sleep(20)

def Fpga():
    fpga.main_loop()

def Bus():
    print("Test bus process")
    sleep(10)

if __name__ == '__main__':
    tasks = [Camera, Fpga, Bus]
    processes = {}
    n = 0

    for task in tasks:
        p = Process(target=task)
        p.start()
        processes[n] = (p, task)
        n += 1

    while(len(processes) > 0):
        for n in list(processes):
            (p, t) = processes[n]
            if p.exitcode is None: # Not finished
                if not p.is_alive(): # Not running
                    print(t, "not finished and not running")
                    # Do error handling and restarting here assigning the new process to processes[n]
                else:
                    print(t, "not finished")
            elif p.exitcode < 0:
                print(t, "ended with an error", p.exitcode)
                # Handle this either by restarting or delete the entry so it is removed from list as for else
            else:
                print(t, "finished")
                p.join() # Allow tidyup
                del processes[n] # Removed finished items from the dictionary 
                # When none are left then loop will end
        sleep(5)
    print("FINISHED")


