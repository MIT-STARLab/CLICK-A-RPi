from multiprocessing import Process
from time import sleep

from hazelnut import MemInfo

import sys
import os
import zmq


CH_HEARTBEAT_PORT = "5555"
HK_CONTROL_PORT = "5556"

FPGA_MAP_ANSWER_PORT = "5557"
FPGA_MAP_REQUEST_PORT = "5558"

PAT_HEALTH_PORT = "5559"
PAT_CONTROL_PORT = "5560"

TX_PACKETS_PORT = "5561"
RX_CMD_PACKETS_PORT = "5562"
RX_PAT_PACKETS_PORT = "5563"

def PAT():
    print("Start PAT process")
    context = zmq.Context()
    health_socket = context.socket(zmq.PUB)
    health_socket.connect("tcp://127.0.0.1:%s" % PAT_HEALTH_PORT)

    control_socket = context.socket(zmq.SUB)
    control_socket.connect("tcp://127.0.0.1:%s" % PAT_CONTROL_PORT)

    tx_socket = context.socket(zmq.PUB)
    tx_socket.connect("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)

    rx_pat_socket = context.socket(zmq.SUB)
    rx_pat_socket.connect("tcp://127.0.0.1:%s" % RX_PAT_PACKETS_PORT)

    fpga_req_socket = context.socket(zmq.PUB)
    fpga_req_socket.connect("tcp://127.0.0.1:%s" % FPGA_MAP_REQUEST_PORT)

    fpga_ans_socket = context.socket(zmq.SUB)
    fpga_ans_socket.connect("tcp://127.0.0.1:%s" % FPGA_MAP_ANSWER_PORT)

    n = 0
    while True:
        health_socket.send_string("Message from PAT %s" % n)
        tx_socket.send_string("Message from PAT %s" % n)
        n += 1
        sleep(5)


def CommandHandler():
    print("Start CommandHandler process")
    context = zmq.Context()

    hk_control_socket = context.socket(zmq.PUB)
    hk_control_socket.connect("tcp://127.0.0.1:%s" % HK_CONTROL_PORT)

    ch_heartbeat_socket = context.socket(zmq.PUB)
    ch_heartbeat_socket.connect("tcp://127.0.0.1:%s" % CH_HEARTBEAT_PORT)

    fpga_req_socket = context.socket(zmq.PUB)
    fpga_req_socket.connect("tcp://127.0.0.1:%s" % FPGA_MAP_REQUEST_PORT)

    fpga_ans_socket = context.socket(zmq.SUB)
    fpga_ans_socket.connect("tcp://127.0.0.1:%s" % FPGA_MAP_ANSWER_PORT)

    pat_control_socket = context.socket(zmq.PUB)
    pat_control_socket.connect("tcp://127.0.0.1:%s" % PAT_CONTROL_PORT)

    rx_cmd_socket = context.socket(zmq.SUB)
    rx_cmd_socket.connect("tcp://127.0.0.1:%s" % RX_CMD_PACKETS_PORT)

    tx_socket = context.socket(zmq.PUB)
    tx_socket.connect("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)

    n = 0
    while True:
        pat_control_socket.send_string("Message from CH %s" % n)
        pat_control_socket.send_string("Message from CH %s" % n)
        n += 1
        sleep(5)

def FpgaDriver():
    print("Start FPGA Driver process")
    context = zmq.Context()
    req_socket = context.socket(zmq.SUB)
    req_socket.connect("tcp://127.0.0.1:%s" % FPGA_MAP_REQUEST_PORT)
    ans_socket = context.socket(zmq.PUB)
    ans_socket.connect("tcp://127.0.0.1:%s" % FPGA_MAP_ANSWER_PORT)
    
    while True:
        try:
            result = req_socket.recv(flags=zmq.NOBLOCK) #this doesn't block
            print("FPGA: received ", result) 
            ans_socket.send_string("FPGA answer to request %s" % result.decode('ascii')[-1])
        except:
            pass
            
        sleep(1)
        

def BusInterface():
    print("Start Bus process")
    context = zmq.Context()
    tx_socket = context.socket(zmq.SUB)
    tx_socket.connect("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)

    rx_cmd_socket = context.socket(zmq.PUB)
    rx_cmd_socket.connect("tcp://127.0.0.1:%s" % RX_CMD_PACKETS_PORT)
    rx_pat_socket = context.socket(zmq.PUB)
    rx_pat_socket.connect("tcp://127.0.0.1:%s" % RX_PAT_PACKETS_PORT)

    while True:
        try:
            print("Bus: received ", tx_socket.recv(flags=zmq.NOBLOCK)) #this doesn't block
        except:
            pass
        sleep(1)

class Housekeeping:
    tasks = [PAT, FpgaDriver, BusInterface, CommandHandler, CommandHandler]
    processes = {}
    
    context = zmq.Context()
    pat_health_socket = context.socket(zmq.SUB)
    fpga_req_socket = context.socket(zmq.PUB)
    fpga_ans_socket = context.socket(zmq.SUB)
    tx_socket = context.socket(zmq.PUB)
    hk_control_socket = context.socket(zmq.SUB)
    ch_heartbeat_socket = context.socket(zmq.SUB)
    

    def __init__(self):

        self.pat_health_socket.connect("tcp://127.0.0.1:%s" % PAT_HEALTH_PORT)
        self.fpga_req_socket.connect("tcp://127.0.0.1:%s" % FPGA_MAP_REQUEST_PORT)
        self.fpga_ans_socket.connect("tcp://127.0.0.1:%s" % FPGA_MAP_ANSWER_PORT)
        self.tx_socket.connect("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)
        self.hk_control_socket.connect("tcp://127.0.0.1:%s" % HK_CONTROL_PORT)
        self.ch_heartbeat_socket.connect("tcp://127.0.0.1:%s" % CH_HEARTBEAT_PORT)

        n = 0
        for task in self.tasks:
            p = Process(target=task)
            p.start()
            self.processes[n] = (p, task)
            n += 1

    def check_fpga_mmap(self):
        return

    def check_pat(self):
        return

    def check_mem(self):
        #Uses hazelnut library
        mem = MemInfo()
        mem_total = mem.get('MemTotal') # total usable RAM of the system (in kB)
        mem_free = mem.get('MemFree') # total amount of free memory (includes low_Free)
        mem_available = mem.get('MemAvailable') # estimate of how much memory is available for starting new apps
        print("MemTotal: ", mem_total, "MemFree: ", mem_free, "MemAvailble: ", mem_available)

    def run(self):
        print("Start HK process")  
        n = 0
        while True:
            # Ping Bus and FPGA
            self.tx_socket.send_string("Message part a from HK %s" % n)
            self.tx_socket.send_string("Message part b from HK %s" % n)
            self.fpga_req_socket.send_string("Request from HK %s" % n)
            
            # Check for PAT health messages
            try:
                print("HK: received ", self.pat_health_socket.recv(flags=zmq.NOBLOCK))
            except:
                pass
            
            # Check for FPGA answer messages
            try:
                print("HK: received ", self.fpga_ans_socket.recv(flags=zmq.NOBLOCK))
            except:
                pass
            
            n += 1

            # Check processes are alive
            # for i in list(self.processes):
            #     (p, t) = self.processes[i]
            #     if p.exitcode is None: # Not finished
            #         if not p.is_alive(): # Not running
            #             print(t, "not finished and not running")
            #             # Do error handling and restarting here assigning the new process to processes[n]
            #         else:
            #             print(t, "not finished")
            #     elif p.exitcode < 0:
            #         print(t, "ended with an error", p.exitcode)
            #         # Handle this either by restarting or delete the entry so it is removed from list as for else
            #     else:
            #         print(t, "finished")
            #         p.join() # Allow tidyup
            #         del self.processes[i] # Removed finished items from the dictionary 

            sleep(2)

if __name__ == '__main__':
    housekeeper = Housekeeping()
    housekeeper.run()






