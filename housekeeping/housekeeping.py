from multiprocessing import Process
from time import sleep

import sys
import time
import os
import subprocess
import zmq

import struct
import psutil

sys.path.append('/root/CLICK-A-RPi/lib/')
sys.path.append('../lib/')
from ipc_packets import TxPacket, HandlerHeartbeatPacket, FPGAMapRequestPacket, FPGAMapAnswerPacket, HousekeepingControlPacket, PATControlPacket, PATHealthPacket
from options import PAT_HEALTH_PORT, FPGA_MAP_REQUEST_PORT, FPGA_MAP_ANSWER_PORT
from options import TX_PACKETS_PORT, HK_CONTROL_PORT, CH_HEARTBEAT_PORT, CH_HEARTBEAT_PD
from zmqTxRx import push_zmq, send_zmq, recv_zmq

FPGA_CHECK_PD = 6000
CPU_CHECK_PD = 6000

PAT_HEALTH_PD = 6000
#CH_HEARTBEAT_PD = 6000 - import from options (shared with command handler)
FPGA_ANS_PD = 6000

CPU_APID = 0x312
PAT_HEALTH_APID = 0x313
FPGA_MAP_APID = 0x314

## 1 indicates the housekeeper will send packets
## 0 indicates the housekeeper will not send packets
HK_SEND_MODE = 1

class Housekeeping:
    pid = os.getpid()

    context = zmq.Context()
    pat_health_socket = context.socket(zmq.SUB)
    fpga_req_socket = context.socket(zmq.PUB)
    fpga_ans_socket = context.socket(zmq.SUB)
    tx_socket = context.socket(zmq.PUB)
    hk_control_socket = context.socket(zmq.SUB)
    ch_heartbeat_socket = context.socket(zmq.SUB)

    poller = zmq.Poller()

    packet_buf = []
    procs = {'camera':0xA0, 'commandhandler':0xA1, 'fpga':0xA2, 'bus_tx':0xA3, 'bus_rx':0xA4}

    def __init__(self):

        self.pat_health_socket.connect("tcp://127.0.0.1:%s" % PAT_HEALTH_PORT)
        self.fpga_req_socket.connect("tcp://127.0.0.1:%s" % FPGA_MAP_REQUEST_PORT)
        self.fpga_ans_socket.connect("tcp://127.0.0.1:%s" % FPGA_MAP_ANSWER_PORT)
        self.tx_socket.connect("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)
        self.hk_control_socket.connect("tcp://127.0.0.1:%s" % HK_CONTROL_PORT)
        self.ch_heartbeat_socket.connect("tcp://127.0.0.1:%s" % CH_HEARTBEAT_PORT)


        self.poller.register(self.pat_health_socket, zmq.POLLIN)
        self.poller.register(self.hk_control_socket, zmq.POLLIN)
        self.poller.register(self.ch_heartbeat_socket, zmq.POLLIN)
        self.poller.register(self.fpga_ans_socket, zmq.POLLIN)
        # self.poller.register(fpga_req_socket, zmq.POLLOUT)
        # self.poller.register(tx_socket, zmq.POLLOUT)
        self.fpga_ans_socket.setsockopt(zmq.SUBSCRIBE, str(self.pid).encode('ascii'))

    def check_fpga_mmap(self):
        pkt = FPGAMapRequestPacket()
        tc_pkt = pkt.encode(self.pid, 0, 0, 0, 15) # registers 0 through 14
        self.fpga_req_socket.send(tc_pkt)
        tc_pkt = pkt.encode(self.pid, 1, 0, 32, 7) # registers 32 through 38
        self.fpga_req_socket.send(tc_pkt)
        tc_pkt = pkt.encode(self.pid, 2, 0, 49, 20) # registers 49 through 68
        self.fpga_req_socket.send(tc_pkt)
        tc_pkt = pkt.encode(self.pid, 3, 0, 96, 22) # registers 96 through 117
        return

    def check_cpu(self):

        pkt = bytearray()

        mem = psutil.virtual_memory()
        mem_total = mem.total
        mem_available = mem.available
        #print("MemTotal: ", mem_total, "MemAvailable: ", mem_available)

        pkt.extend(struct.pack('L', mem_available))

        for p in psutil.process_iter(['pid','name','cpu_percent','memory_percent']):
            p_name = p.info['name']
            if p_name in self.procs:
                print(p_name, ": ", p.cpu_percent(), " - ", p.memory_percent('uss'))
                pkt.extend(struct.pack('B', self.procs[p_name]))
                pkt.extend(struct.pack('L', int(p.cpu_percent())))
                pkt.extend(struct.pack('L', int(p.memory_percent('uss'))))

        count = 0;
        with open('/mnt/journal/id.txt', 'r') as boot_id_list:
            for count, l in enumerate(boot_id_list, 1):
                pass

        pkt.extend(struct.pack('B', count))
    
        return pkt

    def handle_hk_pkt(self, data, process):
        pkt = TxPacket()
        if process is 'camera':
            apid = PAT_HEALTH_APID
            pat_pkt = PATHealthPacket()
            _, return_addr, size, payload = pat_pkt.decode(data)
            payload = data

        elif process is 'fpga':
            apid = FPGA_MAP_APID
            fpga_pkt = FPGAMapAnswerPacket()
            ##TBD - Should actually check the received packet for errors
            return_addr, rq_number, rw_flag, error, start_addr, size, read_data = fpga_pkt.decode(data)
            if error is 0:
                payload = read_data
            else:
                return

        elif process is 'cpu':
            apid = CPU_APID
            payload = data

        raw_pkt = pkt.encode(apid, payload)
        self.packet_buf.append(raw_pkt)

    def restart_process(self, process):
        status = subprocess.call("systemctl --user restart " + process)

    def handle_hk_command(self, command):
        hk = HousekeepingControlPacket()
        cmd, payload = hk.decode(command)
        ##TBD - handle command

    def run(self):
        print("Start HK process")

        curr_time = time.time()

        fpga_check_ts = curr_time
        cpu_check_ts = curr_time
        pat_health_packet_ts = curr_time
        ch_heartbeat_ts = curr_time
        fpga_ans_ts = curr_time

        hk_send_mode = HK_SEND_MODE

        while True:
            ## Check if it's time to check FPGA health or memory, do so if necessary
            if ((time.time() - fpga_check_ts) > FPGA_CHECK_PD):
                self.check_fpga_mmap()
                fpga_check_ts = time.time()
                fpga_req_num_cnt += 1

            if ((time.time() - cpu_check_ts) > CPU_CHECK_PD):
                message = self.check_cpu()
                self.handle_hk_pkt(message, 'cpu')
                ##handle CPU data packet
                cpu_check_ts = time.time()

            ## Receive packets from the other processes
            sockets = dict(self.poller.poll(0)) # No timeout
            if self.pat_health_socket in sockets and sockets[self.pat_health_socket] == zmq.POLLIN:
                message = self.pat_health_socket.recv()
                ##handle pat health packet
                self.handle_hk_pkt(message, 'pat')
                pat_health_packet_ts = time.time()

            elif self.fpga_ans_socket in sockets and sockets[self.fpga_ans_socket] == zmq.POLLIN:
                message = self.fpga_ans_socket.recv()
                self.handle_hk_pkt(message, 'fpga')
                ##handle fpga health packet

            elif self.ch_heartbeat_socket in sockets and sockets[self.ch_heartbeat_socket] == zmq.POLLIN:
                print("Received CH Heartbeat")
                message = self.ch_heartbeat_socket.recv()
                ch_packet = HandlerHeartbeatPacket()
                ch_packet.decode(message)
                print(ch_packet) 
                ##handle ch heartbeat packet, currently ignoring timestamp and assuming 1 command handler
                ch_heartbeat_ts = time.time()

            elif self.hk_control_socket in sockets and sockets[self.hk_control_socket] == zmq.POLLIN:
                message = self.hk_control_socket.recv()
                self.handle_hk_command(message)

            ## Check if any timeouts have been exceeded
            curr_time = time.time()
            if ((curr_time - pat_health_packet_ts) > PAT_HEALTH_PD):
                self.restart_process("camera")

            if ((curr_time - ch_heartbeat_ts) > CH_HEARTBEAT_PD):
                self.restart_process("commandhandler")

            if ((fpga_ans_ts - fpga_check_ts) < 0) and ((curr_time - fpga_check_ts) > FPGA_CHECK_PD):
                self.restart_process("fpga")

            if (HK_SEND_MODE):
                ## Send to tx socket
                while self.packet_buf:
                    self.tx_socket.send(self.packet_buf.pop(0))

if __name__ == '__main__':
    housekeeper = Housekeeping()
    housekeeper.run()
