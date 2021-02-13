#!/usr/bin/env python
from multiprocessing import Process

import sys
import time
import os
import threading
import struct
import zmq
import psutil
import binascii
import struct
import subprocess
import Queue

sys.path.append('/root/lib/')
sys.path.append('../lib/')

import ipc_helper
from ipc_packets import TxPacket, CHHeartbeatPacket, FPGAMapRequestPacket, FPGAMapAnswerPacket, HKControlPacket, PATControlPacket, PATHealthPacket
from options import *
from fpga_map import FPGA_TELEM, FPGA_TELEM_TYPE
from zmqTxRx import push_zmq, send_zmq, recv_zmq

HK_PAT_ID = 0xA0
HK_CH_ID = 0xA1
HK_FPGA_ID = 0xA2
HK_PKT_ID = 0xA3
HK_DEPKT_ID = 0xA4
HK_SYS_ID = 0xA5

class ResetTimer:
    def __init__(self, interval, function, *args, **kwargs):
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs

        self.timer = None

    def callback(self):
        self.function(*self.args, **self.kwargs)
        self.start()

    def cancel(self):
        self.timer.cancel()

    def set_interval(self, interval):
        self.interval = interval

    def start(self):
        self.timer = threading.Timer(self.interval, self.callback)
        self.timer.start()

class WatchdogTimer:
    def __init__(self, timeout, function, *args, **kwargs):
        self.timeout = timeout
        self.function = function
        self.args = args
        self.kwargs = kwargs

        self.timer = None

    def callback(self):
        self.function(*self.args, **self.kwargs)

    def cancel(self):
        self.timer.cancel()

    def set_timeout(self, timeout):
        self.timeout = timeout

    def kick(self):
        self.cancel()
        self.start()

    def start(self):
        self.timer = threading.Timer(self.timeout, self.callback)
        self.timer.start()

class Housekeeping:
    # TODO: Update if necessary
    procs = {HK_PAT_ID:'/root/bin/pat',
             HK_CH_ID:'/root/commandhandler/commandhandler.py',
             HK_FPGA_ID:'/root/fpga/fpga.py',
             HK_PKT_ID:'/root/bus/packetizer.py',
             HK_DEPKT_ID:'/root/bus/depacketizer.py',
             HK_SYS_ID: '/root/housekeeping/housekeeping.py'}

    procs.update(dict(reversed(item) for item in procs.items()))

    def __init__(self):
        self.pid = os.getpid()

        # Initialize enable
        self.all_pkts_send_enable = HK_ALLPKTS_SEND_ENABLE
        self.fpga_req_enable = HK_FPGA_REQ_ENABLE
        self.sys_hk_send_enable  = HK_SYS_HK_SEND_ENABLE
        self.fpga_hk_send_enable = HK_FPGA_HK_SEND_ENABLE
        self.pat_hk_send_enable = HK_PAT_HK_SEND_ENABLE
        self.ch_restart_enable = HK_CH_RESTART_ENABLE
        self.pat_restart_enable = HK_PAT_RESTART_ENABLE
        self.fpga_restart_enable = HK_FPGA_RESTART_ENABLE

        # Initialize timing/check-related variables
        self.fpga_check_period = HK_FPGA_CHECK_PD
        self.sys_check_period = HK_SYS_CHECK_PD
        self.ch_heartbeat_period = HK_CH_HEARTBEAT_PD
        self.pat_health_period = HK_PAT_HEALTH_PD

        self.fpga_check_timer = ResetTimer(self.fpga_check_period, self.alert_fpga_check)
        self.sys_check_timer = ResetTimer(self.sys_check_period, self.alert_sys_check)

        self.ch_heartbeat_wds = {}
        self.pat_health_wd = WatchdogTimer(self.pat_health_period, self.alert_missing_pat)

        self.fpga_interface = ipc_helper.FPGAClientInterface()

        self.fpga_queue = Queue.Queue()

        self.fpga_check_flag = threading.Event()
        self.sys_check_flag = threading.Event()
        self.missing_ch_flag = threading.Event()
        self.missing_pat_flag = threading.Event()
        self.missing_fpga_flag = threading.Event()

        self.fpga_check_flag.clear()
        self.sys_check_flag.clear()
        self.missing_ch_flag.clear()
        self.missing_pat_flag.clear()
        self.missing_fpga_flag.clear()

        # Initialize zmq-related variables
        # TODO: Update zmq connections, use library

        self.context = zmq.Context()
        self.pat_health_socket = self.context.socket(zmq.SUB)
        self.tx_socket = self.context.socket(zmq.PUB)
        self.hk_control_socket = self.context.socket(zmq.SUB)
        self.ch_heartbeat_socket = self.context.socket(zmq.SUB)

        self.pat_health_socket.bind("tcp://127.0.0.1:%s" % PAT_HEALTH_PORT) #pat process is not already running
        self.tx_socket.connect("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)
        self.hk_control_socket.connect("tcp://127.0.0.1:%s" % HK_CONTROL_PORT)
        self.ch_heartbeat_socket.connect("tcp://127.0.0.1:%s" % CH_HEARTBEAT_PORT)

        self.pat_health_socket.setsockopt(zmq.SUBSCRIBE, b'')

        self.poller = zmq.Poller()

        self.poller.register(self.pat_health_socket, zmq.POLLIN)
        self.poller.register(self.hk_control_socket, zmq.POLLIN)
        self.poller.register(self.ch_heartbeat_socket, zmq.POLLIN)

        # Initialize packet buffer
        self.packet_buf = []

        # Initialize counters
        self.sys_hk_count = 0
        self.fpga_hk_count = 0
        self.ack_cmd_count = 0
        self.err_cmd_count = 0

        # Initialize command tracking
        self.last_ack_cmd_id = 0
        self.last_err_cmd_id = 0

    def alert_fpga_check(self):
        self.fpga_check_flag.set()

    def alert_sys_check(self):
        self.sys_check_flag.set()

    def alert_missing_ch(self):
        self.missing_ch_flag.set()

    def alert_missing_pat(self):
        self.missing_pat_flag.set()

    def alert_missing_fpga(self):
        self.missing_fpga_flag.set()

    def init_fpga_read(self):
        if (self.fpga_req_enable):
            fpga_thread = threading.Thread(target=self.fpga_read)
            fpga_thread.setDaemon(True)
            fpga_thread.start()

    def fpga_read(self):
        try:
            read = self.fpga_interface.read_reg(FPGA_TELEM_TYPE)
            self.fpga_queue.put(read)
        except:
            print("Message missing fpga")
            self.alert_missing_fpga()

    def check_fpga(self, answer_pkt):
        pkt = ''

        # 0: HK counter
        pkt += struct.pack('B', self.fpga_hk_count % 256)

        # 1-244: FPGA Telemetry Packet
        # Handle FPGA packet format
        for i in range(0, len(answer_pkt)):
            pkt += struct.pack(FPGA_TELEM_TYPE[i], answer_pkt[i])
        return pkt

    def check_sys(self):
        pkt = ''

        # 0: HK counter
        pkt += struct.pack('B', self.sys_hk_count % 256)

        # 1: Enable flags
        enables = 0
        enables |= (self.all_pkts_send_enable & 1) << 7
        enables |= (self.fpga_req_enable & 1) << 6
        enables |= (self.sys_hk_send_enable & 1) << 5
        enables |= (self.fpga_hk_send_enable & 1) << 4
        enables |= (self.pat_hk_send_enable & 1) << 3
        enables |= (self.ch_restart_enable & 1) << 2
        enables |= (self.pat_restart_enable & 1) << 1
        enables |= (self.fpga_restart_enable & 1) << 0

        pkt += struct.pack('B', enables)

        # 2: FPGA housekeeping period
        pkt += struct.pack('B', self.fpga_check_period)
        # 3: System housekeeping period
        pkt += struct.pack('B', self.sys_check_period)
        # 4: Command handler heartbeat period
        pkt += struct.pack('B', self.ch_heartbeat_period)
        # 5: PAT health period
        pkt += struct.pack('B', self.pat_health_period)

        # 7: Acknowledged command count
        pkt += struct.pack('B', self.ack_cmd_count)
        # 8: Last acknowledged command ID
        pkt += struct.pack('B', self.last_ack_cmd_id)
        # 9: Error command count
        pkt += struct.pack('B', self.err_cmd_count)
        # 10: Last error command ID
        pkt += struct.pack('B', self.last_err_cmd_id)

        # 11-12: Boot count
        try:
            with open('/mnt/journal/id.txt', 'r') as boot_id_list:
                for count, l in enumerate(boot_id_list, 1):
                    pass
            pkt += struct.pack('!L', count)
        except:
            # Error if journal file can't be opened
            pkt += struct.pack('!L', 0xFFFFFFFF)

        # 13-16: Disk usage
        # TODO: Update path if necessary
        disk = psutil.disk_usage('/')
        pkt += struct.pack('!L', (disk.used % 2**32))

        # 17-20: Disk free
        pkt += struct.pack('!L', (disk.free % 2**32))

        # 21-24: Available virtual memory
        vmem = psutil.virtual_memory()
        pkt += struct.pack('!L', (vmem.available % 2**32))

        # 25-N: Process info
        for p in psutil.process_iter(['name','cmdline','cpu_percent','memory_percent']):
            p_name = ''
            if (p.info['name'] == 'python' and len(p.info['cmdline']) == 3):
                p_name = p.info['cmdline'][2]
            elif (p.info['name'] == 'pat' and len(p.info['cmdline']) == 1):
                p_name = p.info['cmdline'][0]

            if p_name in self.procs:
                pkt += struct.pack('B', self.procs[p_name])
                pkt += struct.pack('B', (p.cpu_percent() % 256))
                pkt += struct.pack('B', (p.memory_percent('rss') % 256))
        return pkt


    def handle_hk_pkt(self, data, process_id):
        # TODO: Update packet handling if necessary

        if (process_id == HK_PAT_ID):
            pat_pkt = PATHealthPacket()
            apid = TLM_HK_PAT
            payload, _, _, _ = pat_pkt.decode(data)
            #payload = struct.pack('%ds'%len(payload), payload) #for readability, could have this, though it doesn't do anything (packed string = original string)
            print('Handling PAT pkt w/ payload: ', payload)

        elif (process_id == HK_FPGA_ID):
            apid = TLM_HK_FPGA_MAP
            payload = struct.pack('B', data)
            print('Handling FPGA pkt w/ payload: ', payload)

        elif (process_id == HK_SYS_ID):
            apid = TLM_HK_SYS
            payload = data #data is already a packed byte string
            print('Handling SYS pkt w/ payload: ', payload)

        elif (process_id == HK_CH_ID):
            apid = TLM_HK_CH
            ch_pkt = HKControlPacket()
            origin, _, data = ch_pkt.decode(data)
            # TODO: Maybe format this better
            payload = str(origin)+": "+data
            #payload = struct.pack('%ds'%len(payload), payload) #for readability, could have this, though it doesn't do anything (packed string = original string)
            print('Handling CH pkt w/ payload: ', payload)

        pkt = TxPacket()
        raw_pkt = pkt.encode(apid, payload) #payload needs to be a single packed byte string e.g. '\x00\x01'
        self.packet_buf.append(raw_pkt)

    def restart_process(self, process_id):
        print("Restart process")
        if (process_id == HK_CH_ID and self.ch_restart_enable):
            print("Restart ch")
            subprocess.call("systemctl --user restart commandhandler.service", shell = True)
        if (process_id == HK_PAT_ID and self.pat_restart_enable):
            print("Restart pat")
            subprocess.call("systemctl --user restart pat.service", shell = True)
        if (process_id == HK_FPGA_ID and self.fpga_restart_enable):
            print("Restart fpga")
            status = subprocess.call("systemctl --user restart fpga.service", shell = True)

        err_pkt = TxPacket()
        raw_err_pkt = err_pkt.encode(ERR_HK_RESTART, struct.pack('B', process_id))
        self.packet_buf.append(raw_err_pkt)

    def handle_hk_command(self, command):
        flags, new_fpga_check_pd, new_sys_check_pd, new_ch_heartbeat_pd, new_pat_health_pd = struct.unpack('!BBBBB', command)

        self.all_pkts_send_enable = (flags >> 7) & 1
        self.fpga_req_enable = (flags >> 6) & 1
        self.sys_hk_send_enable  = (flags >> 5) & 1
        self.fpga_hk_send_enable = (flags >> 4) & 1
        self.pat_hk_send_enable = (flags >> 3) & 1
        self.ch_restart_enable = (flags >> 2) & 1
        self.pat_restart_enable = (flags >> 1) & 1
        self.fpga_restart_enable = (flags >> 0) & 1

        self.fpga_check_period = new_fpga_check_pd
        self.sys_check_period = new_sys_check_pd
        self.ch_heartbeat_period = new_ch_heartbeat_pd
        self.pat_health_period = new_pat_health_pd

    def run(self):
        self.fpga_check_timer.start()
        self.sys_check_timer.start()
        self.pat_health_wd.start()

        while True:
            # Periodically send FPGA request
            if (self.fpga_check_flag.is_set()):
                self.init_fpga_read()
                self.fpga_check_flag.clear()

            # Periodically generate system HK
            if (self.sys_check_flag.is_set()):
                message = self.check_sys()
                self.handle_hk_pkt(message, HK_SYS_ID)
                self.sys_check_flag.clear()

            if (self.missing_ch_flag.is_set()):
                self.restart_process(HK_CH_ID)
                self.missing_ch_flag.clear()

            if (self.missing_pat_flag.is_set()):
                self.restart_process(HK_PAT_ID)
                self.missing_pat_flag.clear()

            if (self.missing_fpga_flag.is_set()):
                self.restart_process(HK_FPGA_ID)
                self.missing_fpga_flag.clear()

            if(self.fpga_queue.qsize() > 0):
                pkt = self.check_fpga(self.fpga_queue.get())
                self.handle_hk_pkt(pkt, HK_FPGA_ID)

            # Receive packets from the other processes
            sockets = dict(self.poller.poll(500)) # 500 ms timeout
            if self.pat_health_socket in sockets and sockets[self.pat_health_socket] == zmq.POLLIN:
                message = self.pat_health_socket.recv()
                self.handle_hk_pkt(message, HK_PAT_ID)
                self.pat_health_wd.kick()

            elif self.ch_heartbeat_socket in sockets and sockets[self.ch_heartbeat_socket] == zmq.POLLIN:
                message = self.ch_heartbeat_socket.recv()
                ch_packet = CHHeartbeatPacket()
                ch_packet.decode(message)

                if ch_packet.origin in self.ch_heartbeat_wds:
                    self.ch_heartbeat_wds[ch_packet.origin].kick()
                else:
                    # TODO: restarting commandhandler needs to consider multiple commandhandlers
                    self.ch_heartbeat_wds[ch_packet.origin] = WatchdogTimer(self.ch_heartbeat_period, self.alert_missing_ch)
                    self.ch_heartbeat_wds[ch_packet.origin].start()

            elif self.hk_control_socket in sockets and sockets[self.hk_control_socket] == zmq.POLLIN:
                message = self.hk_control_socket.recv()
                # Check if command or log or ack packet
                hk_packet = HKControlPacket()
                hk_packet.decode(message)
                if (hk_packet.command == HK_CONTROL_ACK):
                    # Handle ack packet
                    cmd_id, status = struct.unpack('HH', hk_packet.payload)

                    if (status == CMD_ACK):
                        self.ack_cmd_count += 1
                        self.last_ack_cmd_id = cmd_id
                    elif (status == CMD_ERR):
                        self.err_cmd_count += 1
                        self.last_err_cmd_id = cmd_id

                elif (hk_packet.command == HK_CONTROL_LOG):
                    # TODO: Potentially update handling log packet
                    self.handle_hk_pkt(message, TLM_HK_CH)

                elif (hk_packet.command == CMD_PL_SET_HK):
                    # handle command packet
                    self.handle_hk_command(hk_packet.payload)

            # Send housekeeping packets to TX packets queue
            while self.packet_buf:
                current_pkt = self.packet_buf.pop(0)
                if (self.all_pkts_send_enable):
                    self.tx_socket.send(current_pkt)

            # print("HK is running")

if __name__ == '__main__':
    housekeeper = Housekeeping()
    housekeeper.run()
