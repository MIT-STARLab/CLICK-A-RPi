from multiprocessing import Process

import sys
import time
import os
import threading
import struct
import subprocess
import zmq
import psutil
import binascii
import struct
sys.path.append('/root/CLICK-A-RPi/lib/')
sys.path.append('../lib/')
from ipc_packets import TxPacket, HandlerHeartbeatPacket, FPGAMapRequestPacket, FPGAMapAnswerPacket, HousekeepingControlPacket, PATControlPacket, PATHealthPacket
from options import PAT_HEALTH_PORT, FPGA_MAP_REQUEST_PORT, FPGA_MAP_ANSWER_PORT
from options import TX_PACKETS_PORT, HK_CONTROL_PORT, CH_HEARTBEAT_PORT, TLM_HK_SYS, TLM_HK_PAT, TLM_HK_FPGA_MAP
from options import HK_FPGA_CHECK_PD, HK_SYS_CHECK_PD, HK_CH_HEARTBEAT_PD, HK_PAT_HEALTH_PD, HK_FPGA_ANS_PD
from options import HK_FPGA_REQ_ENABLE, HK_SYS_HK_SEND_ENABLE, HK_FPGA_HK_SEND_ENABLE, HK_PAT_HK_SEND_ENABLE, HK_CH_RESTART_ENABLE, HK_PAT_RESTART_ENABLE, HK_FPGA_RESTART_ENABLE, HK_ALLPKTS_SEND_ENABLE
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

class ProcessID:
    def __init__(self, id, name):
        self.id = id
        self. name = name

class Housekeeping:
    # TODO: Update this for whatever the real process names are
    # TODO: Add additional processes as necessary
    procs = {HK_PAT_ID:'pat process',
             HK_CH_ID:'ch process',
             HK_FPGA_ID:'fpga process',
             HK_PKT_ID:'packetizer process',
             HK_DEPKT_ID:'depacketizer process'}
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
        self.fpga_ans_period = HK_FPGA_ANS_PD

        self.fpga_check_timer = ResetTimer(self.fpga_check_period, self.alert_fpga_check)
        self.sys_check_timer = ResetTimer(self.sys_check_period, self.alert_sys_check)

        self.ch_heartbeat_wd = WatchdogTimer(self.ch_heartbeat_period, self.alert_missing_ch)
        self.pat_health_wd = WatchdogTimer(self.pat_health_period, self.alert_missing_pat)

        self.fpga_answer_timer = None

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
        self.fpga_req_socket = self.context.socket(zmq.PUB)
        self.fpga_ans_socket = self.context.socket(zmq.SUB)
        self.tx_socket = self.context.socket(zmq.PUB)
        self.hk_control_socket = self.context.socket(zmq.SUB)
        self.ch_heartbeat_socket = self.context.socket(zmq.SUB)

        self.pat_health_socket.bind("tcp://127.0.0.1:%s" % PAT_HEALTH_PORT) #pat process is not already running
        
        self.fpga_req_socket.connect("tcp://127.0.0.1:%s" % FPGA_MAP_REQUEST_PORT)
        self.fpga_ans_socket.connect("tcp://127.0.0.1:%s" % FPGA_MAP_ANSWER_PORT)
        self.tx_socket.connect("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)
        self.hk_control_socket.connect("tcp://127.0.0.1:%s" % HK_CONTROL_PORT)
        self.ch_heartbeat_socket.connect("tcp://127.0.0.1:%s" % CH_HEARTBEAT_PORT)

        self.pat_health_socket.setsockopt(zmq.SUBSCRIBE, b'')
        self.fpga_ans_socket.setsockopt(zmq.SUBSCRIBE, str(self.pid).encode('ascii'))

        self.poller = zmq.Poller()

        self.poller.register(self.pat_health_socket, zmq.POLLIN)
        self.poller.register(self.hk_control_socket, zmq.POLLIN)
        self.poller.register(self.ch_heartbeat_socket, zmq.POLLIN)
        self.poller.register(self.fpga_ans_socket, zmq.POLLIN)

        # Initialize packet buffer
        self.packet_buf = []

        # Initialize counters
        self.fpga_req_count = 0
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

    def send_fpga_request(self):
        # TODO: Update this with properly-formatted FPGA telemetry request
        pkt = FPGAMapRequestPacket()
        tc_pkt = pkt.encode(return_addr=self.pid,
                            rq_number=self.fpga_req_count,
                            rw_flag=0,
                            start_addr=200,
                            size=19)

        if (self.fpga_req_enable):
            self.fpga_req_socket.send(tc_pkt)
            self.fpga_req_count += 1
            self.fpga_answer_timer = threading.Timer(self.fpga_check_period, self.alert_missing_fpga)

    def check_fpga(self, answer_pkt):
        pkt = []

        # 0: HK counter
        pkt.extend(struct.pack('B', self.fpga_hk_count % 256))

        # 1-N:
        # TODO: Format packet properly if necessary
        pkt.extend(answer_pkt)

    def check_sys(self):
        pkt = []

        # 0: HK counter
        pkt.extend(struct.pack('B', self.sys_hk_count % 256))

        # 1: Enable flags
        enables = 0
        if (self.all_pkts_send_enable):
            enables |= 1 << 7
        if (self.fpga_req_enable):
            enables |= 1 << 6
        if (self.sys_hk_send_enable):
            enables |= 1 << 5
        if (self.fpga_hk_send_enable):
            enables |= 1 << 4
        if (self.pat_hk_send_enable):
            enables |= 1 << 3
        if (self.ch_restart_enable):
            enables |= 1 << 2
        if (self.pat_restart_enable):
            enables |= 1 << 1
        if (self.fpga_restart_enable):
            enables |= 1 << 0
        pkt.extend(struct.pack('B', enables))

        # 2: FPGA housekeeping period
        pkt.extend(struct.pack('B', self.fpga_check_period))
        # 3: System housekeeping period
        pkt.extend(struct.pack('B', self.sys_check_period))
        # 4: Command handler heartbeat period
        pkt.extend(struct.pack('B', self.ch_heartbeat_period))
        # 5: PAT health period
        pkt.extend(struct.pack('B', self.pat_health_period))
        # 6: FPGA response period
        pkt.extend(struct.pack('B', self.fpga_ans_period))

        # 7: Acknowledged command count
        pkt.extend(struct.pack('B', self.ack_cmd_count))
        # 8: Last acknowledged command ID
        pkt.extend(struct.pack('B', self.last_ack_cmd_id))
        # 9: Error command count
        pkt.extend(struct.pack('B', self.err_cmd_count))
        # 10: Last error command ID
        pkt.extend(struct.pack('B', self.last_err_cmd_id))

        # 11-12: Boot count
        try:
            with open('/mnt/journal/id.txt', 'r') as boot_id_list:
                for count, l in enumerate(boot_id_list, 1):
                    pass
            pkt.extend(struct.pack('H', count))
        except:
            # Error if journal file can't be opened
            pkt.extend([0xFF, 0xFF])

        # 13-16: Disk usage
        # TODO: Update path if necessary
        disk = psutil.disk_usage('/')
        pkt.extend(struct.pack('>L', (disk.used % 2**32)))

        # 17-20: Disk free
        pkt.extend(struct.pack('>L', (disk.free % 2**32)))

        # 21-24: Available virtual memory
        vmem = psutil.virtual_memory()
        pkt.extend(struct.pack('>L', (vmem.available % 2**32)))

        # 25-N: Process info
        for p in psutil.process_iter(['pid','name','cpu_percent','memory_percent']):
            p_name = p.info['name']
            if p_name in self.procs:
                pkt.extend(struct.pack('B', self.procs[p_name]))
                pkt.extend(struct.pack('B', (p.cpu_percent() % 256)))
                pkt.extend(struct.pack('B', (p.memory_percent('rss') % 256)))

        #compile list of packed byte strings (pkt) into single packed byte string
        message = ''
        for i in range(len(pkt)):
            message += pkt[i]
        return message

    def handle_hk_pkt(self, data, process_id):
        # TODO: Update packet handling

        if (process_id == HK_PAT_ID):
            pat_pkt = PATHealthPacket()
            apid = TLM_HK_PAT
            payload, _, _, _ = pat_pkt.decode(data)
            #payload = struct.pack('%ds'%len(payload), payload) #for readability, could have this, though it doesn't do anything (packed string = original string)
            print('Handling PAT pkt w/ payload: ', payload)

        elif (process_id == HK_FPGA_ID):
            apid = TLM_HK_FPGA_MAP
            payload = struct.pack('B', data) #TBR if the data is already a packed byte string or not. FPGA packet definition beyond 1 byte HK counter is TBD
            print('Handling FPGA pkt w/ payload: ', payload)

        elif (process_id == HK_SYS_ID):
            apid = TLM_HK_SYS
            payload = data #data is already a packed byte string
            print('Handling SYS pkt w/ payload: ', payload)

        elif (process_id == HK_CH_ID):
            # TODO: Update command counters
            apid = TLM_HK_CH
            ch_pkt = CHHealthPacket()
            _, size, payload = ch_pkt.decode(data)
            #payload = struct.pack('%ds'%len(payload), payload) #for readability, could have this, though it doesn't do anything (packed string = original string)
            print('Handling CH pkt w/ payload: ', payload)

        pkt = TxPacket()
        raw_pkt = pkt.encode(apid, payload) #payload needs to be a single packed byte string e.g. '\x00\x01'
        self.packet_buf.append(raw_pkt)

    def restart_process(self, process_id):
        process_name = self.procs[process_id]
        # print("Restart "+process_name)
        if ((process_id == HK_CH_ID & self.ch_restart_enable) |
            (process_id == HK_PAT_ID & self.pat_restart_enable) |
            (process_id == HK_FPGA_ID & self.fpga_restart_enable)):
            status = subprocess.call("systemctl --user restart " + process_name)
            # TODO: Format and send the error packet

    def handle_hk_command(self, command):
        hk = HousekeepingControlPacket()
        cmd, payload = hk.decode(command)
        # TODO: Handle housekeeping command

    def run(self):
        self.fpga_check_timer.start()
        self.sys_check_timer.start()
        self.ch_heartbeat_wd.start()
        self.pat_health_wd.start()

        while True:
            # Periodically send FPGA request
            if (self.fpga_check_flag.is_set()):
                self.send_fpga_request()
                self.fpga_check_flag.clear()

            # Periodically generate system HK
            if (self.sys_check_flag.is_set()):
                message = self.check_sys()
                self.handle_hk_pkt(message, HK_SYS_ID)
                self.sys_check_flag.clear()

            if (self.missing_ch_flag.is_set()):
                # TODO: implement error-handling for not receiving command handler heartbeat
                self.restart_process(HK_CH_ID)
                self.missing_ch_flag.clear()

            if (self.missing_pat_flag.is_set()):
                # TODO: implement error handling for not receiving pat health packet
                self.restart_process(HK_PAT_ID)
                self.missing_pat_flag.clear()

            if (self.missing_fpga_flag.is_set()):
                # TODO: implement error handling for not receiving fpga response
                self.restart_process(HK_FPGA_ID)
                self.missing_fpga_flag.clear()

            # Receive packets from the other processes
            sockets = dict(self.poller.poll(500)) # 500 ms timeout

            if self.pat_health_socket in sockets and sockets[self.pat_health_socket] == zmq.POLLIN:
                message = self.pat_health_socket.recv()
                self.handle_hk_pkt(message, HK_PAT_ID)
                self.pat_health_wd.kick()

            elif self.ch_heartbeat_socket in sockets and sockets[self.ch_heartbeat_socket] == zmq.POLLIN:
                message = self.ch_heartbeat_socket.recv()
                # Check if heartbeat or health packet
                if (len(message) == 8):
                    # TODO: consider more than 1 command handler
                    ch_packet = HandlerHeartbeatPacket()
                    ch_packet.decode(message)
                    # TODO: implement something with timestamp
                    self.ch_heartbeat_wd.kick()
                else:
                    self.handle_hk_pkt(message, HK_CH_ID)

            elif self.fpga_ans_socket in sockets and sockets[self.fpga_ans_socket] == zmq.POLLIN:
                message = self.fpga_ans_socket.recv()
                pkt = self.check_fpga(message)
                self.handle_hk_pkt(pkt, HK_FPGA_ID)
                self.fpga_check_timer.cancel()

            elif self.hk_control_socket in sockets and sockets[self.hk_control_socket] == zmq.POLLIN:
                message = self.hk_control_socket.recv()
                self.handle_hk_command(message)

            # Send housekeeping packets to TX packets queue
            while self.packet_buf:
                current_pkt = self.packet_buf.pop(0)
                if (self.all_pkts_send_enable):
                    self.tx_socket.send(current_pkt)

            # print("HK is running")

if __name__ == '__main__':
    housekeeper = Housekeeping()
    housekeeper.run()
