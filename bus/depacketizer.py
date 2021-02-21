#!/usr/bin/env python
from multiprocessing import Process
from time import sleep

import sys
import os
import zmq
import struct

import binascii
from crccheck.crc import Crc16CcittFalse as crc16

sys.path.append('/root/CLICK-A-RPi/lib/')
sys.path.append('/root/lib/') #flight path
from ipc_packets import RxCommandPacket, RxPATPacket, TxPacket
from options import *
from zmqTxRx import push_zmq, send_zmq, recv_zmq

SPI_DEV = '/dev/bct'

CCSDS_HEADER_LEN = 6
APID_INDEX = 1
PKT_LEN_INDEX = 4

class Depacketizer:
    ccsds_sync = bytearray([0x35, 0x2E, 0xF8, 0x53])

    def __init__(self):

        self.context = zmq.Context()

        self.rx_cmd_socket = self.context.socket(zmq.PUB)
        self.rx_pat_socket = self.context.socket(zmq.PUB)
        self.tx_socket = self.context.socket(zmq.PUB)

        self.spi = open(SPI_DEV, 'rb', buffering=0)

        self.bus_pkts_buffer = []
        self.ipc_pkts_buffer = []

        # #ZMQ REQ worker socket for load balancing
        # ipc_client = ipc_loadbalancer.ClientInterface(context)
        self.rx_cmd_socket.bind("tcp://127.0.0.1:%s" % LOAD_BALANCER_PORT)
        self.rx_pat_socket.bind("tcp://127.0.0.1:%s" % RX_PAT_PACKETS_PORT)
        self.tx_socket.connect("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)

    def read_data(self, n_bytes):
        buf = []
        while len(buf) < n_bytes:
            try:
                data = self.spi.read(n_bytes - len(buf))
                if len(data) > 0:
                    buf.extend(bytearray(data))
            except Exception, err:
                print(str(err))
                pass

        return buf

    def acquire_bus_pkt(self):
        # returns a full bus packet without the sync marker

        buf = []
        # Read 1 byte from SPI device until the sync marker is found
        sync_index = 0
        while (sync_index < (len(self.ccsds_sync))):
            b = self.read_data(1)
            if(b[0] == self.ccsds_sync[sync_index]):
                # buf.append(b)
                sync_index += 1
            elif (b[0] == self.ccsds_sync[0]):
                # buf = [b]
                sync_index = 1
            else:
                sync_index = 0
        #print('found sync!')
        # Read 6 CCSDS header bytes
        buf = self.read_data(CCSDS_HEADER_LEN)

        apid =  buf[APID_INDEX]

        pkt_len = (buf[PKT_LEN_INDEX] << 8) | buf[PKT_LEN_INDEX + 1] + 1
        # Read payload data bytes and crc bytes
        pkt = self.read_data(pkt_len)
        # Assuming crc is included in the packet length

        buf.extend(pkt)

        # Check crc
        crc_index = CCSDS_HEADER_LEN + pkt_len - 2
        crc = (buf[crc_index] << 8) | buf[crc_index + 1]

        #Calculate CRC over the entire packet
        crcinst = crc16()
        crc_check = crc16.calc(buf[:crc_index])

        if (crc == crc_check):
            self.bus_pkts_buffer.append(buf)
        else:
            print('crc did not work')
            err_pkt = TxPacket()
            err_pkt_pl = struct.pack('!H%dB' % len(buf), len(buf), *buf)
            raw_err_pkt = err_pkt.encode(ERR_DPKT_CRC_INVALID, err_pkt_pl)
            self.tx_socket.send(raw_err_pkt)

    def handle_bus_pkts(self):

        try:
            pkt = self.bus_pkts_buffer[0]
        except IndexError:
            # Empty buffer, but that's ok
            return
        # Check if pkt is part of a sequence
        # 0b00 - continuation, 0b01 - first of group, 0b10 - last of group, 0b11 - standalone
        seq_flag = (pkt[2] & 0b11000000) >> 6

        seq_cnt = ((pkt[2] & 0b00111111) << 8) | pkt[3] # should probably maintain a counter and check this

        if (seq_flag == 0b11):
            # pkt is standalone
            apid = ((pkt[0] & 0b00000011) << 8) | pkt[1]
            ts_sec = (((((pkt[6] << 8) | pkt[7]) << 8) | pkt[8]) << 8) | pkt[9]
            ts_subsec = pkt[10]
            # this is the number of bytes after the primary ccsds header
            pkt_len = (pkt[4] << 8) | pkt[5] + 1
            # pkt_len minus secondary header len (6) and crc (2)
            data_len = pkt_len - 8
            data = pkt[12:(12+data_len)]

            # make Ipc packet, add to outgoing buffer
            ipc_pkt = RxCommandPacket()

            raw_ipc_pkt = ipc_pkt.encode(apid, ts_sec, ts_subsec, data)
            self.ipc_pkts_buffer.append(raw_ipc_pkt)
            self.bus_pkts_buffer.pop(0)

        elif (seq_flag == 0b01):
            # pkt is first of sequence

            # Check that the entire sequence is received
            completed = False
            for i in range(len(self.bus_pkts_buffer)):
                ipkt = self.bus_pkts_buffer[i]
                seq_flag = (ipkt[2] & 0b11000000) >> 6
                if (seq_flag == 0b01):
                    completed = True
            if (completed is True):
                # Entire sequence is received

                # Get first packet
                pkt = self.bus_pkts_buffer.pop(0)
                apid = ((pkt[0] & 0b00000011) << 8) | pkt[1]
                ts_sec = (((((pkt[6] << 8) | pkt[7]) << 8) | pkt[8]) << 8) | pkt[9]
                ts_subsec = pkt[10]
                pkt_len = (pkt[4] << 8) | pkt[5] + 1
                data_len = pkt_len - 8
                data = pkt[12:(12+data_len)]

                # Get continuation packets
                pkt = self.bus_pkts_buffer.pop(0)
                seq_flag = (pkt[2] & 0b11000000) >> 6
                while(seq_flag == 0b00):
                    pkt_len = (pkt[4] << 8) | pkt[5] + 1
                    data_len = pkt_len - 8
                    data.extend(pkt[12:(12+data_len)])

                    pkt = self.bus_pkts_buffer.pop(0)
                    seq_flag = (pkt[2] & 0b11000000) >> 6
                # get last completed packet
                pkt_len = (pkt[4] << 8) | pkt[5] + 1
                data_len = pkt_len - 8
                data.extend(pkt[12:(12+data_len)])

                # make Ipc packet, add to outgoing buffer
                ipc_pkt = RxCommandPacket()

                raw_ipc_pkt = ipc_pkt.encode(apid, ts_sec, ts_subsec, data)

                self.ipc_pkts_buffer.append(raw_ipc_pkt)
                del self.bus_pkts_buffer[:seq_cnt]
        else:
            # pkt is not standalone or first of sequence
            self.bus_pkts_buffer.pop(0) # Just remove it

        return

    def send_ipc_pkts(self):
        try:
            raw_ipc_pkt = self.ipc_pkts_buffer.pop(0)
        except IndexError as e:
            # Empty buffer, but that's ok
            return

        ipc_pkt = RxCommandPacket()
        APID,_,_,_ = ipc_pkt.decode(raw_ipc_pkt)
        if(APID != APID_TIME_AT_TONE):
            #don't print the time at tone packets
            print(binascii.hexlify(raw_ipc_pkt))

        # in the future, consider the pat packets and send those separately
        self.rx_cmd_socket.send(raw_ipc_pkt)
        # #send payload to idle worker (this will wait until a worker becomes idle)
        # ipc_client.send_request(raw)

    def send_noop(self):
        print("Depacketizer No-op test")
        while True:
            apid = CMD_PL_NOOP
            ts_sec = 0
            ts_subsec = 0
            data = []
            ipc_pkt = RxCommandPacket()
            raw_ipc_pkt = ipc_pkt.encode(apid, ts_sec, ts_subsec, data)
            self.ipc_pkts_buffer.append(raw_ipc_pkt)
            self.send_ipc_pkts()
            sleep(5)

    def run(self):
        print("Start Depacketizer")
        while True:

            self.acquire_bus_pkt() # BLOCKS

            # Parse packet from buffer
            self.handle_bus_pkts()
            self.send_ipc_pkts()


    def run_test(self):
        print("Start depacketizer test")

        sync_index = 0
        while (sync_index < (len(self.ccsds_sync))):
            b = self.read_data(1)
            if(b[0] == self.ccsds_sync[sync_index]):
                # buf.append(b)
                sync_index += 1
            elif (b[0] == self.ccsds_sync[0]):
                # buf = [b]
                sync_index = 1
            else:
                sync_index = 0
        print('found sync!')
        # Read 6 CCSDS header bytes
        buf = self.read_data(CCSDS_HEADER_LEN)
        pkt_len = (buf[PKT_LEN_INDEX] << 8) | buf[PKT_LEN_INDEX + 1] + 1
        # Read payload data bytes and crc bytes
        pkt = self.read_data(pkt_len)
        buf.extend(pkt)

        print(binascii.hexlify(buf))

        while(True):
            b = self.read_data(103)
            print(binascii.hexlify(b))


if __name__ == '__main__':
    depacketizer = Depacketizer()
    #depacketizer.run()
    depacketizer.send_noop()
