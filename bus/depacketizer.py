from multiprocessing import Process
from time import sleep

import sys
import os
import zmq

from crccheck.crc import Crc16Ccitt as crc16

sys.path.append('/home/pi/CLICK-A-RPi/lib/')
from ipc_packets import RxCommandPacket, RxPATPacket
from options import RX_CMD_PACKETS_PORT, RX_PAT_PACKETS_PORT
from zmqTxRx import push_zmq, send_zmq, recv_zmq

SPI_DEV = '/dev/bct'

CCSDS_HEADER_LEN = 6
PKT_LEN_INDEX = 4

class Depacketizer:
    context = zmq.Context()

    rx_cmd_socket = context.socket(zmq.PUB)
    rx_pat_socket = context.socket(zmq.PUB)

    #spi = spidev.SpiDev()
    # spi = open(SPI_DEV, os.O_RDWR)
    spi = open(SPI_DEV, 'r')

    bus_pkts_buffer = []
    ipc_pkts_buffer = []

    def __init__(self):
        self.rx_cmd_socket.bind("tcp://127.0.0.1:%s" % RX_CMD_PACKETS_PORT)
        self.rx_pat_socket.bind("tcp://127.0.0.1:%s" % RX_PAT_PACKETS_PORT)

    def acquire_bus_pkt(self):
        # returns a full bus packet without the sync marker
        ccsds_sync = [0x35, 0x2E, 0xF8, 0x53]
        buf = []
        # Read 1 byte from SPI device until the sync marker is found
        sync_index = 0
        while (sync_index < (len(ccsds_sync))):
            b = self.spi.read(1)
            if(b == ccsds_sync[sync_index]):
                # buf.append(b)
                sync_index += 1
            elif (b == ccsds_sync[0]):
                # buf = [b]
                sync_index = 1
            else:
                sync_index = 0

        # Read 6 CCSDS header bytes
        buf = self.spi.read(CCSDS_HEADER_LEN)
        pkt_len = (buf[PKT_LEN_INDEX] << 8) | buf[PKT_LEN_INDEX + 1] + 1

        # Read payload data bytes and crc bytes
        pkt = self.spi.read(pkt_len)
        # Assuming crc is included in the packet length

        buf.extend(pkt)

        # Check crc
        crc_index = CCSDS_HEADER_LEN + pkt_len - 2
        crc = (pkt[crc_index] << 8) | pkt[crc_index + 1]

        crc_check = crc16.calc(buf[:crc_index])
        #confirm CRC calculated over entire packet

        if (crc == crc_check):
            return buf
        else:
            return None

    def handle_bus_pkts(self):
        try:
            pkt = self.bus_pkts_buffer[0]
        except IndexError as e:
            # Empty buffer
            return

        # Check if pkt is part of a sequence
        # 0b00 - continuation, 0b01 - first of group, 0b10 - last of group, 0b11 - standalone
        seq_flag = (pkt[2] & 0b11000000) >> 4

        # Assuming that packets are received in the correct sequence...
        if (seq_flag == 0b11 or seq_flag == 0b01):
            # pkt is standalone or first of sequence
            seq_cnt = ((pkt[2] & 0b00111111) << 8) | pkt[3]

            if (seq_flag == 0b01):
                # Check that the entire sequence is received
                completed = False

                for i in range(len(self.bus_pkts_buffer)):
                    ipkt = self.bus_pkts_buffer[i]
                    seq_flag = (ipkt[2] & 0b11000000) >> 4
                    if (seq_flag == 0b01):
                        completed = True

            if (completed is True):
                # Entire sequence is received

                apid = ((pkt[0] & 0b00000011) << 8) | pkt[1]

                ts_sec = (((((pkt[6] << 8) | pkt[7]) << 8) | pkt[8]) << 8) | pkt[9]
                ts_subsec = pkt[10] * 200

                data_len = ((pkt[19] << 8) | pkt[20])

                data = pkt[21:(21+data_len)]

                for i in range(1, seq_cnt):
                    pkt = bus_pkts_buffer[i]
                    data_len += ((pkt[19] << 8) | pkt[20])
                    data.extend(pkt[21:(21+data_len)])

                # make Ipc packet, add to outgoing buffer
                if (apid == 0x01):
                    ipc_pkt = RxCommmandPacket()
                else: # assume this is PAT idk
                    ipc_pkt = RxPatPacket()

                raw_ipc_pkt = ipc_pkt.encode(apid, ts_sec, ts_subsec, data)

                self.ipc_pkts_buffer.append(raw_ipc_pkt)

                del self.bus_pkts_buffer[:seq_cnt]

            #else: # End of sequence not received yet

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
        APID,_ = ipc_pkt.decode(raw_ipc_pkt)
        if (APID == 0x01):
            self.rx_cmd_socket.send(raw_ipc_pkt)
        else:
            self.rx_pat_socket.send(raw_ipc_pkt)

    def run(self):
        print("Start Depacketizer")
        while True:

            bus_packet = self.acquire_bus_pkt() # BLOCKS

            print(bus_packet)
            self.bus_pkts_buffer.append(bus_packet)

            # Parse packet from buffer
            self.handle_bus_pkts()
            self.send_ipc_pkts()




if __name__ == '__main__':
    depacketizer = Depacketizer()
    depacketizer.run()
