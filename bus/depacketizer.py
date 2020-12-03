from multiprocessing import Process
from time import sleep

import sys
import os
import zmq
import spidev

from crccheck.crc import Crc16Ccitt as crc16

sys.path.append('/home/pi/CLICK-A-RPi/lib/')
from ipc_packets import RxCommandPacket, RxPATPacket
from options import RX_CMD_PACKETS_PORT, RX_PAT_PACKETS_PORT
from zmqTxRx import push_zmq, send_zmq, recv_zmq

SPI_DEV = '/dev/bct'

BUS_RX_DATA_LEN = 512
BUS_TX_DATA_LEN = 4100 - 12 # from bus interface doc

SPI_XFER_LEN = 20

class Depacketizer:
    context = zmq.Context()

    rx_cmd_socket = context.socket(zmq.PUB)
    rx_pat_socket = context.socket(zmq.PUB)

    #spi = spidev.SpiDev()
    # spi = open(SPI_DEV, os.O_RDWR | os.O_CREAT)
    spi = open(SPI_DEV, 'r')

    bus_rx_bytes_buffer = []
    bus_rx_pkts_buffer = []
    rx_ipc_pkts_buffer = []

    def __init__(self):
        self.rx_cmd_socket.bind("tcp://127.0.0.1:%s" % RX_CMD_PACKETS_PORT)
        self.rx_pat_socket.bind("tcp://127.0.0.1:%s" % RX_PAT_PACKETS_PORT)

    def bus_parse_bytes(self):
        ccsds_sync = [0x35, 0x2E, 0xF8, 0x53]

        # Find sync marker
        b = 0
        for b in range(len(self.bus_rx_bytes_buffer)-len(ccsds_sync)+1):
            if (self.bus_rx_bytes_buffer[b:b+4] == ccsds_sync):
                break

        start_index = b+len(ccsds_sync)
        pkt_len_index = start_index + 4
        if (pkt_len_index + 1 < len(self.bus_rx_bytes_buffer)):
            # What if entire packet hasn't been received yet?
            return

        pkt_len = (self.bus_rx_bytes_buffer[pkt_len_index] << 8) | self.bus_rx_bytes_buffer[pkt_len_index + 1] + 1

        crc_index = start_index + 6 + pkt_len - 2
        if (crc_index + 1 < len(self.bus_rx_bytes_buffer)):
            # What if entire packet hasn't been received yet?
            return

        # Check CRC
        crc = (self.bus_rx_bytes_buffer[crc_index] << 8) | self.bus_rx_bytes_buffer[crc + 1]

        crc_check = crc16.calc(self.bus_rx_bytes_buffer[start_index:crc_index]) #confirm CRC calculated over entire packet

        if (crc == crc_check):
            self.bus_rx_pkts_buffer.append(self.bus_rx_bytes_buffer[start_index:(crc_index+2)])

        del self.bus_rx_bytes_buffer[:(crc_index+2)]

        return

    def acquire_bus_pkt(self):
        ccsds_sync = [0x35, 0x2E, 0xF8, 0x53]
        bytes_buffer = []

        # Read 1 byte from SPI device until start of sync marker is found
        b = self.spi.read(1))
        while (b != ccsds_sync[0]):
            b = self.spi.read(1)


        raw_bus_data = self.spi.read(SPI_XFER_LEN)

        b = 0
        for b in range(len(self.bus_rx_bytes_buffer)-len(ccsds_sync)+1):
            if (self.bus_rx_bytes_buffer[b:b+4] == ccsds_sync):
                break

        start_index = b+len(ccsds_sync)
        pkt_len_index = start_index + 4
        if (pkt_len_index + 1 < len(self.bus_rx_bytes_buffer)):
            # What if entire packet hasn't been received yet?
            return

        pkt_len = (self.bus_rx_bytes_buffer[pkt_len_index] << 8) | self.bus_rx_bytes_buffer[pkt_len_index + 1] + 1

        crc_index = start_index + 6 + pkt_len - 2
        if (crc_index + 1 < len(self.bus_rx_bytes_buffer)):
            # What if entire packet hasn't been received yet?
            return

        # Check CRC
        crc = (self.bus_rx_bytes_buffer[crc_index] << 8) | self.bus_rx_bytes_buffer[crc + 1]

        crc_check = crc16.calc(self.bus_rx_bytes_buffer[start_index:crc_index]) #confirm CRC calculated over entire packet

        if (crc == crc_check):
            self.bus_rx_pkts_buffer.append(self.bus_rx_bytes_buffer[start_index:(crc_index+2)])

        del self.bus_rx_bytes_buffer[:(crc_index+2)]

        return

    def bus_parse_pkts(self):
        pkt = self.bus_rx_pkts_buffer[0]
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
                for i in range(len(self.bus_rx_pkts_buffer)):
                    ipkt = self.bus_rx_pkts_buffer[i]
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
                    pkt = bus_rx_pkts_buffer[i]
                    data_len += ((pkt[19] << 8) | pkt[20])
                    data.extend(pkt[21:(21+data_len)])

                # make Ipc packet, add to outgoing buffer
                if (apid == 0x01):
                    ipc_pkt = RxCommmandPacket()
                else: # assume this is PAT idk
                    ipc_pkt = RxPatPacket()

                ipc_pkt.encode(apid, ts_sec, ts_subsec, data)

                self.rx_ipc_pkts_buffer.append(ipc_pkt)

                del self.bus_rx_pkts_buffer[:seq_cnt]

            #else: # End of sequence not received yet

        else:
            # pkt is not standalone or first of sequence
            self.bus_rx_pkts_buffer.pop(0) # Just remove it

        return

    def send_ipc_pkts(self):

        ipc_pkt = self.rx_ipc_pkts_buffer.pop(0)
        if (ipc_pkt.APID == 0x01):
            self.rx_cmd_socket.send(ipc_pkt.encode())
        else:
            self.rx_pat_socket.send(ipc_pkt.encode())

    def run(self):

        while True:

            bus_packet
            raw_bus_data = self.spi.read(SPI_XFER_LEN) #blocks

            print(raw_bus_data)

            # self.bus_rx_bytes_buffer.extend(raw_bus_data)
            #
            # # Parse packet from buffer
            # self.bus_parse_bytes()
            # self.bus_parse_pkts()
            # self.send_ipc_pkts()




if __name__ == '__main__':
    depacketizer = Depacketizer()
    depacketizer.run()
