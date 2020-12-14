from multiprocessing import Process
from time import sleep

import sys
import os
import zmq

from crccheck.crc import Crc16Ccitt as crc16

sys.path.append('/home/pi/CLICK-A-RPi/lib/')
from ipc_packets import TxPacket
from options import TX_PACKETS_PORT
from zmqTxRx import push_zmq, send_zmq, recv_zmq

SPI_DEV = '/dev/bct'

BUS_DATA_LEN = 4100 - 12 # from bus interface doc

SPI_XFER_LEN = 105

class Packetizer:
    context = zmq.Context()

    tx_socket = context.socket(zmq.SUB)

    #spi = spidev.SpiDev()
    # spi = open(SPI_DEV, os.O_RDWR)
    spi = open(SPI_DEV, 'w')

    bus_pkts_buffer = []
    ipc_pkts_buffer = []


    def __init__(self):

        self.tx_socket.bind("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)
        self.tx_socket.subscribe("")

    def send_test_pkt(self):
        tx = []
        tx = [0 for i in range(SPI_XFER_LEN)]
        apid = 0x250
        packet_len = SPI_XFER_LEN - 11
        tx[0] = 0x35
        tx[1] = 0x2E
        tx[2] = 0xF8
        tx[3] = 0x53
        tx[4] = (apid >> 8) & 0xFF
        tx[5] = (apid & 0xFF)
        tx[6] = 0b11000000; #sequence count is 0
        tx[7] = 0x00; #sequence count is 0
        tx[8] = (packet_len >> 8) & 0xFF;
        tx[9] =  packet_len & 0xFF;
        i = 10
        while ( i < SPI_XFER_LEN-12):
          tx[i] = i % 0xFF
          i += 1
        crc = crc16.calc(tx[:SPI_XFER_LEN-2])
        tx[SPI_XFER_LEN-2] = (crc >> 8) & 0xFF
        tx[SPI_XFER_LEN-1] =  crc & 0xFF

        self.spi.write(bytearray(tx))

    def handle_tx_pkts(self):
        try:
            raw_ipc_pkt = self.ipc_pkts_buffer.pop(0)
        except IndexError as e:
            # Empty buffer, but that's ok
            return

        ipc_pkt = TxPacket()
        apid, pkt_data = ipc_pkt.decode(raw_bus_data)

        # 0b00 - continuation, 0b01 - first of group, 0b10 - last of group, 0b11 - standalone
        seq_cnt = 0;
        seq_flag = 0b01
        while (len(pkt_data) > (BUS_TX_DATA_LEN)):
            bus_tx_pkt = []
            bus_tx_pkt[0] = ((apid >> 8) & 0b00000111)
            bus_tx_pkt[1] = (apid & 0xFF)
            bus_tx_pkt[2] = (seq_flag << 6) | ((seq_cnt >> 8) & 0b00111111)
            bus_tx_pkt[3] = (seq_cnt & 0xFF)
            bus_tx_pkt[4] = ((BUS_DATA_LEN - 1) >> 8) & 0xFF
            bus_tx_pkt[5] = (BUS_DATA_LEN -1) & 0xFF

            bus_tx_pkt.extend(pkt_data[:BUS_DATA_LEN])
            crc = crc16.calc(bus_tx_pkt)
            bus_tx_pkt.extend(crc)

            self.bus_pkts_buffer.append(bus_tx_pkt)

            del pkt_data[:BUS_DATA_LEN]

            seq_cnt += 1
            seq_flag = 0b00

        #Last or only packet
        if (seq_flag == 0b01):
            seq_flag = 0b11
        else:
            seq_flag = 0b10

        bus_tx_pkt = []
        bus_tx_pkt[0] = ((apid >> 8) & 0b00000111)
        bus_tx_pkt[1] = (apid & 0xFF)
        bus_tx_pkt[2] = (seq_flag << 6) | ((seq_cnt >> 8) & 0b00111111)
        bus_tx_pkt[3] = (seq_cnt & 0xFF)
        bus_tx_pkt[4] = ((len(pkt_data) - 1) >> 8) & 0xFF
        bus_tx_pkt[5] = (len(pkt_data) - 1) & 0xFF

        bus_tx_pkt.extend(pkt_data)
        crc = crc16.calc(bus_tx_pkt)
        bus_tx_pkt.extend(crc)

        self.bus_pkts_buffer.append(bus_tx_pkt)

    def send_bus_pkts(self):
        try:
            raw_bus_pkt = self.bus_pkts_buffer.pop(0)
        except IndexError as e:
            # Empty buffer, but that's ok
            return

        self.spi.write(raw_bus_pkt)

    def run(self):
        print("Start Packetizer")
        while True:

            ipc_pkt = self.tx_socket.recv() # BLOCKS

            print(ipc_pkt)
            self.ipc_pkts_buffer.append(ipc_pkt)

            self.handle_ipc_pkts()
            self.send_bus_pkts()



if __name__ == '__main__':
    bus_interface = Packetizer()
    bus_interface.run()
