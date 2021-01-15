from multiprocessing import Process
from time import sleep

import sys
import os
import zmq

from crccheck.crc import Crc16CcittFalse as crc16

sys.path.append('/root/lib/')
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
    spi = open(SPI_DEV, 'wb', buffering=0)

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
        tx[6] = 0b11000000 #sequence count is 0
        tx[7] = 0x00 #sequence count is 0
        tx[8] = (packet_len >> 8) & 0xFF
        tx[9] =  packet_len & 0xFF
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
        apid, pkt_data = ipc_pkt.decode(raw_ipc_pkt)

        # 0b00 - continuation, 0b01 - first of group, 0b10 - last of group, 0b11 - standalone
        seq_cnt = 0
        seq_flag = 0b01
        while (len(pkt_data) > (BUS_DATA_LEN)):
            bus_tx_pkt = []
            bus_tx_pkt.append(0x35)
            bus_tx_pkt.append(0x2E)
            bus_tx_pkt.append(0xF8)
            bus_tx_pkt.append(0x53)
            bus_tx_pkt.append((apid >> 8) & 0b00000111)
            bus_tx_pkt.append(apid & 0xFF)
            bus_tx_pkt.append((seq_flag << 6) | ((seq_cnt >> 8) & 0b00111111))
            bus_tx_pkt.append(seq_cnt & 0xFF)
            bus_tx_pkt.append(((BUS_DATA_LEN - 1) >> 8) & 0xFF)
            bus_tx_pkt.append((BUS_DATA_LEN -1) & 0xFF)

            bus_tx_pkt.extend(bytearray(pkt_data[:BUS_DATA_LEN]))
            crc = crc16.calc(bus_tx_pkt)
            bus_tx_pkt.extend([crc >> 8, crc & 0xFF])


            self.bus_pkts_buffer.append(bus_tx_pkt)

            del pkt_data[:BUS_DATA_LEN]

            seq_cnt += 1
            seq_flag = 0b00

        #Last or only packet
        if (seq_flag == 0b01):
            seq_flag = 0b11
        else:
            seq_flag = 0b10

        ###Start Revised Packet Definition - Tested and Works###
        sync = []
        sync.append(0x35)
        sync.append(0x2E)
        sync.append(0xF8)
        sync.append(0x53)

        pkt = []
        pkt.append((apid >> 8) & 0b00000111)
        pkt.append(apid & 0xFF)
        pkt.append((seq_flag << 6) | ((seq_cnt >> 8) & 0b00111111))
        pkt.append(seq_cnt & 0xFF)
        pkt.append(((len(pkt_data) + 1) >> 8) & 0xFF) #include length of CRC = 2
        pkt.append((len(pkt_data) + 1) & 0xFF) #include length of CRC = 2
        pkt.extend(bytearray(pkt_data[:BUS_DATA_LEN]))
        crc = crc16.calc(pkt) #do not include sync bytes in this calculation (hence the separation of sync and pkt)
        pkt.extend([crc >> 8, crc & 0xFF])

        bus_tx_pkt = []
        bus_tx_pkt.extend(sync)
        bus_tx_pkt.extend(pkt)
        print('bus_tx_pkt: ', bus_tx_pkt) #for debug
        ###End Revised Packet Definition###

        self.bus_pkts_buffer.append(bus_tx_pkt)

    def send_bus_pkts(self):
        try:
            raw_bus_pkt = self.bus_pkts_buffer.pop(0)
        except IndexError as e:
            # Empty buffer, but that's ok
            return

        self.spi.write(bytearray(raw_bus_pkt))

    def run(self):
        print("Start Packetizer")
        while True:

            ipc_pkt = self.tx_socket.recv() # BLOCKS

            print(ipc_pkt)
            self.ipc_pkts_buffer.append(ipc_pkt)

            self.handle_tx_pkts()
            self.send_bus_pkts()



if __name__ == '__main__':
    bus_interface = Packetizer()
    bus_interface.run()
