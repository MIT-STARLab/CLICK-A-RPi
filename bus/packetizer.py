from multiprocessing import Process
from time import sleep

import sys
import os
import zmq
import spidev

from crccheck.crc import Crc16Ccitt as crc16

sys.path.append('/home/pi/CLICK-A-RPi/lib/')
from ipc_packets import TxPacket
from options import TX_PACKETS_PORT
from zmqTxRx import push_zmq, send_zmq, recv_zmq

SPI_DEV = '/dev/bct'

BUS_RX_DATA_LEN = 512
BUS_TX_DATA_LEN = 4100 - 12 # from bus interface doc

SPI_XFER_LEN = 105

class Packetizer:
    context = zmq.Context()

    tx_socket = context.socket(zmq.SUB)

    #spi = spidev.SpiDev()
    # spi = open(SPI_DEV, os.O_RDWR | os.O_CREAT)
    spi = open(SPI_DEV, 'w')

    tx_ipc_pkts_buffer = []
    bus_tx_pkts_buffer = []

    def __init__(self):

        self.tx_socket.bind("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)
        self.tx_socket.subscribe("")


    def bus_parse_bytes(self):
        ccsds_sync = [0x35, 0x2E, 0xF8, 0x53]

        if (len(self.bus_rx_bytes_buffer) == 0):
            # Nothing received, nothing to do
            return

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

        # If this needs a topic do it here
        if (ipc_pkt.APID == 0x01):
            self.rx_cmd_socket.send(ipc_pkt.encode())
        else:
            self.rx_pat_socket.send(ipc_pkt.encode())

    def recv_ipc_pkts(self):
        try:
            received = self.tx_socket.recv(flags=zmq.NOBLOCK)
            ipc_pkt = TxPacket()
            ipc_pkt.decode(received)
            self.tx_ipc_pkts_buffer.append(ipc_pkt)
        except:
            # Nothing received, that's ok
            pass

    def tx_parse_pkts(self):
        try:
            ipc_pkt = self.tx_ipc_pkts_buffer.pop(0)
        except IndexError as e:
            # Empty buffer, but that's ok
            return

        pkt_data = ipc_pkt.payload
        # 0b00 - continuation, 0b01 - first of group, 0b10 - last of group, 0b11 - standalone
        seq_cnt = 0;
        seq_flag = 0b01
        while (len(pkt_data) > (BUS_TX_DATA_LEN)):
            bus_tx_pkt = []
            bus_tx_pkt[0] = ((ipc_pkt.APID >> 8) & 0b00000111)
            bus_tx_pkt[1] = (ipc_pkt.APID & 0xFF)
            bus_tx_pkt[2] = (seq_flag << 6) | ((seq_cnt >> 8) & 0b00111111)
            bus_tx_pkt[3] = (seq_cnt & 0xFF)
            bus_tx_pkt[4] = ((BUS_TX_DATA_LEN - 1) >> 8) & 0xFF
            bus_tx_pkt[5] = (BUS_TX_DATA_LEN -1) & 0xFF

            bus_tx_pkt.extend(pkt_data[:BUS_TX_DATA_LEN])
            crc = crc16.calc(bus_tx_pkt)
            bus_tx_pkt.extend(crc)
            self.bus_tx_pkts_buffer.append(bus_tx_pkt)
            del pkt_data[:BUS_TX_DATA_LEN]

            seq_cnt += 1
            seq_flag = 0b00

        #Last or only packet
        if (seq_flag == 0b01):
            seq_flag = 0b11
        else:
            seq_flag = 0b10

        bus_tx_pkt = []
        bus_tx_pkt[0] = ((ipc_pkt.APID >> 8) & 0b00000111)
        bus_tx_pkt[1] = (ipc_pkt.APID & 0xFF)
        bus_tx_pkt[2] = (seq_flag << 6) | ((seq_cnt >> 8) & 0b00111111)
        bus_tx_pkt[3] = (seq_cnt & 0xFF)
        bus_tx_pkt[4] = ((len(pkt_data) - 1) >> 8) & 0xFF
        bus_tx_pkt[5] = (len(pkt_data) - 1) & 0xFF

        bus_tx_pkt.extend(pkt_data)
        crc = crc16.calc(bus_tx_pkt)
        bus_tx_pkt.extend(crc)
        self.bus_tx_pkts_buffer.append(bus_tx_pkt)

    def run(self):

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

        while True:

            # Check for data from bus
            # raw_bus_data = [0x35, 0x2E, 0xF8, 0x53, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C]
            #raw_bus_data = self.spi.readbytes(BUS_RX_DATA_LEN)
            # Add bus data to raw buffer
            #self.bus_rx_bytes_buffer.extend(raw_bus_data)

            raw_bus_data = self.spi.read(SPI_XFER_LEN) #blocks
            # raw_bus_data = os.read(self.spi, BUS_RX_DATA_LEN) #blocks
            print(raw_bus_data)
            # self.bus_rx_bytes_buffer.extend(raw_bus_data)
            #
            # # Parse packet from buffer
            # self.bus_parse_bytes()
            # self.bus_parse_pkts()
            # self.send_ipc_pkts()
            #
            # self.recv_ipc_pkts()
            #
            # self.tx_parse_pkts()

            self.spi.write(bytearray(tx))
            # os.write(self.spi, tx)



if __name__ == '__main__':
    bus_interface = BusInterface()
    bus_interface.run()
