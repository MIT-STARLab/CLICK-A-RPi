from multiprocessing import Process
from time import sleep

import sys
import os
import zmq
import spidev

from crccheck.crc import Crc16Ccitt as crc16

import ipc_packets

TX_PACKETS_PORT = "5561"
RX_CMD_PACKETS_PORT = "5562"
RX_PAT_PACKETS_PORT = "5563"

SPI_BUS = 0
SPI_DEV = 0
SPI_HZ = 12000000

VNC_ADDR = 0b000

BUS_RX_DATA_LEN = 512
BUS_TX_DATA_LEN = 4100 - 12 # from bus interface doc

class BusInterface:
    context = zmq.Context()

    rx_cmd_socket = context.socket(zmq.PUB)
    rx_pat_socket = context.socket(zmq.PUB)
    tx_socket = context.socket(zmq.SUB)

    spi = spidev.SpiDev()
    spi.max_speed_hz = SPI_HZ

    bus_rx_bytes_buffer = []
    bus_rx_pkts_buffer = []
    rx_ipc_pkts_buffer = []
    tx_ipc_pkts_buffer = []
    bus_tx_pkts_buffer = []

    def __init__(self):

        self.rx_cmd_socket.bind("tcp://127.0.0.1:%s" % RX_CMD_PACKETS_PORT)
        self.rx_pat_socket.bind("tcp://127.0.0.1:%s" % RX_PAT_PACKETS_PORT)
        self.tx_socket.bind("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)
        self.tx_socket.subscribe("")

        self.spi.open(SPI_BUS, SPI_DEV)


    def spi_read_duplex(self):
        read_cmd = (((VNC_ADDR << 1) + 0b1) << 4) + 0b1111
        
        read_req = bytearray(BUS_RX_DATA_LEN *2)
        read_req[0] = read_cmd
        
        response = self.spi.xfer2(list(read_req))
        # should check if response is None

        ack = (response[0] >> 1) & 1
        # rxf = (response[0] >> 2) & 1
        # txe = (response[0] >> 3) & 1

        if (ack == 0):
            # Slave incorrectly decoded its address
            return None

        i = 0
        raw_data = []
        while (i < BUS_RX_DATA_LEN ):
            #ack = (response[i*2] >> 1) & 1
            #rxf = (response[i*2] >> 2) & 1
            txe = (response[i*2] >> 3) & 1

            if (txe == 1):
                break

            raw_data[i] = response[i*2 + 1]
            i += 1
        return raw_data


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
        assert pkt_len_index + 1 < len(self.bus_rx_bytes_buffer)
            # What if entire packet hasn't been received yet?

        pkt_len = (self.bus_rx_bytes_buffer[pkt_len_index] << 8) | self.bus_rx_bytes_buffer[pkt_len_index + 1] + 1

        crc_index = start_index + 6 + pkt_len - 2 
        assert crc_index + 1 < len(self.bus_rx_bytes_buffer)
            # What if entire packet hasn't been received yet?
        
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

            if (seq_cnt < len(self.bus_rx_pkts_buffer)):
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
        ipc_pkt = self.tx_ipc_pkts_buffer.pop(0)
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

        while True:

            # Check for data from bus
            # raw_bus_data = [0x35, 0x2E, 0xF8, 0x53, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C]
            raw_bus_data = self.spi.readbytes(BUS_RX_DATA_LEN)
            assert raw_bus_data is not None
            # Add bus data to raw buffer
            self.bus_rx_bytes_buffer.extend(raw_bus_data)
            # Parse packet from buffer
            self.bus_parse_bytes()

            self.bus_parse_pkts()

            self.send_ipc_pkts()

            self.recv_ipc_pkts() 

            self.tx_parse_pkts()

            self.spi.writebytes2(bus_tx_pkts_buffer.pop(0))


if __name__ == '__main__':
    bus_interface = BusInterface()
    bus_interface.run()






