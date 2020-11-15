from multiprocessing import Process
from time import sleep

import sys
import os
import zmq
import spidev

TX_PACKETS_PORT = "5561"
RX_CMD_PACKETS_PORT = "5562"
RX_PAT_PACKETS_PORT = "5563"

SPI_BUS = 0
SPI_DEV = 0
SPI_HZ = 12000000

VNC_ADDR = 0b000
#BUS_PKT_HDR_LEN = 21
#BUS_PKT_DATA_LEN = 5
BUS_DATA_LEN = 512


class BusInterface:
    context = zmq.Context()

    rx_cmd_socket = context.socket(zmq.PUB)
    rx_pat_socket = context.socket(zmq.PUB)
    tx_socket = context.socket(zmq.SUB)

    spi = spidev.SpiDev()
    spi.max_speed_hz = SPI_HZ

    bus_bytes_buffer = []
 

    def __init__(self):

        self.rx_cmd_socket.bind("tcp://127.0.0.1:%s" % RX_CMD_PACKETS_PORT)
        self.rx_pat_socket.bind("tcp://127.0.0.1:%s" % RX_PAT_PACKETS_PORT)
        self.tx_socket.bind("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)

        #self.spi.open(SPI_BUS, SPI_DEV)


    def spi_read(self):
        read_cmd = (((VNC_ADDR << 1) + 0b1) << 4) + 0b1111
        
        read_req = bytearray(BUS_PKT_LEN*2)
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
        while (i < BUS_PKT_LEN):
            #ack = (response[i*2] >> 1) & 1
            #rxf = (response[i*2] >> 2) & 1
            txe = (response[i*2] >> 3) & 1

            if (txe == 1):
                break

            raw_data[i] = response[i*2 + 1]
            i += 1
        return raw_data

    def bus_parse(self):
        ccsds_sync = [0x35, 0x2E, 0xF8, 0x53]

        if (len(bus_bytes_buffer) == 0):
            # Nothing received, nothing to do
            return

        b = 0
        for b in range(len(bus_bytes_buffer)-len(ccsds_sync)+1):
            if (bus_bytes_buffer[b:b+4] == ccsds_sync):
                break

        start_index = b+len(ccsds_sync)
        pkt_len_index = start_index + 4
        assert pkt_len_index + 1 < len(bus_bytes_buffer)
            # What if entire packet hasn't been received yet?

        pkt_len = (bus_bytes_buffer[pkt_len_index] << 8) | bus_bytes_buffer[pkt_len_index + 1] + 1

        crc_index = start_index + 6 + pkt_len - 2 
        assert crc_index + 1 < len(bus_bytes_buffer)
            # What if entire packet hasn't been received yet?

        crc = (bus_bytes_buffer[crc_index] << 8) | bus_bytes_buffer[crc + 1]

        #check crc!! 
        





    def run(self):

        while True:

            # Check for data from bus
            raw_bus_data = self.spi_read()
            assert raw_bus_data is not None
            # Add bus data to raw buffer
            bus_bytes_buffer.append(raw_bus_data)
            # Parse packet from buffer

            

if __name__ == '__main__':
    bus_interface = BusInterface()
    bus_interface.run()






