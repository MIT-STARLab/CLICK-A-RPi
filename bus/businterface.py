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

class BusInterface:
    context = zmq.Context()

    rx_cmd_socket = context.socket(zmq.PUSH)
    rx_pat_socket = context.socket(zmq.PUSH)
    tx_socket = context.socket(zmq.PULL)

    spi = spidev.SpiDev()

    poller = zmq.Poller()

    def __init__(self):

        self.rx_cmd_socket.bind("tcp://127.0.0.1:%s" % FPGA_MAP_REQUEST_PORT)
        self.rx_pat_socket.bind("tcp://127.0.0.1:%s" % FPGA_MAP_ANSWER_PORT)
        self.tx_socket.bind("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)

        self.spi.open(SPI_BUS, SPI_DEV)

        self.poller.register(tx_socket, POLLIN)


    def spi_read(self):
        return

    def run(self):

        while True:
            sockets = dict(self.poller.poll())
            if socket_pull in sockets and sockets[socket_pull] == zmq.POLLIN:
                message = socket_pull.recv()

            

if __name__ == '__main__':
    bus_interface = BusInterface()
    bus_interface.run()






