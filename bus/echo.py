import binascii
from crccheck.crc import Crc16CcittFalse as crc16

# Echo bus packets back with apid 0x3FF
# Based on Hannah's depacketizer code

SPI_DEV = '/dev/bct'

CCSDS_HEADER_LEN = 6
PKT_LEN_INDEX = 4

class Echoer:
    ccsds_sync = bytearray([0x35, 0x2E, 0xF8, 0x53])

    spi = open(SPI_DEV, 'r+b', buffering=0)

    bus_pkts_buffer = []

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
        # returns a full bus packet

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

        # Read 6 CCSDS header bytes
        buf = self.read_data(CCSDS_HEADER_LEN)
        pkt_len = (buf[PKT_LEN_INDEX] << 8) | buf[PKT_LEN_INDEX + 1] + 1
        # Read payload data bytes and crc bytes
        buf.extend(self.read_data(pkt_len))
 
        # Check crc
        crc_index = CCSDS_HEADER_LEN + pkt_len - 2
        crc = (buf[crc_index] << 8) | buf[crc_index + 1]

        # Calculate CRC over the entire packet
        crc_check = crc16.calc(buf[:crc_index])

        if (crc == crc_check):
            self.bus_pkts_buffer.append(buf)
        else:
            print('CRC error')

    def echo_pkt(self):
        try:
            buf = self.bus_pkts_buffer[0]
        except IndexError as e:
            # Empty buffer, but that's ok
            return

        # Change APID
        buf[0] = 0x3
        buf[1] = 0xFF

        # Update CRC
        crc_index = len(buf) - 2
        crc_new = crc16.calc(buf[:crc_index])
        buf[crc_index] = crc_new >> 8
        buf[crc_index + 1] = crc_new & 0xFF

        # Add sync marker and send
        pkt = []
        pkt.extend(self.ccsds_sync)
        pkt.extend(buf)
        self.spi.write(bytearray(pkt))

    def run(self):
        print("Starting packet echo")
        while True:
            self.acquire_bus_pkt()
            self.echo_pkt()
            self.bus_pkts_buffer[:] = []

if __name__ == '__main__':
    echoer = Echoer()
    echoer.run()
