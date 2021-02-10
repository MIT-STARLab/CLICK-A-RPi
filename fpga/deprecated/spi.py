#!/usr/bin/env python
import spidev
import memorymap

spi_freq = 1000000
edfa_timeout = 1000
registers = memorymap.MemoryMap()
status = registers.get_register('flg')
edfa_tx = registers.get_register('etx')
edfa_rx = registers.get_register('erx')

def xfer(spi, addr, write_flag, value):
    rx = []
    zero = [0, 0]
    tx = [(addr & 0x7F) | ((write_flag & 1) << 7), (value & 0xFF)]
    try:
        spi.xfer(tx, spi_freq)
        rx = spi.xfer(zero, spi_freq)
    except:
        pass
    if len(rx) == 2:
        addr = int(rx[0] & 0x7F)
        value = int(rx[1])
    else:
        addr = 0
        value = 0
    return addr, value

def xfer_once(addr, write_flag, value):
    try:
        spi = spidev.SpiDev()
        spi.open(1, 0)
        addr, value = xfer(spi, addr, write_flag, value)
        spi.close()
    except Exception, err:
        print(str(err))
        addr = 0
        value = 0
    return addr, value

def read_register(addr):
    addr, value = xfer_once(addr, 0, 0)
    return value

def write_register(addr, value):
    addr, value = xfer_once(addr, 1, value)
    return value

def verify_boot():
    addr, value = xfer_once(status, 0, 0)
    dcm_locked = value & 1
    crc_err = (value >> 4) & 1
    if addr == status and dcm_locked == 1 and crc_err == 0:
        return True
    return False

def edfa(cmd):
    i = 0
    timeout = edfa_timeout
    cmd = cmd + '\r'
    reply = []
    result = ''
    try:
        spi = spidev.SpiDev()
        spi.open(1, 0)
        
        while i < len(cmd) and timeout > 0:
            addr, value = xfer(spi, status, 0, 0)
            tx_ready = (value >> 6) & 1
            if tx_ready == 1:
                addr, value = xfer(spi, edfa_tx, 1, ord(cmd[i]))
                if value == ord(cmd[i]):
                    i = i + 1
                    timeout = edfa_timeout
                else:
                    break
            else:
                timeout = timeout - 1

        if i != len(cmd):
            print("EDFA timeout, written " + str(i) + " bytes")

        else:
            while timeout > 0:
                addr, value = xfer(spi, status, 0, 0)
                rx_ready = (value >> 5) & 1
                if rx_ready == 1:
                    addr, value = xfer(spi, edfa_rx, 0, 0)
                    if addr == edfa_rx:
                        reply.append(value)
                        timeout = edfa_timeout
                        if value == 62:
                            break
                else:
                    timeout = timeout - 1

            if value != 62:
                print("EDFA timeout, read " + str(len(reply)) + " bytes")
            
            else:
                for i, char in enumerate(reply):
                    if char == 13:
                        reply[i] = 10
                reply = ''.join([chr(x) for x in reply]).split('\n')
                for i, msg in enumerate(reply):
                    if len(msg) > 1 and i < (len(reply) - 1):
                        result = msg

    except Exception, err:
        print(str(err))

    return result
    