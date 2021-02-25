import zmq
import shutil
import sys
import traceback
import os
import struct
import binascii

from ipc_packets import *
from options import *

sys.path.append('/root/lib/') #flight path
sys.path.append('/home/pi/CLICK-A-RPi/') #testpath
sys.path.append('../lib/') #test path

def send_exception(tx_socket, err):
    error_type = type(err).__name__
    error_type_len = len(error_type)

    file_name = os.path.basename(sys.exc_info()[2].tb_frame.f_code.co_filename)
    file_name_len = len(file_name)

    line_num = sys.exc_info()[2].tb_lineno

    full_trace = traceback.format_exc()
    full_trace_len = len(full_trace)


    try:
        errno = err.errno
    except:
        errno = 0

    err_pkt = TxPacket()

    pkt_payload = ''
    pkt_payload += struct.pack('B', error_type_len)
    pkt_payload += struct.pack('!%ds' % error_type_len, error_type)
    pkt_payload += struct.pack('B', errno)
    pkt_payload += struct.pack('!H', file_name_len)
    pkt_payload += struct.pack('!%ds' % file_name_len, file_name)
    pkt_payload += struct.pack('!H', line_num)

    if (FULL_TRACE_ENABLE):
        available_space = (BUS_DATA_LEN-error_type_len-file_name_len-6)
        if(full_trace_len > available_space):
            pkt_payload += struct.pack('!H', available_space)
            pkt_payload += struct.pack('!%ds' % available_space, full_trace)
        else:
            pkt_payload += struct.pack('!H', full_trace_len)
            pkt_payload += struct.pack('!%ds' % full_trace_len, full_trace)

    raw_err_pkt = err_pkt.encode(ERR_GEN_EXCEPTION, pkt_payload)
    tx_socket.send(raw_err_pkt)
    if (RAISE_ENABLE):
        raise
