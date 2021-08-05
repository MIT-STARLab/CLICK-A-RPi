#!/usr/bin/env python
import hashlib
import math
import struct
import binascii
import os
import re
import shutil
import sys
import errno

import zmq

sys.path.append('/root/lib/') #flight path
sys.path.append('/home/pi/CLICK-A-RPi/') #testpath
sys.path.append('../lib/') #test path

from ipc_packets import TxPacket, RxCommandPacket
from options import FL_ERR_EMPTY_DIR, FL_ERR_FILE_NAME, FL_ERR_SEQ_LEN, FL_ERR_MISSING_CHUNK, FL_SUCCESS
from options import TX_PACKETS_PORT, TLM_DL_FILE, TLM_ASSEMBLE_FILE, TLM_DISASSEMBLE_FILE, FILE_MGMT_MAX
from errors import *

class FileError(Exception):
    def __init__(self, flag):
     self.flag = flag
    def __str__(self):
     return repr(self.flag)

def safe_open_w(path):
    dir = os.path.dirname(path)
    if dir:
        make_dirs(dir)

    return open(path, 'wb+')

def make_dirs(path):
    try:
        os.makedirs(path)
    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise

def auto_downlink_file(rx_pkt_payload, socket_tx_packets):
    req_raw_size = len(rx_pkt_payload) - 6
    transfer_id, _, _, _ = struct.unpack('!HHH%ds'%req_raw_size, rx_pkt_payload)

    disassemble_file(rx_pkt_payload, socket_tx_packets)

    request_flag = 0xFF
    request_file_cmd = struct.pack('!HBHH', transfer_id, request_flag, 0, 0)
    request_file(request_file_cmd, socket_tx_packets)

def zip_downlink_file(rx_pkt_payload, socket_tx_packets, zip_file_stem = ""):
    req_raw_size = len(rx_pkt_payload) - 7
    flag, transfer_id, chunk_size, file_name_len, file_name_payload = struct.unpack('!BHHH%ds'%req_raw_size, rx_pkt_payload)
    file_name = file_name_payload[0:file_name_len]

    #zip file
    try:
        if zip_file_stem:
            #custom zip file name
            zip_file_name = '%s.tar.gz' % zip_file_stem
        else:
            #default zip file name
            zip_file_name = '%s.tar.gz' % (os.path.splitext(file_name)[0])
        
        #zip with tar
        os.system('tar -zcvf %s %s' % (zip_file_name, file_name)) 

        if(flag != 0x00): 
            #downlink zip file
            zip_file_name_len = len(zip_file_name)
            tx_pkt_payload = struct.pack('!HHH%ds'%(zip_file_name_len), transfer_id, chunk_size, zip_file_name_len, zip_file_name)
            disassemble_file(tx_pkt_payload, socket_tx_packets)

            request_flag = 0xFF
            request_file_cmd = struct.pack('!HBHH', transfer_id, request_flag, 0, 0)
            request_file(request_file_cmd, socket_tx_packets)

            if(flag == 0xFF):
                #delete zip file            
                recursive = 0x00 #not recursive
                del_payload = struct.pack('!BH%ds'%(zip_file_name_len), recursive, zip_file_name_len, zip_file_name)
                del_file(del_payload, socket_tx_packets)
                            
    except Exception as e:
        send_exception(socket_tx_packets, e)

def zip_downlink_pat_data(rx_pkt_payload, socket_tx_packets):
    downlink_flag, chronological_flag, transfer_id, chunk_size, directory_id = struct.unpack('!BBHHH', rx_pkt_payload)
    if(chronological_flag == 0x00):
        #directory id is chronological order (i.e. directory id = experiment id)
        file_name = "/root/log/pat/%d" % directory_id
        file_name_len = len(file_name)
        tx_packet_payload = struct.pack('!BHHH%ds'%file_name_len, downlink_flag, transfer_id, chunk_size, file_name_len, file_name)
        zip_downlink_file(tx_packet_payload, socket_tx_packets)
    else:
        #directory id is reverse chronological order (i.e. directory id = max_exp_id - exp_id)
        #try:
        pat_directory_listing = os.listdir('/root/log/pat') #Get directory list
        pat_exp_ids = []
        for directory in pat_directory_listing:
            if(directory.isdigit()):
                pat_exp_ids.append(int(directory))
        pat_exp_ids.sort(reverse=True)
        exp_id = pat_exp_ids[directory_id] #index with 0 as most recent experiment, 1 as previous experiment, etc.
        file_name = "/root/log/pat/%d" % exp_id
        file_name_len = len(file_name)
        tx_packet_payload = struct.pack('!BHHH%ds'%file_name_len, downlink_flag, transfer_id, chunk_size, file_name_len, file_name)
        zip_downlink_file(tx_packet_payload, socket_tx_packets, str(directory_id))
            
        # except Exception as e:
        #     send_exception(socket_tx_packets, e)

def disassemble_file(rx_pkt_payload, socket_tx_packets):
    req_raw_size = len(rx_pkt_payload) - 6
    transfer_id, chunk_size, file_name_len, file_name_payload = struct.unpack('!HHH%ds'%req_raw_size, rx_pkt_payload)
    file_name = file_name_payload[0:file_name_len]
    try:
        hash_func = hashlib.md5()
        with open(file_name, "rb") as source_file:
            buf = source_file.read(1024) # Hash block size is 1024, change if necessary
            while (len(buf) > 0):
                hash_func.update(buf)
                buf = source_file.read(1024)

        file_hash = hash_func.digest()

        hash_file_name = '/root/file_staging/'+str(transfer_id)+'/'+'md5.hash'
        # hash_file_name = 'test_file_staging/'+str(transfer_id)+'/'+'md5.hash'

        with safe_open_w(hash_file_name) as hash_file:
            hash_file.write(file_hash)

        with open(file_name, 'rb') as source_file:
            file_len = os.stat(file_name).st_size
            seq_len = int(math.ceil(float(file_len)/chunk_size))
            seq_num = 1

            while (seq_num*chunk_size) < file_len:
                chunk_data = source_file.read(chunk_size)
                chunk_file_name = '/root/file_staging/'+str(transfer_id)+'/'+str(seq_num)+'_'+str(seq_len)+'.chunk'
                # chunk_file_name = 'test_file_staging/'+str(transfer_id)+'/'+str(seq_num)+'_'+str(seq_len)+'.chunk'
                with safe_open_w(chunk_file_name) as chunk_file:
                    chunk_file.write(chunk_data)
                seq_num += 1

            if (((seq_num - 1) * chunk_size) < file_len):
                chunk_data_len = file_len - ((seq_num - 1) * chunk_size)
                chunk_data = source_file.read(chunk_data_len)
                chunk_file_name = '/root/file_staging/'+str(transfer_id)+'/'+str(seq_num)+'_'+str(seq_len)+'.chunk'
                # chunk_file_name = 'test_file_staging/'+str(transfer_id)+'/'+str(seq_num)+'_'+str(seq_len)+'.chunk'
                with safe_open_w(chunk_file_name) as chunk_file:
                    chunk_file.write(chunk_data)

        # Send telemetry
        txpacket = TxPacket()
        raw_packet = txpacket.encode(APID = TLM_DISASSEMBLE_FILE, payload = struct.pack('!HH', transfer_id, seq_num))
        socket_tx_packets.send(raw_packet)

        #Memory Management
        manage_file_staging()

    except Exception as e:
        send_exception(socket_tx_packets, e)

def request_file(rx_pkt_payload, socket_tx_packets):
    transfer_id, all_flag, chunk_start_index, num_chunks = struct.unpack('!HBHH', rx_pkt_payload)
    try:
        # FOR TEST:
        # all_files = [f for f in os.listdir('test_file_staging/'+str(transfer_id)+'/') if f.endswith('.chunk')]
        # chunk_files = sorted(all_files, key=lambda s: int(s.split('_')[0]))

        all_files = [f for f in os.listdir('/root/file_staging/'+str(transfer_id)+'/') if f.endswith('.chunk')]
        chunk_files = sorted(all_files, key=lambda s: int(s.split('_')[0]))

        # Dir is empty -> error
        if not chunk_files:
            raise FileError(FL_ERR_EMPTY_DIR)

        # Get the full sequence length from the first packet
        chunk_name_pattern = re.compile('\d{1,5}_\d{1,5}.chunk')
        if chunk_name_pattern.match(chunk_files[0]) is None:
            # Chunk file name is not correct
            raise FileError(FL_ERR_FILE_NAME)

        _,raw_seq_len = chunk_files[0].split('_')
        seq_len = int(raw_seq_len[:-6])

        if (all_flag == 0xFF):
            chunk_start_index = 1
            num_chunks = seq_len

        # Check that you're not requesting out of bounds
        if (chunk_start_index + num_chunks - 1 > seq_len):
            raise FileError(FL_ERR_OUT_OF_BOUNDS)

        # Retrieve hash
        hash_file_name = '/root/file_staging/'+str(transfer_id)+'/'+'md5.hash'
        # hash_file_name = 'test_file_staging/'+str(transfer_id)+'/'+'md5.hash'

        with open(hash_file_name, 'rb') as hash_file:
            file_hash = hash_file.read()


        for i in range(chunk_start_index, chunk_start_index + num_chunks):
            # FOR TEST:
            file_name = '/root/file_staging/'+str(transfer_id)+'/'+str(i)+'_'+str(seq_len)+'.chunk'
            # file_name = 'test_file_staging/'+str(transfer_id)+'/'+str(i)+'_'+str(seq_len)+'.chunk'
            with open(file_name, 'rb') as chunk_file:
                chunk_size = os.stat(file_name).st_size
                packet_payload = chunk_file.read()
                packet = struct.pack('!H%dsHHH%ds' % (16, chunk_size), transfer_id, file_hash, i, seq_len, chunk_size, packet_payload)
                # print(i, seq_len)
            txpacket = TxPacket()
            raw_packet = txpacket.encode(APID = TLM_DL_FILE, payload = packet)
            socket_tx_packets.send(raw_packet)

    except Exception as e:
        send_exception(socket_tx_packets, e)

def uplink_file(rx_pkt_payload, socket_tx_packets):
    req_raw_size = len(rx_pkt_payload) - 8
    transfer_id, seq_num, seq_len, chunk_len, chunk_payload = struct.unpack('!HHHH%ds'%req_raw_size, rx_pkt_payload)

    chunk_data = chunk_payload[0:chunk_len]

    try:
        if (seq_num > seq_len):
            raise FileError(FL_ERR_SEQ_LEN)
        # FOR TEST:
        # file_name = 'test_file_staging/'+str(transfer_id)+'/'+('%05d' % seq_num)+'_'+('%05d' % seq_len)+'.chunk'
        file_name = '/root/file_staging/'+str(transfer_id)+'/'+str(seq_num)+'_'+str(seq_len)+'.chunk'
        with safe_open_w(file_name) as chunk_file:
            chunk_file.write(chunk_data)
    except Exception as e:
        send_exception(socket_tx_packets, e)

def assemble_file(rx_pkt_payload, socket_tx_packets):
    req_raw_size = len(rx_pkt_payload) - 4
    transfer_id, file_name_len, file_name_payload = struct.unpack('!HH%ds'%req_raw_size, rx_pkt_payload)
    print('assemble_file - received transfer_id: ', transfer_id)
    file_name = file_name_payload[0:file_name_len]
    print('assemble file - file_name: ', file_name)
    missing_chunks = []
    pkt_data = ''
    try:
        # FOR TEST:
        # all_files = [f for f in os.listdir('test_file_staging/'+str(transfer_id)+'/') if f.endswith('.chunk')]
        # chunk_files = sorted(all_files, key=lambda s: int(s.split('_')[0]))

        all_files = [f for f in os.listdir('/root/file_staging/'+str(transfer_id)+'/') if f.endswith('.chunk')]
        chunk_files = sorted(all_files, key=lambda s: int(s.split('_')[0]))

        # Check that the directory isn't empty
        if not chunk_files:
            # Dir is empty -> no chunks are received yet
            raise FileError(FL_ERR_EMPTY_DIR)

        # Assume the first chunk has the correct sequence count

        _,seq_len = chunk_files[0].split('_')
        seq_len_all = int(seq_len[:-6])
        # FOR TEST:
        # chunk_size_all = os.stat('test_file_staging/'+str(transfer_id)+'/'+chunk_files[0]).st_size
        chunk_size_all = os.stat('/root/file_staging/'+str(transfer_id)+'/'+chunk_files[0]).st_size

        chunk_name_pattern = re.compile('\d{1,5}_\d{1,5}.chunk')

        with safe_open_w(file_name) as out_file:

            for i in range(seq_len_all):

                if chunk_name_pattern.match(chunk_files[i]) is None:
                    # Chunk file name is not correct
                    raise FileError(FL_ERR_FILE_NAME)

                # Check file names are valid
                seq_num, seq_len = chunk_files[i].split('_')
                seq_num = int(seq_num)
                seq_len = int(seq_len[:-6])

                if seq_len != seq_len_all:
                    # Sequence count doesn't match
                    raise FileError(FL_ERR_SEQ_LEN)

                # FOR TEST:
                # chunk_size = os.stat('test_file_staging/'+str(transfer_id)+'/'+chunk_files[i]).st_size
                chunk_size = os.stat('/root/file_staging/'+str(transfer_id)+'/'+chunk_files[i]).st_size

                if (seq_num != i+1):
                    print('seq_num: %d != i: %d' % (seq_num, i+1))
                    missing_chunks.append(i)
                else:
                    # FOR TEST:
                    # with open('test_file_staging/'+str(transfer_id)+'/'+chunk_files[i], 'rb') as curr_chunk:
                    with open('/root/file_staging/'+str(transfer_id)+'/'+chunk_files[i], 'rb') as curr_chunk:
                        # Read the entire chunk, may be better to buffer?
                        out_file.write(curr_chunk.read())

        if missing_chunks:
            raise FileError(FL_ERR_MISSING_CHUNK)


    except FileError as e:
        print('assemble_file - except')
        print('transfer_id: ', transfer_id)
        print('Error: ', e)
        pkt_data = format_err_response(transfer_id, e, missing_chunks)

    except Exception as e:
        send_exception(socket_tx_packets, e)

    else:
        print('assemble_file - else')
        pkt_data = format_success_response(transfer_id)

    finally:
        # FOR TEST
        # print(pkt_data)
        # print(binascii.hexlify(bytearray(pkt_data)))
        print('assemble_file - pkt_data: ', pkt_data)
        tx_pkt = TxPacket()
        raw_tx_pkt = tx_pkt.encode(TLM_ASSEMBLE_FILE, pkt_data) #pkt_data is a single byte string (e.g. the output of struct.pack)
        socket_tx_packets.send(raw_tx_pkt)


def format_err_response(transfer_id, file_error, missing_chunks):
    pkt = ''
    pkt += struct.pack('!H', transfer_id)
    pkt += struct.pack('B', file_error.flag)
    pkt += struct.pack('!H', len(missing_chunks))
    for chunk_id in missing_chunks:
        pkt += struct.pack('!H', chunk_id)
    return pkt

def format_success_response(transfer_id):
    pkt = ''
    pkt += struct.pack('!H', transfer_id)
    pkt += struct.pack('B', FL_SUCCESS)
    pkt += struct.pack('!H', 0)
    return pkt

def validate_file(rx_pkt_payload, socket_tx_packets):
    req_raw_size = len(rx_pkt_payload) - 18
    file_hash, file_name_len, file_name_payload = struct.unpack('!%dsH%ds'% (16, req_raw_size), rx_pkt_payload)

    file_name = file_name_payload[0:file_name_len]

    try:
        hash_func = hashlib.md5()
        with open(file_name, 'rb') as source_file:
            buf = source_file.read(1024) # Hash block size is 1024, change if necessary
            while (len(buf) > 0):
                hash_func.update(buf)
                buf = source_file.read(1024)
        check_hash = hash_func.digest()

        if (check_hash != file_hash):
            print('Hash Check Failed!')
            err_pkt = TxPacket()
            err_pkt_payload = ''
            pkt_payload += struct.pack('!16s', check_hash)
            pkt_payload += struct.pack('!16s', file_hash)
            pkt_payload += struct.pack('!H', file_name_len)
            pkt_payload += struct.pack('!%ds' % file_name_len, file_name)
            raw_err_pkt = err_pkt.encode(ERR_FL_FILE_INVALID, pkt_payload)
            socket_tx_packets.send(raw_err_pkt)

    except Exception as e:
        send_exception(socket_tx_packets, e)

def move_file(rx_pkt_payload, socket_tx_packets):
    req_raw_size = len(rx_pkt_payload) - 4
    src_file_name_len, dest_file_name_len, file_names = struct.unpack('!HH%ds'% (req_raw_size), rx_pkt_payload)

    src_file_name = file_names[:src_file_name_len]
    dest_file_name = file_names[src_file_name_len:src_file_name_len+dest_file_name_len]

    try:
        shutil.move(src_file_name, dest_file_name)
    except Exception as e:
        send_exception(socket_tx_packets, e)


def del_file(rx_pkt_payload, socket_tx_packets):
    req_raw_size = len(rx_pkt_payload) - 3
    recursive, file_name_len, file_name_payload = struct.unpack('!BH%ds'% (req_raw_size), rx_pkt_payload)
    file_name = file_name_payload[0:file_name_len]

    try:
        if (recursive == 0xFF):
            shutil.rmtree(file_name)
        else:
            os.remove(file_name)
    except Exception as e:
        send_exception(socket_tx_packets, e)


def manage_file_staging():
    #count the number of directories in /root/file_staging/, and delete old directories if the directory limit is exceeded
    directory_listing = os.listdir('/root/file_staging') #Get directory list
    num_directories = len(directory_listing)
    if(num_directories > FILE_MGMT_MAX):
        #create ordered integer list of transfer ids
        transfer_id_listing = [0]*num_directories #initialize list of transfer ids
        for i in range(0,num_directories):
            transfer_id_listing[i] = int(directory_listing[i])
        transfer_id_listing.sort()

        #delete old directories
        num_deletions = num_directories - FILE_MGMT_MAX
        for i in range(0,num_deletions):
            shutil.rmtree('/root/file_staging/%d' % transfer_id_listing[i])


def auto_assemble_file(rx_pkt_payload, socket_tx_packets):
    req_raw_size = len(rx_pkt_payload) - 20
    transfer_id, file_hash, dest_file_name_len, dest_file_name_payload = struct.unpack('!H%dsH%ds'% (16, req_raw_size), rx_pkt_payload)
    dest_file_name = dest_file_name_payload[0:dest_file_name_len]

    temp_file_name = '/root/file_staging/'+str(transfer_id)+'/reassembled_file.temp'
    temp_file_name_len = len(temp_file_name)
    assm_file_cmd = struct.pack('!HH%ds' % (temp_file_name_len), transfer_id, temp_file_name_len, temp_file_name)
    assemble_file(assm_file_cmd, socket_tx_packets)

    val_file_cmd = struct.pack('!%dsH%ds' % (16, temp_file_name_len), file_hash, temp_file_name_len, temp_file_name)
    validate_file(val_file_cmd, socket_tx_packets)

    mov_file_cmd = struct.pack('!HH%ds%ds' % (temp_file_name_len, dest_file_name_len), temp_file_name_len, dest_file_name_len, temp_file_name, dest_file_name)
    move_file(mov_file_cmd, socket_tx_packets)

    del_flag = 0xFF
    del_file_name = '/root/file_staging/'+str(transfer_id)
    del_file_name_len = len(del_file_name)
    del_file_cmd = struct.pack('!BH%ds' % del_file_name_len, del_flag, del_file_name_len, del_file_name)
    del_file(del_file_cmd, socket_tx_packets)


def file_test():
    context = zmq.Context()
    socket_tx = context.socket(zmq.PUB)
    socket_tx.connect("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)

    _ = input('Enter Anything to Continue to PL_REQUEST_FILE using test_file.txt as source.')

    '''PL_AUTO_DOWNLINK_FILE send file test'''
    send_file_name = 'test_file.txt'
    send_file_name_len = len(send_file_name)
    send_file_transfer_id = 0x1234
    send_file_chunk_size = 1024
    send_file_cmd = struct.pack('!HHH%ds' % (send_file_name_len), send_file_transfer_id, send_file_chunk_size, send_file_name_len, send_file_name)
    print('Test PL_AUTO_DOWNLINK_FILE using ' + send_file_name)
    print('send_file_cmd: ', send_file_cmd)
    auto_downlink_file(send_file_cmd, socket_tx)

    _ = input('Enter Anything to Continue to PL_UPLOAD_FILE using test_file.txt as source.')

    '''PL_UPLINK_FILE receive file test'''
    receive_file_len = os.stat('test_file.txt').st_size
    receive_transfer_id = 56789
    receive_chunk_size = 800
    receive_seq_len = math.ceil(float(receive_file_len)/receive_chunk_size)
    receive_seq_num = 1
    with open('test_file.txt', 'rb') as rec_test_file:

        while (receive_seq_num*receive_chunk_size) < receive_file_len:
            receive_data = rec_test_file.read(receive_chunk_size)
            receive_file_cmd = struct.pack('!HHHH%ds' % (receive_chunk_size), receive_transfer_id, receive_seq_num, receive_seq_len, receive_chunk_size, receive_data)
            print('receive_seq_num: ', receive_seq_num)
            print('receive_file_cmd: ', receive_file_cmd)

            uplink_file(receive_file_cmd, socket_tx)
            print('Chunk file created at /root/file_staging/'+str(receive_transfer_id)+'/'+str(receive_seq_num)+'_'+str(receive_seq_len)+'.chunk')

            receive_seq_num += 1

        if (((receive_seq_num - 1) * receive_chunk_size) < receive_file_len):
            receive_chunk_size = receive_file_len - ((receive_seq_num - 1) * receive_chunk_size)
            receive_data = rec_test_file.read(receive_chunk_size)
            receive_file_cmd = struct.pack('!HHHH%ds' % (receive_chunk_size), receive_transfer_id, receive_seq_num, receive_seq_len, receive_chunk_size, receive_data)
            print('receive_seq_num: ', receive_seq_num)
            print('receive_file_cmd: ', receive_file_cmd)

            uplink_file(receive_file_cmd, socket_tx)
            print('Chunk file created at /root/file_staging/'+str(receive_transfer_id)+'/'+str(receive_seq_num)+'_'+str(receive_seq_len)+'.chunk')

    _ = input('Enter Anything to Continue to PL_ASSEMBLE_FILE to ' + '/root/file_staging/'+str(receive_transfer_id)+'/reassembled_file.txt')

    '''PL_ASSEMBLE_FILE assemble file test'''
    assm_file_name = '/root/file_staging/'+str(receive_transfer_id)+'/reassembled_file.txt'
    assm_file_name_len = len(assm_file_name)
    assm_file_transfer_id = 56789
    assm_file_cmd = struct.pack('!HH%ds' % (assm_file_name_len), assm_file_transfer_id, assm_file_name_len, assm_file_name)
    print('assm_file_cmd: ', assm_file_cmd)
    assemble_file(assm_file_cmd, socket_tx)

    _ = input('Enter Anything to Continue to PL_VALIDATE_FILE with ' + '/root/file_staging/'+str(receive_transfer_id)+'/reassembled_file.txt')

    '''PL_VALIDATE_FILE validate file test'''
    val_file_name = '/root/file_staging/'+str(receive_transfer_id)+'/reassembled_file.txt'
    val_file_name_len = len(val_file_name)

    val_hash_func = hashlib.md5()
    with open(val_file_name, 'rb') as source_file:
        buf = source_file.read(1024)
        while (len(buf) > 0):
            val_hash_func.update(buf)
            buf = source_file.read(1024)
    val_file_hash = val_hash_func.digest()
    print('val_file_hash: ', val_file_hash)

    val_file_transfer_id = 0x1234
    val_file_cmd = struct.pack('!%dsH%ds' % (16, val_file_name_len), val_file_hash, val_file_name_len, val_file_name)
    print('val_file_cmd: ', val_file_cmd)

    validate_file(val_file_cmd, socket_tx)

    '''PL_MOVE_FILE move file test'''
    mov_src_file_name = '/root/file_staging/'+str(receive_transfer_id)+'/reassembled_file.txt'
    mov_src_file_name_len = len(mov_src_file_name)

    mov_dest_file_name = '/root/commandhandler/final_file.txt'
    mov_dest_file_name_len = len(mov_dest_file_name)

    _ = input('Enter Anything to Continue to PL_MOVE_FILE. Source: ' + mov_src_file_name + '. Destination: ' + mov_dest_file_name)

    mov_file_cmd = struct.pack('!HH%ds%ds' % (mov_src_file_name_len, mov_dest_file_name_len), mov_src_file_name_len, mov_dest_file_name_len, mov_src_file_name, mov_dest_file_name)
    print('mov_file_cmd: ', mov_file_cmd)

    move_file(mov_file_cmd, socket_tx)

    _ = input('Enter Anything to Continue to PL_DEL_FILE with /root/commandhandler/final_file.txt')

    '''PL_DEL_FILE delete file test'''
    del_flag = 0x00
    del_file_name = '/root/commandhandler/final_file.txt'
    del_file_name_len = len(del_file_name)

    del_file_cmd = struct.pack('!BH%ds' % del_file_name_len, del_flag, del_file_name_len, del_file_name)

    del_file(del_file_cmd, socket_tx)

    _ = input('Enter Anything to Continue to recursive PL_DEL_FILE with staging directory /root/file_staging/56789')

    del_flag = 0xFF
    del_file_name = '/root/file_staging/56789'
    del_file_name_len = len(del_file_name)

    del_file_cmd = struct.pack('!BH%ds' % del_file_name_len, del_flag, del_file_name_len, del_file_name)

    del_file(del_file_cmd, socket_tx)

def file_test2():
    context = zmq.Context()
    socket_tx = context.socket(zmq.PUB)
    socket_tx.connect("tcp://127.0.0.1:%s" % TX_PACKETS_PORT)

    '''PL_AUTO_DOWNLINK_FILE downlink file test'''
    auto_dl_file_name = 'test_file.txt'
    auto_dl_file_name_len = len(auto_dl_file_name)
    auto_dl_file_transfer_id = 5555
    auto_dl_file_chunk_size = 1024
    auto_dl_file_cmd = struct.pack('!HHH%ds' % (auto_dl_file_name_len), auto_dl_file_transfer_id, auto_dl_file_chunk_size, auto_dl_file_name_len, auto_dl_file_name)

    auto_downlink_file(auto_dl_file_cmd, socket_tx)

    '''PL_DISASSEMBLE_FILE disassemble file test'''
    disassemble_file_name = 'test_file.txt'
    disassemble_file_name_len = len(disassemble_file_name)
    disassemble_file_transfer_id = 1234
    disassemble_file_chunk_size = 1024
    disassemble_file_cmd = struct.pack('!HHH%ds' % (disassemble_file_name_len), disassemble_file_transfer_id, disassemble_file_chunk_size, disassemble_file_name_len, disassemble_file_name)

    disassemble_file(disassemble_file_cmd, socket_tx)

    '''PL_REQUEST_FILE request file test'''
    request_file_transfer_id = 1234
    request_flag = 0x00
    request_start_index = 3
    request_chunk_num = 5
    request_file_cmd = struct.pack('!HBHH', request_file_transfer_id, request_flag, request_start_index, request_chunk_num)

    request_file(request_file_cmd, socket_tx)

if __name__ == '__main__':
    file_test()
