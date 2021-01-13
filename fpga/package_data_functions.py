#!/usr/bin/env python
#
# Defines bitmask (used synonymously with bit-to-symbol mapping), padding, length_data_per_frame, and increment_fill, and software_modulator functions
from __future__ import division
import math
import struct
import time

def length_data_per_frame(ppm_order, n, k, time=1):
    #inputs: ppm order, n and k used for encoding, length of frame in seconds
    #outputs: length of frame (amount of information that can be in a frame) in bytes
    if ppm_order == 4 or ppm_order == 8 or ppm_order == 64:
        bytes_per_bitmask = 3
    elif ppm_order == 16:
        bytes_per_bitmask = 1
    elif ppm_order == 32:
        bytes_per_bitmask = 5
    elif ppm_order == 128:
        bytes_per_bitmask = 7
    else:
        raise ValueError('Invalid ppm order. Try again')
    guardtime = ppm_order/4
    sym_time = 5e-9*(ppm_order+guardtime)
    bits_per_sym = math.log(ppm_order,2)
    data_rate = bits_per_sym/sym_time
    sym_rate = data_rate/8
    num_cw = math.ceil(time*sym_rate/n)
    sym_packet = num_cw*n
    # want sym_packet to be divisible by the number of bytes we have to load so no leftover bytes
    while (sym_packet % bytes_per_bitmask) != 0:
        num_cw += 1
        sym_packet = num_cw*n
    msgsym_packet = sym_packet*(k/n)
    return int(msgsym_packet-4) # length in bytes


def increment_fill(amount_fill, ppm_order):
    # inputs: amount of bytes needed to fill, ppm order
    # outputs: char string of fill values
    fill_list = []
    for i in range(0,amount_fill):
        fill_list.append(chr(i%255)) # fill from 0 to 255
    return ''.join(fill_list)

def padding(filename, ppm_order, n, k):
    #inputs: source file to pad to length equal to an integer number of frames, ppm order, n and k used for encoding
    #output: padded data as binary data (char string)
    # first 4 bytes of every frame denote amount of data contained in frame, the rest of the data in the frame is assumed to be just fill (incremental fill)
    with open(filename,'rb') as f:
        data = f.read()
    data = [byte for byte in data]
    data_per_frame = length_data_per_frame(ppm_order, n, k)
    if len(data) < data_per_frame:
        # one frame that is less than frame length
        amount_fill = data_per_frame - len(data)
        end_data_pointer = struct.pack('>I',(len(data)+4)) 
        return end_data_pointer + ''.join(data) + increment_fill(amount_fill, ppm_order)
    elif len(data) == data_per_frame:
        # exactly one frame
        end_data_pointer = struct.pack('>I',(len(data)+4))
        return end_data_pointer + ''.join(data)
    else:
        # more than one frame (could be very slow because deleting large part of a list...may need to change)
        num_full_frames = int(len(data)/data_per_frame)
        frames_list = []
        for i in range(1,num_full_frames+1):
            end_data_pointer = struct.pack('>I',(data_per_frame+4))
            frame = data[:data_per_frame]
            del data[:data_per_frame]
            frames_list.append(end_data_pointer + ''.join(frame))
        amount_fill = data_per_frame - len(data)
        end_data_pointer = struct.pack('>I',(len(data)+4))
        frames_list.append(end_data_pointer + ''.join(data) + increment_fill(amount_fill, ppm_order))
        return ''.join(frames_list)

def bitmask(intrlv_data, ppm_order, n, k, save=False, outfile =''):
    #inputs: intrlv_data is a list of char strings (each frame is a char string), ppm_order is integer, n and k are integer values used for encoding, save and outfile are optional inputs in case the bitmask (bit-to-symbol mapped) data needs to be saved to a file
    #outputs: bitmasked (bit-to-symbol mapped) data with frame and tracking bytes as binary data (char string)
    
    bitmask_data = []
    data_per_frame = math.ceil((length_data_per_frame(ppm_order, n, k)+4)*(n/k))
    # when testing RS codes, only keep line in else statement (comment out rest)
    if ppm_order == 4 or ppm_order == 8:
        m = 6
    else:
        m = int(math.log(ppm_order,2))

    for intrlv_frame in intrlv_data:

        intrlv_frame = [ord(byte) for byte in intrlv_frame]
        # convert each byte to binary representation
        data = ["{:08b}".format(byte) for byte in intrlv_frame]
        # make one long binary string
        stringdata = ''.join(data)
        # pull out m bit chunks (where m is log2(ppm_order)) to make each bitmask byte (only works for testing RS code)
        bitmask_string = ''.join(chr(int(stringdata[i:i+m],2)) for i in xrange(0,len(stringdata),m))
        # determine framebyte
        if ppm_order == 4:
            framebyte = 0x83
        elif ppm_order == 8:
            framebyte = 0x87
        elif ppm_order == 16:
            framebyte = 0x8F
        elif ppm_order == 32:
            framebyte = 0x9F
        elif ppm_order == 64:
            framebyte = 0xBF
        elif ppm_order == 128:
            framebyte = 0xFF
        else:
            raise('PPM order not supported')
        bitmask_data.append(chr(framebyte)) # no framebyte when TESTING RS CODES
        bitmask_data.append(bitmask_string)
       
    # specify tracking mode at the end of a set of frames (tracking mode may need to be counter in flight module, but prob not important because we know how many symbols are in a complete frame, so any fill can be used)
    bitmask_data.append(chr(framebyte)) # no trackingbyte when TESTING RS CODES

    # output
    if save:
        with open(outfile,'wb') as g:
            g.write(''.join(bitmask_data))
    return ''.join(bitmask_data)


def software_modulator(bitmask_data,outfilename):
    # Inputs: bitmask data as binary data, file name to save software modulation file to
    # outputs: returns ppm orders and symbols, saves ppm orders and symbols to txt file, saves binary file with symbols 

    bitmask_data = [ord(byte) for byte in bitmask_data]
    ppm_order = None
    ppm_orders = []
    symbols = []
    acq4 = [0,0,1,3,0,0,3,1,2,1,2,0,3,0,3,2]
    acq8 = [0,1,3,7,0,1,7,2,4,2,5,1,7,1,7,5]
    acq16 = [0,2,7,14,1,2,15,5,8,4,10,2,14,3,14,11]
    reverse_bitmask = None
    for byte in bitmask_data:
        if byte == 0x83:
            symbols.extend(acq4)
            ppm_order = 4
            ppm_orders.append(4)
            reverse_bitmask = [0x30,0x0C,0x03]
        
        elif byte == 0x87:
            symbols.extend(acq8)
            ppm_order = 8
            ppm_orders.append(8)
            reverse_bitmask = [0x38,0x07]
         
        elif byte == 0x8F:
            symbols.extend(acq16)
            ppm_order = 16
            ppm_orders.append(16)
          
        elif byte == 0x9F:
            symbols.extend(acq16)
            ppm_order = 32
            ppm_orders.append(32)
           
        elif byte == 0xBF:
            symbols.extend(acq16)
            ppm_order = 64
            ppm_orders.append(64)
            
        elif byte == 0xFF:
            symbols.extend(acq16)
            ppm_order = 128
            ppm_orders.append(128)
            
        else:
            if ppm_order == 4:
                symbols.append((reverse_bitmask[0] & byte) >> 4)
                symbols.append((reverse_bitmask[1] & byte) >> 2)
                symbols.append(reverse_bitmask[2] & byte)
            elif ppm_order == 8:
                symbols.append((reverse_bitmask[0] & byte) >> 3)
                symbols.append(reverse_bitmask[1] & byte)
            else:
                symbols.append(byte)

    outfilename_softmod = outfilename.replace('bitmask_files','software_modulated_files')
    outfilenametxt = outfilename_softmod + 'softmod.txt'
    outfilenamebinary = outfilename_softmod + 'softmod'

    with open(outfilenametxt,'wb') as f:
        f.write('%s' % ppm_orders)
        f.write('%s' % symbols)

    symbols_chr = [chr(symbol) for symbol in symbols]

    with open(outfilenamebinary,'wb') as g:
        g.write(''.join(symbols_chr))

    return ppm_orders,symbols 
















