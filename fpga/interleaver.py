#!/usr/bin/env python
#
# Name: nodeIntrlv.py
#
# Copyright (C) 2016 Caleb Ziegler
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Interleave encoded file/data or deinterleave corrupted/error file prior
# to decoding

from __future__ import division
import os
import sys
#from PIL import Image
import math
from package_data_functions import *

def intrlvFile(filename, ppm_order, n, k, save=False,out_filename=''):
    #inputs: name of input file, ppm order, n and k used for encoding
    #outputs: char string, interleaved data saved in file if specified
    data_per_frame = int(math.ceil(length_data_per_frame(ppm_order, n, k) + 4)*(n/k))    
    col = n
    with open(filename,'rb') as f:
        data = f.read()
    data = [byte for byte in data]
    intrlvdata = []
    num_frames = len(data)/data_per_frame
    print(len(data))
    print(data_per_frame)
    if num_frames.is_integer():
        pass
    else:
        raise ValueError('Wrong amount of data pass to interleaver')
    count = 0
    # Check to make sure we interleave over one frame
    while count < num_frames:
        codeword = []
        frame = data[count*data_per_frame:(count+1)*data_per_frame]
        if len(frame) % (col) != 0:
            print('Error: number of columns should be n used for encoding')
            raise ValueError('n or filesize wrong')
        else:
            # Cut data into list of  codewords
            rows = int(len(frame) / col)
            for i in range(0, rows):
                codeword.append(frame[i * col:(i + 1) * col])

        # Perform interleaving
        intrlvframe = ''
        for i in range(0, col):
            for j in range(0, rows):
                intrlvframe = intrlvframe + codeword[j][i]
        intrlvdata.append(intrlvframe)
        count+= 1

    # save to file or return data
    if save:
        with open(out_filename, 'wb') as g:
            g.write(''.join(str(frame) for frame in intrlvdata))
    else:
        # intrlvdata is list of frames (each frame is string of chars)
        return ''.join(intrlvdata)


def deintrlvFile(filename, ppm_order, n, k, save=False, out_filename=''):
    #inputs: name of input file, ppm order, n and k used for encoding
    #outputs: char string, deinterleaved data saved in file if specified
    data_per_frame = int(math.ceil(length_data_per_frame(ppm_order, n, k) + 4)*(n/k))
    col = n
    with open(filename,'rb') as f:
        intrlv = f.read()
    intrlv = [byte for byte in intrlv]
    deintrlvdata = []
    num_frames = len(intrlv)/data_per_frame
    if num_frames.is_integer():
        pass
    else:
        raise ValueError('Wrong amount of data pass to interleaver')
    count = 0
    # Check to make sure we deinterleave over one frame
    while count < num_frames:
        codeword = []
        frame = intrlv[count*data_per_frame:(count+1)*data_per_frame]
        if len(frame) % col != 0:
            print('Error: number of columns should be n used for encoding')
            raise ValueError('n or filesize wrong')
        else:
            rows = int(len(frame) / col)
            # Split data up into codewords of length rows
            for i in range(0, col):
                codeword.append(frame[i * rows:(i + 1) * rows])

        # Perform deinterleaving
        deintrlvframe = ''
        for i in range(0, rows):
            for j in range(0, col):
                deintrlvframe = deintrlvframe + codeword[j][i]
        deintrlvdata.append(deintrlvframe)
        count += 1

    # save file or return deinterleaved string of chars
    if save:
        with open(out_filename, 'wb') as g:
            g.write(''.join(str(frame) for frame in deintrlvdata))
    else:
        return ''.join(deintrlvdata)


def intrlvData(data, ppm_order, n, k, save=False, out_filename=''):
    #inputs: data as a char string, ppm order, n and k used for encoding
    #outputs: list of char strings (where each char string is a frame), saved file of interleaved data if specified
    data_per_frame = int(math.ceil(length_data_per_frame(ppm_order, n, k) + 4)*(n/k))    
    intrlvdata = []  
    col = n
    data = [byte for byte in data] 
    num_frames = len(data)/data_per_frame
    if num_frames.is_integer():
        pass
    else:
        raise ValueError('Wrong amount of data pass to interleaver')
    count = 0
    while count < num_frames:
        codeword = []
        frame = data[count*data_per_frame:(count+1)*data_per_frame]
        # Check to make sure we interleave over one frame 
        if (len(frame) % col) != 0:
            print('Error: not interleaving over one frame - number of columns should be n for encoding')
            raise ValueError('n or filesize wrong')
        else:
            # Save data as list of codewords
            rows = int(len(frame) / col)
            for i in range(0, rows):
                codeword.append(frame[i * col:(i + 1) * col])

        # Perform interleaving
        intrlvframe = ''
        for i in range(0, col):
            for j in range(0, rows):
                intrlvframe = intrlvframe + codeword[j][i]
        intrlvdata.append(intrlvframe)
        count += 1

    # Save file or return interleaved string of chars
    if save:
        with open(out_filename,'wb') as g:
            g.write(''.join(str(frame) for frame in intrlvdata))
    else:
        #return intrlvdata
        return intrlvdata

def deintrlvData(data, ppm_order, n, k, save=False, out_filename=''):
    #inputs: data as a char string, ppm order, n and k used for encoding
    #outputs: char string with frames concatenated, saved file of deinterleaved data if specified
    data_per_frame = int(math.ceil(length_data_per_frame(ppm_order, n, k) + 4)*(n/k))
    col = n
    intrlv = [byte for byte in data]
    deintrlvdata = []
    num_frames = len(intrlv)/data_per_frame
    if num_frames.is_integer():
        pass
    else:
        raise ValueError('Wrong amount of data pass to interleaver')
    count = 0
    # Check to make sure we deinterleave over one frame
    while count < num_frames:
        codeword = []
        frame = intrlv[count*data_per_frame:(count+1)*data_per_frame]
        if len(frame) % col != 0:
            print('Error: number of columns should be n used for encoding')
            raise ValueError('n or filesize wrong')
        else:
            rows = int(len(frame) / col)
            # Split data up into codewords of length rows
            for i in range(0, col):
                codeword.append(frame[i * rows:(i + 1) * rows])

        # Perform deinterleaving
        deintrlvframe = ''
        for i in range(0, rows):
            for j in range(0, col):
                deintrlvframe = deintrlvframe + codeword[j][i]
        deintrlvdata.append(deintrlvframe)
        count += 1

    # Save file or return deinterleaved string
    if save:
        with open(out_filename,'wb') as g:
            g.write(''.join(str(frame) for frame in deintrlvdata))
    else:
        return ''.join(deintrlvdata)
