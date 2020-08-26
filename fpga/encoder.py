#!/usr/bin/env python
#
# Name: nodeRS.py
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
# This program relies heavily on the unireedsolomon library located at:
# https://github.com/lrq3000/unireedsolomon


import unireedsolomon
import os, sys
from math import ceil
#from PIL import Image


def encodeFile(filename,n,k,save=False,out_filename=''):
    # inputs: file to encode, n and k used for encoding, option to save file, output file name
    # outputs: encoded data as a char string, encoded data saved to file if specified
    # Initialize RS coder and coded string
    coder = unireedsolomon.rs.RSCoder(n,k)
    coded = ''

    # Open file as object and read length k segments of file
    f = open(filename,'rb')

    # Read in k-length segments of the file and encode those k length segments, then
    # add the encoded blocks (now of length n) to make encoded message
    while True:
        block = f.read(k)
        if not block:
            break
        c = coder.encode_fast(block)
        coded = coded + c

    # Close file to clear memory
    f.close()

    # Save to image or return (save to image only works for PNG images
    # and text - loss of data for JPEG files)
    # Returning encoded string as variable is guaranteed to work
    if save:
        with open(out_filename, 'wb') as g:
            g.write(coded)
    else:
        return coded


def decodeFile(filename,n,k,save=False,out_filename=''):
    # inputs: file to decode, n and k used for decoding, option to save file, output file name
    # outputs: decoded data as a char string, decoded data saved to file if specified
    # Initialize RS coder and decoded string
    coder = unireedsolomon.rs.RSCoder(n,k)
    decoded = ''

    # read file's contents
    f = open(filename,'rb')

    # While loop for decoding data (read in n-length
    # segments of data) and concatenate all decoded strings 
    # to recover message
    while True:
        chunk = f.read(n)
        if not chunk:
            break
        try:
            d = coder.decode_fast(chunk,True)
            d = d[0]					# Message stored in first item of tuple
        except:
            d = chunk[:k]                               # if we can't decode, just take msg symbols
        decoded = decoded + d

    # close file when done
    f.close()

    # Save to file or return (don't need to save to greyscale image
    # like with encoded data, so just write decoded message to file)
    if save:
        with open(out_filename,'wb') as g: 
            g.write(decoded)
    else:
        return decoded


def encodeData(data,n,k,save=False,out_filename=''):
    # inputs: data as a char string, n and k used for encoding, option to save file, output file name
    # outputs: encoded data as a char string, encoded data saved to file if specified
    # Initialize RS coder and coded string
    coder = unireedsolomon.rs.RSCoder(n,k)
    coded = ''

    # While loop to read in k length segments of data
    count = 0
    NumCodeWords = ceil(float(len(data))/float(k))
    while count < NumCodeWords:
        # If on last codeword, go from current spot in data
        # to last value in data
        if count == NumCodeWords - 1:
            chunk = data[count*k:]
        else:
            chunk = data[count*k:(count+1)*k]
        c = coder.encode_fast(chunk)
        coded = coded + c
        count += 1

    # Save to image or return (save to image only works for PNG images
    # and text - loss of data for JPEG files)
    # Returning encoded string as variable is guaranteed to work
    if save:
        with open(out_filename,'wb') as g:
            g.write(coded)
    else:
        return coded


def decodeData(data,n,k,save=False,out_filename=''):
    # inputs: data as a char string, n and k used for decoding, option to save file, output file name
    # outputs: decoded data as a char string, decoded data saved to file if specified
    # Initialize RS coder and coded string
    coder = unireedsolomon.rs.RSCoder(n,k)
    decoded = ''

    # While loop for decoding data (read in n-length
    # segments of data) and concatenate all decoded strings 
    # to recover message
    count = 0
    NumCodeWords = len(data)/n
    while count < NumCodeWords:
        chunk = data[count*n:(count+1)*n]
        try:
            d = coder.decode_fast(chunk,True)
            d = d[0]					# Message stored in first item of tuple
        except:
            d = chunk[:k]                               # if too many errors, take msg symbols
        decoded = decoded + d
        count += 1	

    # Save to file or return
    if save:
        with open(out_filename,'wb') as g:
            g.write(decoded)
    else:
        return decoded

