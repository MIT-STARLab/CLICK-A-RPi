/*
** Filename: crc.c
** Author: Ondrej
** 
** CRC functions
*/

#include "crc.h"

// Ref: BCT XB1 User Guide pg. 66
uint16 crc_16_update(uint16 crc, uint8 *data, size_t len)
{
    uint16 byte = 0;
    uint8 bit = 8, msb = 0;
    while (byte < len)
    {
        msb = crc >> 15;
        crc <<= 1;
        crc |= (data[byte] >> --bit) & 1;
        if (msb) crc ^= 0x8005;
        if (bit < 1)
        {
            bit = 8;
            byte++;
        }
    }
    return crc;
}

// Ref: BCT XB1 User Guide pg. 66
uint16 crc_16_finalize(uint16 crc)
{
    uint8 push[] = {0, 0};
    return crc_16_update(crc, push, 2);
}