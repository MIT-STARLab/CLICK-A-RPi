/*
** Filename: crc.h
** Author: Ondrej
** 
** CRC functions
*/

#ifndef _crc_H_
#define _crc_H_

#include "vos.h"
#include "config.h"

uint16 crc_16_update(uint16 crc, uint8 *data, size_t len);
uint16 crc_16_finalize(uint16 crc);

#endif /* _crc_H_ */