/*
** Filename: crc.h
** Author: Ondrej
**
** CRC function used by BCT
*/

#ifndef _crc_H_
#define _crc_H_

#include <stdint.h>
#include <linux/types.h>

uint16_t crc_16_update(uint16_t crc, uint8_t *data, uint16_t len);

#endif /* _crc_H_ */
