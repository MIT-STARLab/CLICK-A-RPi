/*
** Filename: crc.h
** Author: Ondrej
**
** CRC functions
*/

#ifndef _crc_H_
#define _crc_H_

uint16_t crc_16_update(uint16_t crc, uint8_t *data, size_t len);
uint16_t crc_16_finalize(uint16_t crc);

#endif /* _crc_H_ */
