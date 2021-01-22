/* 
 * FPGALink C interface
 * Author: Ondrej
 */

#ifndef _FL_H_
#define _FL_H_

#include "libfpgalink.h"

uint8 fpgalink_init(struct FLContext **handle);
FLStatus fpgalink_read(struct FLContext *handle, uint8 channel, uint8 *value);
FLStatus fpgalink_write(struct FLContext *handle, uint8 channel, uint8 value);

#endif
