/*
** Filename: test_write.c
** Author: Ondrej
** 
** Test write to CLICK kernel SPI driver
*/

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include "crc.h"

#define DEVICE "bct"
#define PACKET_SYNC_LEN 4
#define PACKET_HEADER_LEN 6
#define PACKET_OVERHEAD (PACKET_SYNC_LEN + PACKET_HEADER_LEN)
#define PACKET_TM_MAX_LEN 4100
#define DEFAULT_WRITE_LEN 100

/* CCSDS header in little endian */
typedef struct {
    uint8_t apid_msb : 3;
    uint8_t secondary : 1;
    uint8_t type : 1;
    uint8_t version : 3;
    uint8_t apid_lsb;
    uint8_t seq_msb : 6;
    uint8_t grouping : 2;
    uint8_t seq_lsb;
    uint8_t len_msb;
    uint8_t len_lsb;
} packet_header_t;

/* Usage sudo ./test_write [len] [offset] [repeat] */
int main (int argc, char *argv[])
{
    int fd;
    uint32_t i;
    uint8_t *buffer = NULL;
    packet_header_t *header = NULL;
    uint16_t offset = 0, len = DEFAULT_WRITE_LEN, repeat = 1, crc = 0;

    /* Usage */
    if (argc == 1)
    {
        printf("Usage: %s [len] [offset] [repeat]\n", argv[0]);
        return 0;
    }

    /* Read packet size */
    if (argc > 1 && atoi(argv[1]) <= PACKET_TM_MAX_LEN && atoi(argv[1]) > DEFAULT_WRITE_LEN)
    {
        len = atoi(argv[1]);
    }

    /* Read packet start offset */
    if (argc > 2 && atoi(argv[2]) <= DEFAULT_WRITE_LEN)
    {
        offset = atoi(argv[2]);
    }

    /* Read repeat count */
    if (argc > 3 && atoi(argv[3]) <= 500)
    {
        repeat = atoi(argv[3]);
    }

    /* Assemble packet */
    buffer = calloc(offset + len, 1);
    if (buffer)
    {
        buffer[offset + 0] = 0x35;
        buffer[offset + 1] = 0x2E;
        buffer[offset + 2] = 0xF8;
        buffer[offset + 3] = 0x53;
        header = (packet_header_t*) (buffer + offset + PACKET_SYNC_LEN);
        header->apid_lsb = 0x33;
        header->apid_msb = 0x3;
        header->grouping = 0x3;
        header->len_lsb = (len - PACKET_OVERHEAD - 1) & 0xFF;
        header->len_msb = (len - PACKET_OVERHEAD - 1) >> 8;

        /* Write to kernel driver */
        fd = open("/dev/" DEVICE, O_WRONLY);
        if (fd >= 0)
        {
            for(i = 0; i < repeat; i++)
            {
                *((uint32_t*)(buffer + offset + PACKET_OVERHEAD)) = (i+1);
                crc = crc_16_update(0xFFFF, (uint8_t*) header, len - PACKET_SYNC_LEN - 2);
                buffer[offset + len - 2] = crc >> 8;
                buffer[offset + len - 1] = crc & 0xFF;
                if(write(fd, buffer, offset + len) != (offset + len))
                {
                    printf("Failed to write to /dev/" DEVICE "\n");
                    break;
                }
            }
            printf("Done\n");
            close(fd);
        }
        else printf("Failed to open /dev/" DEVICE "\n");
        free(buffer);
    }
    else printf("Failed to allocate memory\n");

    return 0;
}
