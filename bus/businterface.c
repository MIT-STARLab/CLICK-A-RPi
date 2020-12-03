#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <time.h>

#include "crc.h"


#define SPI_DEV "/dev/bct"

#define SPI_XFER_LEN 105

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

uint8_t tx[SPI_XFER_LEN];
uint8_t rx[SPI_XFER_LEN];

int main(int argc, char **argv)
{
	int i, ret, status;
  int spi_fd;

  // initialize tx for test
  uint16_t apid = 0x250;
  uint16_t packet_len = SPI_XFER_LEN - 11; //minus header(10) minus 1
  tx[0] = 0x35;
  tx[1] = 0x2E;
  tx[2] = 0xF8;
  tx[3] = 0x53;
  tx[4] = (uint8_t)(apid >> 8) & 0xFF;
  tx[5] = (uint8_t)(apid & 0xFF);
  tx[6] = 0b11000000; //sequence count is 0
  tx[7] = 0x00; //sequence count is 0
  tx[8] = (uint8_t)(packet_len >> 8) & 0xFF;
  tx[9] = (uint8_t) packet_len & 0xFF;

  for (i = 10; i < (SPI_XFER_LEN-12); i++) {
    tx[i] = i % 0xFF;
  }

  uint16_t crc = 0;
  crc = crc_16_update(0, tx, SPI_XFER_LEN - 2);

  tx[SPI_XFER_LEN-2] = (uint8_t)(crc >> 8) & 0xFF;
  tx[SPI_XFER_LEN-1] = (uint8_t) crc & 0xFF;

  spi_fd = open(SPI_DEV, O_RDONLY);
  if (spi_fd < 0) {
    perror("can't open device");
  }

	while (1) {
    status = read(spi_fd, rx, 20);

    printf("response(%2d): \n", status);
    if (status < 0) {
      return;
    }
      for (i = 0; i < 20; i++) {
        printf(" %02x", rx[i]);
      }
      printf("\n");
    }


	return ret;
}
