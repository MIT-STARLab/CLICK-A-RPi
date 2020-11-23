#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
// #include <gpiod.h>
#include <limits.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <pigpiod_if2.h>
#include <time.h>

#include "crc.h"

#ifndef	CONSUMER
#define	CONSUMER	"Consumer"
#endif

#define GPIO_CHIP "gpiochip0"
#define GPIO_LINE 18
#define GPIO_TIMEOUT LONG_MAX

#define SPI_DEV "/dev/spidev0.0"
#define SPI_DELAY 0
#define SPI_SPEED 12000000
#define SPI_BITS 8
#define SPI_XFER_LEN 105

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

uint8_t tx[SPI_XFER_LEN];
uint8_t rx[SPI_XFER_LEN];

int spi_transfer(int fd)
{
  int status, i, errno;
  struct spi_ioc_transfer xfer;

  memset(&xfer, 0, sizeof(xfer));

  xfer.tx_buf = (unsigned long)tx;
	xfer.rx_buf = (unsigned long)rx;
	xfer.len = ARRAY_SIZE(tx);
	xfer.delay_usecs = SPI_DELAY;
	xfer.speed_hz = SPI_SPEED;
	xfer.bits_per_word = SPI_BITS;

  errno = 0;
	status = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSAGE");
	}

  if (errno != 0)
  {
    printf("SPI IOCTL status(%d)error(%d) %s\n", status, errno, strerror(errno));
  }

  printf("response(%2d): ", status);
  for (i = 0; i < SPI_XFER_LEN; i++)
    printf(" %02x");
  printf("\n");
  return status;
}


int main(int argc, char **argv)
{
	// struct timespec ts = {GPIO_TIMEOUT, 0};
	// struct gpiod_line_event event;
	// struct gpiod_chip *chip;
	// struct gpiod_line *line;
	int i, ret, status;

  int spi_fd;

  int gpioHandle = pigpio_start(0, 0);
  ret = set_mode(gpioHandle, GPIO_LINE, PI_INPUT);
  if ( ret != 0 ) {
    printf("Can't set mode %d\n", ret);
  }
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

  uint16 crc;
  crc = crc_16_update(0, tx, SPI_XFER_LEN - 2);
  crc = crc_16_finalize(crc);
  
  tx[SPI_XFER_LEN-2] = (uint8_t)(crc >> 8) & 0xFF;
  tx[SPI_XFER_LEN-1] = (uint8_t) crc & 0xFF;

  spi_fd = open(SPI_DEV, O_RDWR);
  if (spi_fd < 0) {
    perror("can't open device");
    goto end;
  }

	// chip = gpiod_chip_open_by_name(GPIO_CHIP);
	// if (!chip) {
	// 	perror("Open chip failed\n");
	// 	ret = -1;
	// 	goto end;
	// }
  //
	// line = gpiod_chip_get_line(chip, GPIO_LINE);
	// if (!line) {
	// 	perror("Get line failed\n");
	// 	ret = -1;
	// 	goto close_chip;
	// }
  //
	// ret = gpiod_line_request_both_edges_events(line, CONSUMER);
	// if (ret < 0) {
	// 	perror("Request event notification failed\n");
	// 	ret = -1;
	// 	goto release_line;
	// }
  int edge;
	while (1) {
    edge = wait_for_edge(gpioHandle, GPIO_LINE, EITHER_EDGE, GPIO_TIMEOUT);
    if (edge == 1) {
      printf("Received interrupt: %d\n", GPIO_LINE);
      status = spi_transfer(spi_fd);
    }
		// ret = gpiod_line_event_wait(line, &ts); //this blocks
		// if (ret < 0) {
    //   perror("Wait event notification failed\n");
    //   ret = -1;
    //   goto release_line;
		// }
    // else if (ret == 1) {
    //   printf("Wait event notification on line #%u timeout\n", GPIO_LINE);
    //   status = spi_transfer(spi_fd);
    // }
	}

	ret = 0;
release_line:
// 	gpiod_line_release(line);
close_chip:
    pigpio_stop(gpioHandle);
// 	gpiod_chip_close(chip);
end:
	return ret;
}
