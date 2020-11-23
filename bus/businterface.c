#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <gpiod.h>
#include <limits.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <zmq.h>

#ifndef	CONSUMER
#define	CONSUMER	"Consumer"
#endif

#define GPIO_CHIP "gpiochip0"
#define GPIO_LINE 25
#define GPIO_TIMEOUT LONG_MAX

#define SPI_DEV "/dev/spidev0.0"
#define SPI_DELAY 0
#define SPI_SPEED 500000
#define SPI_BITS 8
#define SPI_XFER_LEN 984

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

int spi_transfer(int fd)
{
  int status;
  uint8_t tx[SPI_XFER_LEN];
  uint8_t rx[SPI_XFER_LEN];

  // initialize tx for test
  int i;
  for (i = 0; i < SPI_XFER_LEN; i++) {
    tx[i] = i % 0xFF;
  }

  struct spi_ioc_transfer xfer = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = SPI_XFER_LEN,
		.delay_usecs = SPI_DELAY,
		.speed_hz = SPI_SPEED,
		.bits_per_word = SPI_BITS,
	};

	status = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
	if (status < 0) {
		perror("SPI_IOC_MESSAGE");
	}

  printf("response(%2d, %2d): ", len, status);
  for (i = 0; i < SPI_XFER_LEN; i++)
    printf(" %02x");
  printf("\n");
  return status;
}


int main(int argc, char **argv)
{
	struct timespec ts = {GPIO_TIMEOUT, 0};
	struct gpiod_line_event event;
	struct gpiod_chip *chip;
	struct gpiod_line *line;
	int i, ret, status;

  int spi_fd;

  spi_fd = open(SPI_DEV, O_RDWR);
  if (spi_fd < 0) {
    perror("can't open device");
    goto end;
  }

	chip = gpiod_chip_open_by_name(GPIO_CHIP);
	if (!chip) {
		perror("Open chip failed\n");
		ret = -1;
		goto end;
	}

	line = gpiod_chip_get_line(chip, GPIO_LINE);
	if (!line) {
		perror("Get line failed\n");
		ret = -1;
		goto close_chip;
	}

	ret = gpiod_line_request_both_edges_events(line, CONSUMER);
	if (ret < 0) {
		perror("Request event notification failed\n");
		ret = -1;
		goto release_line;
	}

	while (1) {
		ret = gpiod_line_event_wait(line, &ts); //this blocks
		if (ret < 0) {
      perror("Wait event notification failed\n");
      ret = -1;
      goto release_line;
		}
    else if (ret == 1) {
      printf("Wait event notification on line #%u timeout\n", GPIO_LINE);
      status = spi_transfer(spi_fd);
    }
	}

	ret = 0;
release_line:
	gpiod_line_release(line);
close_chip:
	gpiod_chip_close(chip);
end:
	return ret;
}
