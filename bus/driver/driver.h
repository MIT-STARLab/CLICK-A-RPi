/*
** Filename: driver.h
** Author: Ondrej
** 
** CLICK kernel SPI driver
** Handles communication with the VNC2L
*/

#ifndef _driver_H_
#define _driver_H_

#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/kfifo.h>
#include <linux/wait.h>

/* High-level definitions */
#define NAME "click_spi"
#define DEVICE "bct"
#define SPI_FREQ 8000000
#define RX_FIFO_LEN (1 << 22)
#define MAX_TX_QUEUE_LEN (1 << 26)
#define PACKET_SYNC_MARKER 0x352EF853
#define PACKET_SYNC_LEN 4
#define PACKET_HEADER_LEN 6
#define PACKET_OVERHEAD (PACKET_SYNC_LEN + PACKET_HEADER_LEN)
#define PACKET_TC_MAX_LEN 1024
#define PACKET_TM_MAX_LEN 4100
#define PACKET_LATENCY 4
#define DEFAULT_READ_LEN 128
#define SYNC_VALID(sync) (sync == PACKET_SYNC_MARKER)
#define SYNC_UPDATE(sync, ptr) sync = ((sync << 8) | ((*(ptr)) & 0xFF))
#define NOOP_APID 0x2FF

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

/* Tx packet queue entry */
typedef struct {
    uint16_t len;
    uint8_t *data;
    struct list_head list;
} packet_t;

/* SPI driver data */
typedef struct {
    struct spi_device *spi;
    struct spi_message msg;
    struct spi_transfer xfer;
    struct completion *xfer_done;
    struct list_head *tx_queue;
    struct kfifo *rx_fifo;
    struct cdev fs_dev;
    struct class *cl;
    dev_t dev_num;
    uint32_t sync;
    uint16_t read_len;
    uint16_t payload_len;
    uint16_t apid;
    uint8_t *read_ptr;
    packet_header_t *header;
    packet_t *tx;
    uint8_t *rx;
    bool busy;
} spi_data_t;

/* Public prototypes */
irqreturn_t interrupt_handler(int irq, void *arg);
void spi_xfer_complete(void *arg);
void packet_free(packet_t *packet);

#endif /* _driver_H_ */
