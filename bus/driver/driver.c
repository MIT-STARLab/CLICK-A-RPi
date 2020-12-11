/*
** Filename: driver.c
** Author: Ondrej
** 
** CLICK kernel SPI driver
** Handles communication with the VNC2L
*/

#include "driver.h"

/* Reinitialize SPI data structure */
static void spi_data_init(spi_data_t *data)
{
    data->sync = 0;
    data->read_len = 0;
    data->payload_len = 0;
    data->busy = true;
    data->read_ptr = NULL;
    data->header = NULL;
    data->tx = list_first_entry_or_null(data->tx_queue, packet_t, list);
}

/* SPI transmit-receive function */
static void spi_xfer(spi_data_t *data, uint8_t *rx, uint16_t len)
{
    data->xfer.tx_buf = data->tx ? data->tx->data : NULL;
    data->xfer.rx_buf = rx;
    data->xfer.len = len;
    data->read_len += len;
    data->read_ptr = rx;
    if(spi_async(data->spi, &data->msg) != 0)
    {
        dev_warn_once(&data->spi->dev, "spi_async error\n");
    }
}

/* Main VNC2L interrupt handler */
irqreturn_t interrupt_handler(int irq, void *arg)
{
    spi_data_t *data = arg;
    if (data->busy == false)
    {
        spi_data_init(data);
        if (data->tx != NULL)
        {
            list_del(&data->tx->list);
            spi_xfer(data, data->rx, data->tx->len);
        }
        else spi_xfer(data, data->rx, DEFAULT_READ_LEN);
    }
    dev_info_once(&data->spi->dev, "IRQ is working\n");
    return IRQ_HANDLED;
}

/* SPI completion handler */
void spi_xfer_complete(void *arg)
{
    unsigned int copied = 0;
    spi_data_t *data = arg;

    /* Free tx packet memory, if any */
    if (data->tx)
    {
        packet_free(data->tx);
        data->tx = NULL;
    }
    
    /* Start parsing rx data */
    while(data->read_len)
    {
        /* Waiting for sync marker */
        if (!SYNC_VALID(data->sync))
        {
            data->read_len--;
            SYNC_UPDATE(data->sync, data->read_ptr++);
            if (SYNC_VALID(data->sync))
            {
                data->header = (packet_header_t*) data->read_ptr;
                /* Request more header data if needed */
                if(data->read_len < PACKET_HEADER_LEN)
                {
                    data->read_ptr += data->read_len;
                    spi_xfer(data, data->read_ptr, PACKET_HEADER_LEN - data->read_len);
                    return;
                }
            }
        }
        /* After sync, read CCSDS header */
        else if (data->payload_len == 0)
        {
            data->payload_len = ((data->header->len_msb << 8) | data->header->len_lsb) + 1;
            data->read_ptr += PACKET_HEADER_LEN;
            data->read_len -= PACKET_HEADER_LEN;
            /* Request more payload data if needed */
            if(data->read_len < data->payload_len)
            {
                data->read_ptr += data->read_len;
                spi_xfer(data, data->read_ptr, data->payload_len - data->read_len);
                return;
            }
        }
        /* Finish reading packet */
        else
        {
            data->read_ptr += data->read_len;
            data->read_len = 0;
        }
    }

    /* Check if successful */
    if (data->payload_len > 0)
    {
        /* Try adding packet to fifo */
        if(kfifo_avail(data->rx_fifo) >= (PACKET_OVERHEAD + data->payload_len))
        {
            copied = kfifo_in(data->rx_fifo, ((uint8_t*) data->header) - PACKET_SYNC_LEN,
                PACKET_OVERHEAD + data->payload_len);
            if (copied == (PACKET_OVERHEAD + data->payload_len))
            {
                dev_info_once(&data->spi->dev, "packets are being received\n");
            }
            else dev_warn_once(&data->spi->dev, "rx fifo copy error\n");
        }
        else dev_warn_once(&data->spi->dev, "rx fifo is full\n");
    }
    else dev_warn_once(&data->spi->dev, "invalid packet received\n");

    /* Clear busy flag and wake up read function */
    data->busy = false;
    complete(data->xfer_done);
}
