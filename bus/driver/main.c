/*
** Filename: main.c
** Author: Ondrej
** 
** Initializes the CLICK kernel SPI driver
*/

#include "driver.h"

/* Private variables */
static LIST_HEAD(tx_queue);
static DECLARE_COMPLETION(xfer_done);
static struct kfifo rx_fifo;
static size_t tx_queue_len = 0;
static uint8_t dev_busy = false;

/* Free tx packet memory */
void packet_free(packet_t *packet)
{
    if(packet != NULL)
    {
        if(packet->data != NULL)
        {
            kfree(packet->data);
            tx_queue_len -= packet->len;
        }
        kfree(packet);
        tx_queue_len -= sizeof(packet_t);
    }
}

/* Clear tx packet queue on exit */
static void tx_queue_clear(void)
{
    packet_t *packet;
    struct list_head *pos, *n;
    list_for_each_safe(pos, n, &tx_queue)
    {
        packet = list_entry(pos, packet_t, list);
        list_del(pos);
        packet_free(packet);
    }
}

/* Callback on /dev open
** Return busy error if open for reading, unless write-only requested */
static int device_open(struct inode *inode, struct file *file)
{
    if ((file->f_flags & O_ACCMODE) != O_WRONLY)
    {
        if (dev_busy) return -EBUSY;
        dev_busy = true;
    }
    return 0;
}

/* Callback on /dev close */
static int device_close(struct inode *inode, struct file *file)
{
    if ((file->f_flags & O_ACCMODE) != O_WRONLY) dev_busy = false;
    return 0;
}

/* Rx packet forwarding to userspace via read on /dev */
static ssize_t packet_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
    unsigned int copied = 0;
    if(f->f_flags & O_NONBLOCK)
    {
        pr_warn_once(NAME ": non-block read requested\n");
        return -EAGAIN;
    }

    /* Wait for SPI transfers until len data is read */
    while (kfifo_len(&rx_fifo) < len)
    {
        wait_for_completion(&xfer_done);
    }

    /* Try read data from rx fifo */
    if(kfifo_len(&rx_fifo) >= len)
    {
        if(kfifo_to_user(&rx_fifo, buf, len, &copied) == 0)
        {
            if(copied == len)
            {
                return len;
            }
            else pr_warn_once(NAME ": kfifo_to_user copied only %d out of %d", copied, len);
        }
        else pr_warn_once(NAME ": kfifo_to_user failed");
    }
    else pr_warn_once(NAME ": read request for %d but fifo only has %d\n", len, kfifo_len(&rx_fifo));
    return -EAGAIN;
}

/* Add tx packet from userspace to kernel memory queue via write to /dev */
static ssize_t packet_write(struct file *f, const char __user *buf, size_t len, loff_t *off)
{
    int res = 0;
    packet_t *new_packet = NULL;
    uint16_t len_tot = len + (2 * PACKET_LATENCY);

    /* Check memory availability */
    if (len_tot > PACKET_TM_MAX_LEN || (MAX_TX_QUEUE_LEN - tx_queue_len) < (len_tot + sizeof(packet_t)) ||
        ((new_packet = kzalloc(sizeof(packet_t), GFP_KERNEL)) == NULL))
    {
        res = -ENOMEM;
        goto err_nomem;
    }
    
    /* Allocate buffer */
    new_packet->len = len_tot;
    INIT_LIST_HEAD(&new_packet->list);
    if((new_packet->data = kzalloc(len_tot, GFP_KERNEL | GFP_DMA)) == NULL)
    {
        res = -ENOMEM;
        goto err_buf;
    }

    /* Copy and add to queue */
    if(copy_from_user(new_packet->data + PACKET_LATENCY, buf, len) == 0)
    {
        tx_queue_len += (len_tot + sizeof(packet_t));
        list_add_tail(&new_packet->list, &tx_queue);
        return len;
    }
    else res = -EAGAIN;
    
    kfree(new_packet->data);
  err_buf:
    kfree(new_packet);
  err_nomem:
    pr_warn_once(NAME ": tx packet memory allocation error\n");
    return res;
}

/* Access structure for /dev */
static const struct file_operations file_ops =
{
    .owner = THIS_MODULE,   
    .read = packet_read,
    .write = packet_write,
    .open = device_open,
    .release = device_close
};

/* Called when SPI driver is registered */
static int driver_probe(struct spi_device *spi)
{
    int res = 0;
    spi_data_t *data = NULL;
    struct device *dev_ret = NULL;

    /* Allocate SPI data */
    if ((data = devm_kzalloc(&spi->dev, sizeof(spi_data_t), GFP_KERNEL)) == NULL ||
        (data->rx = devm_kzalloc(&spi->dev, PACKET_TM_MAX_LEN, GFP_KERNEL | GFP_DMA)) == NULL ||
        kfifo_alloc(&rx_fifo, RX_FIFO_LEN, GFP_KERNEL) != 0)
    {
        dev_err(&spi->dev, "failed to allocate spi_data\n");
        return -ENOMEM;
    }

    /* Initialize SPI data */
    spi_message_init(&data->msg);
    spi_message_add_tail(&data->xfer, &data->msg);
    data->spi = spi;
    data->busy = false;
    data->msg.complete = spi_xfer_complete;
    data->msg.context = data;
    data->xfer.speed_hz = SPI_FREQ;
    data->xfer_done = &xfer_done;
    data->tx_queue = &tx_queue;
    data->rx_fifo = &rx_fifo;
    spi_set_drvdata(spi, data);

    /* Allocate device region */
    if((res = alloc_chrdev_region(&data->dev_num, 0, 1, DEVICE)) != 0)
    {
        dev_err(&spi->dev, "alloc_chrdev_region returned error %d\n", res);
        goto err_init;
    }

    /* Create /sys/class region */
    if(IS_ERR(data->cl = class_create(THIS_MODULE, DEVICE)))
    {
        res = PTR_ERR(data->cl);
        dev_err(&spi->dev, "class_create returned error %d\n", res);
        goto err_class;
    }

    /* Initialize device */
    if (IS_ERR(dev_ret = device_create(data->cl, NULL, data->dev_num, NULL, DEVICE)))
    {
        res = PTR_ERR(dev_ret);
        dev_err(&spi->dev, "device_create returned error %d\n", res);
        goto err_dev_create;
    }

    /* Create /dev region */
    cdev_init(&data->fs_dev, &file_ops);
    if ((res = cdev_add(&data->fs_dev, data->dev_num, 1)) != 0)
    {
        dev_err(&spi->dev, "cdev_add returned error %d\n", res);
        goto err_dev_add;
    }

    /* Request IRQ (pre-configured in click_spi.dts) */
    if ((res = request_irq(spi->irq, interrupt_handler, 0, NAME, data)) != 0)
    {
        dev_err(&spi->dev, "request_irq returned error %d for irq %d\n", res, spi->irq);
    }

    /* Success */
    else
    {
        dev_notice(&spi->dev, "loaded\n");
        return res;
    }

    cdev_del(&data->fs_dev);
  err_dev_add:
    device_destroy(data->cl, data->dev_num);
  err_dev_create:
    class_destroy(data->cl);
  err_class:
    unregister_chrdev_region(data->dev_num, 1);
  err_init:
    kfifo_free(&rx_fifo);
    return res;
}

/* Called when SPI driver is removed */
static int driver_remove(struct spi_device *spi)
{
    spi_data_t *data = spi_get_drvdata(spi);
    free_irq(spi->irq, data);
    cdev_del(&data->fs_dev);
    device_destroy(data->cl, data->dev_num);
    class_destroy(data->cl);
    unregister_chrdev_region(data->dev_num, 1);
    kfifo_free(&rx_fifo);
    tx_queue_clear();
    dev_notice(&spi->dev, "unloaded\n");
    return 0;
}

/* Device tree identifier (as in click_spi.dts) */
static const struct of_device_id click_spi_dt[] = {
    { .compatible = "mit,click_spi" }, {}
};
MODULE_DEVICE_TABLE(of, click_spi_dt);

/* Driver registration structure */
static struct spi_driver click_spi_driver = {
    .driver = {
        .name = NAME,
        .of_match_table = click_spi_dt,
     },
    .probe = driver_probe,
    .remove = driver_remove,
};
module_spi_driver(click_spi_driver);

MODULE_AUTHOR("ondrej@mit.edu");
MODULE_DESCRIPTION("CLICK SPI driver");
MODULE_LICENSE("GPL and additional rights");
