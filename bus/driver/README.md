# CLICK SPI driver
Source for the kernel driver handling communication with the VNC2L over SPI.

- `driver.h` contains some configuration parameters (SPI freq, max memory allocation, etc.) and type definitions
- `driver.c` implements all SPI communication logic
- `main.c` does kernel module init and handles queueing and reading of packets
- `click_spi.dts` configures the RPi device tree to use the driver on SPI0 CE0 and sets the interrupt GPIO
- `test_write.c` shows an example program used to write a telemetry packet to the driver

## Packet exchange with user-space software
The driver creates a new [character device](https://linux-kernel-labs.github.io/refs/heads/master/labs/device_drivers.html) called `/dev/bct` which is used to read and write packets.

Any process can open the device for writing (O_WRONLY) and write raw packets, which will be queued in memory using a [kernel linked list](https://www.kernel.org/doc/html/v4.14/core-api/kernel-api.html). Each packet must be written using a single write call. Then, every time the VNC2L triggers an interrupt, the driver will transmit the first packet in the list. The maximum queue memory size is set in `driver.h` via `MAX_TX_QUEUE_LEN` and has no upper limit apart from available RAM. Currently it's set to 2^26 bytes (67 MB). If the limit is reached, the write function will return -12 (ENOMEM).

Only one process can open the device for reading (O_RDWR/O_RDONLY). The read calls are blocking, and will return when the requested amount of bytes are available. All SPI transfers append received data into a [kernel fifo](https://www.kernel.org/doc/htmldocs/kernel-api/kfifo.html) buffer and all `/dev/bct` reads will pull from this fifo. The fifo memory size is set via `RX_FIFO_LEN` and is limited to 2^22 bytes (4 MB) by kfifo implementation.

## Hardware configuration
The configurable HW parameters include the SPI frequency (via `SPI_FREQ`) and the GPIO pin which is used to handle the interrupt from VNC2L, set in `click_spi.dts`. The GPIO pin can be changed from its default (GPIO25) in `/boot/config.txt` by passing `dtoverlay=click_spi,irq=25`.

## Building (with buildroot)
The CLICK image generation tool will automatically pull this git repo and compile the kernel driver for the image.

## Building (without buildroot)
There are two steps to build the driver:
1. Compiling the kernel module
2. Compiling the device tree overlay

The module can be cross-compiled on any linux host with the following packages:
```
sudo apt install git bc bison flex libssl-dev make libc6-dev libncurses5-dev crossbuild-essential-armhf
```
First prepare a Linux source directory:
```
git clone --depth=1 https://github.com/raspberrypi/linux
cd linux
export MAKE="make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-"
KERNEL="kernel7"
$MAKE bcm2709_defconfig
$MAKE prepare
$MAKE modules_prepare
```
Then, navigate to the driver source directory and run:
```
make LINUX_DIR="<path to linux repo>"
```
The `click_spi.dts` file has to be compiled using the [device-tree-compiler](https://packages.debian.org/buster/device-tree-compiler). The following command will build it:
```
dtc -O dtb -o click_spi.dtbo -b 0 -@ click_spi.dts
```
The output `click_spi.dtbo` has to be put into `/boot/overlays/`. To enable it, `dtoverlay=click_spi` has to be added to `/boot/config.txt`.
