modname = click_spi

obj-m := $(modname).o
$(modname)-y := main.o driver.o

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C .. M=$(shell pwd) modules
	# scp $(modname).ko click:~/flight/spi_driver/
clean:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C .. M=$(shell pwd) clean
