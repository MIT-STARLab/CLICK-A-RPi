modname = click_spi

obj-m := $(modname).o
$(modname)-y := main.o driver.o

all:
	$(MAKE) -C $(LINUX_DIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(LINUX_DIR) M=$(PWD) clean
	