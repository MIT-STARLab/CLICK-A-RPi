modname = click_spi

obj-m := $(modname).o
$(modname)-y := main.o driver.o

all:
	$(MAKE) -C $(LINUX_DIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(LINUX_DIR) M=$(PWD) clean
symbols:
	readelf -s $(modname).ko | grep -oP "(?<= UND ).*" > symbols.txt
test:
	$(CC) test_write.c crc.c -o test_tlm
