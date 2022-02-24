PWD         := $(shell pwd)
KVERSION    := $(shell uname -r)
KERNEL_DIR  ?= /lib/modules/$(KVERSION)/build

obj-m       := ch341-buses.o

ch341-buses-objs := ch341-core.o ch341-i2c.o ch341-gpio.o ch341-spi.o

all:
	make -C $(KERNEL_DIR) M=$(PWD) modules
clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean
