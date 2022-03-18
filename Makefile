PWD         := $(shell pwd)
KVERSION    := $(shell uname -r)
KERNEL_DIR  ?= /lib/modules/$(KVERSION)/build

obj-m += ch341-core.o
obj-m += i2c-ch341.o
obj-m += gpio-ch341.o
obj-m += spi-ch341.o

all:
	make -C $(KERNEL_DIR) M=$(PWD) modules
clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean
