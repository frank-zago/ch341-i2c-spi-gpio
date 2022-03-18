PWD         := $(shell pwd)
KVERSION    := $(shell uname -r)
KDIR        ?= /lib/modules/$(KVERSION)/build

obj-m += ch341-core.o
obj-m += i2c-ch341.o
obj-m += gpio-ch341.o
obj-m += spi-ch341.o

all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean
