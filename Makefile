PWD := $(shell pwd)
KVERSION    := $(shell uname -r)
KDIR        := /lib/modules/$(KVERSION)/build
DKMS       := $(shell which dkms 2> /dev/null)
MODULE_DIR  := /lib/modules/$(KVERSION)


obj-m += ch341-core.o
obj-m += i2c-ch341.o
obj-m += gpio-ch341.o
obj-m += spi-ch341.o

all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean

ifeq ($(DKMS),)  # if DKMS is not installed
install: all
	make -C $(KDIR) M=$(PWD) modules_install
	depmod
else
# Package version and name from dkms.conf
VER := $(shell sed -n 's/^PACKAGE_VERSION=\([^\n]*\)/\1/p' dkms.conf 2> /dev/null)
MODULE_NAME := $(shell sed -n 's/^PACKAGE_NAME=\([^\n]*\)/\1/p' dkms.conf 2> /dev/null)
MODULE_INSTALLED := $(shell dkms status $(MODULE_NAME))
install: all
ifneq ($(MODULE_INSTALLED),)
	echo Module $(MODULE_NAME) is installed ... uninstall it first
	make uninstall
endif
	cp -R . /usr/src/$(MODULE_NAME)-$(VER)
	dkms install -m $(MODULE_NAME) -v $(VER)
uninstall:
	dkms remove -m $(MODULE_NAME) -v $(VER) --all
	rm -rf /usr/src/$(MODULE_NAME)-$(VER)
endif
