PWD := $(shell pwd)
KVERSION    := $(shell uname -r)
ifeq ($(KERNEL_SRC),) # KERNEL_SRC is set to kernel src dir in Yocto Linux build
    KDIR    ?= /lib/modules/$(KVERSION)/build
else
    KDIR    ?= $(KERNEL_SRC)
endif
DKMS        := $(shell which dkms 2> /dev/null)
MODULE_DIR  := /lib/modules/$(KVERSION)


obj-m += ch341-core.o
obj-m += i2c-ch341.o
obj-m += gpio-ch341.o
obj-m += spi-ch341.o

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

ifeq ($(DKMS),)  # if DKMS is not installed
install: all modules_install
	depmod
else
# Package version and name from dkms.conf
VER := $(shell sed -n 's/^PACKAGE_VERSION=\([^\n]*\)/\1/p' dkms.conf 2> /dev/null)
MODULE_NAME := $(shell sed -n 's/^PACKAGE_NAME=\([^\n]*\)/\1/p' dkms.conf 2> /dev/null)
MODULE_INSTALLED := $(shell dkms status $(MODULE_NAME))
install: all
ifneq ($(MODULE_INSTALLED),)
	echo Module $(MODULE_NAME) is installed ... uninstall it first
	$(MAKE) uninstall
endif
	cp -R . /usr/src/$(MODULE_NAME)-$(VER)
	dkms install -m $(MODULE_NAME) -v $(VER)
uninstall:
	dkms remove -m $(MODULE_NAME) -v $(VER) --all
	rm -rf /usr/src/$(MODULE_NAME)-$(VER)
endif
