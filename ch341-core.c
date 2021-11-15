// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the CH341A, and CH341B USB to I2C/SPI/GPIO adapter
 * Driver for the CH341T USB to I2C adapter
 *
 * Copyright 2021, Frank Zago
 * Copyright (c) 2017 Gunar Schorcht (gunar@schorcht.net)
 * Copyright (c) 2016 Tse Lun Bien
 * Copyright (c) 2014 Marco Gittler
 * Copyright (c) 2006-2007 Till Harbaum (Till@Harbaum.org)
 *
 * The full UART functionality is handled by the CH341 serial driver
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include "ch341.h"

static void ch341_usb_free_device(struct ch341_device *dev)
{
	ch341_spi_remove(dev);
	ch341_gpio_remove(dev);
	ch341_i2c_remove(dev);

	usb_set_intfdata(dev->iface, NULL);
	usb_put_dev(dev->usb_dev);

	kfree(dev);
}

static int ch341_usb_probe(struct usb_interface *iface,
			   const struct usb_device_id *usb_id)
{
	struct usb_host_endpoint *endpoints;
	struct ch341_device *dev;
	int rc;

	dev = kzalloc(sizeof(struct ch341_device), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->usb_dev = usb_get_dev(interface_to_usbdev(iface));
	dev->iface = iface;
	mutex_init(&dev->usb_lock);

	if (iface->cur_altsetting->desc.bNumEndpoints != 3) {
		rc = -EIO;
		goto free_dev;
	}

	endpoints = iface->cur_altsetting->endpoint;
	if (!usb_endpoint_is_bulk_in(&endpoints[0].desc) ||
	    !usb_endpoint_is_bulk_out(&endpoints[1].desc) ||
	    !usb_endpoint_xfer_int(&endpoints[2].desc)) {
		rc = -EIO;
		goto free_dev;
	}

	dev->ep_in = endpoints[0].desc.bEndpointAddress;
	dev->ep_out = endpoints[1].desc.bEndpointAddress;
	dev->ep_intr = endpoints[2].desc.bEndpointAddress;
	dev->ep_intr_interval = endpoints[2].desc.bInterval;

	usb_set_intfdata(iface, dev);

	rc = ch341_i2c_init(dev);
	if (rc)
		goto free_dev;

	rc = ch341_gpio_init(dev);
	if (rc)
		goto rem_i2c;

	rc = ch341_spi_init(dev);
	if (rc)
		goto rem_gpio;

	return 0;

rem_gpio:
	ch341_gpio_remove(dev);

rem_i2c:
	ch341_i2c_remove(dev);

free_dev:
	usb_put_dev(dev->usb_dev);
	kfree(dev);

	return rc;
}

static void ch341_usb_disconnect(struct usb_interface *usb_if)
{
	struct ch341_device *dev = usb_get_intfdata(usb_if);

	ch341_usb_free_device(dev);
}

static const struct usb_device_id ch341_usb_table[] = {
	{ USB_DEVICE(0x1a86, 0x5512) },
	{ }
};

static struct usb_driver ch341_usb_driver = {
	.name       = "ch341-buses",
	.id_table   = ch341_usb_table,
	.probe      = ch341_usb_probe,
	.disconnect = ch341_usb_disconnect
};

module_usb_driver(ch341_usb_driver);

MODULE_AUTHOR("Various");
MODULE_DESCRIPTION("ch341 USB to I2C/SPI/GPIO adapter");
MODULE_LICENSE("GPL v2");
