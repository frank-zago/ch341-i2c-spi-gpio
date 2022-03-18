// SPDX-License-Identifier: GPL-2.0
/*
 * Core driver for the CH341A, CH341B and CH341T in I2C/SPI/GPIO
 * mode. There are cell drivers available for I2C and GPIO. SPI is not
 * yet supported.
 *
 * Copyright 2022, Frank Zago
 * Copyright (c) 2017 Gunar Schorcht (gunar@schorcht.net)
 * Copyright (c) 2016 Tse Lun Bien
 * Copyright (c) 2014 Marco Gittler
 * Copyright (c) 2006-2007 Till Harbaum (Till@Harbaum.org)
 */

#include <linux/kernel.h>
#include "ch341.h"
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>

static const struct mfd_cell ch341_devs[] = {
	{ .name = "ch341-gpio", },
	{ .name = "ch341-i2c", },
};

static int ch341_usb_probe(struct usb_interface *iface,
			   const struct usb_device_id *usb_id)
{
	struct usb_endpoint_descriptor *bulk_out;
	struct usb_endpoint_descriptor *bulk_in;
	struct usb_endpoint_descriptor *intr_in;
	struct ch341_ddata *ddata;
	int ret;

	ddata = devm_kzalloc(&iface->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->usb_dev = interface_to_usbdev(iface);
	mutex_init(&ddata->usb_lock);

	ret = usb_find_common_endpoints(iface->cur_altsetting, &bulk_in,
					&bulk_out, &intr_in, NULL);
	if (ret) {
		dev_err(&iface->dev, "Could not find all endpoints\n");
		return -ENODEV;
	}

	ddata->ep_in = bulk_in->bEndpointAddress;
	ddata->ep_out = bulk_out->bEndpointAddress;
	ddata->ep_intr = intr_in->bEndpointAddress;
	ddata->ep_intr_interval = intr_in->bInterval;

	usb_set_intfdata(iface, ddata);

	ret = devm_mfd_add_devices(&iface->dev, PLATFORM_DEVID_AUTO, ch341_devs,
				   ARRAY_SIZE(ch341_devs), NULL, 0, NULL);
	if (ret)
		dev_err(&iface->dev, "Failed to add child devices\n");

	return ret;
}

static void ch341_usb_disconnect(struct usb_interface *usb_if)
{

}

static const struct usb_device_id ch341_usb_table[] = {
	{ USB_DEVICE(0x1a86, 0x5512) },
	{ }
};
MODULE_DEVICE_TABLE(usb, ch341_usb_table);

static struct usb_driver ch341_usb_driver = {
	.name       = "ch341-mfd",
	.id_table   = ch341_usb_table,
	.probe      = ch341_usb_probe,
	.disconnect = ch341_usb_disconnect,
};
module_usb_driver(ch341_usb_driver);

MODULE_AUTHOR("Frank Zago <frank@zago.net>");
MODULE_DESCRIPTION("CH341 USB to I2C/SPI/GPIO adapter");
MODULE_LICENSE("GPL");
