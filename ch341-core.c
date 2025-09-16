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

#define CH341_USB_EP_INT   0x01
#define CH341_USB_EP_BULK  0x02

#define USB_DIR(d) \
	((d) == USB_DIR_OUT ? 0 : \
	 (d) == USB_DIR_IN ? 1  : \
	 0)

struct ch341_ep_map {
	u8 bulk[2]; /* [O] out, [1] in */
	u8 intr[2]; /* [0] out, [1] in */
};

static const struct mfd_cell ch341_devs[] = {
	{ .name = "ch341-gpio", },
	{ .name = "ch341-i2c", },
};

/* helper for probing */
static const struct usb_endpoint_descriptor *
ch341_find_ep(const struct usb_host_interface *alt, u8 dir, u8 xfertype,
              const struct ch341_ep_map *map)
{
	for (int i = 0; i < alt->desc.bNumEndpoints; i++) {
		const struct usb_endpoint_descriptor *ep = &alt->endpoint[i].desc;

		if (dir == USB_DIR_IN &&
		    !usb_endpoint_dir_in(ep))
			continue;
		if (dir == USB_DIR_OUT &&
		    !usb_endpoint_dir_out(ep))
			continue;

		if (map) {
			u8 want = 0;

			switch (xfertype) {
			case USB_ENDPOINT_XFER_BULK:
				want = map->bulk[USB_DIR(dir)];
				break;
			case USB_ENDPOINT_XFER_INT:
				want = map->intr[USB_DIR(dir)];
				break;
			case USB_ENDPOINT_XFER_CONTROL:
				break;
			}

			/* "don’t care" */
			if (want && ep->bEndpointAddress != want)
				continue;
		}

		if ((xfertype == USB_ENDPOINT_XFER_BULK    && usb_endpoint_xfer_bulk(ep)) ||
		    (xfertype == USB_ENDPOINT_XFER_INT     && usb_endpoint_xfer_int(ep))  ||
		    (xfertype == USB_ENDPOINT_XFER_CONTROL && usb_endpoint_xfer_control(ep)))
			return ep;

	}
	return NULL;
}

static int ch341_usb_probe(struct usb_interface *iface,
			   const struct usb_device_id *usb_id)
{
	const struct ch341_ep_map *map = (const void *)usb_id->driver_info;
	const struct usb_host_interface *alt = iface->cur_altsetting;
	const struct usb_endpoint_descriptor *bulk_out = NULL;
	const struct usb_endpoint_descriptor *bulk_in = NULL;
	const struct usb_endpoint_descriptor *intr_in = NULL;
	struct ch341_ddata *ddata;
	int ret;

	if (alt->desc.bInterfaceClass != USB_CLASS_VENDOR_SPEC) {
		dev_warn(&iface->dev, "Not a vendor-specific interface; skip\n");
		return -ENODEV;
	}

	ddata = devm_kzalloc(&iface->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->usb_dev = interface_to_usbdev(iface);
	mutex_init(&ddata->usb_lock);

	bulk_in  = ch341_find_ep(alt, USB_DIR_IN, USB_ENDPOINT_XFER_BULK, map);
	bulk_out = ch341_find_ep(alt, USB_DIR_OUT, USB_ENDPOINT_XFER_BULK, map);
	intr_in  = ch341_find_ep(alt, USB_DIR_IN, USB_ENDPOINT_XFER_INT, map);
	if (!bulk_in || !bulk_out || !intr_in) {
		dev_err(&iface->dev, "Missing some required endpoints (bulk-in/out or int-in)\n");
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

static const struct ch341_ep_map map_1a86_5512 = {
	.bulk[USB_DIR(USB_DIR_IN)] = CH341_USB_EP_BULK | USB_DIR_IN,
	.bulk[USB_DIR(USB_DIR_OUT)] = CH341_USB_EP_BULK | USB_DIR_OUT,
	.intr[USB_DIR(USB_DIR_IN)] = CH341_USB_EP_INT | USB_DIR_IN,
};

static const struct usb_device_id ch341_usb_table[] = {
	{
		USB_DEVICE_AND_INTERFACE_INFO(0x1a86, 0x5512,
		USB_CLASS_VENDOR_SPEC, 0x01, 0x02), /* bInterfaceSubClass, bInterfaceProtocol */
		.driver_info = (kernel_ulong_t)&map_1a86_5512, // optional, but nice to set
	},
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
