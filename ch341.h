/* SPDX-License-Identifier: GPL-2.0 */
/* Definitions for the CH341 driver */

#include <linux/mutex.h>
#include <linux/types.h>

#define DEFAULT_TIMEOUT_MS 1000	/* 1s USB requests timeout */

/*
 * All commands fit inside a 32-byte segment. There may be several of
 * these segments in a USB command.
 */
#define SEG_SIZE 32

struct usb_device;
struct usb_interface;

struct ch341_ddata {
	struct usb_device *usb_dev;
	struct mutex usb_lock;

	int ep_in;
	int ep_out;
	int ep_intr;
	u8 ep_intr_interval;
};
