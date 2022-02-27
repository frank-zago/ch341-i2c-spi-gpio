// SPDX-License-Identifier: GPL-2.0
/*
 * I2C interface for the CH341A, CH341B and CH341T.
 *
 * Copyright 2021, Frank Zago
 * Copyright (c) 2016 Tse Lun Bien
 * Copyright (c) 2014 Marco Gittler
 * Copyright (C) 2006-2007 Till Harbaum (Till@Harbaum.org)
 */

#include "ch341.h"

/* Support not yet implemented */
#define CH341_I2C_LOW_SPEED 0      /* 20kHz */
#define CH341_I2C_STANDARD_SPEED 1 /* 100kHz */
#define CH341_I2C_FAST_SPEED 2     /* 400kHz */
#define CH341_I2C_HIGH_SPEED 3     /* 750kHz */

#define CH341_CMD_I2C_STREAM 0xAA
#define CH341_CMD_I2C_STM_END 0x00

#define CH341_CMD_I2C_STM_STA 0x74
#define CH341_CMD_I2C_STM_STO 0x75
#define CH341_CMD_I2C_STM_OUT 0x80
#define CH341_CMD_I2C_STM_IN 0xC0
#define CH341_CMD_I2C_STM_SET 0x60
#define CH341_CMD_I2C_STM_MS  0x50

#define CH341_CTRL_VENDOR_VERSION 0x5F	 /* version of chip */
#define CH341_CTRL_VENDOR_READ_TYPE 0XC0 /* vendor control read */

/* Append a write command to the current request. A set of 32-byte
 * packets is filled. Each packet starts with STREAM and finishes with
 * END, and contains an OUT field, leaving up to 29 bytes of data. The
 * first packet must also include a START and the device address.
 */
static int append_write(struct ch341_device *dev, const struct i2c_msg *msg)
{
	u8 *out = dev->i2c_buf;
	int len;
	u8 *p;
	bool start_done = false;

	len = msg->len;
	p = msg->buf;

	while (len) {
		int to_write;
		int avail;

		if (dev->idx_out % SEG_SIZE) {
			/* Finish current packet, and advance to the next one */
			out[dev->idx_out++] = CH341_CMD_I2C_STM_END;
			dev->out_seg++;
			dev->idx_out = dev->out_seg * SEG_SIZE;

			if (dev->out_seg == SEG_COUNT)
				return -E2BIG;
		}

		out[dev->idx_out++] = CH341_CMD_I2C_STREAM;

		/* account for stream start and end */
		avail = SEG_SIZE - 3;

		if (!start_done) {
			/* Each message has a start */
			out[dev->idx_out++] = CH341_CMD_I2C_STM_STA;

			avail -= 2; /* room for STA and device address */
		}

		to_write = min_t(int, len, avail);

		if (!start_done) {
			out[dev->idx_out++] = CH341_CMD_I2C_STM_OUT | (to_write + 1);
			out[dev->idx_out++] = msg->addr << 1;

			start_done = true;
		} else {
			out[dev->idx_out++] = CH341_CMD_I2C_STM_OUT | to_write;
		}

		memcpy(&out[dev->idx_out], p, to_write);
		dev->idx_out += to_write;
		len -= to_write;
		p += to_write;
	}

	return 0;
}

/* Append a read command to the request. It usually follows a write
 * command. When that happens, the driver will attempt to concat the
 * read command into the same packet.  Each read command, of up to 32
 * bytes, must be written to a new packet. It is not possible to
 * concat them.
 */
static int append_read(struct ch341_device *dev, const struct i2c_msg *msg)
{
	u8 *out = dev->i2c_buf;
	bool start_done = false;
	int len;

	len = msg->len;

	while (len) {
		int to_read;

		if (dev->idx_out % SEG_SIZE) {
			if (!start_done &&
			    (dev->idx_out % SEG_SIZE) <  (SEG_SIZE - 7)) {
				/* There's enough left for a read */
			} else {
				/* Finish current packet, and advance to the next one */
				out[dev->idx_out++] = CH341_CMD_I2C_STM_END;
				dev->out_seg++;
				dev->idx_out = dev->out_seg * SEG_SIZE;

				if (dev->out_seg == SEG_COUNT)
					return -E2BIG;

				out[dev->idx_out++] = CH341_CMD_I2C_STREAM;
			}
		} else {
			out[dev->idx_out++] = CH341_CMD_I2C_STREAM;
		}

		if (!start_done) {
			/* Each message has a start */
			out[dev->idx_out++] = CH341_CMD_I2C_STM_STA;
			out[dev->idx_out++] = CH341_CMD_I2C_STM_OUT | 1;
			out[dev->idx_out++] = msg->addr << 1 | 1;

			start_done = true;
		}

		/* Apparently the last command must be an STM_IN to
		 * read the last byte. Without it, the adapter gets
		 * lost.
		 */
		to_read = min_t(int, len, 32);
		len -= to_read;
		if (len == 0) {
			if (to_read > 1)
				out[dev->idx_out++] = CH341_CMD_I2C_STM_IN | (to_read - 1);
			out[dev->idx_out++] = CH341_CMD_I2C_STM_IN;
		} else {
			out[dev->idx_out++] = CH341_CMD_I2C_STM_IN | to_read;
		}
	}

	return 0;
}

static int ch341_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct ch341_device *dev = adapter->algo_data;
	int retval;
	int i;
	u8 *out = dev->i2c_buf;
	int actual;

	/* Prepare the request */
	dev->idx_out = 0;
	dev->out_seg = 0;

	for (i = 0; i != num; i++) {
		if (msgs[i].flags & I2C_M_RD)
			retval = append_read(dev, &msgs[i]);
		else
			retval = append_write(dev, &msgs[i]);

		if (retval)
			return retval;
	}

	/* Finish the last packet */
	if (SEG_SIZE - (dev->idx_out % SEG_SIZE) < 2) {
		out[dev->idx_out++] = CH341_CMD_I2C_STM_END;

		dev->out_seg++;
		if (dev->out_seg == SEG_COUNT)
			return -E2BIG;

		dev->idx_out = dev->out_seg * SEG_SIZE;

		out[dev->idx_out++] = CH341_CMD_I2C_STREAM;
	}

	out[dev->idx_out++] = CH341_CMD_I2C_STM_STO;
	out[dev->idx_out++] = CH341_CMD_I2C_STM_END;

	dev_dbg(&dev->adapter.dev, "bulk_out request with %d bytes\n",
		dev->idx_out);

	mutex_lock(&dev->usb_lock);

	/* Issue the request */
	retval = usb_bulk_msg(dev->usb_dev,
			      usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
			      dev->i2c_buf, dev->idx_out, &actual, DEFAULT_TIMEOUT);
	if (retval < 0) {
		mutex_unlock(&dev->usb_lock);
		return retval;
	}

	for (i = 0; i != num; i++) {
		if (!(msgs[i].flags & I2C_M_RD))
			continue;

		retval = usb_bulk_msg(dev->usb_dev,
				      usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
				      dev->i2c_buf, msgs[i].len, &actual, DEFAULT_TIMEOUT);

		if (retval) {
			mutex_unlock(&dev->usb_lock);
			return retval;
		}

		if (actual != msgs[i].len) {
			mutex_unlock(&dev->usb_lock);
			return -EIO;
		}

		memcpy(msgs[i].buf, dev->i2c_buf, actual);
	}

	mutex_unlock(&dev->usb_lock);

	return num;
}

static u32 ch341_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm ch341_i2c_algorithm = {
	.master_xfer = ch341_i2c_xfer,
	.functionality = ch341_i2c_func,
};

void ch341_i2c_remove(struct ch341_device *dev)
{
	if (!dev->i2c_init)
		return;

	i2c_del_adapter(&dev->adapter);
}

int ch341_i2c_init(struct ch341_device *dev)
{
	int retval;
	int actual;

	dev->adapter.owner = THIS_MODULE;
	dev->adapter.class = I2C_CLASS_HWMON;
	dev->adapter.algo = &ch341_i2c_algorithm;
	dev->adapter.algo_data = dev;
	snprintf(dev->adapter.name, sizeof(dev->adapter.name),
		 "CH341 I2C USB bus %03d device %03d",
		 dev->usb_dev->bus->busnum, dev->usb_dev->devnum);

	dev->adapter.dev.parent = &dev->iface->dev;

	/* Set ch341 i2c speed */
	dev->i2c_buf[0] = CH341_CMD_I2C_STREAM;
	dev->i2c_buf[1] = CH341_CMD_I2C_STM_SET | CH341_I2C_STANDARD_SPEED;
	dev->i2c_buf[2] = CH341_CMD_I2C_STM_END;
	mutex_lock(&dev->usb_lock);
	retval = usb_bulk_msg(dev->usb_dev,
			      usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
			      dev->i2c_buf, 3, &actual, DEFAULT_TIMEOUT);
	mutex_unlock(&dev->usb_lock);
	if (retval < 0) {
		dev_err(&dev->iface->dev, "Cannot set I2C speed\n");
		return -EIO;
	}

	i2c_add_adapter(&dev->adapter);
	dev->i2c_init = true;

	return 0;
}
