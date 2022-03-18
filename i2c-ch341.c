// SPDX-License-Identifier: GPL-2.0
/*
 * I2C cell driver for the CH341A, CH341B and CH341T.
 *
 * Copyright 2022, Frank Zago
 * Copyright (c) 2016 Tse Lun Bien
 * Copyright (c) 2014 Marco Gittler
 * Copyright (C) 2006-2007 Till Harbaum (Till@Harbaum.org)
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include "ch341.h"
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/usb.h>

/* I2C bus speed. Speed selection is not implemented. */
#define CH341_I2C_20KHZ  0
#define CH341_I2C_100KHZ 1
#define CH341_I2C_400KHZ 2
#define CH341_I2C_750KHZ 3

/* I2C chip commands */
#define CH341_CMD_I2C_STREAM 0xAA
#define CH341_CMD_I2C_STM_END 0x00

#define CH341_CMD_I2C_STM_STA 0x74
#define CH341_CMD_I2C_STM_STO 0x75
#define CH341_CMD_I2C_STM_OUT 0x80
#define CH341_CMD_I2C_STM_IN 0xC0
#define CH341_CMD_I2C_STM_SET 0x60

/*
 * The maximum request size is 4096 bytes, both for reading and
 * writing, split in up to 128 32-byte segments, which includes the
 * CH341 commands and the I2C data.
 */
#define SEG_COUNT 128

/*
 * Limit the transfer size that can be written. 4KiB is the maximum
 * size of the whole buffer, but it must include all the command
 * delimiters. 3KiB sounds reasonable.
 */
#define MAX_RW_LENGTH 3072

struct ch341_i2c {
	struct i2c_adapter adapter;

	/* I2C request and response state */
	int idx_out;		/* current offset in buf */
	int out_seg;		/* current segment */
	u8 *i2c_buf;
};

/*
 * Special case the no data transfer (SMBUS quick message), since some
 * parameters are a bit different than the normal case.
 */
static int no_data_xfer(struct i2c_adapter *adapter, const struct i2c_msg *msg)
{
	struct ch341_i2c *dev = i2c_get_adapdata(adapter);
	struct ch341_ddata *ddata = adapter->algo_data;
	u8 *out = dev->i2c_buf;
	int actual;
	int ret;

	out[0] = CH341_CMD_I2C_STREAM;
	out[1] = CH341_CMD_I2C_STM_STA;
	out[2] = CH341_CMD_I2C_STM_OUT;
	out[3] = msg->addr << 1 | (msg->flags & I2C_M_RD);
	out[4] = CH341_CMD_I2C_STM_IN;
	out[5] = CH341_CMD_I2C_STM_STO;
	out[6] = CH341_CMD_I2C_STM_END;

	mutex_lock(&ddata->usb_lock);

	/* Issue the request */
	ret = usb_bulk_msg(ddata->usb_dev,
			   usb_sndbulkpipe(ddata->usb_dev, ddata->ep_out),
			   dev->i2c_buf, 7, &actual, DEFAULT_TIMEOUT_MS);
	if (ret < 0) {
		mutex_unlock(&ddata->usb_lock);
		return ret;
	}

	ret = usb_bulk_msg(ddata->usb_dev,
			   usb_rcvbulkpipe(ddata->usb_dev, ddata->ep_in),
			   dev->i2c_buf, 2, &actual,
			   DEFAULT_TIMEOUT_MS);
	if (ret < 0) {
		mutex_unlock(&ddata->usb_lock);
		return ret;
	}

	mutex_unlock(&ddata->usb_lock);

	if (actual != 2 || dev->i2c_buf[0] & 0x80)
		return -ETIMEDOUT;

	return 1;
}

/*
 * Append a write command to the current request. A set of 32-byte
 * packets is filled. Each packet starts with STREAM and finishes with
 * END, and contains an OUT field, leaving up to 29 bytes of data. The
 * first packet must also include a START and the device address.
 */
static int append_write(struct ch341_i2c *dev, const struct i2c_msg *msg)
{
	bool start_done = false;
	u8 *out = dev->i2c_buf;
	int len;
	u8 *p;

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

/*
 * Append a read command to the request. It usually follows a write
 * command. When that happens, the driver will attempt to concat the
 * read command into the same packet.  Each read command, of up to 32
 * bytes, must be written to a new packet. It is not possible to
 * concat them.
 */
static int append_read(struct ch341_i2c *dev, const struct i2c_msg *msg)
{
	bool start_done = false;
	u8 *out = dev->i2c_buf;
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
	struct ch341_i2c *dev = i2c_get_adapdata(adapter);
	struct ch341_ddata *ddata = adapter->algo_data;
	u8 *out = dev->i2c_buf;
	int actual;
	int ret;
	int i;

	/* Special case */
	if (num == 1 && msgs[0].len == 0)
		return no_data_xfer(adapter, &msgs[0]);

	/* Prepare the request */
	dev->idx_out = 0;
	dev->out_seg = 0;

	for (i = 0; i != num; i++) {
		if (msgs[i].flags & I2C_M_RD)
			ret = append_read(dev, &msgs[i]);
		else
			ret = append_write(dev, &msgs[i]);

		if (ret)
			return ret;
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

	mutex_lock(&ddata->usb_lock);

	/* Issue the request */
	ret = usb_bulk_msg(ddata->usb_dev,
			   usb_sndbulkpipe(ddata->usb_dev, ddata->ep_out),
			   dev->i2c_buf, dev->idx_out, &actual, DEFAULT_TIMEOUT_MS);
	if (ret < 0) {
		mutex_unlock(&ddata->usb_lock);
		return ret;
	}

	for (i = 0; i != num; i++) {
		if (!(msgs[i].flags & I2C_M_RD))
			continue;

		ret = usb_bulk_msg(ddata->usb_dev,
				  usb_rcvbulkpipe(ddata->usb_dev, ddata->ep_in),
				  dev->i2c_buf, msgs[i].len, &actual,
				  DEFAULT_TIMEOUT_MS);

		if (ret) {
			mutex_unlock(&ddata->usb_lock);
			return ret;
		}

		if (actual != msgs[i].len) {
			mutex_unlock(&ddata->usb_lock);
			return -EIO;
		}

		memcpy(msgs[i].buf, dev->i2c_buf, actual);
	}

	mutex_unlock(&ddata->usb_lock);

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

static const struct i2c_adapter_quirks ch341_i2c_quirks = {
	.max_read_len = MAX_RW_LENGTH,
	.max_write_len = MAX_RW_LENGTH,
};

static int ch341_i2c_probe(struct platform_device *pdev)
{
	struct ch341_ddata *ddata = dev_get_drvdata(pdev->dev.parent);
	struct ch341_i2c *ch341_i2c;
	int actual;
	int ret;

	ch341_i2c = devm_kzalloc(&pdev->dev, sizeof(*ch341_i2c), GFP_KERNEL);
	if (ch341_i2c == NULL)
		return -ENOMEM;

	ch341_i2c->i2c_buf = devm_kzalloc(&pdev->dev, SEG_COUNT * SEG_SIZE, GFP_KERNEL);
	if (ch341_i2c->i2c_buf == NULL)
		return -ENOMEM;

	ch341_i2c->adapter.owner = THIS_MODULE;
	ch341_i2c->adapter.class = I2C_CLASS_HWMON;
	ch341_i2c->adapter.algo = &ch341_i2c_algorithm;
	ch341_i2c->adapter.algo_data = ddata;
	ch341_i2c->adapter.quirks = &ch341_i2c_quirks;
	ch341_i2c->adapter.dev.parent = &pdev->dev;
	snprintf(ch341_i2c->adapter.name, sizeof(ch341_i2c->adapter.name),
		 "CH341 I2C USB bus %03d device %03d",
		 ddata->usb_dev->bus->busnum, ddata->usb_dev->devnum);

	i2c_set_adapdata(&ch341_i2c->adapter, ch341_i2c);
	platform_set_drvdata(pdev, ch341_i2c);

	/* Set ch341 i2c speed */
	ch341_i2c->i2c_buf[0] = CH341_CMD_I2C_STREAM;
	ch341_i2c->i2c_buf[1] = CH341_CMD_I2C_STM_SET | CH341_I2C_100KHZ;
	ch341_i2c->i2c_buf[2] = CH341_CMD_I2C_STM_END;
	mutex_lock(&ddata->usb_lock);
	ret = usb_bulk_msg(ddata->usb_dev,
			   usb_sndbulkpipe(ddata->usb_dev, ddata->ep_out),
			   ch341_i2c->i2c_buf, 3, &actual, DEFAULT_TIMEOUT_MS);
	mutex_unlock(&ddata->usb_lock);

	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot set I2C speed\n");
		return ret;
	}

	return devm_i2c_add_adapter(&pdev->dev, &ch341_i2c->adapter);
}

static struct platform_driver ch341_i2c_driver = {
	.driver.name	= "ch341-i2c",
	.probe		= ch341_i2c_probe,
};
module_platform_driver(ch341_i2c_driver);

MODULE_AUTHOR("Frank Zago <frank@zago.net>");
MODULE_DESCRIPTION("CH341 USB to I2C");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ch341-i2c");
