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
#include <linux/version.h>

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

/* Limit the transfer read size in one go to 32 bytes. */
#define MAX_RD_LENGTH 32

struct ch341_i2c {
	struct i2c_adapter adapter;
	u8 *i2c_buf;
};

/*
 * Check whether the device is busy. The CH341 sets a bit in the first
 * byte if it cannot process a new command yet.
 */
static int is_dev_busy(const struct i2c_adapter *adapter, const struct i2c_msg *msg)
{
	const struct ch341_i2c *dev = i2c_get_adapdata(adapter);
	struct ch341_ddata *ddata = adapter->algo_data;
	u8 *out = dev->i2c_buf;
	int actual;
	int ret;

	out[0] = CH341_CMD_I2C_STREAM;
	out[1] = CH341_CMD_I2C_STM_STA;
	out[2] = CH341_CMD_I2C_STM_OUT;
	out[3] = msg->addr << 1;
	out[4] = CH341_CMD_I2C_STM_STO;
	out[5] = CH341_CMD_I2C_STM_END;

	mutex_lock(&ddata->usb_lock);

	ret = usb_bulk_msg(ddata->usb_dev,
			   usb_sndbulkpipe(ddata->usb_dev, ddata->ep_out),
			   dev->i2c_buf, 6, &actual, DEFAULT_TIMEOUT_MS);
	if (ret == 0) {
		ret = usb_bulk_msg(ddata->usb_dev,
				   usb_rcvbulkpipe(ddata->usb_dev, ddata->ep_in),
				   dev->i2c_buf, SEG_SIZE, &actual,
				   DEFAULT_TIMEOUT_MS);
	}

	mutex_unlock(&ddata->usb_lock);

	if (ret == 0 && (actual != 1 || dev->i2c_buf[0] & 0x80))
		ret = -ETIMEDOUT;

	return ret;
}

static int ch341_i2c_write(const struct i2c_adapter *adapter, const struct i2c_msg *msg)
{
	const struct ch341_i2c *dev = i2c_get_adapdata(adapter);
	struct ch341_ddata *ddata = adapter->algo_data;
	const u8 *src = msg->buf;
	bool start_done = false;
	bool stop_done = false;
	unsigned int len;
	int actual;
	int ret;

	ret = is_dev_busy(adapter, msg);
	if (ret)
		return ret;

	len = msg->len;

	while (!stop_done) {
		u8 *out = dev->i2c_buf;
		unsigned int to_write;

		*out++ = CH341_CMD_I2C_STREAM;

		if (start_done)  {
			to_write = min_t(unsigned int, len, SEG_SIZE - 3);
			if (to_write)
				*out++ = CH341_CMD_I2C_STM_OUT | to_write;
		} else {
			to_write = min_t(unsigned int, len, SEG_SIZE - 5);
			*out++ = CH341_CMD_I2C_STM_STA;
			*out++ = CH341_CMD_I2C_STM_OUT | (to_write + 1);
			*out++ = msg->addr << 1;

			start_done = true;
		}

		memcpy(out, src, to_write);
		out += to_write;
		src += to_write;
		len -= to_write;

		/* Corner case. There may not be enough room to add
		 * the i2c STOP in this packet. Add it to the next
		 * (empty) packet.
		 */
		if (len == 0 && ((out - dev->i2c_buf) < (SEG_SIZE - 1))) {
			*out++ = CH341_CMD_I2C_STM_STO;
			stop_done = true;
		}

		*out++ = CH341_CMD_I2C_STM_END;

		mutex_lock(&ddata->usb_lock);

		ret = usb_bulk_msg(ddata->usb_dev,
				   usb_sndbulkpipe(ddata->usb_dev, ddata->ep_out),
				   dev->i2c_buf, out - dev->i2c_buf, &actual, DEFAULT_TIMEOUT_MS);

		mutex_unlock(&ddata->usb_lock);

		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ch341_i2c_read(const struct i2c_adapter *adapter, const struct i2c_msg *msg)
{
	struct ch341_i2c *dev = i2c_get_adapdata(adapter);
	struct ch341_ddata *ddata = adapter->algo_data;
	u8 *dst = msg->buf;
	unsigned int len;
	int actual;
	int ret;

	len = msg->len;

	while (len) {
		u8 *out = dev->i2c_buf;
		unsigned int to_read;

		to_read = min_t(unsigned int, len, SEG_SIZE - 1);

		mutex_lock(&ddata->usb_lock);

		*out++ = CH341_CMD_I2C_STREAM;
		*out++ = CH341_CMD_I2C_STM_STA;
		*out++ = CH341_CMD_I2C_STM_OUT;  /* request status byte */
		*out++ = (msg->addr << 1) | 1;

		/* The last command must be an STM_IN to read
		 * the last byte. Without it, the adapter gets
		 * lost.
		 */
		if (to_read > 1)
			*out++ = CH341_CMD_I2C_STM_IN | (to_read - 1);
		*out++ = CH341_CMD_I2C_STM_IN;

		*out++ = CH341_CMD_I2C_STM_STO;
		*out++ = CH341_CMD_I2C_STM_END;

		/* Issue the request */
		ret = usb_bulk_msg(ddata->usb_dev,
				   usb_sndbulkpipe(ddata->usb_dev, ddata->ep_out),
				   dev->i2c_buf, out - dev->i2c_buf, &actual, DEFAULT_TIMEOUT_MS);
		if (ret < 0) {
			mutex_unlock(&ddata->usb_lock);
			return ret;
		}

		ret = usb_bulk_msg(ddata->usb_dev,
				   usb_rcvbulkpipe(ddata->usb_dev, ddata->ep_in),
				   dev->i2c_buf, to_read + 1, &actual,
				   DEFAULT_TIMEOUT_MS);
		if (ret) {
			mutex_unlock(&ddata->usb_lock);
			return ret;
		}

		if (actual < 1 || dev->i2c_buf[0] & 0x80) {
			mutex_unlock(&ddata->usb_lock);
			return -ETIMEDOUT;
		}

		memcpy(dst, &dev->i2c_buf[1], to_read);

		mutex_unlock(&ddata->usb_lock);

		len -= to_read;
		dst += to_read;
	}

	return 0;
}

static int ch341_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	int ret;
	int i;

	for (i = 0; i != num; i++) {
		struct i2c_msg *msg = &msgs[i];

		if (msg->flags & I2C_M_RD)
			ret = ch341_i2c_read(adapter, msg);
		else
			ret = ch341_i2c_write(adapter, msg);

		if (ret)
			return ret;
	}

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
	.max_read_len = MAX_RD_LENGTH,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,13,0)
static void devm_i2c_del_adapter(void *adapter)
{
	i2c_del_adapter(adapter);
}
#endif

static int ch341_i2c_probe(struct platform_device *pdev)
{
	struct ch341_ddata *ddata = dev_get_drvdata(pdev->dev.parent);
	struct ch341_i2c *ch341_i2c;
	int actual;
	int ret;

	ch341_i2c = devm_kzalloc(&pdev->dev, sizeof(*ch341_i2c), GFP_KERNEL);
	if (ch341_i2c == NULL)
		return -ENOMEM;

	ch341_i2c->i2c_buf = devm_kzalloc(&pdev->dev, SEG_SIZE, GFP_KERNEL);
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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,13,0)
	return devm_i2c_add_adapter(&pdev->dev, &ch341_i2c->adapter);
#else
	ret = i2c_add_adapter(&ch341_i2c->adapter);
	if (ret < 0)
		return ret;

	return devm_add_action_or_reset(&pdev->dev, devm_i2c_del_adapter, &ch341_i2c->adapter);
#endif
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
