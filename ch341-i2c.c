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


static int ch341_i2c_quick_write(struct ch341_device *dev, struct i2c_msg *msg)
{
    int retval, actual = 0;
    uint8_t *ptr = 0;

    mutex_lock(&dev->usb_lock);
    ptr = dev->i2c_outbuf;
    *ptr++ = CH341_CMD_I2C_STREAM;
    *ptr++ = CH341_CMD_I2C_STM_STA;
    *ptr++ = CH341_CMD_I2C_STM_OUT; // wlen=0 is same as wlen=1 but also ask for status from I2C
    *ptr++ = msg->addr << 1;
    *ptr++ = CH341_CMD_I2C_STM_STO;

    retval = usb_bulk_msg(dev->usb_dev, usb_sndbulkpipe(dev->usb_dev, dev->ep_out), dev->i2c_outbuf, ptr - dev->i2c_outbuf, &actual, 2000);
    if (retval < 0)
        goto unlock;
    retval = usb_bulk_msg(dev->usb_dev, usb_rcvbulkpipe(dev->usb_dev, dev->ep_in), dev->i2c_inbuf, 8, &actual, 2000);
    if (retval < 0)
        goto unlock;
    if (dev->i2c_inbuf[0] & 0x80) // check status for NACK
        retval = -ETIMEDOUT;
unlock:
    mutex_unlock(&dev->usb_lock);
    return retval;
}

int ch341_i2c_read(struct ch341_device *dev, struct i2c_msg *msg)
{
    int byteoffset = 0, bytestoread;
    int ret = 0, actual = 0;
    uint8_t *ptr;
    while (msg->len - byteoffset > 0) {
        mutex_lock(&dev->usb_lock);

        bytestoread = msg->len - byteoffset;
        if (bytestoread > 31)
            bytestoread = 31;
        ptr = dev->i2c_outbuf;
        *ptr++ = CH341_CMD_I2C_STREAM;
        *ptr++ = CH341_CMD_I2C_STM_STA;
        *ptr++ = CH341_CMD_I2C_STM_OUT; // wlen=0 is same as wlen=1 but also ask for status from I2C
        *ptr++ = (msg->addr << 1) | 1;
        if (bytestoread > 1) {
            *ptr++ = CH341_CMD_I2C_STM_IN | (bytestoread - 1);
        }
        *ptr++ = CH341_CMD_I2C_STM_IN;
        *ptr++ = CH341_CMD_I2C_STM_STO;

        ret = usb_bulk_msg(dev->usb_dev, usb_sndbulkpipe(dev->usb_dev, dev->ep_out), dev->i2c_outbuf, ptr - dev->i2c_outbuf, &actual, 2000);
        if (ret < 0)
            goto unlock;
        ret = usb_bulk_msg(dev->usb_dev, usb_rcvbulkpipe(dev->usb_dev, dev->ep_in), dev->i2c_inbuf, 32, &actual, 2000);
        if (ret < 0)
            goto unlock;
        if (dev->i2c_inbuf[0] & 0x80) // check status for NACK
            ret = -ETIMEDOUT;
        if (ret > -1) {
            memcpy(&msg->buf[byteoffset], &dev->i2c_inbuf[1], bytestoread);
            byteoffset += bytestoread;
        }
unlock:
        mutex_unlock(&dev->usb_lock);

        if (ret < 0)
            break;
    }
    return ret;
}

static int ch341_usb_transfer(struct ch341_device *ch341_dev, int out_len,int in_len)
{
    int retval;
    int actual;

    //DEV_INFO (CH341_IF_ADDR, "bulk_out %d bytes, bulk_in %d bytes", out_len, (in_len == 0) ? 0 : CH341_USB_MAX_BULK_SIZE);

    retval = usb_bulk_msg(ch341_dev->usb_dev, usb_sndbulkpipe(ch341_dev->usb_dev, ch341_dev->ep_out),
                            ch341_dev->i2c_outbuf, out_len, &actual, 2000);
    if (retval < 0)
        return retval;

    if (in_len == 0)
        return actual;

    memset(ch341_dev->i2c_inbuf, 0, sizeof(ch341_dev->i2c_inbuf));
    retval = usb_bulk_msg(ch341_dev->usb_dev, usb_rcvbulkpipe(ch341_dev->usb_dev, ch341_dev->ep_in),
                            ch341_dev->i2c_inbuf, SEG_SIZE, &actual, 2000);

    if (retval < 0)
        return retval;

    return actual;
}

int ch341_i2c_write(struct ch341_device *dev, struct i2c_msg *msg) {
    unsigned left = msg->len, avail, wlen;
    uint8_t *ptr = msg->buf, *outptr, *lenptr;
    int ret = 0;
    bool first = true;
    do {
        mutex_lock(&dev->usb_lock);

        outptr = dev->i2c_outbuf;
        *outptr++ = CH341_CMD_I2C_STREAM;
        if (first) { // Start packet
            *outptr++ = CH341_CMD_I2C_STM_STA;
        }
        lenptr = outptr++;
        if (first) {
            *outptr++ = msg->addr << 1;
        }
        avail = 32 - (outptr - dev->i2c_outbuf);
        wlen = avail;
        if (left < avail) {
            wlen = left;
        } else if (left == avail) {
            wlen = avail - 1;
        }
        memcpy(outptr, ptr, wlen);
        outptr += wlen;
        ptr += wlen;
        left -= wlen;
        wlen = outptr - lenptr - 1;
        *lenptr = CH341_CMD_I2C_STM_OUT | wlen;
        if (left == 0) {  // Stop packet
            *outptr++ = CH341_CMD_I2C_STM_STO;
        }
        first = false;
        ret = ch341_usb_transfer(dev, outptr - dev->i2c_outbuf, 0);

        mutex_unlock(&dev->usb_lock);

        if (ret < 0)
            break;
    } while (left);

    return ret;
}

#define CHECK_PARAM_RET(cond,err) if (!(cond)) return err;

static int ch341_i2c_xfer(struct i2c_adapter *adpt, struct i2c_msg *msgs, int num)
{
    struct ch341_device *dev;
    int result;
    int i;

    CHECK_PARAM_RET(adpt, EIO);
    CHECK_PARAM_RET(msgs, EIO);
    CHECK_PARAM_RET(num > 0, EIO);

    dev = (struct ch341_device *)adpt->algo_data;

    CHECK_PARAM_RET(dev, EIO);

    for (i = 0; i < num; ++i) {
        if (msgs[i].flags & I2C_M_TEN) {
            dev_err(&dev->iface->dev, "10 bit i2c addresses are not supported");
            result = -EINVAL;
            break;
        }
        if (msgs[i].flags & I2C_M_RD) {
            result = ch341_i2c_read(dev, &msgs[i]); // checks for NACK of msg->addr
            if (result < 0)
                break;
        } else {
            result = ch341_i2c_quick_write(dev, &msgs[i]); // checks for NACK of msg->addr
            if (result < 0)
                break;
            result = ch341_i2c_write(dev, &msgs[i]);
            if (result < 0)
                break;
        }
    }

    if (result < 0)
        return result;

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
    dev->i2c_outbuf[0] = CH341_CMD_I2C_STREAM;
    dev->i2c_outbuf[1] = CH341_CMD_I2C_STM_SET | CH341_I2C_STANDARD_SPEED;
	mutex_lock(&dev->usb_lock);
	retval = usb_bulk_msg(dev->usb_dev,
			      usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
                  dev->i2c_outbuf, 2, &actual, DEFAULT_TIMEOUT);
	mutex_unlock(&dev->usb_lock);
	if (retval < 0) {
		dev_err(&dev->iface->dev, "Cannot set I2C speed\n");
		return -EIO;
	}

	i2c_add_adapter(&dev->adapter);
	dev->i2c_init = true;

	return 0;
}
