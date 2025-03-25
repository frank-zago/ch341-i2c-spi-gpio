// SPDX-License-Identifier: GPL-2.0
/*
 * SPI interface for the CH341A and CH341B chips.
 * Devices are attached as children of the CH341 GPIO driver devices.
 *
 * Copyright 2022, Frank Zago
 * Copyright (c) 2017 Gunar Schorcht (gunar@schorcht.net)
 * Copyright (c) 2016 Tse Lun Bien
 * Copyright (c) 2014 Marco Gittler
 * Copyright (c) 2006-2007 Till Harbaum (Till@Harbaum.org)
 */

#include <linux/bitrev.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/version.h>

#include "ch341.h"

/* Compatibility for older kernels. */
#ifndef SPI_CONTROLLER_MUST_RX
#define SPI_CONTROLLER_MUST_RX SPI_MASTER_MUST_RX
#define SPI_CONTROLLER_MUST_TX SPI_MASTER_MUST_TX
#endif

/*
 * Only one frequency of about 1.5MHz, which could be the on-board
 * 12MHz divided by 8. Keep a min_freq low to avoid errors from callers
 * asking for a lower frequency.
 */
#define CH341_SPI_MIN_FREQ          400
#define CH341_SPI_MAX_FREQ          1500000

#define CH341_CMD_SPI_STREAM 0xA8

/* Number of SPI devices the device can control */
#define CH341_SPI_MAX_NUM_DEVICES 4

/*
 * Number of segments in each SPI transfer buffer. The device will
 * crash if more than 4.
 */
#define CH341_SPI_NSEGS 4

/*
 * Pin configuration. The SPI interfaces may use up to 7 pins
 * allocated to GPIOs.
 */
struct spi_gpio {
	unsigned int hwnum;
	const char *label;
	enum gpiod_flags dflags;
};

struct ch341_spi {
	struct spi_controller *controller;
	struct mutex spi_lock;
	u8 cs_allocated;    /* bitmask of allocated CS for SPI */
	struct gpio_desc *spi_gpio_core_desc[3];
	struct spi_client {
		struct spi_device *slave;
		struct gpio_desc *gpio;
		u8 buf[CH341_SPI_NSEGS * SEG_SIZE];
	} spi_clients[CH341_SPI_MAX_NUM_DEVICES];

	struct ch341_ddata *ch341;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,7,0)
	struct gpio_device *gdev;
#endif
	struct gpio_chip *gpiochip;
};

static const struct spi_gpio spi_gpio_core[] = {
	{ 3,  "SCK", GPIOD_OUT_HIGH },
	{ 5,  "MOSI", GPIOD_OUT_HIGH },
	{ 7,  "MISO", GPIOD_IN }
};

static const struct spi_gpio spi_gpio_cs[CH341_SPI_MAX_NUM_DEVICES] = {
	{ 0,  "CS0", GPIOD_OUT_HIGH },
	{ 1,  "CS1", GPIOD_OUT_HIGH },
	{ 2,  "CS2", GPIOD_OUT_HIGH },
	{ 4,  "CS3", GPIOD_OUT_HIGH } /* Only on CH341A/B, not H */
};

static size_t cha341_spi_max_tx_size(struct spi_device *spi)
{
	return CH341_SPI_NSEGS * (SEG_SIZE - 1);
}

/* Send a command and get a reply if requested */
static int spi_transfer(struct ch341_spi *dev, u8 *buf, int len, int nsegs)
{
	struct ch341_ddata *ch341 = dev->ch341;
	int sz_read = 0;
	int actual;
	int ret;

	mutex_lock(&ch341->usb_lock);

	ret = usb_bulk_msg(ch341->usb_dev,
			   usb_sndbulkpipe(ch341->usb_dev, ch341->ep_out),
			   buf, len, &actual, DEFAULT_TIMEOUT_MS);
	if (ret < 0)
		goto out_unlock;

	do {
		ret = usb_bulk_msg(ch341->usb_dev,
				   usb_rcvbulkpipe(ch341->usb_dev, ch341->ep_in),
				   &buf[sz_read], SEG_SIZE, &actual,
				   DEFAULT_TIMEOUT_MS);

		if (ret == 0)
			sz_read += actual;

		nsegs--;

	} while (ret == 0 && nsegs);

	if (ret == 0)
		ret = sz_read;

out_unlock:
	mutex_unlock(&ch341->usb_lock);

	return ret;
}

static void ch341_memcpy_bitswap(u8 *dest, const u8 *src, size_t n)
{
	while (n--)
		*dest++ = bitrev8(*src++);
}

/*
 * Copy a TX buffer to the device buffer, adding the STREAM command as
 * needed.
 */
static unsigned int copy_to_device(u8 *buf, unsigned int buf_idx,
				   const u8 *tx_buf, unsigned int len, bool lsb)
{
	int to_copy;

	while (len) {
		if (buf_idx % SEG_SIZE == 0) {
			buf[buf_idx] = CH341_CMD_SPI_STREAM;
			buf_idx++;
		}

		to_copy = min(len, SEG_SIZE - (buf_idx % SEG_SIZE));

		if (tx_buf) {
			if (lsb)
				memcpy(&buf[buf_idx], tx_buf, to_copy);
			else
				ch341_memcpy_bitswap(&buf[buf_idx], tx_buf,
						     to_copy);
		} else {
			memset(&buf[buf_idx], 0, to_copy);
		}

		len -= to_copy;
		buf_idx += to_copy;
		tx_buf += to_copy;
	}

	return buf_idx;
}

/* Copy from the device buffer to a receive buffer */
static unsigned int copy_from_device(u8 *rx_buf, const u8 *buf,
				     unsigned int buf_idx, unsigned int len,
				     bool lsb)
{
	if (rx_buf) {
		if (lsb)
			memcpy(rx_buf, &buf[buf_idx], len);
		else
			ch341_memcpy_bitswap(rx_buf, &buf[buf_idx], len);
	}

	buf_idx += len;

	return buf_idx;
}

/* Send a message */
static int ch341_spi_transfer_one_message(struct spi_controller *controller,
					  struct spi_message *m)
{
	struct ch341_spi *dev = spi_controller_get_devdata(controller);
	struct spi_device *spi = m->spi;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,3,0)
	struct spi_client *client = &dev->spi_clients[spi_get_chipselect(spi, 0)];
#else
	struct spi_client *client = &dev->spi_clients[spi->chip_select];
#endif
	bool lsb = spi->mode & SPI_LSB_FIRST;
	struct spi_transfer *xfer;
	unsigned int buf_idx = 0;
	unsigned int tx_len = 0;
	struct gpio_desc *cs;
	bool skip = false;
	int status;

	m->actual_length = 0;
	mutex_lock(&dev->spi_lock);
	list_for_each_entry(xfer, &m->transfers, transfer_list) {
		if (!xfer->len) {
			status = 0;
			skip = true;
			goto complete_xfer;
		}

		if (spi->mode & SPI_NO_CS) {
			cs = NULL;
		} else {
			cs = client->gpio;

			if (spi->mode & SPI_CS_HIGH)
				gpiod_set_value_cansleep(cs, 1);
			else
				gpiod_set_value_cansleep(cs, 0);
		}

		buf_idx = copy_to_device(client->buf, buf_idx, xfer->tx_buf, xfer->len, lsb);
		tx_len += xfer->len;

		status = spi_transfer(dev, client->buf, buf_idx, buf_idx - tx_len);
		if (cs) {
			if (spi->mode & SPI_CS_HIGH)
				gpiod_set_value_cansleep(cs, 0);
			else
				gpiod_set_value_cansleep(cs, 1);
		}


		if (status >= 0) {
			buf_idx = 0;
			buf_idx = copy_from_device(xfer->rx_buf, client->buf,
						   buf_idx, xfer->len, lsb);
		} else {
			goto complete_xfer;
		}

		m->actual_length += tx_len;

complete_xfer:
		if (status < 0 ) {
			spi_transfer_delay_exec(xfer);
			goto exit;
		} else if (xfer->cs_change || skip) {
			spi_transfer_delay_exec(xfer);
		}
	}

exit:
	mutex_unlock(&dev->spi_lock);
	m->status = status;
	spi_finalize_current_message(controller);

	return 0;
}

static void release_core_gpios(struct ch341_spi *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dev->spi_gpio_core_desc); i++) {
		gpiochip_free_own_desc(dev->spi_gpio_core_desc[i]);
		dev->spi_gpio_core_desc[i] = NULL;
	}
}

static int request_core_gpios(struct ch341_spi *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(spi_gpio_core); i++) {
		struct gpio_desc *desc;
		const struct spi_gpio *gpio = &spi_gpio_core[i];

		desc = gpiochip_request_own_desc(dev->gpiochip,
						 gpio->hwnum,
						 gpio->label,
						 GPIO_LOOKUP_FLAGS_DEFAULT,
						 gpio->dflags);
		if (IS_ERR(desc)) {
			dev_warn(&dev->controller->dev,
				 "Unable to reserve GPIO %s for SPI\n",
				 gpio->label);
			release_core_gpios(dev);

			return PTR_ERR(desc);
		}

		dev->spi_gpio_core_desc[i] = desc;
	}

	return 0;
}

/* Add a new device, defined by the board_info. */
static int add_slave(struct ch341_spi *dev, struct spi_board_info *board_info)
{
	unsigned int cs = board_info->chip_select;
	struct spi_client *client;
	struct gpio_desc *desc;
	int ret;

	/* Sanity check */
	if (cs >= dev->controller->num_chipselect)
		return -EINVAL;

	client = &dev->spi_clients[cs];
	board_info->bus_num = dev->controller->bus_num;

	mutex_lock(&dev->spi_lock);

	if (dev->cs_allocated & BIT(cs)) {
		ret = -EADDRINUSE;
		goto unlock;
	}

	if (dev->cs_allocated == 0) {
		/* No CS allocated yet. Grab the core gpios too */
		ret = request_core_gpios(dev);
		if (ret)
			goto unlock;

		/* Set clock to low */
		gpiod_set_value_cansleep(dev->spi_gpio_core_desc[0], 0);
	}

	/* Allocate the CS GPIO */
	desc = gpiochip_request_own_desc(dev->gpiochip,
					 spi_gpio_cs[cs].hwnum,
					 spi_gpio_cs[cs].label,
					 GPIO_LOOKUP_FLAGS_DEFAULT,
					 spi_gpio_cs[cs].dflags);
	if (IS_ERR(desc)) {
		dev_warn(&dev->controller->dev,
			 "Unable to reserve GPIO %s for SPI\n",
			 spi_gpio_cs[cs].label);
		ret = PTR_ERR(desc);
		goto unreg_core_gpios;
	}

	client->gpio = desc;
	dev->cs_allocated |= BIT(cs);

	client->slave = spi_new_device(dev->controller, board_info);
	if (!client->slave) {
		ret = -ENOMEM;
		goto release_cs;
	}

	mutex_unlock(&dev->spi_lock);

	return 0;

release_cs:
	gpiochip_free_own_desc(client->gpio);
	client->gpio = NULL;
	dev->cs_allocated &= ~BIT(cs);

unreg_core_gpios:
	if (dev->cs_allocated == 0)
		release_core_gpios(dev);

unlock:
	mutex_unlock(&dev->spi_lock);

	return ret;
}

static int remove_slave(struct ch341_spi *dev, unsigned int cs)
{
	int ret;

	if (cs >= ARRAY_SIZE(spi_gpio_cs))
		return -EINVAL;

	mutex_lock(&dev->spi_lock);

	if (dev->cs_allocated & BIT(cs)) {
		dev->cs_allocated &= ~BIT(cs);

		spi_unregister_device(dev->spi_clients[cs].slave);
		dev->spi_clients[cs].slave = NULL;

		gpiochip_free_own_desc(dev->spi_clients[cs].gpio);
		dev->spi_clients[cs].gpio = NULL;

		if (dev->cs_allocated == 0) {
			/* Last slave. Release the core GPIOs */
			release_core_gpios(dev);
		}

		ret = 0;
	} else {
		ret = -ENODEV;
	}

	mutex_unlock(&dev->spi_lock);

	return ret;
}

/*
 * sysfs entry to add a new device. It takes a string with 2
 * parameters: the modalias and the chip select number. For instance
 * "spi-nor 0" or "spidev 1".
 */
static ssize_t new_device_store(struct device *mdev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct spi_controller *controller = container_of(mdev, struct spi_controller, dev);
	struct ch341_spi *dev = spi_controller_get_devdata(controller);
	struct spi_board_info board_info = {
		.mode = SPI_MODE_0,
		.max_speed_hz = CH341_SPI_MAX_FREQ,
	};
	char *req_org;
	char *req;
	char *str;
	int ret;

	req_org = kstrdup(buf, GFP_KERNEL);
	if (!req_org)
		return -ENOMEM;

	req = req_org;

	str = strsep(&req, " ");
	if (str == NULL) {
		ret = -EINVAL;
		goto free_req;
	}

	ret = strscpy(board_info.modalias, str, sizeof(board_info.modalias));
	if (ret < 0)
		goto free_req;

	str = strsep(&req, " ");
	if (str == NULL) {
		ret = -EINVAL;
		goto free_req;
	}
	ret = kstrtou16(str, 0, &board_info.chip_select);
	if (ret)
		goto free_req;

	ret = add_slave(dev, &board_info);
	if (ret)
		goto free_req;

	kfree(req_org);

	return count;

free_req:
	kfree(req_org);

	return ret;
}

/*
 * sysfs entry to remove an existing device at a given chip select. It
 * takes a string with a single number. For instance "2".
 */
static ssize_t delete_device_store(struct device *mdev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct spi_controller *controller = container_of(mdev, struct spi_controller, dev);
	struct ch341_spi *dev = spi_controller_get_devdata(controller);
	int cs;
	int ret;

	ret = kstrtouint(buf, 0, &cs);
	if (ret)
		return ret;

	ret = remove_slave(dev, cs);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR_WO(new_device);
static DEVICE_ATTR_WO(delete_device);

static
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,11,0)
int
#else
void
#endif
ch341_spi_remove(struct platform_device *pdev)
{
	struct ch341_spi *dev = platform_get_drvdata(pdev);
	int cs;

	device_remove_file(&dev->controller->dev, &dev_attr_new_device);
	device_remove_file(&dev->controller->dev, &dev_attr_delete_device);

	for (cs = 0; cs < ARRAY_SIZE(dev->spi_clients); cs++)
		remove_slave(dev, cs);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,7,0)
	gpio_device_put(dev->gdev);
#endif

	spi_unregister_controller(dev->controller);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,11,0)
	return 0;
#endif
}

static int match_gpiochip_parent(struct gpio_chip *gc,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,9,0)
				 const
#endif
				 void *data)
{
	return gc->parent == data;
}

static int ch341_spi_probe(struct platform_device *pdev)
{
	struct ch341_ddata *ch341 = dev_get_drvdata(pdev->dev.parent->parent);
	struct spi_controller *controller;
	struct ch341_spi *dev;
	int ret;

	controller = spi_alloc_master(&pdev->dev, sizeof(*dev));
	if (!controller)
		return -ENOMEM;

	dev = spi_controller_get_devdata(controller);

	dev->controller = controller;
	dev->ch341 = ch341;
	platform_set_drvdata(pdev, dev);

	/* Find the parent's gpiochip */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,7,0)
	dev->gpiochip = gpiochip_find(pdev->dev.parent, match_gpiochip_parent);
#else
	dev->gdev = gpio_device_find(pdev->dev.parent, match_gpiochip_parent);
	if (!dev->gdev) {
		ret = -ENODEV;
		goto unreg_controller;
	}

	dev->gpiochip = gpio_device_get_chip(dev->gdev);
#endif
	if (!dev->gpiochip) {
		dev_err(&controller->dev, "Parent GPIO chip not found!\n");
		ret = -ENODEV;
		goto put_gpio;
	}

	mutex_init(&dev->spi_lock);

	controller->bus_num = -1;
	controller->num_chipselect = CH341_SPI_MAX_NUM_DEVICES;
	controller->mode_bits = SPI_MODE_0 | SPI_LSB_FIRST;
	controller->flags = SPI_CONTROLLER_MUST_RX | SPI_CONTROLLER_MUST_TX;
	controller->bits_per_word_mask = SPI_BPW_MASK(8);
	controller->transfer_one_message = ch341_spi_transfer_one_message;
	controller->max_speed_hz = CH341_SPI_MAX_FREQ;
	controller->min_speed_hz = CH341_SPI_MIN_FREQ;
	controller->max_transfer_size = cha341_spi_max_tx_size;
	controller->max_message_size = cha341_spi_max_tx_size;

	ret = spi_register_controller(controller);
	if (ret)
		goto put_gpio;

	ret = device_create_file(&controller->dev, &dev_attr_new_device);
	if (ret) {
		dev_err(&controller->dev, "Cannot create new_device file\n");
		goto put_gpio;
	}

	ret = device_create_file(&controller->dev, &dev_attr_delete_device);
	if (ret) {
		dev_err(&controller->dev, "Cannot create delete_device file\n");
		goto del_new_device;
	}

	return 0;

del_new_device:
	device_remove_file(&dev->controller->dev, &dev_attr_new_device);

put_gpio:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,7,0)
	gpio_device_put(dev->gdev);

unreg_controller:
#endif
	spi_controller_put(controller);

	return ret;
}

static struct platform_driver ch341_spi_driver = {
	.driver.name = "ch341-spi",
	.probe	     = ch341_spi_probe,
	.remove	     = ch341_spi_remove,
};
module_platform_driver(ch341_spi_driver);

MODULE_AUTHOR("Frank Zago <frank@zago.net>");
MODULE_DESCRIPTION("CH341 USB to SPI");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ch341-spi");
