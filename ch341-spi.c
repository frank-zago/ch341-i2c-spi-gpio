// SPDX-License-Identifier: GPL-2.0
/*
 * SPI interface for the CH341A and CH341B chips.
 *
 * Copyright 2021, Frank Zago
 * Copyright (c) 2017 Gunar Schorcht (gunar@schorcht.net)
 * Copyright (c) 2016 Tse Lun Bien
 * Copyright (c) 2014 Marco Gittler
 * Copyright (c) 2006-2007 Till Harbaum (Till@Harbaum.org)
 */

#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/bitrev.h>

#include "ch341.h"

/* Only one frequency of about 1.5MHz, which could be the on-board
 * 12MHz divided by 8. Keep a min_freq low to avoid errors from callers
 * asking for a lower frequency.
 */
#define CH341_SPI_MIN_FREQ          400
#define CH341_SPI_MAX_FREQ          1500000

#define CH341_CMD_SPI_STREAM 0xA8

/* Pin configuration. The SPI interfaces may use up to 7 pins
 * allocated to GPIOs.
 */
struct spi_gpio {
	unsigned int hwnum;
	const char *label;
	enum gpiod_flags dflags;
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

struct ch341_spi_priv {
	struct ch341_device *ch341_dev;
};

static size_t cha341_spi_max_tx_size(struct spi_device *spi)
{
	return SEG_SIZE - 1;
}

/* Send a command and get a reply if requested */
static int spi_transfer(struct ch341_device *dev, int len)
{
	int actual;
	int rc;

	mutex_lock(&dev->usb_lock);

	rc = usb_bulk_msg(dev->usb_dev,
			  usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
			  dev->spi_buf, len + 1,
			  &actual, DEFAULT_TIMEOUT);
	if (rc < 0)
		goto done;

	rc = usb_bulk_msg(dev->usb_dev,
			  usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
			  dev->spi_buf, SEG_SIZE, &actual, DEFAULT_TIMEOUT);

	if (rc == 0)
		rc = actual;

done:
	mutex_unlock(&dev->usb_lock);

	return rc;
}

static void ch341_memcpy_bitswap(u8 *dest, const u8 *src, size_t n)
{
	while (n--)
		*dest++ = bitrev8(*src++);
}

/* Send a message */
static int ch341_spi_transfer_one(struct spi_master *master,
				  struct spi_device *spi,
				  struct spi_transfer *xfer)
{
	struct ch341_spi_priv *priv = spi_master_get_devdata(master);
	struct ch341_device *dev = priv->ch341_dev;
	struct gpio_desc *cs;
	bool lsb = spi->mode & SPI_LSB_FIRST;
	int rc;

	mutex_lock(&dev->spi_buf_lock);

	if (spi->mode & SPI_NO_CS) {
		cs = NULL;
	} else {
		cs = dev->spi_gpio_cs_desc[spi->chip_select];

		if (spi->mode & SPI_CS_HIGH)
			gpiod_set_value_cansleep(cs, 1);
		else
			gpiod_set_value_cansleep(cs, 0);
	}

	dev->spi_buf[0] = CH341_CMD_SPI_STREAM;

	/* The CH341 will send LSB first, so bit reversing may need to happen. */
	if (lsb)
		memcpy(&dev->spi_buf[1], xfer->tx_buf, xfer->len);
	else
		ch341_memcpy_bitswap(&dev->spi_buf[1], xfer->tx_buf, xfer->len);

	rc = spi_transfer(dev, xfer->len);

	if (cs && (xfer->cs_change || spi_transfer_is_last(master, xfer))) {
		if (spi->mode & SPI_CS_HIGH)
			gpiod_set_value_cansleep(cs, 0);
		else
			gpiod_set_value_cansleep(cs, 1);
	}

	if (rc >= 0 && xfer->rx_buf) {
		if (lsb)
			memcpy(xfer->rx_buf, dev->spi_buf, xfer->len);
		else
			ch341_memcpy_bitswap(xfer->rx_buf, dev->spi_buf, xfer->len);
	}

	spi_finalize_current_transfer(master);

	mutex_unlock(&dev->spi_buf_lock);

	return rc;
}

static void release_core_gpios(struct ch341_device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dev->spi_gpio_core_desc); i++) {
		gpiochip_free_own_desc(dev->spi_gpio_core_desc[i]);
		dev->spi_gpio_core_desc[i] = NULL;
	}
}

static int request_core_gpios(struct ch341_device *dev)
{
	int rc;
	int i;

	for (i = 0; i < ARRAY_SIZE(spi_gpio_core); i++) {
		struct gpio_desc *desc;
		const struct spi_gpio *gpio = &spi_gpio_core[i];

		desc = gpiochip_request_own_desc(&dev->gpio,
						 gpio->hwnum,
						 gpio->label,
						 GPIO_LOOKUP_FLAGS_DEFAULT,
						 gpio->dflags);
		if (IS_ERR(desc)) {
			dev_warn(&dev->master->dev,
				 "Unable to reserve GPIO %s for SPI\n",
				 gpio->label);
			rc = PTR_ERR(desc);
			goto unreg_gpios;
		}

		dev->spi_gpio_core_desc[i] = desc;
	}

	return 0;

unreg_gpios:
	release_core_gpios(dev);

	return rc;
}

/* Add a new device, defined by the board_info. */
static int add_slave(struct ch341_device *dev, struct spi_board_info *board_info)
{
	struct gpio_desc *desc;
	unsigned int cs = board_info->chip_select;
	int rc;

	/* Sanity check */
	if (cs >= dev->master->num_chipselect)
		return -EINVAL;

	board_info->bus_num = dev->master->bus_num;

	mutex_lock(&dev->spi_lock);

	if (dev->cs_allocated & BIT(cs)) {
		rc = -EADDRINUSE;
		goto unlock;
	}

	if (dev->cs_allocated == 0) {
		/* No CS allocated yet. Grab the core gpios too */
		rc = request_core_gpios(dev);
		if (rc)
			goto unlock;

		/* Set clock to low */
		gpiod_set_value_cansleep(dev->spi_gpio_core_desc[0], 0);
	}

	/* Allocate the CS GPIO */
	desc = gpiochip_request_own_desc(&dev->gpio,
					 spi_gpio_cs[cs].hwnum,
					 spi_gpio_cs[cs].label,
					 GPIO_LOOKUP_FLAGS_DEFAULT,
					 spi_gpio_cs[cs].dflags);
	if (IS_ERR(desc)) {
		dev_warn(&dev->master->dev,
			 "Unable to reserve GPIO %s for SPI\n",
			 spi_gpio_cs[cs].label);
		rc = PTR_ERR(desc);
		goto unreg_core_gpios;
	}

	dev->spi_gpio_cs_desc[cs] = desc;
	dev->cs_allocated |= BIT(cs);

	dev->slaves[cs] = spi_new_device(dev->master, board_info);
	if (!dev->slaves[cs]) {
		rc = -ENOMEM;
		goto release_cs;
	}

	mutex_unlock(&dev->spi_lock);

	return 0;

release_cs:
	gpiochip_free_own_desc(dev->spi_gpio_cs_desc[cs]);
	dev->spi_gpio_cs_desc[cs] = NULL;
	dev->cs_allocated &= ~BIT(cs);

unreg_core_gpios:
	if (dev->cs_allocated == 0)
		release_core_gpios(dev);

unlock:
	mutex_unlock(&dev->spi_lock);

	return rc;
}

static int remove_slave(struct ch341_device *dev, unsigned int cs)
{
	int rc;

	if (cs >= ARRAY_SIZE(spi_gpio_cs))
		return -EINVAL;

	mutex_lock(&dev->spi_lock);

	if (dev->cs_allocated & BIT(cs)) {
		dev->cs_allocated &= ~BIT(cs);

		spi_unregister_device(dev->slaves[cs]);
		dev->slaves[cs] = NULL;

		gpiochip_free_own_desc(dev->spi_gpio_cs_desc[cs]);
		dev->spi_gpio_cs_desc[cs] = NULL;

		if (dev->cs_allocated == 0) {
			/* Last slave. Release the core GPIOs */
			release_core_gpios(dev);
		}

		rc = 0;
	} else {
		rc = -ENODEV;
	}

	mutex_unlock(&dev->spi_lock);

	return rc;
}

/* sysfs entry to add a new device. It takes a string with 2
 * parameters: the modalias and the chip select number. For instance
 * "spi-nor 0" or "spidev 1".
 */
static ssize_t new_device_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct spi_master *master = container_of(dev, struct spi_master, dev);
	struct ch341_spi_priv *priv = spi_master_get_devdata(master);
	struct spi_board_info board_info = {
		.mode = SPI_MODE_0,
		.max_speed_hz = CH341_SPI_MAX_FREQ,
	};
	char *req_org;
	char *req;
	char *str;
	int rc;

	req_org = kstrdup(buf, GFP_KERNEL);
	if (!req_org)
		return -ENOMEM;

	req = req_org;

	str = strsep(&req, " ");
	if (str == NULL) {
		rc = -EINVAL;
		goto free_req;
	}

	rc = strscpy(board_info.modalias, str, sizeof(board_info.modalias));
	if (rc < 0)
		goto free_req;

	str = strsep(&req, " ");
	rc = kstrtou16(str, 0, &board_info.chip_select);
	if (rc)
		goto free_req;

	rc = add_slave(priv->ch341_dev, &board_info);
	if (rc)
		goto free_req;

	kfree(req_org);

	return count;

free_req:
	kfree(req_org);

	return rc;
}

/* sysfs entry to remove an existing device at a given chip select. It
 * takes a string with a single number. For instance "2".
 */
static ssize_t delete_device_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct spi_master *master = container_of(dev, struct spi_master, dev);
	struct ch341_spi_priv *priv = spi_master_get_devdata(master);
	int cs;
	int rc;

	rc = kstrtouint(buf, 0, &cs);
	if (rc)
		return rc;

	rc = remove_slave(priv->ch341_dev, cs);
	if (rc)
		return rc;

	return count;
}

static DEVICE_ATTR_WO(new_device);
static DEVICE_ATTR_WO(delete_device);

void ch341_spi_remove(struct ch341_device *dev)
{
	int cs;

	if (dev->master == NULL)
		return;

	device_remove_file(&dev->master->dev, &dev_attr_new_device);
	device_remove_file(&dev->master->dev, &dev_attr_delete_device);

	for (cs = 0; cs < ARRAY_SIZE(dev->slaves); cs++)
		remove_slave(dev, cs);

	spi_unregister_master(dev->master);
}

int ch341_spi_init(struct ch341_device *dev)
{
	struct spi_master *master;
	struct ch341_spi_priv *priv;
	int rc;

	dev->master = spi_alloc_master(&dev->iface->dev,
				       sizeof(struct ch341_device *));
	if (!dev->master)
		return -ENOMEM;

	master = dev->master;
	mutex_init(&dev->spi_lock);
	mutex_init(&dev->spi_buf_lock);

	priv = spi_master_get_devdata(master);
	priv->ch341_dev = dev;

	master->bus_num = -1;
	master->num_chipselect = CH341_SPI_MAX_NUM_DEVICES;
	master->mode_bits = SPI_MODE_0 | SPI_LSB_FIRST;
	master->flags = SPI_MASTER_MUST_RX | SPI_MASTER_MUST_TX;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->transfer_one = ch341_spi_transfer_one;
	master->max_speed_hz = CH341_SPI_MAX_FREQ;
	master->min_speed_hz = CH341_SPI_MIN_FREQ;
	master->max_transfer_size = cha341_spi_max_tx_size;
	master->max_message_size = cha341_spi_max_tx_size;

	rc = spi_register_master(master);
	if (rc)
		goto unreg_master;

	rc = device_create_file(&master->dev, &dev_attr_new_device);
	if (rc) {
		dev_err(&master->dev, "Cannot create new_device file\n");
		goto unreg_master;
	}

	rc = device_create_file(&master->dev, &dev_attr_delete_device);
	if (rc) {
		dev_err(&master->dev, "Cannot create delete_device file\n");
		goto del_new_device;
	}

	return 0;

del_new_device:
	device_remove_file(&dev->master->dev, &dev_attr_new_device);

unreg_master:
	spi_master_put(master);
	dev->master = NULL;

	return rc;
}
