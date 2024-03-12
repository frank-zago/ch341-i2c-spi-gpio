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
	struct spi_controller *master;
	struct mutex spi_lock;
	u8 cs_allocated;    /* bitmask of allocated CS for SPI */
	struct gpio_desc *sck, *mosi, *miso;
	struct spi_client {
		struct spi_device *slave;
		struct gpio_desc *gpio;
		u8 buf[CH341_SPI_NSEGS * SEG_SIZE];
	} spi_clients[CH341_SPI_MAX_NUM_DEVICES];

	struct ch341_ddata *ch341;
};

static struct gpiod_lookup_table gpios_table = {
       .dev_id = NULL,
       .table = {
               GPIO_LOOKUP("ch341",  3,  "sck", GPIO_LOOKUP_FLAGS_DEFAULT),
               GPIO_LOOKUP("ch341",  5,  "mosi", GPIO_LOOKUP_FLAGS_DEFAULT),
               GPIO_LOOKUP("ch341",  7,  "miso", GPIO_LOOKUP_FLAGS_DEFAULT),

               GPIO_LOOKUP_IDX("ch341",  0,  "cs", 0, GPIOD_OUT_HIGH),
               GPIO_LOOKUP_IDX("ch341",  1,  "cs", 1, GPIOD_OUT_HIGH),
               GPIO_LOOKUP_IDX("ch341",  2,  "cs", 2, GPIOD_OUT_HIGH),
               GPIO_LOOKUP_IDX("ch341",  4,  "cs", 3, GPIOD_OUT_HIGH), /* Only on CH341A/B, not H */
               { }
       }
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
static int ch341_spi_transfer_one_message(struct spi_controller *master,
					  struct spi_message *m)
{
	struct ch341_spi *dev = spi_controller_get_devdata(master);
	struct spi_device *spi = m->spi;
	struct spi_client *client = &dev->spi_clients[spi_get_chipselect(spi, 0)];
	bool lsb = spi->mode & SPI_LSB_FIRST;
	struct spi_transfer *xfer;
	unsigned int buf_idx = 0;
	unsigned int tx_len = 0;
	struct gpio_desc *cs;
	int status;

	// sure would be cool if we could just use spi_set_cs & all the gpio magic
	// that comes with it
	if (spi->mode & SPI_NO_CS) {
		cs = NULL;
	} else {
		cs = client->gpio;

		if (spi->mode & SPI_CS_HIGH)
			gpiod_set_value_cansleep(cs, 1);
		else
			gpiod_set_value_cansleep(cs, 0);
	}

	list_for_each_entry(xfer, &m->transfers, transfer_list) {
		buf_idx = copy_to_device(client->buf, buf_idx, xfer->tx_buf, xfer->len, lsb);

		tx_len += xfer->len;
	}

	status = spi_transfer(dev, client->buf, buf_idx, buf_idx - tx_len);
	if (cs) {
		if (spi->mode & SPI_CS_HIGH)
			gpiod_set_value_cansleep(cs, 0);
		else
			gpiod_set_value_cansleep(cs, 1);
	}

	if (status >= 0) {
		buf_idx = 0;
		list_for_each_entry(xfer, &m->transfers, transfer_list)
			buf_idx = copy_from_device(xfer->rx_buf, client->buf,
						   buf_idx, xfer->len, lsb);

		m->actual_length = tx_len;
		status = 0;
	}

	m->status = status;
	spi_finalize_current_message(master);

	return 0;
}

/* Add a new device, defined by the board_info. */
static int ch341_setup(struct spi_device *spi)
{
	struct spi_controller *ctrl = spi->controller;
	struct ch341_spi *dev = spi_controller_get_devdata(ctrl);
	unsigned int cs = spi_get_chipselect(spi, 0);
	struct spi_client *client;
	struct gpio_desc *desc;
	int ret;

	// /* Sanity check */
	// if (cs >= dev->master->num_chipselect)
	// 	return -EINVAL;

	client = &dev->spi_clients[cs];

	mutex_lock(&dev->spi_lock);

	if (dev->cs_allocated & BIT(cs)) {
		ret = -EADDRINUSE;
		goto unlock;
	}

	if (dev->cs_allocated == 0) {
		/* Set clock to low */
		gpiod_set_value_cansleep(dev->sck, 0);
	}

	client->gpio = spi_get_csgpiod(spi, 0);
	dev->cs_allocated |= BIT(cs);

	client->slave = spi;

	mutex_unlock(&dev->spi_lock);

	return 0;

release_cs:
	client->gpio = NULL;
	dev->cs_allocated &= ~BIT(cs);

unlock:
	mutex_unlock(&dev->spi_lock);

	return ret;
}

static void ch341_cleanup(struct spi_device *spi)
{
	struct spi_controller *ctrl = spi->controller;
	struct ch341_spi *dev = spi_controller_get_devdata(ctrl);
	unsigned int cs = spi_get_chipselect(spi, 0);

	if (cs >= dev->master->num_chipselect)
		return;

	mutex_lock(&dev->spi_lock);

	if (dev->cs_allocated & BIT(cs)) {
		dev->cs_allocated &= ~BIT(cs);
	}

	mutex_unlock(&dev->spi_lock);
}

static int ch341_spi_probe(struct platform_device *pdev)
{
	struct ch341_ddata *ch341 = dev_get_drvdata(pdev->dev.parent);
	struct spi_controller *master;
	struct ch341_spi *dev;
	int ret;

	master = devm_spi_alloc_master(&pdev->dev, sizeof(*dev));
	if (!master)
		return -ENOMEM;

	dev = spi_controller_get_devdata(master);

	dev->master = master;
	dev->ch341 = ch341;
	platform_set_drvdata(pdev, dev);

	mutex_init(&dev->spi_lock);

	master->setup = ch341_setup;
	master->cleanup = ch341_cleanup;
	master->bus_num = -1;
	master->num_chipselect = CH341_SPI_MAX_NUM_DEVICES;
	master->mode_bits = SPI_MODE_0 | SPI_LSB_FIRST;
	master->flags = SPI_CONTROLLER_MUST_RX | SPI_CONTROLLER_MUST_TX;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->transfer_one_message = ch341_spi_transfer_one_message;
	master->max_speed_hz = CH341_SPI_MAX_FREQ;
	master->min_speed_hz = CH341_SPI_MIN_FREQ;
	master->max_transfer_size = cha341_spi_max_tx_size;
	master->max_message_size = cha341_spi_max_tx_size;
	master->use_gpio_descriptors = true;

	gpios_table.dev_id = NULL;
    gpiod_add_lookup_table(&gpios_table);

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret)
		goto unregister_table;

	gpios_table.dev_id = dev_name(&master->dev);

	struct gpio_desc *sck, *mosi, *miso;
	dev->sck = sck = devm_gpiod_get(&master->dev, "sck", GPIOD_OUT_HIGH);
	if (IS_ERR(sck)) {
		dev_warn(&dev->master->dev, "Unable to reserve GPIO 'sck' for SPI\n");
		ret = PTR_ERR(sck);
		goto unregister_table;
	}
	gpiod_set_consumer_name(sck, "spi SCK");

	dev->mosi = mosi = devm_gpiod_get(&master->dev, "mosi", GPIOD_OUT_HIGH);
	if (IS_ERR(mosi)) {
		dev_warn(&dev->master->dev, "Unable to reserve GPIO 'mosi' for SPI\n");
		ret = PTR_ERR(mosi);
		goto unregister_table;
	}
	gpiod_set_consumer_name(mosi, "spi MOSI");

	dev->miso = miso = devm_gpiod_get(&master->dev, "miso", GPIOD_IN);
	if (IS_ERR(miso)) {
		dev_warn(&dev->master->dev, "Unable to reserve GPIO 'miso' for SPI\n");
		ret = PTR_ERR(miso);
		goto unregister_table;
	}
	gpiod_set_consumer_name(miso, "spi MISO");

	for (int i=0; i < master->num_chipselect; i++) {
		struct spi_board_info board_info = {
			.mode = SPI_MODE_0,
			.max_speed_hz = CH341_SPI_MAX_FREQ,
		};
		board_info.chip_select = i;
		spi_new_device(master, &board_info);
	}

unregister_table:
	gpiod_remove_lookup_table(&gpios_table);

	return ret;
}

static struct platform_driver ch341_spi_driver = {
	.driver.name = "ch341-spi",
	.probe	     = ch341_spi_probe,
};
module_platform_driver(ch341_spi_driver);

MODULE_AUTHOR("Frank Zago <frank@zago.net>");
MODULE_DESCRIPTION("CH341 USB to SPI");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ch341-spi");
