// SPDX-License-Identifier: GPL-2.0
/*
 * GPIO interface for the CH341A and CH341B chips.
 *
 * Copyright 2021, Frank Zago
 * Copyright (c) 2017 Gunar Schorcht (gunar@schorcht.net)
 * Copyright (c) 2016 Tse Lun Bien
 * Copyright (c) 2014 Marco Gittler
 * Copyright (c) 2006-2007 Till Harbaum (Till@Harbaum.org)
 */

/* Notes.
 *
 * For the CH341, 0=IN, 1=OUT, but for the GPIO subsystem, 1=IN and
 * 0=OUT. Some translation happens in a couple places.
 */

#include "ch341.h"

#define CH341_GPIO_NUM_PINS         16    /* Number of GPIO pins */

#define CH341_PARA_CMD_STS          0xA0  /* Get pins status */
#define CH341_CMD_UIO_STREAM        0xAB  /* UIO stream command */

#define CH341_CMD_UIO_STM_IN        0x00  /* UIO interface IN command (D0~D7) */
#define CH341_CMD_UIO_STM_OUT       0x80  /* UIO interface OUT command (D0~D5) */
#define CH341_CMD_UIO_STM_DIR       0x40  /* UIO interface DIR command (D0~D5) */
#define CH341_CMD_UIO_STM_END       0x20  /* UIO interface END command */

/* Masks to describe the 8 GPIOs (pins 15 to 22, a.k.a. D0 to D7.)
 * D0 to D5 can read/write, but pins D6 and D7 can only read.
 */
static const u16 pin_can_output = 0b00111111;

static void ch341_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct ch341_device *dev = gpiochip_get_data(chip);

	seq_printf(s, "pin config  : %02x  (0=IN, 1=OUT)\n", dev->gpio_dir);
	seq_printf(s, "last read   : %02x\n", dev->gpio_last_read);
	seq_printf(s, "last written: %02x\n", dev->gpio_last_written);
}

/* Send a command and get a reply if requested */
static int gpio_transfer(struct ch341_device *dev, int out_len, int in_len)
{
	int actual;
	int rc;

	mutex_lock(&dev->usb_lock);

	rc = usb_bulk_msg(dev->usb_dev,
			  usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
			  dev->gpio_buf, out_len,
			  &actual, DEFAULT_TIMEOUT);
	if (rc < 0)
		goto done;

	if (in_len == 0) {
		rc = actual;
		goto done;
	}

	rc = usb_bulk_msg(dev->usb_dev,
			  usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
			  dev->gpio_buf, SEG_SIZE, &actual, DEFAULT_TIMEOUT);

	if (rc == 0)
		rc = actual;

done:
	mutex_unlock(&dev->usb_lock);

	return rc;
}

/* Read the GPIO line status. */
static int read_inputs(struct ch341_device *dev)
{
	int result;

	mutex_lock(&dev->gpio_lock);

	dev->gpio_buf[0] = CH341_PARA_CMD_STS;

	result = gpio_transfer(dev, 1, 1);

	/* The status command returns 6 bytes of data. Byte 0 has
	 * status for lines 0 to 7, and byte 1 is lines 8 to 15. The
	 * 3rd has the status for the SCL/SDA/SCK pins. The 4th byte
	 * might have some remaining pin status. Byte 5 and 6 content
	 * is unknown.
	 */
	if (result == 6)
		dev->gpio_last_read = le16_to_cpu(*(u16 *)dev->gpio_buf);

	mutex_unlock(&dev->gpio_lock);

	return (result != 6) ? result : 0;
}

static int ch341_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ch341_device *dev = gpiochip_get_data(chip);
	int rc;

	rc = read_inputs(dev);
	if (rc)
		return rc;

	return (dev->gpio_last_read & BIT(offset)) ? 1 : 0;
}

static int ch341_gpio_get_multiple(struct gpio_chip *chip,
				   unsigned long *mask, unsigned long *bits)
{
	struct ch341_device *dev = gpiochip_get_data(chip);
	int rc;

	rc = read_inputs(dev);
	if (rc)
		return rc;

	*bits = dev->gpio_last_read & *mask;

	return 0;
}

static void write_outputs(struct ch341_device *dev)
{
	mutex_lock(&dev->gpio_lock);

	dev->gpio_buf[0] = CH341_CMD_UIO_STREAM;
	dev->gpio_buf[1] = CH341_CMD_UIO_STM_DIR | dev->gpio_dir;
	dev->gpio_buf[2] = CH341_CMD_UIO_STM_OUT | (dev->gpio_last_written & dev->gpio_dir);
	dev->gpio_buf[3] = CH341_CMD_UIO_STM_END;

	gpio_transfer(dev, 4, 0);

	mutex_unlock(&dev->gpio_lock);
}

static void ch341_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ch341_device *dev = gpiochip_get_data(chip);

	if (value)
		dev->gpio_last_written |= BIT(offset);
	else
		dev->gpio_last_written &= ~BIT(offset);

	write_outputs(dev);
}

static void ch341_gpio_set_multiple(struct gpio_chip *chip,
				    unsigned long *mask, unsigned long *bits)
{
	struct ch341_device *dev = gpiochip_get_data(chip);

	dev->gpio_last_written &= ~*mask;
	dev->gpio_last_written |= (*bits & *mask);

	write_outputs(dev);
}

static int ch341_gpio_get_direction(struct gpio_chip *chip,
				    unsigned int offset)
{
	struct ch341_device *dev = gpiochip_get_data(chip);

	return (dev->gpio_dir & BIT(offset)) ? 0 : 1;
}

static int ch341_gpio_direction_input(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct ch341_device *dev = gpiochip_get_data(chip);

	dev->gpio_dir &= ~BIT(offset);

	return 0;
}

static int ch341_gpio_direction_output(struct gpio_chip *chip,
				       unsigned int offset, int value)
{
	struct ch341_device *dev = gpiochip_get_data(chip);
	u16 mask = BIT(offset);

	if (!(pin_can_output & mask))
		return -EINVAL;

	dev->gpio_dir |= mask;

	ch341_gpio_set(chip, offset, value);

	return 0;
}

void ch341_gpio_remove(struct ch341_device *dev)
{
	if (!dev->gpio_init)
		return;

	gpiochip_remove(&dev->gpio);
}

int ch341_gpio_init(struct ch341_device *dev)
{
	struct gpio_chip *gpio = &dev->gpio;
	int result;

	gpio->label = "ch341";
	gpio->parent = &dev->usb_dev->dev;
	gpio->owner = THIS_MODULE;
	gpio->get_direction = ch341_gpio_get_direction;
	gpio->direction_input = ch341_gpio_direction_input;
	gpio->direction_output = ch341_gpio_direction_output;
	gpio->get = ch341_gpio_get;
	gpio->get_multiple = ch341_gpio_get_multiple;
	gpio->set = ch341_gpio_set;
	gpio->set_multiple = ch341_gpio_set_multiple;
	gpio->dbg_show = ch341_gpio_dbg_show;
	gpio->base = -1;
	gpio->ngpio = CH341_GPIO_NUM_PINS;
	gpio->can_sleep = true;

	dev->gpio_dir = 0;	/* All pins as input */

	mutex_init(&dev->gpio_lock);

	result = gpiochip_add_data(gpio, dev);
	if (result) {
		dev_err(&dev->usb_dev->dev, "Could not add GPIO\n");
		return result;
	}

	dev->gpio_init = true;

	return 0;
}
