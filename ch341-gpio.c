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

/* Masks to describe the 16 GPIOs. Pins D0 to D5 (mapped to GPIOs 0 to
 * 5) can read/write, but the other pins can only read.
 */
static const u16 pin_can_output = 0b111111;

/* Only GPIO 10 (INT# line) has hardware interrupt */
#define CH341_GPIO_INT_LINE 10

static void ch341_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct ch341_device *dev = gpiochip_get_data(chip);

	seq_printf(s, "pin config  : %04x  (0=IN, 1=OUT)\n", dev->gpio_dir);
	seq_printf(s, "last read   : %04x\n", dev->gpio_last_read);
	seq_printf(s, "last written: %04x\n", dev->gpio_last_written);
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
		dev->gpio_last_read = le16_to_cpu(*(__le16 *)dev->gpio_buf);

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

	write_outputs(dev);

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

static void ch341_complete_intr_urb(struct urb *urb)
{
	struct ch341_device *dev = urb->context;
	int rc;

	if (!urb->status) {
		/* Data is 8 bytes. Byte 0 might be the length of
		 * significant data, which is 3 more bytes. Bytes 1
		 * and 2, and possibly 3, are the pin status. The byte
		 * order is different than for the GET_STATUS
		 * command. Byte 1 is GPIOs 8 to 15, and byte 2 is
		 * GPIOs 0 to 7.
		 *
		 * Something like this (with locking?) could be done,
		 * but there's nothing to retrieve that info without
		 * doing another USB read:
		 *
		 *   dev->gpio_last_read = be16_to_cpu(*(u16 *)&dev->gpio_buf_intr[1]);
		 */

		handle_nested_irq(dev->gpio_irq.num);

		rc = usb_submit_urb(dev->gpio_irq.urb, GFP_ATOMIC);
		if (rc)
			usb_unanchor_urb(dev->gpio_irq.urb);
	} else {
		usb_unanchor_urb(dev->gpio_irq.urb);
	}
}

static int ch341_gpio_irq_set_type(struct irq_data *data, u32 type)
{
	struct ch341_device *dev = irq_data_get_irq_chip_data(data);

	if (data->irq != dev->gpio_irq.num || type != IRQ_TYPE_EDGE_RISING)
		return -EINVAL;

	return 0;
}

static void ch341_gpio_irq_enable(struct irq_data *data)
{
	struct ch341_device *dev = irq_data_get_irq_chip_data(data);
	int rc;

	dev->gpio_irq.enabled = true;

	/* The URB might have just been unlinked in
	 * ch341_gpio_irq_disable, but the completion handler hasn't
	 * been called yet.
	 */
	if (!usb_wait_anchor_empty_timeout(&dev->gpio_irq.urb_out, 5000))
		usb_kill_anchored_urbs(&dev->gpio_irq.urb_out);

	usb_anchor_urb(dev->gpio_irq.urb, &dev->gpio_irq.urb_out);
	rc = usb_submit_urb(dev->gpio_irq.urb, GFP_ATOMIC);
	if (rc)
		usb_unanchor_urb(dev->gpio_irq.urb);
}

static void ch341_gpio_irq_disable(struct irq_data *data)
{
	struct ch341_device *dev = irq_data_get_irq_chip_data(data);

	dev->gpio_irq.enabled = false;
	usb_unlink_urb(dev->gpio_irq.urb);
}

/* Convert the GPIO index to the IRQ number */
static int ch341_gpio_to_irq(struct gpio_chip *chip, unsigned int offset)
{
	struct ch341_device *dev = gpiochip_get_data(chip);

	if (offset != CH341_GPIO_INT_LINE)
		return -ENXIO;

	return dev->gpio_irq.num;
}

/* Allocate a software driven IRQ, for GPIO 10 */
static int ch341_gpio_get_irq(struct ch341_device *dev)
{
	int rc;

	dev->gpio_irq.irq.name = dev->gpio_irq.name;
	dev->gpio_irq.irq.irq_set_type = ch341_gpio_irq_set_type;
	dev->gpio_irq.irq.irq_enable = ch341_gpio_irq_enable;
	dev->gpio_irq.irq.irq_disable = ch341_gpio_irq_disable;

	rc = irq_alloc_desc(0);
	if (rc < 0) {
		dev_err(&dev->usb_dev->dev, "Cannot allocate an IRQ desc");
		return rc;
	}

	dev->gpio_irq.num = rc;
	dev->gpio_irq.enabled = false;

	irq_set_chip_data(dev->gpio_irq.num, dev);
	irq_set_chip_and_handler(dev->gpio_irq.num, &dev->gpio_irq.irq,
				 handle_simple_irq);

	return 0;
}

void ch341_gpio_remove(struct ch341_device *dev)
{
	if (!dev->gpio_init)
		return;

	usb_kill_anchored_urbs(&dev->gpio_irq.urb_out);
	usb_free_urb(dev->gpio_irq.urb);

	gpiochip_remove(&dev->gpio);
	irq_free_desc(dev->gpio_irq.num);
}

int ch341_gpio_init(struct ch341_device *dev)
{
	struct gpio_chip *gpio = &dev->gpio;
	int rc;

	snprintf(dev->gpio_irq.name, sizeof(dev->gpio_irq.name),
		 "ch341-%s-gpio", dev_name(&dev->usb_dev->dev));
	dev->gpio_irq.name[sizeof(dev->gpio_irq.name) - 1] = 0;

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
	gpio->to_irq = ch341_gpio_to_irq;

	dev->gpio_dir = 0;	/* All pins as input */

	mutex_init(&dev->gpio_lock);

	rc = ch341_gpio_get_irq(dev);
	if (rc)
		return rc;

	rc = gpiochip_add_data(gpio, dev);
	if (rc) {
		dev_err(&dev->usb_dev->dev, "Could not add GPIO\n");
		goto release_irq;
	}

	/* create an URB for handling interrupt */
	dev->gpio_irq.urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!dev->gpio_irq.urb) {
		dev_err(&dev->usb_dev->dev, "Cannot alloc the int URB");
		rc = -ENOMEM;
		goto release_gpio;
	}

	usb_fill_int_urb(dev->gpio_irq.urb, dev->usb_dev,
			 usb_rcvintpipe(dev->usb_dev, dev->ep_intr),
			 dev->gpio_irq.buf, CH341_USB_MAX_INTR_SIZE,
			 ch341_complete_intr_urb, dev, dev->ep_intr_interval);

	init_usb_anchor(&dev->gpio_irq.urb_out);

	dev->gpio_init = true;

	return 0;

release_gpio:
	gpiochip_remove(&dev->gpio);

release_irq:
	irq_free_desc(dev->gpio_irq.num);

	return rc;
}
