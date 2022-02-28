/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Definitions for CH341 driver
 */

#include <linux/usb.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>

#define DEFAULT_TIMEOUT 1000	/* 1s USB requests timeout */

/* I2C - The maximum request size is 4096 bytes, both for reading and
 * writing, split in up to 128 32-byte segments. The I2C stream must
 * start and stop in each 32-byte segment. Reading must also be
 * split, with up to 32-byte per segment.
 */
#define SEG_SIZE 32
#define SEG_COUNT 128

/* Number of SPI devices the device can control */
#define CH341_SPI_MAX_NUM_DEVICES 4

/* Number of segments in each SPI transfer buffer. The device will
 * crash if more than 4.
 */
#define CH341_SPI_NSEGS 4

#define CH341_USB_MAX_INTR_SIZE 8

struct ch341_device {
	struct usb_device *usb_dev;
	struct usb_interface *iface;
	struct mutex usb_lock;

	int ep_in;
	int ep_out;
	int ep_intr;
	u8 ep_intr_interval;

	/* I2C */
	struct i2c_adapter adapter;
	bool i2c_init;

	/* I2C request and response state */
	int idx_out;		/* current offset in buf */
	int out_seg;		/* current segment */
	u8 i2c_buf[SEG_COUNT * SEG_SIZE];

	/* GPIO */
	struct gpio_chip gpio;
	struct mutex gpio_lock;
	bool gpio_init;
	u16 gpio_dir;		/* 1 bit per pin, 0=IN, 1=OUT. */
	u16 gpio_last_read;	/* last GPIO values read */
	u16 gpio_last_written;	/* last GPIO values written */
	u8 gpio_buf[SEG_SIZE];

	struct {
		char name[32];
		bool enabled;
		struct irq_chip irq;
		int num;
		struct urb *urb;
		struct usb_anchor urb_out;
		u8 buf[CH341_USB_MAX_INTR_SIZE];
	} gpio_irq;

	/* SPI */
	struct spi_master *master;
	struct mutex spi_lock;
	u8 cs_allocated;	/* bitmask of allocated CS for SPI */
	struct gpio_desc *spi_gpio_core_desc[3];
	struct spi_client {
		struct spi_device *slave;
		struct gpio_desc *gpio;
		u8 buf[CH341_SPI_NSEGS * SEG_SIZE];
	} spi_clients[CH341_SPI_MAX_NUM_DEVICES];
};

void ch341_i2c_remove(struct ch341_device *dev);
int ch341_i2c_init(struct ch341_device *dev);
void ch341_gpio_remove(struct ch341_device *dev);
int ch341_gpio_init(struct ch341_device *dev);
void ch341_spi_remove(struct ch341_device *dev);
int ch341_spi_init(struct ch341_device *dev);
