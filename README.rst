WinChipHead (沁恒) CH341 linux driver for I2C / SPI and GPIO mode
================================================================

The CH341 is declined in several flavors, and may support one or more
of UART, SPI, I2C and GPIO, but not always simultaneously::

  - CH341 A/B/F: UART, SPI, I2C and GPIO
  - CH341 C/T: UART and I2C
  - CH341 H: SPI

They work in 3 different modes, with only one being presented
depending on the USB PID::

  - 0x5523: UART mode, covered by the USB `ch341` serial driver
  - 0x5512: SPI/I2C/GPIO mode, covered by this ch341_buses driver
  - 0x5584: some printer mode emulation (?) - no linux driver

The 0x5512 PID is unfortunately also claimed in the linux kernels 5.10
or above by the driver for the UART part, and will conflict with this
driver. Blacklisting that module or deleting it will solve that
problem. In `/etc/modprobe.d/blacklist.conf`, add the following line
to prevent loading of the serial driver::

  blacklist ch341

Mode selection is done at the hardware level by tying some
pins. Breakout boards with one of the CH341 chip usually have one or
more jumpers to select which mode they work on. At least one model
(CJMCU-341) appears to need bridging some solder pads to select a
different default.

The various CH341 appear to be indistinguishable from the
software. For instance the ch341-buses driver will present a GPIO
interface for the CH341T although physical pins are not present, and
the device will accept GPIO commands.

Some breakout boards work in 3.3V and 5V depending on some
jumpers.

The black chip programmer with a ZIF socket will power the CH341 at
3.3V if a jumper is set, but will only output 5V to the chips to be
programmed, which is not always desirable. A hardware hack to use 3.3V
everywhere, involving some soldering, is available there:

  https://eevblog.com/forum/repair/ch341a-serial-memory-programmer-power-supply-fix/

The ch341-buses driver has been tested with a CH341A, CH341B and
CH341T.

Some sample code for the CH341 is available at the manufacturer
website:

  http://wch-ic.com/products/CH341.html

The following repository contains a lot of information on these chips,
including datasheets.

  https://github.com/boseji/CH341-Store.git

This driver is based on, merges, and expands the following
pre-existing works:

  https://github.com/gschorcht/spi-ch341-usb.git
  https://github.com/gschorcht/i2c-ch341-usb.git


Building the driver
-------------------

The driver will build for the active kernel::

  $ make

This will create `ch341-buses.ko`, which can the be insmod'ed.

The driver has been tested with a linux kernel 5.11. It will also
build for a linux kernel 5.14.

Setup
-----

Although it's possible to access everything as root, or even to give a
user some rights to access the i2c/spi/gpio subsystem, some of these
resources are critical to the system, and reading or writing to them
might make a system unstable.

  WARNING! Accidentally accessing a motherboard or graphic card I2C or
  SPI device may render the former unoperable. Double check that the
  correct device is accessed.

The following is more safe. As root, create a group, add the user to
the group and create a udev rule for that group that will bind to the
devices recognized by the driver::

  $ groupadd ch341
  $ adduser "$USER" ch341
  $ echo 'SUBSYSTEMS=="usb" ATTRS{idProduct}=="5512" ATTRS{idVendor}=="1a86" GROUP="ch341" MODE="0660"' > /etc/udev/rules.d/99-ch341.rules

After plugging in the USB device, the various /dev entries will be
accessible to the ch341 group::

  $ ls -l /dev/* | grep ch341
  crw-rw----   1 root ch341   254,   2 Sep 20 01:12 /dev/gpiochip2
  crw-rw----   1 root ch341    89,  11 Sep 20 01:12 /dev/i2c-11
  crw-rw----   1 root ch341   153,   0 Sep 20 01:12 /dev/spidev0.0


I2C
---

The ch341 supports 4 different speeds: 20kHz, 100kHz, 400kHz and
750kHz. The driver only supports 100kHz by default, and that currently
cannot be dynamically changed. It is possible to change it in the
ch341_i2c_init() function. A future patch should address that issue.

To find the device number::

  $ i2cdetect -l
  ...
  i2c-11        unknown           CH341 I2C USB bus 003 device 005        N/A

Adding support for a device supported by Linux is easy. For instance::

  modprobe bmi160_i2c
  echo "bmi160 0x68" > /sys/bus/i2c/devices/i2c-$DEV/new_device

or::

  modprobe tcs3472
  echo "tcs3472 0x29" > /sys/bus/i2c/devices/i2c-$DEV/new_device

Files from these drivers will be created somewhere in
/sys/bus/i2c/devices/i2c-$DEV/

Caveats
~~~~~~~

The ch341 doesn't work with a Wii nunchuk, possibly because the
pull-up value is too low (1500 ohms).

i2c AT24 eeproms can be read but not programmed properly because the
at24 linux driver tries to write a byte at a time, and doesn't wait at
all (or enough) between writes. Data corruption on writes does occur.

The driver doesn't support detection of I2C device present on the
bus. Apparently when a device is not present at a given adress, the
CH341 will return an extra byte of data, but the driver doesn't
support that. This may be adressed in a future patch.


The GPIOs
---------

8 GPIOs are available on the CH341 A/B/F. The first 6 are input/output,
and the last 2 are input only.

Pinout and their names as they appear on some breakout boards::

  CH341A/B/F     GPIO  Names                    Mode
    pin          line

   15             0     D0, CS0                  input/output
   16             1     D1, CS1                  input/output
   17             2     D2, CS2                  input/output
   18             3     D3, SCK, DCK             input/output
   19             4     D4, DOUT2, CS3           input/output
   20             5     D5, MOSI, DOUT, SDO      input/output
   21             6     D6, DIN2                 input
   22             7     D7, MISO, DIN            input

They can be used with the standard linux GPIO interface. Note that
MOSI/MISO/SCK may be used by SPI, when SPI is enabled.

To drive the GPIOs, one can use the regular linux tools. `gpiodetect`
will report the device number to use for the other tools (run as root)::

  $ gpiodetect
  ...
  gpiochip2 [ch341] (8 lines)

  $ gpioinfo gpiochip2
  gpiochip2 - 8 lines:
          line   0:      unnamed       unused   input  active-high
          line   1:      unnamed       unused   input  active-high
          line   2:      unnamed       unused   input  active-high
          line   3:      unnamed       unused   input  active-high
          line   4:      unnamed       unused   input  active-high
          line   5:      unnamed       unused   input  active-high
          line   6:      unnamed       unused   input  active-high
          line   7:      unnamed       unused   input  active-high

  $ gpioset gpiochip2 0=0 1=1 2=0
  $ gpioget gpiochip2 5

If the SPI mode is enabled, the MOSI, MISO and SCK, and possible one
or more of CS0/1/2, won't be available.

On Ubuntu 21.04, the `libgpio` is too old and will return error when
acessing the device. Use a more recent library. The `master` branch
from the git tree works well::

  https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git


SPI
---

See above for how SPI and GPIO exclusively share some pins.

By default, SPI is enabled and only controls chip select line 0
(CS0). This can be changed with the 'spi_cs' module parameter. This
affect all devices controlled by the driver. A future enhancement
might allow a per-device setting. If 'spi_cs' is set to -1, SPI is
disabled altogether and all pins can be used for GPIOs. Otherwise
which pins are available for SPI can be set by the 'spi_cs', which is
a mask of the CS lines::

   1: CS0
   2: CS1
   4: CS2
   8: CS3

For instance, loading the driver as follows will enable SPI, with CS0
and CS2::

  $ insmod ch341-buses.ko spi_cs=5

Loading the driver that way will disable the SPI support::

  $ insmod ch341-buses.ko spi_cs=-1
