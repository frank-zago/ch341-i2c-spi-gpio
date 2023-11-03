WinChipHead (沁恒) CH341 linux drivers for I2C / SPI and GPIO mode
=================================================================

The CH341 is declined in several flavors, and may support one or more
of UART, SPI, I2C and GPIO, but not always simultaneously::

  - CH341 A/B/F: UART, Printer, SPI, I2C and GPIO
  - CH341 C/T: UART and I2C
  - CH341 H: SPI

They work in 3 different modes, with only one being presented
depending on the USB PID::

  - 0x5523: UART mode, covered by the USB `ch341` serial driver
  - 0x5512: SPI/I2C/GPIO mode, covered by this set of drivers
  - 0x5584: Parallel printer mode, covered by the USB `usblp` driver

From linux kernel 5.10 to 5.16, the 0x5512 PID was unfortunately also
claimed by the driver for the UART part, and will conflict with these
drivers. Blacklisting that module or deleting it will solve that
problem. In `/etc/modprobe.d/blacklist.conf`, add the following line
to prevent loading of the serial driver::

  blacklist ch341

Mode selection is done at the hardware level by tying some
pins. Breakout boards with one of the CH341 chip usually have one or
more jumpers to select which mode they work on. At least one model
(CJMCU-341) appears to need bridging some solder pads to select a
different default. Breakout boards also don't usually offer an option
to configure the chip into printer mode; for that case, connect the
SCL and SDA lines directly together.

The various CH341 appear to be indistinguishable from the
software. For instance the gpio-ch341 driver will present a GPIO
interface for the CH341T although physical pins are not present, and
the device will accept GPIO commands.

Some breakout boards work in 3.3V and 5V depending on some
jumpers.

The black chip programmer with a ZIF socket will power the CH341 at
3.3V if a jumper is set, but will only output 5V to the chips to be
programmed, which is not always desirable. A hardware hack to use 3.3V
everywhere, involving some soldering, is available there::

  https://eevblog.com/forum/repair/ch341a-serial-memory-programmer-power-supply-fix/

These drivers have been tested with a CH341A, CH341B and CH341T.

Some sample code for the CH341 is available at the manufacturer
website::

  http://wch-ic.com/products/CH341.html

The following repository contains a lot of information on these chips,
including datasheets.

  https://github.com/boseji/CH341-Store.git

This set of drivers is based on, merges, and expands the following
pre-existing works::

  https://github.com/gschorcht/spi-ch341-usb.git
  https://github.com/gschorcht/i2c-ch341-usb.git

Warning: try not to yank the USB device out if it's being used. The
linux subsystems gpio and spi may crash or leak resources. This is not
a problem with the drivers, but the subsystems themselves.


Building the drivers
--------------------

The drivers will build for the active kernel::

  $ make

This will create four drivers: `ch341-core.ko`, `gpio-ch341.ko`,
`i2c-ch341.ko` and `spi-ch341.ko` which can then be insmod'ed, in that
order.

This driver used to be in a single piece, called `ch341-buses.ko`.
However since the goal is to upstream it, it was not possible to keep
it as is.

It is possibly to override the target kernel by setting the KDIR
variable in the working environment, to point to a built kernel tree.

These drivers have been tested with a linux kernel 6.2, and should
still build for older kernels.

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
devices recognized by the core driver::

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

I2cdetect requires the i2c-dev module to be loaded. You can load it once::

  $ modprobe i2c-dev

Or add it to /etc/modules-load.d/ to autoload the module at boot time::

  $ echo 'i2c-dev' > /etc/modules-load.d/i2c-dev.conf
  $ systemctl restart systemd-modules-load.service

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

The GPIOs
---------

16 GPIOs are available on the CH341 A/B/F. The first 6 are input/output,
and the last 10 are input only.

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
    5             8     ERR                      input
    6             9     PEMP                     input
    7            10     INT                      input
    8            11     SLCT (SELECT)            input
    ?            12     ?                        input
   27            13     WT (WAIT)                input
    4            14     DS (Data Select?)        input
    3            15     AS (Address Select?)     input


They can be used with the standard linux GPIO interface. Note that
MOSI/MISO/SCK may be used by SPI, when SPI is enabled.

To drive the GPIOs, one can use the regular linux tools. `gpiodetect`
will report the device number to use for the other tools (run as root)::

  $ gpiodetect
  ...
  gpiochip2 [ch341] (16 lines)

  $ gpioinfo gpiochip2
  gpiochip2 - 16 lines:
          line   0:      unnamed       unused   input  active-high
          line   1:      unnamed       unused   input  active-high
          line   2:      unnamed       unused   input  active-high
          line   3:      unnamed       unused   input  active-high
          line   4:      unnamed       unused   input  active-high
          line   5:      unnamed       unused   input  active-high
          line   6:      unnamed       unused   input  active-high
          line   7:      unnamed       unused   input  active-high
	  [......]
          line  15:      unnamed       unused   input  active-high

  $ gpioset gpiochip2 0=0 1=1 2=0
  $ gpioget gpiochip2 5

If the SPI mode is enabled, the MOSI, MISO and SCK, and possible one
or more of CS0/1/2, won't be available.

On Ubuntu 21.04, the `libgpio` is too old and will return an error
when accessing the device. Use a more recent library. The `master`
branch from the git tree works well::

  https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git

GPIO interrupt
~~~~~~~~~~~~~~

The INT pin, corresponding to GPIO 10 is an input pin that can trigger
an interrupt on a rising edge. Only that pin is able to generate an
interrupt, and only on a rising edge. Trying to monitor events on
another GPIO, or that GPIO on something other than a rising edge, will
be rejected.

As an example, physically connect the INT pin to CS2. Start the
monitoring of the INT pin::

  $ gpiomon -r gpiochip2 10

The INT will be triggered by setting CS2 low then high::

  $ gpioset gpiochip2 2=0 && gpioset gpiochip2 2=1

`gpiomon` will report rising events like this:

  event:  RISING EDGE offset: 10 timestamp: [     191.539358302]
  ...


SPI
---

See above for how SPI and GPIO exclusively share some pins.

Only SPI mode 0 (CPOL=0, CPHA=0) appears to be supported by the ch341.

As long as no SPI device has been instantiated, all the GPIOs are
available for general use. When the first device is instantiated, the
driver will try to claim the SPI lines, plus one of the chip select.

To instantiate a device, echo a command string to the device's sysfs
'new_device' file. The command is the driver to use followed by the CS
number. For instance, the following declares a user device (spidev) at
CS 0, and a flash memory at CS 1::

  $ echo "spidev 0" > /sys/class/spi_master/spi0/new_device
  $ echo "spi-nor 1" > /sys/class/spi_master/spi0/new_device

Starting with the Linux kernel 5.15 or 5.16, the following steps are
also needed for each added device for the /dev/spidevX entries to
appear::

    echo spidev > /sys/bus/spi/devices/spi0.0/driver_override
    echo spi0.0 > /sys/bus/spi/drivers/spidev/bind

Change spi0 and spi0.0 as appropriate.

After these command, the GPIO lines will report::

  $ gpioinfo gpiochip2
  gpiochip2 - 16 lines:
          line   0:      unnamed        "CS0"  output  active-high [used]
          line   1:      unnamed        "CS1"  output  active-high [used]
          line   2:      unnamed       unused   input  active-high
          line   3:      unnamed        "SCK"  output  active-high [used]
          line   4:      unnamed       unused   input  active-high
          line   5:      unnamed       "MOSI"  output  active-high [used]
          line   6:      unnamed       unused   input  active-high
          line   7:      unnamed       "MISO"   input  active-high [used]
          line   8:      unnamed       unused   input  active-high
          ...
          line  15:      unnamed       unused   input  active-high

To remove a device, echo its CS to 'delete_device'. The following will
remove the spidev device created on CS 1 above::

  $ echo "1" > /sys/class/spi_master/spi0/delete_device

If all the devices are deleted, the SPI driver will release the SPI
lines, which become available again for GPIO operations.


Developing the drivers
----------------------

This driver (and other USB drivers) can easily be developed and
tested in a VM, using QEMU and virtme (available in some distributions or at
https://git.kernel.org/cgit/utils/kernel/virtme/virtme.git/).

The following command will boot a VM under 10 seconds with any CH341
in I2C mode passed through::

  virtme-run --pwd --installed-kernel --qemu-opts -usb -device usb-host,vendorid=0x1a86,productid=0x5512

Build the VM on the host, but test the module in the VM. Add
the --rwdir option to be able to write files to the host. Type `ctrl-a x`
to exit the VM.

The amount of loaded drivers is going to be minimal. More modules may
need to be loaded, such as i2c-dev, spi-nor or mtd, depending on
usage.
