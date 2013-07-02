Minix i2c Driver
================

WARNING: code is in development and not ready for prime time use yet.
The current state of affairs is really pretty basic. Feedback is welcome.
Send comments to tcort@minix3.org or contact tcort on IRC (irc.freenode.net).

Overview
--------

This is the driver for the i2c bus. When complete, it will provide the same
/dev interface as Linux and NetBSD/OpenBSD. 

Organization and Layout
-----------------------

i2c.c					generic i2c bus driver
arch/					arch specific code
	earm/				earm specific code
		omap_i2c.c		AM335X/DM37XX i2c bus driver	
		omap_i2c.h		AM335X/DM37XX function prototypes
		omap_i2c_registers.h 	AM335X/DM37XX register offsets, etc.

Testing the Code
----------------

Below are the steps needed to start up the i2c driver instances.

Creating the device files:

cd /dev && MAKEDEV i2c-1 && MAKEDEV i2c-2 && MAKEDEV i2c-3

Starting up the instances:

/bin/service up /usr/sbin/i2c -dev /dev/i2c-1 -label i2c_1 -args instance=1
/bin/service up /usr/sbin/i2c -dev /dev/i2c-2 -label i2c_2 -args instance=2
/bin/service up /usr/sbin/i2c -dev /dev/i2c-3 -label i2c_3 -args instance=3

There is an i2cscan program which can detect devices on the bus:

i2cscan -r /dev/i2c-1
i2cscan -r /dev/i2c-2
i2cscan -r /dev/i2c-3

Limitations
-----------

The i2c controllers used in the am335x and the dm37xx do not support zero
byte transfers. Writing 0 bytes is a common method used to probe the bus
for devices. Most of the address ranges i2cscan scans are done by this
method. Therefore, only a subset of devices on the bus will be detected by
i2cscan (i.e. the devices it detects using the 1 byte read method). See
the register description for I2C_CNT in the technical reference manuals
for details about why 0 byte transfers are not allowed.

I2C Related Task List
---------------------

The following is a list of tasks I'm working on:

 - Define i2c protocol for messages from other drivers, document on wiki
 - Implement NetBSD / OpenBSD i2c Interface
 - CAT24C256 test program that uses /dev interface
 - CAT24C256 Driver (EEPROM)
 - Test all i2c buses on BBB and xM using CAT24C256's on a bread board.
 - Test usr/sbin/i2cscan
 - Final clean up / review of i2c & cat24c256 drivers, then submit to mainline.

 - Enhance fb driver to read EDID info and setup the display accordingly.
 - TPS65217C Driver (PMIC)
 - Weather Cape (Depends on getting the hardware)
   * Drivers for each IC.
   * Demo program (some sort of data logger, plot the data, and share).
 - General Cape Infrastructure (Depends on getting the hardware)
   * The EEPROM on each cape contains pinmux info for that board. Need to
     develop code to apply the settings from the EEPROM.
