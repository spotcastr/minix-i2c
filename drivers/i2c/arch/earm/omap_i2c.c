/*
 * This file implements support for i2c on the BeagleBone and BeagleBoard-xM
 *
 * Based on the NetBSD driver sys/arch/arm/omap/omap3_i2c.c which has the
 * following terms and conditions:
 *
 * Copyright (c) 2012 Jared D. McNeill <jmcneill@invisible.ca>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/* kernel headers */
#include <minix/chardriver.h>
#include <minix/clkconf.h>
#include <minix/drivers.h>
#include <minix/ds.h>
#include <minix/log.h>
#include <minix/mmio.h>
#include <minix/padconf.h>
#include <minix/sysutil.h>
#include <minix/type.h>

/* device headers */
#include <dev/i2c/i2c_io.h>
#include <minix/i2c.h>

/* system headers */
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>

/* usr headers */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* local headers */
#include "omap_i2c.h"

/* defines the set of register */

typedef struct omap_i2c_registers
{
	vir_bytes I2C_REVNB_LO;	/* AM335X Only */
	vir_bytes I2C_REVNB_HI;	/* AM335X Only */
	vir_bytes I2C_REV;	/* DM37XX Only */
	vir_bytes I2C_IE;	/* DM37XX Only */
	vir_bytes I2C_STAT;	/* DM37XX Only */
	vir_bytes I2C_SYSC;
	vir_bytes I2C_IRQSTATUS_RAW;	/* AM335X Only */
	vir_bytes I2C_IRQSTATUS;	/* AM335X Only */
	vir_bytes I2C_IRQENABLE_SET;	/* AM335X Only */
	vir_bytes I2C_IRQENABLE_CLR;	/* AM335X Only */
	vir_bytes I2C_WE;
	vir_bytes I2C_DMARXENABLE_SET;	/* AM335X Only */
	vir_bytes I2C_DMATXENABLE_SET;	/* AM335X Only */
	vir_bytes I2C_DMARXENABLE_CLR;	/* AM335X Only */
	vir_bytes I2C_DMATXENABLE_CLR;	/* AM335X Only */
	vir_bytes I2C_DMARXWAKE_EN;	/* AM335X Only */
	vir_bytes I2C_DMATXWAKE_EN;	/* AM335X Only */
	vir_bytes I2C_SYSS;
	vir_bytes I2C_BUF;
	vir_bytes I2C_CNT;
	vir_bytes I2C_DATA;
	vir_bytes I2C_CON;
	vir_bytes I2C_OA;	/* AM335X Only */
	vir_bytes I2C_OA0;	/* DM37XX Only */
	vir_bytes I2C_SA;
	vir_bytes I2C_PSC;
	vir_bytes I2C_SCLL;
	vir_bytes I2C_SCLH;
	vir_bytes I2C_SYSTEST;
	vir_bytes I2C_BUFSTAT;
	vir_bytes I2C_OA1;
	vir_bytes I2C_OA2;
	vir_bytes I2C_OA3;
	vir_bytes I2C_ACTOA;
	vir_bytes I2C_SBLOCK;
} omap_i2c_regs_t;

/* generic definition an i2c bus */

typedef struct omap_i2c_bus
{
	enum bus_types
	{ am335x, dm37xx } bus_type;
	phys_bytes mr_base;
	phys_bytes mr_size;
	vir_bytes mapped_addr;
	omap_i2c_regs_t *regs;
	uint32_t functional_clock;
	uint32_t module_clock;
	int irq;
	int irq_hook_id;
	int irq_hook_kernel_id;
} omap_i2c_bus_t;

/* Define the registers for each chip */

static omap_i2c_regs_t am335x_i2c_regs = {
	.I2C_REVNB_LO = AM335X_I2C_REVNB_LO,
	.I2C_REVNB_HI = AM335X_I2C_REVNB_HI,
	.I2C_SYSC = AM335X_I2C_SYSC,
	.I2C_IRQSTATUS_RAW = AM335X_I2C_IRQSTATUS_RAW,
	.I2C_IRQSTATUS = AM335X_I2C_IRQSTATUS,
	.I2C_IRQENABLE_SET = AM335X_I2C_IRQENABLE_SET,
	.I2C_IRQENABLE_CLR = AM335X_I2C_IRQENABLE_CLR,
	.I2C_WE = AM335X_I2C_WE,
	.I2C_DMARXENABLE_SET = AM335X_I2C_DMARXENABLE_SET,
	.I2C_DMATXENABLE_SET = AM335X_I2C_DMATXENABLE_SET,
	.I2C_DMARXENABLE_CLR = AM335X_I2C_DMARXENABLE_CLR,
	.I2C_DMATXENABLE_CLR = AM335X_I2C_DMATXENABLE_CLR,
	.I2C_DMARXWAKE_EN = AM335X_I2C_DMARXWAKE_EN,
	.I2C_DMATXWAKE_EN = AM335X_I2C_DMATXWAKE_EN,
	.I2C_SYSS = AM335X_I2C_SYSS,
	.I2C_BUF = AM335X_I2C_BUF,
	.I2C_CNT = AM335X_I2C_CNT,
	.I2C_DATA = AM335X_I2C_DATA,
	.I2C_CON = AM335X_I2C_CON,
	.I2C_OA = AM335X_I2C_OA,
	.I2C_SA = AM335X_I2C_SA,
	.I2C_PSC = AM335X_I2C_PSC,
	.I2C_SCLL = AM335X_I2C_SCLL,
	.I2C_SCLH = AM335X_I2C_SCLH,
	.I2C_SYSTEST = AM335X_I2C_SYSTEST,
	.I2C_BUFSTAT = AM335X_I2C_BUFSTAT,
	.I2C_OA1 = AM335X_I2C_OA1,
	.I2C_OA2 = AM335X_I2C_OA2,
	.I2C_OA3 = AM335X_I2C_OA3,
	.I2C_ACTOA = AM335X_I2C_ACTOA,
	.I2C_SBLOCK = AM335X_I2C_SBLOCK
};

static omap_i2c_regs_t dm37xx_i2c_regs = {
	.I2C_REV = DM37XX_I2C_REV,
	.I2C_IE = DM37XX_I2C_IE,
	.I2C_STAT = DM37XX_I2C_STAT,
	.I2C_WE = DM37XX_I2C_WE,
	.I2C_SYSS = DM37XX_I2C_SYSS,
	.I2C_BUF = DM37XX_I2C_BUF,
	.I2C_CNT = DM37XX_I2C_CNT,
	.I2C_DATA = DM37XX_I2C_DATA,
	.I2C_SYSC = DM37XX_I2C_SYSC,
	.I2C_CON = DM37XX_I2C_CON,
	.I2C_OA0 = DM37XX_I2C_OA0,
	.I2C_SA = DM37XX_I2C_SA,
	.I2C_PSC = DM37XX_I2C_PSC,
	.I2C_SCLL = DM37XX_I2C_SCLL,
	.I2C_SCLH = DM37XX_I2C_SCLH,
	.I2C_SYSTEST = DM37XX_I2C_SYSTEST,
	.I2C_BUFSTAT = DM37XX_I2C_BUFSTAT,
	.I2C_OA1 = DM37XX_I2C_OA1,
	.I2C_OA2 = DM37XX_I2C_OA2,
	.I2C_OA3 = DM37XX_I2C_OA3,
	.I2C_ACTOA = DM37XX_I2C_ACTOA,
	.I2C_SBLOCK = DM37XX_I2C_SBLOCK
};

/* Define the buses available on each chip */

static omap_i2c_bus_t am335x_i2c_buses[] = {
	{am335x, AM335X_I2C0_BASE, AM335X_I2C0_SIZE, 0, &am335x_i2c_regs,
		    AM335X_FUNCTIONAL_CLOCK, AM335X_MODULE_CLOCK,
	    AM335X_I2C0_IRQ, 1, 1},
	{am335x, AM335X_I2C1_BASE, AM335X_I2C1_SIZE, 0, &am335x_i2c_regs,
		    AM335X_FUNCTIONAL_CLOCK, AM335X_MODULE_CLOCK,
	    AM335X_I2C1_IRQ, 2, 3},
	{am335x, AM335X_I2C2_BASE, AM335X_I2C2_SIZE, 0, &am335x_i2c_regs,
		    AM335X_FUNCTIONAL_CLOCK, AM335X_MODULE_CLOCK,
	    AM335X_I2C2_IRQ, 3, 3}
};

#define AM335X_OMAP_NBUSES (sizeof(am335x_i2c_buses) / sizeof(omap_i2c_bus_t))

static omap_i2c_bus_t dm37xx_i2c_buses[] = {
	{dm37xx, DM37XX_I2C0_BASE, DM37XX_I2C0_SIZE, 0, &dm37xx_i2c_regs,
		    DM37XX_FUNCTIONAL_CLOCK, DM37XX_MODULE_CLOCK,
	    DM37XX_I2C0_IRQ, 1, 1},
	{dm37xx, DM37XX_I2C1_BASE, DM37XX_I2C1_SIZE, 0, &dm37xx_i2c_regs,
		    DM37XX_FUNCTIONAL_CLOCK, DM37XX_MODULE_CLOCK,
	    DM37XX_I2C1_IRQ, 2, 2},
	{dm37xx, DM37XX_I2C2_BASE, DM37XX_I2C2_SIZE, 0, &dm37xx_i2c_regs,
		    DM37XX_FUNCTIONAL_CLOCK, DM37XX_MODULE_CLOCK,
	    DM37XX_I2C2_IRQ, 3, 3}
};

#define DM37XX_OMAP_NBUSES (sizeof(dm37xx_i2c_buses) / sizeof(omap_i2c_bus_t))

#define I2C_F_STOP	0x04

/* Globals */

static omap_i2c_bus_t *omap_i2c_buses;	/* all available buses for this SoC */
static omap_i2c_bus_t *omap_i2c_bus;	/* the bus selected at start-up */
static int omap_i2c_nbuses;	/* number of buses supported by SoC */

/* logging - use with log_warn(), log_info(), log_debug(), log_trace() */
static struct log log = {
	.name = "i2c",
	.log_level = LEVEL_INFO,
	.log_func = default_log
};

/* Local Function Prototypes */

/* Implementation of Generic I2C Interface using Bus Specific Code */
static int omap3_i2c_exec(minix_i2c_ioctl_exec_t * m);

/* Bus Specific Code */
static int omap3_i2c_reset(void);
static int omap3_i2c_read(i2c_addr_t addr, uint8_t * buf, size_t buflen,
    int flags);
static int omap3_i2c_write(i2c_addr_t addr, const uint8_t * buf, size_t buflen,
    int flags);
static int omap3_i2c_wait(uint16_t mask, uint16_t val);
static int omap3_i2c_stat(void);
static int omap3_i2c_flush(void);

static void omap3_i2c_padconf(int i2c_bus_id);
static void omap3_i2c_clkconf(int i2c_bus_id);
static void omap3_i2c_irq_enable(void);

static uint16_t omap3_i2c_read_status(void);
static void omap3_i2c_write_status(uint16_t mask);

/* Helpers for register access */

#define reg_read(a)    (*(volatile uint16_t *)(omap_i2c_bus->mapped_addr + a))
#define reg_write(a,v) (*(volatile uint16_t *)(omap_i2c_bus->mapped_addr + a) = (v))
#define reg_set_bit(a,v)   reg_write((a), reg_read((a)) | (1<<v))
#define reg_clear_bit(a,v) reg_write((a), reg_read((a)) & ~(1<<v))

/*
 * Performs the action in minix_i2c_ioctl_exec_t.
 */
static int
omap3_i2c_exec(minix_i2c_ioctl_exec_t * ioctl_exec)
{
	int err;
	int flags;

	err = 0;
	flags = 0;

	if (ioctl_exec->iie_cmdlen > 0) {
		err =
		    omap3_i2c_write(ioctl_exec->iie_addr, ioctl_exec->iie_cmd,
		    ioctl_exec->iie_cmdlen,
		    I2C_OP_READ_P(ioctl_exec->iie_op) ? 0 : I2C_F_STOP);
		if (err)
			goto done;

		omap3_i2c_flush();
	}

	if (I2C_OP_STOP_P(ioctl_exec->iie_op))
		flags |= I2C_F_STOP;

	/*
	 * Zero data bytes transfers are not allowed. The controller treats
	 * I2C_CNT register value of 0x0 as 65536. This is true for both the
	 * am335x and dm37xx. Full details in the TRM on the I2C_CNT page.
	 */
	if (ioctl_exec->iie_buflen == 0) {
		err = EINVAL;
		goto done;
	}

	if (I2C_OP_READ_P(ioctl_exec->iie_op)) {
		err = omap3_i2c_read(ioctl_exec->iie_addr, ioctl_exec->iie_buf,
		    ioctl_exec->iie_buflen, flags);
	} else {
		err =
		    omap3_i2c_write(ioctl_exec->iie_addr, ioctl_exec->iie_buf,
		    ioctl_exec->iie_buflen, flags);
	}

      done:

	if (err) {
		omap3_i2c_reset();
	}

	omap3_i2c_flush();

	return err;
}

static int
omap3_i2c_reset(void)
{
	int tries;
	uint16_t intmask;
	uint32_t psc, scll, sclh;

	/* Disable to do soft reset */
	reg_write(omap_i2c_bus->regs->I2C_CON, 0);
	micro_delay(50000);

	/* Do a soft reset */
	reg_write(omap_i2c_bus->regs->I2C_SYSC, (1 << SRST));

	/* Have to temporarily enable I2C to read RDONE */
	reg_set_bit(omap_i2c_bus->regs->I2C_CON, I2C_EN);
	micro_delay(50000);

	/* wait for reset to complete */
	for (tries = 0; tries < 10000; tries++) {
		if (reg_read(omap_i2c_bus->regs->I2C_SYSS) & (1 << RDONE)) {
			break;
		}
		micro_delay(1000);
	}

	/* Disable */
	if (reg_read(omap_i2c_bus->regs->I2C_CON) & (1 << I2C_EN)) {
		reg_write(omap_i2c_bus->regs->I2C_CON, 0);
		micro_delay(50000);
	}

	/* XXX standard speed only */
	psc =
	    (omap_i2c_bus->functional_clock / omap_i2c_bus->module_clock) - 1;
	scll = sclh =
	    (omap_i2c_bus->module_clock / (2 * BUS_SPEED_100KHz)) - 6;

	reg_write(omap_i2c_bus->regs->I2C_PSC, psc);
	reg_write(omap_i2c_bus->regs->I2C_SCLL, scll);
	reg_write(omap_i2c_bus->regs->I2C_SCLH, sclh);

	/* Set own I2C address */
	if (omap_i2c_bus->bus_type == am335x) {
		reg_write(omap_i2c_bus->regs->I2C_OA, I2C_OWN_ADDRESS);
	} else if (omap_i2c_bus->bus_type == dm37xx) {
		reg_write(omap_i2c_bus->regs->I2C_OA0, I2C_OWN_ADDRESS);
	} else {
		log_warn(&log, "Don't know how to set own address.\n");
	}

	/* Enable */
	reg_write(omap_i2c_bus->regs->I2C_CON, (1 << I2C_EN));
	micro_delay(50000);

	/* Enable interrupts (required even for polling mode) */
	intmask = 0;
	intmask |= (1 << ROVR);
	intmask |= (1 << AERR);
	intmask |= (1 << XRDY);
	intmask |= (1 << RRDY);
	intmask |= (1 << ARDY);
	intmask |= (1 << NACK);
	intmask |= (1 << AL);

	if (omap_i2c_bus->bus_type == am335x) {
		reg_write(omap_i2c_bus->regs->I2C_IRQENABLE_SET, intmask);
	} else if (omap_i2c_bus->bus_type == dm37xx) {
		reg_write(omap_i2c_bus->regs->I2C_IE, intmask);
	} else {
		log_warn(&log, "Don't know how to enable interrupts.\n");
	}

	return 0;
}

static int
omap3_i2c_read(i2c_addr_t addr, uint8_t * buf, size_t buflen, int flags)
{
	uint16_t con, stat;
	int err, i, retry;

	err = omap3_i2c_wait(1 << BB, 0);
	if (err) {
		log_warn(&log, "Bus Busy\n");
		return err;
	}

	/* Set control register */
	con |= (1 << I2C_EN);	/* enabled */
	con |= (1 << MST);	/* master mode */
	con |= (1 << STT);	/* start condition */

	if (flags & I2C_F_STOP)
		con |= (1 << STP);	/* stop condition */

	/* Set address of slave device */
	addr &= ADDRESS_MASK;	/* sanitize address (10-bit max) */
	if (addr & ~0x7f) {
		/* 10-bit extended address in use, need to set XSA */
		con |= (1 << XSA);
	}

	reg_write(omap_i2c_bus->regs->I2C_CNT, buflen);
	reg_write(omap_i2c_bus->regs->I2C_SA, addr);
	reg_write(omap_i2c_bus->regs->I2C_CON, con);

	for (i = 0; i < buflen; i++) {
		/* Ready to read? */
		stat = omap3_i2c_stat();

		if ((stat & (1 << RRDY)) == 0) {
			log_warn(&log, "No RRDY Flag: stat = %x\n", stat);
			return EBUSY;
		}

		buf[i] = reg_read(omap_i2c_bus->regs->I2C_DATA) & 0xff;

		omap3_i2c_write_status((1 << RRDY));
	}

	if (omap3_i2c_read_status() & (1 << NACK)) {
		log_warn(&log, "NACK\n");
		return EIO;
	}

	retry = 1000;
	/* reg_write(omap_i2c_bus->regs->I2C_CON, 0); */
	while (omap3_i2c_read_status()
	    || (reg_read(omap_i2c_bus->regs->I2C_CON) & (1 << MST))) {
		micro_delay(1000);
		omap3_i2c_write_status(0xffff);
		micro_delay(1000);
		if (--retry == 0) {
			break;
		}
	}

	return 0;
}

static int
omap3_i2c_write(i2c_addr_t addr, const uint8_t * buf, size_t buflen, int flags)
{
	uint16_t con, stat;
	int err, i, retry;

	err = omap3_i2c_wait(1 << BB, 0);
	if (err) {
		log_warn(&log, "Bus Busy\n");
		return err;
	}

	/* Set control register */
	con |= (1 << I2C_EN);	/* enabled */
	con |= (1 << MST);	/* master mode */
	con |= (1 << STT);	/* start condition */

	if (flags & I2C_F_STOP)
		con |= (1 << STP);	/* stop condition */

	con |= (1 << TRX);	/* TRX mode */

	/* Set address of slave device */
	addr &= ADDRESS_MASK;	/* sanitize address (10-bit max) */
	if (addr & ~0x7f) {
		/* 10-bit extended address in use, need to set XSA */
		con |= (1 << XSA);
	}

	reg_write(omap_i2c_bus->regs->I2C_SA, addr);
	reg_write(omap_i2c_bus->regs->I2C_CNT, buflen);
	reg_write(omap_i2c_bus->regs->I2C_CON, con);

	for (i = 0; i < buflen; i++) {
		/* Ready to write? */
		stat = omap3_i2c_stat();

		if ((stat & (1 << XRDY)) == 0) {
			log_warn(&log, "No XRDY Flag: stat = %x\n", stat);
			return EBUSY;
		}

		reg_write(omap_i2c_bus->regs->I2C_DATA, buf[i]);

		/* clear the write ready flag */
		omap3_i2c_write_status((1 << XRDY));
	}

	if (omap3_i2c_read_status() & (1 << NACK)) {
		log_warn(&log, "NACK\n");
		return EIO;
	}

	retry = 1000;
	/* reg_write(omap_i2c_bus->regs->I2C_CON, 0); */
	while (omap3_i2c_read_status()
	    || (reg_read(omap_i2c_bus->regs->I2C_CON) & (1 << MST))) {
		micro_delay(1000);
		omap3_i2c_write_status(0xffff);
		micro_delay(1000);
		if (--retry == 0) {
			break;
		}
	}

	return 0;
}

static int
omap3_i2c_wait(uint16_t mask, uint16_t val)
{
	int retry = 10;
	uint16_t v;

	omap3_i2c_write_status(0xffff);

	while (((v = omap3_i2c_read_status()) & mask) != val) {
		omap3_i2c_write_status(v);
		--retry;
		if (retry == 0) {
			log_warn(&log,
			    "wait timeout, mask = %x val = %x stat = %x\n",
			    mask, val, v);
			return EBUSY;
		}
		micro_delay(50000);
	}

	omap3_i2c_write_status(0xffff);

	return 0;
}

static int
omap3_i2c_stat(void)
{
	uint16_t v;
	int retry = 10;

	while (--retry > 0) {
		v = omap3_i2c_read_status();
		if ((v & ((1 << ROVR) | (1 << XUDF) | (1 << XRDY) | (1 << RRDY)
			    | (1 << ARDY) | (1 << NACK) | (1 << AL))) != 0) {
			break;
		}
		micro_delay(1000);
	}

	return v;
}

/*
 * Drain the incoming FIFO.
 *
 * Usually called to clear any garbage that may be in the buffer before
 * doing a read.
 */
static int
omap3_i2c_flush(void)
{
	int retry = 1000;
	uint16_t v;

	/* while bytes available for reading */
	while ((v = omap3_i2c_read_status()) & (1 << RRDY)) {

		if (--retry == 0) {
			log_warn(&log, "flush timeout, stat = %x\n", v);
			return EBUSY;
		}

		/* consume the byte and throw it away */
		(void) reg_read(omap_i2c_bus->regs->I2C_DATA);

		/* clear the read ready flag */
		omap3_i2c_write_status(1 << RRDY);
		micro_delay(1000);
	}

	omap3_i2c_write_status(0xffff);
	reg_write(omap_i2c_bus->regs->I2C_CNT, 0);

	return 0;
}

static void
omap3_i2c_clkconf(int i2c_bus_id)
{
	clkconf_init();

	if (omap_i2c_bus->bus_type == dm37xx) {

		clkconf_set(CM_ICLKEN1_CORE, BIT((15 + i2c_bus_id)),
		    0xffffffff);
		clkconf_set(CM_FCLKEN1_CORE, BIT((15 + i2c_bus_id)),
		    0xffffffff);

	} else if (omap_i2c_bus->bus_type == am335x) {

		switch (i2c_bus_id) {
		case 0:
			clkconf_set(CM_WKUP_I2C0_CLKCTRL, BIT(1), 0xffffffff);
			break;
		case 1:
			clkconf_set(CM_PER_I2C1_CLKCTRL, BIT(1), 0xffffffff);
			break;
		case 2:
			clkconf_set(CM_PER_I2C2_CLKCTRL, BIT(1), 0xffffffff);
			break;
		default:
			log_warn(&log, "Invalid i2c_bus_id\n");
			break;
		}
	}

	clkconf_release();
}

static void
omap3_i2c_padconf(int i2c_bus_id)
{
	int pinopts;

	padconf_init();

	if (omap_i2c_bus->bus_type == am335x) {

		/* use the options suggested in starterware driver */
		pinopts =
		    CONTROL_CONF_SLEWCTRL | CONTROL_CONF_RXACTIVE |
		    CONTROL_CONF_PUTYPESEL;

		switch (i2c_bus_id) {
		case 0:
			pinopts |= CONTROL_CONF_MUXMODE(0);
			padconf_set(CONTROL_CONF_I2C0_SDA, 0xffffffff,
			    pinopts);
			padconf_set(CONTROL_CONF_I2C0_SCL, 0xffffffff,
			    pinopts);
			break;

		case 1:
			pinopts |= CONTROL_CONF_MUXMODE(2);
			padconf_set(CONTROL_CONF_SPI0_CS0, 0xffffffff,
			    pinopts);
			padconf_set(CONTROL_CONF_SPI0_D1, 0xffffffff, pinopts);
			break;

		case 2:
			pinopts |= CONTROL_CONF_MUXMODE(3);
			padconf_set(CONTROL_CONF_UART1_CTSN,
			    0xffffffff, pinopts);
			padconf_set(CONTROL_CONF_UART1_RTSN,
			    0xffffffff, pinopts);
			break;

		default:
			log_warn(&log, "Invalid i2c_bus_id\n");
			break;
		}
	} else if (omap_i2c_bus->bus_type == dm37xx) {

		pinopts = PADCONF_PULL_MODE_PU_DIS;

		pinopts |= PADCONF_MUXMODE(0);	/* all have mmode 0 */

		switch (i2c_bus_id) {

		case 0:
			padconf_set(CONTROL_PADCONF_HSUSB0_DATA7,
			    0xffff0000, (pinopts << 16));
			padconf_set(CONTROL_PADCONF_I2C1_SDA,
			    0x0000ffff, pinopts);
			break;

		case 1:
			padconf_set(CONTROL_PADCONF_I2C1_SDA,
			    0xffff0000, (pinopts << 16));
			padconf_set(CONTROL_PADCONF_I2C2_SDA,
			    0x0000ffff, pinopts);
			break;

		case 2:
			padconf_set(CONTROL_PADCONF_I2C2_SDA,
			    0xffff0000, (pinopts << 16));
			padconf_set(CONTROL_PADCONF_I2C3_SDA,
			    0x0000ffff, pinopts);
			break;

		default:
			log_warn(&log, "Invalid i2c_bus_id\n");
			break;

		}

	}

	padconf_release();
}

static void
omap3_i2c_irq_enable(void)
{
	static int enabled = 0;

	if (!enabled) {
		if (sys_irqsetpolicy(omap_i2c_bus->irq, 0,
			&omap_i2c_bus->irq_hook_kernel_id) != OK) {
			log_warn(&log, "Couldn't set irq policy\n");
		} else {
			if (sys_irqenable(&omap_i2c_bus->irq_hook_kernel_id) !=
			    OK) {
				log_warn(&log,
				    "Couldn't enable irq %d (hooked)\n",
				    omap_i2c_bus->irq);
			}

		}
		enabled = 1;
	}

}

static uint16_t
omap3_i2c_read_status(void)
{
	uint16_t status = 0x0000;

	if (omap_i2c_bus->bus_type == am335x) {
		/* TRM says to use RAW for polling for events */
		status = reg_read(omap_i2c_bus->regs->I2C_IRQSTATUS_RAW);
	} else if (omap_i2c_bus->bus_type == dm37xx) {
		status = reg_read(omap_i2c_bus->regs->I2C_STAT);
	} else {
		log_warn(&log, "Don't know how to read i2c bus status.\n");
	}

	return status;
}

static void
omap3_i2c_write_status(uint16_t mask)
{
	if (omap_i2c_bus->bus_type == am335x) {
		/* write 1's to IRQSTATUS (not RAW) to clear the bits */
		reg_write(omap_i2c_bus->regs->I2C_IRQSTATUS, mask);
	} else if (omap_i2c_bus->bus_type == dm37xx) {
		reg_write(omap_i2c_bus->regs->I2C_STAT, mask);
	} else {
		log_warn(&log, "Don't know how to clear i2c bus status.\n");
	}
}

/* Quick and Dirty Test Function - TODO move into separate driver later */

/* address of BBB's on-board eeprom (note: no on-board eeprom for the xM). */
#define EEPROM_ADDR (0x50)

static void eeprom_dump(void);

struct eeprom_layout
{
	u32_t magic_number;	/* Should be 0xEE3355AA (on am335x at least) */
	char board_name[8];	/* Warning: strings not NULL terminated */
	char version[4];
	char serial_number[12];
	char config[32];
	char mac_addrs[3][6];
};

/* interrogate the on board EEPROM chip for info about the board */
static void
eeprom_dump(void)
{
	int r;
	uint8_t memaddr[2] = { '\0', '\0' };	/* start reading at 0 */
	char board_name[9];
	struct eeprom_layout eeprom_contents;

	r = omap3_i2c_write(EEPROM_ADDR, memaddr, sizeof(memaddr), I2C_F_STOP);
	if (r != 0) {
		log_warn(&log, "eeprom: omap3_i2c_write failed (r=%d)\n", r);
		return;
	}

	r = omap3_i2c_read(EEPROM_ADDR, (uint8_t *) & eeprom_contents, 32,
	    I2C_F_STOP);
	if (r == 0) {
		memset(board_name, '\0', 9);
		memcpy(board_name, eeprom_contents.board_name, 8);

		log_info(&log, "eeprom: magic_number=%x board_name='%s'\n",
		    eeprom_contents.magic_number, board_name);
	} else {
		log_warn(&log, "eeprom: omap3_i2c_read failed (r=%d)\n", r);
	}
}

/* End of EEPROM Test Code */

int
omap3_interface_setup(int (**process) (minix_i2c_ioctl_exec_t * ioctl_exec),
    int i2c_bus_id)
{
	int i2c_rev, major, minor;
	struct minix_mem_range mr;

	/* Fill in the function pointer */

	*process = omap3_i2c_exec;

	/* Select the correct i2c definition for this SoC */

#if defined(AM335X)
	omap_i2c_buses = am335x_i2c_buses;
	omap_i2c_nbuses = AM335X_OMAP_NBUSES;
#elif defined(DM37XX)
	omap_i2c_buses = dm37xx_i2c_buses;
	omap_i2c_nbuses = DM37XX_OMAP_NBUSES;
#else
#error				/* Unsupported SoC */
#endif

	if (i2c_bus_id < 0 || i2c_bus_id >= omap_i2c_nbuses) {
		return EINVAL;
	}

	/* select the bus to operate on */
	omap_i2c_bus = &omap_i2c_buses[i2c_bus_id];

	/*
	 * Map I2C Registers
	 */

	/* Configure memory access */
	mr.mr_base = omap_i2c_bus->mr_base;	/* start addr */
	mr.mr_limit = mr.mr_base + omap_i2c_bus->mr_size;	/* end addr */

	/* ask for privileges to access the I2C memory range */
	if (sys_privctl(SELF, SYS_PRIV_ADD_MEM, &mr) != OK) {
		panic("Unable to obtain i2c memory range privileges");
	}

	/* map the memory into this process */
	omap_i2c_bus->mapped_addr = (vir_bytes) vm_map_phys(SELF,
	    (void *) omap_i2c_bus->mr_base, omap_i2c_bus->mr_size);

	if (omap_i2c_bus->mapped_addr == (vir_bytes) MAP_FAILED) {
		panic("Unable to map i2c registers");
	}

	/* Enable Clocks */
	omap3_i2c_clkconf(i2c_bus_id);

	/* Configure Pins */
	omap3_i2c_padconf(i2c_bus_id);

	/* Enable IRQs */
	omap3_i2c_irq_enable();

	/* Bring up I2C module */
	omap3_i2c_reset();

	/* Get I2C Revision */
	if (omap_i2c_bus->bus_type == am335x) {
		/* I2C_REVLO revision: major (bits 10-8), minor (bits 5-0) */
		i2c_rev = reg_read(omap_i2c_bus->regs->I2C_REVNB_LO);
		major = (i2c_rev >> 8) & 0x07;
		minor = i2c_rev & 0x3f;
	} else if (omap_i2c_bus->bus_type == dm37xx) {
		/* I2C_REV revision: major (bits 7-4), minor (bits 3-0) */
		i2c_rev = reg_read(omap_i2c_bus->regs->I2C_REV);
		major = (i2c_rev >> 4) & 0x0f;
		minor = i2c_rev & 0x0f;
	} else {
		panic("Don't know how to read i2c revision.");
	}

	/* display i2c revision information */
	log_info(&log, "i2c_%d: I2C rev %d.%d\n", (i2c_bus_id + 1),
	    major, minor);

	/* TODO remove me, just here for intial testing */
	if (i2c_bus_id == 0 && omap_i2c_bus->bus_type == am335x) {
		eeprom_dump();
	}
	/* else dm37xx doesn't have an eeprom onboard */
	return OK;
}
