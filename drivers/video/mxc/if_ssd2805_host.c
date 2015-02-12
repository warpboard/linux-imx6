/*
 * SSD2805 MIPI controller driver
 *
 * Copyright (C) 2014 Revolution Robotics, Inc.
 * Adapted from the Si14 SpA SSD2805 MIPI controller driver
 *
 * Copyright (C) 2011 Si14 SpA
 * Author: Luca Burelli <luca.burelli@si14.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

//#define DEBUG
//#define DEBUG_ELCDIF

/* Enable GPIO readback code.
 * WARNING: this messes with pin configuration and must be used only while
 * the video data transfer is disabled.
 */
//#define LCDIF_GPIO_READ

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <mach/gpio.h>
#include <mach/clock.h>
#include <mach/system.h>
#include <asm/io.h>

#include "if_ssd2805.h"
#include "if_ssd2805_lowlevel.h"
#include "ssd2805.h"

#include "elcdif_regs.h"
#include "../../../arch/arm/mach-mx6/board-mx6sl_warp.h"

#define REGS_LCDIF_BASE IO_ADDRESS(ELCDIF_PHY_BASE_ADDR)

#ifdef LCDIF_GPIO_READ
static void set_lcd_pads(void){
	mxc_iomux_v3_setup_multiple_pads(mcu8080display_pads,ARRAY_SIZE(mcu8080display_pads));
	gpio_direction_input(PINID_MIPI_TE); 		// MIPI_TE
	gpio_direction_output(PINID_LCD_RSTN, 1);  	// LCD_RSTn
	gpio_direction_input(PINID_LCD_INTN); 		// LCD_INTn
	gpio_direction_input(PINID_MIPI_BSYNC);  	// MIPI_B_SYNC
	gpio_direction_output(PINID_MIPI_RSTN, 1); 	// MIPI_RSTn
	gpio_direction_output(PINID_LCD_RD, 1); 	// LCD_RDX
	gpio_direction_output(PINID_LCD_RS, 1); 	// LCD_DCX(ssd)/LCD_RS(mx6)
}

static void set_lcd_dat_gpio_out(void){
	gpio_direction_output(PINID_LCD_DAT0,1); 	// LCD_DAT0
	gpio_direction_output(PINID_LCD_DAT1,1); 	// LCD_DAT1
	gpio_direction_output(PINID_LCD_DAT2,1); 	// LCD_DAT2
	gpio_direction_output(PINID_LCD_DAT3,1); 	// LCD_DAT3
	gpio_direction_output(PINID_LCD_DAT4,1); 	// LCD_DAT4
	gpio_direction_output(PINID_LCD_DAT5,1); 	// LCD_DAT5
	gpio_direction_output(PINID_LCD_DAT6,1); 	// LCD_DAT6
	gpio_direction_output(PINID_LCD_DAT7,1); 	// LCD_DAT7
#if (LCDIF_BUS_WIDTH == 16)
	gpio_direction_output(PINID_LCD_DAT8,1); 	// LCD_DAT8
	gpio_direction_output(PINID_LCD_DAT9,1); 	// LCD_DAT9
	gpio_direction_output(PINID_LCD_DAT10,1); 	// LCD_DAT10
	gpio_direction_output(PINID_LCD_DAT11,1); 	// LCD_DAT11
	gpio_direction_output(PINID_LCD_DAT12,1); 	// LCD_DAT12
	gpio_direction_output(PINID_LCD_DAT13,1); 	// LCD_DAT13
	gpio_direction_output(PINID_LCD_DAT14,1); 	// LCD_DAT14
	gpio_direction_output(PINID_LCD_DAT15,1); 	// LCD_DAT15
#endif
	gpio_direction_output(PINID_LCD_CS,1); 		// LCD_CS
	gpio_direction_output(PINID_LCD_RS,1); 		// LCD_RS
	gpio_direction_output(PINID_LCD_WR,1); 		// LCD_WR
	gpio_direction_output(PINID_LCD_RD,1); 		// LCD_RD
}

static void set_lcd_dat_gpio_in(void){
	gpio_direction_input(PINID_LCD_DAT0); 	// LCD_DAT0
	gpio_direction_input(PINID_LCD_DAT1); 	// LCD_DAT1
	gpio_direction_input(PINID_LCD_DAT2); 	// LCD_DAT2
	gpio_direction_input(PINID_LCD_DAT3); 	// LCD_DAT3
	gpio_direction_input(PINID_LCD_DAT4); 	// LCD_DAT4
	gpio_direction_input(PINID_LCD_DAT5); 	// LCD_DAT5
	gpio_direction_input(PINID_LCD_DAT6); 	// LCD_DAT6
	gpio_direction_input(PINID_LCD_DAT7); 	// LCD_DAT7
#if (LCDIF_BUS_WIDTH == 16)
	gpio_direction_input(PINID_LCD_DAT8); 	// LCD_DAT8
	gpio_direction_input(PINID_LCD_DAT9); 	// LCD_DAT9
	gpio_direction_input(PINID_LCD_DAT10); 	// LCD_DAT10
	gpio_direction_input(PINID_LCD_DAT11); 	// LCD_DAT11
	gpio_direction_input(PINID_LCD_DAT12); 	// LCD_DAT12
	gpio_direction_input(PINID_LCD_DAT13); 	// LCD_DAT13
	gpio_direction_input(PINID_LCD_DAT14); 	// LCD_DAT14
	gpio_direction_input(PINID_LCD_DAT15); 	// LCD_DAT15
#endif
}

static void set_lcd_pads_gpio(void){
	mxc_iomux_v3_setup_multiple_pads(mcu8080display_gpio_pads,ARRAY_SIZE(mcu8080display_gpio_pads));

	gpio_direction_input(PINID_LCD_DAT0); 	// LCD_DAT0
	gpio_direction_input(PINID_LCD_DAT1); 	// LCD_DAT1
	gpio_direction_input(PINID_LCD_DAT2); 	// LCD_DAT2
	gpio_direction_input(PINID_LCD_DAT3); 	// LCD_DAT3
	gpio_direction_input(PINID_LCD_DAT4); 	// LCD_DAT4
	gpio_direction_input(PINID_LCD_DAT5); 	// LCD_DAT5
	gpio_direction_input(PINID_LCD_DAT6); 	// LCD_DAT6
	gpio_direction_input(PINID_LCD_DAT7); 	// LCD_DAT7
	gpio_direction_input(PINID_LCD_DAT8); 	// LCD_DAT8
	gpio_direction_input(PINID_LCD_DAT9); 	// LCD_DAT9
	gpio_direction_input(PINID_LCD_DAT10); 	// LCD_DAT10
	gpio_direction_input(PINID_LCD_DAT11); 	// LCD_DAT11
	gpio_direction_input(PINID_LCD_DAT12); 	// LCD_DAT12
	gpio_direction_input(PINID_LCD_DAT13); 	// LCD_DAT13
	gpio_direction_input(PINID_LCD_DAT14); 	// LCD_DAT14
	gpio_direction_input(PINID_LCD_DAT15); 	// LCD_DAT15

	gpio_direction_input(PINID_MIPI_TE); 		// MIPI_TE
	gpio_direction_output(PINID_LCD_RSTN, 1);  	// LCD_RSTn
	gpio_direction_input(PINID_LCD_INTN); 		// LCD_INTn
	gpio_direction_output(PINID_MIPI_RSTN	, 1); 	// MIPI_RSTn
	gpio_direction_output(PINID_LCD_RD, 1); 	// LCD_RDX
	gpio_direction_output(PINID_LCD_RS, 1); 	// LCD_DCX
}
#endif // LCDIF_GPIO_READ

#ifdef DEBUG_ELCDIF
void dump(void)
{
	static struct {
		char *name;
		int ofs;
		int last_val;
	} REGS[] = {
#define _entry(x) { #x, HW_ELCDIF_ ## x, -1 }
		_entry(CTRL),
		_entry(CTRL1),
		_entry(TRANSFER_COUNT),
		_entry(CUR_BUF),
		_entry(NEXT_BUF),
		_entry(TIMING),
		_entry(DATA),
		_entry(BM_ERROR_STAT),
		_entry(STAT),
		_entry(VERSION),
		_entry(DEBUG0),
		_entry(DEBUG1),
		{ 0, 0 }
	};
	int i = -1;
	while (REGS[++i].name) {
		int val = __raw_readl(REGS_LCDIF_BASE + REGS[i].ofs);
		if (val != REGS[i].last_val) {
			printk(KERN_INFO "  dump: [%18s] = %08x\tlast %08x bitmask %08x\n", REGS[i].name, val, REGS[i].last_val, val ^ REGS[i].last_val);
			REGS[i].last_val = val;
		}
	}
}
#else
void dump(void) { }
#endif

static void lcdif_write(void *ptr, unsigned len, int is_data)
{
	uint8_t *buf = ptr;

#ifdef DEBUG
	dump_stack();
#endif
	if (instance->pll_configured)
	{
		__raw_writel(BF_ELCDIF_TIMING_CMD_HOLD(LCDIF_FAST_FREQDIV) |
			BF_ELCDIF_TIMING_CMD_SETUP(LCDIF_FAST_FREQDIV) |
			BF_ELCDIF_TIMING_DATA_HOLD(LCDIF_FAST_FREQDIV) |
			BF_ELCDIF_TIMING_DATA_SETUP(LCDIF_FAST_FREQDIV),
			elcdif_base + HW_ELCDIF_TIMING);
	}else
	{
		__raw_writel(BF_ELCDIF_TIMING_CMD_HOLD(LCDIF_SLOW_FREQDIV) |
			BF_ELCDIF_TIMING_CMD_SETUP(LCDIF_SLOW_FREQDIV) |
			BF_ELCDIF_TIMING_DATA_HOLD(LCDIF_SLOW_FREQDIV) |
			BF_ELCDIF_TIMING_DATA_SETUP(LCDIF_SLOW_FREQDIV),
			elcdif_base + HW_ELCDIF_TIMING);
	}

	pr_debug( " write: wait for ready\n");
	dump();

	while (__raw_readl(elcdif_base + HW_ELCDIF_CTRL) & BM_ELCDIF_CTRL_RUN);


	__raw_writel(BM_ELCDIF_CTRL_ELCDIF_MASTER,
		     elcdif_base + HW_ELCDIF_CTRL_CLR);

	gpio_direction_output(PINID_LCD_RS, is_data?1:0);

	while (len > 0)
	{
		unsigned burst, count;
#if (LCDIF_BUS_WIDTH == 8)
		count = burst = min(len, 0xffffu);
#elif (LCDIF_BUS_WIDTH == 16)
		burst = min(len, 0x1fffeu);
		count = (burst+1) / 2;
#endif
		len -= burst;

		__raw_writel(BF_ELCDIF_TRANSFER_COUNT_V_COUNT(1) | BF_ELCDIF_TRANSFER_COUNT_H_COUNT(1),
			     elcdif_base + HW_ELCDIF_TRANSFER_COUNT);

		do {
			/* make sure the LFIFO is not full */
			while (__raw_readl(elcdif_base + HW_ELCDIF_STAT) & BM_ELCDIF_STAT_LFIFO_FULL);
			while (__raw_readl(elcdif_base + HW_ELCDIF_CTRL) & BM_ELCDIF_CTRL_RUN);
			__raw_writel(BM_ELCDIF_CTRL_RUN, elcdif_base + HW_ELCDIF_CTRL_SET);
#if (LCDIF_BUS_WIDTH == 8)
			/* write 8 bits */
			__raw_writeb(*(uint8_t*)buf, elcdif_base + HW_ELCDIF_DATA);
			buf += 1;
#elif (LCDIF_BUS_WIDTH == 16)
			/* write 16 bits anyway, MIPI hardware will discard higher 8 */
			__raw_writew(*(uint16_t*)buf, elcdif_base + HW_ELCDIF_DATA);
			buf += 2;
#endif
		} while(--count > 0);

		pr_debug( " write: wait for done\n");
		dump();

		while (__raw_readl(elcdif_base + HW_ELCDIF_CTRL) & BM_ELCDIF_CTRL_RUN);

		pr_debug( " DONE!\n");
		dump();
	}
}

#ifdef LCDIF_GPIO_READ
static int lcdif_read(u16 reg, u16 *rbuf, int rlen)
{
#warning __ GPIO READ ACTIVE - FOR DEBUG USE ONLY __

	unsigned int currGPIO = 0;
	instance->gpio_read_sem = 1;

	void __iomem *gpio3_base;
	gpio3_base = ioremap(GPIO3_BASE_ADDR, SZ_4K); // Bad practice, don't do this.

	void __iomem *gpio2_base;
	gpio2_base = ioremap(GPIO2_BASE_ADDR, SZ_4K); // Bad practice, don't do this.

	while(__raw_readl(elcdif_base + HW_ELCDIF_CTRL) & BM_ELCDIF_CTRL_RUN);
	// set pins as GPIO outputs, all ones
	set_lcd_pads_gpio();
	set_lcd_dat_gpio_out();

	// write command cycle
	udelay(1000);
	gpio_direction_output(PINID_LCD_RS, 0);
	udelay(1000);
	gpio_direction_output(PINID_LCD_CS, 0);
	udelay(1000);
#if (LCDIF_BUS_WIDTH == 8)
	currGPIO =  __raw_readl(gpio2_base);
	currGPIO = (reg << 20) | (currGPIO & 0xf00fffff); 	// LCD_DAT[11:0] go to GPIO2[31:20]
	__raw_writel(currGPIO, gpio2_base);
#elif (LCDIF_BUS_WIDTH == 16)
	currGPIO =  __raw_readl(gpio3_base);
	currGPIO = (reg >> 12) | (currGPIO & 0xfffffff0); 	// LCD_DAT[15:12] go to GPIO3[3:0]
	__raw_writel(currGPIO, gpio3_base);

	currGPIO =  __raw_readl(gpio2_base);
	currGPIO = (reg << 20) | (currGPIO & 0x000fffff); 	// LCD_DAT[11:0] go to GPIO2[31:20]
	__raw_writel(currGPIO, gpio2_base);
#endif
	udelay(1000);
	gpio_direction_output(PINID_LCD_WR, 0);
	udelay(1000);
	gpio_direction_output(PINID_LCD_WR, 1);
	udelay(1000);

	// switch data bus to read mode
	set_lcd_dat_gpio_in();

	// read data cycle(s)
	gpio_direction_output(PINID_LCD_RS, 1);

	mdelay(1);
	while (rlen > 0) {
		gpio_direction_output(PINID_LCD_RD, 0);
		udelay(1000);
#if (LCDIF_BUS_WIDTH == 8)
		currGPIO =  __raw_readl(gpio2_base);
		reg = ((currGPIO >> 20) & 0x000000ff); 	// LCD_DAT[11:0] come from GPIO2[31:20]

		gpio_direction_output(PINID_LCD_RD, 1);
		udelay(1000);
		gpio_direction_output(PINID_LCD_RD, 0);
		udelay(1000);

		currGPIO =  __raw_readl(gpio2_base);
		reg |= ((currGPIO >> 12) & 0x0000ff00); 	// LCD_DAT[11:0] come from GPIO2[31:20]
#elif (LCDIF_BUS_WIDTH == 16)
		currGPIO =  __raw_readl(gpio3_base);
		reg = ((currGPIO << 12) & 0x0000f000); 	// LCD_DAT[15:12] go to GPIO3[3:0]

		currGPIO =  __raw_readl(gpio2_base);
		reg |= ((currGPIO >> 20) & 0x00000fff); 	// LCD_DAT[11:0] come from GPIO2[31:20]
#endif
		gpio_direction_output(PINID_LCD_RD, 1);
		udelay(1000);
		rlen -= 2;
		*rbuf++ = (u16) reg;
	}

	// set pins as everyone expects, also releases CS and RS
	set_lcd_dat_gpio_out();

	// set pads for LCD MCU 8080 interface
	set_lcd_pads();

	instance->gpio_read_sem = 0;

	iounmap(gpio2_base); // Bad practice, don't do this.
	iounmap(gpio3_base); // Bad practice, don't do this.

	return 0;
}
#else /* LCDIF_GPIO_READ */
static int lcdif_read(u16 reg, u16 *rbuf, int rlen)
{
	/* not implemented */
	return -EIO;
}
#endif /* LCDIF_GPIO_READ */

int ssd2805_command(int cmd)
{
	pr_debug("%s(cmd %04x)\n", __func__, cmd);

	lcdif_write(&cmd, 1, false);

	return 0;
}

int ssd2805_write_reg(int reg, int value)
{
	pr_debug("%s(reg %04x, value %04x)\n", __func__, reg, value);

	lcdif_write(&reg, 1, false);
	lcdif_write(&value, 2, true);
	instance->reg_map[reg] = value;

	return 0;
}

int ssd2805_read_reg(int reg)
{
	u16 value;
	int ret;
	ret = lcdif_read((u16) reg, &value, 2);

	if (ret < 0)
		return ret;

	instance->reg_map[reg] = value;
	return value;
}

int mipi_dcs_command(int cmd)
{
	pr_debug("%s(cmd %04x)\n", __func__, cmd);

	if (!instance)	return -ENODEV;

	/* TDC is OUTGOING DATA payload excluding the command */
	ssd2805_ensure_reg(SSD2805_REG_PSCR1, SSD2805_PSCR1_TDCL(0));
	ssd2805_ensure_reg(SSD2805_REG_PSCR2, SSD2805_PSCR2_TDCH(0));

	return ssd2805_command(cmd);
}

int mipi_dcs_write(int cmd, u8 *wbuf, int wlen)
{
	u16 value;

	pr_debug("%s(cmd %04x, len %04x)\n", __func__, cmd, wlen);

	if (!instance)	return -ENODEV;

	ssd2805_ensure_reg(SSD2805_REG_PSCR1, SSD2805_PSCR1_TDCL(wlen & 0xffff));
	ssd2805_ensure_reg(SSD2805_REG_PSCR2, SSD2805_PSCR2_TDCH(wlen >> 16));

	/* make sure REN is not set */
	value = ssd2805_query_reg(SSD2805_REG_CFGR);
	if (value & SSD2805_CFGR_REN)
		ssd2805_write_reg(SSD2805_REG_CFGR, value & ~SSD2805_CFGR_REN);

	/* send command */
	lcdif_write(&cmd, 1, false);

	/* send data */
	lcdif_write(wbuf, wlen, true);

	return 0;
}

int mipi_dcs_read(int cmd, u8 *rbuf, int rlen)
{
	return lcdif_read((u16)cmd, (u16*) rbuf, rlen);
}
