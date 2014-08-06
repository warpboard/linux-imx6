//#define DEBUG_SSD2805C
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
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <mach/gpio.h>
#include <mach/system.h>
#include <asm/io.h>

#include "if_ssd2805.h"
#include "if_ssd2805_lowlevel.h"
#include "ssd2805.h"

#define MIPI_CLK_FREQ		12000000

struct ssd2805_device *instance;

#ifdef DEBUG_SSD2805C
void __maybe_unused ssd2805_dump_registers(void)
{
#define SSD2805_DUMP_REG(reg, desc)	printk(KERN_INFO "%7s [%02x]: %04x | %s\n", #reg, SSD2805_REG_ ## reg, ssd2805_read_reg(SSD2805_REG_ ## reg), desc)
	SSD2805_DUMP_REG(DIR,	"Device Identification Register");
	SSD2805_DUMP_REG(VICR1,	"RGB Interface Control Register 1");
	SSD2805_DUMP_REG(VICR2,	"RGB Interface Control Register 2");
	SSD2805_DUMP_REG(VICR3,	"RGB Interface Control Register 3");
	SSD2805_DUMP_REG(VICR4,	"RGB Interface Control Register 4");
	SSD2805_DUMP_REG(VICR5,	"RGB Interface Control Register 5");
	SSD2805_DUMP_REG(VICR6,	"RGB Interface Control Register 6");
	SSD2805_DUMP_REG(CFGR,	"Configuration Register");
	SSD2805_DUMP_REG(VCR,	"VC Control Register");
	SSD2805_DUMP_REG(PCR,	"PLL Control Register");
	SSD2805_DUMP_REG(PLCR,	"PLL Configuration Register");
	SSD2805_DUMP_REG(CCR,	"Clock Control Register");
	SSD2805_DUMP_REG(PSCR1,	"Packet Size Control Register 1");
	SSD2805_DUMP_REG(PSCR2,	"Packet Size Control Register 2");
	SSD2805_DUMP_REG(PSCR3,	"Packet Size Control Register 3");
	SSD2805_DUMP_REG(GPDR,	"Generic Packet Drop Register");
	SSD2805_DUMP_REG(OCR,	"Operation Control Register");
	SSD2805_DUMP_REG(MRSR,	"Maximum Return Size Register");
	SSD2805_DUMP_REG(RDCR,	"Return Data Count Register");
	SSD2805_DUMP_REG(ARSR,	"ACK Response Register");
	SSD2805_DUMP_REG(LCR,	"Line Control Register");
	SSD2805_DUMP_REG(ICR,	"Interrupt Control Register");
	SSD2805_DUMP_REG(ISR,	"Interrupt Status Register");
	SSD2805_DUMP_REG(ESR,	"Error Status Register");
	SSD2805_DUMP_REG(DAR1,	"Delay Adjustment Register 1");
	SSD2805_DUMP_REG(DAR2,	"Delay Adjustment Register 2");
	SSD2805_DUMP_REG(DAR3,	"Delay Adjustment Register 3");
	SSD2805_DUMP_REG(DAR4,	"Delay Adjustment Register 4");
	SSD2805_DUMP_REG(DAR5,	"Delay Adjustment Register 5");
	SSD2805_DUMP_REG(DAR6,	"Delay Adjustment Register 6");
	SSD2805_DUMP_REG(HTTR1,	"HS TX Timer Register 1");
	SSD2805_DUMP_REG(HTTR2,	"HS TX Timer Register 2");
	SSD2805_DUMP_REG(LRTR1,	"LP RX Timer Register 1");
	SSD2805_DUMP_REG(LRTR2,	"LP RX Timer Register 2");
	SSD2805_DUMP_REG(TSR,	"TE Status Register");
	SSD2805_DUMP_REG(LRR,	"SPI Read Register");
	SSD2805_DUMP_REG(TR,	"Test Register");
	SSD2805_DUMP_REG(RR,	"Read Register");
#undef	SSD2805_DUMP_REG
}
#else
void ssd2805_dump_registers(void) { }
#endif

static int ssd2805_setup_pll(int pll_freq)
{
	/* fPRE = fREF / N, fPRE > 5MHz
	 * fVCO = fPRE * M, 500 > fVCO > 225MHz
	 * fOUT = fVCO / P
	 */
	int N, M, P;
	int N_best, M_best, P_best, error;

	ssd2805_dump_registers();

	/* disable PLL */
	instance->pll_configured = 0;
	ssd2805_ensure_reg(SSD2805_REG_PCR, 0);

	if (!pll_freq) return 0;

	error = pll_freq;
	N_best = M_best = P_best = -1;

	for (N=1; N<=16; ++N) {
		int M_min, M_max;
		int fPRE = MIPI_CLK_FREQ/N;
		if (fPRE < 5000000) continue;

		M_min=(225000000+fPRE-1)/fPRE;	M_min=max(2, min(256, M_min));
		M_max=(500000000)/fPRE;		M_max=max(2, min(256, M_max));
		for (M=M_min; M<M_max; ++M) {
			int fVCO = fPRE * M;
			int P_opt = fVCO/pll_freq;
			int P_min = max(P_opt-1, 1);
			int P_max = min(P_opt+1, 16);

			for (P=P_min; P<=P_max; ++P) {
				int fOUT = fVCO / P;
				if (abs(fOUT-pll_freq) < error) {
					error = abs(fOUT-pll_freq);
					N_best = N;
					M_best = M;
					P_best = P;
					if (error == 0) goto found;
				}
			}
		}
	}

	if (N_best < 0)
		return -EINVAL;

found:
	pr_info("Setting up PLL for %iM->%iM with error %ik (N=%i, M=%i, P=%i)\n", MIPI_CLK_FREQ/1000000, pll_freq/1000000, error/1000, N_best, M_best, P_best);

	/* update values and enable PLL */
	ssd2805_write_reg(SSD2805_REG_PLCR, SSD2805_PLCR_DIV(N_best-1) | SSD2805_PLCR_MUL(M_best-1) | SSD2805_PLCR_PDIV(P_best-1));
	ssd2805_write_reg(SSD2805_REG_PCR, SSD2805_PCR_PEN);

	/*
	 * Wait for PLL_LOCK bit; this requires readback from MIPI
	 */
	error = 1000;
	do {
		P = ssd2805_read_reg(SSD2805_REG_ISR);
		if (P<0) {
			/* read status failed, assume OK after delay */
			mdelay(50);
			break;
		}
		--error;
	} while (!(P & SSD2805_ISR_PLS) && error);

	if (!(P & SSD2805_ISR_PLS)) {
		pr_err(SSD2805_MODULE_NAME ": failed to lock PLL at %i, sreg %04x", pll_freq, P);
		return -EIO;
	}
	instance->pll_configured = 1;
	return 0;
}

int ssd2805_setup_commands(int pll_freq)
{
	int r;

	if (!instance) return -ENODEV;
	if(!instance->pll_configured){
		r = ssd2805_setup_pll(pll_freq);
		if (r < 0) return r;
	}

	/* disable all virtual circuit IDs */
	ssd2805_ensure_reg(SSD2805_REG_CCR, 0x0004);
	ssd2805_ensure_reg(SSD2805_REG_VCR, 0);

	/* setup initial CFGR */
	ssd2805_ensure_reg(SSD2805_REG_CFGR, SSD2805_CFGR_EOT | SSD2805_CFGR_DCS | SSD2805_CFGR_HCLK);

	return 0;
}

int ssd2805_setup_host(void)
{
	if (!instance) return -ENODEV;

	ssd2805_dump_registers();

	/*
	 * Enable high speed (HS) communication
	 */
	ssd2805_ensure_reg(SSD2805_REG_CFGR, SSD2805_CFGR_EOT | SSD2805_CFGR_DCS | SSD2805_CFGR_CKE | SSD2805_CFGR_HS);

	/* Biggest possible (optimal) transfer size */
	ssd2805_ensure_reg(SSD2805_REG_PSCR3, 0x400);

	/* Set a low HS TX timer value to fix MIPI transmission problems.
	 * Without this, some packets are completely discarded, resulting in
	 * partial image updates. */
	ssd2805_ensure_reg(SSD2805_REG_HTTR1, 3000);
	ssd2805_ensure_reg(SSD2805_REG_HTTR2, 0);

	/* Use PO IRQ as BUSY indication. NOTE: this is unusable on MX23 due
	 * to signal polarity. */
	ssd2805_write_reg(SSD2805_REG_ICR, SSD2805_ICR_POE);

	// Use RGB, instead of BGR
	ssd2805_write_reg(SSD2805_REG_TR, 0x7);

	ssd2805_write_reg(SSD2805_REG_PSCR1, SSD2805_PSCR1_TDCL((240 * 240 * 3) & 0xffff));
	ssd2805_write_reg(SSD2805_REG_PSCR2, SSD2805_PSCR2_TDCH((240 * 240 * 3) >> 16));

	ssd2805_dump_registers();

	return 0;
}

void ssd2805_setup_fbxfer(int cmd, int wlen)
{
	ssd2805_ensure_reg(SSD2805_REG_PSCR1, SSD2805_PSCR1_TDCL(wlen & 0xffff));
	ssd2805_ensure_reg(SSD2805_REG_PSCR2, SSD2805_PSCR2_TDCH(wlen >> 16));
	ssd2805_command(cmd);
}


int ssd2805_init(void)
{
	instance = kzalloc(sizeof(*instance), GFP_KERNEL);
	if (instance == NULL) {
		pr_err(SSD2805_MODULE_NAME ": out of memory\n");
		return -ENOMEM;
	}
	memset(&instance->reg_map, 0xff, sizeof(instance->reg_map));

	return 0;
}
