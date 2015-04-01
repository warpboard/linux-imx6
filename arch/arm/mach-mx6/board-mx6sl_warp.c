/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Copyright (C) 2014 Kynetics, LLC
 * Adaptations for WaRP board: Nicola La Gloria
 *
 * Copyright (C) 2014 Revolution Robotics, Inc.
 * Adaptations for WaRP board: Jacob Postman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/max77696.h>
#include <sound/wm8962.h>
#include <sound/pcm.h>
#include <linux/power/sabresd_battery.h>
#include <linux/ion.h>
#include <linux/pwm.h>
#include <linux/pwm_clk.h>
#include <linux/wlan_plat.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6sl.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/imx_rfkill.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6sl_common.h"
#include "board-mx6sl_warp.h"


static int spdc_sel;
static void mx6sl_warp_suspend_enter(void);
static void mx6sl_warp_suspend_exit(void);

struct clk *extern_audio_root;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

/* Additional Defines for BRCM WLAN */
#define BRCM 1
#define GPIO_BCMDHD_PWR		WARP_WL_REG_ON
#define GPIO_BCMDHD_OOB		WARP_GPIO0_WL_HOSTWAKE

static struct ion_platform_data imx_ion_data = {
	.nr = 1,
	.heaps = {
		{
		.id = 0,
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "vpu_ion",
		.size = SZ_16M,
		.cacheable = 1,
		},
	},
};

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

static const struct pm_platform_data mx6sl_warp_pm_data __initconst = {
	.name		= "imx_pm",
	.suspend_enter = mx6sl_warp_suspend_enter,
	.suspend_exit = mx6sl_warp_suspend_exit,
};

static int plt_sd_pad_change(unsigned int index, int clock)
{
	/* LOW speed is the default state of SD pads */
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	iomux_v3_cfg_t *sd_pads_200mhz = NULL;
	iomux_v3_cfg_t *sd_pads_100mhz = NULL;
	iomux_v3_cfg_t *sd_pads_50mhz = NULL;

	u32 sd_pads_200mhz_cnt;
	u32 sd_pads_100mhz_cnt;
	u32 sd_pads_50mhz_cnt;

	switch (index) {
	case 0:
		sd_pads_200mhz = mx6sl_sd1_200mhz;
		sd_pads_100mhz = mx6sl_sd1_100mhz;
		sd_pads_50mhz = mx6sl_sd1_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6sl_sd1_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6sl_sd1_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6sl_sd1_50mhz);
		break;
	case 1:
		sd_pads_200mhz = mx6sl_sd2_200mhz;
		sd_pads_100mhz = mx6sl_sd2_100mhz;
		sd_pads_50mhz = mx6sl_sd2_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6sl_sd2_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6sl_sd2_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6sl_sd2_50mhz);
		break;
	case 2:
		sd_pads_200mhz = mx6sl_sd3_200mhz;
		sd_pads_100mhz = mx6sl_sd3_100mhz;
		sd_pads_50mhz = mx6sl_sd3_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6sl_sd3_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6sl_sd3_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6sl_sd3_50mhz);
		break;
	default:
		printk(KERN_ERR "no such SD host controller index %d\n", index);
		return -EINVAL;
	}

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;
		BUG_ON(!sd_pads_200mhz);
		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_200mhz,
							sd_pads_200mhz_cnt);
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;
		BUG_ON(!sd_pads_100mhz);
		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_100mhz,
							sd_pads_100mhz_cnt);
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;
		BUG_ON(!sd_pads_50mhz);
		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_50mhz,
							sd_pads_50mhz_cnt);
	}
}

static const struct esdhc_platform_data mx6_evk_sd1_data __initconst = {
	.cd_gpio		= MX6_BRD_SD1_CD,
	.wp_gpio		= MX6_BRD_SD1_WP,
	.support_8bit		= 1,
	.support_18v		= 1,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.platform_pad_change = plt_sd_pad_change,
};

static const struct esdhc_platform_data mx6_warp_sd2_data __initconst = {
	.always_present = 1,
	.cd_gpio		= MX6_BRD_SD2_CD,
	.wp_gpio		= MX6_BRD_SD2_WP,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.support_18v		= 1,
	.platform_pad_change = plt_sd_pad_change,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct esdhc_platform_data mx6_warp_sd3_data __initconst = {
	.cd_gpio		= ESDHC_CD_PERMANENT, /* MX6_BRD_SD3_CD, */
	.wp_gpio		= -1,
	.always_present		= 1,  /* does not look to make change with BCMDHD in terms of setting */
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.support_18v		= 1,
	.platform_pad_change = plt_sd_pad_change,
};

#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

static const struct anatop_thermal_platform_data
	mx6sl_anatop_thermal_data __initconst = {
			.name = "anatop_thermal",
	};

static int mx6_evk_spi_cs[] = {
	MX6_BRD_ECSPI1_CS0,
};

static int mx6_warp_spi_cs[] = {
	MX6_BRD_ECSPI2_CS0,
};

static const struct spi_imx_master mx6_evk_spi_data __initconst = {
	.chipselect     = mx6_evk_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6_evk_spi_cs),
};

static const struct spi_imx_master mx6_warp_spi_data __initconst = {
	.chipselect     = mx6_warp_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6_warp_spi_cs),
};

#ifdef CONFIG_SENSORS_FXOS8700_SPI
static struct spi_board_info fxos8700_spi1_board_info[] __initdata = {
       {       // The modalias must be the same as spi device driver name
       .modalias       = "fxos8700",
       .max_speed_hz   = 500000,
       .bus_num        = 1,
       .chip_select    = 0,
       },
};
#endif

static void spi_device_init(void)
{
#ifdef CONFIG_SENSORS_FXOS8700_SPI
       spi_register_board_info(fxos8700_spi1_board_info,
                               ARRAY_SIZE(fxos8700_spi1_board_info));
#endif
}

static struct imx_ssi_platform_data mx6_sabresd_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static int spdif_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_actual;
	rate_actual = clk_round_rate(clk, rate);
	clk_set_rate(clk, rate_actual);
	return 0;
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx		= 1,
	.spdif_rx		= 0,
	.spdif_clk_44100	= 1,
	.spdif_clk_48000	= -1,
	.spdif_div_44100	= 23,
	.spdif_clk_set_rate	= spdif_clk_set_rate,
	.spdif_clk		= NULL,
};

int hdmi_enabled;
static int __init hdmi_setup(char *__unused)
{
	hdmi_enabled = 1;
	return 1;
}
__setup("hdmi", hdmi_setup);

static iomux_v3_cfg_t mx6sl_sii902x_hdmi_pads_enabled[] = {
	MX6SL_PAD_LCD_RESET__GPIO_2_19,
	MX6SL_PAD_EPDC_PWRCTRL3__GPIO_2_10,
};

static int sii902x_get_pins(void)
{
	/* Sii902x HDMI controller */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_sii902x_hdmi_pads_enabled, \
		ARRAY_SIZE(mx6sl_sii902x_hdmi_pads_enabled));

	/* Reset Pin */
	gpio_request(MX6_BRD_LCD_RESET, "disp0-reset");
	gpio_direction_output(MX6_BRD_LCD_RESET, 1);

	/* Interrupter pin GPIO */
	gpio_request(MX6SL_BRD_EPDC_PWRCTRL3, "disp0-detect");
	gpio_direction_input(MX6SL_BRD_EPDC_PWRCTRL3);
       return 1;
}

static void sii902x_put_pins(void)
{
	gpio_free(MX6_BRD_LCD_RESET);
	gpio_free(MX6SL_BRD_EPDC_PWRCTRL3);
}

static void sii902x_hdmi_reset(void)
{
	gpio_set_value(MX6_BRD_LCD_RESET, 0);
	msleep(10);
	gpio_set_value(MX6_BRD_LCD_RESET, 1);
	msleep(10);
}

static struct fsl_mxc_lcd_platform_data sii902x_hdmi_data = {
       .ipu_id = 0,
       .disp_id = 0,
       .reset = sii902x_hdmi_reset,
       .get_pins = sii902x_get_pins,
       .put_pins = sii902x_put_pins,
};

static struct imxi2c_platform_data mx6_evk_i2c0_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data warp_i2c1_data = {
	.bitrate = 400000,
};

static struct imxi2c_platform_data mx6sl_warp_i2c2_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("ft5x06-ts", 0x38),
		.irq = gpio_to_irq(PINID_FT5X06_INT),
	}
};

static struct mxc_dvfs_platform_data mx6sl_evk_dvfscore_data = {
	.reg_id			= "VDDCORE",
	.soc_id			= "VDDSOC",
	.clk1_id		= "cpu_clk",
	.clk2_id		= "gpc_dvfs_clk",
	.gpc_cntr_offset	= MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset	= MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset	= MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset	= MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask		= 0x1F800,
	.prediv_offset		= 11,
	.prediv_val		= 3,
	.div3ck_mask		= 0xE0000000,
	.div3ck_offset		= 29,
	.div3ck_val		= 2,
	.emac_val		= 0x08,
	.upthr_val		= 25,
	.dnthr_val		= 9,
	.pncthr_val		= 33,
	.upcnt_val		= 10,
	.dncnt_val		= 10,
	.delay_time		= 80,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_32M,
};

void __init early_console_setup(unsigned long base, struct clk *clk);

static inline void mx6_evk_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL); /* DEBUG UART1 */
}

static int epdc_get_pins(void)
{
	int ret = 0;

	/* Claim GPIOs for EPDC pins - used during power up/down */
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_0, "epdc_d0");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_1, "epdc_d1");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_2, "epdc_d2");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_3, "epdc_d3");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_4, "epdc_d4");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_5, "epdc_d5");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_6, "epdc_d6");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_7, "epdc_d7");
	ret |= gpio_request(MX6SL_BRD_EPDC_GDCLK, "epdc_gdclk");
	ret |= gpio_request(MX6SL_BRD_EPDC_GDSP, "epdc_gdsp");
	ret |= gpio_request(MX6SL_BRD_EPDC_GDOE, "epdc_gdoe");
	ret |= gpio_request(MX6SL_BRD_EPDC_GDRL, "epdc_gdrl");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCLK, "epdc_sdclk");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDOE, "epdc_sdoe");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDLE, "epdc_sdle");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDSHR, "epdc_sdshr");
	ret |= gpio_request(MX6SL_BRD_EPDC_BDR0, "epdc_bdr0");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE0, "epdc_sdce0");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE1, "epdc_sdce1");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE2, "epdc_sdce2");

	return ret;
}

static void epdc_put_pins(void)
{
	gpio_free(MX6SL_BRD_EPDC_SDDO_0);
	gpio_free(MX6SL_BRD_EPDC_SDDO_1);
	gpio_free(MX6SL_BRD_EPDC_SDDO_2);
	gpio_free(MX6SL_BRD_EPDC_SDDO_3);
	gpio_free(MX6SL_BRD_EPDC_SDDO_4);
	gpio_free(MX6SL_BRD_EPDC_SDDO_5);
	gpio_free(MX6SL_BRD_EPDC_SDDO_6);
	gpio_free(MX6SL_BRD_EPDC_SDDO_7);
	gpio_free(MX6SL_BRD_EPDC_GDCLK);
	gpio_free(MX6SL_BRD_EPDC_GDSP);
	gpio_free(MX6SL_BRD_EPDC_GDOE);
	gpio_free(MX6SL_BRD_EPDC_GDRL);
	gpio_free(MX6SL_BRD_EPDC_SDCLK);
	gpio_free(MX6SL_BRD_EPDC_SDOE);
	gpio_free(MX6SL_BRD_EPDC_SDLE);
	gpio_free(MX6SL_BRD_EPDC_SDSHR);
	gpio_free(MX6SL_BRD_EPDC_BDR0);
	gpio_free(MX6SL_BRD_EPDC_SDCE0);
	gpio_free(MX6SL_BRD_EPDC_SDCE1);
	gpio_free(MX6SL_BRD_EPDC_SDCE2);
}

static void epdc_enable_pins(void)
{
	/* Configure MUX settings to enable EPDC use */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_epdc_enable_pads, \
				ARRAY_SIZE(mx6sl_brd_epdc_enable_pads));

	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_0);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_1);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_2);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_3);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_4);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_5);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_6);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_7);
	gpio_direction_input(MX6SL_BRD_EPDC_GDCLK);
	gpio_direction_input(MX6SL_BRD_EPDC_GDSP);
	gpio_direction_input(MX6SL_BRD_EPDC_GDOE);
	gpio_direction_input(MX6SL_BRD_EPDC_GDRL);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCLK);
	gpio_direction_input(MX6SL_BRD_EPDC_SDOE);
	gpio_direction_input(MX6SL_BRD_EPDC_SDLE);
	gpio_direction_input(MX6SL_BRD_EPDC_SDSHR);
	gpio_direction_input(MX6SL_BRD_EPDC_BDR0);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE0);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE1);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE2);
}

static void epdc_disable_pins(void)
{
	/* Configure MUX settings for EPDC pins to
	 * GPIO and drive to 0. */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_epdc_disable_pads, \
				ARRAY_SIZE(mx6sl_brd_epdc_disable_pads));

	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_0, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_1, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_2, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_3, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_4, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_5, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_6, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_7, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDCLK, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDSP, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDOE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDRL, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCLK, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDOE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDLE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDSHR, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_BDR0, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE0, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE1, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE2, 0);
}

static struct fb_videomode e60_v110_mode = {
	.name = "E60_V110",
	.refresh = 50,
	.xres = 800,
	.yres = 600,
	.pixclock = 18604700,
	.left_margin = 8,
	.right_margin = 178,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode e60_v220_mode = {
	.name = "E60_V220",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 30000000,
	.left_margin = 8,
	.right_margin = 164,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
	.refresh = 85,
	.xres = 800,
	.yres = 600,
};
static struct fb_videomode e060scm_mode = {
	.name = "E060SCM",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 26666667,
	.left_margin = 8,
	.right_margin = 100,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode e97_v110_mode = {
	.name = "E97_V110",
	.refresh = 50,
	.xres = 1200,
	.yres = 825,
	.pixclock = 32000000,
	.left_margin = 12,
	.right_margin = 128,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

static struct imx_epdc_fb_mode panel_modes[] = {
	{
		&e60_v110_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		428,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e60_v220_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		465,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		9,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e060scm_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		419,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		5,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e97_v110_mode,
		8,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		632,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		3,      /* num_ce */
	}
};

static struct imx_epdc_fb_platform_data epdc_data = {
	.epdc_mode = panel_modes,
	.num_modes = ARRAY_SIZE(panel_modes),
	.get_pins = epdc_get_pins,
	.put_pins = epdc_put_pins,
	.enable_pins = epdc_enable_pins,
	.disable_pins = epdc_disable_pins,
};

static int spdc_get_pins(void)
{
	int ret = 0;

	/* Claim GPIOs for SPDC pins - used during power up/down */
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_0, "SPDC_D0");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_1, "SPDC_D1");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_2, "SPDC_D2");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_3, "SPDC_D3");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_4, "SPDC_D4");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_5, "SPDC_D5");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_6, "SPDC_D6");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_7, "SPDC_D7");

	ret |= gpio_request(MX6SL_BRD_EPDC_GDOE, "SIPIX_YOE");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_9, "SIPIX_PWR_RDY");

	ret |= gpio_request(MX6SL_BRD_EPDC_GDSP, "SIPIX_YDIO");

	ret |= gpio_request(MX6SL_BRD_EPDC_GDCLK, "SIPIX_YCLK");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDSHR, "SIPIX_XDIO");

	ret |= gpio_request(MX6SL_BRD_EPDC_SDLE, "SIPIX_LD");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE1, "SIPIX_SOE");

	ret |= gpio_request(MX6SL_BRD_EPDC_SDCLK, "SIPIX_XCLK");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_10, "SIPIX_SHD_N");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE0, "SIPIX2_CE");

	return ret;
}

static void spdc_put_pins(void)
{
	gpio_free(MX6SL_BRD_EPDC_SDDO_0);
	gpio_free(MX6SL_BRD_EPDC_SDDO_1);
	gpio_free(MX6SL_BRD_EPDC_SDDO_2);
	gpio_free(MX6SL_BRD_EPDC_SDDO_3);
	gpio_free(MX6SL_BRD_EPDC_SDDO_4);
	gpio_free(MX6SL_BRD_EPDC_SDDO_5);
	gpio_free(MX6SL_BRD_EPDC_SDDO_6);
	gpio_free(MX6SL_BRD_EPDC_SDDO_7);

	gpio_free(MX6SL_BRD_EPDC_GDOE);
	gpio_free(MX6SL_BRD_EPDC_SDDO_9);
	gpio_free(MX6SL_BRD_EPDC_GDSP);
	gpio_free(MX6SL_BRD_EPDC_GDCLK);
	gpio_free(MX6SL_BRD_EPDC_SDSHR);
	gpio_free(MX6SL_BRD_EPDC_SDLE);
	gpio_free(MX6SL_BRD_EPDC_SDCE1);
	gpio_free(MX6SL_BRD_EPDC_SDCLK);
	gpio_free(MX6SL_BRD_EPDC_SDDO_10);
	gpio_free(MX6SL_BRD_EPDC_SDCE0);
}

static void spdc_enable_pins(void)
{
	/* Configure MUX settings to enable SPDC use */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_spdc_enable_pads, \
				ARRAY_SIZE(mx6sl_brd_spdc_enable_pads));

	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_0);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_1);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_2);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_3);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_4);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_5);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_6);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_7);
	gpio_direction_input(MX6SL_BRD_EPDC_GDOE);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_9);
	gpio_direction_input(MX6SL_BRD_EPDC_GDSP);
	gpio_direction_input(MX6SL_BRD_EPDC_GDCLK);
	gpio_direction_input(MX6SL_BRD_EPDC_SDSHR);
	gpio_direction_input(MX6SL_BRD_EPDC_SDLE);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE1);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCLK);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_10);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE0);
}

static void spdc_disable_pins(void)
{
	/* Configure MUX settings for SPDC pins to
	 * GPIO and drive to 0. */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_spdc_disable_pads, \
				ARRAY_SIZE(mx6sl_brd_spdc_disable_pads));

	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_0, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_1, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_2, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_3, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_4, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_5, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_6, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_7, 0);

	gpio_direction_output(MX6SL_BRD_EPDC_GDOE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_9, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDSP, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDCLK, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDSHR, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDLE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE1, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCLK, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_10, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE0, 0);
}

static struct imx_spdc_panel_init_set spdc_init_set = {
	.yoe_pol = false,
	.dual_gate = false,
	.resolution = 0,
	.ud = false,
	.rl = false,
	.data_filter_n = true,
	.power_ready = true,
	.rgbw_mode_enable = false,
	.hburst_len_en = true,
};

static struct fb_videomode erk_1_4_a01 = {
	.name = "ERK_1_4_A01",
	.refresh = 50,
	.xres = 800,
	.yres = 600,
	.pixclock = 40000000,
	.vmode = FB_VMODE_NONINTERLACED,
};

static struct imx_spdc_fb_mode spdc_panel_modes[] = {
	{
		&erk_1_4_a01,
		&spdc_init_set,
		.wave_timing = "pvi"
	},
};

static struct imx_spdc_fb_platform_data spdc_data = {
	.spdc_mode = spdc_panel_modes,
	.num_modes = ARRAY_SIZE(spdc_panel_modes),
	.get_pins = spdc_get_pins,
	.put_pins = spdc_put_pins,
	.enable_pins = spdc_enable_pins,
	.disable_pins = spdc_disable_pins,
};

static int __init early_use_spdc_sel(char *p)
{
	spdc_sel = 1;
	return 0;
}
early_param("spdc", early_use_spdc_sel);

static void setup_spdc(void)
{
	/* GPR0[8]: 0:EPDC, 1:SPDC */
	if (spdc_sel)
		mxc_iomux_set_gpr_register(0, 8, 1, 1);
}

static void imx6_evk_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX6_BRD_USBOTG1_PWR, 1);
	else
		gpio_set_value(MX6_BRD_USBOTG1_PWR, 0);
}

static void imx6_evk_usbh1_vbus(bool on)
{
	if (on)
		gpio_set_value(MX6_BRD_USBOTG2_PWR, 1);
	else
		gpio_set_value(MX6_BRD_USBOTG2_PWR, 0);
}

static void __init mx6_evk_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	/* disable external charger detect,
	 * or it will affect signal quality at dp.
	 */

	/*ret = gpio_request(MX6_BRD_USBOTG1_PWR, "usbotg-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6_BRD_USBOTG1_PWR:%d\n", ret);
		return;
	}
	gpio_direction_output(MX6_BRD_USBOTG1_PWR, 0);

	ret = gpio_request(MX6_BRD_USBOTG2_PWR, "usbh1-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6_BRD_USBOTG2_PWR:%d\n", ret);
		return;
	}
	gpio_direction_output(MX6_BRD_USBOTG2_PWR, 0);

	mx6_set_otghost_vbus_func(imx6_evk_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6_evk_usbh1_vbus);
*/
#ifdef CONFIG_USB_EHCI_ARC_HSIC
	mx6_usb_h2_init();
#endif
}

static struct platform_pwm_backlight_data mx6_evk_pwm_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 128,
	.pwm_period_ns	= 50000,
};
static struct fb_videomode wvga_video_modes[] = {
	{
	 /* 800x480 @ 57 Hz , pixel clk @ 32MHz */
	 "SEIKO-WVGA", 60, 800, 480, 29850, 89, 164, 23, 10, 10, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

static struct mxc_fb_platform_data wvga_fb_data[] = {
	{
	 .interface_pix_fmt = V4L2_PIX_FMT_RGB24,
	 .mode_str = "SEIKO-WVGA",
	 .mode = wvga_video_modes,
	 .num_modes = ARRAY_SIZE(wvga_video_modes),
	 },
};

static struct platform_device lcd_wvga_device = {
	.name = "lcd_seiko",
};

static struct fb_videomode hdmi_video_modes[] = {
	{
	 /* 1920x1080 @ 60 Hz , pixel clk @ 148MHz */
	 "sii9022x_1080p60", 60, 1920, 1080, 6734, 148, 88, 36, 4, 44, 5,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

static struct mxc_fb_platform_data hdmi_fb_data[] = {
	{
	 .interface_pix_fmt = V4L2_PIX_FMT_RGB24,
	 .mode_str = "1920x1080M@60",
	 .mode = hdmi_video_modes,
	 .num_modes = ARRAY_SIZE(hdmi_video_modes),
	 },
};

//----------------------- REVO LH154 LCD ----------------------------------
static struct fb_videomode lh154_video_modes[] = {
	{
	.name = "LH154",
	.refresh = 60,
	.xres = 240,
	.yres = 240,
	.pixclock = 289352,
	.left_margin = 0,
	.right_margin = 0,
	.upper_margin = 0,
	.lower_margin = 0,
	.hsync_len = 0,
	.vsync_len = 0,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
	},
};

static struct mxc_fb_platform_data lh154_fb_data[] = {
	{
	.interface_pix_fmt = V4L2_PIX_FMT_RGB24, // ELCDIF_PIX_FMT_RGB666 ?
	.mode_str = "LH154",
	.mode = lh154_video_modes,
	.num_modes = ARRAY_SIZE(lh154_video_modes),
	},
};

static struct platform_device lcd_lh154_device = {
	.name = "lcd_lh154",
};

//---------------------------------------------------------


static void __init elan_ts_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_elan_pads,
					ARRAY_SIZE(mx6sl_brd_elan_pads));

	/* ELAN Touchscreen */
	gpio_request(MX6SL_BRD_ELAN_INT, "elan-interrupt");
	gpio_direction_input(MX6SL_BRD_ELAN_INT);

	gpio_request(MX6SL_BRD_ELAN_CE, "elan-cs");
	gpio_direction_output(MX6SL_BRD_ELAN_CE, 1);
	gpio_direction_output(MX6SL_BRD_ELAN_CE, 0);

	gpio_request(MX6SL_BRD_ELAN_RST, "elan-rst");
	gpio_direction_output(MX6SL_BRD_ELAN_RST, 1);
	gpio_direction_output(MX6SL_BRD_ELAN_RST, 0);
	mdelay(1);
	gpio_direction_output(MX6SL_BRD_ELAN_RST, 1);
	gpio_direction_output(MX6SL_BRD_ELAN_CE, 1);
}

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{
	u32 value;
	void __iomem *mx6_snvs_base = MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);

	value = readl(mx6_snvs_base + SNVS_LPCR);
	/* set TOP and DP_EN bit */
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

static void __init uart3_init(void){
	mxc_iomux_v3_setup_multiple_pads(mx6sl_uart3_pads,
		ARRAY_SIZE(mx6sl_uart3_pads));
	imx6sl_add_imx_uart(2, NULL);
}

static void mx6sl_warp_suspend_enter()
{
	iomux_v3_cfg_t *p = suspend_enter_pads;
	int i;

	/* Set PADCTRL to 0 for all IOMUX. */
	for (i = 0; i < ARRAY_SIZE(warp_suspend_enter_pads); i++) {
		suspend_exit_pads[i] = *p;
		*p &= ~MUX_PAD_CTRL_MASK;
		/* Enable the Pull down and the keeper
		  * Set the drive strength to 0.
		  */
		*p |= ((u64)0x3000 << MUX_PAD_CTRL_SHIFT);
		p++;
	}
	mxc_iomux_v3_get_multiple_pads(warp_suspend_exit_pads,
			ARRAY_SIZE(warp_suspend_exit_pads));
	mxc_iomux_v3_setup_multiple_pads(warp_suspend_enter_pads,
			ARRAY_SIZE(warp_suspend_enter_pads));

}

static struct platform_pwm_clk_data mx6_warp_pwm4_clk_data = {
    .pwm_id        = 3,
    .pwm_period_ns    = 100,
};

static void mx6sl_warp_suspend_exit()
{
	mxc_iomux_v3_setup_multiple_pads(warp_suspend_exit_pads,
			ARRAY_SIZE(warp_suspend_exit_pads));
}

#if BRCM  /* add routines for BRCM Wireless */
static int bcmdhd_set_power(int on)
{
       /* can only set GPIO for correctly implemented hardware */
       gpio_set_value(WARP_WL_REG_ON, on);
       msleep(500);
       return 0;
}

static int bcmdhd_set_card_detect(int detect)
{
	/* turn on/off GPIO so carddetect will register in SDIO driver */
	/* dummy function call in this case */
	return 0;
}

static struct wifi_platform_data bcmdhd_data = {
       .set_power      = bcmdhd_set_power,
       .set_carddetect = bcmdhd_set_card_detect,
};

static struct resource bcmdhd_res[] = {
       {
               .name = "bcmdhd_wlan_irq",
               .start = GPIO_BCMDHD_OOB,
               .end = GPIO_BCMDHD_OOB,
               .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
        }
};

static struct platform_device bcmdhd_device = {
       .name   = "bcmdhd_wlan",
       .dev    = {
               .platform_data = &bcmdhd_data,
       },
       .num_resources = ARRAY_SIZE(bcmdhd_res),
       .resource = bcmdhd_res,
};

static struct gpio bcmdhd_gpios[] __initdata = {
       { GPIO_BCMDHD_PWR,      GPIOF_OUT_INIT_LOW,	"bcmdhd_pwr"    },
       { GPIO_BCMDHD_OOB,      GPIOF_IN,		"bcmdhd_oob"    },
};

static void __init bcmdhd_init(void)
{
       int status;
	   /* OOB IRQ and WLAN ENABLE lines are configured elsewhere for correct logic level */
       /* Request of GPIO lines */
       status = gpio_request_array(bcmdhd_gpios, ARRAY_SIZE(bcmdhd_gpios));
       if (status) {
               return;
       }

       bcmdhd_res[0].start = gpio_to_irq(GPIO_BCMDHD_OOB);
       bcmdhd_res[0].end = bcmdhd_res[0].start;
       platform_device_register(&bcmdhd_device);
}
#endif  /* BRCM WLAN modifications */

int max77696_pmic_gpio_init(void)
{
	int ret = 0;
	gpio_request(IMX_GPIO_NR(3,22), "max77696-irq");
	gpio_direction_input(IMX_GPIO_NR(3,22));
	return ret;
}

/*!
 * Board specific initialization.
 */
static void __init mx6_warp_init(void)
{
	u32 i;

	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_pads,
					ARRAY_SIZE(mx6sl_brd_pads));
	mxc_iomux_v3_setup_multiple_pads(warp_brd_pads,
					ARRAY_SIZE(warp_brd_pads));

	max77696_pmic_gpio_init();

//	mxc_iomux_v3_setup_multiple_pads(mcu8080display_pads,ARRAY_SIZE(mcu8080display_pads));

	gpio_request(PINID_MIPI_TE,    "lcd_te");	// MIPI_TE
	gpio_request(PINID_LCD_INTN,   "lcd_intn");	// LCD_INTn
	gpio_request(PINID_LCD_RD,     "lcd_rd"); 	// LCD_RDX
	gpio_request(PINID_LCD_RS,     "lcd_rs"); 	// LCD_DCX
	gpio_request(PINID_LCD_RSTN,   "lcd_rstn");  	// LCD_RSTn
	gpio_request(PINID_MIPI_BSYNC, "lcd_bsync");  	// MIPI_B_SYNC
	gpio_request(PINID_MIPI_RSTN,  "mipi_rstn"); 	// MIPI_RSTn

	gpio_direction_input(PINID_MIPI_TE); 		// MIPI_TE
	gpio_direction_output(PINID_LCD_RSTN, 1);  	// LCD_RSTn
	gpio_direction_input(PINID_LCD_INTN); 		// LCD_INTn
	gpio_direction_input(PINID_MIPI_BSYNC);  	// MIPI_B_SYNC
	gpio_direction_output(PINID_MIPI_RSTN, 1); 	// MIPI_RSTn
	gpio_direction_output(PINID_LCD_RD, 1); 	// LCD_RDX
	gpio_direction_output(PINID_LCD_RS, 1); 	// LCD_DCX(ssd)/LCD_RS(mx6)

	gpio_request(PINID_FT5X06_INT, "ft5216_int");
	gpio_direction_input(PINID_FT5X06_INT);

//	elan_ts_init(); // REVO - removed

//	gp_reg_id = mx6sl_evk_dvfscore_data.reg_id;
//	soc_reg_id = mx6sl_evk_dvfscore_data.soc_id;

//	imx6q_add_imx_snvs_rtc();

	imx6q_add_imx_i2c(0, &mx6_evk_i2c0_data);
	imx6q_add_imx_i2c(1, &warp_i2c1_data);

//	i2c_register_board_info(0, mxc_i2c0_board_info,
//			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));

	/* SPI */
	imx6q_add_ecspi(1, &mx6_warp_spi_data);
	spi_device_init();

//	imx6q_add_anatop_thermal_imx(1, &mx6sl_anatop_thermal_data);

	mx6_evk_init_uart();

	//platform_device_register(&evk_vmmc_reg_devices);
	imx6q_add_sdhci_usdhc_imx(1, &mx6_warp_sd2_data);

#if BRCM
	/* bcm dhd wifi init */
	bcmdhd_init();
	gpio_direction_output(WARP_WL_REG_ON, 1); // set direction to output and value to 1
	bcmdhd_set_power(1);  /* need to take WLAN core out of reset prior to probing */
	imx6q_add_sdhci_usdhc_imx(2, &mx6_warp_sd3_data);
#endif /* BRCM */

	mx6_evk_init_usb();
//	imx6q_add_otp();

	// PWM Output Clock to SSD2805C

	printk(KERN_INFO "REVO: Starting PWMs ******************************\n");
	imx6q_add_mxc_pwm(3);
	imx6sl_add_mxc_pwm_clk(3, &mx6_warp_pwm4_clk_data);

/*	if (hdmi_enabled) {
		imx6dl_add_imx_elcdif(&hdmi_fb_data[0]);
	} else {
		imx6dl_add_imx_elcdif(&wvga_fb_data[0]);

		gpio_request(MX6_BRD_LCD_PWR_EN, "elcdif-power-on");
		gpio_direction_output(MX6_BRD_LCD_PWR_EN, 1);
		mxc_register_device(&lcd_wvga_device, NULL);
	}*/

	imx6dl_add_imx_elcdif(&lh154_fb_data[0]);

	mxc_register_device(&lcd_lh154_device, NULL);
	imx6dl_add_imx_pxp();
	imx6dl_add_imx_pxp_client();

	setup_spdc();
	/*
	if (!spdc_sel)
		imx6dl_add_imx_epdc(&epdc_data);
	else
		imx6sl_add_imx_spdc(&spdc_data);
	*/
//	imx6q_add_dvfs_core(&mx6sl_evk_dvfscore_data);

	uart3_init();

//	mxc_register_device(&mxc_bt_rfkill, &mxc_bt_rfkill_data);

//	imx6q_add_viim();
//	imx6q_add_imx2_wdt(0, NULL);

	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
//	imx6sl_add_imx_keypad(&mx6sl_evk_map_data);
//	imx6q_add_busfreq();
//	imx6sl_add_dcp();
//	imx6sl_add_rngb();
	imx6sl_add_imx_pxp_v4l2();

	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
	/* Register charger chips */

//	platform_device_register(&evk_max8903_charger_1);
//	pm_power_off = mx6_snvs_poweroff;
//	imx6q_add_pm_imx(0, &mx6sl_warp_pm_data);

	if (imx_ion_data.heaps[0].size)
		platform_device_register_resndata(NULL, "ion-mxc", 0, NULL, 0, \
		&imx_ion_data, sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));

}

extern void __iomem *twd_base;
static void __init mx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6sl_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init   = mx6_timer_init,
};

static void __init mx6_warp_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, MEMBLOCK_ALLOC_ACCESSIBLE);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

#if defined(CONFIG_ION)
	if (imx_ion_data.heaps[0].size) {
		phys = memblock_alloc(imx_ion_data.heaps[0].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[0].size);
		imx_ion_data.heaps[0].base = phys;
	}
#endif
}

MACHINE_START(MX6SL_EVK, "WaRP Board Wearable Reference Platform")
	.boot_params	= MX6SL_PHYS_OFFSET + 0x100,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= mx6_warp_init,
	.timer		= &mxc_timer,
	.reserve	= mx6_warp_reserve,
MACHINE_END
