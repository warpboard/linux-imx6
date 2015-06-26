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
#include "board-mx6sl_warp.h"


static void mx6sl_warp_suspend_enter(void);
static void mx6sl_warp_suspend_exit(void);

extern char *gp_reg_id;
extern char *soc_reg_id;

// Additional Defines for BRCM WLAN
#define BRCM 1

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

static int warp_spi1_cs[] = {
	MX6_BRD_ECSPI1_CS0,
};

static const struct spi_imx_master warp_spi1_data __initconst = {
	.chipselect     = warp_spi1_cs,
	.num_chipselect = ARRAY_SIZE(warp_spi1_cs),
};

static int warp_spi2_cs[] = {
	MX6_BRD_ECSPI2_CS0,
};

static const struct spi_imx_master warp_spi2_data __initconst = {
	.chipselect     = warp_spi2_cs,
	.num_chipselect = ARRAY_SIZE(warp_spi2_cs),
};


#ifdef CONFIG_SENSORS_FXOS8700_SPI
static struct spi_board_info fxos8700_spi_board_info[] __initdata = {
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
       spi_register_board_info(fxos8700_spi_board_info,
                               ARRAY_SIZE(fxos8700_spi_board_info));
#endif
}

static struct imxi2c_platform_data warp_i2c1_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data warp_i2c2_data = {
	.bitrate = 400000,
};

static struct imxi2c_platform_data warp_i2c3_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("ft5x06-ts", 0x38),
		.irq = gpio_to_irq(PINID_FT5X06_INT),
	}
};

static struct mxc_dvfs_platform_data mx6sl_dvfscore_data = {
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

#ifdef MURATA_BLUETOOTH_ENABLE

static int bt_uart_enabled=0;
static int __init bt_uart_setup(char * __unused)
{
	bt_uart_enabled = 1;
	return 1;
}
__setup("bluetooth", bt_uart_setup);

#ifdef WARP_REV1P10
static const struct imxuart_platform_data mx6sl_warp_uart2_data __initconst = {
//	.flags      = IMXUART_HAVE_RTSCTS,
	.dma_req_rx = MX6Q_DMA_REQ_UART2_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART2_TX,
};

static void __init uart2_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6sl_uart2_pads,
					ARRAY_SIZE(mx6sl_uart2_pads));
	// uart2 data NULL to workaround board rev1.10 CTS/RTS issue
	imx6sl_add_imx_uart(1, NULL); //&mx6sl_warp_uart2_data);
}

#else
static const struct imxuart_platform_data mx6sl_warp_uart5_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS,
	.dma_req_rx = MX6Q_DMA_REQ_UART5_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART5_TX,
};

static void __init uart5_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6sl_uart5_pads,
		ARRAY_SIZE(mx6sl_uart5_pads));
	imx6sl_add_imx_uart(4, &mx6sl_warp_uart5_data);
}
#endif

static void mx6sl_warp_bt_reset(void)
{
	/* Toggle BT_RST_N; drive low and then drive high again */
	gpio_request(WARP_BT_RST_N, "bt-rst-n");
	gpio_direction_output(WARP_BT_RST_N, 0);
	/* pull down BT_RST_N pin at least >10ms */
	mdelay(10);
	/* now drive BT_RST_N high */
	gpio_set_value(WARP_BT_RST_N, 1);
	gpio_free(WARP_BT_RST_N);
}

static int mx6sl_warp_bt_power_change(int status)
{
	if (status)
		mx6sl_warp_bt_reset();
	return 0;
}

static struct platform_device mxc_bt_rfkill = {
	.name = "mxc_bt_rfkill",
};

static struct imx_bt_rfkill_platform_data mxc_bt_rfkill_data = {
	.power_change = mx6sl_warp_bt_power_change,
};

#endif /* MURATA_BLUETOOTH_ENABLE */

static inline void uart1_init(void)
{
	imx6q_add_imx_uart(0, NULL); /* DEBUG UART1 */
}

static void warp_usbotg_vbus(bool on)
{
	// PMIC should handle otg/host interaction
	// but if necessary additional control can
	// be added here.
}

static void warp_usbh1_vbus(bool on)
{
	// PMIC should handle otg/host interaction
	// but if necessary additional control can
	// be added here.
}

static void __init mx6_evk_init_usb(void)
{
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	mx6_set_otghost_vbus_func(warp_usbotg_vbus);
	mx6_set_host1_vbus_func(warp_usbh1_vbus);

#ifdef CONFIG_USB_EHCI_ARC_HSIC
	mx6_usb_h2_init();
#endif
}

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


#define SNVS_LPCR 0x38
static unsigned char pmic_powerdown_mode=0x20;
static int pmic_powerdown_setup(char * __unused)
{
	pmic_powerdown_mode = 0x04;
	return 1;
}
__setup("pmic_usefullshutdown", pmic_powerdown_setup);

static void mx6_snvs_poweroff(void)
{
	u32 value;
	void __iomem *mx6_snvs_base = MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);

	if(pmic_powerdown_mode == 0x20)
		printk(KERN_INFO "Removing power. VSYS1 going off.\n");
	else if(pmic_powerdown_mode == 0x04)
		printk(KERN_INFO "Removing power. Leaving VSYS1 on. \n");


	value = readl(mx6_snvs_base + SNVS_LPCR);
	/* set TOP and DP_EN bit */
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);


	// Disable RTC Alarm and set UIC_WK to edge triggered
	max77696_chip_write(0,0x02,0x31);

	// Disable Backlight
	max77696_chip_write(0,0x6C,0x00);

	// Enter Factory Ship (turn off all power including SYS1)
	max77696_chip_write(0,0,0x01);
	max77696_chip_write(0,0,pmic_powerdown_mode);

}

static void __init uart3_init(void){
	mxc_iomux_v3_setup_multiple_pads(mx6sl_uart3_pads,
		ARRAY_SIZE(mx6sl_uart3_pads));
	imx6sl_add_imx_uart(2, NULL);
}

static void mx6sl_warp_suspend_enter()
{
	iomux_v3_cfg_t *p = warp_suspend_enter_pads;
	int i;

	/* Set PADCTRL to 0 for all IOMUX. */
	for (i = 0; i < ARRAY_SIZE(warp_suspend_enter_pads); i++) {
		warp_suspend_exit_pads[i] = *p;
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
               .start = WARP_GPIO0_WL_HOSTWAKE,
               .end = WARP_GPIO0_WL_HOSTWAKE,
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
       { WARP_WL_REG_ON,      GPIOF_OUT_INIT_LOW,	"bcmdhd_pwr"    },
       { WARP_GPIO0_WL_HOSTWAKE,      GPIOF_IN,		"bcmdhd_oob"    },
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

       bcmdhd_res[0].start = gpio_to_irq(WARP_GPIO0_WL_HOSTWAKE);
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
	mxc_iomux_v3_setup_multiple_pads(warp_brd_pads,
					ARRAY_SIZE(warp_brd_pads));

	max77696_pmic_gpio_init();

	gp_reg_id = mx6sl_dvfscore_data.reg_id;
	soc_reg_id = mx6sl_dvfscore_data.soc_id;

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

	imx6q_add_imx_i2c(0, &warp_i2c1_data);
	imx6q_add_imx_i2c(1, &warp_i2c2_data);

	i2c_register_board_info(1, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	/* SPI */
	imx6q_add_ecspi(1, &warp_spi2_data);
	spi_device_init();

	imx6q_add_anatop_thermal_imx(1, &mx6sl_anatop_thermal_data);

	uart1_init();

	imx6q_add_sdhci_usdhc_imx(1, &mx6_warp_sd2_data);

#if BRCM
	/* bcm dhd wifi init */
	bcmdhd_init();
	gpio_direction_output(WARP_WL_REG_ON, 1); // set direction to output and value to 1
	bcmdhd_set_power(1);  /* need to take WLAN core out of reset prior to probing */
	imx6q_add_sdhci_usdhc_imx(2, &mx6_warp_sd3_data);
#endif /* BRCM */

	mx6_evk_init_usb();

	// PWM Output Clock to SSD2805C
	printk(KERN_INFO "Starting clock output for MIPI bridge\n");
	imx6q_add_mxc_pwm(3);
	imx6sl_add_mxc_pwm_clk(3, &mx6_warp_pwm4_clk_data);

	imx6dl_add_imx_elcdif(&lh154_fb_data[0]);

	mxc_register_device(&lcd_lh154_device, NULL);
	imx6dl_add_imx_pxp();
	imx6dl_add_imx_pxp_client();

	imx6q_add_dvfs_core(&mx6sl_dvfscore_data);

	uart3_init();
#ifdef MURATA_BLUETOOTH_ENABLE
	if(bt_uart_enabled)
	{
#ifdef WARP_REV1P10
		uart2_init();
#else
		uart5_init();
#endif
		/* Drive BT_REG_ON high to ensure BT core is not held in reset. */
		gpio_request(WARP_BT_REG_ON, "bt-reg-on");
		gpio_request(WARP_BT_GPIO1_BTWAKE, "bt-wake");
		gpio_request(WARP_BT_GPIO1_HOSTWAKE, "bt-hostwake");
		gpio_request(WARP_BT_RST_N, "bt-rst-n");

		gpio_direction_output(WARP_BT_REG_ON, 1);
		gpio_direction_output(WARP_BT_GPIO1_BTWAKE, 1);
		gpio_direction_output(WARP_BT_RST_N, 1);
		gpio_direction_input(WARP_BT_GPIO1_HOSTWAKE);

		gpio_free(WARP_BT_REG_ON);
		mdelay(100);  /* delay to let BT core out of reset */

		/* Drive BT_GPIO1_BTWAKE high as well to make sure BT device does not sleep */
		/* NOTE: move this code to handshaking/powercontrol routine at a later date */
		gpio_free(WARP_BT_GPIO1_BTWAKE);
		mdelay(100);  /* delay to let BT core out of reset */

		/* setup "handle" for BT stack to (un)reset BT core with BT_RST_N */
		mxc_register_device(&mxc_bt_rfkill, &mxc_bt_rfkill_data);
	}
#endif /* MURATA_BLUETOOTH_ENABLE */

	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_add_busfreq();
	imx6sl_add_dcp();
	imx6sl_add_rngb();
	imx6sl_add_imx_pxp_v4l2();

	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);

	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_pm_imx(0, &mx6sl_warp_pm_data);

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
