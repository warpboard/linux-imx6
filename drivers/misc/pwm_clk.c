/*
 * Copyright (C) 2014 Revolution Robotics, Inc.
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

/* Based on drivers/video/backlight/pwm_bl.c
 *
 * Simple PWM based clock waveform output
 *  Board code has to setup
 *  1) Pin configuration so PWM waveforms can output
 *  2) Platform_data being correctly configured
 *    Example:
 *      static struct platform_pwm_clk_data mx6_warp_pwm4_clk_data = {
 *        .pwm_id		= 2,
 *	  .pwm_period_ns	= 100,
 *      };
 *
 *      imx6q_add_mxc_pwm(2);
 *      imx6sl_add_mxc_pwm_clk(2, &mx6_warp_pwm3_clk_data);
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_clk.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <mach/hardware.h>

struct pwm_clk_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
};

#define HZ_TO_NANOSECONDS(x) (1000000000UL/(x))

#define MX_PWMCR                	0x00    /* PWM Control Register */
#define MX_PWMSR			0x04
#define MX_PWMIR			0x08
#define MX_PWMSAR                	0x0C    /* PWM Sample Register */
#define MX_PWMPR                 	0x10    /* PWM Period Register */
#define MX_PWMCNR			0x14
#define MX_PWMCR_PRESCALER(x)    (((x - 1) & 0xFFF) << 4)
#define MX_PWMCR_DOZEEN                (1 << 24)
#define MX_PWMCR_WAITEN                (1 << 23)
#define MX_PWMCR_DBGEN			(1 << 22)
#define MX_PWMCR_CLKSRC_IPG_HIGH (2 << 16)
#define MX_PWMCR_CLKSRC_IPG      (1 << 16)
#define MX_PWMCR_SWR             (1 << 3)
#define MX_PWMCR_EN              (1 << 0)

#define MX_PWMCR_STOPEN		(1 << 25)
#define MX_PWMCR_DOZEEN                (1 << 24)
#define MX_PWMCR_WAITEN                (1 << 23)
#define MX_PWMCR_DBGEN			(1 << 22)
#define MX_PWMCR_CLKSRC_IPG		(1 << 16)
#define MX_PWMCR_CLKSRC_IPG_32k	(3 << 16)

static int pwm_clk_probe(struct platform_device *pdev)
{
	struct platform_pwm_clk_data *data = pdev->dev.platform_data;
	struct pwm_clk_data *pclkdata;
	int ret;
	int reg =0;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pclkdata = kzalloc(sizeof(*pclkdata), GFP_KERNEL);
	if (!pclkdata) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pclkdata->period = data->pwm_period_ns;
	pclkdata->dev = &pdev->dev;

	pclkdata->pwm = pwm_request(data->pwm_id, "pwmclock");
	if (IS_ERR(pclkdata->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for output\n");
		ret = PTR_ERR(pclkdata->pwm);
		goto err_pwm;
	} else
		dev_dbg(&pdev->dev, "got PWM for output\n");

	pwm_config(pclkdata->pwm, pclkdata->period / 2, pclkdata->period);
	pwm_enable(pclkdata->pwm);

	platform_set_drvdata(pdev, pclkdata);
	return 0;

err_pwm:
	kfree(pclkdata);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_clk_remove(struct platform_device *pdev)
{
	struct pwm_clk_data *pclkdata = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

//	pwm_config(pclkdata->pwm, 0, pclkdata->period);
//	pwm_disable(pclkdata->pwm);
	pwm_free(pclkdata->pwm);
	kfree(pclkdata);
	return 0;
}

#ifdef CONFIG_PM
static int pwm_clk_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct pwm_clk_data *pclkdata = platform_get_drvdata(pdev);

//	if(pclkdata->period)
//		pwm_disable(pclkdata->pwm);
	return 0;
}

static int pwm_clk_resume(struct platform_device *pdev)
{
	struct pwm_clk_data *pclkdata = platform_get_drvdata(pdev);

	if (pclkdata->period) {
//		pwm_config(pclkdata->pwm, pclkdata->period / 2, pclkdata->period);
//		pwm_enable(pclkdata->pwm);
	}

	return 0;
}
#else
#define pwm_clk_suspend	NULL
#define pwm_clk_resume	NULL
#endif

static struct platform_driver pwm_clk_driver = {
	.driver		= {
		.name	= "pwm-clk",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_clk_probe,
	.remove		= pwm_clk_remove,
	.suspend	= pwm_clk_suspend,
	.resume		= pwm_clk_resume,
};

static int __init pwm_clk_init(void)
{
	return platform_driver_register(&pwm_clk_driver);
}
subsys_initcall(pwm_clk_init);

static void __exit pwm_clk_exit(void)
{
	platform_driver_unregister(&pwm_clk_driver);
}
module_exit(pwm_clk_exit);

MODULE_AUTHOR("Jacob Postman <jacob@revolution-robotics.com");
MODULE_DESCRIPTION("PWM clock output");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-clk");
