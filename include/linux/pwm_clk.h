/*
 * Generic PWM output driver data - see drivers/misc/pwm_clk.c
 */
#ifndef __LINUX_PWM_CLK_H
#define __LINUX_PWM_CLK_H

struct platform_pwm_clk_data {
	int pwm_id;
	unsigned int pwm_period_ns;
	int (*init)(struct device *dev);
	void (*exit)(struct device *dev);
};

#define imx6sl_add_mxc_pwm_clk(id, pdata) 			\
	platform_device_register_resndata(NULL, "pwm-clk",	\
			id, NULL, 0, pdata, sizeof(*pdata));

#endif
