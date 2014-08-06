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

#ifndef _BOARD_MX6SL_WARP_H
#define _BOARD_MX6SL_WARP_H

// PMIC
#define MX6_GPIO_PMIC_IRQ       IMX_GPIO_NR(3, 22)
// SPI for Accelerometer
#define MX6_BRD_ECSPI2_CS0	IMX_GPIO_NR(4, 15)	// ECSPI2_SS0

// GPIO for WL
#define WARP_WL_REG_ON			IMX_GPIO_NR(4, 5)
#define WARP_GPIO0_WL_HOSTWAKE		IMX_GPIO_NR(4, 7)
#define WARP_BT_RST_N			IMX_GPIO_NR(4, 6)
#define WARP_BT_REG_ON			IMX_GPIO_NR(3, 28)


// WaRP Specific PAD_CTRL Definitions

#define MX6SL_PWM_PAD_CTRL    ( PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_80ohm |    \
                PAD_CTL_SRE_FAST | PAD_CTL_LVE)

static iomux_v3_cfg_t warp_brd_pads[] = {

	/* PWM OUTPUT */
	MX6SL_PAD_AUD_MCLK__PWM4_PWMO | MUX_PAD_CTRL(MX6SL_PWM_PAD_CTRL)    // LCD_CLK
};

#endif
