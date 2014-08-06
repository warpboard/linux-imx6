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

#ifndef __IF_SSD2805_LOWLEVEL_H__
#define __IF_SSD2805_LOWLEVEL_H__

#define SSD2805_MODULE_NAME		"if_ssd2805"

#define MAX_PAYLOAD_SIZE		2048

struct ssd2805_device {
	int			pll_configured;
	int			enabled;
	int			configured;
	int			gpio_read_sem;
	int			reg_map[256];
};

extern struct ssd2805_device *instance;

void ssd2805_dump_registers(void);
void dump(void);

/* in lowlevel files */
int ssd2805_command(int cmd);
int ssd2805_write_reg(int reg, int value);
int ssd2805_read_reg(int reg);
int mipi_dcs_command(int cmd);
int mipi_dcs_write(int cmd, u8 *wbuf, int wlen);
int mipi_dcs_read(int cmd, u8 *rbuf, int rlen);

/* inlines */
extern inline int ssd2805_ensure_reg(int reg, int value)
{
	return (instance->reg_map[reg] == value) ? 0 : ssd2805_write_reg(reg, value);
}

extern inline int ssd2805_query_reg(int reg)
{
	return (instance->reg_map[reg] != -1) ? instance->reg_map[reg] : ssd2805_read_reg(reg);
}

#endif /* __IF_SSD2805_LOWLEVEL_H__ */
