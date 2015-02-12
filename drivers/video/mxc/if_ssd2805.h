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

#ifndef __LCD_MIPID_H
#define __LCD_MIPID_H

int ssd2805_init(void);

int ssd2805_setup_commands(int pll_freq);
int ssd2805_setup_host(void);

void ssd2805_setup_fbxfer(int cmd, int wlen);

int mipi_dcs_command(int cmd);
int mipi_dcs_write(int cmd, u8 *wbuf, int wlen);
int mipi_dcs_read(int cmd, u8 *rbuf, int rlen);

extern void __iomem *elcdif_base;

#endif
