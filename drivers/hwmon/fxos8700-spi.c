/*
 *  fxos8700-spi.c - Linux kernel modules for FXOS8700 6-Axis Acc and Mag
 *  Combo Sensor spi driver
 *
 *  Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#include <linux/input-polldev.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include <linux/gpio.h>
#include <linux/delay.h>

#include "fxos8700.h"

#define MMA955XL_SPI_WRITE(reg)   	( reg | 0x80 )    // bit 7 is to be 1
#define MMA955XL_SPI_READ(reg)   	( reg & (~0x80)) // bit 7 is to be 0
static int fxos8700_spi_write_byte(struct fxos8700_data *pdata, u8 addr , u8 val)
{
	int ret = 0;
	char tmp_tx[16];

	struct spi_device * spi = (struct spi_device *)pdata->bus_priv;
	tmp_tx[0] = MMA955XL_SPI_WRITE(addr);
	tmp_tx[1] = addr;
	tmp_tx[2] = val;
	ret = spi_write(spi,tmp_tx,3);
	if(ret < 0){
		printk(KERN_ERR "%s ,write data error\n",__FUNCTION__);
		return -EINVAL;
	}
	return 0;

}
static int fxos8700_spi_read_byte (struct fxos8700_data *pdata, u8 addr)
{
	int ret = 0;
	u8 tmp_tx[2];
	u8 val;

	struct spi_device * spi = (struct spi_device *)pdata->bus_priv;
	tmp_tx[0] = MMA955XL_SPI_READ(addr);
	tmp_tx[1] = addr;
	ret = spi_write_then_read(spi,tmp_tx,2,&val,1);
	if(ret < 0){
		printk(KERN_ERR "%s ,read data error  ret %d\n",__FUNCTION__,ret);
		return -EINVAL;
	}
	return val;

}
static int fxos8700_spi_read_block (struct fxos8700_data *pdata, u8 addr,u8* buf, int len)
{
	int ret = 0;
	u8 tmp_tx[2];

	struct spi_device * spi = (struct spi_device *)pdata->bus_priv;
	tmp_tx[0] = MMA955XL_SPI_READ(addr);
	tmp_tx[1] = addr;
	ret = spi_write_then_read(spi,tmp_tx,2,buf,len);
	if(ret < 0){
		printk(KERN_ERR "%s ,read data error  ret %d\n",__FUNCTION__,ret);
		return -EINVAL;
	}
	return len;

}


static int fxos8700_spi_xfer(struct spi_device *spi,
			   u16 cmd, u8 count, u16 *tx_buf, u16 *rx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfers;
	void *spi_data;
	u16 *command;
	u16 *_rx_buf = _rx_buf; /* shut gcc up */
	u8 idx;
	int ret;

	xfers = spi_data = kzalloc(sizeof(*xfers) * (count + 2), GFP_KERNEL);
	if (!spi_data)
		return -ENOMEM;

	spi_message_init(&msg);

	command = spi_data;
	command[0] = cmd;
	if (count == 1) {
		command[1] = *tx_buf;
		tx_buf = &command[1];
		_rx_buf = rx_buf;
		rx_buf = &command[2];
	}

	++xfers;
	xfers[0].tx_buf = command;
	xfers[0].len = 2;
	spi_message_add_tail(&xfers[0], &msg);
	++xfers;

	for (idx = 0; idx < count; ++idx) {
		if (rx_buf)
			xfers[idx].rx_buf = &rx_buf[idx];
		if (tx_buf)
			xfers[idx].tx_buf = &tx_buf[idx];
		xfers[idx].len = 2;
		spi_message_add_tail(&xfers[idx], &msg);
	}

	ret = spi_sync(spi, &msg);

	if (count == 1)
		_rx_buf[0] = command[2];

	kfree(spi_data);

	return ret;
}

static int fxos8700_spi_read(struct spi_device *spi, u8 reg)
{
	u16 ret = 0;
	u16 dummy = 0;

	return fxos8700_spi_xfer(spi, MMA955XL_SPI_READ(reg), 1, &dummy, &ret) ? : ret;
}

// this function reads "count" bytes from the "reg" register to "dst"
static uint8_t spi_read_reg_burst(struct spi_device *spi, uint8_t reg, uint8_t *dst, size_t count)
{
	struct spi_transfer t[2];
	struct spi_message m;

	spi_message_init(&m);

	memset(t, 0, sizeof(t));

	t[0].tx_buf = &reg;
	t[0].len = 1;
	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = (uint8_t *)dst;
	t[1].len = count;
	spi_message_add_tail(&t[1], &m);

	spi_sync(spi, &m);

	return 0;
}

// this wrapper function returns the value of register "reg"
static uint8_t spi_read_reg(struct spi_device *spi, uint8_t reg)
{
	uint8_t res;

	if (spi_read_reg_burst(spi, reg, &res, 1) == 0)
		return res;

	return 0;
}


static int fxos8700_spi_probe(struct spi_device *spi)
{
	int ret;
	unsigned int val;

	gpio_request(83, "FXOS8700-RST");//GPIO3_19 -> ( (3-1) * 32) + 19 = 83
	gpio_direction_output(83, 0);//Reset


	spi->mode = SPI_MODE_0;
	ret = spi_setup(spi);
	if(ret)
		return ret;
	fxos8700_dev.bus_priv = spi;
	fxos8700_dev.bus_type = BUS_SPI;
	fxos8700_dev.write_byte = fxos8700_spi_write_byte;
	fxos8700_dev.read_byte= fxos8700_spi_read_byte;
	fxos8700_dev.read_block = fxos8700_spi_read_block;
	spi_set_drvdata(spi, &fxos8700_dev);

//Perform a dummy read
	val = fxos8700_spi_read_byte(&fxos8700_dev, FXOS8700_WHO_AM_I);
	printk("FXOS8700_WHO_AM_I val 0x%x\n",val);

	return fxos8700_device_init(&fxos8700_dev);
}
static int fxos8700_spi_remove(struct spi_device *client)
{
	return fxos8700_device_remove(&fxos8700_dev);
}

static const struct of_device_id fxos8700_dt_ids[] = {
	{ .compatible = "nxp,fxos8700"},
	{ }
};
MODULE_DEVICE_TABLE(of, fxos8700_dt_ids);


static struct spi_driver fxos8700_spi_driver = {
	.driver	 = {
		.name   = "fxos8700",
		.owner  = THIS_MODULE,	
		.of_match_table = of_match_ptr(fxos8700_dt_ids),
	},
	.probe   = fxos8700_spi_probe,
	.remove  = fxos8700_spi_remove,
};

static int __init fxos8700_spi_init(void)
{
	/* register driver */
	int res;
	res = spi_register_driver(&fxos8700_spi_driver);
	if (res < 0) {
		printk(KERN_INFO "add fxos8700 spi driver failed\n");
		return -ENODEV;
	}
	return 0;
}

static void __exit fxos8700_spi_exit(void)
{
	spi_unregister_driver(&fxos8700_spi_driver);
}


module_init(fxos8700_spi_init);
module_exit(fxos8700_spi_exit);

MODULE_DESCRIPTION("fxos8700 3-Axis Smart Orientation and Motion Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");
