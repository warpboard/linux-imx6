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

static int __devinit fxos8700_spi_probe(struct spi_device *spi)
{
	int ret;
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
	return fxos8700_device_init(&fxos8700_dev);
}
static int __devexit fxos8700_spi_remove(struct spi_device *client)
{
	return fxos8700_device_remove(&fxos8700_dev);
}

#ifdef CONFIG_PM
static int fxos8700_spi_suspend(struct spi_device *spi, pm_message_t mesg)
{
	return fxos8700_suspend(spi_get_drvdata(spi));
}

static int lfxos8700_spi_resume(struct spi_device *spi)
{
	return fxos8700_resume(spi_get_drvdata(spi));
}

#else
#define fxos8700_spi_suspend	NULL
#define lfxos8700_spi_resume	NULL
#endif

static struct spi_driver fxos8700_spi_driver = {
	.driver	 = {
		.name   = "fxos8700",
		.owner  = THIS_MODULE,	},
	.probe   = fxos8700_spi_probe,
	.remove  = __devexit_p(fxos8700_spi_remove),
	.suspend = fxos8700_spi_suspend,
	.resume  = lfxos8700_spi_resume,
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
