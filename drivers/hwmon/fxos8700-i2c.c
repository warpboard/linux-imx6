/*
 *  fxos8700-i2c.c - Linux kernel modules for FXOS8700 6-Axis Acc and Mag
 *  Combo Sensor i2c driver
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>
#include "fxos8700.h"

static int fxos8700_i2c_write_byte(struct fxos8700_data *pdata, u8 addr,u8 val){
	struct i2c_client * client = (struct i2c_client *)pdata->bus_priv;
	return i2c_smbus_write_byte_data(client,addr,val);
}
static int fxos8700_i2c_read_byte(struct fxos8700_data *pdata, u8 addr){
	struct i2c_client * client = (struct i2c_client *)pdata->bus_priv;
	return i2c_smbus_read_byte_data(client,addr);
}
static int fxos8700_i2c_read_block(struct fxos8700_data *pdata, u8 addr,u8* buf, int len){
	struct i2c_client * client = (struct i2c_client *)pdata->bus_priv;
	return i2c_smbus_read_i2c_block_data(client,addr,len,buf);
}

static int __devinit fxos8700_i2c_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	fxos8700_dev.bus_priv = client;
	fxos8700_dev.bus_type = BUS_I2C;
	fxos8700_dev.read_byte= fxos8700_i2c_read_byte;
	fxos8700_dev.write_byte = fxos8700_i2c_write_byte;
	fxos8700_dev.read_block = fxos8700_i2c_read_block;
	i2c_set_clientdata(client,&fxos8700_dev);
	return fxos8700_device_init(&fxos8700_dev);
}
static int __devexit fxos8700_i2c_remove(struct i2c_client *client)
{
	return fxos8700_device_remove(&fxos8700_dev);
}
#ifdef CONFIG_PM
static int fxos8700_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{

	return fxos8700_suspend(i2c_get_clientdata(client));
}

static int fxos8700_i2c_resume(struct i2c_client *client)
{
	return fxos8700_resume(i2c_get_clientdata(client));
}

#else
#define fxos8700_i2c_suspend	NULL
#define fxos8700_i2c_resume	NULL
#endif

static const struct i2c_device_id fxos8700_i2c_id[] = {
	{"fxos8700", 0},
	{ }
};

static struct i2c_driver fxos8700_i2c_driver = {
	.driver = {
		   .name = "fxos8700",
		   .owner = THIS_MODULE,
		   },
	.suspend = fxos8700_i2c_suspend,
	.resume = fxos8700_i2c_resume,
	.probe = fxos8700_i2c_probe,
	.remove = __devexit_p(fxos8700_i2c_remove),
	.id_table = fxos8700_i2c_id,
};

static int __init fxos8700_i2c_init(void)
{
	/* register driver */
	int res;
	res = i2c_add_driver(&fxos8700_i2c_driver);
	if (res < 0) {
		printk(KERN_INFO "add fxos8700 i2c driver failed\n");
		return -ENODEV;
	}
	return res;
}

static void __exit fxos8700_i2c_exit(void)
{
	i2c_del_driver(&fxos8700_i2c_driver);
}


module_init(fxos8700_i2c_init);
module_exit(fxos8700_i2c_exit);
