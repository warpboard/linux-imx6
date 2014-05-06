/*
 * MAX77696 I2C Support
 *
 * Copyright (C) 2013 Maxim Integrated
 *
 * This file is part of MAX77696 PMIC Linux Driver
 *
 * MAX77696 PMIC Linux Driver is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * MAX77696 PMIC Linux Driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MAX77696 PMIC Linux Driver. If not, see http://www.gnu.org/licenses/.
 */

//#define DEBUG
//#define VERBOSE_DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>

#include <linux/mfd/max77696.h>

#define DRIVER_DESC    "MAX77696 I2C Support"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maxim-ic.com>"
#define DRIVER_NAME    MAX77696_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION

/* MAX77696 I2C Slave Addresses
 *   Real-Time Clock / Energy Harvester  W 0xD0  R 0xD1
 *   USB Interface Circuit               W 0x6A  R 0x6B
 *   Fuel Gauge                          W 0x68  R 0x69
 *   Power Management / GPIO/ other      W 0x78  R 0x79
 */
#define RTC_I2C_ADDR   (0xD0>>1)
#define UIC_I2C_ADDR   (0x6A>>1)
#define GAUGE_I2C_ADDR (0x68>>1)

extern int max77696_chip_init (struct max77696_chip *chip,
    struct max77696_platform_data* pdata);
extern void max77696_chip_exit (struct max77696_chip *chip);

/* for internal reference */
struct max77696_chip* max77696;

/* Reading from Sequential Registers */
static int max77696_i2c_seq_read (struct max77696_i2c *me,
    u8 addr, u8 *dst, u16 len)
{
    struct i2c_client *client = me->client;
    struct i2c_adapter *adap = client->adapter;
    struct i2c_msg msg[2];
    int rc;

    msg[0].addr   = client->addr;
    msg[0].flags  = client->flags & I2C_M_TEN;
    msg[0].len    = 1;
    msg[0].buf    = (char*)(&addr);

    msg[1].addr   = client->addr;
    msg[1].flags  = client->flags & I2C_M_TEN;
    msg[1].flags |= I2C_M_RD;
    msg[1].len    = len;
    msg[1].buf    = (char*)dst;

    rc = i2c_transfer(adap, msg, 2);

    /* If everything went ok (i.e. 2 msg transmitted), return 0,
       else error code. */
    return (rc == 2) ? 0 : rc;
}

/* Writing to Sequential Registers */
static int max77696_i2c_seq_write (struct max77696_i2c *me,
    u8 addr, const u8 *src, u16 len)
{
    struct i2c_client *client = me->client;
    struct i2c_adapter *adap = client->adapter;
    struct i2c_msg msg[1];
    u8 buf[len + 1];
    int rc;

    buf[0] = addr;
    memcpy(&buf[1], src, len);

    msg[0].addr  = client->addr;
    msg[0].flags = client->flags & I2C_M_TEN;
    msg[0].len   = len + 1;
    msg[0].buf   = (char*)buf;

    rc = i2c_transfer(adap, msg, 1);

    /* If everything went ok (i.e. 1 msg transmitted), return 0,
       else error code. */
    return (rc == 1) ? 0 : rc;
}

/* Writing Multiple Bytes Using Register-Data Pairs */
static int max77696_i2c_pair_write (struct max77696_i2c *me,
    u8 addr, const u8 *src, u16 len)
{
    struct i2c_client *client = me->client;
    struct i2c_adapter *adap = client->adapter;
    struct i2c_msg msg[1];
    u8 buf[len << 1];
    int i, j, rc;

    for (i = j = 0; i < len; i++) {
        buf[j++] = addr + i;
        buf[j++] = src[i];
    }

    msg[0].addr  = client->addr;
    msg[0].flags = client->flags & I2C_M_TEN;
    msg[0].len   = len << 1;
    msg[0].buf   = (char*)buf;

    rc = i2c_transfer(adap, msg, 1);

    /* If everything went ok (i.e. 1 msg transmitted), return 0,
       else error code. */
    return (rc == 1) ? 0 : rc;
}

/* Reading from a Single Register */
static int max77696_i2c_single_read (struct max77696_i2c *me, u8 addr, u8 *val)
{
    return max77696_i2c_seq_read(me, addr, val, 1);
}

/* Writing to a Single Register */
static int max77696_i2c_single_write (struct max77696_i2c *me, u8 addr, u8 val)
{
    struct i2c_client *client = me->client;
    struct i2c_adapter *adap = client->adapter;
    struct i2c_msg msg[1];
    u8 buf[2];
    int rc;

    buf[0] = (char)addr;
    buf[1] = (char)val;

    msg[0].addr  = client->addr;
    msg[0].flags = client->flags & I2C_M_TEN;
    msg[0].len   = 2;
    msg[0].buf   = (char*)buf;

    rc = i2c_transfer(adap, msg, 1);

    /* If everything went ok (i.e. 1 msg transmitted), return 0,
       else error code. */
    return (rc == 1) ? 0 : rc;
}

static struct i2c_board_info max77696_rtc_i2c_board_info = {
    I2C_BOARD_INFO(MAX77696_RTC_NAME, RTC_I2C_ADDR),
};

static struct i2c_board_info max77696_uic_i2c_board_info = {
    I2C_BOARD_INFO(MAX77696_UIC_NAME, UIC_I2C_ADDR),
};

static struct i2c_board_info max77696_gauge_i2c_board_info = {
    I2C_BOARD_INFO(MAX77696_GAUGE_NAME, GAUGE_I2C_ADDR),
};

static __devinit int max77696_i2c_probe (struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct max77696_platform_data* pdata = client->dev.platform_data;
    struct max77696_chip *chip;
    int rc;

    if (unlikely(!pdata)) {
        dev_err(&(client->dev), "platform data is missing\n");
        return -EINVAL;
    }

    chip = kzalloc(sizeof(*chip), GFP_KERNEL);
    if (unlikely(!chip)) {
        dev_err(&(client->dev),
            "out of memory (%uB requested)\n", sizeof(*chip));
        return -ENOMEM;
    }

    max77696 = chip;

    dev_set_drvdata(&(client->dev), chip);
    chip->dev  = &(client->dev);
    chip->kobj = &(client->dev.kobj);

    chip->core_irq = pdata->core_irq;
    chip->irq_base = pdata->irq_base;

    chip->pmic_i2c.client = client;
    i2c_set_clientdata(chip->pmic_i2c.client, chip);

    chip->pmic_i2c.read       = max77696_i2c_single_read;
    chip->pmic_i2c.write      = max77696_i2c_single_write;
    chip->pmic_i2c.bulk_read  = max77696_i2c_seq_read;
    chip->pmic_i2c.bulk_write = max77696_i2c_pair_write;

    chip->rtc_i2c.client = i2c_new_device(client->adapter,
        &max77696_rtc_i2c_board_info);
    if (unlikely(!chip->rtc_i2c.client)) {
        dev_err(chip->dev, "failed to create rtc i2c device\n");
        rc = -EIO;
        goto out_err;
    }

    i2c_set_clientdata(chip->rtc_i2c.client, chip);

    chip->rtc_i2c.read       = max77696_i2c_single_read;
    chip->rtc_i2c.write      = max77696_i2c_single_write;
    chip->rtc_i2c.bulk_read  = max77696_i2c_seq_read;
    chip->rtc_i2c.bulk_write = max77696_i2c_seq_write;

    chip->uic_i2c.client = i2c_new_device(client->adapter,
        &max77696_uic_i2c_board_info);
    if (unlikely(!chip->uic_i2c.client)) {
        dev_err(chip->dev, "failed to create uic i2c device\n");
        rc = -EIO;
        goto out_err;
    }

    i2c_set_clientdata(chip->uic_i2c.client, chip);

    chip->uic_i2c.read       = max77696_i2c_single_read;
    chip->uic_i2c.write      = max77696_i2c_single_write;
    chip->uic_i2c.bulk_read  = max77696_i2c_seq_read;
    chip->uic_i2c.bulk_write = max77696_i2c_seq_write;

    chip->gauge_i2c.client = i2c_new_device(client->adapter,
        &max77696_gauge_i2c_board_info);
    if (unlikely(!chip->gauge_i2c.client)) {
        dev_err(chip->dev, "failed to create gauge i2c device\n");
        rc = -EIO;
        goto out_err;
    }

    i2c_set_clientdata(chip->gauge_i2c.client, chip);

    chip->gauge_i2c.read       = max77696_i2c_single_read;
    chip->gauge_i2c.write      = max77696_i2c_single_write;
    chip->gauge_i2c.bulk_read  = max77696_i2c_seq_read;
    chip->gauge_i2c.bulk_write = max77696_i2c_seq_write;

    rc = max77696_chip_init(chip, pdata);
    if (unlikely(rc)) {
        goto out_err;
    }

    pr_info(DRIVER_DESC" "DRIVER_VERSION" Installed\n");
    return 0;

out_err:
    max77696 = NULL;
    if (likely(chip->gauge_i2c.client)) {
        i2c_unregister_device(chip->gauge_i2c.client);
    }
    if (likely(chip->uic_i2c.client)) {
        i2c_unregister_device(chip->uic_i2c.client);
    }
    if (likely(chip->rtc_i2c.client)) {
        i2c_unregister_device(chip->rtc_i2c.client);
    }
    i2c_set_clientdata(client, NULL);
    kfree(chip);
    return rc;
}

static __devexit int max77696_i2c_remove (struct i2c_client *client)
{
    struct max77696_chip *chip;

    chip = i2c_get_clientdata(client);
    i2c_set_clientdata(client, NULL);

    BUG_ON(chip != max77696);
    max77696 = NULL;

    if (likely(chip)) {
        max77696_chip_exit(chip);

        if (likely(chip->gauge_i2c.client)) {
            i2c_unregister_device(chip->gauge_i2c.client);
        }

        if (likely(chip->uic_i2c.client)) {
            i2c_unregister_device(chip->uic_i2c.client);
        }

        if (likely(chip->rtc_i2c.client)) {
            i2c_unregister_device(chip->rtc_i2c.client);
        }

        kfree(chip);
    }

    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_i2c_suspend (struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct max77696_chip *chip = i2c_get_clientdata(client);

    if (likely(chip->core_irq && device_may_wakeup(dev))) {
        enable_irq_wake(chip->core_irq);
    }
    return 0;
}

static int max77696_i2c_resume (struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct max77696_chip *chip = i2c_get_clientdata(client);

    if (likely(chip->core_irq && device_may_wakeup(dev))) {
        disable_irq_wake(chip->core_irq);
    }
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_i2c_pm,
    max77696_i2c_suspend, max77696_i2c_resume);

static const struct i2c_device_id max77696_i2c_ids[] = {
    { DRIVER_NAME, 0 },
    { },
};
MODULE_DEVICE_TABLE((me)->i2c, max77696_i2c_ids);

static struct i2c_driver max77696_i2c_driver = {
    .driver.name  = DRIVER_NAME,
    .driver.owner = THIS_MODULE,
    .driver.pm    = &max77696_i2c_pm,
    .probe        = max77696_i2c_probe,
    .remove       = __devexit_p(max77696_i2c_remove),
    .id_table     = max77696_i2c_ids,
};

static __init int max77696_i2c_driver_init (void)
{
    return i2c_add_driver(&max77696_i2c_driver);
}

static __exit void max77696_i2c_driver_exit (void)
{
    i2c_del_driver(&max77696_i2c_driver);
}

arch_initcall(max77696_i2c_driver_init);
module_exit(max77696_i2c_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);

