/*
 * MAX77696 ONKEY Driver
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

#include <linux/input.h>
#include <linux/mfd/max77696.h>

#define DRIVER_DESC    "MAX77696 ONKEY Driver"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maxim-ic.com>"
#define DRIVER_NAME    MAX77696_ONKEY_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"

#define KEY_DOWN_VALUE 1
#define KEY_UP_VALUE   0

enum {
    ONKEY_KEY_EN0 = 0,
    ONKEY_KEY_1SEC,
    ONKEY_KEY_MRWRN,

    ONKEY_NR_KEYS,
};

struct max77696_onkey {
    struct mutex          lock;
    struct max77696_chip *chip;
    struct device        *dev;

    /* filled from pdata */
    unsigned int          keycode[ONKEY_NR_KEYS];

    struct input_dev     *input;
    unsigned int          dben0_irq;
    unsigned int          irq[ONKEY_NR_KEYS]; /* key_up interrupts */
    bool                  pressed[ONKEY_NR_KEYS];
};

static irqreturn_t max77696_onkey_down_isr (int irq, void *data)
{
    struct max77696_onkey *me = data;
    int i;

    for (i = 0; i < ONKEY_NR_KEYS; i++) {
        if (likely(me->keycode[i] < KEY_MAX)) {
            me->pressed[i] = 1;
            input_report_key(me->input, me->keycode[i], KEY_DOWN_VALUE);
            input_sync(me->input);
        }
    }

    return IRQ_HANDLED;
}

static irqreturn_t max77696_onkey_1sec_isr (int irq, void *data)
{
    struct max77696_onkey *me = data;

    if (likely(me->keycode[ONKEY_KEY_1SEC] < KEY_MAX)) {
        me->pressed[ONKEY_KEY_1SEC] = 0;
        input_report_key(me->input, me->keycode[ONKEY_KEY_1SEC], KEY_UP_VALUE);
        input_sync(me->input);
    }

    return IRQ_HANDLED;
}

static irqreturn_t max77696_onkey_mrwrn_isr (int irq, void *data)
{
    struct max77696_onkey *me = data;

    if (likely(me->keycode[ONKEY_KEY_MRWRN] < KEY_MAX)) {
        me->pressed[ONKEY_KEY_MRWRN] = 0;
        input_report_key(me->input, me->keycode[ONKEY_KEY_MRWRN], KEY_UP_VALUE);
        input_sync(me->input);
    }

    return IRQ_HANDLED;
}

static irqreturn_t max77696_onkey_up_isr (int irq, void *data)
{
    struct max77696_onkey *me = data;
    int i;

    for (i = ONKEY_NR_KEYS-1; i >= 0; i--) {
        if (likely(me->keycode[i] < KEY_MAX && likely(me->pressed[i]))) {
            me->pressed[i] = 0;
            input_report_key(me->input, me->keycode[i], KEY_UP_VALUE);
            input_sync(me->input);
        }
    }

    return IRQ_HANDLED;
}

static __devinit int max77696_onkey_probe (struct platform_device *pdev)
{
    struct max77696_chip *chip = dev_get_drvdata(pdev->dev.parent);
    struct max77696_onkey_platform_data *pdata = pdev->dev.platform_data;
    struct max77696_onkey *me;
    int i, rc;

    if (unlikely(!pdata)) {
        dev_err(&(pdev->dev), "platform data is missing\n");
        return -EINVAL;
    }

    me = kzalloc(sizeof(*me), GFP_KERNEL);
    if (unlikely(!me)) {
        dev_err(&(pdev->dev), "out of memory (%uB requested)\n", sizeof(*me));
        return -ENOMEM;
    }

    platform_set_drvdata(pdev, me);

    mutex_init(&(me->lock));
    me->chip = chip;
    me->dev  = &(pdev->dev);

    me->keycode[ONKEY_KEY_EN0  ] = pdata->onkey_keycode;
    me->keycode[ONKEY_KEY_1SEC ] = pdata->hold_1sec_keycode;
    me->keycode[ONKEY_KEY_MRWRN] = pdata->mr_warn_keycode;

    me->dben0_irq = max77696_topsysint_to_irq(chip, EN0_FALLING);

    me->irq[ONKEY_KEY_EN0  ] = max77696_topsysint_to_irq(chip, EN0_RISING);
    me->irq[ONKEY_KEY_1SEC ] = max77696_topsysint_to_irq(chip, EN0_1SEC);
    me->irq[ONKEY_KEY_MRWRN] = max77696_topsysint_to_irq(chip, MR_WARNING);

    me->input = input_allocate_device();
    if (unlikely(!me->input)) {
        rc = -ENOMEM;
        dev_err(me->dev, "failed to allocate input device [%d]\n", rc);
        goto out_err_alloc_input;
    }

    for (i = 0; i < ONKEY_NR_KEYS; i++) {
        if (likely(me->keycode[i] < KEY_MAX)) {
            input_set_capability(me->input, EV_KEY, me->keycode[i]);
        }
    }

    me->input->name       = DRIVER_NAME;
    me->input->phys       = DRIVER_NAME"/input0";
    me->input->dev.parent = me->dev;

    rc = input_register_device(me->input);
    if (unlikely(rc)) {
        dev_err(me->dev, "failed to register input device [%d]\n", rc);
        goto out_err_reg_input;
    }

    /* Initial configurations */
    max77696_topsys_enable_en0_delay(pdata->wakeup_1sec_delayed_since_onkey_down);
    max77696_topsys_enable_mr_wakeup(pdata->wakeup_after_mro);
    max77696_topsys_set_mr_time(pdata->manual_reset_time);
    max77696_topsys_enable_mr(pdata->manual_reset_time > 0);

    rc = request_threaded_irq(me->irq[ONKEY_KEY_MRWRN],
        NULL, max77696_onkey_mrwrn_isr, IRQF_ONESHOT, DRIVER_NAME".mrwrn", me);

    if (unlikely(rc < 0)) {
        dev_err(me->dev, "failed to request IRQ(%d) [%d]\n",
            me->irq[ONKEY_KEY_MRWRN], rc);
        goto out_err_req_mr_wrn_irq;
    }

    rc = request_threaded_irq(me->irq[ONKEY_KEY_1SEC],
        NULL, max77696_onkey_1sec_isr, IRQF_ONESHOT, DRIVER_NAME".1sec", me);

    if (unlikely(rc < 0)) {
        dev_err(me->dev, "failed to request IRQ(%d) [%d]\n",
            me->irq[ONKEY_KEY_1SEC], rc);
        goto out_err_req_en0_1sec_irq;
    }

    rc = request_threaded_irq(me->irq[ONKEY_KEY_EN0],
        NULL, max77696_onkey_up_isr, IRQF_ONESHOT, DRIVER_NAME".en0up", me);

    if (unlikely(rc < 0)) {
        dev_err(me->dev, "failed to request IRQ(%d) [%d]\n",
            me->irq[ONKEY_KEY_EN0], rc);
        goto out_err_req_en0_up_irq;
    }

    rc = request_threaded_irq(me->dben0_irq,
        NULL, max77696_onkey_down_isr, IRQF_ONESHOT, DRIVER_NAME".en0down", me);

    if (unlikely(rc < 0)) {
        dev_err(me->dev, "failed to request IRQ(%d) [%d]\n",
            me->dben0_irq, rc);
        goto out_err_req_en0_down_irq;
    }

    pr_info(DRIVER_DESC" "DRIVER_VERSION" Installed\n");
    return 0;

out_err_req_en0_down_irq:
    free_irq(me->irq[ONKEY_KEY_EN0], me);
out_err_req_en0_up_irq:
    free_irq(me->irq[ONKEY_KEY_1SEC], me);
out_err_req_en0_1sec_irq:
    free_irq(me->irq[ONKEY_KEY_MRWRN], me);
out_err_req_mr_wrn_irq:
    input_unregister_device(me->input);
    me->input = NULL;
out_err_reg_input:
    input_free_device(me->input);
out_err_alloc_input:
    mutex_destroy(&(me->lock));
    platform_set_drvdata(pdev, NULL);
    kfree(me);
    return rc;
}

static __devexit int max77696_onkey_remove (struct platform_device *pdev)
{
    return 0;
}

static void max77696_onkey_shutdown (struct platform_device *pdev)
{
    struct max77696_onkey *me = platform_get_drvdata(pdev);
    int i;

    free_irq(me->dben0_irq, me);

    for (i = 0; i < ONKEY_NR_KEYS; i++) {
        if (likely(me->irq[i] > 0)) {
            free_irq(me->irq[i], me);
        }
    }

    input_unregister_device(me->input);

    mutex_destroy(&(me->lock));
    platform_set_drvdata(pdev, NULL);
    kfree(me);
}

static struct platform_driver max77696_onkey_driver = {
    .driver.name  = DRIVER_NAME,
    .driver.owner = THIS_MODULE,
    .probe        = max77696_onkey_probe,
    .remove       = __devexit_p(max77696_onkey_remove),
    .shutdown     = max77696_onkey_shutdown,
};

static __init int max77696_onkey_driver_init (void)
{
    return platform_driver_register(&max77696_onkey_driver);
}

static __exit void max77696_onkey_driver_exit (void)
{
    platform_driver_unregister(&max77696_onkey_driver);
}

module_init(max77696_onkey_driver_init);
module_exit(max77696_onkey_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);

