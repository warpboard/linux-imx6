/*
 * MAX77696 TOPSYS Interface
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

#include <linux/irq.h>
#include <linux/mfd/max77696.h>

#define DRIVER_DESC    "MAX77696 TOPSYS Interface"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maxim-ic.com>"
#define DRIVER_NAME    MAX77696_TOPSYS_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION

#define GLBLCNFG0_REG                    0x00

#define GLBLCNFG1_REG                    0x01
//      RESERVED                         BIT (7)
#define GLBLCNFG1_GLBL_LPM_MASK          BIT (6)
#define GLBLCNFG1_GLBL_LPM_SHIFT         6
#define GLBLCNFG1_MREN_MASK              BIT (5)
#define GLBLCNFG1_MREN_SHIFT             5
#define GLBLCNFG1_MRT_MASK               BITS(4,2)
#define GLBLCNFG1_MRT_SHIFT              2
#define GLBLCNFG1_EN0DLY_MASK            BIT (1)
#define GLBLCNFG1_EN0DLY_SHIFT           1
#define GLBLCNFG1_STBYEN_MASK            BIT (0)
#define GLBLCNFG1_STBYEN_SHIFT           0

#define GLBLCNFG2_REG                    0x02
//      RESERVED                         BIT (7)
#define GLBLCNFG2_WDTEN_MASK             BIT (6)
#define GLBLCNFG2_WDTEN_SHIFT            6
#define GLBLCNFG2_TWD_MASK               BITS(5,4)
#define GLBLCNFG2_TWD_SHIFT              4
#define GLBLCNFG2_RTCAWK_MASK            BIT (3)
#define GLBLCNFG2_RTCAWK_SHIFT           3
#define GLBLCNFG2_WDWK_MASK              BIT (2)
#define GLBLCNFG2_WDWK_SHIFT             2
#define GLBLCNFG2_MROWK_MASK             BIT (1)
#define GLBLCNFG2_MROWK_SHIFT            1
#define GLBLCNFG2_UICWK_EDGE_MASK        BIT (0)
#define GLBLCNFG2_UICWK_EDGE_SHIFT       0

#define GLBLCNFG3_REG                    0x03
//      RESERVED                         BITS(7,5)
#define GLBLCNFG3_LBHYST_MASK            BITS(4,3)
#define GLBLCNFG3_LBHYST_SHIFT           3
#define GLBLCNFG3_LBDAC_MASK             BITS(2,0)
#define GLBLCNFG3_LBDAC_SHIFT            0

#define GLBLCNFG4_REG                    0x04
//      RESERVED                         BITS(7,2)
#define GLBLCNFG4_WDTC_MASK              BITS(1,0)
#define GLBLCNFG4_WDTC_SHIFT             0

#define GLBLCNFG5_REG                    0xAF
//      RESERVED                         BITS(7,2)
#define GLBLCNFG5_NRSO_DEL_MASK          BITS(1,0)
#define GLBLCNFG5_NRSO_DEL_SHIFT         0

#define GLBLINT_REG                      0x05
#define GLBLINTM_REG                     0x06

#define GLBLSTAT_REG                     0x07
#define GLBLSTAT_EN0_S_MASK              BIT (7)
#define GLBLSTAT_EN0_S_SHIFT             7
#define GLBLSTAT_MBALLLOW_MASK           BIT (3)
#define GLBLSTAT_MBALLLOW_SHIFT          3
#define GLBLSTAT_TJALRM1_MASK            BIT (2)
#define GLBLSTAT_TJALRM1_SHIFT           2
#define GLBLSTAT_TJALRM2_MASK            BIT (1)
#define GLBLSTAT_TJALRM2_SHIFT           1
#define GLBLSTAT_IRQ_MASK                BIT (0)
#define GLBLSTAT_IRQ_SHIFT               0

#define GLBLINT_EN0_R                    BIT (7)
#define GLBLINT_EN0_F                    BIT (6)
#define GLBLINT_EN0_1SEC                 BIT (5)
#define GLBLINT_MRWRN                    BIT (4)
#define GLBLINT_MBATTLOW_R               BIT (3)
#define GLBLINT_TJALRM1_R                BIT (2)
#define GLBLINT_TJALRM2_R                BIT (1)
#define GLBLINT_IRQ                      BIT (0)

#define __get_i2c(chip)                  (&((chip)->pmic_i2c))
#define __lock(me)                       mutex_lock(&((me)->lock))
#define __unlock(me)                     mutex_unlock(&((me)->lock))

#define TOPSYS_REG(reg)                  ((u8)(reg##_REG))
#define TOPSYS_REG_BITMASK(reg, bits)    ((u8)(reg##_##bits##_MASK))
#define TOPSYS_REG_BITSHIFT(reg, bits)        (reg##_##bits##_SHIFT)

#define TOPSYS_REG_BITGET(reg, bit, val) \
        ((val) >> TOPSYS_REG_BITSHIFT(reg, bit))
#define TOPSYS_REG_BITSET(reg, bit, val) \
        ((val) << TOPSYS_REG_BITSHIFT(reg, bit))

/* TOPSYS Register Read/Write */
#define max77696_topsys_reg_read(me, reg, val_ptr) \
        max77696_read((me)->i2c, TOPSYS_REG(reg), val_ptr)
#define max77696_topsys_reg_write(me, reg, val) \
        max77696_write((me)->i2c, TOPSYS_REG(reg), val)
#define max77696_topsys_reg_bulk_read(me, reg, dst, len) \
        max77696_bulk_read((me)->i2c, TOPSYS_REG(reg), dst, len)
#define max77696_topsys_reg_bulk_write(me, reg, src, len) \
        max77696_bulk_write((me)->i2c, TOPSYS_REG(reg), src, len)
#define max77696_topsys_reg_read_masked(me, reg, mask, val_ptr) \
        max77696_read_masked((me)->i2c, TOPSYS_REG(reg), mask, val_ptr)
#define max77696_topsys_reg_write_masked(me, reg, mask, val) \
        max77696_write_masked((me)->i2c, TOPSYS_REG(reg), mask, val)

/* TOPSYS Register Single Bit Ops */
#define max77696_topsys_reg_get_bit(me, reg, bit, val_ptr) \
        ({\
            int __rc = max77696_topsys_reg_read_masked(me, reg,\
                TOPSYS_REG_BITMASK(reg, bit), val_ptr);\
            *(val_ptr) = TOPSYS_REG_BITGET(reg, bit, *(val_ptr));\
            __rc;\
        })
#define max77696_topsys_reg_set_bit(me, reg, bit, val) \
        ({\
            max77696_topsys_reg_write_masked(me, reg,\
                TOPSYS_REG_BITMASK(reg, bit),\
                TOPSYS_REG_BITSET(reg, bit, val));\
        })

#define max77696_topsys_global_config(me, cfg_reg, cfg_item, cfg_val) \
        ({\
            int __rc;\
            __lock(me);\
            __rc = max77696_topsys_reg_set_bit(me,\
                cfg_reg, cfg_item, (u8)(cfg_val));\
            __unlock(me);\
            if (unlikely(__rc)) {\
                dev_err(me->dev, ""#cfg_reg" write error [%d]\n", __rc);\
            }\
            __rc;\
        })

#define NR_TOPSYS_IRQ                  MAX77696_TOPSYSINT_NR_IRQS

struct max77696_topsys {
    struct mutex          lock;
    struct max77696_chip *chip;
    struct max77696_i2c  *i2c;
    struct device        *dev;
    struct kobject       *kobj;

    unsigned int          top_irq;
    unsigned int          irq_base;

    u8                    irq_unmask_new;
    u8                    irq_unmask_curr;
    u8                    irq_wakeup_bitmap;
};

static void max77696_topsys_irq_mask (struct irq_data *data)
{
    struct max77696_topsys *me = irq_data_get_irq_chip_data(data);
    u8 irq_bit = (u8)(1 << (data->irq - me->irq_base + 1));

    if (unlikely(!(me->irq_unmask_new & irq_bit))) {
        return;
    }

    me->irq_unmask_new &= ~irq_bit;

    if (unlikely(me->irq_unmask_new == GLBLINT_IRQ)) {
        me->irq_unmask_new = 0;
    }
}

static void max77696_topsys_irq_unmask (struct irq_data *data)
{
    struct max77696_topsys *me = irq_data_get_irq_chip_data(data);
    u8 irq_bit = (u8)(1 << (data->irq - me->irq_base + 1));

    if (unlikely((me->irq_unmask_new & irq_bit))) {
        return;
    }

    if (unlikely(!me->irq_unmask_new)) {
        me->irq_unmask_new = GLBLINT_IRQ;
    }

    me->irq_unmask_new |= irq_bit;
}

static void max77696_topsys_irq_bus_lock (struct irq_data *data)
{
    struct max77696_topsys *me = irq_data_get_irq_chip_data(data);

    __lock(me);
}

/*
 * genirq core code can issue chip->mask/unmask from atomic context.
 * This doesn't work for slow busses where an access needs to sleep.
 * bus_sync_unlock() is therefore called outside the atomic context,
 * syncs the current irq mask state with the slow external controller
 * and unlocks the bus.
 */

static void max77696_topsys_irq_bus_sync_unlock (struct irq_data *data)
{
    struct max77696_topsys *me = irq_data_get_irq_chip_data(data);
    int rc;

    if (unlikely(me->irq_unmask_new == me->irq_unmask_curr)) {
        goto out;
    }

    disable_irq(me->top_irq);

    rc = max77696_write(me->i2c, GLBLINTM_REG, ~(me->irq_unmask_new));
    if (unlikely(rc)) {
        dev_err(me->dev, "GLBLINTM_REG write error [%d]\n", rc);
        goto out;
    }

    me->irq_unmask_curr = me->irq_unmask_new;

    if (likely(me->irq_unmask_new)) {
        enable_irq(me->top_irq);
    }

out:
    __unlock(me);
}

static int max77696_topsys_irq_set_type (struct irq_data *data,
        unsigned int type)
{
    struct max77696_topsys *me = irq_data_get_irq_chip_data(data);

    if (unlikely(type & ~(IRQ_TYPE_EDGE_BOTH | IRQ_TYPE_LEVEL_MASK))) {
        dev_err(me->dev, "unsupported irq type %d\n", type);
        return -EINVAL;
    }

    return 0;
}

static int max77696_topsys_irq_set_wake (struct irq_data *data, unsigned int on)
{
    struct max77696_topsys *me = irq_data_get_irq_chip_data(data);
    u8 irq_bit = (u8)(1 << (data->irq - me->irq_base + 1));

    if (on) {
        if (unlikely(!me->irq_wakeup_bitmap)) {
            enable_irq_wake(me->top_irq);
        }
        me->irq_wakeup_bitmap |=  irq_bit;
    } else {
        me->irq_wakeup_bitmap &= ~irq_bit;
        if (unlikely(!me->irq_wakeup_bitmap)) {
            disable_irq_wake(me->top_irq);
        }
    }

    return 0;
}

/* IRQ chip operations
 */
static struct irq_chip max77696_topsys_irq_chip = {
    .name                = DRIVER_NAME,
//  .flags               = IRQCHIP_SET_TYPE_MASKED,
    .irq_mask            = max77696_topsys_irq_mask,
    .irq_unmask          = max77696_topsys_irq_unmask,
    .irq_bus_lock        = max77696_topsys_irq_bus_lock,
    .irq_bus_sync_unlock = max77696_topsys_irq_bus_sync_unlock,
    .irq_set_type        = max77696_topsys_irq_set_type,
    .irq_set_wake        = max77696_topsys_irq_set_wake,
};

static const char *max77696_topsys_irq_names[] = {
    [MAX77696_TOPSYSINT_THERM_ALARM_1] = "THERM_ALARM_1",
    [MAX77696_TOPSYSINT_THERM_ALARM_0] = "THERM_ALARM_0",
    [MAX77696_TOPSYSINT_BATT_LOW]      = "BATT_LOW",
    [MAX77696_TOPSYSINT_MR_WARNING]    = "MR_WARNING",
    [MAX77696_TOPSYSINT_EN0_1SEC]      = "EN0_1SEC",
    [MAX77696_TOPSYSINT_EN0_FALLING]   = "EN0_FALLING",
    [MAX77696_TOPSYSINT_EN0_RISING]    = "EN0_RISING",
};

static irqreturn_t max77696_topsys_isr (int irq, void *data)
{
    struct max77696_topsys *me = data;
    u8 interrupted;
    int i;

    max77696_read(me->i2c, GLBLINT_REG, &interrupted);
    dev_dbg(me->dev, "GLBLINT %02X\n", interrupted);

    for (i = 0; i < NR_TOPSYS_IRQ; i++) {
        u8 irq_bit = (1 << (i + 1));

        if (unlikely(!(me->irq_unmask_new & irq_bit))) {
            /* if the irq is disabled, then ignore a below process */
            continue;
        }

        if (likely(interrupted & irq_bit)) {
            dev_dbg(me->dev, "handle_nested_irq %s(%d)\n",
                max77696_topsys_irq_names[i], me->irq_base + i);
            handle_nested_irq(me->irq_base + i);
        }
    }

    return IRQ_HANDLED;
}

/*** TOPSYS Low-Battery Monitor ***/

/* Set Low-Battery Comparator Hysteresis */
static int max77696_topsys_mbattlow_set_lbhyst (struct max77696_topsys *me,
    unsigned int mV)
{
    u8 lbhyst;

    if (mV > 400) {
        lbhyst = 3;
    } else if (mV > 300) {
        lbhyst = 2;
    } else if (mV > 200) {
        lbhyst = 1;
    } else {
        lbhyst = 0;
    }

    return max77696_topsys_global_config(me, GLBLCNFG3, LBHYST, lbhyst);
}

/* Set Low-Battery DAC Falling Threshold */
static int max77696_topsys_mbattlow_set_lbdac (struct max77696_topsys *me,
    unsigned int mV)
{
    u8 lbdac;

    if (mV > 3300) {
        lbdac = 7;
    } else if (mV > 3200) {
        lbdac = 6;
    } else if (mV > 3100) {
        lbdac = 5;
    } else if (mV > 3000) {
        lbdac = 4;
    } else if (mV > 2900) {
        lbdac = 3;
    } else if (mV > 2800) {
        lbdac = 2;
    } else if (mV > 2700) {
        lbdac = 1;
    } else {
        lbdac = 0;
    }

    return max77696_topsys_global_config(me, GLBLCNFG3, LBDAC, lbdac);
}

static irqreturn_t max77696_topsys_mbattlow_isr (int irq, void *data)
{
    struct max77696_topsys *me = data;
    struct power_supply *psy;
    u8 mbattlow;

    max77696_topsys_reg_get_bit(me, GLBLSTAT, MBALLLOW, &mbattlow);
    dev_dbg(me->dev, "MBATTLOW Status VMBATT %s VMBATL\n",
        mbattlow? "<" : ">");

    psy = power_supply_get_by_name(MAX77696_PSY_BATT_NAME);
    if (likely(psy)) {
        power_supply_changed(psy);
    }

    return IRQ_HANDLED;
}

static int max77696_topsys_mbattlow_setup (struct max77696_topsys *me,
    struct max77696_platform_data *pdata)
{
    unsigned int irq, lbdac, lbhyst;
    int rc;

    irq    = max77696_topsysint_to_irq(me->chip, BATT_LOW);
    lbdac  = pdata->core_mbattlow_falling_threshold_mV;
    lbhyst = pdata->core_mbattlow_comparator_hysteresis_mV;

    rc = max77696_topsys_mbattlow_set_lbdac(me, lbdac);
    if (unlikely(rc)) {
        dev_err(me->dev, "failed to set LBDAC [%d]\n", rc);
        goto out;
    }

    rc = max77696_topsys_mbattlow_set_lbhyst(me, lbhyst);
    if (unlikely(rc)) {
        dev_err(me->dev, "failed to set LBHYST [%d]\n", rc);
        goto out;
    }

    rc = request_threaded_irq(irq, NULL, max77696_topsys_mbattlow_isr,
        IRQF_ONESHOT, DRIVER_NAME".battlow", me);

    if (unlikely(rc < 0)) {
        dev_err(me->dev, "failed to request IRQ(%d) [%d]\n", irq, rc);
        goto out;
    }

    dev_info(me->dev, "Low-Battery DAC Falling Threshold %4umV\n", lbdac);
    dev_info(me->dev, "Low-Battery Comparator Hysteresis %4umV\n", lbhyst);

out:
    return rc;
}

/*** TOPSYS Init/Exit ***/

int max77696_topsys_init (struct max77696_chip *chip,
    struct max77696_platform_data* pdata)
{
    struct max77696_topsys *me;
    u8 interrupted;
    int i, rc;

    pr_info(DRIVER_DESC" "DRIVER_VERSION"\n");

    me = kzalloc(sizeof(*me), GFP_KERNEL);
    if (unlikely(!me)) {
        dev_err(chip->dev, "out of memory (%uB requested)\n", sizeof(*me));
        return -ENOMEM;
    }

    mutex_init(&(me->lock));
    me->chip = chip;
    me->i2c  = __get_i2c(chip);
    me->dev  = chip->dev;
    me->kobj = &(chip->dev->kobj);

    me->top_irq  = max77696_rootint_to_irq(chip, TOPSYS);
    me->irq_base = max77696_topsysint_irq_base(chip);

    /* Disable all TOPSYS interrupts */
    max77696_write(me->i2c, GLBLINTM_REG, 0xFF);

    /* Clear TOPSYS interrupt status */

    max77696_read(me->i2c, GLBLINT_REG, &interrupted);

    dev_dbg(me->dev, "initial TOPSYS interrupt status: %02X\n", interrupted);
    for (i = 0; i < NR_TOPSYS_IRQ; i++) {
        dev_dbg(me->dev, "    %-13s  %s\n",
            max77696_topsys_irq_names[i],
            (interrupted & (1 << (i + 1)))? "o" : "-");
    }

    /* Register all TOPSYS interrupts */

    for (i = 0; i < NR_TOPSYS_IRQ; i++) {
        unsigned int irq = me->irq_base + i;

        irq_set_chip_data       (irq, me);
        irq_set_chip_and_handler(irq, &max77696_topsys_irq_chip, handle_simple_irq);
        irq_set_nested_thread   (irq, 1);

#ifdef CONFIG_ARM
        /*
         * ARM needs us to explicitly flag the IRQ as VALID,
         * once we do so, it will also set the noprobe.
         */
        set_irq_flags  (irq, IRQF_VALID);
#else
        irq_set_noprobe(irq);
#endif
    }

    rc = request_threaded_irq(me->top_irq,
        NULL, max77696_topsys_isr, IRQF_ONESHOT, DRIVER_NAME, me);

    if (unlikely(rc < 0)) {
        dev_err(me->dev,
            "failed to request IRQ(%d) [%d]\n", me->top_irq, rc);
        goto out_err_req_irq;
    }

    rc = max77696_topsys_mbattlow_setup(me, pdata);
    if (unlikely(rc < 0)) {
        goto out_err_mbattlow_setup;
    }

    BUG_ON(chip->topsys_ptr);
    chip->topsys_ptr = me;

    return 0;

out_err_mbattlow_setup:
    free_irq(me->top_irq, me);
out_err_req_irq:
    mutex_destroy(&(me->lock));
    kfree(me);
    return rc;
}

void max77696_topsys_exit (struct max77696_chip *chip)
{
    struct max77696_topsys *me = chip->topsys_ptr;
    int i;

    chip->topsys_ptr = NULL;

    for (i = 0; i < NR_TOPSYS_IRQ; i++) {
        unsigned int irq = me->irq_base + i;

        irq_set_handler  (irq, NULL);
        irq_set_chip_data(irq, NULL);
    }

    mutex_destroy(&(me->lock));
    kfree(me);
}

/*******************************************************************************
 * EXTERNAL FUNCTIONS
 ******************************************************************************/

extern struct max77696_chip* max77696;

/* Global Low-Power Mode */
int max77696_topsys_set_global_lp_mode (bool level)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    return max77696_topsys_global_config(me, GLBLCNFG1, GLBL_LPM, !!level);
}
EXPORT_SYMBOL(max77696_topsys_set_global_lp_mode);

/* Enable manual reset */
int max77696_topsys_enable_mr (bool enable)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    return max77696_topsys_global_config(me, GLBLCNFG1, MREN, !!enable);
}
EXPORT_SYMBOL(max77696_topsys_enable_mr);

/* Set manual reset time */
int max77696_topsys_set_mr_time (unsigned int seconds)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;
    u8 mrt;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    if (seconds > 10) {
        mrt = 7;
    } else if (seconds > 8) {
        mrt = 6;
    } else if (seconds > 6) {
        mrt = 5;
    } else if (seconds > 5) {
        mrt = 4;
    } else if (seconds > 4) {
        mrt = 3;
    } else if (seconds > 3) {
        mrt = 2;
    } else if (seconds > 2) {
        mrt = 1;
    } else {
        mrt = 0;
    }

    return max77696_topsys_global_config(me, GLBLCNFG1, MRT, mrt);
}
EXPORT_SYMBOL(max77696_topsys_set_mr_time);

/* Enable EN0 delay */
int max77696_topsys_enable_en0_delay (bool enable)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    return max77696_topsys_global_config(me, GLBLCNFG1, EN0DLY, !!enable);
}
EXPORT_SYMBOL(max77696_topsys_enable_en0_delay);

/* Enable Standby */
int max77696_topsys_enable_standy (bool enable)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    return max77696_topsys_global_config(me, GLBLCNFG1, STBYEN, !!enable);
}
EXPORT_SYMBOL(max77696_topsys_enable_standy);

/* Enable manual reset wakeup */
int max77696_topsys_enable_mr_wakeup (bool enable)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    return max77696_topsys_global_config(me, GLBLCNFG2, MROWK, !!enable);
}
EXPORT_SYMBOL(max77696_topsys_enable_mr_wakeup);

/* Enable the system watchdog timer */
int max77696_topsys_enable_wdt (bool enable)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    return max77696_topsys_global_config(me, GLBLCNFG2, WDTEN, !!enable);
}
EXPORT_SYMBOL(max77696_topsys_enable_wdt);

/* Set the system watchdog timer period */
int max77696_topsys_set_wdt_period (unsigned int seconds)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;
    u8 twd = ((seconds > 64)? 3 : ((seconds > 16)? 2 : ((seconds > 2)? 1 : 0)));

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    return max77696_topsys_global_config(me, GLBLCNFG2, TWD, twd);
}
EXPORT_SYMBOL(max77696_topsys_set_wdt_period);

/* Wakeup on any RTC alarm */
int max77696_topsys_enable_rtc_wakeup (bool enable)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    return max77696_topsys_global_config(me, GLBLCNFG2, RTCAWK, !!enable);
}
EXPORT_SYMBOL(max77696_topsys_enable_rtc_wakeup);

/* Automatic Wakeup Due to System Watchdog Reset */
int max77696_topsys_enable_wdt_wakeup (bool enable)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    return max77696_topsys_global_config(me, GLBLCNFG2, WDWK, !!enable);
}
EXPORT_SYMBOL(max77696_topsys_enable_wdt_wakeup);

/* UIC Wakeup Signal is Edge Triggered */
int max77696_topsys_enable_uic_edge_wakeup (bool enable)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    return max77696_topsys_global_config(me, GLBLCNFG2, UICWK_EDGE, !!enable);
}
EXPORT_SYMBOL(max77696_topsys_enable_uic_edge_wakeup);

/* Set Low-Battery Comparator Hysteresis */
int max77696_topsys_set_lbhyst (unsigned int mV)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    return max77696_topsys_mbattlow_set_lbhyst(me, mV);
}
EXPORT_SYMBOL(max77696_topsys_set_lbhyst);

/* Set Low-Battery DAC Falling Threshold */
int max77696_topsys_set_lbdac (unsigned int mV)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    return max77696_topsys_mbattlow_set_lbdac(me, mV);
}
EXPORT_SYMBOL(max77696_topsys_set_lbdac);

/* Clear the system watchdog timer */
int max77696_topsys_clear_wdt (void)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    return max77696_topsys_global_config(me, GLBLCNFG4, WDTC, 1);
}
EXPORT_SYMBOL(max77696_topsys_clear_wdt);

/* Set nRSTOUT Programmable Delay Timer */
int max77696_topsys_set_rso_delay (unsigned int time_us)
{
    struct max77696_chip *chip = max77696;
    struct max77696_topsys *me;
    u8 rso_delay;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->topsys_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_topsys is not ready\n", __func__);
        return -ENODEV;
    }

    if (time_us > 40960) {
        rso_delay = 3;
    } else if (time_us > 10240) {
        rso_delay = 2;
    } else if (time_us >  1280) {
        rso_delay = 1;
    } else {
        rso_delay = 0;
    }

    return max77696_topsys_global_config(me, GLBLCNFG5, NRSO_DEL, rso_delay);
}
EXPORT_SYMBOL(max77696_topsys_set_rso_delay);

