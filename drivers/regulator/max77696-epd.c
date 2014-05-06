/*
 * Max77696 E-Paper Display Power Supplies Driver
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/max77696.h>

#define DRIVER_DESC    "Max77696 E-Paper Display Power Supplies Driver"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maxim-ic.com>"
#define DRIVER_NAME    MAX77696_EPD_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"

#define EPD_VREG_NAME                 MAX77696_EPD_NAME"-vreg"
#define EPD_VREG_DESC_NAME(_name)     MAX77696_NAME"-vreg-"_name
#define EPD_NREG                      MAX77696_EPD_NR_REGS

#define EPD_EPDCNFG_REG               0x60
#define EPD_EPDCNFG_EPDEN_M           BIT (7)
#define EPD_EPDCNFG_EPDEN_S           7
#define EPD_EPDCNFG_VCOMEN_M          BIT (6)
#define EPD_EPDCNFG_VCOMEN_S          6
#define EPD_EPDCNFG_SSHVINP_M         BIT (5)
#define EPD_EPDCNFG_SSHVINP_S         5
#define EPD_EPDCNFG_SSHVEE_M          BIT (4)
#define EPD_EPDCNFG_SSHVEE_S          4
#define EPD_EPDCNFG_SSHVNEG_M         BIT (3)
#define EPD_EPDCNFG_SSHVNEG_S         3
#define EPD_EPDCNFG_SSHVPOS_M         BIT (2)
#define EPD_EPDCNFG_SSHVPOS_S         2
#define EPD_EPDCNFG_SSHVDDH_M         BIT (1)
#define EPD_EPDCNFG_SSHVDDH_S         1

#define EPD_EPDINTS_REG               0x61
#define EPD_EPDINT_REG                0x62
#define EPD_EPDINTM_REG               0x63

#define EPD_EPDVCOM_REG               0x64
#define EPD_EPDVCOM_VCOM_M            BITS(7,0)
#define EPD_EPDVCOM_VCOM_S            0

#define EPD_EPDVEE_REG                0x65

#define EPD_EPDVNEG_REG               0x66
#define EPD_EPDVNEG_VNEG_M            BITS(7,0)
#define EPD_EPDVNEG_VNEG_S            0

#define EPD_EPDVPOS_REG               0x67
#define EPD_EPDVDDH_REG               0x68
#define EPD_EPDSEQ_REG                0x69
#define EPD_EPDOKINTS_REG             0x6A

#define EPD_EPDOKINT_REG              0xB2
#define EPD_EPDOKINT_EPDPOK_M         BIT (7)
#define EPD_EPDOKINT_EPDPOK_S         7
#define EPD_EPDOKINTM_REG             0xB3
#define EPD_EPDVCOMR_REG              0xB4
#define EPD_EPDVCOMR_VCOMR_M          BIT (0)
#define EPD_EPDVCOMR_VCOMR_S          0
#define EPD_EPDDIS_REG                0xB5

#define EPD_INVALID_REG               0xFF

#define EPD_INT_VCOMFLT               BIT (6) /* VCOM output fault interrupt */
#define EPD_INT_HVINPFLT              BIT (5) /* HVINP output fault interrupt */
#define EPD_INT_VEEFLT                BIT (4) /* VEE output fault interrupt */
#define EPD_INT_VNEGFLT               BIT (3) /* VNEG output fault interrupt */
#define EPD_INT_VPOSFLT               BIT (2) /* VPOS output fault interrupt */
#define EPD_INT_VDDHFLT               BIT (1) /* VDDH output fault interrupt */
#define EPD_INT_VC5FLT                BIT (0) /* VC5 output fault interrupt */

#define EPD_OKINT_EPDPOK              BIT (7) /* EPD Power-OK interrupt */
#define EPD_OKINT_EPDPDN              BIT (6) /* EPD Powered Down interrupt */

#define EPD_REG(reg)                  ((u8)(EPD_##reg##_REG))
#define EPD_REG_BITMASK(reg, bit)     ((u8)(EPD_##reg##_##bit##_M))
#define EPD_REG_BITSHIFT(reg, bit)         (EPD_##reg##_##bit##_S)

#define EPD_REG_BITGET(reg, bit, val) \
        ((u8)(((val) & EPD_REG_BITMASK(reg, bit))\
        >> EPD_REG_BITSHIFT(reg, bit)))
#define EPD_REG_BITSET(reg, bit, val) \
    ((u8)(((val) << EPD_REG_BITSHIFT(reg, bit))\
        & EPD_REG_BITMASK(reg, bit)))

#define mV_to_uV(mV)                  (mV * 1000)
#define uV_to_mV(uV)                  (uV / 1000)
#define V_to_uV(V)                    (mV_to_uV(V * 1000))
#define uV_to_V(uV)                   (uV_to_mV(uV) / 1000)

struct max77696_epd_vreg_desc {
    struct regulator_desc rdesc;

    bool                  negative;
    int                   offset_uV;
    int                   min_uV, max_uV, step_uV;
    u8                    vout_reg;
};

struct max77696_epd_vreg {
    struct max77696_epd_vreg_desc *desc;
    struct regulator_dev          *rdev;
    struct platform_device        *pdev;
};

struct max77696_epd {
    struct mutex               lock;
    struct max77696_chip      *chip;
    struct max77696_i2c       *i2c;
    struct device             *dev;

    unsigned int               pok_wait_timeout_ms;
    bool                       control_vcom_en;

    struct max77696_epd_vreg   vreg[EPD_NREG];

    unsigned int               irq;
    u8                         irq_unmask;
    u8                         interrupted;
    u8                         ok_irq_unmask;
    u8                         ok_interrupted;

    struct completion          pwr_change_done;
};

#define __get_i2c(chip)                (&((chip)->pmic_i2c))
#define __lock(me)                     mutex_lock(&((me)->lock))
#define __unlock(me)                   mutex_unlock(&((me)->lock))

/* EPD Register Read/Write */
#define max77696_epd_reg_read(me, reg, val_ptr) \
    max77696_read((me)->i2c, EPD_REG(reg), val_ptr)
#define max77696_epd_reg_write(me, reg, val) \
        max77696_write((me)->i2c, EPD_REG(reg), val)
#define max77696_epd_reg_bulk_read(me, reg, dst, len) \
        max77696_bulk_read((me)->i2c, EPD_REG(reg), dst, len)
#define max77696_epd_reg_bulk_write(me, reg, src, len) \
        max77696_bulk_write((me)->i2c, EPD_REG(reg), src, len)
#define max77696_epd_reg_read_masked(me, reg, mask, val_ptr) \
        max77696_read_masked((me)->i2c, EPD_REG(reg), mask, val_ptr)
#define max77696_epd_reg_write_masked(me, reg, mask, val) \
        max77696_write_masked((me)->i2c, EPD_REG(reg), mask, val)

/* EPD Register Single Bit Ops */
#define max77696_epd_reg_get_bit(me, reg, bit, val_ptr) \
        ({\
            int __rc = max77696_epd_reg_read_masked(me, reg,\
                EPD_REG_BITMASK(reg, bit), val_ptr);\
            *(val_ptr) = EPD_REG_BITGET(reg, bit, *(val_ptr));\
            __rc;\
        })
#define max77696_epd_reg_set_bit(me, reg, bit, val) \
        ({\
            max77696_epd_reg_write_masked(me, reg,\
                EPD_REG_BITMASK(reg, bit), EPD_REG_BITSET(reg, bit, val));\
        })

  #define __msleep(msec) msleep_interruptible((unsigned int)(msec))
//#define __msleep(msec) msleep((unsigned int)(msec))
//#define __msleep(msec) mdelay((unsigned int)(msec))

#define max77696_epd_write_irq_mask(me) \
        do {\
            int _rc = max77696_epd_reg_write(me, EPDINTM, ~((me)->irq_unmask));\
            if (unlikely(_rc)) {\
                dev_err((me)->dev, "EPDINTM write error [%d]\n", _rc);\
            }\
        } while (0)

static __inline void max77696_epd_enable_irq (struct max77696_epd* me,
    u8 irq_bits, bool forced)
{
    if (unlikely(!forced && (me->irq_unmask & irq_bits) == irq_bits)) {
        /* already unmasked */
        return;
    }

    /* set enabled flag */
    me->irq_unmask |= irq_bits;
    max77696_epd_write_irq_mask(me);
}

static __inline void max77696_epd_disable_irq (struct max77696_epd* me,
    u8 irq_bits, bool forced)
{
    if (unlikely(!forced && (me->irq_unmask & irq_bits) == 0)) {
        /* already masked */
        return;
    }

    /* clear enabled flag */
    me->irq_unmask &= ~irq_bits;
    max77696_epd_write_irq_mask(me);
}

#define max77696_epd_write_ok_irq_mask(me) \
        do {\
            int _rc = max77696_epd_reg_write(me, EPDOKINTM,\
                ~((me)->ok_irq_unmask));\
            if (unlikely(_rc)) {\
                dev_err((me)->dev, "EPDOKINTM write error [%d]\n", _rc);\
            }\
        } while (0)

static __inline void max77696_epd_enable_ok_irq (struct max77696_epd* me,
    u8 ok_irq_bits, bool forced)
{
    if (unlikely(!forced && (me->ok_irq_unmask & ok_irq_bits) == ok_irq_bits)) {
        /* already unmasked */
        return;
    }

    /* set enabled flag */
    me->ok_irq_unmask |= ok_irq_bits;
    max77696_epd_write_ok_irq_mask(me);
}

static __inline void max77696_epd_disable_ok_irq (struct max77696_epd* me,
    u8 ok_irq_bits, bool forced)
{
    if (unlikely(!forced && (me->ok_irq_unmask & ok_irq_bits) == 0)) {
        /* already masked */
        return;
    }

    /* clear enabled flag */
    me->ok_irq_unmask &= ~ok_irq_bits;
    max77696_epd_write_ok_irq_mask(me);
}

static int max77696_epd_wait_for_power_change (struct max77696_epd *me,
    bool pwrup)
{
    struct completion *completion = &(me->pwr_change_done);
    unsigned long timeout = msecs_to_jiffies(me->pok_wait_timeout_ms);
    u8 ok_irq_bit = (pwrup? EPD_OKINT_EPDPOK : EPD_OKINT_EPDPDN);
    int rc;

    INIT_COMPLETION(*completion);
    max77696_epd_enable_ok_irq(me, ok_irq_bit, 0);

    rc = wait_for_completion_interruptible_timeout(completion, timeout);

    if (likely(rc > 0)) {
        rc = 0;
        goto out;
    }

    if (unlikely(rc < 0)) {
        dev_err(me->dev, "failed to wait for EPD power %s [%d]\n",
            pwrup? "up" : "down", rc);
        goto out;
    }

    dev_err(me->dev, "EPD power %s timed out\n", pwrup? "up" : "down");
    rc = -ETIMEDOUT;

out:
    /* Disable IRQ after completion */
    max77696_epd_disable_ok_irq(me, ok_irq_bit, 0);
    return rc;
}

#define __rdev_name(rdev_ptr) \
        (rdev_ptr->desc->name)
#define __rdev_to_max77696_epd(rdev_ptr) \
        ((struct max77696_epd*)rdev_get_drvdata(rdev_ptr))
#define __rdev_to_max77696_epd_vreg(rdev_ptr) \
        (&(__rdev_to_max77696_epd(rdev_ptr)->vreg[rdev_ptr->desc->id]))

static int max77696_epd_vreg_set_voltage (struct regulator_dev *rdev,
    int min_uV, int max_uV, unsigned *selector)
{
    struct max77696_epd *me = __rdev_to_max77696_epd(rdev);
    struct max77696_epd_vreg *vreg = __rdev_to_max77696_epd_vreg(rdev);
    struct max77696_epd_vreg_desc *desc = vreg->desc;
    u8 val;
    int rc;

    __lock(me);

    min_uV -= desc->offset_uV;
    max_uV -= desc->offset_uV;

    dev_vdbg(me->dev, "%s set_voltage(min %duV, max %duV)\n",
        __rdev_name(rdev), min_uV, max_uV);

    if (unlikely(min_uV < desc->min_uV || min_uV > desc->max_uV)) {
        dev_err(me->dev, "%s setting voltage out of range\n",
            __rdev_name(rdev));
        rc = -EINVAL;
        goto out;
    }

    if (desc->negative) {
        min_uV = -min_uV + desc->max_uV;
    } else {
        min_uV =  min_uV - desc->min_uV;
    }

    val = (u8)DIV_ROUND_UP(min_uV, desc->step_uV);

    dev_vdbg(me->dev, "%s write vout reg addr %02X val %02X\n",
        __rdev_name(rdev), desc->vout_reg, val);

    rc = max77696_write(me->i2c, desc->vout_reg, val);
    if (unlikely(rc)) {
        dev_err(me->dev, "%s VOUT_REG write error [%d]\n",
            __rdev_name(rdev), rc);
        goto out;
    }

out:
    __unlock(me);
    return rc;
}

static int max77696_epd_vreg_get_voltage (struct regulator_dev *rdev)
{
    struct max77696_epd *me = __rdev_to_max77696_epd(rdev);
    struct max77696_epd_vreg *vreg = __rdev_to_max77696_epd_vreg(rdev);
    struct max77696_epd_vreg_desc *desc = vreg->desc;
    u8 val;
    int voltage = 0, rc;

    __lock(me);

    rc = max77696_read(me->i2c, desc->vout_reg, &val);
    if (unlikely(rc)) {
        dev_err(me->dev, "%s VOUT_REG read error [%d]\n",
            __rdev_name(rdev), rc);
        goto out;
    }

    dev_vdbg(me->dev, "%s read vout reg addr %02X val %02X\n",
        desc->rdesc.name, desc->vout_reg, val);

    voltage = (int)val * desc->step_uV;

    if (desc->negative) {
        voltage = -voltage + desc->max_uV;
    } else {
        voltage =  voltage + desc->min_uV;
    }
    voltage = min(desc->max_uV, max(desc->min_uV, voltage));

    voltage += desc->offset_uV;

out:
    __unlock(me);
    return voltage;
}

static struct regulator_ops max77696_epd_vreg_ops = {
    .set_voltage = max77696_epd_vreg_set_voltage,
    .get_voltage = max77696_epd_vreg_get_voltage,
};

static int max77696_epd_vreg_disp_enable (struct regulator_dev *rdev)
{
    struct max77696_epd *me = __rdev_to_max77696_epd(rdev);
    int rc;

    __lock(me);

    dev_dbg(me->dev, "enabling DISPLAY ...\n");

    rc = max77696_epd_reg_set_bit(me, EPDCNFG, EPDEN, 1);
    if (unlikely(rc)) {
        dev_err(me->dev, "EPDCNFG write error [%d]\n", rc);
        goto out;
    }

    rc = max77696_epd_wait_for_power_change(me, 1);
    if (unlikely(rc)) {
        goto out;
    }

    dev_dbg(me->dev, "enabled DISPLAY\n");

out:
    __unlock(me);
    return rc;
}

static int max77696_epd_vreg_disp_disable (struct regulator_dev *rdev)
{
    struct max77696_epd *me = __rdev_to_max77696_epd(rdev);
    int rc;

    __lock(me);

    dev_dbg(me->dev, "disabling DISPLAY ...\n");

    rc = max77696_epd_reg_set_bit(me, EPDCNFG, EPDEN, 0);
    if (unlikely(rc)) {
        dev_err(me->dev, "EPDCNFG write error [%d]\n", rc);
        goto out;
    }

    rc = max77696_epd_wait_for_power_change(me, 0);
    if (unlikely(rc)) {
        goto out;
    }

    dev_dbg(me->dev, "disabled DISPLAY\n");

out:
    __unlock(me);
    return rc;
}

static int max77696_epd_vreg_disp_is_enabled (struct regulator_dev *rdev)
{
    struct max77696_epd *me = __rdev_to_max77696_epd(rdev);
    u8 epden = 0;
    int rc;

    __lock(me);

    rc = max77696_epd_reg_get_bit(me, EPDCNFG, EPDEN, &epden);
    if (unlikely(rc)) {
        dev_err(me->dev, "EPDCNFG read error [%d]\n", rc);
        return 0;
    }

    __unlock(me);
    return !!epden;
}

static struct regulator_ops max77696_epd_vreg_disp_ops = {
    .enable      = max77696_epd_vreg_disp_enable,
    .disable     = max77696_epd_vreg_disp_disable,
    .is_enabled  = max77696_epd_vreg_disp_is_enabled,
};

static int max77696_epd_vreg_vcom_set_voltage (struct regulator_dev *rdev,
    int min_uV, int max_uV, unsigned *selector)
{
    struct max77696_epd *me = __rdev_to_max77696_epd(rdev);
    struct max77696_epd_vreg *vreg = __rdev_to_max77696_epd_vreg(rdev);
    struct max77696_epd_vreg_desc *desc = vreg->desc;
    u8 vcomr, vcom;
    int rc;

    __lock(me);

    min_uV -= desc->offset_uV;
    max_uV -= desc->offset_uV;

    dev_vdbg(me->dev, "%s set_voltage(min %duV, max %duV)\n",
        __rdev_name(rdev), min_uV, max_uV);

    if (unlikely(min_uV < desc->min_uV || min_uV > desc->max_uV)) {
        dev_err(me->dev, "%s setting voltage out of range\n",
            __rdev_name(rdev));
        rc = -EINVAL;
        goto out;
    }

    min_uV = -min_uV + desc->max_uV;
    min_uV = DIV_ROUND_UP(min_uV, desc->step_uV);
    vcomr  = (u8)(min_uV > 0xFF);
    vcom   = (u8)(min_uV & 0xFF);

    dev_dbg(me->dev, "writing EPDVCOMR %02x EPDVCOM %02x\n", vcomr, vcom);

    rc = max77696_epd_reg_set_bit(me, EPDVCOMR, VCOMR, vcomr);
    if (unlikely(rc)) {
        dev_err(me->dev, "EPDVCOMR write error [%d]\n", rc);
        goto out;
    }

    rc = max77696_epd_reg_set_bit(me, EPDVCOM, VCOM, vcom);
    if (unlikely(rc)) {
        dev_err(me->dev, "EPDVCOM write error [%d]\n", rc);
        goto out;
    }

out:
    __unlock(me);
    return rc;
}

static int max77696_epd_vreg_vcom_get_voltage (struct regulator_dev *rdev)
{
    struct max77696_epd *me = __rdev_to_max77696_epd(rdev);
    struct max77696_epd_vreg *vreg = __rdev_to_max77696_epd_vreg(rdev);
    struct max77696_epd_vreg_desc *desc = vreg->desc;
    u8 vcomr, vcom;
    int voltage = 0, rc;

    __lock(me);

    rc = max77696_epd_reg_get_bit(me, EPDVCOMR, VCOMR, &vcomr);
    if (unlikely(rc)) {
        dev_err(me->dev, "EPDVCOMR read error [%d]\n", rc);
        goto out;
    }

    rc = max77696_epd_reg_get_bit(me, EPDVCOM, VCOM, &vcom);
    if (unlikely(rc)) {
        dev_err(me->dev, "EPDVCOM read error [%d]\n", rc);
        goto out;
    }

    dev_dbg(me->dev, "read EPDVCOMR %02x EPDVCOM %02x\n", vcomr, vcom);

    voltage = (((int)vcomr << 8) + (int)vcom) * desc->step_uV;
    voltage = -voltage + desc->max_uV;
    voltage = min(desc->max_uV, max(desc->min_uV, voltage));

    voltage += desc->offset_uV;

out:
    __unlock(me);
    return voltage;
}

static int max77696_epd_vreg_vcom_enable (struct regulator_dev *rdev)
{
    struct max77696_epd *me = __rdev_to_max77696_epd(rdev);
    int rc;

    __lock(me);

    dev_dbg(me->dev, "enabling VCOM ...\n");

    rc = max77696_epd_reg_set_bit(me, EPDCNFG, VCOMEN, 1);
    if (unlikely(rc)) {
        dev_err(me->dev, "EPDCNFG write error [%d]\n", rc);
        goto out;
    }

    dev_dbg(me->dev, "enabled VCOM\n");

out:
    __unlock(me);
    return rc;
}

static int max77696_epd_vreg_vcom_disable (struct regulator_dev *rdev)
{
    struct max77696_epd *me = __rdev_to_max77696_epd(rdev);
    int rc;

    __lock(me);

    dev_dbg(me->dev, "disabling VCOM ...\n");

    rc = max77696_epd_reg_set_bit(me, EPDCNFG, VCOMEN, 0);
    if (unlikely(rc)) {
        dev_err(me->dev, "EPDCNFG write error [%d]\n", rc);
        goto out;
    }

    dev_dbg(me->dev, "disabled VCOM\n");

out:
    __unlock(me);
    return rc;
}

static int max77696_epd_vreg_vcom_is_enabled (struct regulator_dev *rdev)
{
    struct max77696_epd *me = __rdev_to_max77696_epd(rdev);
    u8 vcomen = 0;
    int rc;

    __lock(me);

    rc = max77696_epd_reg_get_bit(me, EPDCNFG, VCOMEN, &vcomen);
    if (unlikely(rc)) {
        dev_err(me->dev, "EPDCNFG read error [%d]\n", rc);
    }

    __unlock(me);
    return !!vcomen;
}

static struct regulator_ops max77696_epd_vreg_vcom_ops = {
    .set_voltage = max77696_epd_vreg_vcom_set_voltage,
    .get_voltage = max77696_epd_vreg_vcom_get_voltage,

    /* enable/disable regulator */
    .enable      = max77696_epd_vreg_vcom_enable,
    .disable     = max77696_epd_vreg_vcom_disable,
    .is_enabled  = max77696_epd_vreg_vcom_is_enabled,
};

#define max77696_epd_vreg_vee_ops  max77696_epd_vreg_ops
#define max77696_epd_vreg_vneg_ops max77696_epd_vreg_ops
#define max77696_epd_vreg_vpos_ops max77696_epd_vreg_ops
#define max77696_epd_vreg_vddh_ops max77696_epd_vreg_ops

#define EPD_VREG_DESC(_id, _name, _rops, _vout_reg, _neg, _ofst, _min, _max, _step) \
        [MAX77696_EPD_ID_##_id] = {\
            .rdesc.name  = EPD_VREG_DESC_NAME(_name),\
            .rdesc.id    = MAX77696_EPD_ID_##_id,\
            .rdesc.ops   = &max77696_epd_vreg_##_rops,\
            .rdesc.type  = REGULATOR_VOLTAGE,\
            .rdesc.owner = THIS_MODULE,\
            .negative    = _neg,\
            .offset_uV   = _ofst,\
            .min_uV      = _min,\
            .max_uV      = _max,\
            .step_uV     = _step,\
            .vout_reg    = EPD_REG(_vout_reg),\
        }

#define VREG_DESC(id) (&(max77696_epd_vreg_descs[id]))
static struct max77696_epd_vreg_desc max77696_epd_vreg_descs[EPD_NREG] = {
    #undef  NEG_SIGN
    #define NEG_SIGN 1 /* negative */
    #undef  POS_SIGN
    #define POS_SIGN 0 /* positive */
    #undef  UNK_SIGN
    #define UNK_SIGN 0 /* unknown */

    EPD_VREG_DESC(DISP,  "disp", disp_ops, INVALID, UNK_SIGN,        0,         0,         0,      0),
    EPD_VREG_DESC(VCOM,  "vcom", vcom_ops, EPDVCOM, NEG_SIGN,  5000000, - 5000000,         0,   9800),
    EPD_VREG_DESC(VEE,   "vee",  vee_ops,  EPDVEE,  NEG_SIGN, 28020000, -28020000, -15000000, 420000),
    EPD_VREG_DESC(VNEG,  "vneg", vneg_ops, EPDVNEG, POS_SIGN,  1600000, - 1600000,   1587500,  12500),
    EPD_VREG_DESC(VPOS,  "vpos", vpos_ops, EPDVPOS, POS_SIGN,        0,   8000000,  18000000, 500000),
    EPD_VREG_DESC(VDDH,  "vddh", vddh_ops, EPDVDDH, POS_SIGN,        0,  15000000,  29500000, 470000),
};

static int max77696_epd_vreg_probe (struct platform_device *pdev)
{
    struct max77696_epd *me = platform_get_drvdata(pdev);
    struct max77696_epd_vreg *vreg = &(me->vreg[pdev->id]);
    struct max77696_epd_vreg_desc *desc = VREG_DESC(pdev->id);
    struct regulator_init_data *init_data = dev_get_platdata(&(pdev->dev));
    struct regulator_dev *rdev;
    int rc;

    /* Overwrite correct offset of in/out */
    init_data->constraints.uV_offset = desc->offset_uV;

    /* Save my descriptor */
    vreg->desc = desc;

    /* Register my own regulator device */
    rdev = regulator_register(&(desc->rdesc), &(pdev->dev), init_data, me);
    if (unlikely(IS_ERR(rdev))) {
        rc = PTR_ERR(rdev);
        rdev = NULL;

        dev_err(&(pdev->dev), "failed to register regulator for %s [%d]\n",
            desc->rdesc.name, rc);
        goto out_err;
    }

    vreg->rdev = rdev;
    platform_set_drvdata(pdev, rdev);

    return 0;

out_err:
    if (likely(rdev)) {
        regulator_unregister(rdev);
        me->vreg[pdev->id].rdev = NULL;
    }
    return rc;
}

static int max77696_epd_vreg_remove (struct platform_device *pdev)
{
    struct regulator_dev *rdev = platform_get_drvdata(pdev);
    struct max77696_epd_vreg *vreg = __rdev_to_max77696_epd_vreg(rdev);

    regulator_unregister(rdev);
    vreg->rdev = NULL;

    return 0;
}

static struct platform_driver max77696_epd_vreg_driver = {
    .probe       = max77696_epd_vreg_probe,
    .remove      = max77696_epd_vreg_remove,
    .driver.name = EPD_VREG_NAME,
};

static __always_inline
void max77696_epd_unregister_vreg_drv (struct max77696_epd *me)
{
    platform_driver_unregister(&max77696_epd_vreg_driver);
}

static __always_inline
int max77696_epd_register_vreg_drv (struct max77696_epd *me)
{
    return platform_driver_register(&max77696_epd_vreg_driver);
}

static __always_inline
void max77696_epd_unregister_vreg_dev (struct max77696_epd *me, int id)
{
    /* nothing to do */
}

static __always_inline
int max77696_epd_register_vreg_dev (struct max77696_epd *me, int id,
    struct regulator_init_data *init_data)
{
    struct max77696_epd_vreg *vreg = &(me->vreg[id]);
    int rc;

    vreg->pdev = platform_device_alloc(EPD_VREG_NAME, id);
    if (unlikely(!vreg->pdev)) {
        dev_err(me->dev, "failed to alloc pdev for %s.%d\n",
            EPD_VREG_NAME, id);
        rc = -ENOMEM;
        goto out_err;
    }

    platform_set_drvdata(vreg->pdev, me);
    vreg->pdev->dev.platform_data = init_data;
    vreg->pdev->dev.parent        = me->dev;

    rc = platform_device_add(vreg->pdev);
    if (unlikely(rc)) {
        dev_err(me->dev, "failed to pdev for %s.%d [%d]\n",
            EPD_VREG_NAME, id, rc);
        goto out_err;
    }

    return 0;

out_err:
    if (likely(vreg->pdev)) {
        platform_device_del(vreg->pdev);
        vreg->pdev = NULL;
    }
    return rc;
}

static void max77696_epd_unregister_vreg (struct max77696_epd *me)
{
    int i;

    for (i = 0; i < EPD_NREG; i++) {
        max77696_epd_unregister_vreg_dev(me, i);
    }

    max77696_epd_unregister_vreg_drv(me);
}

static int max77696_epd_register_vreg (struct max77696_epd *me,
    struct regulator_init_data init_data[EPD_NREG])
{
    int i, rc;

    rc = max77696_epd_register_vreg_drv(me);
    if (unlikely(rc)) {
        dev_err(me->dev, "failed to register vreg drv for %s [%d]\n",
            EPD_VREG_NAME, rc);
        goto out;
    }

    for (i = 0; i < EPD_NREG; i++) {
        dev_vdbg(me->dev, "registering vreg dev for %s.%d ...\n",
            EPD_VREG_NAME, i);
        rc = max77696_epd_register_vreg_dev(me, i, &(init_data[i]));
        if (unlikely(rc)) {
            dev_err(me->dev, "failed to register vreg dev for %s.%d [%d]\n",
                EPD_VREG_NAME, i, rc);
            goto out;
        }
    }

out:
    return rc;
}

static irqreturn_t max77696_epd_isr (int irq, void *data)
{
    struct max77696_epd *me = data;

    max77696_epd_reg_read(me, EPDINT, &(me->interrupted));
    max77696_epd_reg_read(me, EPDOKINT, &(me->ok_interrupted));

    dev_dbg(me->dev, "EPDINT   %02X EN %02X\n",
        me->interrupted, me->irq_unmask);
    me->interrupted &= me->irq_unmask;

    dev_dbg(me->dev, "EPDOKINT %02X EN %02X\n",
        me->ok_interrupted, me->ok_irq_unmask);
    me->ok_interrupted &= me->ok_irq_unmask;

    if (likely(me->ok_interrupted)) {
        complete(&(me->pwr_change_done));
    }

    return IRQ_HANDLED;
}

static __devinit int max77696_epd_probe (struct platform_device *pdev)
{
    struct max77696_chip *chip = dev_get_drvdata(pdev->dev.parent);
    struct max77696_epd_platform_data *pdata = dev_get_platdata(&(pdev->dev));
    struct max77696_epd *me;
    int rc;

    if (unlikely(!pdata)) {
        dev_err(&(pdev->dev), "platform data is missing\n");
        return -ENODEV;
    }

    me = kzalloc(sizeof(*me), GFP_KERNEL);
    if (unlikely(!me)) {
        dev_err(&(pdev->dev), "out of memory (%uB requested)\n", sizeof(*me));
        return -ENOMEM;
    }

    platform_set_drvdata(pdev, me);

    mutex_init(&(me->lock));
    me->chip = chip;
    me->i2c  = __get_i2c(chip);
    me->dev  = &(pdev->dev);

    me->pok_wait_timeout_ms = pdata->pok_wait_timeout_ms;
    me->control_vcom_en     = pdata->control_vcom_en;

    me->irq = max77696_rootint_to_irq(chip, EPD);
    init_completion(&(me->pwr_change_done));

    /* Disable all EPD interrupts */
    max77696_epd_disable_irq(me, 0xFF, 1);
    max77696_epd_disable_ok_irq(me, 0xFF, 1);

    /* Register EPD power supplies driver & device */
    rc = max77696_epd_register_vreg(me, pdata->init_data);
    if (unlikely(rc)) {
        goto out_err_reg_vregs;
    }

    /* Request EPD interrupt */
    rc = request_threaded_irq(me->irq,
        NULL, max77696_epd_isr, IRQF_ONESHOT, DRIVER_NAME, me);
    if (unlikely(rc < 0)) {
        dev_err(me->dev, "failed to request IRQ(%d) [%d]\n", me->irq, rc);
        goto out_err_req_epd_irq;
    }

    pr_info(DRIVER_DESC" "DRIVER_VERSION" Installed\n");
    return 0;

out_err_req_epd_irq:
    max77696_epd_unregister_vreg(me);
out_err_reg_vregs:
    mutex_destroy(&(me->lock));
    platform_set_drvdata(pdev, NULL);
    kfree(me);
    return rc;
}

static __devexit int max77696_epd_remove (struct platform_device *pdev)
{
    struct max77696_epd *me = platform_get_drvdata(pdev);

    free_irq(me->irq, me);

    max77696_epd_unregister_vreg(me);

    mutex_destroy(&(me->lock));
    platform_set_drvdata(pdev, NULL);
    kfree(me);

    return 0;
}

static struct platform_driver max77696_epd_driver = {
    .driver.name  = DRIVER_NAME,
    .driver.owner = THIS_MODULE,
    .probe        = max77696_epd_probe,
    .remove       = __devexit_p(max77696_epd_remove),
};

static __init int max77696_epd_driver_init (void)
{
    return platform_driver_register(&max77696_epd_driver);
}

static __exit void max77696_epd_driver_exit (void)
{
    platform_driver_unregister(&max77696_epd_driver);
}

subsys_initcall(max77696_epd_driver_init);
module_exit(max77696_epd_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);

