/*
 * MAX77696 USB Interface Circuit Driver
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

#define DRIVER_DESC    "MAX77696 USB Interface Circuit Driver"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maxim-ic.com>"
#define DRIVER_NAME    MAX77696_UIC_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"

#define UIC_DEVICEID_REG              0x00
#define UIC_DEVICEID_CHIPID_M         BITS(7,4)
#define UIC_DEVICEID_CHIPID_S         4
#define UIC_DEVICEID_CHIPREV_M        BITS(3,0)
#define UIC_DEVICEID_CHIPREV_S        0

#define UIC_INT1_REG                  0x01
#define UIC_INT2_REG                  0x02

#define UIC_STATUS1_REG               0x03
#define UIC_STATUS1_DCDTMR_M          BIT (7)
#define UIC_STATUS1_DCDTMR_S          7
#define UIC_STATUS1_CHGDETACT_M       BIT (6)
#define UIC_STATUS1_CHGDETACT_S       6
#define UIC_STATUS1_VBVOLT_M          BIT (4)
#define UIC_STATUS1_VBVOLT_S          4
#define UIC_STATUS1_CHGTYP_M          BITS(3,0)
#define UIC_STATUS1_CHGTYP_S          0

#define UIC_STATUS2_REG               0x04
#define UIC_STATUS2_ADCERROR_M        BIT (7)
#define UIC_STATUS2_ADCERROR_S        7
#define UIC_STATUS2_ENUSTAT_M         BIT (6)
#define UIC_STATUS2_ENUSTAT_S         6
#define UIC_STATUS2_ADC_M             BITS(4,0)
#define UIC_STATUS2_ADC_S             0

#define UIC_INTMASK1_REG              0x05
#define UIC_INTMASK2_REG              0x06

#define UIC_SYSCTRL1_REG              0x07
#define UIC_SYSCTRL1_INT_TYP_M        BIT (7)
#define UIC_SYSCTRL1_INT_TYP_S        7
#define UIC_SYSCTRL1_INT_DLY_M        BIT (6)
#define UIC_SYSCTRL1_INT_DLY_S        6
#define UIC_SYSCTRL1_INT_POL_M        BIT (5)
#define UIC_SYSCTRL1_INT_POL_S        5
#define UIC_SYSCTRL1_INT_EN_M         BIT (4)
#define UIC_SYSCTRL1_INT_EN_S         4
#define UIC_SYSCTRL1_USBSWC_M         BITS(3,2)
#define UIC_SYSCTRL1_USBSWC_S         2
#define UIC_SYSCTRL1_LOW_POW_M        BIT (0)
#define UIC_SYSCTRL1_LOW_POW_S        0

#define UIC_SYSCTRL2_REG              0x08
#define UIC_SYSCTRL2_ADC_DEB_M        BITS(4,3)
#define UIC_SYSCTRL2_ADC_DEB_S        3
#define UIC_SYSCTRL2_DCDCPL_M         BIT (2)
#define UIC_SYSCTRL2_DCDCPL_S         2
#define UIC_SYSCTRL2_IDAUTOSWC_M      BIT (1)
#define UIC_SYSCTRL2_IDAUTOSWC_S      1
#define UIC_SYSCTRL2_ADC_EN_M         BIT (0)
#define UIC_SYSCTRL2_ADC_EN_S         0

#define UIC_CDETCTRL_REG              0x09
#define UIC_CDETCTRL_SFEN_M           BITS(7,6)
#define UIC_CDETCTRL_SFEN_S           6
#define UIC_CDETCTRL_CDPDET_M         BIT (5)
#define UIC_CDETCTRL_CDPDET_S         5
#define UIC_CDETCTRL_DCHKTM_M         BIT (4)
#define UIC_CDETCTRL_DCHKTM_S         4
#define UIC_CDETCTRL_DCD2SCT_M        BIT (3)
#define UIC_CDETCTRL_DCD2SCT_S        3
#define UIC_CDETCTRL_DCDEN_M          BIT (2)
#define UIC_CDETCTRL_DCDEN_S          2
#define UIC_CDETCTRL_CHGTYPMAN_M      BIT (1)
#define UIC_CDETCTRL_CHGTYPMAN_S      1
#define UIC_CDETCTRL_CHGDETEN_M       BIT (0)
#define UIC_CDETCTRL_CHGDETEN_S       0

#define UIC_MANCTRL_REG               0x0A
#define UIC_MANCTRL_OIDSET_M          BITS(7,6)
#define UIC_MANCTRL_OIDSET_S          6
#define UIC_MANCTRL_FIDSET_M          BITS(5,4)
#define UIC_MANCTRL_FIDSET_S          4
#define UIC_MANCTRL_MANSET_M          BIT (3)
#define UIC_MANCTRL_MANSET_S          3
#define UIC_MANCTRL_ISET_M            BITS(2,0)
#define UIC_MANCTRL_ISET_S            0

#define UIC_CHGCTRL_REG               0x0B
#define UIC_CHGCTRL_AUTH500_M         BIT (7)
#define UIC_CHGCTRL_AUTH500_S         7
#define UIC_CHGCTRL_ENUMSUB_M         BIT (6)
#define UIC_CHGCTRL_ENUMSUB_S         6
#define UIC_CHGCTRL_DCPSET_M          BITS(5,4)
#define UIC_CHGCTRL_DCPSET_S          4
#define UIC_CHGCTRL_ENUMEN_M          BIT (3)
#define UIC_CHGCTRL_ENUMEN_S          3
#define UIC_CHGCTRL_IMAX_M            BITS(2,0)
#define UIC_CHGCTRL_IMAX_S            0

#define UIC_ENUCTRL_REG               0x0C
#define UIC_ENUCTRL_ENUMTM_M          BITS(1,0)
#define UIC_ENUCTRL_ENUMTM_S          0

#define UIC_INTSTS_REG                0x0D
#define UIC_INTSTS_UICWK_M            BIT (7)
#define UIC_INTSTS_UICWK_S            7
#define UIC_INTSTS_CHG_DET_ACT_M      BIT (6)
#define UIC_INTSTS_CHG_DET_ACT_S      6
#define UIC_INTSTS_FACTORY_M          BIT (5)
#define UIC_INTSTS_FACTORY_S          5
#define UIC_INTSTS_DB_M               BIT (4)
#define UIC_INTSTS_DB_S               4
#define UIC_INTSTS_SFEN_M             BIT (3)
#define UIC_INTSTS_SFEN_S             3
#define UIC_INTSTS_ISET_M             BITS(2,0)
#define UIC_INTSTS_ISET_S             0

/*** Interrupt Bits ***/
#define UIC_INT2_ADCERROR             BIT (15)
#define UIC_INT2_ENUSTAT              BIT (14)
//                                    BIT (13)
//                                    BIT (12)
//                                    BIT (11)
//                                    BIT (10)
//                                    BIT ( 9)
#define UIC_INT2_ADC                  BIT ( 8)
#define UIC_INT1_DCDTMR               BIT ( 7)
#define UIC_INT1_CHGDETACTRISE        BIT ( 6)
#define UIC_INT1_CHGDETACTFALL        BIT ( 5)
//                                    BIT ( 4)
//                                    BIT ( 3)
//                                    BIT ( 2)
#define UIC_INT1_VBVOLT               BIT ( 1)
#define UIC_INT1_CHGTYP               BIT ( 0)

#define UIC_INT2_ALL                  0xC1
#define UIC_INT1_ALL                  0xE3

struct max77696_uic {
    struct mutex          lock;
    struct max77696_chip *chip;
    struct max77696_i2c  *i2c;
    struct device        *dev;
    struct kobject       *kobj;

    void                  (*uic_notify) (const struct max77696_uic_notify*);

    struct delayed_work   psy_work;
    unsigned int          irq;
    u8                    irq_unmask[2];
    u8                    interrupted[2];
    u8                    vb_volt;
    u8                    chg_type;
    u8                    adc_code;
    u8                    i_set;
};

#define __psy_work_to_max77696_uic(psy_work_ptr) \
        container_of(psy_work_ptr, struct max77696_uic, psy_work.work)

#define __get_i2c(chip)               (&((chip)->uic_i2c))
#define __lock(me)                    mutex_lock(&((me)->lock))
#define __unlock(me)                  mutex_unlock(&((me)->lock))

#define UIC_REG(reg)                  ((u8)(UIC_##reg##_REG))
#define UIC_REG_BITMASK(reg, bit)     ((u8)(UIC_##reg##_##bit##_M))
#define UIC_REG_BITSHIFT(reg, bit)         (UIC_##reg##_##bit##_S)

#define UIC_REG_BITGET(reg, bit, val) \
        ((u8)(((val) & UIC_REG_BITMASK(reg, bit))\
        >> UIC_REG_BITSHIFT(reg, bit)))
#define UIC_REG_BITSET(reg, bit, val) \
        ((u8)(((val) << UIC_REG_BITSHIFT(reg, bit))\
        & UIC_REG_BITMASK(reg, bit)))

/* UIC Register Read/Write */
#define max77696_uic_reg_read(me, reg, val_ptr) \
        max77696_read((me)->i2c, UIC_REG(reg), val_ptr)
#define max77696_uic_reg_write(me, reg, val) \
        max77696_write((me)->i2c, UIC_REG(reg), val)
#define max77696_uic_reg_bulk_read(me, reg, dst, len) \
        max77696_bulk_read((me)->i2c, UIC_REG(reg), dst, len)
#define max77696_uic_reg_bulk_write(me, reg, src, len) \
        max77696_bulk_write((me)->i2c, UIC_REG(reg), src, len)
#define max77696_uic_reg_read_masked(me, reg, mask, val_ptr) \
        max77696_read_masked((me)->i2c, UIC_REG(reg), mask, val_ptr)
#define max77696_uic_reg_write_masked(me, reg, mask, val) \
        max77696_write_masked((me)->i2c, UIC_REG(reg), mask, val)

/* UIC Register Single Bit Ops */
#define max77696_uic_reg_get_bit(me, reg, bit, val_ptr) \
        ({\
            int __rc = max77696_uic_reg_read_masked(me, reg,\
                UIC_REG_BITMASK(reg, bit), val_ptr);\
            *(val_ptr) = UIC_REG_BITGET(reg, bit, *(val_ptr));\
            __rc;\
        })
#define max77696_uic_reg_set_bit(me, reg, bit, val) \
        ({\
            max77696_uic_reg_write_masked(me, reg,\
                UIC_REG_BITMASK(reg, bit), UIC_REG_BITSET(reg, bit, val));\
        })

#define max77696_uic_write_irq_mask(me) \
        do {\
            u8 _buf[2] = {\
                [0] = ((me)->irq_unmask[0] & UIC_INT1_ALL),\
                [1] = ((me)->irq_unmask[1] & UIC_INT2_ALL),\
            };\
            int _rc = max77696_uic_reg_bulk_write(me, INTMASK1, _buf, 2);\
            dev_vdbg((me)->dev,\
                "written INTMASK1 0x%02X INTMASK1 0x%02X [%d]\n",\
                _buf[0], _buf[1], _rc);\
            if (unlikely(_rc)) {\
                dev_err((me)->dev, "INTMASK write error [%d]\n", _rc);\
            }\
        } while (0)

static __inline void max77696_uic_enable_irq (struct max77696_uic *me,
    u16 irq_bits, bool forced)
{
    u8 irq1_bits = (irq_bits >> 0);
    u8 irq2_bits = (irq_bits >> 8);

    if (unlikely(!forced)) {
        if (unlikely((me->irq_unmask[0] & irq1_bits) == irq1_bits &&
                     (me->irq_unmask[1] & irq2_bits) == irq2_bits)) {
            /* already unmasked */
            return;
        }
    }

    if (unlikely(!me->irq_unmask[0] && !me->irq_unmask[1])) {
        max77696_uic_reg_set_bit(me, SYSCTRL1, INT_EN, 1);
      //max77696_topsys_enable_uic_edge_wakeup(1);

        enable_irq(me->irq);
      //enable_irq_wake(me->irq);
    }

    /* set enabled flag */
    me->irq_unmask[0] |= irq1_bits;
    me->irq_unmask[1] |= irq2_bits;
    max77696_uic_write_irq_mask(me);
}

static __inline void max77696_uic_disable_irq (struct max77696_uic *me,
    u16 irq_bits, bool forced)
{
    u8 irq1_bits = (irq_bits >> 0);
    u8 irq2_bits = (irq_bits >> 8);

    if (unlikely(!forced)) {
        if (unlikely((me->irq_unmask[0] & irq1_bits) == 0 &&
                     (me->irq_unmask[1] & irq2_bits) == 0)) {
            /* already masked */
            return;
        }
    }

    /* clear enabled flag */
    me->irq_unmask[0] &= ~irq1_bits;
    me->irq_unmask[1] &= ~irq2_bits;

    if (unlikely(!me->irq_unmask[0] && !me->irq_unmask[1])) {
        max77696_uic_reg_set_bit(me, SYSCTRL1, INT_EN, 0);
      //max77696_topsys_enable_uic_edge_wakeup(0);

      //disable_irq_wake(me->irq);
        disable_irq(me->irq);
    }

    max77696_uic_write_irq_mask(me);
}

static __inline const char* max77696_uic_chgtype_string (u8 chg_type)
{
    switch (chg_type) {
    case MAX77696_UIC_CHGTYPE_USB:
        return "USB Cable";
    case MAX77696_UIC_CHGTYPE_CDP:
        return "Charging Downstream Port";
    case MAX77696_UIC_CHGTYPE_DEDICATED_1P5A:
        return "Dedicated Charger";
    case MAX77696_UIC_CHGTYPE_APPLE_0P5AC:
    case MAX77696_UIC_CHGTYPE_APPLE_1P0AC:
    case MAX77696_UIC_CHGTYPE_APPLE_2P0AC:
        return "Apple Charger";
    case MAX77696_UIC_CHGTYPE_OTH_0:
    case MAX77696_UIC_CHGTYPE_OTH_1:
        return "Other Charger";
    case MAX77696_UIC_CHGTYPE_SELFENUM_0P5AC:
        return "Self Enumerated";
    }

    return "(unknown)";
}

static __inline const char* max77696_uic_iset_string (u8 i_set)
{
    switch (i_set) {
    case 0b000: return "0.0";
    case 0b001: return "0.1";
    case 0b010: return "0.4";
    case 0b011: return "0.5";
    case 0b100: return "1.0";
    case 0b101: return "1.5";
    case 0b110: return "2.0";
    }

    return "(unknown)";
}

static void max77696_uic_psy_work (struct work_struct *work)
{
    struct max77696_uic *me = __psy_work_to_max77696_uic(work);
    u8 chg_type, status[2], manctrl;

    __lock(me);

    chg_type = me->chg_type;

    max77696_uic_reg_bulk_read(me, STATUS1, status, 2);

    me->vb_volt  = UIC_REG_BITGET(STATUS1, VBVOLT, status[0]);
    me->chg_type = UIC_REG_BITGET(STATUS1, CHGTYP, status[0]);
    me->adc_code = UIC_REG_BITGET(STATUS2, ADC,    status[1]);

    if (unlikely(!me->vb_volt)) {
        /* This time is when the cable is removed. */
        me->chg_type = 0;
        me->adc_code = 0;
        me->i_set    = 0;
        goto out;
    }

    max77696_uic_reg_read(me, MANCTRL, &manctrl);
    me->i_set = UIC_REG_BITGET(MANCTRL, ISET, manctrl);

out:
    dev_dbg(me->dev, "VBVOLT 0x%X CHGTYP 0x%X ADC 0x%X ISET 0x%X",
        me->vb_volt, me->chg_type, me->adc_code, me->i_set);
    dev_dbg(me->dev, "Detected USB Charger  %s\n",
        max77696_uic_chgtype_string(me->chg_type));
    dev_dbg(me->dev, "Input Current Limit   %sA\n",
        max77696_uic_iset_string(me->i_set));
    if (likely(me->chg_type != chg_type)) {
        if (likely(me->uic_notify)) {
            struct max77696_uic_notify noti = {
                .vb_volt  = me->vb_volt,
                .chg_type = me->chg_type,
                .adc_code = me->adc_code,
                .i_set    = me->i_set,
            };
            me->uic_notify(&noti);
        }
    }
    __unlock(me);
    return;
}

static irqreturn_t max77696_uic_isr (int irq, void *data)
{
    struct max77696_uic *me = data;

    /* read INT register to clear bits ASAP */
    max77696_uic_reg_bulk_read(me, INT1, me->interrupted, 2);
    dev_dbg(me->dev, "INT1 %02X EN %02X\n",
        me->interrupted[0], me->irq_unmask[0]);
    dev_dbg(me->dev, "INT2 %02X EN %02X\n",
        me->interrupted[1], me->irq_unmask[1]);

    schedule_delayed_work(&(me->psy_work), HZ/10);

    return IRQ_HANDLED;
}

/* The following lists will be propulated with the UIC device parameters
 * and read/write function pointers
 */

#define MAX77696_UIC_ATTR(name, mode, reg, bits) \
static ssize_t max77696_uic_##name##_show (struct device *dev,\
    struct device_attribute *devattr, char *buf)\
{\
    struct platform_device *pdev = to_platform_device(dev);\
    struct max77696_uic *me = platform_get_drvdata(pdev);\
    u8 val;\
    int rc;\
    __lock(me);\
    rc = max77696_uic_reg_get_bit(me, reg, bits, &val);\
    if (unlikely(rc)) {\
        dev_err(dev, ""#reg" read error [%d]\n", rc);\
        goto out;\
    }\
    rc = (int)snprintf(buf, PAGE_SIZE, "%u\n", val);\
out:\
    __unlock(me);\
    return (ssize_t)rc;\
}\
static ssize_t max77696_uic_##name##_store (struct device *dev,\
    struct device_attribute *devattr, const char *buf, size_t count)\
{\
    struct platform_device *pdev = to_platform_device(dev);\
    struct max77696_uic *me = platform_get_drvdata(pdev);\
    u8 val;\
    int rc;\
    __lock(me);\
    val = (u8)simple_strtoul(buf, NULL, 10);\
    rc = max77696_uic_reg_set_bit(me, reg, bits, val);\
    if (unlikely(rc)) {\
        dev_err(dev, ""#reg" write error [%d]\n", rc);\
        goto out;\
    }\
out:\
    __unlock(me);\
    return (ssize_t)count;\
}\
static DEVICE_ATTR(name, mode, max77696_uic_##name##_show,\
    max77696_uic_##name##_store)

MAX77696_UIC_ATTR(chipid,    S_IRUGO,         DEVICEID, CHIPID);
MAX77696_UIC_ATTR(chiprev,   S_IRUGO,         DEVICEID, CHIPREV);

MAX77696_UIC_ATTR(dcdtmr,    S_IRUGO,         STATUS1,  DCDTMR);
MAX77696_UIC_ATTR(chgdetact, S_IRUGO,         STATUS1,  CHGDETACT);
MAX77696_UIC_ATTR(vbvolt,    S_IRUGO,         STATUS1,  VBVOLT);
MAX77696_UIC_ATTR(chgtyp,    S_IRUGO,         STATUS1,  CHGTYP);

MAX77696_UIC_ATTR(adcerror,  S_IRUGO,         STATUS2,  ADCERROR);
MAX77696_UIC_ATTR(enustat,   S_IRUGO,         STATUS2,  ENUSTAT);
MAX77696_UIC_ATTR(adc,       S_IRUGO,         STATUS2,  ADC);

MAX77696_UIC_ATTR(usbswc,    S_IWUSR|S_IRUGO, SYSCTRL1, USBSWC);
MAX77696_UIC_ATTR(low_pow,   S_IWUSR|S_IRUGO, SYSCTRL1, LOW_POW);

MAX77696_UIC_ATTR(adc_deb,   S_IWUSR|S_IRUGO, SYSCTRL2, ADC_DEB);
MAX77696_UIC_ATTR(dcdcpl,    S_IWUSR|S_IRUGO, SYSCTRL2, DCDCPL);
MAX77696_UIC_ATTR(idautoswc, S_IWUSR|S_IRUGO, SYSCTRL2, IDAUTOSWC);
MAX77696_UIC_ATTR(adc_en,    S_IWUSR|S_IRUGO, SYSCTRL2, ADC_EN);

MAX77696_UIC_ATTR(sfen,      S_IWUSR|S_IRUGO, CDETCTRL, SFEN);
MAX77696_UIC_ATTR(cdpdet,    S_IWUSR|S_IRUGO, CDETCTRL, CDPDET);
MAX77696_UIC_ATTR(dchktm,    S_IWUSR|S_IRUGO, CDETCTRL, DCHKTM);
MAX77696_UIC_ATTR(dcd2sct,   S_IWUSR|S_IRUGO, CDETCTRL, DCD2SCT);
MAX77696_UIC_ATTR(dcden,     S_IWUSR|S_IRUGO, CDETCTRL, DCDEN);
MAX77696_UIC_ATTR(chgtypman, S_IWUSR|S_IRUGO, CDETCTRL, CHGTYPMAN);
MAX77696_UIC_ATTR(chgdeten,  S_IWUSR|S_IRUGO, CDETCTRL, CHGDETEN);

static struct attribute *max77696_uic_attr[] = {
    &dev_attr_chipid.attr,
    &dev_attr_chiprev.attr,
    &dev_attr_dcdtmr.attr,
    &dev_attr_chgdetact.attr,
    &dev_attr_vbvolt.attr,
    &dev_attr_chgtyp.attr,
    &dev_attr_adcerror.attr,
    &dev_attr_enustat.attr,
    &dev_attr_adc.attr,
    &dev_attr_usbswc.attr,
    &dev_attr_low_pow.attr,
    &dev_attr_adc_deb.attr,
    &dev_attr_dcdcpl.attr,
    &dev_attr_idautoswc.attr,
    &dev_attr_adc_en.attr,
    &dev_attr_sfen.attr,
    &dev_attr_cdpdet.attr,
    &dev_attr_dchktm.attr,
    &dev_attr_dcd2sct.attr,
    &dev_attr_dcden.attr,
    &dev_attr_chgtypman.attr,
    &dev_attr_chgdeten.attr,
    NULL
};

static const struct attribute_group max77696_uic_attr_group = {
    .attrs = max77696_uic_attr,
};

static __devinit int max77696_uic_probe (struct platform_device *pdev)
{
    struct max77696_chip *chip = dev_get_drvdata(pdev->dev.parent);
    struct max77696_uic_platform_data *pdata = pdev->dev.platform_data;
    struct max77696_uic *me;
    u16 irq_bits;
    int rc;

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
    me->i2c  = __get_i2c(chip);
    me->dev  = &(pdev->dev);
    me->kobj = &(pdev->dev.kobj);

    me->uic_notify = pdata->uic_notify;

    me->irq = max77696_rootint_to_irq(chip, UIC);

    INIT_DELAYED_WORK(&(me->psy_work), max77696_uic_psy_work);

    rc = sysfs_create_group(me->kobj, &max77696_uic_attr_group);
    if (unlikely(rc)) {
        dev_err(me->dev, "failed to create attribute group [%d]\n", rc);
        goto out_err;
    }

    /* Disable all UIC interrupts */
    max77696_uic_reg_write(me, INTMASK1, 0x00);
    max77696_uic_reg_write(me, INTMASK2, 0x00);

    /* Initialize UIC interrupt settings */
    max77696_uic_reg_set_bit(me, SYSCTRL1, INT_EN,  0);
    max77696_uic_reg_set_bit(me, SYSCTRL1, INT_TYP, pdata->int_type);
    max77696_uic_reg_set_bit(me, SYSCTRL1, INT_DLY, pdata->int_delay);
    max77696_uic_reg_set_bit(me, SYSCTRL1, INT_POL, pdata->int_polarity);

    rc = request_threaded_irq(me->irq,
        NULL, max77696_uic_isr, IRQF_ONESHOT, DRIVER_NAME, me);

    if (unlikely(rc < 0)) {
        dev_err(me->dev, "failed to request IRQ(%d) [%d]\n", me->irq, rc);
        goto out_err_req_irq;
    }

    disable_irq(me->irq);

    /* Enable UIC interrupts we need */
    irq_bits = (UIC_INT1_VBVOLT|UIC_INT1_CHGTYP);
    max77696_uic_enable_irq(me, irq_bits, 0);

    // Initial update of UIC/Charger status
    max77696_uic_psy_work((struct work_struct *) &(me->psy_work));

    pr_info(DRIVER_DESC" "DRIVER_VERSION" Installed\n");
    return 0;

out_err_req_irq:
    sysfs_remove_group(me->kobj, &max77696_uic_attr_group);
out_err:
    mutex_destroy(&(me->lock));
    platform_set_drvdata(pdev, NULL);
    kfree(me);
    return rc;
}

static __devexit int max77696_uic_remove (struct platform_device *pdev)
{
    struct max77696_uic *me = platform_get_drvdata(pdev);

    free_irq(me->irq, me);
	cancel_delayed_work_sync(&(me->psy_work));

    sysfs_remove_group(me->kobj, &max77696_uic_attr_group);

    mutex_destroy(&(me->lock));
    platform_set_drvdata(pdev, NULL);
    kfree(me);

    return 0;
}

static struct platform_driver max77696_uic_driver = {
    .driver.name  = DRIVER_NAME,
    .driver.owner = THIS_MODULE,
    .probe        = max77696_uic_probe,
    .remove       = __devexit_p(max77696_uic_remove),
};

static __init int max77696_uic_driver_init (void)
{
    return platform_driver_register(&max77696_uic_driver);
}

static __exit void max77696_uic_driver_exit (void)
{
    platform_driver_unregister(&max77696_uic_driver);
}

late_initcall(max77696_uic_driver_init);
module_exit(max77696_uic_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);

