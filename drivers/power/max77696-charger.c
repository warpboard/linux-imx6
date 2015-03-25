/*
 * MAX77696 Main Charger Driver
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

#include <linux/power_supply.h>
#include <linux/mfd/max77696.h>

#define DRIVER_DESC    "MAX77696 Main Charger Driver"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maxim-ic.com>"
#define DRIVER_NAME    MAX77696_CHARGER_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"

#define DEGREE_SIGN_UTF8 "\xC2\xB0"
#define DEGREE_SIGN      DEGREE_SIGN_UTF8
#define DEGREE_CELSIUS   DEGREE_SIGN"C"

#define CHGA_PSY_WORK_DELAY            (HZ/10)

#define CHGA_CHG_INT_REG               0x08
#define CHGA_CHG_INT_MASK_REG          0x09

#define CHGA_CHG_INT_OK_REG            0x0A
#define CHGA_CHG_INT_OK_CHGINA_M       BIT (6)
#define CHGA_CHG_INT_OK_CHGINA_S       6
        /*
         * Single-Bit CHGINA Input Status Indicator. See CHGINA_DTLS[1:0] for
         * more information.
         * 0=The CHGINA input is invalid. CHGINA_DTLS[1:0]!=0b11.
         * 1=The CHGINA input is valid (i.e. okay). CHGINA_DTLS[1:0]=0b11.
         */
#define CHGA_CHG_INT_OK_CHG_M          BIT (4)
#define CHGA_CHG_INT_OK_CHG_S          4
        /*
         * Single-Bit Charger Status Indicator. See CHG_DTLS[3:0] for more
         * information.
         * 0=The charger has suspended charging.
         * 1=The charger is okay.
         */
#define CHGA_CHG_INT_OK_BAT_M          BIT (3)
#define CHGA_CHG_INT_OK_BAT_S          3
        /*
         * Single-Bit Battery Status Indicator. See BAT_DTLS for more
         * information.
         * 0=The battery has an issue and the charger has been suspended.
         *   BAT_DTLS[2:0]!=0b011 or 0b100.
         * 1=The battery is okay. BAT_DTLS[2:0]=0b011 or 0b100.
         */
#define CHGA_CHG_INT_OK_THM_M          BIT (2)
#define CHGA_CHG_INT_OK_THM_S          2
        /*
         * Single-Bit Thermistor Status Indicator. See the THM_DTLS[2:0] for
         * more information.
         * 0=The thermistor temperature is <T1 or >T4. THM_DTLS[2:0]!=0b000
         * 1=The thermistor temperature is inside of the allowable range for
         *   charging (i.e. okay). THM_DTLS[2:0]=0b000
         */
#define CHGA_CHG_INT_OK_SYS2_M         BIT (0)
#define CHGA_CHG_INT_OK_SYS2_S         0
        /*
         * Single-Bit SYS2 Status Indicator. See SYS2_DTLS[3:0] for more
         * information.
         * 0=Something powered by the SYS2 node has hit current limit.
         *   SYS2_DTLS[3:0]!=0b000.
         * 1=The SYS2 node is okay. SYS2_DTLS[1:0]=0b000.
         */

#define CHGA_CHG_DTLS_00_REG           0x0B
#define CHGA_CHG_DTLS_00_CHGINA_DTLS_M BITS(6,5)
#define CHGA_CHG_DTLS_00_CHGINA_DTLS_S 5
#define CHGA_CHG_DTLS_00_THM_DTLS_M    BITS(2,0)
#define CHGA_CHG_DTLS_00_THM_DTLS_S    0

#define CHGA_CHG_DTLS_01_REG           0x0C
#define CHGA_CHG_DTLS_01_TREG_M        BIT (7)
#define CHGA_CHG_DTLS_01_TREG_S        7
#define CHGA_CHG_DTLS_01_BAT_DTLS_M    BITS(6,4)
#define CHGA_CHG_DTLS_01_BAT_DTLS_S    4
#define CHGA_CHG_DTLS_01_CHG_DTLS_M    BITS(3,0)
#define CHGA_CHG_DTLS_01_CHG_DTLS_S    0

#define CHGA_CHG_DTLS_02_REG           0x0D
#define CHGA_CHG_DTLS_02_SYS2_DTLS_M   BITS(3,0)
#define CHGA_CHG_DTLS_02_SYS2_DTLS_S   0

//                                     BIT (7)
#define CHGA_INT_CHGINA                BIT (6)
//                                     BIT (5)
#define CHGA_INT_CHG                   BIT (4)
#define CHGA_INT_BAT                   BIT (3)
#define CHGA_INT_THM                   BIT (2)
//                                     BIT (1)
#define CHGA_INT_SYS2                  BIT (0)
#define CHGA_INT_ALL                   0x5D

#define CHGA_CHG_CNFG_00_REG           0x0F
#define CHGA_CHG_CNFG_00_WDTEN_M       BIT (4)
#define CHGA_CHG_CNFG_00_WDTEN_S       4
#define CHGA_CHG_CNFG_00_MODE_M        BITS(3,0)
#define CHGA_CHG_CNFG_00_MODE_S        0

#define CHGA_CHG_CNFG_01_REG           0x10
#define CHGA_CHG_CNFG_01_PQEN_M        BIT (7)
#define CHGA_CHG_CNFG_01_PQEN_S        7
#define CHGA_CHG_CNFG_01_CHG_RSTRT_M   BITS(5,4)
#define CHGA_CHG_CNFG_01_CHG_RSTRT_S   4
#define CHGA_CHG_CNFG_01_FCHGTIME_M    BITS(2,0)
#define CHGA_CHG_CNFG_01_FCHGTIME_S    0

#define CHGA_CHG_CNFG_02_REG           0x11
#define CHGA_CHG_CNFG_02_OTGA_ILIM_M   BIT (7)
#define CHGA_CHG_CNFG_02_OTGA_ILIM_S   7
#define CHGA_CHG_CNFG_02_CHG_CC_M      BITS(5,0)
#define CHGA_CHG_CNFG_02_CHG_CC_S      0

#define CHGA_CHG_CNFG_03_REG           0x12
#define CHGA_CHG_CNFG_03_TO_TIME_M     BITS(5,3)
#define CHGA_CHG_CNFG_03_TO_TIME_S     3
#define CHGA_CHG_CNFG_03_TO_ITH_M      BITS(2,0)
#define CHGA_CHG_CNFG_03_TO_ITH_S      0

#define CHGA_CHG_CNFG_04_REG           0x13
#define CHGA_CHG_CNFG_04_MINVSYS1_M    BITS(7,5)
#define CHGA_CHG_CNFG_04_MINVSYS1_S    5
#define CHGA_CHG_CNFG_04_CHG_CV_PRM_M  BITS(4,0)
#define CHGA_CHG_CNFG_04_CHG_CV_PRM_S  0

#define CHGA_CHG_CNFG_05_REG           0x14
#define CHGA_CHG_CNFG_05_CHG_CV_JTA_M  BITS(4,0)
#define CHGA_CHG_CNFG_05_CHG_CV_JTA_S  0

#define CHGA_CHG_CNFG_06_REG           0x15
#define CHGA_CHG_CNFG_06_CHGPROT_M     BITS(3,2)
#define CHGA_CHG_CNFG_06_CHGPROT_S     2
#define CHGA_CHG_CNFG_06_WDTCLR_M      BITS(1,0)
#define CHGA_CHG_CNFG_06_WDTCLR_S      0

#define CHGA_CHG_CNFG_07_REG           0x16
#define CHGA_CHG_CNFG_07_REGTEMP_M     BITS(6,5)
#define CHGA_CHG_CNFG_07_REGTEMP_S     5

#define CHGA_CHG_CNFG_08_REG           0x17

#define CHGA_CHG_CNFG_09_REG           0x18
#define CHGA_CHG_CNFG_09_CHGA_ICL_M    BIT (7)
#define CHGA_CHG_CNFG_09_CHGA_ICL_S    7
#define CHGA_CHG_CNFG_09_CHGINA_ILIM_M BITS(6,0)
#define CHGA_CHG_CNFG_09_CHGINA_ILIM_S 0

#define CHGA_CHG_CNFG_10_REG           0x19

#define CHGA_CHG_CNFG_11_REG           0x1A
#define CHGA_CHG_CNFG_11_VSYS2SET_M    BITS(6,0)
#define CHGA_CHG_CNFG_11_VSYS2SET_S    0

#define CHGA_CHG_CNFG_12_REG           0x1B
#define CHGA_CHG_CNFG_12_CHG_LPM_M     BIT (7)
#define CHGA_CHG_CNFG_12_CHG_LPM_S     7
#define CHGA_CHG_CNFG_12_VCHGIN_REG_M  BITS(4,3)
#define CHGA_CHG_CNFG_12_VCHGIN_REG_S  3
#define CHGA_CHG_CNFG_12_B2SOVRC_M     BITS(2,0)
#define CHGA_CHG_CNFG_12_B2SOVRC_S     0

#define CHGA_CHG_CNFG_13_REG           0x1C

#define CHGA_CHG_CNFG_14_REG           0x1D
#define CHGA_CHG_CNFG_14_JEITA_M       BIT (7)
#define CHGA_CHG_CNFG_14_JEITA_S       7
#define CHGA_CHG_CNFG_14_T4_M          BITS(6,5)
#define CHGA_CHG_CNFG_14_T4_S          5
#define CHGA_CHG_CNFG_14_T3_M          BITS(4,3)
#define CHGA_CHG_CNFG_14_T3_S          3
#define CHGA_CHG_CNFG_14_T2_M          BIT (2)
#define CHGA_CHG_CNFG_14_T2_S          2
#define CHGA_CHG_CNFG_14_T1_M          BITS(1,0)
#define CHGA_CHG_CNFG_14_T1_S          0

#define CHGA_REG(reg)                  ((u8)(CHGA_##reg##_REG))
#define CHGA_REG_BITMASK(reg, bit)     ((u8)(CHGA_##reg##_##bit##_M))
#define CHGA_REG_BITSHIFT(reg, bit)         (CHGA_##reg##_##bit##_S)

#define CHGA_REG_BITGET(reg, bit, val) \
        ((u8)(((val) & CHGA_REG_BITMASK(reg, bit))\
        >> CHGA_REG_BITSHIFT(reg, bit)))
#define CHGA_REG_BITSET(reg, bit, val) \
        ((u8)(((val) << CHGA_REG_BITSHIFT(reg, bit))\
        & CHGA_REG_BITMASK(reg, bit)))

/* CHGA Register Read/Write */
#define max77696_charger_reg_read(me, reg, val_ptr) \
        max77696_read((me)->i2c, CHGA_REG(reg), val_ptr)
#define max77696_charger_reg_write(me, reg, val) \
        max77696_write((me)->i2c, CHGA_REG(reg), val)
#define max77696_charger_reg_bulk_read(me, reg, dst, len) \
        max77696_bulk_read((me)->i2c, CHGA_REG(reg), dst, len)
#define max77696_charger_reg_bulk_write(me, reg, src, len) \
        max77696_bulk_write((me)->i2c, CHGA_REG(reg), src, len)
#define max77696_charger_reg_read_masked(me, reg, mask, val_ptr) \
        max77696_read_masked((me)->i2c, CHGA_REG(reg), mask, val_ptr)
#define max77696_charger_reg_write_masked(me, reg, mask, val) \
        max77696_write_masked((me)->i2c, CHGA_REG(reg), mask, val)

/* CHGA Register Single Bit Ops */
#define max77696_charger_reg_get_bit(me, reg, bit, val_ptr) \
        ({\
            int __rc = max77696_charger_reg_read_masked(me, reg,\
                CHGA_REG_BITMASK(reg, bit), val_ptr);\
            *(val_ptr) = CHGA_REG_BITGET(reg, bit, *(val_ptr));\
            __rc;\
        })
#define max77696_charger_reg_set_bit(me, reg, bit, val) \
        ({\
            max77696_charger_reg_write_masked(me, reg,\
                CHGA_REG_BITMASK(reg, bit), CHGA_REG_BITSET(reg, bit, val));\
        })

struct max77696_charger {
    struct mutex             lock;
    struct max77696_chip    *chip;
    struct max77696_i2c     *i2c;
    struct device           *dev;
    struct kobject          *kobj;

    void                     (*charger_notify) (struct power_supply*, bool, bool);

    struct power_supply      psy;
    struct delayed_work      psy_work;
    unsigned int             irq;
    u8                       irq_unmask;
    u8                       interrupted;
    bool                     pending_updated;
    bool                     chg_online, chg_enable;
    bool                     wdt_enabled;
    unsigned long            wdt_period;
    struct delayed_work      wdt_work;
};

#define __psy_to_max77696_charger(psy_ptr) \
        container_of(psy_ptr, struct max77696_charger, psy)
#define __psy_work_to_max77696_charger(psy_work_ptr) \
        container_of(psy_work_ptr, struct max77696_charger, psy_work.work)
#define __wdt_work_to_max77696_charger(wdt_work_ptr) \
        container_of(wdt_work_ptr, struct max77696_charger, wdt_work.work)

#define __get_i2c(chip)                 (&((chip)->pmic_i2c))
#define __lock(me)                      mutex_lock(&((me)->lock))
#define __unlock(me)                    mutex_unlock(&((me)->lock))

#define __search_index_of_the_smallest_in_be(_table, _val) \
        ({\
            int __i;\
            for (__i = 0; __i < ARRAY_SIZE(_table); __i++) {\
                if (_table[__i] >= _val) break;\
            }\
            ((__i >= ARRAY_SIZE(_table))? (ARRAY_SIZE(_table) - 1) : __i);\
        })

#define max77696_charger_write_irq_mask(me) \
        do {\
            u8 _buf = ((~((me)->irq_unmask)) & CHGA_INT_ALL);\
            int _rc = max77696_charger_reg_write(me, CHG_INT_MASK, _buf);\
            dev_vdbg((me)->dev, "written CHG_INT_MASK 0x%02X [%d]\n",\
                _buf, _rc);\
            if (unlikely(_rc)) {\
                dev_err((me)->dev, "CHG_INT_MASK write error [%d]\n", _rc);\
            }\
        } while (0)

static __inline void max77696_charger_enable_irq (struct max77696_charger* me,
    u8 irq_bits, bool forced)
{
    if (unlikely(!forced && (me->irq_unmask & irq_bits) == irq_bits)) {
        /* already unmasked */
        return;
    }

    /* set enabled flag */
    me->irq_unmask |= irq_bits;
    max77696_charger_write_irq_mask(me);
}

static __inline void max77696_charger_disable_irq (struct max77696_charger* me,
    u8 irq_bits, bool forced)
{
    if (unlikely(!forced && (me->irq_unmask & irq_bits) == 0)) {
        /* already masked */
        return;
    }

    /* clear enabled flag */
    me->irq_unmask &= ~irq_bits;
    max77696_charger_write_irq_mask(me);
}

static int max77696_charger_lock_config (struct max77696_charger* me,
    u8 config_reg, bool lock)
{
    #define CHGA_PROTECTABLE_CNFG_REGS ((1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<7))

    u8 config_reg_bit;
    int rc;

    config_reg_bit = (1 << (config_reg - CHGA_CHG_CNFG_00_REG));

    if (unlikely(!(CHGA_PROTECTABLE_CNFG_REGS & config_reg_bit))) {
        return 0;
    }

    rc = max77696_charger_reg_set_bit(me, CHG_CNFG_06, CHGPROT, (lock? 0 : 3));
    if (unlikely(rc)) {
        dev_err(me->dev,
            "failed to un/lock config(%02X) [%d]\n", config_reg, rc);
    }

    return rc;
}

#define max77696_charger_write_config(me, cfg_reg, cfg_bit, cfg_val) \
        ({\
            int _rc = max77696_charger_lock_config(me, CHGA_REG(cfg_reg), 0);\
            if (likely(!_rc)) {\
                _rc = max77696_charger_reg_set_bit(me,\
                    cfg_reg, cfg_bit, (u8)(cfg_val));\
                if (likely(!_rc)) {\
                    dev_vdbg((me)->dev,\
                        "set "#cfg_reg"["#cfg_bit"] %02X [%d]\n",\
                        (u8)(cfg_val)  & (CHGA_REG_BITMASK(cfg_reg, cfg_bit) >> CHGA_REG_BITSHIFT(cfg_reg, cfg_bit)),\
                        _rc);\
                } else {\
                    dev_err((me)->dev,\
                        ""#cfg_reg"_REG write error [%d]\n", _rc);\
                }\
                max77696_charger_lock_config(me, CHGA_REG(cfg_reg), 1);\
            }\
            _rc;\
        })
#define max77696_charger_read_config(me, cfg_reg, cfg_bit, cfg_val_ptr) \
        ({\
            int _rc = max77696_charger_reg_get_bit(me,\
                cfg_reg, cfg_bit, (u8*)(cfg_val_ptr));\
            if (unlikely(_rc)) {\
                dev_err((me)->dev, ""#cfg_reg"_REG read error [%d]\n", _rc);\
            }\
            dev_vdbg((me)->dev,\
                "get "#cfg_reg"["#cfg_bit"] %02X [%d]\n",\
                (u8)(*(cfg_val_ptr)), _rc);\
            _rc;\
        })

/* Set/Read charger mode */

static __inline
int max77696_charger_mode_set (struct max77696_charger* me, int mode)
{
    return max77696_charger_write_config(me, CHG_CNFG_00, MODE, mode);
}

static __inline
int max77696_charger_mode_get (struct max77696_charger* me, int *mode)
{
    return max77696_charger_read_config(me, CHG_CNFG_00, MODE, mode);
}

/* Set/read low-battery prequalification enable */

static __inline
int max77696_charger_pqen_set (struct max77696_charger* me, int enable)
{
    return max77696_charger_write_config(me, CHG_CNFG_01, PQEN, !!enable);
}

static __inline
int max77696_charger_pqen_get (struct max77696_charger* me, int *enable)
{
    int rc;
    u8 pqsel;

    rc = max77696_charger_read_config(me, CHG_CNFG_01, PQEN, &pqsel);
    if (likely(!rc)) {
        *enable = (int)(!!pqsel);
    }

    return rc;
}

/* Set/read charger restart threshold */

static __inline
int max77696_charger_rstrt_set (struct max77696_charger* me, int rstrt)
{
    return max77696_charger_write_config(me, CHG_CNFG_01, CHG_RSTRT, rstrt);
}

static __inline
int max77696_charger_rstrt_get (struct max77696_charger* me, int *rstrt)
{
    return max77696_charger_read_config(me, CHG_CNFG_01, CHG_RSTRT, rstrt);
}

/* Set/read fast-charge timer duration */

static __inline
int max77696_charger_fchgtime_set (struct max77696_charger* me, int time_min)
{
    u8 fchgtime;

    if (likely(time_min > 0)) {
        time_min = min(16*60, max(4*60, time_min));
        fchgtime = (u8)DIV_ROUND_UP(time_min, 2*60) - 1;
    } else {
        fchgtime = 0;
    }

    return max77696_charger_write_config(me, CHG_CNFG_01, FCHGTIME, fchgtime);
}

static __inline
int max77696_charger_fchgtime_get (struct max77696_charger* me, int *time_min)
{
    int rc;
    u8 fchgtime;

    rc = max77696_charger_read_config(me, CHG_CNFG_01, FCHGTIME, &fchgtime);
    if (likely(!rc)) {
        *time_min = (int)(fchgtime? ((fchgtime + 1) << 1) : 0) * 60;
    }

    return rc;
}

/* Set/read constant current level */

static __inline
int max77696_charger_cc_sel_set (struct max77696_charger* me, int uA)
{
    u8 cc_sel;

    uA     = min(2100000, max(66667, uA));
    cc_sel = (u8)DIV_ROUND_UP(uA * 3, 100000);

    return max77696_charger_write_config(me, CHG_CNFG_02, CHG_CC, cc_sel);
}

static __inline
int max77696_charger_cc_sel_get (struct max77696_charger* me, int *uA)
{
    int rc;
    u8 cc_sel;

    rc = max77696_charger_read_config(me, CHG_CNFG_02, CHG_CC, &cc_sel);
    if (likely(!rc)) {
        cc_sel = ((cc_sel > 2)? cc_sel : 2);
        *uA    = DIV_ROUND_UP((int)cc_sel * 100000, 3); /* in 33.3mA steps */
    }

    return rc;
}

static __inline
int max77696_charger_to_time_set (struct max77696_charger* me, int time_min)
{
    u8 to_time;

    time_min = min(70, max(0, time_min));
    to_time  = (u8)DIV_ROUND_UP(time_min, 10);

    return max77696_charger_write_config(me, CHG_CNFG_03, TO_TIME, to_time);
}

static __inline
int max77696_charger_to_time_get (struct max77696_charger* me, int *time_min)
{
    int rc;
    u8 to_time;

    rc = max77696_charger_read_config(me, CHG_CNFG_03, TO_TIME, &to_time);
    if (likely(!rc)) {
        *time_min = (int)to_time * 10;
    }

    return rc;
}

static __inline
int max77696_charger_to_ith_set (struct max77696_charger* me, int uA)
{
    u8 to_ith;

    uA     = min(125000, max(50000, uA));
    to_ith = (u8)DIV_ROUND_UP(uA - 50000, 25000);

    return max77696_charger_write_config(me, CHG_CNFG_03, TO_ITH, to_ith);
}

static __inline
int max77696_charger_to_ith_get (struct max77696_charger* me, int *uA)
{
    int rc;
    u8 to_ith;

    rc = max77696_charger_read_config(me, CHG_CNFG_03, TO_ITH, &to_ith);
    if (likely(!rc)) {
        *uA = (int)to_ith * 25000 + 50000; /* offset 50mA, step 25mA */
    }

    return rc;
}

/* Set/read minimum system regulation voltage */

static __inline
int max77696_charger_minvsys1_set (struct max77696_charger* me, int mV)
{
    u8 minvsys1;

    mV       = min(3700, max(3000, mV));
    minvsys1 = (u8)DIV_ROUND_UP(mV - 3000, 100);

    return max77696_charger_write_config(me, CHG_CNFG_04, MINVSYS1, minvsys1);
}

static __inline
int max77696_charger_minvsys1_get (struct max77696_charger* me, int *mV)
{
    int rc;
    u8 minvsys1;

    rc = max77696_charger_read_config(me, CHG_CNFG_04, MINVSYS1, &minvsys1);
    if (likely(!rc)) {
        *mV = (int)minvsys1 * 100 + 3000; /* offset 3V, step 0.1V */
    }

    return rc;
}

/* Set/read constant voltage level */

static const int mV_to_cv_level[] =
{
    3650, 3675, 3700, 3725, 3750, 3775, 3800, 3825, 3850, 3875, 3900, 3925,
    3950, 3975, 4000, 4025, 4050, 4075, 4100, 4125, 4150, 4175, 4200, 4225,
    4250, 4275, 4300, 4325, 4340, 4350, 4375, 4400,
};

static __inline
int max77696_charger_cv_prm_set (struct max77696_charger* me, int mV)
{
    u8 cv_prm;

    cv_prm = (u8)__search_index_of_the_smallest_in_be(mV_to_cv_level, mV);

    return max77696_charger_write_config(me, CHG_CNFG_04, CHG_CV_PRM, cv_prm);
}

static __inline
int max77696_charger_cv_prm_get (struct max77696_charger* me, int *mV)
{
    int rc;
    u8 cv_prm;

    rc = max77696_charger_read_config(me, CHG_CNFG_04, CHG_CV_PRM, &cv_prm);
    if (likely(!rc)) {
        *mV = (int)mV_to_cv_level[cv_prm];
    }

    return rc;
}

static __inline
int max77696_charger_cv_jta_set (struct max77696_charger* me, int mV)
{
    u8 cv_jta;

    cv_jta = (u8)__search_index_of_the_smallest_in_be(mV_to_cv_level, mV);

    return max77696_charger_write_config(me, CHG_CNFG_05, CHG_CV_JTA, cv_jta);
}

static __inline
int max77696_charger_cv_jta_get (struct max77696_charger* me, int *mV)
{
    int rc;
    u8 cv_jta;

    rc = max77696_charger_read_config(me, CHG_CNFG_05, CHG_CV_JTA, &cv_jta);
    if (likely(!rc)) {
        *mV = (int)mV_to_cv_level[cv_jta];
    }

    return rc;
}

static __inline
int max77696_charger_regtemp_set (struct max77696_charger* me, int temp)
{
    u8 regtemp;

    temp    = min(115, max(70, temp));
    regtemp = (u8)DIV_ROUND_UP(temp - 70, 15);

    return max77696_charger_write_config(me, CHG_CNFG_07, REGTEMP, regtemp);
}

static __inline
int max77696_charger_regtemp_get (struct max77696_charger* me, int *temp)
{
    int rc;
    u8 regtemp;

    rc = max77696_charger_read_config(me, CHG_CNFG_07, REGTEMP, &regtemp);
    if (likely(!rc)) {
        *temp = (int)regtemp * 15 + 70; /* offset 70, step 15 */
    }

    return rc;
}

/* Set/Read SYS2 target output voltage in boost mode */

static __inline
int max77696_charger_vsys2set_set (struct max77696_charger* me, int mV)
{
    u8 vsys2set;

    mV       = min(5750, max(3000, mV));
    vsys2set = (u8)DIV_ROUND_UP(mV - 3000, 25);

    return max77696_charger_write_config(me, CHG_CNFG_11, VSYS2SET, vsys2set);
}

static __inline
int max77696_charger_vsys2set_get (struct max77696_charger* me, int *mV)
{
    int rc;
    u8 vsys2set;

    rc = max77696_charger_read_config(me, CHG_CNFG_11, VSYS2SET, &vsys2set);
    if (likely(!rc)) {
        *mV = (int)vsys2set * 25;
        *mV = min(5750, max(3000, *mV));
    }

    return rc;
}

static __inline
int max77696_charger_jeita_set (struct max77696_charger* me, int enable)
{
    return max77696_charger_write_config(me, CHG_CNFG_14, JEITA, !!enable);
}

static __inline
int max77696_charger_jeita_get (struct max77696_charger* me, int *enable)
{
    int rc;
    u8 jeita;

    rc = max77696_charger_read_config(me, CHG_CNFG_14, JEITA, &jeita);
    if (likely(!rc)) {
        *enable = (int)(!!jeita);
    }

    return rc;
}

/* Set/Read hi/low temp level (T1/T4) & low temp limit (T2/T3) */

static __inline
int max77696_charger_t1_set (struct max77696_charger* me, int temp_C)
{
    u8 t1;

    temp_C = min(0, max(-10, temp_C));
    t1     = (u8)DIV_ROUND_UP(-temp_C, 5);

    return max77696_charger_write_config(me, CHG_CNFG_14, T1, t1);
}

static __inline
int max77696_charger_t1_get (struct max77696_charger* me, int *temp_C)
{
    int rc;
    u8 t1;

    rc = max77696_charger_read_config(me, CHG_CNFG_14, T1, &t1);
    if (likely(!rc)) {
        *temp_C = ((int)t1 * -5);
    }

    return rc;
}

static __inline
int max77696_charger_t2_set (struct max77696_charger* me, int temp_C)
{
    u8 t2;

    temp_C = min(15, max(10, temp_C));
    t2     = (u8)DIV_ROUND_UP(temp_C - 10, 5);

    return max77696_charger_write_config(me, CHG_CNFG_14, T2, t2);
}

static __inline
int max77696_charger_t2_get (struct max77696_charger* me, int *temp_C)
{
    int rc;
    u8 t2;

    rc = max77696_charger_read_config(me, CHG_CNFG_14, T2, &t2);
    if (likely(!rc)) {
        *temp_C = ((int)t2 * 5) + 10;
    }

    return rc;
}

static __inline
int max77696_charger_t3_set (struct max77696_charger* me, int temp_C)
{
    u8 t3;

    temp_C = min(50, max(44, temp_C));
    t3     = (u8)DIV_ROUND_UP(50 - temp_C, 2);

    return max77696_charger_write_config(me, CHG_CNFG_14, T3, t3);
}

static __inline
int max77696_charger_t3_get (struct max77696_charger* me, int *temp_C)
{
    int rc;
    u8 t3;

    rc = max77696_charger_read_config(me, CHG_CNFG_14, T3, &t3);
    if (likely(!rc)) {
        *temp_C = 50 - ((int)t3 * 2);
    }

    return rc;
}

static __inline
int max77696_charger_t4_set (struct max77696_charger* me, int temp_C)
{
    u8 t4;

    temp_C = min(60, max(54, temp_C));
    t4     = (u8)DIV_ROUND_UP(60 - temp_C, 2);

    return max77696_charger_write_config(me, CHG_CNFG_14, T4, t4);
}

static __inline
int max77696_charger_t4_get (struct max77696_charger* me, int *temp_C)
{
    int rc;
    u8 t4;

    rc = max77696_charger_read_config(me, CHG_CNFG_14, T4, &t4);
    if (likely(!rc)) {
        *temp_C = 60 - ((int)t4 * 2);
    }

    return rc;
}

/* Set/read charge safety timer */

#define max77696_charger_wdt_stop(me) \
        cancel_delayed_work_sync(&((me)->wdt_work))

#define max77696_charger_wdt_start(me) \
        if (likely((me)->wdt_enabled && (me)->wdt_period > 0)) {\
            if (likely(!delayed_work_pending(&((me)->wdt_work)))) {\
                schedule_delayed_work(&((me)->wdt_work), (me)->wdt_period);\
            }\
        }

static int max77696_charger_wdt_en_set (struct max77696_charger* me, int enable)
{
    int rc;

    max77696_charger_wdt_stop(me);

    enable = !!enable;

    rc = max77696_charger_write_config(me, CHG_CNFG_00, WDTEN, enable);
    if (unlikely(rc)) {
        goto out;
    }

    me->wdt_enabled = (bool)enable;

    max77696_charger_wdt_start(me); /* re-start wdt if enabled */

out:
    return rc;
}

static int max77696_charger_wdt_en_get (struct max77696_charger* me,
    int *enable)
{
    int rc;
    u8 wdten;

    rc = max77696_charger_read_config(me, CHG_CNFG_00, WDTEN, &wdten);
    if (unlikely(rc)) {
        goto out;
    }

    *enable = (int)(!!wdten) + (int)(!!me->wdt_enabled);

out:
    return rc;
}

static __always_inline
int max77696_charger_wdt_period_set (struct max77696_charger* me,
    int period_ms)
{
    max77696_charger_wdt_stop(me);

    me->wdt_period = ((period_ms > 0)?
        msecs_to_jiffies((unsigned int)period_ms) : 0);

    max77696_charger_wdt_start(me); /* re-start wdt if enabled */
    return 0;
}

static __always_inline
int max77696_charger_wdt_period_get (struct max77696_charger* me,
    int *period_ms)
{
    *period_ms = (int)jiffies_to_msecs(me->wdt_period);
    return 0;
}

static void max77696_charger_wdt_ping (struct max77696_charger* me)
{
    max77696_charger_reg_set_bit(me, CHG_CNFG_06, WDTCLR, 1);
    dev_vdbg(me->dev, "watchdog timer cleared\n");
}

static void max77696_charger_wdt_work (struct work_struct *work)
{
    struct max77696_charger *me = __wdt_work_to_max77696_charger(work);

    __lock(me);

    max77696_charger_wdt_ping(me);
    max77696_charger_wdt_start(me);

    __unlock(me);
    return;
}

static bool max77696_charger_update_state (struct max77696_charger* me,
    bool notify)
{
    bool online, enable;
    u8 status;
    int rc;

    /* read CHG_INT_OK as an interrupted status */
    rc = max77696_charger_reg_read(me, CHG_INT_OK, &status);
    if (unlikely(rc)) {
        dev_err(me->dev, "CHG_INT_OK read error [%d]\n", rc);
        return 0;
    }

    dev_dbg(me->dev, "CHG_INT_OK %02X\n", status);
    online = (!!CHGA_REG_BITGET(CHG_INT_OK, CHGINA, status));
    enable = (!!CHGA_REG_BITGET(CHG_INT_OK, CHG,    status));
    enable = (online && enable);

    if (unlikely(me->chg_online == online && me->chg_enable == enable)) {
        if (unlikely(!me->pending_updated)) {
            return 0;
        }
    }

    me->pending_updated = 1;

    me->chg_online      = online;
    me->chg_enable      = enable;
    dev_dbg(me->dev, "ONLINE %d ENABLE %d\n", me->chg_online, me->chg_enable);

    if (likely(notify && me->charger_notify)) {
        me->pending_updated = 0;
        me->charger_notify(&(me->psy), me->chg_online, me->chg_enable);
    }

    return 1; /* updated */
}

static void max77696_charger_psy_work (struct work_struct *work)
{
    struct max77696_charger *me = __psy_work_to_max77696_charger(work);

    __lock(me);
    max77696_charger_update_state(me, 1);
    __unlock(me);
}

#ifdef DEBUG
#define __show_details(me, num, which) \
        do {\
            u8 _dtls;\
            max77696_charger_reg_get_bit(me,\
                CHG_DTLS_0##num, which##_DTLS, &_dtls);\
            dev_info((me)->dev, "IRQ_"#which" details: %02X\n", _dtls);\
        } while (0)
#else /* DEBUG */
#define __show_details(me, num, which) \
        do { } while (0)
#endif /* DEBUG */

static irqreturn_t max77696_charger_isr (int irq, void *data)
{
    struct max77696_charger *me = data;

    /* read INT register to clear bits ASAP */
    max77696_charger_reg_read(me, CHG_INT, &(me->interrupted));
    dev_dbg(me->dev, "CHG_INT %02X EN %02X\n", me->interrupted, me->irq_unmask);
    me->interrupted &= me->irq_unmask;

    if (me->interrupted & CHGA_INT_CHGINA) {
        __show_details(me, 0, CHGINA);
        if (likely(!delayed_work_pending(&(me->psy_work)))) {
            schedule_delayed_work(&(me->psy_work), CHGA_PSY_WORK_DELAY);
        }
    }

    if (me->interrupted & CHGA_INT_THM) {
        __show_details(me, 0, THM);
        /* TODO: put the customer's code here */
    }

    if (me->interrupted & CHGA_INT_CHG) {
        __show_details(me, 1, CHG);
        /* TODO: put the customer's code here */
    }

    if (me->interrupted & CHGA_INT_BAT) {
        __show_details(me, 1, BAT);
        /* TODO: put the customer's code here */
    }

    return IRQ_HANDLED;
}

static int max77696_charger_get_property (struct power_supply *psy,
    enum power_supply_property psp, union power_supply_propval *val)
{
    struct max77696_charger *me = __psy_to_max77696_charger(psy);
    int rc = 0;

    __lock(me);

    switch (psp) {
    case POWER_SUPPLY_PROP_PRESENT:
    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = me->chg_online;
        break;

    default:
        rc = -EINVAL;
        break;
    }

    __unlock(me);
    return rc;
}

static enum power_supply_property max77696_charger_psy_props[] = {
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
};

#define MAX77696_CHARGER_ATTR(name, unitstr) \
static ssize_t max77696_charger_##name##_show (struct device *dev,\
    struct device_attribute *devattr, char *buf)\
{\
    struct platform_device *pdev = to_platform_device(dev);\
    struct max77696_charger *me = platform_get_drvdata(pdev);\
    int val = 0, rc;\
    __lock(me);\
    rc = max77696_charger_##name##_get(me, &val);\
    if (unlikely(rc)) {\
        goto out;\
    }\
    rc = (int)snprintf(buf, PAGE_SIZE, "%d"unitstr"\n", val);\
out:\
    __unlock(me);\
    return (ssize_t)rc;\
}\
static ssize_t max77696_charger_##name##_store (struct device *dev,\
    struct device_attribute *devattr, const char *buf, size_t count)\
{\
    struct platform_device *pdev = to_platform_device(dev);\
    struct max77696_charger *me = platform_get_drvdata(pdev);\
    int val, rc;\
    __lock(me);\
    val = (int)simple_strtol(buf, NULL, 10);\
    rc = max77696_charger_##name##_set(me, val);\
    if (unlikely(rc)) {\
        goto out;\
    }\
out:\
    __unlock(me);\
    return (ssize_t)count;\
}\
static DEVICE_ATTR(name, S_IWUSR|S_IRUGO, max77696_charger_##name##_show,\
    max77696_charger_##name##_store)

MAX77696_CHARGER_ATTR(mode,       ""    );
MAX77696_CHARGER_ATTR(pqen,       ""    );
MAX77696_CHARGER_ATTR(cc_sel,     "uA"  );
MAX77696_CHARGER_ATTR(cv_prm,     "mV"  );
MAX77696_CHARGER_ATTR(cv_jta,     "mV"  );
MAX77696_CHARGER_ATTR(jeita,      ""    );
MAX77696_CHARGER_ATTR(t1,         "C"   );
MAX77696_CHARGER_ATTR(t2,         "C"   );
MAX77696_CHARGER_ATTR(t3,         "C"   );
MAX77696_CHARGER_ATTR(t4,         "C"   );
MAX77696_CHARGER_ATTR(wdt_en,     ""    );
MAX77696_CHARGER_ATTR(wdt_period, "msec");

static struct attribute *max77696_charger_attr[] = {
    &dev_attr_mode.attr,
    &dev_attr_pqen.attr,
    &dev_attr_cc_sel.attr,
    &dev_attr_cv_prm.attr,
    &dev_attr_cv_jta.attr,
    &dev_attr_jeita.attr,
    &dev_attr_t1.attr,
    &dev_attr_t2.attr,
    &dev_attr_t3.attr,
    &dev_attr_t4.attr,
    &dev_attr_wdt_en.attr,
    &dev_attr_wdt_period.attr,
    NULL
};

static const struct attribute_group max77696_charger_attr_group = {
    .attrs = max77696_charger_attr,
};

static __devinit int max77696_charger_probe (struct platform_device *pdev)
{
    struct max77696_chip *chip = dev_get_drvdata(pdev->dev.parent);
    struct max77696_charger_platform_data *pdata = pdev->dev.platform_data;
    struct max77696_charger *me;
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

    me->charger_notify = pdata->charger_notify;

    me->irq = max77696_rootint_to_irq(chip, CHGA);

    /* Disable all charger interrupts */
    max77696_charger_disable_irq(me, CHGA_INT_ALL, 1);

    me->psy.name            = MAX77696_PSY_CHG_NAME;
    me->psy.type            = POWER_SUPPLY_TYPE_MAINS;
    me->psy.get_property    = max77696_charger_get_property;
    me->psy.properties      = max77696_charger_psy_props;
    me->psy.num_properties  = ARRAY_SIZE(max77696_charger_psy_props);
    me->psy.supplied_to     = pdata->batteries;
    me->psy.num_supplicants = pdata->num_batteries;

    INIT_DELAYED_WORK(&(me->psy_work), max77696_charger_psy_work);
    INIT_DELAYED_WORK(&(me->wdt_work), max77696_charger_wdt_work);

    /* Initial configurations */
    dev_dbg(me->dev, "Temperture threshold         T1 %3d"DEGREE_CELSIUS
                                                 " T4 %3d"DEGREE_CELSIUS"\n",
        pdata->t1_C, pdata->t4_C);
    dev_dbg(me->dev, "Temperture threshold         T2 %3d"DEGREE_CELSIUS
                                                 " T3 %3d"DEGREE_CELSIUS"\n",
        pdata->t2_C, pdata->t3_C);
    max77696_charger_t1_set(me, pdata->t1_C);
    max77696_charger_t4_set(me, pdata->t4_C);
    max77696_charger_t2_set(me, pdata->t2_C);
    max77696_charger_t3_set(me, pdata->t3_C);

    dev_dbg(me->dev, "Initial mode                 %s\n",
        (pdata->initial_mode == MAX77696_CHARGER_MODE_OFF    )? "OFF" :
        (pdata->initial_mode == MAX77696_CHARGER_MODE_CHG    )? "CHG" :
        (pdata->initial_mode == MAX77696_CHARGER_MODE_OTG    )? "OTG" :
        (pdata->initial_mode == MAX77696_CHARGER_MODE_ENBUCK )? "ENBUCK" :
        (pdata->initial_mode == MAX77696_CHARGER_MODE_ENBOOST)? "ENBOOST" :
        "(invalid)");
    dev_dbg(me->dev, "SYS2 target output voltage   %d mV\n", pdata->vsys2set_mV);
    max77696_charger_mode_set(me, pdata->initial_mode);
    max77696_charger_vsys2set_set(me, pdata->vsys2set_mV);

    /* Setting parameters by considering the characteristics of the BATTERY */
    if (likely((unsigned)(pdata->rstrt ) <= 2)) {
        dev_dbg(me->dev, "Charger restart threshold    %d mV\n",
            (pdata->rstrt == MAX77696_CHARGER_RSTRT_100MV)? pdata->cv_prm_mV - 100 :
            (pdata->rstrt == MAX77696_CHARGER_RSTRT_150MV)? pdata->cv_prm_mV - 150 :
            (pdata->rstrt == MAX77696_CHARGER_RSTRT_200MV)? pdata->cv_prm_mV - 200 :
            -1);
    } else {
        dev_dbg(me->dev, "Charger restart threshold    -\n");
    }
    dev_dbg(me->dev, "Fast-charge timer duration   %d min\n",
        pdata->fchgtime_min);
    dev_dbg(me->dev, "Fast-charge current          %d uA\n",
        pdata->cc_uA);
    dev_dbg(me->dev, "Top-off timer setting        %d min\n",
        pdata->to_time_min);
    dev_dbg(me->dev, "Top-off current threshold    %d uA\n",
        pdata->to_ith_uA);
    dev_dbg(me->dev, "Battery regulation voltage   %d mV (JEITA = 0)\n",
        pdata->cv_prm_mV);
    dev_dbg(me->dev, "Battery regulation voltage   %d mV (JEITA = 1)\n",
        pdata->cv_jta_mV);

    max77696_charger_rstrt_set(me, pdata->rstrt);
    max77696_charger_fchgtime_set(me, pdata->fchgtime_min);
    max77696_charger_cc_sel_set(me, pdata->cc_uA);
    max77696_charger_to_time_set(me, pdata->to_time_min);
    max77696_charger_to_ith_set(me, pdata->to_ith_uA);
    max77696_charger_cv_prm_set(me, pdata->cv_prm_mV);
    max77696_charger_cv_jta_set(me, pdata->cv_jta_mV);

    /* Setting parameters by considering the characteristics of the SYSTEM */
    dev_dbg(me->dev, "Low-battery prequalification %s\n",
        pdata->pqen? "enabled" : "disabled");
    dev_dbg(me->dev, "Min. sys regulation voltage  %d mV\n",
        pdata->minvsys1_mV);
    dev_dbg(me->dev, "Thermal regulation setpoint  %d "DEGREE_CELSIUS"\n",
        pdata->regtemp_C);

    max77696_charger_pqen_set(me, pdata->pqen);
    max77696_charger_minvsys1_set(me, pdata->minvsys1_mV);
    max77696_charger_regtemp_set(me, pdata->regtemp_C);

    /* First time update */
    max77696_charger_update_state(me, 0);

    rc = sysfs_create_group(me->kobj, &max77696_charger_attr_group);
    if (unlikely(rc)) {
        dev_err(me->dev, "failed to create attribute group [%d]\n", rc);
        goto out_err_sysfs;
    }

    rc = power_supply_register(me->dev, &(me->psy));
    if (unlikely(rc)) {
        dev_err(me->dev, "failed to register MAIN psy device [%d]\n", rc);
        goto out_err_reg_psy;
    }

    /* Request charger interrupt */

    rc = request_threaded_irq(me->irq,
        NULL, max77696_charger_isr, IRQF_ONESHOT, DRIVER_NAME, me);

    if (unlikely(rc < 0)) {
        dev_err(me->dev, "failed to request IRQ(%d) [%d]\n", me->irq, rc);
        goto out_err_req_chg_irq;
    }

    /* Configure wakeup capable */
    max77696_chip_set_wakeup(me->dev, pdata->wakeup_irq);

    BUG_ON(chip->chg_ptr);
    chip->chg_ptr = me;

    /* Enable charger interrupts we need */
    max77696_charger_enable_irq(me,
        CHGA_INT_CHGINA | CHGA_INT_BAT | CHGA_INT_THM, 0);

    /* Set up Main Charger Watchdog */
    if (pdata->wdt_period_ms > 0) {
        dev_info(me->dev, "kicking watchdog every %d msec\n",
            pdata->wdt_period_ms);
        max77696_charger_wdt_period_set(me, pdata->wdt_period_ms);
        max77696_charger_wdt_en_set(me, 1);
    } else {
        dev_warn(me->dev, "watchdog disabled\n");
        max77696_charger_wdt_period_set(me, 0);
        max77696_charger_wdt_en_set(me, 0);
    }

    max77696_charger_update_state(me, 1);

    pr_info(DRIVER_DESC" "DRIVER_VERSION" Installed\n");
    return 0;

out_err_req_chg_irq:
    power_supply_unregister(&(me->psy));
out_err_reg_psy:
    sysfs_remove_group(me->kobj, &max77696_charger_attr_group);
out_err_sysfs:
    mutex_destroy(&(me->lock));
    platform_set_drvdata(pdev, NULL);
    kfree(me);
    return rc;
}

static __devexit int max77696_charger_remove (struct platform_device *pdev)
{
    struct max77696_charger *me = platform_get_drvdata(pdev);

    me->chip->chg_ptr = NULL;

    free_irq(me->irq, me);
    cancel_delayed_work_sync(&(me->psy_work));
    max77696_charger_wdt_stop(me);

    sysfs_remove_group(me->kobj, &max77696_charger_attr_group);
    power_supply_unregister(&(me->psy));

    mutex_destroy(&(me->lock));
    platform_set_drvdata(pdev, NULL);
    kfree(me);

    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_charger_suspend (struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_charger *me = platform_get_drvdata(pdev);

    if (likely(device_may_wakeup(dev))) {
        enable_irq_wake(me->irq);
    }
    return 0;
}

static int max77696_charger_resume (struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_charger *me = platform_get_drvdata(pdev);

    if (likely(device_may_wakeup(dev))) {
        disable_irq_wake(me->irq);
    }
    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_charger_pm,
    max77696_charger_suspend, max77696_charger_resume);

static struct platform_driver max77696_charger_driver = {
    .driver.name  = DRIVER_NAME,
    .driver.owner = THIS_MODULE,
    .driver.pm    = &max77696_charger_pm,
    .probe        = max77696_charger_probe,
    .remove       = __devexit_p(max77696_charger_remove),
};

static __init int max77696_charger_driver_init (void)
{
    return platform_driver_register(&max77696_charger_driver);
}

static __exit void max77696_charger_driver_exit (void)
{
    platform_driver_unregister(&max77696_charger_driver);
}

late_initcall(max77696_charger_driver_init);
module_exit(max77696_charger_driver_exit);

/*******************************************************************************
 * EXTERNAL FUNCTIONS
 ******************************************************************************/

extern struct max77696_chip* max77696;

int max77696_charger_set_mode (int mode)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    rc = max77696_charger_mode_set(me, mode);
    __unlock(me);

    return rc;
}
EXPORT_SYMBOL(max77696_charger_set_mode);

int max77696_charger_get_mode (int *mode)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    rc = max77696_charger_mode_get(me, mode);
    __unlock(me);

    return rc;
}
EXPORT_SYMBOL(max77696_charger_get_mode);

int max77696_charger_set_pq_en (int enable)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    rc = max77696_charger_pqen_set(me, enable);
    __unlock(me);

    return rc;
}
EXPORT_SYMBOL(max77696_charger_set_pq_en);

int max77696_charger_get_pq_en (int *enable)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    rc = max77696_charger_pqen_get(me, enable);
    __unlock(me);

    return rc;
}
EXPORT_SYMBOL(max77696_charger_get_pq_en);

int max77696_charger_set_cc_level (int uA)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    rc = max77696_charger_cc_sel_set(me, uA);
    __unlock(me);

    return rc;
}
EXPORT_SYMBOL(max77696_charger_set_cc_level);

int max77696_charger_get_cc_level (int *uA)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    rc = max77696_charger_cc_sel_get(me, uA);
    __unlock(me);

    return rc;
}
EXPORT_SYMBOL(max77696_charger_get_cc_level);

int max77696_charger_set_jeita_en (int enable)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    rc = max77696_charger_jeita_set(me, enable);
    __unlock(me);

    return rc;
}
EXPORT_SYMBOL(max77696_charger_set_jeita_en);

int max77696_charger_get_jeita_en (int *enable)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    rc = max77696_charger_jeita_get(me, enable);
    __unlock(me);

    return rc;
}
EXPORT_SYMBOL(max77696_charger_get_jeita_en);

int max77696_charger_set_cv_level (bool jeita, int mV)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    if (jeita) {
        rc = max77696_charger_cv_jta_set(me, mV);
    } else {
        rc = max77696_charger_cv_prm_set(me, mV);
    }
    __unlock(me);

    return rc;
}
EXPORT_SYMBOL(max77696_charger_set_cv_level);

int max77696_charger_get_cv_level (bool jeita, int *mV)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    if (jeita) {
        rc = max77696_charger_cv_jta_get(me, mV);
    } else {
        rc = max77696_charger_cv_prm_get(me, mV);
    }
    __unlock(me);

    return rc;
}
EXPORT_SYMBOL(max77696_charger_get_cv_level);

int max77696_charger_set_temp_thres (int t_id, int temp_C)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    if (t_id == 1) {
        rc = max77696_charger_t1_set(me, temp_C);
    } else if (t_id == 2) {
        rc = max77696_charger_t2_set(me, temp_C);
    } else if (t_id == 3) {
        rc = max77696_charger_t3_set(me, temp_C);
    } else if (t_id == 4) {
        rc = max77696_charger_t4_set(me, temp_C);
    } else {
        rc = -EINVAL;
    }
    __unlock(me);

    return rc;
}
EXPORT_SYMBOL(max77696_charger_set_temp_thres);

int max77696_charger_get_temp_thres (int t_id, int *temp_C)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    if (t_id == 1) {
        rc = max77696_charger_t1_get(me, temp_C);
    } else if (t_id == 2) {
        rc = max77696_charger_t2_get(me, temp_C);
    } else if (t_id == 3) {
        rc = max77696_charger_t3_get(me, temp_C);
    } else if (t_id == 4) {
        rc = max77696_charger_t4_get(me, temp_C);
    } else {
        rc = -EINVAL;
    }
    __unlock(me);

    return rc;
}
EXPORT_SYMBOL(max77696_charger_get_temp_thres);

int max77696_charger_set_wdt_period (int period_ms)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    rc = max77696_charger_wdt_period_set(me, period_ms);
    __unlock(me);

    return 0;
}
EXPORT_SYMBOL(max77696_charger_set_wdt_period);

int max77696_charger_get_wdt_period (int *period_ms)
{
    struct max77696_chip *chip = max77696;
    struct max77696_charger *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->chg_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_charger is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);
    rc = max77696_charger_wdt_period_get(me, period_ms);
    __unlock(me);

    return rc;
}
EXPORT_SYMBOL(max77696_charger_get_wdt_period);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);

