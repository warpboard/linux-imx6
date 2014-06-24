/*
 * MAX77696 Fuel Gauge Driver
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
//#define FUNCTION_CALL_DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>

#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/syscalls.h>
#include <linux/mfd/max77696.h>
#undef   current

#define DRIVER_DESC    "MAX77696 Fuel Gauge Driver"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maxim-ic.com>"
#define DRIVER_NAME    MAX77696_GAUGE_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"

#define DEGREE_SIGN_UTF8 "\xC2\xB0"
#define DEGREE_SIGN      DEGREE_SIGN_UTF8
#define DEGREE_CELSIUS   DEGREE_SIGN"C"

#define GAUGE_DEFAULT_SNS_RESISTOR_UOHM    10000 /* micro-ohms */
#define GAUGE_DEFAULT_UPD_INTERVAL_MSEC    5000
#define GAUGE_DEFAULT_UPD_INTERVAL_JIFFIES \
        msecs_to_jiffies(GAUGE_DEFAULT_UPD_INTERVAL_MSEC)
#define GAUGE_CAP_PARAM_NR_REGS            MAX77696_GAUGE_CAP_PARAM_NR_INFOS

#define FG_STATUS_POR            BIT( 1) /* Power-On Reset */
#define FG_STATUS_BST            BIT( 3) /* Battery Status */
#define FG_STATUS_VMN            BIT( 8) /* Minimum V_alrt Threshold Exceeded */
#define FG_STATUS_TMN            BIT( 9) /* Minimum T_alrt Threshold Exceeded */
#define FG_STATUS_SMN            BIT(10) /* Minimum SOC_alrt Threshold Exceeded */
#define FG_STATUS_BI             BIT(11) /* Battery Insertion */
#define FG_STATUS_VMX            BIT(12) /* Maximum V_alrt Threshold Exceeded */
#define FG_STATUS_TMX            BIT(13) /* Maximum T_alrt Threshold Exceeded */
#define FG_STATUS_SMX            BIT(14) /* Maximum SOC_alrt Threshold Exceeded */
#define FG_STATUS_BR             BIT(15) /* Battery Removal */

#define FG_CONFIG_BER            BIT( 0) /* Enable alert on battery removal */
#define FG_CONFIG_BEI            BIT( 1) /* Enable alert on battery insertion */
#define FG_CONFIG_AEN            BIT( 2) /* Enable alert on fuel-gauge outputs */
#define FG_CONFIG_FTHRM          BIT( 3) /* Force Thermistor Bias Switch */
#define FG_CONFIG_ETHRM          BIT( 4) /* Enable Thermistor */
#define FG_CONFIG_ALSH           BIT( 5)
#define FG_CONFIG_I2CSH          BIT( 6) /* I2C Shutdown */
#define FG_CONFIG_SHDN           BIT( 7) /* Shutdown */
#define FG_CONFIG_TEX            BIT( 8) /* Temperature External */
#define FG_CONFIG_TEN            BIT( 9) /* Enable Temperature Channel */
#define FG_CONFIG_AINSH          BIT(10) /* Enable Fuel Gauge shutdown when the battery is removed */
#define FG_CONFIG_ALRTP          BIT(11)
#define FG_CONFIG_VS             BIT(12) /* Voltage ALRT Sticky */
#define FG_CONFIG_TS             BIT(13) /* Temperature ALRT Sticky */
#define FG_CONFIG_SS             BIT(14) /* SOC ALRT Sticky */

#define FG_STATUS_REG            0x00
#define FG_VALRT_TH_REG          0x01
#define FG_TALRT_TH_REG          0x02
#define FG_SALRT_TH_REG          0x03
#define FG_ATRATE_REG            0x04
#define FG_REMCAPREP_REG         0x05
#define FG_SOCREP_REG            0x06
#define FG_AGE_REG               0x07
#define FG_TEMP_REG              0x08
#define FG_VCELL_REG             0x09
#define FG_CURRENT_REG           0x0A
#define FG_AVGCURRENT_REG        0x0B
//                               0x0C
#define FG_SOCMIX_REG            0x0D
#define FG_SOCAV_REG             0x0E
#define FG_REMCAPMIX_REG         0x0F
#define FG_FULLCAP_REG           0x10
#define FG_TTE_REG               0x11
#define FG_QRESIDUAL00_REG       0x12
#define FG_FULLSOCTHR_REG        0x13
//                               0x14
//                               0x15
#define FG_AVGTA_REG             0x16
#define FG_CYCLES_REG            0x17
#define FG_DESIGNCAP_REG         0x18
#define FG_AVGVCELL_REG          0x19
#define FG_MINMAXTEMP_REG        0x1A
#define FG_MINMAXVOLT_REG        0x1B
#define FG_MINMAXCURR_REG        0x1C
#define FG_CONFIG_REG            0x1D
#define FG_ICHGTERM_REG          0x1E
#define FG_REMCAPAV_REG          0x1F
//                               0x20
#define FG_VERSION_REG           0x21
#define FG_QRESIDUAL10_REG       0x22
#define FG_FULLCAPNOM_REG        0x23
#define FG_TEMPNOM_REG           0x24
#define FG_TEMPLIM_REG           0x25
//                               0x26
#define FG_AIN_REG               0x27
#define FG_LEARNCFG_REG          0x28
#define FG_FILTERCFG_REG         0x29
#define FG_RELAXCFG_REG          0x2A
#define FG_MISCCFG_REG           0x2B
#define FG_TGAIN_REG             0x2C
#define FG_TOFF_REG              0x2D
#define FG_CGAIN_REG             0x2E
#define FG_COFF_REG              0x2F
//                               0x30
//                               0x31
#define FG_QRESIDUAL20_REG       0x32
//                               0x33
//                               0x34
#define FG_FULLCAP0_REG          0x35
#define FG_IAVG_EMPTY_REG        0x36
#define FG_FCTC_REG              0x37
#define FG_RCOMP0_REG            0x38
#define FG_TEMPCO_REG            0x39
#define FG_V_EMPTY_REG           0x3A
//                               0x3B
//                               0x3C
#define FG_FSTAT_REG             0x3D
#define FG_TIMER_REG             0x3E
#define FG_SHDNTIMER_REG         0x3F
//                               0x40
//                               0x41
#define FG_QRESIDUAL30_REG       0x42
//                               0x43
//                               0x44
#define FG_DQACC_REG             0x45
#define FG_DPACC_REG             0x46
//                               0x47
#define FG_VFSOC0_REG            0x48
//                               0x49
//                               0x4A
//                               0x4B
//                               0x4C
#define FG_QH_REG                0x4D
//                               0x4E
//                               0x4F
//                               0x50 ... 0x5F
#define FG_VFSOC0_LOCK_REG       0x60
//                               0x61
#define FG_MODEL_LOCK1_REG       0x62
#define FG_MODEL_LOCK2_REG       0x63
//                               0x64
//                               0x65
//                               0x66
//                               0x67
//                               0x68
//                               0x69
//                               0x6A
//                               0x6B
//                               0x6C
//                               0x6D
//                               0x6E
//                               0x6F
//                               0x70 ... 0x7F
#define FG_CHAR_TBL_REG          0x80
// CHARACTERIZATION TABLE ROW 1  0x80 ... 0x8F
// CHARACTERIZATION TABLE ROW 2  0x90 ... 0x9F
// CHARACTERIZATION TABLE ROW 3  0xA0 ... 0xAF
//                               0xB0 ... 0xBF
//                               0xC0 ... 0xCF
//                               0xD0 ... 0xDF
//                               0xE0 ... 0xEF
//                               0xF0
//                               0xF1
//                               0xF2
//                               0xF3
//                               0xF4
//                               0xF5
//                               0xF6
//                               0xF7
//                               0xF8
//                               0xF9
//                               0xFA
#define FG_VFOCV_REG             0xFB
//                               0xFC
//                               0xFD
#define FG_VFSOC_REG             0xFF

#define FG_CHAR_TBL_SZ           MAX77696_GAUGE_CHARACTERIZATION_TABLE_SIZE
#define FG_MODEL_SCALING         0x0001
#define FG_DP_ACC                0x3200 /* 200% */
#define FG_VFSOC0_LOCK_CODE      0x0000
#define FG_VFSOC0_UNLOCK_CODE    0x0080
#define FG_MODEL_UNLOCK_CODE1    0X0059
#define FG_MODEL_UNLOCK_CODE2    0X00C4
#define FG_MODEL_LOCK_CODE1      0X0000
#define FG_MODEL_LOCK_CODE2      0X0000

struct max77696_gauge_cache {
    unsigned long timestamp;
    unsigned long lifetime;
    int           value;
};

#define __init_cache(_me, _cache, _lifetime, _init_val) \
        ({\
            _me->_cache.lifetime  = (unsigned long)(_lifetime);\
            _me->_cache.value     = (int)(_init_val);\
            _me->_cache.timestamp = jiffies;\
            0;\
        })
#define __is_cache_valid(_me, _cache) \
        ({\
            time_before(jiffies, _me->_cache.timestamp + _me->_cache.lifetime);\
        })
#define __get_cache(_me, _cache) \
        ({\
            _me->_cache.value;\
        })
#define __set_cache(_me, _cache, _val) \
        ({\
            int __prev_val = _me->_cache.value;\
            _me->_cache.timestamp = jiffies;\
            _me->_cache.value     = (int)(_val);\
            (__prev_val != _me->_cache.value);\
        })

struct max77696_gauge {
    struct mutex                       lock;
    struct max77696_chip              *chip;
    struct max77696_i2c               *i2c;
    struct device                     *dev;
    struct kobject                    *kobj;

    struct max77696_gauge_config      *config;
    bool                               enable_por_init;

    int                                v_alert_max, v_alert_min; /* 5100 ~    0 [mV] */
    int                                t_alert_max, t_alert_min; /*  127 ~ -128 [C] */
    int                                s_alert_max, s_alert_min; /*  255 ~    0 [%] */
    bool                               enable_alert_on_battery_removal;
    bool                               enable_alert_on_battery_insertion;

    int                                charge_full_design;    /* in uAh */
    int                                battery_full_capacity; /* in percent */
    int                                r_sns;

    unsigned long                      update_interval;
    u16                                polling_properties;

    bool                               support_led_triggers;
    bool                               disallow_led_trigger;
    struct max77696_gauge_led_trigger *default_trigger;
    struct max77696_gauge_led_trigger *charging_trigger;
    struct max77696_gauge_led_trigger *charging_full_trigger;

    bool                               (*battery_online) (void);
    bool                               (*charger_online) (void);
    bool                               (*charger_enable) (void);

    bool                               init_complete;
    struct work_struct                 init_work;
    struct power_supply                psy;
    struct delayed_work                psy_work;

    unsigned int                       irq;
    u16                                irq_unmask;

    /* All voltages, currents, charges, energies, time and temperatures in
     * uV, uA, uAh, uWh, seconds and tenths of degree Celsius
     */
    struct max77696_gauge_cache        socrep;
    struct max77696_gauge_cache        tte;
    struct max77696_gauge_cache        temp;
    struct max77696_gauge_cache        vcell;
    struct max77696_gauge_cache        avgvcell;
    struct max77696_gauge_cache        current;
    struct max77696_gauge_cache        avgcurrent;
    struct max77696_gauge_cache        remcaprep;
    struct max77696_gauge_cache        fullcapnom;
    struct max77696_gauge_cache        cycles;
};

#define __init_work_to_max77696_gauge(init_work_ptr) \
        container_of(init_work_ptr, struct max77696_gauge, init_work)
#define __psy_to_max77696_gauge(psy_ptr) \
        container_of(psy_ptr, struct max77696_gauge, psy)
#define __psy_work_to_max77696_gauge(psy_work_ptr) \
        container_of(psy_work_ptr, struct max77696_gauge, psy_work.work)

#define __get_i2c(chip)                  (&((chip)->gauge_i2c))
#define __lock(me)                       mutex_lock(&((me)->lock))
#define __unlock(me)                     mutex_unlock(&((me)->lock))

#define FG_REG(reg)                      ((u8) (FG_##reg##_REG))
#define FG_REG_BITMASK(reg, bit)         ((u16)(FG_##reg##_##bit##_M))
#define FG_REG_BITSHIFT(reg, bit)              (FG_##reg##_##bit##_S)

#define FG_REG_BITGET(reg, bit, val) \
        ((u8)(((val) & FG_REG_BITMASK(reg, bit))\
        >> FG_REG_BITSHIFT(reg, bit)))
#define FG_REG_BITSET(reg, bit, val) \
        ((u8)(((val) << FG_REG_BITSHIFT(reg, bit))\
        & FG_REG_BITMASK(reg, bit)))

/* GAUGE Register Read/Write */
#define max77696_gauge_reg_read(me, reg, val_ptr) \
        max77696_read((me)->i2c, FG_REG(reg), val_ptr)
#define max77696_gauge_reg_write(me, reg, val) \
        max77696_write((me)->i2c, FG_REG(reg), val)
#define max77696_gauge_reg_bulk_read(me, reg, dst, len) \
        max77696_bulk_read((me)->i2c, FG_REG(reg), dst, len)
#define max77696_gauge_reg_bulk_write(me, reg, src, len) \
        max77696_bulk_write((me)->i2c, FG_REG(reg), src, len)
#define max77696_gauge_reg_read_masked(me, reg, mask, val_ptr) \
        max77696_read_masked((me)->i2c, FG_REG(reg), mask, val_ptr)
#define max77696_gauge_reg_write_masked(me, reg, mask, val) \
        max77696_write_masked((me)->i2c, FG_REG(reg), mask, val)

/* GAUGE Register Single Bit Ops */
#define max77696_gauge_reg_get_bit(me, reg, bit, val_ptr) \
        ({\
            int __rc = max77696_gauge_reg_read_masked(me, reg,\
                FG_REG_BITMASK(reg, bit), val_ptr);\
            *(val_ptr) = FG_REG_BITGET(reg, bit, *(val_ptr));\
            __rc;\
        })
#define max77696_gauge_reg_set_bit(me, reg, bit, val) \
        ({\
            max77696_gauge_reg_write_masked(me, reg,\
                FG_REG_BITMASK(reg, bit),\
                FG_REG_BITSET(reg, bit, val));\
        })

/* GAUGE Register Word I/O */
#define max77696_gauge_reg_read_word(me, reg, val_ptr) \
        ({\
            int __rc = max77696_gauge_reg_bulk_read(me,\
                reg, (u8*)(val_ptr), 2);\
            if (unlikely(__rc)) {\
                dev_err((me)->dev, ""#reg" read error [%d]\n", __rc);\
            }\
            *(val_ptr) = __le16_to_cpu(*val_ptr);\
            __rc;\
        })
#define max77696_gauge_reg_write_word(me, reg, val) \
        ({\
            u16 __buf = __cpu_to_le16(val);\
            int __rc = max77696_gauge_reg_bulk_write(me,\
                reg, (u8*)(&__buf), 2);\
            if (unlikely(__rc)) {\
                dev_err((me)->dev, ""#reg" write error [%d]\n", __rc);\
            }\
            __rc;\
        })

/* GAUGE Register Word Maksed I/O */
#define max77696_gauge_reg_read_masked_word(me, reg, mask, val_ptr) \
        ({\
            int __rc = max77696_gauge_reg_bulk_read(me,\
                reg, (u8*)(val_ptr), 2);\
            if (unlikely(__rc)) {\
                dev_err((me)->dev, ""#reg" read error [%d]\n", __rc);\
            }\
            *(val_ptr) = (__le16_to_cpu(*val_ptr) & (mask));\
            __rc;\
        })
#define max77696_gauge_reg_write_masked_word(me, reg, mask, val) \
        ({\
            u16 __buf = __cpu_to_le16(val);\
            int __rc = max77696_gauge_reg_bulk_read(me,\
                            reg, (u8*)(&__buf), 2);\
            if (unlikely(__rc)) {\
                dev_err((me)->dev, ""#reg" read error [%d]\n", __rc);\
            }\
            __buf = __cpu_to_le16((__buf & (~(mask))) | ((val) & (mask)));\
            __rc = max77696_gauge_reg_bulk_write(me, reg, (u8*)(&__buf), 2);\
            if (unlikely(__rc)) {\
                dev_err((me)->dev, ""#reg" write error [%d]\n", __rc);\
            }\
            __rc;\
        })

static __inline
int __max77696_gauge_reg_write_word_verify (struct max77696_gauge* me,
    u8 reg, u16 val)
{
    u16 r_buf, w_buf = __cpu_to_le16(val);
    int retries, rc;

    for (retries = 3; retries > 0; retries--) {
        rc = max77696_bulk_write(me->i2c, reg, (u8*)(&w_buf), 2);
        if (unlikely(rc)) {
            dev_err(me->dev, "reg %02Xh write error [%d]\n", reg, rc);
            continue;
        }

        rc = max77696_bulk_read (me->i2c, reg, (u8*)(&r_buf), 2);
        if (unlikely(rc)) {
            dev_err(me->dev, "reg %02Xh read error [%d]\n", reg, rc);
            continue;
        }

        if (likely(r_buf == w_buf)) {
            goto out;
        }

        dev_err(me->dev, "reg %02Xh verify error [%d]\n", reg, rc);
        rc = -EIO;
    }

out:
    return rc;
}
#define max77696_gauge_reg_write_word_verify(me, reg, val) \
        __max77696_gauge_reg_write_word_verify(me, FG_REG(reg), val)

/* macros for conversion from 2-byte register value to integer */
#define __u16_to_intval(val) \
        ((int)(val))
#define __s16_to_intval(val) \
        (((val) & 0x8000)? -((int)((0x7fff & ~(val)) + 1)) : ((int)(val)))

  #define __msleep(msec) msleep_interruptible((unsigned int)(msec))
//#define __msleep(msec) msleep((unsigned int)(msec))
//#define __msleep(msec) mdelay((unsigned int)(msec))

#define FG_INT_BITS_MASK   \
        (FG_CONFIG_BER | FG_CONFIG_BEI | FG_CONFIG_AEN | FG_CONFIG_ALSH)
#define FG_INT_BITS_FILTER \
        (FG_INT_BITS_MASK & ~FG_CONFIG_ALSH)

#define max77696_gauge_write_irq_mask(me) \
        do {\
            u16 _mask = FG_INT_BITS_MASK;\
            u16 _val  = ((me)->irq_unmask & FG_INT_BITS_FILTER);\
            int _rc   = max77696_gauge_reg_write_masked_word(me,\
                CONFIG, _mask, _val);\
            if (unlikely(_rc)) {\
                dev_err((me)->dev, "CONFIG write error [%d]\n", _rc);\
            }\
        } while (0)

static __inline void max77696_gauge_enable_irq (struct max77696_gauge* me,
    u16 alert_bits, bool forced)
{
    if (unlikely(!forced && (me->irq_unmask & alert_bits) == alert_bits)) {
        /* already unmasked */
        return;
    }

    /* set enabled flag */
    me->irq_unmask |= alert_bits;
    max77696_gauge_write_irq_mask(me);
}

static __inline void max77696_gauge_disable_irq (struct max77696_gauge* me,
    u16 alert_bits, bool forced)
{
    if (unlikely(!forced && (me->irq_unmask & alert_bits) == 0)) {
        /* already masked */
        return;
    }

    /* clear enabled flag */
    me->irq_unmask &= ~alert_bits;
    max77696_gauge_write_irq_mask(me);
}

#define FG_CAP_PARAM_HDR     "MAX77696 FuelGauge Capacity Parameters Save File"\
                             "\0\x01"

struct max77696_gauge_cap_param {
    char          hdr[sizeof(FG_CAP_PARAM_HDR)];
    unsigned long timestamp;
    u16           reg_val[GAUGE_CAP_PARAM_NR_REGS];
};

#define GAUGE_CAP_PARAM_BUF_INIT(_buf) \
        do {\
            memset(_buf, 0x00, sizeof(*(_buf)));\
            memcpy((_buf)->hdr, FG_CAP_PARAM_HDR, sizeof((_buf)->hdr));\
        } while (0)
#define GAUGE_CAP_PARAM_BUF_HDR_CHECK(_buf) \
        (!memcmp((_buf)->hdr, FG_CAP_PARAM_HDR, sizeof(FG_CAP_PARAM_HDR)))

struct max77696_gauge_cap_param_reg {
    char *name;
    u8    addr;
};

#define GAUGE_CAP_PARAM_REG_NAME(id) (max77696_gauge_cap_param_regs[id].name)
#define GAUGE_CAP_PARAM_REG_ADDR(id) (max77696_gauge_cap_param_regs[id].addr)

static struct max77696_gauge_cap_param_reg
max77696_gauge_cap_param_regs[GAUGE_CAP_PARAM_NR_REGS] = {
    #define GAUGE_CAP_PARAM_REG(_id, _reg) \
            [MAX77696_GAUGE_CAP_PARAM_##_id] = {\
                .name = #_id,\
                .addr = FG_REG(_reg)\
            }
    GAUGE_CAP_PARAM_REG(RCOMP0,      RCOMP0),
    GAUGE_CAP_PARAM_REG(TEMPCO,      TEMPCO),
    GAUGE_CAP_PARAM_REG(FULLCAP,     FULLCAP),
    GAUGE_CAP_PARAM_REG(CYCLES,      CYCLES),
    GAUGE_CAP_PARAM_REG(FULLCAPNOM,  FULLCAPNOM),
    GAUGE_CAP_PARAM_REG(IAVG_EMPTY,  IAVG_EMPTY),
    GAUGE_CAP_PARAM_REG(QRESIDUAL00, QRESIDUAL00),
    GAUGE_CAP_PARAM_REG(QRESIDUAL10, QRESIDUAL10),
    GAUGE_CAP_PARAM_REG(QRESIDUAL20, QRESIDUAL20),
    GAUGE_CAP_PARAM_REG(QRESIDUAL30, QRESIDUAL30),
};

static int max77696_gauge_cap_param_write (struct max77696_gauge *me, int id,
    u16 val)
{
    u16 buf = __cpu_to_le16(val);
    int rc;

    rc = max77696_bulk_write(me->i2c,
        GAUGE_CAP_PARAM_REG_ADDR(id), (u8*)(&buf), 2);

    if (unlikely(rc)) {
        dev_err(me->dev, "failed to write capacity parameter %s(%02Xh) [%d]\n",
            GAUGE_CAP_PARAM_REG_NAME(id), GAUGE_CAP_PARAM_REG_ADDR(id), rc);
        goto out;
    }

    dev_vdbg(me->dev, "written 0x%04X capacity parameter %s(%02Xh)\n",
        val, GAUGE_CAP_PARAM_REG_NAME(id), GAUGE_CAP_PARAM_REG_ADDR(id));

out:
    return rc;
}

static int max77696_gauge_cap_param_read (struct max77696_gauge *me, int id,
    u16 *val)
{
    u16 buf = 0;
    int rc;

    rc = max77696_bulk_read(me->i2c,
        GAUGE_CAP_PARAM_REG_ADDR(id), (u8*)(&buf), 2);

    if (unlikely(rc)) {
        dev_err(me->dev, "failed to read capacity parameter %s(%02Xh) [%d]\n",
            GAUGE_CAP_PARAM_REG_NAME(id), GAUGE_CAP_PARAM_REG_ADDR(id), rc);
        goto out;
    }

    *val = __le16_to_cpu(buf);
    dev_vdbg(me->dev, "read 0x%04X capacity parameter %s(%02Xh)\n",
        *val, GAUGE_CAP_PARAM_REG_NAME(id), GAUGE_CAP_PARAM_REG_ADDR(id));

out:
    return rc;
}

/* Capacity parameters backup to buffer / restore from buffer */
static int max77696_gauge_cap_param_backup_buf (struct max77696_gauge *me,
    struct max77696_gauge_cap_param *buf)
{
    int i, rc;

    dev_dbg(me->dev, "saving learned parameters ...\n");

    GAUGE_CAP_PARAM_BUF_INIT(buf);

    for (i = 0; i < GAUGE_CAP_PARAM_NR_REGS; i++) {
        rc = max77696_gauge_cap_param_read(me, i, &(buf->reg_val[i]));
        if (unlikely(rc)) {
            goto out;
        }
    }

    buf->timestamp = get_seconds();
    dev_vdbg(me->dev, "cap param buf ts = %lu\n", buf->timestamp);

out:
    return rc;
}

static int max77696_gauge_cap_param_restore_buf (struct max77696_gauge *me,
    const struct max77696_gauge_cap_param *buf)
{
    u16 cycles, fullcapnom;
    u16 socmix, remcapmix, fullcap0, dqacc, dpacc, learncfg;
    int rc;

    dev_dbg(me->dev, "restoring capacity parameters ...\n");

    if (unlikely(!GAUGE_CAP_PARAM_BUF_HDR_CHECK(buf))) {
        dev_err(me->dev, "invalid buffer header\n");
        rc = -EINVAL;
        goto out;
    }

    dev_vdbg(me->dev, "cap param buf ts = %lu\n", buf->timestamp);

    #define __cap_param_restore_reg(_me, _buf, _id) \
            max77696_gauge_cap_param_write(_me, MAX77696_GAUGE_CAP_PARAM_##_id,\
                (_buf)->reg_val[MAX77696_GAUGE_CAP_PARAM_##_id])

    __cap_param_restore_reg(me, buf, RCOMP0);
    __cap_param_restore_reg(me, buf, TEMPCO);
    __cap_param_restore_reg(me, buf, IAVG_EMPTY);
    __cap_param_restore_reg(me, buf, FULLCAPNOM);

    __cap_param_restore_reg(me, buf, QRESIDUAL00);
    __cap_param_restore_reg(me, buf, QRESIDUAL10);
    __cap_param_restore_reg(me, buf, QRESIDUAL20);
    __cap_param_restore_reg(me, buf, QRESIDUAL30);

    cycles     = buf->reg_val[MAX77696_GAUGE_CAP_PARAM_CYCLES    ];
    fullcapnom = buf->reg_val[MAX77696_GAUGE_CAP_PARAM_FULLCAPNOM];
    dev_vdbg(me->dev, "cap param FULLCAPNOM = 0x%04X\n", fullcapnom);

    /* Wait 350 msec ---------------------------------------------------------*/
    __msleep(350);

    /* Restore FullCap -------------------------------------------------------*/

    rc = max77696_gauge_reg_read_word(me, FULLCAP0, &fullcap0);
    dev_vdbg(me->dev, "read FULLCAP0(%02Xh) 0x%04X [%d]\n",
        FG_REG(FULLCAP0), fullcap0, rc);
    rc = max77696_gauge_reg_read_word(me, SOCMIX,   &socmix);
    dev_vdbg(me->dev, "read SOCMIX(%02Xh) 0x%04X [%d]\n",
        FG_REG(SOCMIX), socmix, rc);

    remcapmix = (u16)(((u32)socmix * (u32)fullcap0) / 25600);
    rc = max77696_gauge_reg_write_word(me, REMCAPMIX, remcapmix);
    dev_vdbg(me->dev, "update REMCAPMIX(%02Xh) 0x%04X [%d]\n",
        FG_REG(REMCAPMIX), remcapmix, rc);

    __cap_param_restore_reg(me, buf, FULLCAP);

    dqacc = (fullcapnom >> 2);
    dpacc = FG_DP_ACC;

    rc = max77696_gauge_reg_write_word(me, DQACC, dqacc);
    dev_vdbg(me->dev, "update DQACC(%02Xh) 0x%04X [%d]\n",
        FG_REG(DQACC), dqacc, rc);
    rc = max77696_gauge_reg_write_word(me, DPACC, dpacc);
    dev_vdbg(me->dev, "update DPACC(%02Xh) 0x%04X [%d]\n",
        FG_REG(DPACC), dpacc, rc);

    /* Wait 350 msec ---------------------------------------------------------*/
    __msleep(350);

    /* Restore Cycles --------------------------------------------------------*/

    __cap_param_restore_reg(me, buf, CYCLES);

    if (likely(cycles & ~0xFF)) {
        learncfg = 0x0676; /* advance to LearnStage = 7 */
        rc = max77696_gauge_reg_write_word(me, LEARNCFG, learncfg);
        dev_vdbg(me->dev, "update LEARNCFG(%02Xh) 0x%04X [%d]\n",
            FG_REG(LEARNCFG), learncfg, rc);
    }

out:
    return rc;
}

static bool max77696_gauge_update_socrep (struct max77696_gauge *me, bool force)
{
    bool updated = 0;
    u16 socrep;
    int rc;

    if (unlikely(!force && __is_cache_valid(me, socrep))) {
        goto out;
    }

    rc = max77696_gauge_reg_read_word(me, SOCREP, &socrep);
    if (unlikely(rc)) {
        dev_err(me->dev, "SOCREP read error [%d]\n", rc);
        goto out;
    }

    /* Units of LSB = 1 / 256 [%] */
    updated = __set_cache(me, socrep, __u16_to_intval(socrep) >> 8);
    dev_vdbg(me->dev, "(%u) SOCREP      %d%%\n",
        updated, __get_cache(me, socrep));

out:
    return updated;
}

static bool max77696_gauge_update_tte (struct max77696_gauge *me, bool force)
{
    bool updated = 0;
    u16 tte;
    int rc;

    if (unlikely(!force && __is_cache_valid(me, tte))) {
        goto out;
    }

    rc = max77696_gauge_reg_read_word(me, TTE, &tte);
    if (unlikely(rc)) {
        dev_err(me->dev, "TTE read error [%d]\n", rc);
        goto out;
    }

    /* Units of LSB = 5.625 [second] */
    updated = __set_cache(me, tte, (__u16_to_intval(tte) * 5625) / 1000);
    dev_vdbg(me->dev, "(%u) TTE         %dsec\n",
        updated, __get_cache(me, tte));

out:
    return updated;
}

static bool max77696_gauge_update_temp (struct max77696_gauge *me, bool force)
{
    bool updated = 0;
    u16 temp;
    int rc;

    if (unlikely(!force && __is_cache_valid(me, temp))) {
        goto out;
    }

    rc = max77696_gauge_reg_read_word(me, TEMP, &temp);
    if (unlikely(rc)) {
        dev_err(me->dev, "TEMP read error [%d]\n", rc);
        goto out;
    }

    /* Units of LSB = 10 / 256 [tenths of degree Celsius] */
    updated = __set_cache(me, temp, (__s16_to_intval(temp) * 10) >> 8);
    dev_vdbg(me->dev, "(%u) TEMP        %d.%d"DEGREE_CELSIUS"\n",
        updated, __get_cache(me, temp) / 10, __get_cache(me, temp) % 10);

out:
    return updated;
}

static bool max77696_gauge_update_vcell (struct max77696_gauge *me, bool force)
{
    bool updated = 0;
    u16 vcell;
    int rc;

    if (unlikely(!force && __is_cache_valid(me, vcell))) {
        goto out;
    }

    rc = max77696_gauge_reg_read_word(me, VCELL, &vcell);
    if (unlikely(rc)) {
        dev_err(me->dev, "VCELL read error [%d]\n", rc);
        goto out;
    }

    /* Units of LSB = 625 [uV] ; Lower 3bits are don't care */
    updated = __set_cache(me, vcell, (__u16_to_intval(vcell) >> 3) * 625);
    dev_vdbg(me->dev, "(%u) VCELL       %duV\n",
        updated, __get_cache(me, vcell));

out:
    return updated;
}

static bool max77696_gauge_update_avgvcell (struct max77696_gauge *me,
    bool force)
{
    bool updated = 0;
    u16 avgvcell;
    int rc;

    if (unlikely(!force && __is_cache_valid(me, avgvcell))) {
        goto out;
    }

    rc = max77696_gauge_reg_read_word(me, AVGVCELL, &avgvcell);
    if (unlikely(rc)) {
        dev_err(me->dev, "AVGVCELL read error [%d]\n", rc);
        goto out;
    }

    /* Units of LSB = 625 [uV] ; Lower 3bits are don't care */
    updated = __set_cache(me, avgvcell, (__u16_to_intval(avgvcell) >> 3) * 625);
    dev_vdbg(me->dev, "(%u) AVGVCELL    %duV\n",
        updated, __get_cache(me, avgvcell));

out:
    return updated;
}

static bool max77696_gauge_update_current (struct max77696_gauge *me,
    bool force)
{
    bool updated = 0;
    u16 current;
    int rc;

    if (unlikely(!force && __is_cache_valid(me, current))) {
        goto out;
    }

    rc = max77696_gauge_reg_read_word(me, CURRENT, &current);
    if (unlikely(rc)) {
        dev_err(me->dev, "CURRENT read error [%d]\n", rc);
        goto out;
    }

    /* Units of LSB = 1562500pV / Rsense_uOhm [uA]
     * (forget the tenths to avoid overflow of 32-bit multiplication)
     */
    updated = __set_cache(me, current,
        ((__s16_to_intval(current) * 15625) / me->r_sns) * 100);
    dev_vdbg(me->dev, "(%u) CURRENT     %duA\n",
        updated, __get_cache(me, current));

out:
    return updated;
}

static bool max77696_gauge_update_avgcurrent (struct max77696_gauge *me,
    bool force)
{
    bool updated = 0;
    u16 avgcurrent;
    int rc;

    if (unlikely(!force && __is_cache_valid(me, avgcurrent))) {
        goto out;
    }

    rc = max77696_gauge_reg_read_word(me, AVGCURRENT, &avgcurrent);
    if (unlikely(rc)) {
        dev_err(me->dev, "AVGCURRENT read error [%d]\n", rc);
        goto out;
    }

    /* Units of LSB = 1562500pV / Rsense_uOhm [uA]
     * (forget the tenths to avoid overflow of 32-bit multiplication)
     */
    updated = __set_cache(me, avgcurrent,
        ((__s16_to_intval(avgcurrent) * 15625) / me->r_sns) * 100);
    dev_vdbg(me->dev, "(%u) AVGCURRENT  %duA\n",
        updated, __get_cache(me, avgcurrent));

out:
    return updated;
}

static bool max77696_gauge_update_remcaprep (struct max77696_gauge *me,
    bool force)
{
    bool updated = 0;
    u16 remcaprep;
    int rc;

    if (unlikely(!force && __is_cache_valid(me, remcaprep))) {
        goto out;
    }

    rc = max77696_gauge_reg_read_word(me, REMCAPREP, &remcaprep);
    if (unlikely(rc)) {
        dev_err(me->dev, "REMCAPREP read error [%d]\n", rc);
        goto out;
    }

    /* Units of LSB = 5000000pVH / Rsense_uOhm [uAh]
     * (forget the tenths to avoid overflow of 32-bit multiplication)
     */
    updated = __set_cache(me, remcaprep,
        ((__u16_to_intval(remcaprep) * 50000) / me->r_sns) * 100);
    dev_vdbg(me->dev, "(%u) REMCAPREP   %duAh\n",
        updated, __get_cache(me, remcaprep));

out:
    return updated;
}

static bool max77696_gauge_update_fullcapnom(struct max77696_gauge *me,
    bool force)
{
    bool updated = 0;
    u16 fullcapnom;
    int rc;

    if (unlikely(!force && __is_cache_valid(me, fullcapnom))) {
        goto out;
    }

    rc = max77696_gauge_reg_read_word(me, FULLCAPNOM, &fullcapnom);
    if (unlikely(rc)) {
        dev_err(me->dev, "FULLCAPNOM read error [%d]\n", rc);
        goto out;
    }

    /* Units of LSB = 5000000pVH / Rsense_uOhm [uAh]
     * (forget the tenths to avoid overflow of 32-bit multiplication)
     */
    updated = __set_cache(me, fullcapnom,
        ((__u16_to_intval(fullcapnom) * 50000) / me->r_sns) * 100);
    dev_vdbg(me->dev, "(%u) FULLCAPNOM  %duAh\n",
        updated, __get_cache(me, fullcapnom));

out:
    return updated;
}

static bool max77696_gauge_update_cycles (struct max77696_gauge *me, bool force)
{
    bool updated = 0;
    u16 cycles;
    int rc;

    if (unlikely(!force && __is_cache_valid(me, cycles))) {
        goto out;
    }

    rc = max77696_gauge_reg_read_word(me, CYCLES, &cycles);
    if (unlikely(rc)) {
        dev_err(me->dev, "CYCLES read error [%d]\n", rc);
        goto out;
    }

    /* Units of LSB = 1 [%] */
    __set_cache(me, cycles, __u16_to_intval(cycles));
    dev_vdbg(me->dev, "(%u) CYCLES      %d%%\n",
        updated, __get_cache(me, cycles));

out:
    return updated;
}

#define __update_cache(me, cache, force) \
        max77696_gauge_update_##cache (me, force)

static void max77696_gauge_psy_work (struct work_struct *work)
{
    struct max77696_gauge *me = __psy_work_to_max77696_gauge(work);
    bool updated = 0;
    u16 polling_prop = me->polling_properties;

    __lock(me);
    dev_vdbg(me->dev, "--- updating properties @ %ld ---\n", get_seconds());

    if (likely(polling_prop & MAX77696_GAUGE_POLLING_CAPACITY)) {
        updated |= __update_cache(me, socrep, 0);
    }

    if (likely(polling_prop & MAX77696_GAUGE_POLLING_TEMP)) {
        updated |= __update_cache(me, temp, 0);
    }

    if (likely(polling_prop & MAX77696_GAUGE_POLLING_VOLTAGE_NOW)) {
        updated |= __update_cache(me, vcell, 0);
    }

    if (likely(polling_prop & MAX77696_GAUGE_POLLING_VOLTAGE_AVG)) {
        updated |= __update_cache(me, avgvcell, 0);
    }

    if (likely(me->polling_properties & MAX77696_GAUGE_POLLING_CURRENT_NOW)) {
        updated |= __update_cache(me, current, 0);
    }

    if (likely(me->polling_properties & MAX77696_GAUGE_POLLING_CURRENT_AVG)) {
        updated |= __update_cache(me, avgcurrent, 0);
    }

    if (likely(me->polling_properties & MAX77696_GAUGE_POLLING_CHARGE_FULL)) {
        updated |= __update_cache(me, fullcapnom, 0);
    }

    if (likely(me->polling_properties & MAX77696_GAUGE_POLLING_CHARGE_NOW)) {
        updated |= __update_cache(me, remcaprep, 0);
    }

    if (likely(updated)) {
        power_supply_changed(&(me->psy));
    }

    schedule_delayed_work(&(me->psy_work), me->update_interval);

    __unlock(me);
    return;
}

static irqreturn_t max77696_gauge_isr (int irq, void *data)
{
    struct max77696_gauge *me = data;
    u8 interrupted;

    max77696_irq_read_fgirq_status(&interrupted);
    dev_dbg(me->dev, "FG_INT %02X\n", interrupted);

    power_supply_changed(&(me->psy));

    return IRQ_HANDLED;
}

static void max77696_gauge_external_power_changed (struct power_supply *psy)
{
    struct max77696_gauge *me = __psy_to_max77696_gauge(psy);

    __lock(me);

    dev_dbg(me->dev, "external power changed\n");
    power_supply_changed(&(me->psy));

    __unlock(me);
}

static void max77696_gauge_trigger_leds (struct max77696_gauge *me, int status)
{
    struct max77696_gauge_led_trigger* led_trig = NULL;
    int i;

    if (unlikely(!me->support_led_triggers || me->disallow_led_trigger)) {
        return;
    }

    switch (status) {
        case POWER_SUPPLY_STATUS_FULL:
            led_trig = me->charging_full_trigger;
            break;

        case POWER_SUPPLY_STATUS_CHARGING:
            led_trig = me->charging_trigger;
            break;

        default:
            led_trig = me->default_trigger;
            break;
    }

    if (unlikely(!led_trig || !led_trig->enable)) {
        return;
    }

    for (i = 0; i < MAX77696_LED_NR_LEDS; i++) {
        struct max77696_gauge_led_control *led_ctrl = &(led_trig->control[i]);

        max77696_led_enable_manual_mode(i, led_ctrl->manual_mode);
        max77696_led_set_brightness(i, led_ctrl->brightness);
        if (led_ctrl->flashing) {
            max77696_led_set_blink(i,
                led_ctrl->flash_params.duration_ms,
                led_ctrl->flash_params.period_ms);
        } else {
            max77696_led_disable_blink(i);
        }
        max77696_led_update_changes(i);

    }
}

static int max77696_gauge_get_prop_status (struct max77696_gauge *me)
{
    int battery_capacity, rc;

    __update_cache(me, socrep, 0);
    battery_capacity = __get_cache(me, socrep);

    if (unlikely(!me->charger_online || !me->charger_enable)) {
        rc = POWER_SUPPLY_STATUS_UNKNOWN;
        goto out;
    }

    if (unlikely(!me->charger_online())) {
        rc = POWER_SUPPLY_STATUS_DISCHARGING;
        goto out;
    }

    if (unlikely(battery_capacity >= me->battery_full_capacity)) {
        rc = POWER_SUPPLY_STATUS_FULL;
        goto out;
    }

    rc = (me->charger_enable()?
        POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_NOT_CHARGING);

out:
    max77696_gauge_trigger_leds(me, rc);
    return rc;
}

static int max77696_gauge_get_prop_present (struct max77696_gauge *me)
{
    u16 buf;
    int rc;

    if (likely(me->battery_online)) {
        rc = (int)me->battery_online();
        goto out;
    }
    max77696_gauge_reg_read_word(me, STATUS, &buf);
    rc = (!(buf & FG_STATUS_BST));

out:
    return rc;
}

static int max77696_gauge_get_prop_charge_full_design (struct max77696_gauge *me)
{
    return me->charge_full_design;
}

static int max77696_gauge_get_prop_cycle_count (struct max77696_gauge *me)
{
    int cycle_count;

    __update_cache(me, cycles, 0);

    cycle_count  = __get_cache(me, cycles);
    cycle_count -= me->config->cycles;
    cycle_count /= 100;

    return cycle_count;
}

static int max77696_gauge_get_prop_voltage_now (struct max77696_gauge *me)
{
    __update_cache(me, vcell, 0);
    return __get_cache(me, vcell);
}

static int max77696_gauge_get_prop_voltage_avg (struct max77696_gauge *me)
{
    __update_cache(me, avgvcell, 0);
    return __get_cache(me, avgvcell);
}

static int max77696_gauge_get_prop_charge_full (struct max77696_gauge *me)
{
    __update_cache(me, fullcapnom, 0);
    return __get_cache(me, fullcapnom);
}

static int max77696_gauge_get_prop_charge_now (struct max77696_gauge *me)
{
    __update_cache(me, remcaprep, 0);
    return __get_cache(me, remcaprep);
}

static int max77696_gauge_get_prop_capacity (struct max77696_gauge *me)
{
    __update_cache(me, socrep, 0);
    return __get_cache(me, socrep);
}

static int max77696_gauge_get_prop_temp (struct max77696_gauge *me)
{
    __update_cache(me, temp, 0);
    return __get_cache(me, temp);
}

static int max77696_gauge_get_prop_current_now (struct max77696_gauge *me)
{
    __update_cache(me, current, 0);
    return __get_cache(me, current);
}

static int max77696_gauge_get_prop_current_avg (struct max77696_gauge *me)
{
    __update_cache(me, avgcurrent, 0);
    return __get_cache(me, avgcurrent);
}

static int max77696_gauge_get_property (struct power_supply *psy,
    enum power_supply_property psp, union power_supply_propval *val)
{
    struct max77696_gauge *me = __psy_to_max77696_gauge(psy);
    int rc = 0;

    __lock(me);

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = max77696_gauge_get_prop_status(me);
        break;

    case POWER_SUPPLY_PROP_PRESENT:
    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = max77696_gauge_get_prop_present(me);
        break;

    case POWER_SUPPLY_PROP_CYCLE_COUNT:
        val->intval = max77696_gauge_get_prop_cycle_count(me);
        break;

    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = max77696_gauge_get_prop_voltage_now(me);
        break;

    case POWER_SUPPLY_PROP_VOLTAGE_AVG:
        val->intval = max77696_gauge_get_prop_voltage_avg(me);
        break;

    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        val->intval = max77696_gauge_get_prop_charge_full_design(me);
        break;

    case POWER_SUPPLY_PROP_CHARGE_FULL: /* Last Measured Discharge */
        val->intval = max77696_gauge_get_prop_charge_full(me);
        break;

    case POWER_SUPPLY_PROP_CHARGE_NOW: /* Nominal Available Charge */
        val->intval = max77696_gauge_get_prop_charge_now(me);
        break;

    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = max77696_gauge_get_prop_capacity(me);
        break;

    case POWER_SUPPLY_PROP_TEMP:
        val->intval = max77696_gauge_get_prop_temp(me);
        break;

    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = max77696_gauge_get_prop_current_now(me);
        break;

    case POWER_SUPPLY_PROP_CURRENT_AVG:
        val->intval = max77696_gauge_get_prop_current_avg(me);
        break;

    default:
        rc = -EINVAL;
        break;
    }

    __unlock(me);
    return rc;
}

static enum power_supply_property max77696_gauge_psy_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_CYCLE_COUNT,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_VOLTAGE_AVG,
    POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
    POWER_SUPPLY_PROP_CHARGE_FULL,
    POWER_SUPPLY_PROP_CHARGE_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CURRENT_AVG,
};

#define MAX77696_GAUGE_ATTR_CAP_PARAM(_name, _id) \
static ssize_t max77696_gauge_##_name##_show (struct device *dev,\
    struct device_attribute *devattr, char *buf)\
{\
    struct platform_device *pdev = to_platform_device(dev);\
    struct max77696_gauge *me = platform_get_drvdata(pdev);\
    u16 val;\
    int rc;\
    __lock(me);\
    max77696_gauge_cap_param_read(me, MAX77696_GAUGE_CAP_PARAM_##_id, &val);\
    rc = (int)snprintf(buf, PAGE_SIZE, "%u (0x%04X)\n", val, val);\
    __unlock(me);\
    return (ssize_t)rc;\
}\
static ssize_t max77696_gauge_##_name##_store (struct device *dev,\
    struct device_attribute *devattr, const char *buf, size_t count)\
{\
    struct platform_device *pdev = to_platform_device(dev);\
    struct max77696_gauge *me = platform_get_drvdata(pdev);\
    u16 val;\
    __lock(me);\
    val = (u16)simple_strtoul(buf, NULL, 10);\
    max77696_gauge_cap_param_write(me, MAX77696_GAUGE_CAP_PARAM_##_id, val);\
    __unlock(me);\
    return (ssize_t)count;\
}\
static DEVICE_ATTR(_name, S_IWUSR | S_IRUGO,\
    max77696_gauge_##_name##_show, max77696_gauge_##_name##_store)

MAX77696_GAUGE_ATTR_CAP_PARAM(rcomp0,      RCOMP0);
MAX77696_GAUGE_ATTR_CAP_PARAM(tempco,      TEMPCO);
MAX77696_GAUGE_ATTR_CAP_PARAM(fullcap,     FULLCAP);
MAX77696_GAUGE_ATTR_CAP_PARAM(cycles,      CYCLES);
MAX77696_GAUGE_ATTR_CAP_PARAM(fullcapnom,  FULLCAPNOM);
MAX77696_GAUGE_ATTR_CAP_PARAM(iavg_empty,  IAVG_EMPTY);
MAX77696_GAUGE_ATTR_CAP_PARAM(qresidual00, QRESIDUAL00);
MAX77696_GAUGE_ATTR_CAP_PARAM(qresidual10, QRESIDUAL10);
MAX77696_GAUGE_ATTR_CAP_PARAM(qresidual20, QRESIDUAL20);
MAX77696_GAUGE_ATTR_CAP_PARAM(qresidual30, QRESIDUAL30);

static ssize_t max77696_gauge_cap_param_bin_read (struct file *filp,
    struct kobject *kobj, struct bin_attribute *attr,
    char *buf, loff_t off, size_t count)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_gauge *me = platform_get_drvdata(pdev);
    int bufsz = (int)sizeof(struct max77696_gauge_cap_param), rc;

    if (unlikely(!count)) {
        rc = (int)count;
        goto out;
    }

    if (unlikely(off > 0 || count < bufsz)) {
        dev_err(me->dev, "buffer too short\n");
        rc = -EINVAL;
        goto out;
    }

    rc = max77696_gauge_cap_param_backup_buf(me, (void*)buf);
    if (unlikely(rc)) {
        goto out;
    }

    rc = bufsz;

out:
    return (ssize_t)rc;
}

static ssize_t max77696_gauge_cap_param_bin_write (struct file *filp,
    struct kobject *kobj, struct bin_attribute *attr,
    char *buf, loff_t off, size_t count)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_gauge *me = platform_get_drvdata(pdev);
    int bufsz = (int)sizeof(struct max77696_gauge_cap_param), rc;

    if (unlikely(!count)) {
        rc = (int)count;
        goto out;
    }

    if (unlikely(off > 0 || count < bufsz)) {
        dev_err(me->dev, "buffer too short\n");
        rc = -EINVAL;
        goto out;
    }

    rc = max77696_gauge_cap_param_restore_buf(me, (void*)buf);
    if (unlikely(rc)) {
        goto out;
    }

    rc = bufsz;

out:
    return (ssize_t)rc;
}

static struct bin_attribute max77696_gauge_cap_param_bin_attr = {
    .attr.name = "cap_param_bin",
    .attr.mode = S_IWUSR | S_IRUGO,
    .size      = sizeof(struct max77696_gauge_cap_param),
    .read      = max77696_gauge_cap_param_bin_read,
    .write     = max77696_gauge_cap_param_bin_write,
};

static ssize_t max77696_gauge_allow_led_trigger_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_gauge *me = platform_get_drvdata(pdev);
    int rc;

    __lock(me);

    rc  = (int)snprintf(buf, PAGE_SIZE, "%u\n",
        (me->disallow_led_trigger == 0));

    __unlock(me);
    return (ssize_t)rc;
}

static ssize_t max77696_gauge_allow_led_trigger_store (struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_gauge *me = platform_get_drvdata(pdev);

    __lock(me);

    me->disallow_led_trigger = (bool)(simple_strtoul(buf, NULL, 10) == 0);

    __unlock(me);
    return (ssize_t)count;
}

static DEVICE_ATTR(allow_led_trigger, S_IWUSR | S_IRUGO,
    max77696_gauge_allow_led_trigger_show,
    max77696_gauge_allow_led_trigger_store);

static ssize_t max77696_gauge_init_complete_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_gauge *me = platform_get_drvdata(pdev);
    int rc;

    __lock(me);

    rc  = (int)snprintf(buf, PAGE_SIZE, "%u\n", !!me->init_complete);

    __unlock(me);
    return (ssize_t)rc;
}
static DEVICE_ATTR(init_complete, S_IRUGO, max77696_gauge_init_complete_show, NULL);

static struct attribute *max77696_gauge_attr[] = {
    &dev_attr_rcomp0.attr,
    &dev_attr_tempco.attr,
    &dev_attr_fullcap.attr,
    &dev_attr_cycles.attr,
    &dev_attr_fullcapnom.attr,
    &dev_attr_iavg_empty.attr,
    &dev_attr_qresidual00.attr,
    &dev_attr_qresidual10.attr,
    &dev_attr_qresidual20.attr,
    &dev_attr_qresidual30.attr,

    &dev_attr_allow_led_trigger.attr,
    &dev_attr_init_complete.attr,
    NULL
};

static const struct attribute_group max77696_gauge_attr_group = {
    .attrs = max77696_gauge_attr,
};

static int max77696_gauge_load_custom_model (struct max77696_gauge *me)
{
    struct max77696_gauge_config *config = me->config;
    u8 reg = FG_REG(CHAR_TBL);
    u16 tmp, *r_buf, *w_buf = config->cell_char_tbl;
	int retries, bufsz, i, rc = 0;

    bufsz = FG_CHAR_TBL_SZ * sizeof(*r_buf);
    r_buf = kzalloc(bufsz, GFP_KERNEL);
    if (unlikely(!r_buf)) {
        dev_err(me->dev, "out of memory (%uB requested)\n", bufsz);
        return -ENOMEM;
    }

    /* Unlock Model Access --
     * To unlock access to the model the host software must write the following
     */
	max77696_gauge_reg_write_word_verify(me, MODEL_LOCK1,
	    FG_MODEL_UNLOCK_CODE1);
	max77696_gauge_reg_write_word_verify(me, MODEL_LOCK2,
	    FG_MODEL_UNLOCK_CODE2);

    /* Write/Read/Verify the Custom Model --
     * Once the model is unlocked, the host software must write the 48 word
     * model to the MAX77696 Fuel Gauge. The model is located between memory
     * locations 0x80h and 0xAFh.
     */

	for (i = 0; i < FG_CHAR_TBL_SZ; i++) {
	    tmp = __cpu_to_le16(w_buf[i]);
        max77696_bulk_write(me->i2c, reg + i, (u8*)(&tmp), 2);
    }

	for (i = 0; i < FG_CHAR_TBL_SZ; i++) {
		max77696_bulk_read(me->i2c, reg + i, (u8*)(&tmp), 2);
        r_buf[i] = __le16_to_cpu(tmp);
    }

	if (unlikely(memcmp(r_buf, w_buf, bufsz))) {
        dev_err(me->dev, "mismatch found in writing model:\n");
        for (i = 0; i < FG_CHAR_TBL_SZ; i++) {
            dev_err(me->dev, "  %02Xh w:0x%04X r:0x%04X\n",
                reg + i, w_buf[i], r_buf[i]);
        }
        rc = -EIO;
        goto lock_model_access;
	}

lock_model_access:
    for (retries = 3; retries > 0; retries--) {
        /* Lock Model Access --
         * To lock access to the model the host software must write the
         * following.
         */
    	max77696_gauge_reg_write_word_verify(me, MODEL_LOCK1,
    	    FG_MODEL_LOCK_CODE1);
    	max77696_gauge_reg_write_word_verify(me, MODEL_LOCK2,
    	    FG_MODEL_LOCK_CODE2);

        /* Verify that Model Access is locked --
         * If the model remains unlocked, the MAX77696 Fuel Gauge will not be
         * able to monitor the capacity of the battery. Therefore it is very
         * critical that the Model Access is locked. To verify it is locked,
         * simply read back the model as in verification writing model. However,
         * this time, all values should be read as 0x00h.
         */
    	for (i = 0; i < FG_CHAR_TBL_SZ; i++) {
    		max77696_bulk_read((me)->i2c, reg + i, (u8*)(&r_buf[i]), 2);
    		if (unlikely(r_buf[i])) {
    		    dev_err(me->dev, "failed to verify model access is locked\n");
    			rc = -EIO;
    			continue;
    		}
    	}

        /* Verified successfully */
    	goto out;
    }

out:
	kfree(r_buf);
	return rc;
}

static int max77696_gauge_por_init (struct max77696_gauge *me)
{
    struct max77696_gauge_config *config = me->config;
    u16 model_scaling, fullcap, capacity, remcap, repcap, dq_acc, dp_acc;
    u16 vfsoc, status;
    int rc = 0;

    #define POR_INIT_LOG(_me, _format, _arg...) \
            dev_dbg(_me->dev, "### POR_INIT: " _format, ##_arg)

    POR_INIT_LOG(me, "start\n");

    /*
     * INITIALIZE REGISTERS TO RECOMMENDED CONFIGURATION
     * -------------------------------------------------
     * The MAX77696 Fuel Gauge should be initialized prior to being used.
     * The following three registers should be written to these values in order
     * for the MAX77696 Fuel Gauge to perform at its best. These values are
     * written to RAM, so they must be written to the device any time that power
     * is applied or restored to the device. Some registers are updated
     * internally, so it is necessary to verify that the register was written
     * correctly to prevent data collisions.
     */

    /* Delay at least 500ms --
     * After Power up, the MAX77696 Fuel Gauge requires 500ms in order to
     * perform signal debouncing and initial SOC reporting.
     */
    __msleep(500);

    /* Initialize Configuration --
     */
    POR_INIT_LOG(me, "initialize configuration\n");
    max77696_gauge_reg_write_word(me, CONFIG,     config->config         );
    max77696_gauge_reg_write_word(me, FILTERCFG,  config->filter_cfg     );
    /* Note: default value of RelaxCFG until Standby current is known */
  //max77696_gauge_reg_write_word(me, RELAXCFG,   config->relax_cfg      );
    max77696_gauge_reg_write_word(me, LEARNCFG,   config->learn_cfg      );
    max77696_gauge_reg_write_word(me, FULLSOCTHR, config->full_soc_thresh);

    /*
     * LOAD CUSTOM MODEL AND PARAMETERS
     * --------------------------------
     * The custom model that is stored in the MAX77696 Fuel Gauge is also
     * written to RAM and so it must be written to the device any time that
     * power is applied or restored to the device. When the device is powered
     * on, the host software must first unlock write access to the model, write
     * the model, verify the model was written properly, and then lock access to
     * the model. After the model is loaded correctly, simply write a few
     * registers with customized parameters that will be provided by Maxim.
     */

    POR_INIT_LOG(me, "load custom model\n");
    rc = max77696_gauge_load_custom_model(me);
    if (unlikely(rc)) {
        dev_err(me->dev, "failed to load custom model [%d]\n", rc);
        goto out;
    }

    /* Write Custom Parameters --
     * Nine additional registers should be written in this step with values that
     * are provided by Maxim.
     */
    POR_INIT_LOG(me, "write custom parameters\n");
    max77696_gauge_reg_write_word_verify(me, RCOMP0,      config->rcomp0   );
    max77696_gauge_reg_write_word_verify(me, TEMPCO,      config->tempco   );
    max77696_gauge_reg_write_word       (me, ICHGTERM,    config->ichg_term);
    /* TODO: set the TGAIN and TOFF when using thermistor */
  //max77696_gauge_reg_write_word       (me, TGAIN,       config->tgain    );
  //max77696_gauge_reg_write_word       (me, TOFF,        config->toff     );
    max77696_gauge_reg_write_word_verify(me, V_EMPTY,     config->vempty   );
    max77696_gauge_reg_write_word_verify(me, QRESIDUAL00, config->qrtbl00  );
    max77696_gauge_reg_write_word_verify(me, QRESIDUAL10, config->qrtbl10  );
    max77696_gauge_reg_write_word_verify(me, QRESIDUAL20, config->qrtbl20  );
    max77696_gauge_reg_write_word_verify(me, QRESIDUAL30, config->qrtbl30  );

    /* Update Full Capacity Parameters --
     */
    POR_INIT_LOG(me, "update full capacity parameters\n");
    max77696_gauge_reg_write_word_verify(me, FULLCAP,    config->fullcap   );
    max77696_gauge_reg_write_word       (me, DESIGNCAP,  config->design_cap);
    max77696_gauge_reg_write_word_verify(me, FULLCAPNOM, config->fullcapnom);

    /* Delay at least 350ms --
     * This delay must be at least 350ms to allow VFSOC to be calculated from
     * the new configuration.
     */
    __msleep(350);

    /* Write VFSOC value to VFSOC0 --
     */
    POR_INIT_LOG(me, "write VFSOC value to VFSOC0\n");
    max77696_gauge_reg_read_word        (me, VFSOC,       &vfsoc);
    max77696_gauge_reg_write_word_verify(me, VFSOC0_LOCK, FG_VFSOC0_UNLOCK_CODE);
    max77696_gauge_reg_write_word       (me, VFSOC0,      vfsoc);
    max77696_gauge_reg_write_word_verify(me, VFSOC0_LOCK, FG_VFSOC0_LOCK_CODE);
    dev_vdbg(me->dev, "    VFSOC         0x%04X\n", vfsoc);

    /* Advance to Coulomb-Counter Mode --
     * Advancing the cycles register to a higher values makes the fuelgauge
     * behave more like a coulomb counter. MAX77696 Fuel Gauge supports quicker
     * insertion error healing by supporting starting from a lower learn stage.
     *
     * To Advance to Coulomb-Counter Mode, simply write the Cycles register to a
     * value of 96% for MAX77696 Fuel Gauge.
     */
    POR_INIT_LOG(me, "advance to coulomb-counter mode\n");
    max77696_gauge_reg_write_word_verify(me, CYCLES, config->cycles);

    /* Load New Capacity Parameters --
     */

    POR_INIT_LOG(me, "load new capacity parameters\n");

    model_scaling = FG_MODEL_SCALING;
    fullcap       = config->fullcap;
    capacity      = config->batt_cap;
    remcap        = (u16)(((u32)vfsoc * (u32)fullcap) / 25600);
    repcap        = (u16)((u32)remcap * (u32)(capacity / fullcap)
        / model_scaling);
    dq_acc        = (fullcap >> 2); /* dQ_acc = 200% of Cacpaticy */
    dp_acc        = FG_DP_ACC;

    dev_vdbg(me->dev, "    Model Scaling 0x%04X\n", model_scaling);
    dev_vdbg(me->dev, "    VF FullCap    0x%04X\n", fullcap);
    dev_vdbg(me->dev, "    Capacity      0x%04X\n", capacity);
    dev_vdbg(me->dev, "    RemCap        0x%04X\n", remcap);
    dev_vdbg(me->dev, "    RepCap        0x%04X\n", repcap);
    dev_vdbg(me->dev, "    dQ Acc        0x%04X\n", dq_acc);
    dev_vdbg(me->dev, "    dP Acc        0x%04X\n", dp_acc);

    max77696_gauge_reg_write_word       (me, REMCAPMIX,  remcap  );
    max77696_gauge_reg_write_word       (me, REMCAPREP,  repcap  );
    max77696_gauge_reg_write_word_verify(me, DQACC,      dq_acc  );
    max77696_gauge_reg_write_word_verify(me, DPACC,      dp_acc  );
    max77696_gauge_reg_write_word_verify(me, FULLCAP,    capacity);
    max77696_gauge_reg_write_word       (me, DESIGNCAP,  fullcap );
    max77696_gauge_reg_write_word_verify(me, FULLCAPNOM, fullcap );

    /* Update SOC register with new SOC */
    max77696_gauge_reg_write_word(me, SOCREP, vfsoc);

    /* Initialization Complete --
     * Clear the POR bit to indicate that the custom model and parameters were
     * successfully loaded.
     */
    POR_INIT_LOG(me, "clear the POR bit\n");
    max77696_gauge_reg_read_word (me, STATUS, &status);
    max77696_gauge_reg_write_word(me, STATUS, status & ~FG_STATUS_POR);

out:
    POR_INIT_LOG(me, "end [%d]\n", rc);
    return rc;
}

static void max77696_gauge_init_alert (struct max77696_gauge *me)
{
    u8 fgirq_l2 = 0;
    u16 alert_val, alert_en = 0;
    int alert_max, alert_min;

    /* To prevent false interrupts, the threshold registers should be
     * initialized before setting the Aen bit.
     */

    if (likely(me->v_alert_max > me->v_alert_min)) {
        fgirq_l2 |= (MAX77696_FGIRQMASK_VMX|MAX77696_FGIRQMASK_VMN);
        alert_en |= FG_CONFIG_AEN;

        alert_max = min(me->v_alert_max, 5100); /* 255 * 20 */
        alert_min = max(me->v_alert_min,    0);

        dev_dbg(me->dev, "Voltage alert            %d ... %d mV\n",
            alert_min, alert_max);

        alert_val  = ((u16)DIV_ROUND_UP(alert_max, 20) << 8);
        alert_val |= ((u16)DIV_ROUND_UP(alert_min, 20) << 0);
    } else {
        dev_dbg(me->dev, "Voltage alert            off\n");
        alert_val = 0xFF00;
    }

    max77696_gauge_reg_write_word(me, VALRT_TH, __cpu_to_le16(alert_val));

    if (likely(me->t_alert_max > me->t_alert_min)) {
        fgirq_l2 |= (MAX77696_FGIRQMASK_TMX|MAX77696_FGIRQMASK_TMN);
        alert_en |= FG_CONFIG_AEN;

        alert_max = min(me->t_alert_max,  127);
        alert_min = max(me->t_alert_min, -128);

        dev_dbg(me->dev, "Temperture alert         %d ... %d "DEGREE_CELSIUS"\n",
            alert_min, alert_max);

        alert_val  = ((s16)alert_max << 8);
        alert_val |= ((s16)alert_min << 0);
    } else {
        dev_dbg(me->dev, "Temperture alert         off\n");
        alert_val = 0x7F80;
    }

    max77696_gauge_reg_write_word(me, TALRT_TH, __cpu_to_le16(alert_val));

    if (likely(me->s_alert_max > me->s_alert_min)) {
        fgirq_l2 |= (MAX77696_FGIRQMASK_SMX|MAX77696_FGIRQMASK_SMN);
        alert_en |= FG_CONFIG_AEN;

        alert_max = min(me->s_alert_max, 255);
        alert_min = max(me->s_alert_min,   0);

        dev_dbg(me->dev, "SOC alert                %d ... %d %%\n",
            alert_min, alert_max);

        alert_val  = ((u16)alert_max << 8);
        alert_val |= ((u16)alert_min << 0);
    } else {
        dev_dbg(me->dev, "SOC alert                off\n");
        alert_val = 0xFF00;
    }

    max77696_gauge_reg_write_word(me, SALRT_TH, __cpu_to_le16(alert_val));

    /* Enable 2nd level fuel gauge interrupts */
    max77696_irq_enable_fgirq(fgirq_l2, 0);

    dev_dbg(me->dev, "Battery removal alert    %s\n",
        me->enable_alert_on_battery_removal?   "ON" : "off");
    dev_dbg(me->dev, "Battery insertion alert  %s\n",
        me->enable_alert_on_battery_insertion? "ON" : "off");

    alert_en |= ((me->enable_alert_on_battery_removal  )? FG_CONFIG_BER : 0);
    alert_en |= ((me->enable_alert_on_battery_insertion)? FG_CONFIG_BEI : 0);

    /* Enable fuel gauge interrupts we need */
    max77696_gauge_enable_irq(me, alert_en, 0);
}

static void max77696_gauge_init_work (struct work_struct *work)
{
    struct max77696_gauge *me = __init_work_to_max77696_gauge(work);
    u16 status;
    int rc;

    INIT_DELAYED_WORK(&(me->psy_work), max77696_gauge_psy_work);

    max77696_gauge_reg_read_word(me, STATUS, &status);
    dev_dbg(me->dev, "STATUS %04X %s%s%s%s%s%s%s%s%s%s\n",
        status,
        (status & FG_STATUS_POR)? "POR " : "",  /* Power-On Reset */
        (status & FG_STATUS_BST)? "BST " : "",  /* Battery Status */
        (status & FG_STATUS_VMN)? "VMN " : "",  /* Minimum V_alrt Threshold Exceeded */
        (status & FG_STATUS_TMN)? "TMN " : "",  /* Minimum T_alrt Threshold Exceeded */
        (status & FG_STATUS_SMN)? "SMN " : "",  /* Minimum SOC_alrt Threshold Exceeded */
        (status & FG_STATUS_BI )? "BI "  : "",  /* Battery Insertion */
        (status & FG_STATUS_VMX)? "VMX " : "",  /* Maximum V_alrt Threshold Exceeded */
        (status & FG_STATUS_TMX)? "TMX " : "",  /* Maximum T_alrt Threshold Exceeded */
        (status & FG_STATUS_SMX)? "SMX " : "",  /* Maximum SOC_alrt Threshold Exceeded */
        (status & FG_STATUS_BR )? "BR "  : ""); /* Battery Removal */;

    /* Initialize registers according to values from the platform data */
    if (likely(status & FG_STATUS_POR)) {
        if (likely(me->enable_por_init)) {
            rc = max77696_gauge_por_init(me);
            if (unlikely(rc)) {
                dev_err(me->dev, "failed to POR-Init [%d]\n", rc);
                goto out;
            }
        }
    }

    __update_cache(me, socrep,     1);
    __update_cache(me, tte,        1);
    __update_cache(me, remcaprep,  1);
    __update_cache(me, fullcapnom, 1);
    __update_cache(me, cycles,     1);

    me->psy.name                   = MAX77696_PSY_BATT_NAME;
    me->psy.type                   = POWER_SUPPLY_TYPE_BATTERY;
    me->psy.external_power_changed = max77696_gauge_external_power_changed;
    me->psy.get_property           = max77696_gauge_get_property;
    me->psy.properties             = max77696_gauge_psy_props;
    me->psy.num_properties         = ARRAY_SIZE(max77696_gauge_psy_props);

    rc = sysfs_create_bin_file(me->kobj, &max77696_gauge_cap_param_bin_attr);
    if (unlikely(rc)) {
        dev_err(me->dev, "failed to create bin attribute [%d]\n", rc);
        goto out_err_create_sysfs_bin;
    }

    rc = sysfs_create_group(me->kobj, &max77696_gauge_attr_group);
    if (unlikely(rc)) {
        dev_err(me->dev, "failed to create attribute group [%d]\n", rc);
        goto out_err_create_sysfs_grp;
    }

    rc = power_supply_register(me->dev, &(me->psy));
    if (unlikely(rc)) {
        dev_err(me->dev, "failed to register psy device [%d]\n", rc);
        goto out_err_reg_psy;
    }

    max77696_gauge_init_alert(me);

    BUG_ON(me->chip->gauge_ptr);
    me->chip->gauge_ptr = me;

    me->init_complete = 1;
    dev_dbg(me->dev, "pwrup done\n");

    schedule_delayed_work(&(me->psy_work), me->update_interval);
    goto out;

out_err_reg_psy:
    sysfs_remove_group(me->kobj, &max77696_gauge_attr_group);
out_err_create_sysfs_grp:
    sysfs_remove_bin_file(me->kobj, &max77696_gauge_cap_param_bin_attr);
out_err_create_sysfs_bin:
out:
    return;
}

static __always_inline
void __show_led_trigger_config (struct max77696_gauge *me, const char* name,
    struct max77696_gauge_led_trigger* led_trig)
{
    int i;

    if (!led_trig->enable) {
        dev_dbg(me->dev, "LED trigger '%s': off\n", name);
        return;
    }

    dev_dbg(me->dev, "LED trigger '%s':\n", name);

    for (i = 0; i < MAX77696_LED_NR_LEDS; i++) {
        struct max77696_gauge_led_control *led_ctrl = &(led_trig->control[i]);

        dev_dbg(me->dev, "  LED[%u] mode           %s\n",
            i, (led_ctrl->manual_mode? "manual" : "autonomous"));
        dev_dbg(me->dev, "  LED[%u] brightness     %d\n",
            i, led_ctrl->brightness);
        if (led_ctrl->flashing) {
            dev_dbg(me->dev, "  LED[%u] flash duration %lumsec\n",
                i, led_ctrl->flash_params.duration_ms);
            dev_dbg(me->dev, "  LED[%u] flash period   %lumsec\n",
                i, led_ctrl->flash_params.period_ms);
        } else {
            dev_dbg(me->dev, "  LED[%u] flashing       no\n", i);
        }
    }
}

static __devinit int max77696_gauge_probe (struct platform_device *pdev)
{
    struct max77696_chip *chip = dev_get_drvdata(pdev->dev.parent);
    struct max77696_gauge_platform_data *pdata = pdev->dev.platform_data;
    struct max77696_gauge *me;
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

    me->config                            = pdata->config;
    me->enable_por_init                   = pdata->enable_por_init;

    dev_dbg(me->dev, "POR initialization       %s\n",
        me->enable_por_init? "ON" : "off");

    me->v_alert_max                       = pdata->v_alert_max;
    me->v_alert_min                       = pdata->v_alert_min;
    me->t_alert_max                       = pdata->t_alert_max;
    me->t_alert_min                       = pdata->t_alert_min;
    me->s_alert_max                       = pdata->s_alert_max;
    me->s_alert_min                       = pdata->s_alert_min;
    me->enable_alert_on_battery_removal   =
        pdata->enable_alert_on_battery_removal;
    me->enable_alert_on_battery_insertion =
        pdata->enable_alert_on_battery_insertion;

    me->charge_full_design                = pdata->charge_full_design;
    me->battery_full_capacity             = pdata->battery_full_capacity;
    me->r_sns                             = ((pdata->r_sns == 0)?
        GAUGE_DEFAULT_SNS_RESISTOR_UOHM : pdata->r_sns);
    me->update_interval                   = ((pdata->update_interval_ms == 0)?
        GAUGE_DEFAULT_UPD_INTERVAL_JIFFIES : msecs_to_jiffies(pdata->update_interval_ms));

    dev_dbg(me->dev, "Charge full design       %d uAh\n",
        me->charge_full_design);
    dev_dbg(me->dev, "Battery full capacity    %d %%\n",
        me->battery_full_capacity);
    dev_dbg(me->dev, "Sense resistor           %d uOhms\n",
        me->r_sns);
    dev_dbg(me->dev, "Update interval          %u msec\n",
        jiffies_to_msecs(me->update_interval));

    me->polling_properties                = pdata->polling_properties;
    me->support_led_triggers              = pdata->support_led_triggers;
    me->battery_online                    = pdata->battery_online;
    me->charger_online                    = pdata->charger_online;
    me->charger_enable                    = pdata->charger_enable;

    me->default_trigger = &(pdata->default_trigger);
    __show_led_trigger_config(me,
        "default", me->default_trigger);

    me->charging_trigger = &(pdata->charging_trigger);
    __show_led_trigger_config(me,
        "charging", me->charging_trigger);

    me->charging_full_trigger = &(pdata->charging_full_trigger);
    __show_led_trigger_config(me,
        "charging-full", me->charging_full_trigger);

    /* Initialize polling cache */
    __init_cache(me, socrep,     me->update_interval, 0);
    __init_cache(me, tte,        me->update_interval, 0);
    __init_cache(me, temp,       me->update_interval, 0);
    __init_cache(me, vcell,      me->update_interval, 0);
    __init_cache(me, avgvcell,   me->update_interval, 0);
    __init_cache(me, current,    me->update_interval, 0);
    __init_cache(me, avgcurrent, me->update_interval, 0);
    __init_cache(me, remcaprep,  me->update_interval, 0);

    /* Initialize relaxed polling cache */
    __init_cache(me, fullcapnom, me->update_interval << 1, 0);
    __init_cache(me, cycles,     me->update_interval << 1, 0);

    me->irq = max77696_rootint_to_irq(chip, FG);

    /* Disable all fuel gauge interrupts */
    max77696_irq_disable_fgirq(0xFF, 1);
    max77696_gauge_disable_irq(me,
        FG_CONFIG_BER | FG_CONFIG_BEI | FG_CONFIG_AEN, 1);

    rc = request_threaded_irq(me->irq,
        NULL, max77696_gauge_isr, IRQF_ONESHOT, DRIVER_NAME, me);

    if (unlikely(rc < 0)) {
        dev_err(me->dev, "failed to request IRQ(%d) [%d]\n", me->irq, rc);
        goto out_err_req_irq;
    }

//  max77696_gauge_reg_write_word(me, CGAIN, 0x4000);
//  max77696_gauge_reg_write_word(me, COFF,  0x0000);

    pr_info(DRIVER_DESC" "DRIVER_VERSION" Installed\n");

    INIT_WORK(&(me->init_work), max77696_gauge_init_work);
    schedule_work(&(me->init_work));

    return 0;

out_err_req_irq:
    mutex_destroy(&(me->lock));
    platform_set_drvdata(pdev, NULL);
    kfree(me);
    return rc;
}

static __devexit int max77696_gauge_remove (struct platform_device *pdev)
{
    struct max77696_gauge *me = platform_get_drvdata(pdev);

    me->chip->gauge_ptr = NULL;

    free_irq(me->irq, me);
    cancel_delayed_work_sync(&(me->psy_work));

    sysfs_remove_group(me->kobj, &max77696_gauge_attr_group);
    power_supply_unregister(&(me->psy));

    mutex_destroy(&(me->lock));
    platform_set_drvdata(pdev, NULL);
    kfree(me);

    return 0;
}

static struct platform_driver max77696_gauge_driver = {
    .driver.name  = DRIVER_NAME,
    .driver.owner = THIS_MODULE,
    .probe        = max77696_gauge_probe,
    .remove       = __devexit_p(max77696_gauge_remove),
};

static __init int max77696_gauge_driver_init (void)
{
    return platform_driver_register(&max77696_gauge_driver);
}

static __exit void max77696_gauge_driver_exit (void)
{
    platform_driver_unregister(&max77696_gauge_driver);
}

module_init(max77696_gauge_driver_init);
module_exit(max77696_gauge_driver_exit);

/*******************************************************************************
 * EXTERNAL FUNCTIONS
 ******************************************************************************/

extern struct max77696_chip* max77696;

int max77696_gauge_write_cap_param (int id, u16 val)
{
    struct max77696_chip *chip = max77696;
    struct max77696_gauge *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->gauge_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_gauge is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);

    rc = max77696_gauge_cap_param_write(me, id, val);

    __unlock(me);
    return rc;
}
EXPORT_SYMBOL(max77696_gauge_write_cap_param);

int max77696_gauge_read_cap_param (int id, u16 *val)
{
    struct max77696_chip *chip = max77696;
    struct max77696_gauge *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->gauge_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_gauge is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);

    rc = max77696_gauge_cap_param_read(me, id, val);

    __unlock(me);
    return rc;
}
EXPORT_SYMBOL(max77696_gauge_read_cap_param);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);

