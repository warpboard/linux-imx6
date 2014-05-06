/*
 * MAX77696 ADC Driver
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
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/mfd/max77696.h>

#define DRIVER_DESC    "MAX77696 ADC Driver"
#define DRIVER_AUTHOR  "Jayden Cha <jayden.cha@maxim-ic.com>"
#define DRIVER_NAME    MAX77696_ADC_NAME
#define DRIVER_VERSION MAX77696_DRIVER_VERSION".0"

#define DEGREE_SIGN_UTF8               "\xC2\xB0"
#define DEGREE_SIGN                    DEGREE_SIGN_UTF8
#define ADC_UNIT_TEMP                  DEGREE_SIGN"C"
#define ADC_UNIT_VOLT                  "V"
#define ADC_UNIT_AMPS                  "A"

#define ADC_PRINT_RAW                  MAX77696_ADC_PRINT_RAW
#define ADC_PRINT_SCALED               MAX77696_ADC_PRINT_SCALED
#define ADC_PRINT_FULL                 MAX77696_ADC_PRINT_FULL

#define ADC_NR_CHANNELS                MAX77696_ADC_NR_CHS
#define ADC_CONVERSION_TIME_OUT        10   /* in milli-seconds */

#define ADC_ADCCNTL_REG                0x26
//      RESERVED                       BITS(7,5)
#define ADC_ADCCNTL_ADCCONV_M          BIT (4)
#define ADC_ADCCNTL_ADCCONV_S          4
#define ADC_ADCCNTL_ADCAVG_M           BITS(3,2)
#define ADC_ADCCNTL_ADCAVG_S           2
#define ADC_ADCCNTL_ADCREFEN_M         BIT (1)
#define ADC_ADCCNTL_ADCREFEN_S         1
#define ADC_ADCCNTL_ADCEN_M            BIT (0)
#define ADC_ADCCNTL_ADCEN_S            0

#define ADC_ADCDLY_REG                 0x27
//      RESERVED                       BITS(7,5)
#define ADC_ADCDLY_ADCDLY_M            BITS(4,0)
#define ADC_ADCDLY_ADCDLY_S            0

#define ADC_ADCSEL0_REG                0x28
#define ADC_ADCSEL1_REG                0x29

#define ADC_ADCCHSEL_REG               0x2A
//      RESERVED                       BITS(7,5)
#define ADC_ADCCHSEL_ADCCH_M           BITS(4,0)
#define ADC_ADCCHSEL_ADCCH_S           0

#define ADC_ADCDATAL_REG               0x2B
#define ADC_ADCDATAH_REG               0x2C

#define ADC_ADCINT_REG                 0x2E
#define ADC_ADCINTM_REG                0x2F

#define ADC_ADCICNFG_REG               0x30
//      RESERVED                       BITS(7,4)
#define ADC_ADCICNFG_IADCMUX_M         BITS(3,2)
#define ADC_ADCICNFG_IADCMUX_S         2
#define ADC_ADCICNFG_IADC_M            BITS(1,0)
#define ADC_ADCICNFG_IADC_S            0

#define ADC_REG(reg)                   ((u8)(ADC_##reg##_REG))
#define ADC_REG_BITMASK(reg, bit)      ((u8)(ADC_##reg##_##bit##_M))
#define ADC_REG_BITSHIFT(reg, bit)          (ADC_##reg##_##bit##_S)

#define ADC_REG_BITGET(reg, bit, val) \
        ((u8)(((val) & ADC_REG_BITMASK(reg, bit))\
        >> ADC_REG_BITSHIFT(reg, bit)))
#define ADC_REG_BITSET(reg, bit, val) \
        ((u8)(((val) << ADC_REG_BITSHIFT(reg, bit))\
        & ADC_REG_BITMASK(reg, bit)))

/* ADC Sample Average Rate */
#define ADC_AVG_RATE_1_SAMPLE     0
#define ADC_AVG_RATE_2_SAMPLES    1
#define ADC_AVG_RATE_16_SAMPLES   2
#define ADC_AVG_RATE_32_SAMPLES   3

struct max77696_adc {
    struct mutex          lock;
    struct max77696_chip *chip;
    struct max77696_i2c  *i2c;
    struct device        *dev;
    struct kobject       *kobj;

    u8                    print_fmt;
    struct device        *hwmon;
};

#define __get_i2c(chip)                (&((chip)->pmic_i2c))
#define __lock(me)                     mutex_lock(&((me)->lock))
#define __unlock(me)                   mutex_unlock(&((me)->lock))

/* ADC Register Read/Write */
#define max77696_adc_reg_read(me, reg, val_ptr) \
        max77696_read((me)->i2c, ADC_REG(reg), val_ptr)
#define max77696_adc_reg_write(me, reg, val) \
        max77696_write((me)->i2c, ADC_REG(reg), val)
#define max77696_adc_reg_bulk_read(me, reg, dst, len) \
        max77696_bulk_read((me)->i2c, ADC_REG(reg), dst, len)
#define max77696_adc_reg_bulk_write(me, reg, src, len) \
        max77696_bulk_write((me)->i2c, ADC_REG(reg), src, len)
#define max77696_adc_reg_read_masked(me, reg, mask, val_ptr) \
        max77696_read_masked((me)->i2c, ADC_REG(reg), mask, val_ptr)
#define max77696_adc_reg_write_masked(me, reg, mask, val) \
        max77696_write_masked((me)->i2c, ADC_REG(reg), mask, val)

/* ADC Register Single Bit Ops */
#define max77696_adc_reg_get_bit(me, reg, bit, val_ptr) \
        ({\
            int __rc = max77696_adc_reg_read_masked(me, reg,\
                ADC_REG_BITMASK(reg, bit), val_ptr);\
            *(val_ptr) = ADC_REG_BITGET(reg, bit, *(val_ptr));\
            __rc;\
        })
#define max77696_adc_reg_set_bit(me, reg, bit, val) \
        ({\
            max77696_adc_reg_write_masked(me, reg,\
                ADC_REG_BITMASK(reg, bit), ADC_REG_BITSET(reg, bit, val));\
        })

  #define __msleep(msec) msleep_interruptible((unsigned int)(msec))
//#define __msleep(msec) msleep((unsigned int)(msec))
//#define __msleep(msec) mdelay((unsigned int)(msec))

static __inline int max77696_adc_adcdly_read (struct max77696_adc *me, u16 *val)
{
    u8 tmp;
    int rc;

    rc = max77696_adc_reg_get_bit(me, ADCDLY, ADCDLY, &tmp);
    if (unlikely(rc)) {
        dev_err(me->dev, "ADCDLY read error [%d]\n", rc);
        goto out;
    }

    *val = (((u16)tmp + 2) * 500);

out:
    return rc;
}

static __inline int max77696_adc_adcdly_write (struct max77696_adc *me, u16 val)
{
    u8 tmp;
    int rc;

    tmp = ((val > 1000)? ((u8)DIV_ROUND_UP(val, 500) - 2) : 0);

    rc = max77696_adc_reg_set_bit(me, ADCDLY, ADCDLY, tmp);
    if (unlikely(rc)) {
        dev_err(me->dev, "ADCDLY write error [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static __inline int max77696_adc_adcavg_read (struct max77696_adc *me, u8 *val)
{
    u8 tmp;
    int rc;

    rc = max77696_adc_reg_get_bit(me, ADCCNTL, ADCAVG, &tmp);
    if (unlikely(rc)) {
        dev_err(me->dev, "ADCCNTL read error [%d]\n", rc);
        goto out;
    }

    switch (tmp) {
    case ADC_AVG_RATE_1_SAMPLE:
        *val = 1;
        break;

    case ADC_AVG_RATE_2_SAMPLES:
        *val = 2;
        break;

    case ADC_AVG_RATE_16_SAMPLES:
        *val = 16;
        break;

    case ADC_AVG_RATE_32_SAMPLES:
        *val = 32;
        break;
    }

out:
    return rc;
}

static __inline int max77696_adc_adcavg_write (struct max77696_adc *me, u8 val)
{
    u8 tmp;
    int rc;

    if (val >= 32) {
        tmp = ADC_AVG_RATE_32_SAMPLES;
    } else if (val >= 16) {
        tmp = ADC_AVG_RATE_16_SAMPLES;
    } else if (val >= 2) {
        tmp = ADC_AVG_RATE_2_SAMPLES;
    } else {
        tmp = ADC_AVG_RATE_1_SAMPLE;
    }

    rc = max77696_adc_reg_set_bit(me, ADCCNTL, ADCAVG, tmp);
    if (unlikely(rc)) {
        dev_err(me->dev, "ADCCNTL write error [%d]\n", rc);
        goto out;
    }

out:
    return rc;
}

static ssize_t max77696_adc_printfmt_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_adc *me = platform_get_drvdata(pdev);
    int rc;

    __lock(me);

    rc = (int)snprintf(buf, PAGE_SIZE, "%u\n", me->print_fmt);

    rc += (int)snprintf(buf+rc, PAGE_SIZE, "0x%02X RAW     %s\n",
        ADC_PRINT_RAW,    ((me->print_fmt & ADC_PRINT_RAW)?    "yes" : "no"));
    rc += (int)snprintf(buf+rc, PAGE_SIZE, "0x%02X SCALED  %s\n",
        ADC_PRINT_SCALED, ((me->print_fmt & ADC_PRINT_SCALED)? "yes" : "no"));
    rc += (int)snprintf(buf+rc, PAGE_SIZE, "0x%02X FULL    %s\n",
        ADC_PRINT_FULL,   ((me->print_fmt & ADC_PRINT_FULL)?   "yes" : "no"));

    __unlock(me);
    return (ssize_t)rc;
}

static ssize_t max77696_adc_printfmt_store (struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_adc *me = platform_get_drvdata(pdev);

    __lock(me);

    me->print_fmt = (u8)simple_strtoul(buf, NULL, 10);

    __unlock(me);
    return (ssize_t)count;
}

static DEVICE_ATTR(print_fmt, S_IWUSR | S_IRUGO,
    max77696_adc_printfmt_show, max77696_adc_printfmt_store);

static ssize_t max77696_adc_adcavg_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_adc *me = platform_get_drvdata(pdev);
    u8 val;
    int rc;

    __lock(me);

    rc = max77696_adc_adcavg_read(me, &val);
    if (unlikely(rc)) {
        goto out;
    }

    rc  = (int)snprintf(buf, PAGE_SIZE, "%u\n", val);

out:
    __unlock(me);
    return (ssize_t)rc;
}

static ssize_t max77696_adc_adcavg_store (struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_adc *me = platform_get_drvdata(pdev);
    u8 val;
    int rc;

    __lock(me);

    val = (u8)simple_strtoul(buf, NULL, 10);

    rc = max77696_adc_adcavg_write(me, val);
    if (unlikely(rc)) {
        goto out;
    }

out:
    __unlock(me);
    return (ssize_t)count;
}

static DEVICE_ATTR(adc_samples, S_IWUSR | S_IRUGO,
    max77696_adc_adcavg_show, max77696_adc_adcavg_store);

static ssize_t max77696_adc_adcdly_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_adc *me = platform_get_drvdata(pdev);
    u16 val;
    int rc;

    __lock(me);

    rc = max77696_adc_adcdly_read(me, &val);
    if (unlikely(rc)) {
        goto out;
    }

    rc = (int)snprintf(buf, PAGE_SIZE, "%u\n", val);

out:
    __unlock(me);
    return (ssize_t)rc;
}

static ssize_t max77696_adc_adcdly_store (struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_adc *me = platform_get_drvdata(pdev);
    u16 val;
    int rc;

    __lock(me);

    val = (u16)simple_strtoul(buf, NULL, 10);

    rc = max77696_adc_adcdly_write(me, val);
    if (unlikely(rc)) {
        goto out;
    }

out:
    __unlock(me);
    return (ssize_t)count;
}

static DEVICE_ATTR(adc_delay, S_IWUSR | S_IRUGO,
    max77696_adc_adcdly_show, max77696_adc_adcdly_store);

#define max77696_adc_channel_setup_null   NULL
#define max77696_adc_channel_release_null NULL

static int max77696_adc_channel_setup_imon_buck (struct max77696_adc *me,
    u8 channel)
{
    u8 buck = (u8)(channel - MAX77696_ADC_CH_IMONB1);
    return max77696_buck_set_imon_enable(buck, 1);
}

static int max77696_adc_channel_release_imon_buck (struct max77696_adc *me,
    u8 channel)
{
    u8 buck = (u8)(channel - MAX77696_ADC_CH_IMONB1);
    return max77696_buck_set_imon_enable(buck, 0);
}

static int max77696_adc_channel_setup_imon_ldo (struct max77696_adc *me,
    u8 channel)
{
    u8 ldo = (u8)(channel - MAX77696_ADC_CH_IMONL1);
    return max77696_ldo_set_imon_enable(ldo, 1);
}

static int max77696_adc_channel_release_imon_ldo (struct max77696_adc *me,
    u8 channel)
{
    u8 ldo = (u8)(channel - MAX77696_ADC_CH_IMONL1);
    return max77696_ldo_set_imon_enable(ldo, 0);
}

struct max77696_adc_channel {
    u8      physical_channel;
    s32     offset;
    s32     numerator;
    s32     denominator;
    char   *unit_str;

    int (*setup)(struct max77696_adc *me, u8 channel);
    int (*release)(struct max77696_adc *me, u8 channel);
};

#define ADC_CHANNEL(_ch, _phys_ch, _offset, _numerator, _denominator, _unit,\
                    _ctrl_fn) \
        [MAX77696_ADC_CH_##_ch] = {\
            .physical_channel = _phys_ch,\
            .offset           = _offset,\
            .numerator        = _numerator,\
            .denominator      = _denominator,\
            .unit_str         = ADC_UNIT_##_unit,\
            .setup            = max77696_adc_channel_setup_##_ctrl_fn,\
            .release          = max77696_adc_channel_release_##_ctrl_fn,\
        }

static struct max77696_adc_channel max77696_adc_channels[ADC_NR_CHANNELS] =
{
    /* CH00,CH04,CH07,CH08 FORMULA V = 8.192[V] * code / 4095
     *   -> LSB = 8.192 / 4095 = 2.000488...[mV]
     */
    ADC_CHANNEL(VSYS2,   0,      0, 2001, 1000*1000, VOLT, null     ),
    ADC_CHANNEL(VCHGINA, 4,      0, 2001, 1000*1000, VOLT, null     ),
    ADC_CHANNEL(IMONB1,  7,      0, 2001, 1000*1000, VOLT, imon_buck),
    ADC_CHANNEL(IMONB2,  7,      0, 2001, 1000*1000, VOLT, imon_buck),
    ADC_CHANNEL(IMONB3,  8,      0, 2001, 1000*1000, VOLT, imon_buck),
    ADC_CHANNEL(IMONB4,  8,      0, 2001, 1000*1000, VOLT, imon_buck),

    /* CH06,CH09~13 FORMULA V = 2.5[V] * code / 4095
     *   -> LSB = 2.5 / 4095 = 0.610500...[mV]
     */
    ADC_CHANNEL(IMONL1,  6,      0,  611, 1000*1000, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL2,  6,      0,  611, 1000*1000, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL3,  6,      0,  611, 1000*1000, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL4,  6,      0,  611, 1000*1000, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL5,  6,      0,  611, 1000*1000, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL6,  6,      0,  611, 1000*1000, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL7,  6,      0,  611, 1000*1000, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL8,  6,      0,  611, 1000*1000, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL9,  6,      0,  611, 1000*1000, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONL10, 6,      0,  611, 1000*1000, VOLT, imon_ldo ),
    ADC_CHANNEL(IMONB5, 10,      0,  611, 1000*1000, VOLT, imon_buck),
    ADC_CHANNEL(IMONB6, 10,      0,  611, 1000*1000, VOLT, imon_buck),
    ADC_CHANNEL(AIN0,    9,      0,  611, 1000*1000, VOLT, null     ),
    ADC_CHANNEL(AIN1,   11,      0,  611, 1000*1000, VOLT, null     ),
    ADC_CHANNEL(AIN2,   12,      0,  611, 1000*1000, VOLT, null     ),
    ADC_CHANNEL(AIN3,   13,      0,  611, 1000*1000, VOLT, null     ),

    /* CH03 FORMULA V = 1.25[mV] * code
     *   -> LSB = 1.25[mV]
     */
    ADC_CHANNEL(VSYS1,   3,      0, 1250, 1000*1000, VOLT, null     ),

    /* CH05 FORMULA A = 2.54[A] * code / 4095
     *   -> LSB = 2.54 / 4095 = 0.620268...[mA]
     */
    ADC_CHANNEL(ICHGINA, 5,      0,  620, 1000*1000, AMPS, null     ),

    /* CH01 FORMULA temp = (2.5[C] * code / 4095 - 0.7527) / 2.79e-3
     *   -> LSB    = 2.5 / 4095 / 2.79e-3 =   0.218817...[C]
     *   -> offset = 0.7527 / 2.79e-3     = 269.784946...[C]
     */
    ADC_CHANNEL(TDIE,    1, 269785,  219, 1000,      TEMP, null     ),
};

static int max77696_adc_channel_convert (struct max77696_adc *me,
    u8 channel, u16 *raw, s32 *milli_scaled)
{
    struct max77696_adc_channel *adc_ch;
    u8 adc_sel[2], adc_busy, phy_ch;
    u16 _raw;
    s32 _milli_scaled;
    unsigned long timeout;
    int rc;

    adc_ch = &(max77696_adc_channels[channel]);
    phy_ch = adc_ch->physical_channel;

    /* Enable ADC by setting the ADC Enable (ADCEN) bit to 1
     * which in turn forces the ADC reference on
     * even if ADCREFEN bit is set to 0.
     */
    rc = max77696_adc_reg_set_bit(me, ADCCNTL, ADCEN, 1);
    if (unlikely(rc)) {
        dev_err(me->dev, "ADCCNTL write error [%d]\n", rc);
        goto out;
    }

    /* Select ADC channel to be converted. */

    if (phy_ch > 7) {
        adc_sel[0] = 0;
        adc_sel[1] = (1 << (phy_ch - 8));
    } else {
        adc_sel[0] = (1 << (phy_ch    ));
        adc_sel[1] = 0;
    }

    dev_vdbg(me->dev, "ADCSEL0 0x%02X ADCSEL1 0x%02X\n",
        adc_sel[0], adc_sel[1]);

    rc = max77696_adc_reg_bulk_write(me, ADCSEL0, adc_sel, 2);
    if (unlikely(rc)) {
        dev_err(me->dev, "ADCSEL write error [%d]\n", rc);
        goto out;
    }

    /* Setup ADC channel for conversion */
    if (likely(adc_ch->setup)) {
        rc = adc_ch->setup(me, channel);
        if (unlikely(rc)) {
            dev_err(me->dev, "failed to setup channel %u [%d]\n", channel, rc);
            goto out;
        }
    }

    /* Initiate ADC conversion sequence
     * by setting the ADC Start Conversion (ADCCONV) to 1.
     */
    rc = max77696_adc_reg_set_bit(me, ADCCNTL, ADCCONV, 1);
    if (unlikely(rc)) {
        dev_err(me->dev, "ADCCNTL write error [%d]\n", rc);
        goto out;
    }

    /* Check availability of ADC _raw by inspecting the ADCCONV bit.
     * This bit is automatically cleared to 0
     * when an ADC conversion sequence has completed.
     */

    timeout = jiffies + msecs_to_jiffies(ADC_CONVERSION_TIME_OUT);

    do {
        if (unlikely(time_after(jiffies, timeout))) {
            dev_err(me->dev, "adc conversion timed out\n");
            rc = -ETIMEDOUT;
            goto out;
        }
        __msleep(1);
        max77696_adc_reg_get_bit(me, ADCCNTL, ADCCONV, &adc_busy);
    } while (likely(adc_busy));

    /* Read ADC conversion result. */

    rc = max77696_adc_reg_set_bit(me, ADCCHSEL, ADCCH, phy_ch);
    if (unlikely(rc)) {
        dev_err(me->dev, "ADCCHSEL write failed [%d]\n", rc);
        goto out;
    }

    rc = max77696_adc_reg_bulk_read(me, ADCDATAL, (u8*)(&_raw), 2);
    if (unlikely(rc)) {
        dev_err(me->dev, "ADCDATA read failed [%d]\n", rc);
        goto out;
    }

    /* Release ADC channel after conversion */
    if (likely(adc_ch->release)) {
        adc_ch->release(me, channel);
    }

    dev_vdbg(me->dev, "ADCDATAL 0x%02X ADCDATAH 0x%02X\n",
        *((u8*)(&_raw)+0), *((u8*)(&_raw)+1));

    _raw          = (u16)(__le16_to_cpu(_raw) & 0x0fff);
    _milli_scaled = ((s32)(_raw) * adc_ch->numerator) - adc_ch->offset;
    _milli_scaled = DIV_ROUND_UP(_milli_scaled, adc_ch->denominator/1000);

    if (likely(raw)) {
        *raw = _raw;
    }

    if (likely(milli_scaled)) {
        *milli_scaled = _milli_scaled;
    }

out:
    return rc;
}

static ssize_t max77696_adc_channel_show (struct device *dev,
    struct device_attribute *devattr, char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_adc *me = platform_get_drvdata(pdev);
    struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
    struct max77696_adc_channel *adc_channel;
    u8 channel;
    u16 raw;
    s32 scaled_i, scaled_f;
    int rc;

    __lock(me);

    channel     = (u8)(attr->index);
    adc_channel = &(max77696_adc_channels[channel]);

    rc = max77696_adc_channel_convert(me, channel, &raw, &scaled_i);
    if (unlikely(rc)) {
        goto out;
    }

    scaled_f = (scaled_i % 1000);
    scaled_i = (scaled_i / 1000);

    if (likely(me->print_fmt & ADC_PRINT_RAW)) {
        rc += (int)snprintf(buf+rc, PAGE_SIZE, "%u\n", raw);
    }

    if (likely(me->print_fmt & ADC_PRINT_SCALED)) {
        rc += (int)snprintf(buf+rc, PAGE_SIZE, "%d\n",
            scaled_i + (scaled_f >= 500));
    }

    if (likely(me->print_fmt & ADC_PRINT_FULL)) {
        rc += (int)snprintf(buf+rc, PAGE_SIZE, "%d.%03d %s\n",
            scaled_i, scaled_f, adc_channel->unit_str);
    }

out:
    __unlock(me);
    return (ssize_t)rc;
}

#define ADC_CHANNEL_DEV_ATTR(_name, _ch) \
        SENSOR_DEVICE_ATTR(_name, S_IRUGO,\
            max77696_adc_channel_show, NULL, MAX77696_ADC_CH_##_ch)

static ADC_CHANNEL_DEV_ATTR(vsys2,   VSYS2  );
static ADC_CHANNEL_DEV_ATTR(tdie,    TDIE   );
static ADC_CHANNEL_DEV_ATTR(vsys1,   VSYS1  );
static ADC_CHANNEL_DEV_ATTR(vchgina, VCHGINA);
static ADC_CHANNEL_DEV_ATTR(ichgina, ICHGINA);
static ADC_CHANNEL_DEV_ATTR(imonl1,  IMONL1 );
static ADC_CHANNEL_DEV_ATTR(imonl2,  IMONL2 );
static ADC_CHANNEL_DEV_ATTR(imonl3,  IMONL3 );
static ADC_CHANNEL_DEV_ATTR(imonl4,  IMONL4 );
static ADC_CHANNEL_DEV_ATTR(imonl5,  IMONL5 );
static ADC_CHANNEL_DEV_ATTR(imonl6,  IMONL6 );
static ADC_CHANNEL_DEV_ATTR(imonl7,  IMONL7 );
static ADC_CHANNEL_DEV_ATTR(imonl8,  IMONL8 );
static ADC_CHANNEL_DEV_ATTR(imonl9,  IMONL9 );
static ADC_CHANNEL_DEV_ATTR(imonl10, IMONL10);
static ADC_CHANNEL_DEV_ATTR(imonb1,  IMONB1 );
static ADC_CHANNEL_DEV_ATTR(imonb2,  IMONB2 );
static ADC_CHANNEL_DEV_ATTR(imonb3,  IMONB3 );
static ADC_CHANNEL_DEV_ATTR(imonb4,  IMONB4 );
static ADC_CHANNEL_DEV_ATTR(imonb5,  IMONB5 );
static ADC_CHANNEL_DEV_ATTR(imonb6,  IMONB6 );
static ADC_CHANNEL_DEV_ATTR(ain0,    AIN0   );
static ADC_CHANNEL_DEV_ATTR(ain1,    AIN1   );
static ADC_CHANNEL_DEV_ATTR(ain2,    AIN2   );
static ADC_CHANNEL_DEV_ATTR(ain3,    AIN3   );

static struct attribute* max77696_adc_attr[] = {
    &dev_attr_print_fmt.attr,

    /* ADC Sample Average Rate */
    &dev_attr_adc_samples.attr,

    /* ADC Delay (in nano-seconds) */
    &dev_attr_adc_delay.attr,

    /* ADC Channels */
    &sensor_dev_attr_vsys2.dev_attr.attr,
    &sensor_dev_attr_tdie.dev_attr.attr,
    &sensor_dev_attr_vsys1.dev_attr.attr,
    &sensor_dev_attr_vchgina.dev_attr.attr,
    &sensor_dev_attr_ichgina.dev_attr.attr,
    &sensor_dev_attr_imonl1.dev_attr.attr,
    &sensor_dev_attr_imonl2.dev_attr.attr,
    &sensor_dev_attr_imonl3.dev_attr.attr,
    &sensor_dev_attr_imonl4.dev_attr.attr,
    &sensor_dev_attr_imonl5.dev_attr.attr,
    &sensor_dev_attr_imonl6.dev_attr.attr,
    &sensor_dev_attr_imonl7.dev_attr.attr,
    &sensor_dev_attr_imonl8.dev_attr.attr,
    &sensor_dev_attr_imonl9.dev_attr.attr,
    &sensor_dev_attr_imonl10.dev_attr.attr,
    &sensor_dev_attr_imonb1.dev_attr.attr,
    &sensor_dev_attr_imonb2.dev_attr.attr,
    &sensor_dev_attr_imonb3.dev_attr.attr,
    &sensor_dev_attr_imonb4.dev_attr.attr,
    &sensor_dev_attr_imonb5.dev_attr.attr,
    &sensor_dev_attr_imonb6.dev_attr.attr,
    &sensor_dev_attr_ain0.dev_attr.attr,
    &sensor_dev_attr_ain1.dev_attr.attr,
    &sensor_dev_attr_ain2.dev_attr.attr,
    &sensor_dev_attr_ain3.dev_attr.attr,
    NULL
};

static const struct attribute_group max77696_adc_attr_group = {
    .attrs = max77696_adc_attr,
};

static __devinit int max77696_adc_probe (struct platform_device *pdev)
{
    struct max77696_chip *chip = dev_get_drvdata(pdev->dev.parent);
    struct max77696_adc_platform_data *pdata = dev_get_platdata(&(pdev->dev));
    struct max77696_adc *me;
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

    me->print_fmt = pdata->print_fmt;

    /* We don't use any ADC interrupts. We are just polling. */
    max77696_adc_reg_write(me, ADCINTM, 0xFF);

    me->hwmon = hwmon_device_register(me->dev);
    if (unlikely(IS_ERR(me->hwmon))) {
        rc = PTR_ERR(me->hwmon);
        me->hwmon = NULL;

        dev_err(me->dev, "failed to register hwmon device [%d]\n", rc);
        goto out_err;
    }

    rc = sysfs_create_group(me->kobj, &max77696_adc_attr_group);
    if (unlikely(rc)) {
        dev_err(me->dev, "failed to create attribute group [%d]\n", rc);
        goto out_err;
    }

    BUG_ON(chip->adc_ptr);
    chip->adc_ptr = me;

    /* Set defaults given via platform data */
    max77696_adc_adcavg_write(me, pdata->avg_rate);
    max77696_adc_adcdly_write(me, pdata->adc_delay);

    /* Show defaults */
    dev_dbg(me->dev, "average rate: %d sample(s)\n",
        min(max((int)(pdata->avg_rate),     1),    32));
    dev_dbg(me->dev, "adc delay:    %d nsec\n",
        min(max((int)(pdata->adc_delay), 1000), 16500));

    pr_info(DRIVER_DESC" "DRIVER_VERSION" Installed\n");
    return 0;

out_err:
    if (likely(me->hwmon)) {
        hwmon_device_unregister(me->hwmon);
    }
    mutex_destroy(&(me->lock));
    platform_set_drvdata(pdev, NULL);
    kfree(me);
    return rc;
}

static __devexit int max77696_adc_remove (struct platform_device *pdev)
{
    struct max77696_adc *me = platform_get_drvdata(pdev);

    me->chip->adc_ptr = NULL;

    sysfs_remove_group(me->kobj, &max77696_adc_attr_group);
    hwmon_device_unregister(me->hwmon);

    mutex_destroy(&(me->lock));
    platform_set_drvdata(pdev, NULL);
    kfree(me);

    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77696_adc_suspend (struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_adc *me = platform_get_drvdata(pdev);

    /* Abort ADC operation by clearing ADCCONV to 0. */
    max77696_adc_reg_set_bit(me, ADCCNTL, ADCCONV, 0);

    /* Disable ADC reference and core. */
    max77696_adc_reg_set_bit(me, ADCCNTL, ADCREFEN, 0);
    max77696_adc_reg_set_bit(me, ADCCNTL, ADCEN, 0);

    return 0;
}

static int max77696_adc_resume (struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct max77696_adc *me = platform_get_drvdata(pdev);

    /* Enable ADC reference. */
    max77696_adc_reg_set_bit(me, ADCCNTL, ADCREFEN, 1);

    return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77696_adc_pm,
    max77696_adc_suspend, max77696_adc_resume);

static struct platform_driver max77696_adc_driver = {
    .driver.name  = DRIVER_NAME,
    .driver.owner = THIS_MODULE,
    .driver.pm    = &max77696_adc_pm,
    .probe        = max77696_adc_probe,
    .remove       = __devexit_p(max77696_adc_remove),
};

static __init int max77696_adc_driver_init (void)
{
    return platform_driver_register(&max77696_adc_driver);
}

static __exit void max77696_adc_driver_exit (void)
{
    platform_driver_unregister(&max77696_adc_driver);
}

module_init(max77696_adc_driver_init);
module_exit(max77696_adc_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);

/*******************************************************************************
 * EXTERNAL FUNCTIONS
 ******************************************************************************/

extern struct max77696_chip* max77696;

int max77696_adc_read (u8 channel, u16 *raw, s32 *milli_scaled)
{
    struct max77696_chip *chip = max77696;
    struct max77696_adc *me;
    int rc;

    if (unlikely(!chip)) {
        pr_err("%s: max77696_chip is not ready\n", __func__);
        return -ENODEV;
    }

    me = chip->adc_ptr;

    if (unlikely(!me)) {
        pr_err("%s: max77696_adc is not ready\n", __func__);
        return -ENODEV;
    }

    __lock(me);

    if (unlikely(channel >= ADC_NR_CHANNELS)) {
        rc = -EINVAL;
        goto out;
    }

    rc = max77696_adc_channel_convert(me, channel, raw, milli_scaled);

out:
    __unlock(me);
    return rc;
}
EXPORT_SYMBOL(max77696_adc_read);

