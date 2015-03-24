/*
 * MAX77696 Machine-Depends
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>

#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/mfd/max77696.h>

#define CONFIG_MFD_MAX77696_I2C_BUS        0
#define CONFIG_MFD_MAX77696_IRQ            0
#define CONFIG_MFD_MAX77696_IRQ_BASE       250
#define CONFIG_MFD_MAX77696_GPIO_BASE      250

/* TODO: chk 1.2V */
#define CONFIG_MFD_MAX77696_VDDQIN_SUPP    "max77696_buck4"
#define CONFIG_MFD_MAX77696_VDDQIN_VOLT    mV_to_uV(1200)

#define mV_to_uV(mV)  (mV * 1000)
#define uV_to_mV(uV)  (uV / 1000)
#define V_to_uV(V)    (mV_to_uV(V * 1000))
#define uV_to_V(uV)   (uV_to_mV(uV) / 1000)

/******************************************************************************/

#define MAX77696_GPIO_INIT_NC(_gpio) \
        [_gpio] = {\
            .pin_connected = 0,\
        }

#define MAX77696_GPIO_INIT_OUTPUT(_gpio,\
                                  _mode, _pu_en, _pd_en, _cfg, _drv_high) \
        [_gpio] = {\
            .pin_connected = 1,\
            .alter_mode    = MAX77696_GPIO_AME_##_mode,\
            .pullup_en     = _pu_en,\
            .pulldn_en     = _pd_en,\
            .direction     = MAX77696_GPIO_DIR_OUTPUT,\
            .u.output      = {\
                .out_cfg = MAX77696_GPIO_OUTCFG_##_cfg,\
                .drive   = _drv_high,\
            },\
        }

#define MAX77696_GPIO_INIT_INPUT(_gpio,\
                                 _mode, _pu_en, _pd_en, _dbnc, _refe) \
        [_gpio] = {\
            .pin_connected = 1,\
            .alter_mode    = MAX77696_GPIO_AME_##_mode,\
            .pullup_en     = _pu_en,\
            .pulldn_en     = _pd_en,\
            .direction     = MAX77696_GPIO_DIR_INPUT,\
            .u.input       = {\
                .debounce = MAX77696_GPIO_DBNC_##_dbnc##_MSEC,\
                .refe_irq = MAX77696_GPIO_REFEIRQ_##_refe,\
            },\
        }

/******************************************************************************/

#define MAX77696_VREG_CONSUMERS_NAME(_id) \
        max77696_vreg_consumers_##_id
#define MAX77696_VREG_CONSUMERS(_id) \
        static struct regulator_consumer_supply \
            MAX77696_VREG_CONSUMERS_NAME(_id)[]

#define MAX77696_BUCK_INIT(_id, _name, _min_uV, _max_uV,\
                           _apply_uV, _boot_on, _always_on)\
        .init_data[MAX77696_BUCK_ID_##_id] = {\
            .constraints = {\
                .valid_modes_mask   = REGULATOR_MODE_FAST |\
                                      REGULATOR_MODE_NORMAL |\
                                      REGULATOR_MODE_IDLE |\
                                      REGULATOR_MODE_STANDBY,\
                .valid_ops_mask     = REGULATOR_CHANGE_VOLTAGE |\
                                      REGULATOR_CHANGE_STATUS |\
                                      REGULATOR_CHANGE_MODE |\
                                      REGULATOR_CHANGE_DRMS,\
                .min_uV             = _min_uV,\
                .max_uV             = _max_uV,\
                .input_uV           = _max_uV,\
                .apply_uV           = _apply_uV,\
                .always_on          = _always_on,\
                .boot_on            = _boot_on,\
                .name               = _name,\
            },\
            .num_consumer_supplies =\
                ARRAY_SIZE(MAX77696_VREG_CONSUMERS_NAME(_id)),\
            .consumer_supplies = MAX77696_VREG_CONSUMERS_NAME(_id),\
            .supply_regulator = NULL,\
        }
#define MAX77696_LDO_INIT(_id, _name, _min_uV, _max_uV,\
                          _apply_uV, _boot_on, _always_on, _supply_regulator)\
        .init_data[MAX77696_LDO_ID_##_id] = {\
            .constraints = {\
                .valid_modes_mask   = REGULATOR_MODE_NORMAL |\
                                      REGULATOR_MODE_IDLE |\
                                      REGULATOR_MODE_STANDBY,\
                .valid_ops_mask     = REGULATOR_CHANGE_VOLTAGE |\
                                      REGULATOR_CHANGE_STATUS |\
                                      REGULATOR_CHANGE_MODE |\
                                      REGULATOR_CHANGE_DRMS,\
                .min_uV             = _min_uV,\
                .max_uV             = _max_uV,\
                .input_uV           = _max_uV,\
                .apply_uV           = _apply_uV,\
                .always_on          = _always_on,\
                .boot_on            = _boot_on,\
                .name               = _name,\
            },\
            .num_consumer_supplies =\
                ARRAY_SIZE(MAX77696_VREG_CONSUMERS_NAME(_id)),\
            .consumer_supplies = MAX77696_VREG_CONSUMERS_NAME(_id),\
            .supply_regulator = _supply_regulator,\
        }
#define MAX77696_LSW_INIT(_id, _name, _boot_on, _always_on, _supply_regulator)\
        .init_data[MAX77696_LSW_ID_##_id] = {\
            .constraints = {\
                .valid_modes_mask   = 0,\
                .valid_ops_mask     = REGULATOR_CHANGE_STATUS,\
                .min_uV             = 0,\
                .max_uV             = 0,\
                .input_uV           = 0,\
                .apply_uV           = 0,\
                .always_on          = _always_on,\
                .boot_on            = _boot_on,\
                .name               = _name,\
            },\
            .num_consumer_supplies =\
                ARRAY_SIZE(MAX77696_VREG_CONSUMERS_NAME(_id)),\
            .consumer_supplies = MAX77696_VREG_CONSUMERS_NAME(_id),\
            .supply_regulator = _supply_regulator,\
        }
#define MAX77696_EPD_INIT(_id, _name, _min_uV, _max_uV, _ops, _apply_uV,\
                          _boot_on, _always_on)\
        .init_data[MAX77696_EPD_ID_##_id] = {\
            .constraints = {\
                .valid_modes_mask   = 0,\
                .valid_ops_mask     = _ops,\
                .min_uV             = _min_uV,\
                .max_uV             = _max_uV,\
                .input_uV           = _max_uV,\
                .apply_uV           = _apply_uV,\
                .always_on          = _always_on,\
                .boot_on            = _boot_on,\
                .name               = _name,\
            },\
            .num_consumer_supplies =\
                ARRAY_SIZE(MAX77696_VREG_CONSUMERS_NAME(_id)),\
            .consumer_supplies = MAX77696_VREG_CONSUMERS_NAME(_id),\
            .supply_regulator = NULL,\
        }
#define MAX77696_VDDQ_INIT(_id, _name, _min_uV, _max_uV, _apply_uV)\
        .init_data = {\
            .constraints = {\
                .valid_modes_mask   = 0,\
                .valid_ops_mask     = REGULATOR_CHANGE_VOLTAGE,\
                .min_uV             = _min_uV,\
                .max_uV             = _max_uV,\
                .input_uV           = _max_uV,\
                .apply_uV           = _apply_uV,\
                .always_on          = 1,\
                .boot_on            = 1,\
                .name               = _name,\
            },\
            .num_consumer_supplies =\
                ARRAY_SIZE(MAX77696_VREG_CONSUMERS_NAME(_id)),\
            .consumer_supplies = MAX77696_VREG_CONSUMERS_NAME(_id),\
            .supply_regulator = CONFIG_MFD_MAX77696_VDDQIN_SUPP,\
        }

#ifdef CONFIG_REGULATOR_MAX77696

/* BUCK Consumers */
MAX77696_VREG_CONSUMERS(B1) = {
    REGULATOR_SUPPLY("max77696_buck1",      NULL),
    REGULATOR_SUPPLY("VDDCORE",             NULL),
};
MAX77696_VREG_CONSUMERS(B1DVS) = {
    REGULATOR_SUPPLY("max77696_buck1dvs",   NULL),
};
MAX77696_VREG_CONSUMERS(B2) = {
    REGULATOR_SUPPLY("max77696_buck2",      NULL),
    REGULATOR_SUPPLY("VDDSOC",              NULL),
};
MAX77696_VREG_CONSUMERS(B2DVS) = {
    REGULATOR_SUPPLY("max77696_buck2dvs",   NULL),
};
MAX77696_VREG_CONSUMERS(B3) = {
    REGULATOR_SUPPLY("max77696_buck3",      NULL),
};
MAX77696_VREG_CONSUMERS(B4) = {
    REGULATOR_SUPPLY("max77696_buck4",      NULL),
};
MAX77696_VREG_CONSUMERS(B5) = {
    REGULATOR_SUPPLY("max77696_buck5",      NULL),
};
MAX77696_VREG_CONSUMERS(B6) = {
    REGULATOR_SUPPLY("max77696_buck6",      NULL),
};

/* LDO Consumers */
MAX77696_VREG_CONSUMERS(L1) = {
    REGULATOR_SUPPLY("max77696_ldo1",       NULL),
};
MAX77696_VREG_CONSUMERS(L2) = {
    REGULATOR_SUPPLY("max77696_ldo2",       NULL),
};
MAX77696_VREG_CONSUMERS(L3) = {
    REGULATOR_SUPPLY("max77696_ldo3",       NULL),
};
MAX77696_VREG_CONSUMERS(L4) = {
    REGULATOR_SUPPLY("max77696_ldo4",       NULL),
};
MAX77696_VREG_CONSUMERS(L5) = {
    REGULATOR_SUPPLY("max77696_ldo5",       NULL),
};
MAX77696_VREG_CONSUMERS(L6) = {
    REGULATOR_SUPPLY("max77696_ldo6",       NULL),
};
MAX77696_VREG_CONSUMERS(L7) = {
    REGULATOR_SUPPLY("max77696_ldo7",       NULL),
};
MAX77696_VREG_CONSUMERS(L8) = {
    REGULATOR_SUPPLY("max77696_ldo8",       NULL),
};
MAX77696_VREG_CONSUMERS(L9) = {
    REGULATOR_SUPPLY("max77696_ldo9",       NULL),
};
MAX77696_VREG_CONSUMERS(L10) = {
    REGULATOR_SUPPLY("max77696_ldo10",      NULL),
};

/* LSW Consumers */
MAX77696_VREG_CONSUMERS(LSW1) = {
    REGULATOR_SUPPLY("max77696_lsw1",       NULL),
};
MAX77696_VREG_CONSUMERS(LSW2) = {
    REGULATOR_SUPPLY("max77696_lsw2",       NULL),
};
MAX77696_VREG_CONSUMERS(LSW3) = {
    REGULATOR_SUPPLY("max77696_lsw3",       NULL),
};
MAX77696_VREG_CONSUMERS(LSW4) = {
    REGULATOR_SUPPLY("max77696_lsw4",       NULL),
};

/* EPD Consumers */
MAX77696_VREG_CONSUMERS(DISP) = {
    REGULATOR_SUPPLY("max77696_disp",       NULL),
    REGULATOR_SUPPLY("DISPLAY",             NULL),
};
MAX77696_VREG_CONSUMERS(VCOM) = {
    REGULATOR_SUPPLY("max77696_vcom",       NULL),
    REGULATOR_SUPPLY("VCOM",                NULL),
};
MAX77696_VREG_CONSUMERS(VEE) = {
    REGULATOR_SUPPLY("max77696_vee",        NULL),
};
MAX77696_VREG_CONSUMERS(VNEG) = {
    REGULATOR_SUPPLY("max77696_vneg",       NULL),
};
MAX77696_VREG_CONSUMERS(VPOS) = {
    REGULATOR_SUPPLY("max77696_vpos",       NULL),
};
MAX77696_VREG_CONSUMERS(VDDH) = {
    REGULATOR_SUPPLY("max77696_vddh",       NULL),
};

/* VDDQ Consumer */
MAX77696_VREG_CONSUMERS(VDDQ) = {
    REGULATOR_SUPPLY("max77696_vddq",       NULL),
};
#endif /* CONFIG_REGULATOR_MAX77696 */

/******************************************************************************/

#ifdef CONFIG_BATTERY_MAX77696

static char* max77696_batteries[] = {
    MAX77696_PSY_BATT_NAME,
};

static struct max77696_gauge_config max77696_gauge_config_870_1_041312 = {
    .config =           0x2210,
    .filter_cfg =       0x87A4,
    .learn_cfg =        0x2606,
    .full_soc_thresh =  0x5A00,
    .rcomp0 =           0x0041,
    .tempco =           0x1121,
    .ichg_term =        0x0100,
    .vempty =           0xACDA,
    .qrtbl00 =          0x2186,
    .qrtbl10 =          0x2186,
    .qrtbl20 =          0x0680,
    .qrtbl30 =          0x0501,
    .cycles =           0x0060,
    .fullcap =          0x02BC,
    .design_cap =       0x02BC,
    .fullcapnom =       0x02BC,
    .batt_cap =         0x02BC,
    .cell_char_tbl = {
        /* 0x80 */
        0xA8D0, 0xB680, 0xB960, 0xBB80, 0xBBE0, 0xBC30, 0xBD50, 0xBE20,
        0xBE80, 0xC090, 0xC5A0, 0xC730, 0xC9D0, 0xCC30, 0xCE90, 0xD110,
        /* 0x90 */
        0x0160, 0x0D30, 0x0F30, 0x3260, 0x0CD0, 0x1E90, 0x2500, 0x2FD0,
        0x11F0, 0x0D80, 0x0A40, 0x0A70, 0x09C0, 0x09F0, 0x04E0, 0x04E0,
        /* 0xA0 */
        0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
        0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
    },
};

#endif /* CONFIG_BATTERY_MAX77696 */

/******************************************************************************/

static struct max77696_platform_data max77696_pdata = {
    .core_irq         = CONFIG_MFD_MAX77696_IRQ,
    .core_irq_trigger = IRQF_TRIGGER_LOW,
    .irq_base         = CONFIG_MFD_MAX77696_IRQ_BASE,

    .core_supports_debugging_stuff          = 1,

    .core_mbattlow_falling_threshold_mV     = 3200, /* cf. default 3100 */
    .core_mbattlow_comparator_hysteresis_mV =  300, /* cf. default  400 */

    .osc_pdata = {
        .load_cap = MAX77696_32K_LOAD_CAP_22PF,
        .op_mode  = MAX77696_32K_MODE_LOW_POWER,
    },

#ifdef CONFIG_GPIO_MAX77696
/* TODO: configure GPIO for SSD2805 settings
    .gpio_pdata = {
        .irq_base  = CONFIG_MFD_MAX77696_IRQ_BASE+MAX77696_GPIO_IRQ_OFFSET,
        .gpio_base = CONFIG_MFD_MAX77696_GPIO_BASE,
        .bias_en   = 1,
        .init_data = {
            MAX77696_GPIO_INIT_INPUT (0, STDGPIO, 0, 0, 32, NONE),
            MAX77696_GPIO_INIT_NC    (1                         ),
            MAX77696_GPIO_INIT_NC    (2                         ),
            MAX77696_GPIO_INIT_INPUT (3, STDGPIO, 0, 0,  0, NONE),
            MAX77696_GPIO_INIT_NC    (4                         ),
        },
    },
*/
#endif /* CONFIG_GPIO_MAX77696 */

#ifdef CONFIG_WATCHDOG_MAX77696
    .wdt_pdata = {
        .timeout_sec       = 128,
        .ping_interval_sec = 60,
    },
#endif /* CONFIG_WATCHDOG_MAX77696 */

#ifdef CONFIG_RTC_DRV_MAX77696
    .rtc_pdata = {
        .irq_1m = 0,
        .irq_1s = 0,
    },
#endif /* CONFIG_RTC_DRV_MAX77696 */

#ifdef CONFIG_REGULATOR_MAX77696
    .buck_pdata = {
        .support_suspend_ops = 0,

        /*                 id                           min_uV           apply_uV  always_on
                                  name                          max_uV      boot_on
                           --------------------------------------------------------*/
        MAX77696_BUCK_INIT(B1,    "max77696_buck1",     725000, 1300000, 0, 1,     1),
        MAX77696_BUCK_INIT(B1DVS, "max77696_buck1dvs",  600000, 1300000, 0, 0,     0),
        MAX77696_BUCK_INIT(B2,    "max77696_buck2",     725000, 1300000, 0, 0,     0),
        MAX77696_BUCK_INIT(B2DVS, "max77696_buck2dvs",  600000, 1300000, 0, 0,     0),
        MAX77696_BUCK_INIT(B3,    "max77696_buck3",     600000, 3387500, 0, 0,     0),
        MAX77696_BUCK_INIT(B4,    "max77696_buck4",    1200000, 1200000, 1, 1,     1),
        MAX77696_BUCK_INIT(B5,    "max77696_buck5",    1800000, 1800000, 0, 0,     1),
        MAX77696_BUCK_INIT(B6,    "max77696_buck6",    3000000, 3000000, 0, 0,     1),
    },
    .ldo_pdata = {
        .imon_tf = MAX77696_LDO_IMON_TF_1000_OHM,

        /*                id                     min_uV            apply_uV  always_on
                               name                       max_uV      boot_on    supply_regulator
                          -----------------------------------------------------------*/
        MAX77696_LDO_INIT(L1,  "max77696_ldo1",   800000, 3950000, 0, 0,      0, NULL),
        MAX77696_LDO_INIT(L2,  "max77696_ldo2",   800000, 3950000, 0, 0,      0, NULL),
        MAX77696_LDO_INIT(L3,  "max77696_ldo3",   800000, 3950000, 0, 0,      0, NULL),
        MAX77696_LDO_INIT(L4,  "max77696_ldo4",   1800000, 1800000, 0, 0,      0, NULL),
        MAX77696_LDO_INIT(L5,  "max77696_ldo5",   800000, 2375000, 0, 0,      0, NULL),
        MAX77696_LDO_INIT(L6,  "max77696_ldo6",   800000, 3950000, 0, 0,      0, NULL),
        MAX77696_LDO_INIT(L7,  "max77696_ldo7",   800000, 3950000, 0, 0,      0, NULL),
        MAX77696_LDO_INIT(L8,  "max77696_ldo8",   800000, 2375000, 0, 0,      0, NULL),
        MAX77696_LDO_INIT(L9,  "max77696_ldo9",   800000, 2375000, 0, 0,      0, NULL),
        MAX77696_LDO_INIT(L10, "max77696_ldo10", 2400000, 5550000, 0, 0,      0, NULL),
    },
    .lsw_pdata = {
        /*                id    name             boot_on  always_on  supply_regulator
                          -----------------------------------------------*/
        MAX77696_LSW_INIT(LSW1, "max77696_lsw1", 0,       0,         NULL),
        MAX77696_LSW_INIT(LSW2, "max77696_lsw2", 0,       0,         NULL),
        MAX77696_LSW_INIT(LSW3, "max77696_lsw3", 0,       0,         NULL),
        MAX77696_LSW_INIT(LSW4, "max77696_lsw4", 0,       0,         NULL),
    },
    .epd_pdata = {
        .pok_wait_timeout_ms = 10000,
        .control_vcom_en     = 1,

        /*                id                     min_uV                ops                                                  boot_on
                                name                        max_uV                                                       apply_uV  always_on
                          ----------------------------------------------------------------------------------------------------------*/
        MAX77696_EPD_INIT(DISP, "max77696_disp",         0,         0, REGULATOR_CHANGE_STATUS,                          0, 0,     0),
        MAX77696_EPD_INIT(VCOM, "max77696_vcom", - 5000000,         0, REGULATOR_CHANGE_STATUS|REGULATOR_CHANGE_VOLTAGE, 0, 0,     0),
        MAX77696_EPD_INIT(VEE,  "max77696_vee",  -28020000, -15000000,                         REGULATOR_CHANGE_VOLTAGE, 0, 0,     0),
        MAX77696_EPD_INIT(VNEG, "max77696_vneg", - 1600000,   1587500,                         REGULATOR_CHANGE_VOLTAGE, 0, 0,     0),
        MAX77696_EPD_INIT(VPOS, "max77696_vpos",   8000000,  18000000,                         REGULATOR_CHANGE_VOLTAGE, 0, 0,     0),
        MAX77696_EPD_INIT(VDDH, "max77696_vddh",  15000000,  29500000,                         REGULATOR_CHANGE_VOLTAGE, 0, 0,     0),
    },
    .vddq_pdata = {
        #define VDDQIN CONFIG_MFD_MAX77696_VDDQIN_VOLT
        .vddq_in_uV = VDDQIN,
        /* VDDQOUT margin = -60% ~ +64% of VDDQIN/2 */
        MAX77696_VDDQ_INIT(VDDQ,                       /* id        */
                           "max77696_vddq",            /* name      */
                           (VDDQIN/2)-0.60*(VDDQIN/2), /* min_uV    */
                           (VDDQIN/2)+0.64*(VDDQIN/2), /* max_uV    */
                           1),                         /* always_on */
    },
#endif /* CONFIG_REGULATOR_MAX77696 */

#ifdef CONFIG_LEDS_MAX77696
    .led_pdata = {
#if 0//def CONFIG_LEDS_TRIGGERS
        [MAX77696_LED_GREEN] = {
            .manual_mode = 1,
            .info        = {
                .name            = MAX77696_LEDS_NAME".0",
                .default_trigger = MAX77696_GAUGE_NAME"-full",
            },
        },
        [MAX77696_LED_AMBER] = {
            .manual_mode = 1,
            .info        = {
                .name            = MAX77696_LEDS_NAME".1",
                .default_trigger = MAX77696_GAUGE_NAME"-charging",
            },
        },
#else /* CONFIG_LEDS_TRIGGERS */
        [MAX77696_LED_GREEN] = {
            .info = {
                .name = MAX77696_LEDS_NAME".0",
            },
        },
        [MAX77696_LED_AMBER] = {
            .info = {
                .name = MAX77696_LEDS_NAME".1",
            },
        },
#endif /* CONFIG_LEDS_TRIGGERS */
    },
#endif /* CONFIG_LEDS_MAX77696 */

#ifdef CONFIG_BACKLIGHT_MAX77696
        /* no pdata */
#endif /* CONFIG_BACKLIGHT_MAX77696 */

#ifdef CONFIG_SENSORS_MAX77696
    .adc_pdata = {
        .print_fmt = MAX77696_ADC_PRINT_FULL,
        .avg_rate  = 1, /*    1 ~    32 samples, 0 means minimum */
        .adc_delay = 1000, /* 1000 ~ 16500 nsec,    0 means minimum */
    },
#endif /* CONFIG_SENSORS_MAX77696 */

#ifdef CONFIG_BATTERY_MAX77696
    .gauge_pdata = {
        .config                            = &max77696_gauge_config_870_1_041312,
        .enable_por_init                   = 1,
        /* TODO: to be filled later */
        .v_alert_max                       = 5100, /* mV */
        .v_alert_min                       = 3600, /* mV */
        .t_alert_max                       = 0,
        .t_alert_min                       = 0,
        .s_alert_max                       = 0,
        .s_alert_min                       = 0,
        .enable_alert_on_battery_removal   = 0,
        .enable_alert_on_battery_insertion = 0,
        .charge_full_design                = 350*1000, /* in uAh */
        .battery_full_capacity             = 95,       /* in % */
        .r_sns                             = 10000,    /* SENSE_RESISTOR = 10mOhm */
        .update_interval_ms                = 5000,
        .polling_properties                =
            MAX77696_GAUGE_POLLING_CAPACITY|
            MAX77696_GAUGE_POLLING_TEMP,

        /* Must select one of both CONFIG_LEDS_TRIGGERS and
        * GAUGE_SUPPORTS_LEDS_TRIGGERS
        */
#ifndef CONFIG_LEDS_TRIGGERS
        .support_led_triggers              = 1,
#endif /* !CONFIG_LEDS_TRIGGERS */

        .default_trigger                   = {
            .enable                      = 1,
            .control[MAX77696_LED_GREEN] = {
                .manual_mode = 0,
            },
            .control[MAX77696_LED_AMBER] = {
                .manual_mode = 0,
            },
        },
        .charging_trigger                  = {
            .enable                      = 1,
            .control[MAX77696_LED_GREEN] = {
                .manual_mode = 1,
                .brightness  = LED_OFF,
                .flashing    = 0,
            },
            .control[MAX77696_LED_AMBER] = {
                .manual_mode = 1,
                .brightness  = LED_FULL,
                .flashing    = 0,
            },
        },
        .charging_full_trigger             = {
            .enable                      = 1,
            .control[MAX77696_LED_GREEN] = {
                .manual_mode = 1,
                .brightness  = LED_FULL,
                .flashing    = 0,
            },
            .control[MAX77696_LED_AMBER] = {
                .manual_mode = 1,
                .brightness  = LED_OFF,
                .flashing    = 0,
            },
        },
        .battery_online = NULL,
        .charger_online = max77696_charger_online_def_cb,
        .charger_enable = max77696_charger_enable_def_cb,
    },
#endif /* CONFIG_BATTERY_MAX77696 */

#ifdef CONFIG_CHARGER_MAX77696_UIC
    .uic_pdata = {
        .int_type     = MAX77696_UIC_INTTYP_LEVEL,      /* device default: Level      */
        .int_delay    = MAX77696_UIC_INTDLY_2TICKS,     /* device default: 2 ticks    */
        .int_polarity = MAX77696_UIC_INTPOL_ACTIVE_LOW, /* device default: Active Low */
        .uic_notify   = max77696_uic_notify_def_cb,
    },
#endif /* CONFIG_CHARGER_MAX77696_UIC */
#ifdef CONFIG_CHARGER_MAX77696
    .chg_pdata = {
#ifdef CONFIG_BATTERY_MAX77696
        .batteries       = max77696_batteries,
        .num_batteries   = ARRAY_SIZE(max77696_batteries),
#endif /* CONFIG_BATTERY_MAX77696 */
        .wakeup_irq      = 1,
        .t1_C            =  0,                           /* device default:  0 */
        .t2_C            = 10,                           /* device default: 10 */
        .t3_C            = 50,                           /* device default: 50 */
        .t4_C            = 60,                           /* device default: 60 */
        .wdt_period_ms   = 60000,                        /* 60 seconds */
        .initial_mode    = MAX77696_CHARGER_MODE_CHG,    /* device default: MODE_CHG */
        .vsys2set_mV     = 5050,                         /* device default: 5050 */
        .rstrt           = MAX77696_CHARGER_RSTRT_150MV, /* device default: RSTRT_150MV */
        .fchgtime_min    = 600,                          /* device default: 600 */
        .cc_uA           = 466000,                       /* device default: 466000 */
        .to_time_min     = 70,                           /* device default: 70 */
        .to_ith_uA       = 125000,                       /* device default: 125000 */
        .cv_prm_mV       = 4200,                         /* device default: 4200 */
        .cv_jta_mV       = 4200,                         /* device default: 4200 */
        .pqen            = 1,                            /* device default: 1 */
        .minvsys1_mV     = 3000,                         /* device default: 3000 */
        .regtemp_C       = 100,                          /* device default: 100 */
        .charger_notify  = max77696_charger_notify_def_cb,
    },
#endif /* CONFIG_CHARGER_MAX77696 */
#ifdef CONFIG_CHARGER_MAX77696_EH
    .eh_pdata = {
#ifdef CONFIG_BATTERY_MAX77696
        .batteries              = max77696_batteries,
        .num_batteries          = ARRAY_SIZE(max77696_batteries),
#endif /* CONFIG_BATTERY_MAX77696 */
        .initial_mode           = MAX77696_EH_MODE_AUTODETECT,
        .detect_interval_ms     = 1000, /* in msecs */
        .acc_det_gpio           = CONFIG_MFD_MAX77696_GPIO_BASE+0,
        .acc_det_gpio_assert    = 0,    /* active low */
        .acc_det_debounce_ms    = 100,  /* in msec */
        .acc_ilimit             = 400,  /* in mA */
        .charger_notify         = max77696_eh_notify_def_cb,
        .accessory_notify       = NULL,
    },
#endif /* CONFIG_CHARGER_MAX77696_EH */
#ifdef CONFIG_INPUT_MAX77696_ONKEY
    .onkey_pdata = {
        .wakeup_1sec_delayed_since_onkey_down = 0,  /* See EN0DLY in GLBLCNFG1 */
        .wakeup_after_mrwrn                   = 0,
        .wakeup_after_mro                     = 1,  /* See MROWK in GLBLCNFG2 */
        .manual_reset_time                    = 2, /* in seconds */
        .onkey_keycode                        = KEY_POWER,
        .hold_1sec_keycode                    = KEY_POWER,
        .mr_warn_keycode                      = KEY_POWER,
    },
#endif /* CONFIG_INPUT_MAX77696_ONKEY */
};

static __initdata struct i2c_board_info max77696_i2c_board_info = {
    I2C_BOARD_INFO(MAX77696_NAME, MAX77696_I2C_ADDR),
    .platform_data = &max77696_pdata,
};

static __init int board_init_max77696 (void)
{
    return i2c_register_board_info(CONFIG_MFD_MAX77696_I2C_BUS,
        &max77696_i2c_board_info, 1);
}
arch_initcall(board_init_max77696);
