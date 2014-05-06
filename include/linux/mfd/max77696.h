/*
 * MAX77696 MFD Driver Header File
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

#ifndef __MAX77696_H__
#define __MAX77696_H__

#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/power_supply.h>
#include <linux/regulator/machine.h>

#define MAX77696_DRIVER_VERSION            "1.3"

#define MAX77696_I2C_ADDR                  (0x78>>1)

#define MAX77696_NAME                      "max77696"
#define MAX77696_CORE_NAME                 MAX77696_NAME"-core"
#define MAX77696_IRQ_NAME                  MAX77696_NAME"-irq"
#define MAX77696_TOPSYS_NAME               MAX77696_NAME"-topsys"
#define MAX77696_GPIO_NAME                 MAX77696_NAME"-gpio"
#define MAX77696_32K_NAME                  MAX77696_NAME"-32k"
#define MAX77696_WDT_NAME                  MAX77696_NAME"-wdt"
#define MAX77696_RTC_NAME                  MAX77696_NAME"-rtc"
#define MAX77696_BUCK_NAME                 MAX77696_NAME"-buck"
#define MAX77696_LDO_NAME                  MAX77696_NAME"-ldo"
#define MAX77696_LSW_NAME                  MAX77696_NAME"-lsw"
#define MAX77696_EPD_NAME                  MAX77696_NAME"-epd"
#define MAX77696_VDDQ_NAME                 MAX77696_NAME"-vddq"
#define MAX77696_LEDS_NAME                 MAX77696_NAME"-leds"
#define MAX77696_BL_NAME                   MAX77696_NAME"-bl"
#define MAX77696_ADC_NAME                  MAX77696_NAME"-adc"
#define MAX77696_UIC_NAME                  MAX77696_NAME"-uic"
#define MAX77696_GAUGE_NAME                MAX77696_NAME"-battery"
#define MAX77696_CHARGER_NAME              MAX77696_NAME"-charger"
#define MAX77696_EH_NAME                   MAX77696_NAME"-eh"
#define MAX77696_ONKEY_NAME                MAX77696_NAME"-onkey"

/* MAX77696 Module ID */
#define MAX77696_CORE_MID                  0
#define MAX77696_IRQ_MID                   MAX77696_CORE_MID
#define MAX77696_TOPSYS_MID                MAX77696_CORE_MID
#define MAX77696_GPIO_MID                  MAX77696_CORE_MID
#define MAX77696_32K_MID                   MAX77696_CORE_MID
#define MAX77696_WDT_MID                   MAX77696_CORE_MID
#define MAX77696_RTC_MID                   1
#define MAX77696_BUCK_MID                  MAX77696_CORE_MID
#define MAX77696_LDO_MID                   MAX77696_CORE_MID
#define MAX77696_LSW_MID                   MAX77696_CORE_MID
#define MAX77696_EPD_MID                   MAX77696_CORE_MID
#define MAX77696_VDDQ_MID                  MAX77696_CORE_MID
#define MAX77696_LEDS_MID                  MAX77696_CORE_MID
#define MAX77696_BL_MID                    MAX77696_CORE_MID
#define MAX77696_ADC_MID                   MAX77696_CORE_MID
#define MAX77696_UIC_MID                   2
#define MAX77696_GAUGE_MID                 3
#define MAX77696_CHARGER_MID               MAX77696_CORE_MID
#define MAX77696_EH_MID                    MAX77696_RTC_MID
#define MAX77696_ONKEY_MID                 MAX77696_CORE_MID

/* Names of power supplies
 */

/* Fuel Gauge */
#define MAX77696_PSY_BATT_NAME             MAX77696_GAUGE_NAME
/* Main Battery Charger */
#define MAX77696_PSY_CHG_NAME              MAX77696_CHARGER_NAME
/* Energy Harvester */
#define MAX77696_PSY_EH_NAME               MAX77696_EH_NAME

/* Number of interrupts to be reserved for MAX77696 */
#define MAX77696_GPIO_IRQ_OFFSET           \
        (MAX77696_ROOTINT_NR_IRQS + MAX77696_TOPSYSINT_NR_IRQS)
#define MAX77696_NR_IRQS                   \
        (MAX77696_GPIO_IRQ_OFFSET + MAX77696_GPIO_NR_GPIOS)

/* 32kHz OSC load capacitance selection */
#define MAX77696_32K_LOAD_CAP_22PF         0 /* 22pF per node */
#define MAX77696_32K_LOAD_CAP_12PF         1 /* 12pF per node */
#define MAX77696_32K_LOAD_CAP_10PF         2 /* 10pF per node */
#define MAX77696_32K_LOAD_CAP_NONE         3 /* No internal load cap selected */

/* 32kHz OSC operation mode
 *   Whenever the main battery is plugged in and VSYS > VSYSMIN, and the
 *   MODE_SEL bit is set to 1, the crystal driver goes into a high current, high
 *   accuracy state that meets the 15ns cycle by cycle jitter and 45%. 55% duty
 *   cycle spec. As soon as VSYS < VSYSMIN, the oscillator automatically goes
 *   into a low power state where it does not meet the jitter and duty cycle
 *   specification. The assumption is that when VSYS < VSYSMIN, the radio is
 *   off, and tight jitter and duty cycle spec is not required. This prevents
 *   the battery from being too deeply discharge, since the supply current is
 *   reduced.
 */
#define MAX77696_32K_MODE_LOW_POWER        0 /* Low Power */
#define MAX77696_32K_MODE_LOW_JITTER       1 /* Low Jitter */

struct max77696_32k_platform_data {
    int load_cap:2; /* 32kHz OSC load capacitance selection */
    int op_mode:1;  /* 32kHz OSC operation mode             */
};

/* Number of GPIOs */
#define MAX77696_GPIO_NR_GPIOS             5

/* GPIO alternate mode */
#define MAX77696_GPIO_AME_STDGPIO          0 /* Standard GPI or GPO */
#define MAX77696_GPIO_AME_MODE_0           1
#define MAX77696_GPIO_AME_MODE_1           2
#define MAX77696_GPIO_AME_MODE_2           3

/* GPIO direction */
#define MAX77696_GPIO_DIR_OUTPUT           0
#define MAX77696_GPIO_DIR_INPUT            1

/* GPIO output configuration */
#define MAX77696_GPIO_OUTCFG_OPENDRAIN     0
#define MAX77696_GPIO_OUTCFG_PUSHPULL      1

/* Debounce Configuration */
#define MAX77696_GPIO_DBNC_0_MSEC          0
#define MAX77696_GPIO_DBNC_8_MSEC          1
#define MAX77696_GPIO_DBNC_16_MSEC         2
#define MAX77696_GPIO_DBNC_32_MSEC         3

/* Rising edge and falling edge interrupt configuration */
#define MAX77696_GPIO_REFEIRQ_NONE         0x0
#define MAX77696_GPIO_REFEIRQ_FALLING      0x1
#define MAX77696_GPIO_REFEIRQ_RISING       0x2
#define MAX77696_GPIO_REFEIRQ_BOTH         \
        (MAX77696_GPIO_REFEIRQ_FALLING|MAX77696_GPIO_REFEIRQ_RISING)

struct max77696_gpio_init_data {
    int pin_connected:1;
    int alter_mode:2;    /* GPIO Alternate Mode Enable (see datasheet for details) */
    int pullup_en:1;     /* GPIO Pull-Up Enable */
    int pulldn_en:1;     /* GPIO Pull-Down Enable */
    int direction:1;
    union {
        struct {
            int out_cfg:1;  /* Output configuration */
            int drive:1;    /* Output drive level (0:LOW / 1:HIGH) */
        } output;
        struct {
            int debounce:2; /* Debounce configuration */
            int refe_irq:2; /* Rising and falling edge interrupt configuration */
        } input;
    } u;
};

struct max77696_gpio_platform_data {
    int                            irq_base;
    int                            gpio_base;
    int                            bias_en;  /* GPIO Biasing Circuitry Enable */
    struct max77696_gpio_init_data init_data[MAX77696_GPIO_NR_GPIOS];
};

struct max77696_watchdog_platform_data {
    unsigned int timeout_sec;
    unsigned int ping_interval_sec; /* If watchdog daemon is used,
                                      * then set this zero.
                                      */
};

struct max77696_rtc_platform_data {
    bool irq_1m; /* RTC 1 minute interrupt */
    bool irq_1s; /* RTC 1 second interrupt */
};

/* BUCK_ Rising Slew Rate Selection */
#define MAX77696_BUCK_RISING_SLEW_RATE_12p5_MV_PER_USEC  0b00
#define MAX77696_BUCK_RISING_SLEW_RATE_25_MV_PER_USEC    0b01
#define MAX77696_BUCK_RISING_SLEW_RATE_50_MV_PER_USEC    0b10
#define MAX77696_BUCK_RISING_SLEW_RATE_NO_CONTROL        0b11

/* BUCK_ Operating Mode Selection */
#define MAX77696_BUCK_OPERATING_MODE_OFF                 0b00
                              /* Disabled. BUCK_ is OFF. */
#define MAX77696_BUCK_OPERATING_MODE_DYNAMIC_STANDBY     0b01
                              /* Dynamic Standby Mode. BUCK_ operates in Normal
                               * Mode when the Standby Bus is Low and operates
                               * in Standby Mode when the Global Low Power Mode
                               * Bus is High.
                               */
#define MAX77696_BUCK_OPERATING_MODE_FORCED_STANDBY      0b10
                              /* Forced Standby Mode. BUCK_ operates in Standby
                               * Mode regardless of the state of the Global Low
                               * Power Mode Bus.
                               */
#define MAX77696_BUCK_OPERATING_MODE_NORMAL              0b11
                              /* Normal Operation. BUCK_ operates in Normal Mode
                               * regardless of the state of the Global Low Power
                               * Mode Bus.
                               */

/* Number of Buck Regulators */
#define MAX77696_BUCK_NR_REGS          MAX77696_BUCK_ID_MAX

enum {
    MAX77696_BUCK_ID_B1 = 0,
    MAX77696_BUCK_ID_B1DVS,
    MAX77696_BUCK_ID_B2,
    MAX77696_BUCK_ID_B2DVS,
    MAX77696_BUCK_ID_B3,
    MAX77696_BUCK_ID_B4,
    MAX77696_BUCK_ID_B5,
    MAX77696_BUCK_ID_B6,
    MAX77696_BUCK_ID_MAX,
};

struct max77696_buck_platform_data {
    bool                       support_suspend_ops;
    struct regulator_init_data init_data[MAX77696_BUCK_NR_REGS];
};

/* LDOx Power Mode */
#define MAX77696_LDO_POWERMODE_OFF                0b00
                              /* Output Disabled. OUTLDOxx is off. */
#define MAX77696_LDO_POWERMODE_LOW_POWER          0b01
                              /* Group Low-Power Mode. OUTLDOxx operates in
                               * normal mode when the global low-power mode
                               * signal is low. When the global low-power mode
                               * signal is high, OUTLDOxx operates in
                               * low-power mode.
                               */
#define MAX77696_LDO_POWERMODE_FORCED_LOW_POWER   0b10
                              /* Low-Power Mode. OUTLDOxx is forced into
                               * low-power mode. The maximum load current is
                               * 5mA and the quiescent supply current is 1.5uA.
                               */
#define MAX77696_LDO_POWERMODE_NORMAL             0b11
                              /* Normal Mode. OUTLDOxx is forced into its normal
                               * operating mode.
                               */

/* LDOx Transconductance Setting */
#define MAX77696_LDO_TRANSCONDUCTANCE_FAST        0b00
#define MAX77696_LDO_TRANSCONDUCTANCE_MEDIUM_FAST 0b01
#define MAX77696_LDO_TRANSCONDUCTANCE_MEDIUM_SLOW 0b10
#define MAX77696_LDO_TRANSCONDUCTANCE_SLOW        0b11

/* LDOx Soft-Start Slew Rate Configuration
 * (Applies to both start-up and output voltage setting changes)
 */
#define MAX77696_LDO_SLEW_RATE_FAST               0 /* 100mV / usec */
#define MAX77696_LDO_SLEW_RATE_SLOW               1 /*   5mV / usec */

/* LDO Current Monitor Operating Mode */
#define MAX77696_LDO_OPERATING_MODE_AUTONOMOUS    0
#define MAX77696_LDO_OPERATING_MODE_MANUAL_OFF    1
#define MAX77696_LDO_OPERATING_MODE_MANUAL_ON     2

/* Number of LDO Regulators */
#define MAX77696_LDO_NR_REGS           MAX77696_LDO_ID_MAX

enum {
    MAX77696_LDO_ID_L1 = 0,
    MAX77696_LDO_ID_L2,
    MAX77696_LDO_ID_L3,
    MAX77696_LDO_ID_L4,
    MAX77696_LDO_ID_L5,
    MAX77696_LDO_ID_L6,
    MAX77696_LDO_ID_L7,
    MAX77696_LDO_ID_L8,
    MAX77696_LDO_ID_L9,
    MAX77696_LDO_ID_L10,
    MAX77696_LDO_ID_MAX,
};

/* LDO Current Monitor Transfer Function. L_IMON_TF adjusts the monitor's
 * transfer function. This adjustment capability is very similar to the "range"
 * function on a digital multi-meter (DMM).
 */
#define MAX77696_LDO_IMON_TF_8000_OHM 0b000
#define MAX77696_LDO_IMON_TF_4000_OHM 0b001
#define MAX77696_LDO_IMON_TF_2000_OHM 0b010
#define MAX77696_LDO_IMON_TF_1000_OHM 0b011 /* device default */
#define MAX77696_LDO_IMON_TF_500_OHM  0b100
#define MAX77696_LDO_IMON_TF_250_OHM  0b101
#define MAX77696_LDO_IMON_TF_125_OHM  0b110
#define MAX77696_LDO_IMON_TF_62p5_OHM 0b111

struct max77696_ldo_platform_data {
    u8                         imon_tf; /* Current Monitor Transfer Function */
    struct regulator_init_data init_data[MAX77696_LDO_NR_REGS];
};

/* Load Switches */
#define MAX77696_LSW_NR_REGS           MAX77696_LSW_ID_MAX

enum {
    MAX77696_LSW_ID_LSW1 = 0,
    MAX77696_LSW_ID_LSW2,
    MAX77696_LSW_ID_LSW3,
    MAX77696_LSW_ID_LSW4,
    MAX77696_LSW_ID_MAX,
};

struct max77696_lsw_platform_data {
    struct regulator_init_data init_data[MAX77696_LSW_NR_REGS];
};

/* EPD Power Supplies */
#define MAX77696_EPD_NR_REGS           MAX77696_EPD_ID_MAX

enum {
    MAX77696_EPD_ID_DISP = 0,
    MAX77696_EPD_ID_VCOM,
    MAX77696_EPD_ID_VEE,
    MAX77696_EPD_ID_VNEG,
    MAX77696_EPD_ID_VPOS,
    MAX77696_EPD_ID_VDDH,
    MAX77696_EPD_ID_MAX,
};

struct max77696_epd_platform_data {
    unsigned int               pok_wait_timeout_ms;

    /* Flag for control of VCOMEN bit in EPDCNFG register:
     *   setting VCOMEN bit will enable VCOM (ORed with VCEN external pin).
     */
    bool                       control_vcom_en;

    struct regulator_init_data init_data[MAX77696_EPD_NR_REGS];
};

/* LPDDR2 Termination Supply */
struct max77696_vddq_platform_data {
    int                        vddq_in_uV;
    struct regulator_init_data init_data;
};

enum {
    MAX77696_LED_GREEN = 0,
    MAX77696_LED_AMBER,

    MAX77696_LED_NR_LEDS,
};

struct max77696_led_platform_data {
    struct led_info info;
    bool            manual_mode;
};

struct max77696_backlight_platform_data {
};

/* Analog-to-Digital Converter */
#define MAX77696_ADC_NR_CHS  MAX77696_ADC_CH_MAX

enum {
    MAX77696_ADC_CH_VSYS2 = 0,
    MAX77696_ADC_CH_TDIE,
    MAX77696_ADC_CH_VSYS1,
    MAX77696_ADC_CH_VCHGINA,
    MAX77696_ADC_CH_ICHGINA,
    MAX77696_ADC_CH_IMONL1,
    MAX77696_ADC_CH_IMONL2,
    MAX77696_ADC_CH_IMONL3,
    MAX77696_ADC_CH_IMONL4,
    MAX77696_ADC_CH_IMONL5,
    MAX77696_ADC_CH_IMONL6,
    MAX77696_ADC_CH_IMONL7,
    MAX77696_ADC_CH_IMONL8,
    MAX77696_ADC_CH_IMONL9,
    MAX77696_ADC_CH_IMONL10,
    MAX77696_ADC_CH_IMONB1,
    MAX77696_ADC_CH_IMONB2,
    MAX77696_ADC_CH_IMONB3,
    MAX77696_ADC_CH_IMONB4,
    MAX77696_ADC_CH_IMONB5,
    MAX77696_ADC_CH_IMONB6,
    MAX77696_ADC_CH_AIN0,
    MAX77696_ADC_CH_AIN1,
    MAX77696_ADC_CH_AIN2,
    MAX77696_ADC_CH_AIN3,
    MAX77696_ADC_CH_MAX,
};

#define MAX77696_ADC_PRINT_RAW    0b0001 /* integer */
#define MAX77696_ADC_PRINT_SCALED 0b0010 /* integer (milli-scaled) */
#define MAX77696_ADC_PRINT_FULL   0b0100 /* double + unit string */
#define MAX77696_ADC_PRINT_ALL    0xFF

struct max77696_adc_platform_data {
    u8  print_fmt; /* cf. MAX77696_ADC_PRINT_... */
    u8  avg_rate;  /* average rate: 1, 2, 16, 32 samples */
    u16 adc_delay; /* up to 16500 nano-seconds */
};

#define MAX77696_GAUGE_POLLING_CAPACITY    BIT(15)
#define MAX77696_GAUGE_POLLING_TEMP        BIT(14)
#define MAX77696_GAUGE_POLLING_VOLTAGE_NOW BIT(13)
#define MAX77696_GAUGE_POLLING_VOLTAGE_AVG BIT(12)
#define MAX77696_GAUGE_POLLING_CURRENT_NOW BIT(11)
#define MAX77696_GAUGE_POLLING_CURRENT_AVG BIT(10)
#define MAX77696_GAUGE_POLLING_CHARGE_FULL BIT( 9)
#define MAX77696_GAUGE_POLLING_CHARGE_NOW  BIT( 8)

/* Gauge's capacity paramters to be saved & loaded */
enum {
    MAX77696_GAUGE_CAP_PARAM_RCOMP0 = 0,
    MAX77696_GAUGE_CAP_PARAM_TEMPCO,
    MAX77696_GAUGE_CAP_PARAM_FULLCAP,
    MAX77696_GAUGE_CAP_PARAM_CYCLES,
    MAX77696_GAUGE_CAP_PARAM_FULLCAPNOM,
    MAX77696_GAUGE_CAP_PARAM_IAVG_EMPTY,
    MAX77696_GAUGE_CAP_PARAM_QRESIDUAL00,
    MAX77696_GAUGE_CAP_PARAM_QRESIDUAL10,
    MAX77696_GAUGE_CAP_PARAM_QRESIDUAL20,
    MAX77696_GAUGE_CAP_PARAM_QRESIDUAL30,

    MAX77696_GAUGE_CAP_PARAM_NR_INFOS,
};

#define MAX77696_GAUGE_CHARACTERIZATION_TABLE_SIZE (3*16)

struct max77696_gauge_config {
    u16   config;
    u16   filter_cfg;
    u16   learn_cfg;
    u16   full_soc_thresh;
    u16   rcomp0;
    u16   tempco;
    u16   ichg_term;
    u16   vempty;
    u16   qrtbl00;
    u16   qrtbl10;
    u16   qrtbl20;
    u16   qrtbl30;
    u16   cycles;
    u16   fullcap;
    u16   design_cap;
    u16   fullcapnom;
    u16   batt_cap;
    u16   cell_char_tbl[MAX77696_GAUGE_CHARACTERIZATION_TABLE_SIZE];
};

struct max77696_gauge_led_control {
    bool                manual_mode;
    enum led_brightness brightness;
    bool                flashing;
    struct {
        unsigned long duration_ms;
        unsigned long period_ms;
    }                   flash_params;
};

struct max77696_gauge_led_trigger {
    bool                              enable;
    struct max77696_gauge_led_control control[MAX77696_LED_NR_LEDS];
};

struct max77696_gauge_platform_data {
    struct max77696_gauge_config      *config;
    bool                               enable_por_init;

    /* The Alert Threshold values
     *   - Over-/undervoltage       VALRT threshold violation (upper or lower)
     *   - Over-/undertemperature   TALRT threshold violation (upper or lower)
     *   - Over/under SOC           SALRT threshold violation (upper or lower)
     * (the alert will be disabled if max and min are same)
     */
    int                                v_alert_max, v_alert_min; /* 5100 ~    0 [mV] */
    int                                t_alert_max, t_alert_min; /*  127 ~ -128 [C] */
    int                                s_alert_max, s_alert_min; /*  255 ~    0 [%] */
    bool                               enable_alert_on_battery_removal;
    bool                               enable_alert_on_battery_insertion;

    int                                charge_full_design;    /* in uAh */
    int                                battery_full_capacity; /* in percent */

    /*
     * R_sns in micro-ohms.
     * default 10000 (if r_sns = 0) as it is the recommended value by
     * the datasheet although it can be changed by board designers.
     */
    int                                r_sns;

    unsigned int                       update_interval_ms; /* in msec */
    /* ORed MAX77696_GAUGE_POLL_... */
    u16                                polling_properties;

    bool                               support_led_triggers;
    struct max77696_gauge_led_trigger  default_trigger;
    struct max77696_gauge_led_trigger  charging_trigger;
    struct max77696_gauge_led_trigger  charging_full_trigger;

    bool                               (*battery_online) (void);
    bool                               (*charger_online) (void);
    bool                               (*charger_enable) (void);
};

/* UIC Interrupt Type */
#define MAX77696_UIC_INTTYP_LEVEL            0 /* Interrupt is level triggered */
#define MAX77696_UIC_INTTYP_EDGE             1 /* Interrupt is edge triggered */

/* UIC Interrupt Delay */
#define MAX77696_UIC_INTDLY_2TICKS           0 /* 2 x 60KHz-clock ticks */
#define MAX77696_UIC_INTDLY_4TICKS           1 /* 4 x 60KHz-clock ticks */

/* UIC Interrupt Polarity */
#define MAX77696_UIC_INTPOL_ACTIVE_LOW       0
#define MAX77696_UIC_INTPOL_ACTIVE_HIGH      1

/* USB Charger Detection */
#define MAX77696_UIC_CHGTYPE_USB             0b0001 /* USB Cable */
#define MAX77696_UIC_CHGTYPE_CDP             0b0010 /* Charging Downstream Port */
#define MAX77696_UIC_CHGTYPE_DEDICATED_1P5A  0b0011 /* Dedicated Charger: current 1500mA max */
#define MAX77696_UIC_CHGTYPE_APPLE_0P5AC     0b0100 /* Apple Charger: current  500mA max */
#define MAX77696_UIC_CHGTYPE_APPLE_1P0AC     0b0101 /* Apple Charger: current 1500mA max */
#define MAX77696_UIC_CHGTYPE_APPLE_2P0AC     0b0110 /* Apple Charger: current 2000mA max */
#define MAX77696_UIC_CHGTYPE_OTH_0           0b0111 /* Other Charger */
#define MAX77696_UIC_CHGTYPE_SELFENUM_0P5AC  0b1001 /* Self Enumerated: current 500mA max */
#define MAX77696_UIC_CHGTYPE_OTH_1           0b1100 /* Other Charger */

struct max77696_uic_notify {
    u8 vb_volt;
    u8 chg_type;
    u8 adc_code;
    u8 i_set;
};

struct max77696_uic_platform_data {
    u8   int_type;
    u8   int_delay; /* valid only if int_type = 1 */
    u8   int_polarity;
    void (*uic_notify) (const struct max77696_uic_notify *noti);
};

#define MAX77696_CHARGER_MODE_OFF      0b0000
/* When the DC to DC is set for buck converter
 * operation, setting this bit will turn on the charger. When the DC to DC is
 * set for boost converter operation, setting this bit will turn on the SYS2 to
 * CHGINA switch for OTG style operation
*/
#define MAX77696_CHARGER_MODE_CHG      0b0101
#define MAX77696_CHARGER_MODE_OTG      0b1010
/* Enable the DC to DC converter as a buck converter */
#define MAX77696_CHARGER_MODE_ENBUCK   0b0100
/* Enable the DC to DC converter as a boost converter */
#define MAX77696_CHARGER_MODE_ENBOOST  0b1000

/* Charger Restart Threshold */
#define MAX77696_CHARGER_RSTRT_100MV   0b00 /* 100mV below the value programmed
                                             *       by CHG_CV_PRM[4:0] */
#define MAX77696_CHARGER_RSTRT_150MV   0b01 /* 150mV below the value programmed
                                             *       by CHG_CV_PRM[4:0] */
#define MAX77696_CHARGER_RSTRT_200MV   0b10 /* 200mV below the value programmed
                                             *       by CHG_CV_PRM[4:0] */
#define MAX77696_CHARGER_RSTRT_DISABLE 0b11 /* Disabled */

struct max77696_charger_platform_data {
    char **batteries;
    int    num_batteries;

    bool   wakeup_irq;
    int    t1_C, t2_C, t3_C, t4_C;

    int    wdt_period_ms; /* Watchdog Timer Period:   80,000 msec (fixed)
                           * Watchdog Timer Accuracy: -20 ~ +20 %
                           * (To disable wdt, set zero)
                           */

    int    initial_mode;  /* cf. MAX77696_CHARGER_MODE_... */
    int    vsys2set_mV;   /* SYS2 Target Output Voltage in Boost Mode
                           * (3,000 ~ 5,750 mV)
                           */

    /* Considering the characteristics of the battery */
    int    rstrt;         /* cf. MAX77696_CHARGER_RSTRT_... */
    int    fchgtime_min;  /* Fast-Charge Timer Duration
                           * (disable(0) ~ 960 minutes)
                           */
    int    cc_uA;         /* Fast-Charge Current (66,700 ~ 2,100,000 uA) */
    int    to_time_min;   /* Top-Off Timer Setting (0 ~ 70 minutes) */
    int    to_ith_uA;     /* Top-Off Current Threshold (50,000 ~ 125,000 uA) */
    int    cv_prm_mV;     /* Battery Regulation Voltage (3,650 ~ 4,400 mV) */
    int    cv_jta_mV;     /* Battery Regulation Voltage if JEITA = 1
                           * (3,650 ~ 4,400 mV)
                           */

    /* Considering the characteristics of the system: */
    bool   pqen;          /* Low-Battery Prequalification Enable */
    int    minvsys1_mV;   /* Minimum System Regulation Voltage
                           * (3,000 ~ 3,700 mV)
                           */
    int    regtemp_C;     /* Junction Temperature Thermal Regulation Loop
                           * Setpoint (70 ~ 115 degree C)
                           */

    void   (*charger_notify) (struct power_supply *psy,
        bool online, bool enable);
};

#define MAX77696_EH_MODE_SHUTDOWN   0b00
#define MAX77696_EH_MODE_CHARGER    0b01
#define MAX77696_EH_MODE_ACCESSORY  0b10
#define MAX77696_EH_MODE_AUTODETECT 0b11

struct max77696_eh_platform_data {
    char         **batteries;
    int            num_batteries;

    u8             initial_mode;
    unsigned int   detect_interval_ms;

    unsigned int   acc_det_gpio;
    int            acc_det_gpio_assert; /* HIGH(1) or LOW (0) */
    unsigned int   acc_det_debounce_ms;
    int            acc_ilimit;

    void           (*charger_notify) (struct power_supply *psy,
        bool online, bool enable);
    void           (*accessory_notify) (struct power_supply *psy,
        bool present);
};

struct max77696_onkey_platform_data {
    bool         wakeup_1sec_delayed_since_onkey_down;
    bool         wakeup_after_mro;   /* wakeup after manual reset
                                      * 0 = a manual reset results in a power
                                      *     down to the shutdown state
                                      * 1 = a manual reset results in a power
                                      * cycle and the AP active state
                                      */
    int          manual_reset_time;  /* 2s ... 12s */
    bool         wakeup_after_mrwrn; /* wakeup after manual reset warning */

    unsigned int onkey_keycode;      /* Keycode for DBEN0 */
    unsigned int hold_1sec_keycode;  /* Keycode for EN0_1SEC */
    unsigned int mr_warn_keycode;    /* Keycode for MRWRN */
};

/*** MAX77696 PLATFORM DATA STRUCTURES ***/
struct max77696_platform_data {
    unsigned int                            core_irq;
    unsigned long                           core_irq_trigger;

    unsigned int                            irq_base;

    /* chip options */
    bool                                    core_supports_debugging_stuff;
    unsigned int                            core_mbattlow_falling_threshold_mV;
    unsigned int                            core_mbattlow_comparator_hysteresis_mV;

    struct max77696_32k_platform_data       osc_pdata;
    struct max77696_gpio_platform_data      gpio_pdata;
    struct max77696_watchdog_platform_data  wdt_pdata;
    struct max77696_rtc_platform_data       rtc_pdata;
    struct max77696_buck_platform_data      buck_pdata;
    struct max77696_ldo_platform_data       ldo_pdata;
    struct max77696_lsw_platform_data       lsw_pdata;
    struct max77696_epd_platform_data       epd_pdata;
    struct max77696_vddq_platform_data      vddq_pdata;
    struct max77696_led_platform_data       led_pdata[MAX77696_LED_NR_LEDS];
    struct max77696_backlight_platform_data bl_pdata;
    struct max77696_adc_platform_data       adc_pdata;
    struct max77696_gauge_platform_data     gauge_pdata;
    struct max77696_uic_platform_data       uic_pdata;
    struct max77696_charger_platform_data   chg_pdata;
    struct max77696_eh_platform_data        eh_pdata;
    struct max77696_onkey_platform_data     onkey_pdata;
};

struct max77696_i2c {
    struct i2c_client *client;

    int (*read) (struct max77696_i2c *me, u8 addr, u8 *val);
    int (*write) (struct max77696_i2c *me, u8 addr, u8 val);
    int (*bulk_read) (struct max77696_i2c *me, u8 addr, u8 *dst, u16 len);
    int (*bulk_write) (struct max77696_i2c *me, u8 addr, const u8 *src, u16 len);
};

struct max77696_chip {
    struct device                 *dev;
    struct kobject                *kobj;

    struct max77696_i2c            pmic_i2c;
    struct max77696_i2c            rtc_i2c;
    struct max77696_i2c            uic_i2c;
    struct max77696_i2c            gauge_i2c;

    unsigned int                   core_irq;
    unsigned int                   irq_base;

    void                          *irq_ptr;
    void                          *topsys_ptr;
    void                          *buck_ptr;
    void                          *ldo_ptr;
    void                          *led_ptr[MAX77696_LED_NR_LEDS];
    void                          *adc_ptr;
    void                          *gauge_ptr;
    void                          *chg_ptr;
    void                          *eh_ptr;
};

#define max77696_rootint_irq_base(chip) \
        ((chip)->irq_base)
#define max77696_rootint_to_irq(chip, rootint) \
        (max77696_rootint_irq_base(chip) + (MAX77696_ROOTINT_##rootint))

#define max77696_topsysint_irq_base(chip) \
        (max77696_rootint_to_irq(chip,NR_IRQS))
#define max77696_topsysint_to_irq(chip, topsysint) \
        (max77696_topsysint_irq_base(chip) + (MAX77696_TOPSYSINT_##topsysint))

/* ### EXTERNAL APIS ### */

#define pmic_read   max77696_chip_read
#define pmic_write  max77696_chip_write

/* @CORE */
extern int max77696_chip_read (u8 module, u8 addr, u16 *data);
extern int max77696_chip_write (u8 module, u8 addr, u16 data);
extern int max77696_chip_set_wakeup (struct device *dev, bool enable);
extern bool max77696_battery_online_def_cb (void);
extern bool max77696_charger_online_def_cb (void);
extern bool max77696_charger_enable_def_cb (void);
extern void max77696_uic_notify_def_cb (const struct max77696_uic_notify *noti);
extern void max77696_charger_notify_def_cb (struct power_supply *psy,
    bool online, bool enable);
extern void max77696_eh_notify_def_cb (struct power_supply *psy,
    bool online, bool enable);

/* @TOPSYS */
extern int max77696_topsys_set_global_lp_mode (bool level);
extern int max77696_topsys_enable_mr (bool enable);
extern int max77696_topsys_set_mr_time (unsigned int seconds);
extern int max77696_topsys_enable_en0_delay (bool enable);
extern int max77696_topsys_enable_standy (bool enable);
extern int max77696_topsys_enable_mr_wakeup (bool enable);
extern int max77696_topsys_enable_wdt (bool enable);
extern int max77696_topsys_set_wdt_period (unsigned int seconds);
extern int max77696_topsys_enable_rtc_wakeup (bool enable);
extern int max77696_topsys_enable_wdt_wakeup (bool enable);
extern int max77696_topsys_enable_uic_edge_wakeup (bool enable);
extern int max77696_topsys_set_lbhyst (unsigned int mV);
extern int max77696_topsys_set_lbdac (unsigned int mV);
extern int max77696_topsys_clear_wdt (void);
extern int max77696_topsys_set_rso_delay (unsigned int time_us);

/* @IRQ */
extern bool max77696_irq_test_ercflag (u16 flag);
extern void max77696_irq_enable_fgirq (u8 irq_bits, bool forced);
extern void max77696_irq_disable_fgirq (u8 irq_bits, bool forced);
extern int max77696_irq_read_fgirq_status (u8 *status);

/* @BUCK */
extern int max77696_buck_get_rising_slew_rate (u8 buck);
extern int max77696_buck_set_rising_slew_rate (u8 buck, u8 slew_rate);
extern int max77696_buck_get_operating_mode (u8 buck);
extern int max77696_buck_set_operating_mode (u8 buck, u8 mode);
extern int max77696_buck_get_active_discharge_enable (u8 buck);
extern int max77696_buck_set_active_discharge_enable (u8 buck, bool enable);
extern int max77696_buck_get_fpwm_enable (u8 buck);
extern int max77696_buck_set_fpwm_enable (u8 buck, bool enable);
extern int max77696_buck_get_imon_enable (u8 buck);
extern int max77696_buck_set_imon_enable (u8 buck, bool enable);
extern int max77696_buck_get_falling_slew_rate_enable (u8 buck);
extern int max77696_buck_set_falling_slew_rate_enable (u8 buck, bool enable);

/* @LDO */
extern int max77696_ldo_get_power_mode (u8 ldo);
extern int max77696_ldo_set_power_mode (u8 ldo, u8 mode);
extern int max77696_ldo_get_target_voltage (u8 ldo);
extern int max77696_ldo_set_target_voltage (u8 ldo, u8 uV);
extern int max77696_ldo_get_overvoltage_clamp_enable (u8 ldo);
extern int max77696_ldo_set_overvoltage_clamp_enable (u8 ldo, bool enable);
extern int max77696_ldo_get_auto_low_power_mode_enable (u8 ldo);
extern int max77696_ldo_set_auto_low_power_mode_enable (u8 ldo, bool enable);
extern int max77696_ldo_get_transconductance (u8 ldo);
extern int max77696_ldo_set_transconductance (u8 ldo, u8 ratio);
extern int max77696_ldo_get_votage_okay_status (u8 ldo);
extern int max77696_ldo_get_imon_enable (u8 ldo);
extern int max77696_ldo_set_imon_enable (u8 ldo, bool enable);
extern int max77696_ldo_get_active_discharge_enable (u8 ldo);
extern int max77696_ldo_set_active_discharge_enable (u8 ldo, bool enable);
extern int max77696_ldo_get_slew_rate (u8 ldo);
extern int max77696_ldo_set_slew_rate (u8 ldo, u8 slew_rate);
extern int max77696_ldo_get_imon_operating_mode (void);
extern int max77696_ldo_set_imon_operating_mode (u8 mode);
extern int max77696_ldo_get_imon_transfer_function (void);
extern int max77696_ldo_set_imon_transfer_function (u8 tf_val);
extern int max77696_ldo_get_sbias_enable (void);
extern int max77696_ldo_set_sbias_enable (bool enable);
extern int max77696_ldo_get_bias_enable (void);
extern int max77696_ldo_set_bias_enable (bool enable);

/* @LED */
extern void max77696_led_update_changes (unsigned int led_id);
extern void max77696_led_enable_manual_mode (unsigned int led_id,
    bool manual_mode);
extern void max77696_led_set_brightness (unsigned int led_id,
    enum led_brightness value);
extern void max77696_led_set_blink (unsigned int led_id,
    unsigned long duration_ms, unsigned long period_ms);
extern void max77696_led_disable_blink (unsigned int led_id);

/* @ADC */
int max77696_adc_read (u8 channel, u16 *raw, s32 *milli_scaled);
#define max77696_adc_read_raw(channel, raw_ptr) \
        max77696_adc_read(channel, raw_ptr, NULL)
#define max77696_adc_read_milli_scaled(channel, milli_scaled_ptr) \
        max77696_adc_read(channel, NULL, milli_scaled_ptr)
#define max77696_adc_read_buck_current(buck, data_ptr) \
        max77696_adc_read_milli_scaled(MAX77696_ADC_CH_IMONB##buck, data_ptr)
#define max77696_adc_read_ldo_current(ldo, data_ptr) \
        max77696_adc_read_milli_scaled(MAX77696_ADC_CH_IMONL##ldo, data_ptr)

/* @GAUGE */
extern int max77696_gauge_write_cap_param (int id, u16 val);
extern int max77696_gauge_read_cap_param (int id, u16 *val);

/* @CHARGER */
extern int max77696_charger_set_mode (int mode);
extern int max77696_charger_get_mode (int *mode);
extern int max77696_charger_set_pq_en (int enable);
extern int max77696_charger_get_pq_en (int *enable);
extern int max77696_charger_set_cc_level (int uA);
extern int max77696_charger_get_cc_level (int *uA);
extern int max77696_charger_set_jeita_en (int enable);
extern int max77696_charger_get_jeita_en (int *enable);
extern int max77696_charger_set_cv_level (bool jeita, int mV);
extern int max77696_charger_get_cv_level (bool jeita, int *mV);
extern int max77696_charger_set_temp_thres (int t_id, int temp_C);
extern int max77696_charger_get_temp_thres (int t_id, int *temp_C);
extern int max77696_charger_set_wdt_period (int period_ms);
extern int max77696_charger_get_wdt_period (int *period_ms);

/* @EH */
extern int max77696_eh_set_mode (int mode);
extern int max77696_eh_get_mode (int *mode);
extern int max77696_eh_set_accessory_i_limit (int mA);
extern int max77696_eh_get_accessory_i_limit (int *mA);
extern int max77696_eh_set_cv_level (int mV);
extern int max77696_eh_get_cv_level (int *mV);
extern int max77696_eh_set_cc_level (bool pqsel, int mA);
extern int max77696_eh_get_cc_level (bool *pqsel, int *mA);

/* ### INTERNAL USE ONLY ### */

/* PMIC root interrupt definitions */
enum {
    MAX77696_ROOTINT_TOPSYS    =  0,
    MAX77696_ROOTINT_BUCK,    /*  1 */
    MAX77696_ROOTINT_FG,      /*  2 */
    MAX77696_ROOTINT_GPIO,    /*  3 */
    MAX77696_ROOTINT_RTC,     /*  4 */
    MAX77696_ROOTINT_CHGA,    /*  5 */
    MAX77696_ROOTINT_LDO,     /*  6 */
    MAX77696_ROOTINT_UIC,     /*  7 */
    MAX77696_ROOTINT_ADC,     /*  8 */
    MAX77696_ROOTINT_WLED,    /*  9 */
    MAX77696_ROOTINT_EPD,     /* 10 */
    MAX77696_ROOTINT_CHGB,    /* 11 */

    MAX77696_ROOTINT_NR_IRQS,
};

/* PMIC topsys interrupt definitions */
enum {
    MAX77696_TOPSYSINT_THERM_ALARM_1   = 0,
    MAX77696_TOPSYSINT_THERM_ALARM_0, /* 1 */
    MAX77696_TOPSYSINT_BATT_LOW,      /* 2 */
    MAX77696_TOPSYSINT_MR_WARNING,    /* 3 */
    MAX77696_TOPSYSINT_EN0_1SEC,      /* 4 */
    MAX77696_TOPSYSINT_EN0_FALLING,   /* 5 */
    MAX77696_TOPSYSINT_EN0_RISING,    /* 6 */

    MAX77696_TOPSYSINT_NR_IRQS,
};

#undef  BITS
#define BITS(_msb, _lsb)                 ((BIT(_msb)-BIT(_lsb))+BIT(_msb))

#define MAX77696_ERCFLAG0_WDPMIC_FSHDN   BIT ( 0) /* PMIC System Watchdog Full Shutdown */
#define MAX77696_ERCFLAG0_WDPMIC_FRSTRT  BIT ( 1) /* PMIC System Watchdog Full Restart */
#define MAX77696_ERCFLAG0_MR_FSHDN       BIT ( 2) /* Manual Reset Full Shutdown */
#define MAX77696_ERCFLAG0_MR_FRSTRT      BIT ( 3) /* Manual Reset Partial Restart */
#define MAX77696_ERCFLAG0_SFT_PSHDN      BIT ( 4) /* Software Partial Shutdown */
#define MAX77696_ERCFLAG0_SFT_PRSTRT     BIT ( 5) /* Software Partial Restart */
#define MAX77696_ERCFLAG0_SFT_FSHDN      BIT ( 6) /* Software Full Shutdown */
#define MAX77696_ERCFLAG0_SFT_FRSTRT     BIT ( 7) /* Software Full Restart */

#define MAX77696_ERCFLAG1_LBMOK_FSHDN    BIT ( 8) /* Low-Battery Monitor Not Okay Full Shutdown */
#define MAX77696_ERCFLAG1_SYS1UVLO_FSHDN BIT ( 9) /* System 1 Undervoltage Full Shutdown */
#define MAX77696_ERCFLAG1_TOVLO_FSHDN    BIT (10) /* Thermal Overload Full Shutdown */
#define MAX77696_ERCFLAG1_RSTIN_PRSTRT   BIT (11) /* Reset Input Partial Restart */

#define MAX77696_FGIRQMASK_SMX           BIT (5)
#define MAX77696_FGIRQMASK_SMN           BIT (4)
#define MAX77696_FGIRQMASK_VMX           BIT (3)
#define MAX77696_FGIRQMASK_VMN           BIT (2)
#define MAX77696_FGIRQMASK_TMX           BIT (1)
#define MAX77696_FGIRQMASK_TMN           BIT (0)

#define FIN_LOG     ":: --->"
#define FOUT_LOG    ":: <---"

#ifdef FUNCTION_CALL_DEBUG
#define max77696_fin_log(format, args...) \
        pr_debug(FIN_LOG" "format, ##args)
#define max77696_fout_log(format, args...) \
        pr_debug(FOUT_LOG" "format, ##args)
#else
#define max77696_fin_log(format, args...) \
        ({\
            if (0)\
                pr_debug(FIN_LOG" "format, ##args);\
            0;\
        })
#define max77696_fout_log(format, args...) \
        ({\
            if (0)\
                pr_debug(FOUT_LOG" "format, ##args);\
            0;\
        })
#endif

#define FIN0()      max77696_fin_log("%s\n",__func__)
#define FIN1(a)     max77696_fin_log("%s (%X)\n", __func__,(int)(a))
#define FIN2(a,b)   max77696_fin_log("%s (%X,%X)\n", __func__,(int)(a),(int)(b))

#define FOUT0()     max77696_fout_log("%s\n",__func__)
#define FOUT1(a)    max77696_fout_log("%s [%d]\n",__func__,(int)(a))

/*
 *------------------------------------------------------------------------------
 * max77696_read:
 *   (TBD)
 *
 * params:
 *   (TBD)
 *
 * return:
 *   error code
 *------------------------------------------------------------------------------
 */
static __always_inline int max77696_read (struct max77696_i2c *i2c,
    u8 addr, u8 *val)
{
    return i2c->read(i2c, addr, val);
} /* max77696_read */

/*
 *------------------------------------------------------------------------------
 * max77696_write:
 *   (TBD)
 *
 * params:
 *   (TBD)
 *
 * return:
 *   error code
 *------------------------------------------------------------------------------
 */
static __always_inline int max77696_write (struct max77696_i2c *i2c,
    u8 addr, u8 val)
{
    return i2c->write(i2c, addr, val);
} /* max77696_write */

/*
 *------------------------------------------------------------------------------
 * max77696_bulk_read:
 *   (TBD)
 *
 * params:
 *   (TBD)
 *
 * return:
 *   error code
 *------------------------------------------------------------------------------
 */
static __always_inline int max77696_bulk_read (struct max77696_i2c *i2c,
    u8 addr, u8 *dst, u16 len)
{
    return i2c->bulk_read(i2c, addr, dst, len);
} /* max77696_bulk_read */

/*
 *------------------------------------------------------------------------------
 * max77696_bulk_write:
 *   (TBD)
 *
 * params:
 *   (TBD)
 *
 * return:
 *   error code
 *------------------------------------------------------------------------------
 */
static __always_inline int max77696_bulk_write (struct max77696_i2c *i2c,
    u8 addr, const u8 *src, u16 len)
{
    return i2c->bulk_write(i2c, addr, src, len);
} /* max77696_read_masked */

/*
 *------------------------------------------------------------------------------
 * max77696_read_masked:
 *   (TBD)
 *
 * params:
 *   (TBD)
 *
 * return:
 *   error code
 *------------------------------------------------------------------------------
 */
static __always_inline int max77696_read_masked (struct max77696_i2c *i2c,
    u8 addr, u8 mask, u8 *val)
{
    int rc = max77696_read(i2c, addr, val);

    *val &= mask;

    return rc;
} /* max77696_read_masked */

/*
 *------------------------------------------------------------------------------
 * max77696_write_masked:
 *   (TBD)
 *
 * params:
 *   (TBD)
 *
 * return:
 *   error code
 *------------------------------------------------------------------------------
 */
static __always_inline int max77696_write_masked (struct max77696_i2c *i2c,
    u8 addr, u8 mask, u8 val)
{
    int rc;
    u8 buf;

    rc = max77696_read(i2c, addr, &buf);
    if (unlikely(rc)) {
        return rc;
    }

    buf = ((buf & (~mask)) | (val & mask));

    return max77696_write(i2c, addr, buf);
} /* max77696_write_masked */

#endif /* __MAX77696_H__ */

