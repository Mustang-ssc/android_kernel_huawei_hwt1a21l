/*
 * Copyright (C) 2012-2015 HUAWEI
 * Author: L J H
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/* DTS2014071806762 lWX198526 l00220156 20140718 begin */
#ifndef _LINUX_BQ2419X_CHARGER_H
#define _LINUX_BQ2419X_CHARGER_H
/* <DTS2014092405474 caiwei 20140926 begin */
/*<DTS2015010402454 xiongxi xwx234328 20150104 begin*/
#define BOARDID_T1_10 8000
#define BOARDID_T1_A24L 8001
#define BOARDID_T1_A21L 8065
#define BOARDID_T1_A21LT 8052
#define BOARDID_T1_A21W 8017
#define BOARDID_T1_A22L 8033
#define BOARDID_T1_A23L 8049
/*DTS2015010402454 xiongxi xwx234328 20150104 end>*/
/* DTS2014092405474 caiwei 20140926 end> */
#define I2C_ADDR_BQ24192                       (0x6B)

#define BQ2419x_EVENT_UNKOWN                   (0x00)
#define BQ2419x_NO_CHARGER_SOURCE              (0x00)
#define BQ2419x_NOT_CHARGING                   (0x10)
#define BQ2419x_START_CHARGING                 (0x20)
#define BQ2419x_START_AC_CHARGING              (0x30)
#define BQ2419x_START_USB_CHARGING             (0x40)
#define BQ2419x_CHARGE_DONE                    (0x50)
#define BQ2419x_STOP_CHARGING                  (0x60)
#define POWER_SUPPLY_STATE_FAULT               (0x70)
#define POWER_SUPPLY_OVERVOLTAGE               (0x80)
#define POWER_SUPPLY_WEAKSOURCE                (0x90)
#define BQ2419x_RESET_TIMER                    (0x38)

enum usb_charger_type{
    CHARGER_TYPE_USB = 0,
    CHARGER_TYPE_BC_USB,
    CHARGER_TYPE_NON_STANDARD,
    CHARGER_TYPE_STANDARD,
    CHARGER_REMOVED,
    USB_EVENT_OTG_ID,
};

/*Input Source Control Register REG00(default 00110000, or 30)*/
#define INPUT_SOURCE_REG00    0x00

#define BQ2419x_EN_HIZ_SHIFT          7
#define EN_HIZ                          (1)
#define DIS_HIZ                         (0)
#define BQ2419x_VINDPM_SHIFT          3
#define VINDPM_MIN_3880              (3880)
#define VINDPM_4200                  (4200)
#define VINDPM_4360                  (4360)
#define VINDPM_4440                  (4440)
#define VINDPM_4520                  (4520)
#define VINDPM_4600                  (4600)
/*<DTS2014122906101 xiongxi xwx234328 20150126 begin*/
#define VINDPM_4680                  (4680)
/*DTS2014122906101 xiongxi xwx234328 20150126 end>*/
#define VINDPM_MAX_5080              (5080)
#define VINDPM_STEP_80                 (80)
#define IINLIM_100                    (100)
#define IINLIM_150                    (150)
#define IINLIM_500                    (500)
#define IINLIM_900                    (900)
#define IINLIM_1200                  (1200)
#define IINLIM_1500                  (1500)
#define IINLIM_2000                  (2000)
#define IINLIM_3000                  (3000)

/*Power-On Configuration Register REG01(default 00011011, or 1B)*/
#define POWER_ON_CONFIG_REG01    0x01

#define REG_RST               (1 << 7)
#define WATCHDOG_TIMER_RST    (1 << 6)
#define BQ2419x_EN_CHARGER_SHIFT    4
#define DIS_CHARGER                (0)
#define EN_CHARGER                 (1)
#define EN_CHARGER_OTG             (2)
#define BQ2419x_SYS_MIN_SHIFT       1
#define SYS_MIN_MIN_3000        (3000)
#define SYS_MIN_3500            (3500)
#define SYS_MIN_MAX_3700        (3700)
#define SYS_MIN_STEP_100        (100)
#define BOOST_LIM_1300            (1)
#define BOOST_LIM_500             (0)

/*Charge Current Control Register REG02 (bq24190/191/192/193
   default 01100000, or 60; bq24192I default 00100000, or 20)*/
#define CHARGE_CURRENT_REG02    0x02

#define BQ2419x_ICHG_SHIFT    2
#define ICHG_LOW_TEMP    (100)
#define ICHG_MIN         (512)
#define ICHG_640         (640)
#define ICHG_700         (704)
#define ICHG_820         (820)/*768*/
#define ICHG_900         (900)/*896*/
#define ICHG_1000       (1000)/*960*/
#define ICHG_1024       (1024)
#define ICHG_1200       (1200)/*1152*/
#define ICHG_1500       (1500)/*1472*/
#define ICHG_1536       (1536)
#define ICHG_2000       (2000)/*1984*/
#define ICHG_2048       (2048)
#define ICHG_2500       (2500)/*2496*/
#define ICHG_2944       (2944)
#define ICHG_3000       (3000)/*3008*/
#define ICHG_MAX        (4544)
#define ICHG_STEP_64      (64)
#define EN_FORCE_20PCT	   (1)   /*fast charge current as 20% of programmed value in REG02*/
#define DIS_FORCE_20PCT	   (0)   /* fast charge current as programmed value in REG02*/

/*Pre-Charge/Termination Current Control Register REG 03 (default 00010001, or 11)*/
#define PRECHARGE_TERM_CURRENT_REG03  0x03

#define BQ2419x_IPRECHRG_SHIFT    4
#define IPRECHRG_MIN_128    (128)
#define IPRECHRG_256        (256)
#define IPRECHRG_640        (640)
#define IPRECHRG_MAX_2048  (2048)
#define IPRECHRG_STEP_128   (128)
#define ITERM_MIN_128       (128)
#define ITERM_256           (256)
#define ITERM_MAX_2048     (2048)
#define ITERM_STEP_128      (128)

/*Charge Voltage Control Register REG04 (bq24190/191/192/193
    default: 10110010, or B2; bq24192I default 10011010, or 9A)*/
#define CHARGE_VOLTAGE_REG04    0x04

#define BQ2419x_VCHARGE_SHIFT             2
#define VCHARGE_MIN_3504              (3504)
#define VCHARGE_4200                  (4208) //(4.192)
#define VCHARGE_4300                  (4304) //(4.288)
#define VCHARGE_4350                  (4352) //(4.336)
#define VCHARGE_MAX_4400              (4400)
#define VCHARGE_STEP_16               (16)

#define BATLOWV_3000                 (1 << 1)
#define BATLOWV_2800                 (0 << 1)
#define VRECHG_300                   (1 << 0)
#define VRECHG_100                   (0 << 0)

/*Charge Termination/Timer Control Register REG05 (default 10011010, or 9A)*/
#define CHARGE_TERM_TIMER_REG05    0x05

#define BQ2419x_EN_TERM_SHIFT       7
#define EN_TERM                         (1)
#define DIS_TERM                        (0)
#define TERM_STAT                  (0 << 6)
#define WATCHDOG_DIS               (0 << 4)
#define WATCHDOG_40                (1 << 4)
#define BQ2419x_EN_TIMER_SHIFT      3
#define EN_TIMER                        (1)
#define DIS_TIMER                       (0)
#define CHG_TIMER_8                (1 << 1)
#define CHG_TIMER_12               (2 << 1)
#define CHG_TIMER_20               (3 << 1)
#define JEITA_ISET_HALF_ICHG            (0)
#define JEITA_ISET                      (1)

/*IR Compensation / Thermal Regulation Control Register REG06
   (default 00000011, or 03)*/

#define THERMAL_REGUALTION_REG06    0x06

#define BQ2419x_BAT_COMP_SHIFT      5
#define BAT_COMP_MIN_0             (0)
#define BAT_COMP_40               (40)
#define BAT_COMP_MAX_70           (70)
#define BAT_COMP_STEP_10          (10)
#define BQ2419x_VCLAMP_SHIFT        2
#define VCLAMP_MIN_0               (0)
#define VCLAMP_48                 (48)
#define VCLAMP_MAX_112           (112)
#define VCLAMP_STEP_16            (16)
#define TREG_120                   (3)
#define TREG_100                   (2)
#define TREG_80                    (1)
#define TREG_60                    (0)

/*Misc Operation Control Register REG07 (default 01000011, or 43)*/
#define MISC_OPERATION_REG07   0x07

#define BQ2419x_DPDM_EN_SHIFT                7
#define DPDM_EN                            (1)
#define DPDM_DIS                           (0)
#define TMR2X_EN                      (1 << 6)
#define BQ2419x_BATFET_EN_SHIFT              5
#define EN_BATFET                          (0)
#define DIS_BATFET                         (1)
#define JEITA_VSET_4050               (0 << 4)
#define JEITA_VSET_4200               (1 << 4)
#define CHRG_FAULT_INT_DIS            (0 << 1)
#define CHRG_FAULT_INT_EN             (1 << 1)
#define BAT_FAULT_INT_DIS             (0 << 0)
#define BAT_FAULT_INT_EN              (1 << 0)

/*System Status Register REG08*/
#define SYS_STATUS_REG08    0x08

#define BQ2419x_VBUS_STAT_UNKNOWM        (0x00)
#define BQ2419x_VBUS_STAT_USB_HOST       (0x40)
#define BQ2419x_VBUS_STAT_ADATPTER       (0x80)
#define BQ2419x_VBUS_STAT_OTG            (0xC0)
#define BQ2419x_CHGR_STAT_NOT_CHARGING   (0x00)
#define BQ2419x_CHGR_STAT_PRE_CHARGING   (0x10)
#define BQ2419x_CHGR_STAT_FAST_CHARGING  (0x20)
#define BQ2419x_CHGR_STAT_CHAEGE_DONE    (0x30)
#define BQ2419x_NOT_DPM_STAT             (0x00)
#define BQ2419x_DPM_STAT                 (0x08)
#define BQ2419x_NOT_PG_STAT              (0x00)
#define BQ2419x_PG_STAT                  (0x04)
#define BQ2419x_THERM_STAT_NOEMAL        (0x00)
#define BQ2419x_THERM_STAT_TREG          (0x02)
#define BQ2419x_NOT_VSYS_STAT            (0x00)
#define BQ2419x_VSYS_STAT                (0x01)



/*Fault Register REG09*/
#define CHARGER_FAULT_REG09    0x09

#define BQ2419x_WATCHDOG_FAULT          (0x80)
#define BQ2419x_OTG_FAULT               (0x40)
#define BQ2419x_POWER_SUPPLY_OVP        (0x10)
#define BQ2419x_THERMAL_SHUTDOWM        (0x20)
#define BQ2419x_CHRG_TIMER_EXPIRED      (0x30)
#define BQ2419x_BAT_FAULT_OVP           (0x08)
#define BQ2419x_NTC_TS1_COLD            (0x01)
#define BQ2419x_NTC_TS1_HOT             (0x02)
#define BQ2419x_NTC_TS2_COLD            (0x03)
#define BQ2419x_NTC_TS2_HOT             (0x04)
#define BQ2419x_NTC_TS1_TS2_COLD        (0x05)
#define BQ2419x_NTC_TS1_TS2_HOT         (0x06)
#define BQ2419x_NTC_TS1_TS2_HOT_COLD    (0x07)

/*Vender / Part / Revision Status Register REG0A*/

#define PART_REVISION_REG0A     0x0A

#define BQ24190                 (0x00)
#define BQ24191                 (0x08)
#define BQ24192                 (0x10)
#define BQ24192I                (0x18)
/* <DTS2014073105492 modified by l00220156 20140731 for bq24296 begin*/
#define BQ24296                 (0x20)
/* <DTS2014073105492 modified by l00220156 20140731 for bq24296 end*/

/*<DTS2014122508930 added by xiongxi xwx234328 20141227 begin*/
#define BQ24196                 (0x2B)
/*DTS2014122508930 added by xiongxi xwx234328 20141227 end>*/

#define BQ2419x                 (0x28)
#define TS_PROFILE_WINDOW       (0x00)
#define TS_PROFILE_JEITA        (1 << 2)

#define BQ2419x_WATCHDOG_TIMEOUT    (30000)

/*two stage charger*/
#define TWO_STAGE_CHARGE_FIRST_STAGE                 (0x00)
#define TWO_STAGE_CHARGE_SECOND_STAGE                 (0x01)

/*set gpio_074 to control CE pin to disable/enable bq24161 IC*/
#define ENABLE_BQ2419x_CHARGER        114

/*(-10 ) battery temperature is -10 degree*/
#define BQ2419x_COLD_BATTERY_THRESHOLD     (-10)
 /*(0 ) battery temperature is 0 degree*/
#define BQ2419x_COOL_BATTERY_THRESHOLD      (0)
 /*( 5 ) battery temperature is 5 degree*/
#define BQ2419x_BATTERY_THRESHOLD_5       (5)
 /*( 15 ) battery temperature is 15 degree*/
#define BQ2419x_BATTERY_THRESHOLD_15       (10)
 /*( 45 ) battery temperature is 45 degree*/
/* < DTS2014030407914 jiangfei 20140304 begin */
#define BQ2419x_WARM_BATTERY_THRESHOLD     (42)
/* DTS2014030407914 jiangfei 20140304 end > */

/*DTS2014091107692 xiongxi xwx234328 begin*/
 /*( 45 ) battery temperature is 45 degree*/
#define BQ2419x_HOT_BATTERY_THRESHOLD      (45)
/*DTS2014091107692 xiongxi xwx234328 end*/

 /*( 3 ) battery temperature offset is 3 degree*/
#define BQ2419x_TEMPERATURE_OFFSET          (1)

 /* default battery capacity for error that can't get Capacity */
#define BQ2419x_DEFAULT_CAPACITY            (3760)
/* < DTS2014031501494 yuanzhen 20140317 begin */
 /* default charge parameter when low temperature */
#define DEFAULT_CHARGE_PARAM_LOW_TEMP       (3)
/* DTS2014031501494 yuanzhen 20140317 end > */

 /*(3.0V) battery preconditioning voltage is 3.0V*/

 /*low temperature charge termination voltage*/
/* < DTS2014031501494 yuanzhen 20140317 begin */
/* < DTS2014011515091 yuanzhen 20140115 begin*/
#define BQ2419x_HIGH_TEMP_DELTA  (250)
#define OTG_INT_WORK_TIMER (1000)
#define OTG_INT_WORK_CNT (2)
/* DTS2014011515091 yuanzhen 20140115 end > */
/* DTS2014031501494 yuanzhen 20140317 end > */
#define BQ2419x_LOW_TEMP_TERM_VOLTAGE    (4000)
#define BQ2419x_LOW_TEMP_NOT_CHARGING_VOLTAGE    (4100)
#define BQ2419x_NORNAL_ICHRG_VOLTAGE     (3400)
#define CAPACITY_LEVEL_HIGH_THRESHOLD    (90)

/* < DTS2014031501494 yuanzhen 20140317 begin */
struct bq2419x_platform_data {
    int max_charger_currentmA;
    int max_charger_voltagemV;
    int termination_currentmA;
    int max_cin_limit_currentmA;
    int gpio;
    int gpio_int;
    int gpio_otg;
};
/* DTS2014031501494 yuanzhen 20140317 end > */

struct charge_params {
    unsigned long       currentmA;
    unsigned long       voltagemV;
    unsigned long       term_currentmA;
    unsigned long       enable_iterm;
    bool                enable;
};

/* < DTS2014031501494 yuanzhen 20140317 begin */
struct bq2419x_device_info {
    struct device        *dev;
    struct i2c_client    *client;
    struct charge_params  params;
    struct delayed_work   bq2419x_charger_work;
    struct delayed_work   bq2419x_usb_otg_work;
    struct work_struct    usb_work;
    struct delayed_work   otg_int_work;

    unsigned int      otg_int_work_cnt;
    unsigned int      wakelock_enabled;
    unsigned short    input_source_reg00;
    unsigned short    power_on_config_reg01;
    unsigned short    charge_current_reg02;
    unsigned short    prechrg_term_current_reg03;
    unsigned short    charge_voltage_reg04;
    unsigned short    term_timer_reg05;
    unsigned short    thermal_regulation_reg06;
    unsigned short    misc_operation_reg07;
    unsigned short    system_status_reg08;
    unsigned short    charger_fault_reg09;
    unsigned short    bqchip_version;

    unsigned int      max_currentmA;
    unsigned int      max_voltagemV;
    unsigned int      max_cin_currentmA;

    unsigned int    cin_dpmmV;
    unsigned int    cin_limit;
    /* < DTS2014011515091 yuanzhen 20140115 begin*/
    unsigned int    hot_design_current;
    /* DTS2014011515091 yuanzhen 20140115 end > */
    unsigned int    chrg_config;
    unsigned int    sys_minmV;
    unsigned int    currentmA;
    unsigned int    prechrg_currentmA;
    unsigned int    term_currentmA;
    unsigned int    voltagemV;
    unsigned int    watchdog_timer;
    unsigned int    chrg_timer;
    unsigned int    bat_compohm;
    unsigned int    comp_vclampmV;

    bool    hz_mode;
    bool    boost_lim;
    bool    enable_low_chg;
    bool    cfg_params;
    bool    enable_iterm;
    bool    enable_timer;
    bool    enable_batfet;
    bool    cd_active;
    bool    factory_flag;
    bool    calling_limit;
    bool    battery_present;
    bool    enable_dpdm;
    int     charge_status;
    int     charger_source;
    int     timer_fault;
    unsigned int    battery_temp_status;
    unsigned long           event;
    struct notifier_block   nb;

    int                      gpio;
    int     gpio_int;
    int     irq_int;
    int     battery_voltage;
    int     temperature_cold;
    int     temperature_cool;
    int     temperature_warm;
    int     temperature_hot;
    bool    coldhot_charging_flag;
    bool    limit_charging_flag;
    bool    battery_full;
    int     temperature_5;
    int     temperature_15;
    u32    charge_full_count;

    /* these parameters are for charging between 0C-5C & 5C-15C, 1-0.1*capacity...
       charge_in_temp_5 means the parameter for charging between 0C-5C. */
    unsigned int design_capacity;
    unsigned int charge_in_temp_5;
    unsigned int charge_in_temp_15;

    int  japan_charger;
    int  is_two_stage_charger;
    int  two_stage_charger_status;
    int  first_stage_voltage;
    int  second_stage_voltage;
    int  is_disable_cool_temperature_charger;
    int  high_temp_para;
/* <DTS2014092405474 caiwei 20140926 begin */
    unsigned int  hw_board_id;
/* DTS2014092405474 caiwei 20140926 end> */
/* <DTS2014093003947 (40~45 degree charge current 1A) caiwei 20141009 begin */
    bool hw_is_pad;
/* DTS2014093003947 (40~45 degree charge current 1A) caiwei 20141009 end> */
};
/* DTS2014031501494 yuanzhen 20140317 end > */

int bq2419x_register_notifier(struct notifier_block *nb,
                unsigned int events);
int bq2419x_unregister_notifier(struct notifier_block *nb,
                unsigned int events);
/* <Begin DTS2014080603839 modified by l00220156 for interface of vm and charger 20140806 > */
int bq2419x_get_charge_type(void);
/* <End DTS2014080603839 modified by l00220156 for interface of vm and charger 20140806 > */
/* < DTS2014011515091 yuanzhen 20140115 begin*/
void bq2419x_config_input_source_reg(struct bq2419x_device_info *di);
/* DTS2014011515091 yuanzhen 20140115 end > */
/* <DTS2014012404412 sunwenyong 20140128 begin */
void bq2419x_set_enable_hz_mode_for_factory_test(struct bq2419x_device_info *di, int val);
void bq2419x_set_enable_charger_for_factory_test(struct bq2419x_device_info *di, int val);
/* DTS2014012404412 sunwenyong 20140128 end> */
#endif
/* DTS2014071806762 lWX198526 l00220156 20140718 end */
