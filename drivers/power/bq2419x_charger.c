/*
 * drivers/power/bq2419x_charger.c
 *
 * BQ24190/1/2/3/4 charging driver
 *
 * Copyright (C) 2012-2015 HUAWEI, Inc.
 * Author: HUAWEI, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/* DTS2014071806762 lWX198526 l00220156 20140718 begin */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/wakelock.h>
#include <linux/usb/otg.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/power/bq2419x_charger.h>
//#include <linux/power/bq27510_bms.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
/* <DTS2014092405474 caiwei 20140926 begin */
#include <soc/qcom/socinfo.h>
/* DTS2014092405474 caiwei 20140926 end> */
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#include <linux/power/hisi_coul_drv.h>

struct bq2419x_device_info *bq_device_19x;

extern struct hisi_coul_ops *g_coul_ops;

//extern struct bq27510_device_info *g_battery_measure_by_bq27510_device;
//extern int get_battery_present_status(void);
//extern void do_power_supply_update(void);

static struct wake_lock chrg_lock;
u32 wakeup_timer_seconds;
/* <DTS2014093003947 (report SOC 100%) caiwei 20141009 begin */
#define CHARGE_DONE_COUNT_MAX (10)
int g_charge_done_count = 0;
/* DTS2014093003947 (report SOC 100%) caiwei 20141009 end >*/
/* < DTS2014031501494 yuanzhen 20140317 begin */
static unsigned int irq_int_active;
/* DTS2014031501494 yuanzhen 20140317 end > */
/* <DTS2014111002912 caiwei 20141112 begin */
static unsigned int last_otg_status = 0; //record OTG status
/* DTS2014111002912 caiwei 20141112 end> */
/* <DTS2015012010751 caiwei 20150120 begin */
static unsigned int factory_flag_5V2A = 0;
/* DTS2015012010751 caiwei 20150120 end> */
enum{
    BATTERY_HEALTH_TEMPERATURE_NORMAL = 0,
    BATTERY_HEALTH_TEMPERATURE_OVERLOW,
    BATTERY_HEALTH_TEMPERATURE_LOW,
    BATTERY_HEALTH_TEMPERATURE_NORMAL_HIGH,
    BATTERY_HEALTH_TEMPERATURE_HIGH,
    /* < DTS2014030407914 jiangfei 20140304 begin */
    BATTERY_HEALTH_TEMPERATURE_HIGH_HOT,
    /* DTS2014030407914 jiangfei 20140304 end > */
    BATTERY_HEALTH_TEMPERATURE_OVERHIGH,
    BATTERY_HEALTH_TEMPERATURE_15,
    BATTERY_HEALTH_TEMPERATURE_5,
    BATTERY_HEALTH_TEMPERATURE_HIGH_CP1,
    BATTERY_HEALTH_TEMPERATURE_HIGH_CP2,
    BATTERY_HEALTH_TEMPERATURE_HIGH_CP3,
    BATTERY_HEALTH_TEMPERATURE_HIGH_CP4,
    BATTERY_HEALTH_TEMPERATURE_HIGH_CP5,
};
enum{
    NORMAL_TEMP_CONFIG = 0,  // (BATTERY_HEALTH_TEMPERATURE_NORMAL)
    NORMAL_HIGH_TEMP_CONFIG, // (BATTERY_HEALTH_TEMPERATURE_NORMAL_HIGH)
    HIGH_TEMP_CONFIG,        // (BATTERY_HEALTH_TEMPERATURE_HIGH)
    TEMP_CONFIG_15,
    TEMP_CONFIG_5,
    NORMAL_SECOND_STAGE,//for two charging stage
    NORMAL_HIGH_TEMP_CONFIG_CP1,
    NORMAL_HIGH_TEMP_CONFIG_CP2,
    NORMAL_HIGH_TEMP_CONFIG_CP3,
    NORMAL_HIGH_TEMP_CONFIG_CP4,
};

//tempture checkpoint
struct bq2419x_high_temp_cp{
    int cp1;
    int cp2;
    int cp3;
    int cp4;
    int cp5;
    int cp6;
};

enum{
    HIGH_TEMP_CP_U9701L = 0,
    HIGH_TEMP_CP_U9700L = 1,
};

struct bq2419x_high_temp_cp japan_temp_cp[] ={
    {40, 42, 43, 45, 53, 55}, //HIGH_TEMP_CP_U9701L
    {37, 39, 40, 42, 43, 45},//HIGH_TEMP_CP_U9700L
    {39, 41, 42, 45, 53, 55},//HIGH_TEMP_CP_U9700GVC China area
    {0},
};

#if defined(CONFIG_CHARGER_BQ2419x) || defined(CONFIG_QPNP_VM_BMS)
/* <DTS2014102505836 caiwei 20141030 begin */
void do_power_supply_update_otg(void)
{
    struct power_supply* usb_psy = NULL;
    static int charger_type;

    usb_psy = power_supply_get_by_name("usb");
    if (!usb_psy)
    {
        pr_err("usb supply not found deferring probe\n");
        return;
    }
    pr_info("power_supply_set_supply_type(usb, POWER_SUPPLY_TYPE_UNKNOWN)\n");
    charger_type = POWER_SUPPLY_TYPE_UNKNOWN;

    power_supply_set_supply_type(usb_psy, charger_type);

}
/* DTS2014102505836 caiwei 20141030 end> */
void do_power_supply_update(void)
{
    struct power_supply *battery_psy = NULL;
    
    battery_psy = power_supply_get_by_name("battery");
    if (!battery_psy) {
        pr_err("usb supply not found deferring probe\n");
        return ;
    }
	/*DTS2014091301574,begin:rm linear charger, 2014.9.30*/
    pr_info("power_supply_changed(battery_psy)\n");
	/*DTS2014091301574,end:rm linear charger, 2014.9.30*/
    power_supply_changed(battery_psy);
    
    if(g_coul_ops != NULL)
    {
        //power_supply_changed(&(g_battery_measure_by_qpnp_vm_bms->bms_psy));
        /* <DTS2013122802817 chenyuanquan 20131228 begin */
        //BQ27510_DBG("%s\n",__func__);
        /* DTS2013122802817 chenyuanquan 20131228 end> */
    }
    else
    {
        pr_err("%s:called before init\n",__func__);
    }
}
#endif

static int bq2419x_get_max_charge_voltage(struct bq2419x_device_info *di)
{
    bool ret = 0;	

    /*
    * This function is used while one product have two different batterys
    * which have different max voltage. But we don't suggest this situation.
    * As normal condition, the max voltage is configured on device tree.
    */

    return ret;
}

static int bq2419x_write_block(struct bq2419x_device_info *di, u8 *value,
                               u8 reg, unsigned num_bytes)
{
    struct i2c_msg msg[1];
    int ret = 0;

    *value = reg;

    msg[0].addr = di->client->addr;
    msg[0].flags = 0;
    msg[0].buf = value;
    msg[0].len = num_bytes + 1;

   ret = i2c_transfer(di->client->adapter, msg, 1);

    /* i2c_transfer returns number of messages transferred */
    if (ret != 1) {
        dev_err(di->dev,
                "i2c_write failed to transfer all messages\n");
        if (ret < 0)
            return ret;
        else
            return -EIO;
     } else {
        return 0;
   }
}

static int bq2419x_read_block(struct bq2419x_device_info *di, u8 *value,
                            u8 reg, unsigned num_bytes)
{
    struct i2c_msg msg[2];
    u8 buf = 0;
    int ret = 0;

    buf = reg;

    msg[0].addr = di->client->addr;
    msg[0].flags = 0;
    msg[0].buf = &buf;
    msg[0].len = 1;

    msg[1].addr = di->client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].buf = value;
    msg[1].len = num_bytes;

    ret = i2c_transfer(di->client->adapter, msg, 2);

    /* i2c_transfer returns number of messages transferred */
    if (ret != 2) {
         dev_err(di->dev,
              "i2c_write failed to transfer all messages\n");
        if (ret < 0)
            return ret;
        else
            return -EIO;
    } else {
        return 0;
    }
}

static int bq2419x_write_byte(struct bq2419x_device_info *di, u8 value, u8 reg)
{
    /* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
    u8 temp_buffer[2] = { 0 };

    /* offset 1 contains the data */
    temp_buffer[1] = value;
    return bq2419x_write_block(di, temp_buffer, reg, 1);
}

static int bq2419x_read_byte(struct bq2419x_device_info *di, u8 *value, u8 reg)
{
    return bq2419x_read_block(di, value, reg, 1);
}
/* < DTS2014011515091 yuanzhen 20140115 begin*/
/* get the minimal current about cin*/
static int bq2419x_config_hot_design_current(struct bq2419x_device_info *di)
{
    /* <DTS2014012204748 chenyuanquan 20140312 begin */
    /* thermal design will set this as 0 when cpu cools down, so avoid the issue */
    if (di->hot_design_current == 0)
        di->hot_design_current = di->max_cin_currentmA;
    /* DTS2014012204748 chenyuanquan 20140312 end> */
    return di->cin_limit <= di->hot_design_current ? di->cin_limit : di->hot_design_current;
}

/* < DTS2014031501494 yuanzhen 20140317 begin */
void bq2419x_config_input_source_reg(struct bq2419x_device_info *di)
{
    unsigned int vindpm = 0;
    unsigned int current_I_limit = 0;
    u8 Vdpm = 0;
    u8 Iin_limit = 0;
    
    vindpm = di->cin_dpmmV;
    
    if(vindpm < VINDPM_MIN_3880)
        vindpm = VINDPM_MIN_3880;
    else if (vindpm > VINDPM_MAX_5080)
        vindpm = VINDPM_MAX_5080;
    
    current_I_limit = bq2419x_config_hot_design_current(di);
    
    if (current_I_limit <= IINLIM_100)
        Iin_limit = 0;
    else if (current_I_limit > IINLIM_100 && current_I_limit <= IINLIM_150)
        Iin_limit = 1;
    else if (current_I_limit > IINLIM_150 && current_I_limit <= IINLIM_500)
        Iin_limit = 2;
    else if (current_I_limit > IINLIM_500 && current_I_limit <= IINLIM_900)
        Iin_limit = 3;
    else if (current_I_limit > IINLIM_900 && current_I_limit <= IINLIM_1200)
        Iin_limit = 4;
    else if (current_I_limit > IINLIM_1200 && current_I_limit <= IINLIM_1500)
        Iin_limit = 5;
    else if (current_I_limit > IINLIM_1500 && current_I_limit <= IINLIM_2000)
        Iin_limit = 6;
    else if (current_I_limit > IINLIM_2000 && current_I_limit <= IINLIM_3000)
        Iin_limit = 7;
    else
        Iin_limit = 4;
    
    Vdpm = (vindpm -VINDPM_MIN_3880)/VINDPM_STEP_80;
    
    di->input_source_reg00 = (di->hz_mode << BQ2419x_EN_HIZ_SHIFT)
                    | (Vdpm << BQ2419x_VINDPM_SHIFT) |Iin_limit;
    
    bq2419x_write_byte(di, di->input_source_reg00, INPUT_SOURCE_REG00);
    return;
}
/* DTS2014011515091 yuanzhen 20140115 end > */
static void bq2419x_config_power_on_reg(struct bq2419x_device_info *di)
{
    unsigned int sys_min = 0;
    u8 Sysmin = 0;
    
    sys_min = di->sys_minmV;
    
    if(sys_min < SYS_MIN_MIN_3000)
        sys_min = SYS_MIN_MIN_3000;
    else if (sys_min > SYS_MIN_MAX_3700)
        sys_min = SYS_MIN_MAX_3700;
    
    Sysmin = (sys_min -SYS_MIN_MIN_3000)/SYS_MIN_STEP_100;
    
    di->power_on_config_reg01 = WATCHDOG_TIMER_RST
        | (di->chrg_config << BQ2419x_EN_CHARGER_SHIFT)
        | (Sysmin << BQ2419x_SYS_MIN_SHIFT) | di->boost_lim;
    
    bq2419x_write_byte(di, di->power_on_config_reg01, POWER_ON_CONFIG_REG01);
    return;
}

static void bq2419x_config_current_reg(struct bq2419x_device_info *di)
{
    unsigned int currentmA = 0;
    u8 Vichrg = 0;

    currentmA = di->currentmA;
    /* if currentmA is below ICHG_MIN, we can set the ICHG to 5*currentmA and
       set the FORCE_20PCT in REG02 to make the true current 20% of the ICHG*/
    if (currentmA < ICHG_MIN) {
        currentmA = currentmA * 5;
        di->enable_low_chg = EN_FORCE_20PCT;
    } else {
        di->enable_low_chg = DIS_FORCE_20PCT;
    }
    if (currentmA < ICHG_MIN)
        currentmA = ICHG_MIN;
    else if(currentmA > ICHG_MAX)
        currentmA = ICHG_MAX;
    Vichrg = (currentmA - ICHG_MIN)/ICHG_STEP_64;

    di->charge_current_reg02 = (Vichrg << BQ2419x_ICHG_SHIFT) | di->enable_low_chg;

     bq2419x_write_byte(di, di->charge_current_reg02, CHARGE_CURRENT_REG02);
     return;
}
/* DTS2014031501494 yuanzhen 20140317 end > */

static void bq2419x_config_prechg_term_current_reg(struct bq2419x_device_info *di)
{
    unsigned int precurrentmA = 0;
    unsigned int term_currentmA = 0;
    u8 Prechg = 0;
    u8 Viterm = 0;

    precurrentmA = di->prechrg_currentmA;
    term_currentmA = di->term_currentmA;

    if(precurrentmA < IPRECHRG_MIN_128)
        precurrentmA = IPRECHRG_MIN_128;
    if(term_currentmA < ITERM_MIN_128)
        term_currentmA = ITERM_MIN_128;

    if((di->bqchip_version & BQ24192I)) {
        if(precurrentmA > IPRECHRG_640)
            precurrentmA = IPRECHRG_640;
     }

    if ((di->bqchip_version & (BQ24190|BQ24191|BQ24192))) {
        if (precurrentmA > IPRECHRG_MAX_2048)
            precurrentmA = IPRECHRG_MAX_2048;
    }

    if (term_currentmA > ITERM_MAX_2048)
        term_currentmA = ITERM_MAX_2048;

    Prechg = (precurrentmA - IPRECHRG_MIN_128)/IPRECHRG_STEP_128;
    Viterm = (term_currentmA-ITERM_MIN_128)/ITERM_STEP_128;

    di->prechrg_term_current_reg03 = (Prechg <<  BQ2419x_IPRECHRG_SHIFT| Viterm);
    bq2419x_write_byte(di, di->prechrg_term_current_reg03, PRECHARGE_TERM_CURRENT_REG03);
    return;
}

static void bq2419x_config_voltage_reg(struct bq2419x_device_info *di)
{
    unsigned int voltagemV = 0;
    u8 Voreg = 0;

    voltagemV = di->voltagemV;
    if (voltagemV < VCHARGE_MIN_3504)
        voltagemV = VCHARGE_MIN_3504;
    else if (voltagemV > VCHARGE_MAX_4400)
        voltagemV = VCHARGE_MAX_4400;

    Voreg = (voltagemV - VCHARGE_MIN_3504)/VCHARGE_STEP_16;

    di->charge_voltage_reg04 = (Voreg << BQ2419x_VCHARGE_SHIFT) | BATLOWV_3000 |VRECHG_100;
    bq2419x_write_byte(di, di->charge_voltage_reg04, CHARGE_VOLTAGE_REG04);
    return;
}

static void bq2419x_config_term_timer_reg(struct bq2419x_device_info *di)
{
    di->term_timer_reg05 = (di->enable_iterm << BQ2419x_EN_TERM_SHIFT)
        | di->watchdog_timer | (di->enable_timer << BQ2419x_EN_TIMER_SHIFT)
        | di->chrg_timer | JEITA_ISET;

    bq2419x_write_byte(di, di->term_timer_reg05, CHARGE_TERM_TIMER_REG05);
    return;
}

static void bq2419x_config_thernal_regulation_reg(struct bq2419x_device_info *di)
{
    unsigned int batcomp_ohm = 0;
    unsigned int vclampmV = 0;
    u8 Batcomp = 0;
    u8 Vclamp = 0;

    batcomp_ohm = di->bat_compohm;
    vclampmV = di->comp_vclampmV;

    if(batcomp_ohm > BAT_COMP_MAX_70)
        batcomp_ohm = BAT_COMP_MAX_70;

    if(vclampmV > VCLAMP_MAX_112)
        vclampmV = VCLAMP_MAX_112;

    Batcomp = batcomp_ohm/BAT_COMP_STEP_10;
    Vclamp = vclampmV/VCLAMP_STEP_16;

    di->thermal_regulation_reg06 = (Batcomp << BQ2419x_BAT_COMP_SHIFT)
                           |(Vclamp << BQ2419x_VCLAMP_SHIFT) |TREG_120;

    bq2419x_write_byte(di, di->thermal_regulation_reg06, THERMAL_REGUALTION_REG06);
    return;
}

static void bq2419x_config_misc_operation_reg(struct bq2419x_device_info *di)
{
    di->misc_operation_reg07 = (di->enable_dpdm << BQ2419x_DPDM_EN_SHIFT)
          | TMR2X_EN |(di->enable_batfet<< BQ2419x_BATFET_EN_SHIFT)
          | CHRG_FAULT_INT_EN |BAT_FAULT_INT_EN;

    bq2419x_write_byte(di, di->misc_operation_reg07, MISC_OPERATION_REG07);
    return;
}

/*deal with poor charger when capacity is more than 90% ,if hardware does not
 use REGN for usb_int,pls delete the follow fuction*/
static void bq2419x_reset_vindpm(struct bq2419x_device_info *di)
{
    int battery_capacity = 0;

    if ((di->charger_source == POWER_SUPPLY_TYPE_MAINS)&&(di->cin_dpmmV != VINDPM_4600)){
         /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 begin*/
         if (g_coul_ops->battery_capacity){
             battery_capacity = g_coul_ops->battery_capacity();    
         } else {
            // 默认保证DPM
             battery_capacity = CAPACITY_LEVEL_HIGH_THRESHOLD + 1;
             dev_notice(di->dev, "g_coul_ops->battery_capacity NULL, battery_capacity=%d\n", battery_capacity);            
         }
         /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 end*/
         if(battery_capacity > CAPACITY_LEVEL_HIGH_THRESHOLD){
             di->cin_dpmmV = VINDPM_4600;
             bq2419x_config_input_source_reg(di);
         }
     }
     return;
}

/*0 = temperature less than 42,1 = temperature more than 42 and less 45,2 more than 45 */
static void bq2419x_calling_limit_ac_input_current(struct bq2419x_device_info *di,int flag)
{
    if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
    {
       switch(flag){
       case NORMAL_TEMP_CONFIG:
           if(di->calling_limit){
               di->cin_limit = IINLIM_900;
               di->currentmA = ICHG_820;
           } else {
               di->cin_limit = di->max_cin_currentmA;
               di->currentmA = di->max_currentmA;
           }
           break;
       case NORMAL_HIGH_TEMP_CONFIG:
           if(di->calling_limit){
               di->cin_limit = IINLIM_900;
               di->currentmA = ICHG_820;
           } else {
               if(di->cin_limit == di->max_cin_currentmA)
                   di->currentmA =  di->max_currentmA;
               di->cin_limit = di->cin_limit;
               di->currentmA = di->currentmA;
           }

           break;
       case HIGH_TEMP_CONFIG:
           if(di->calling_limit){
               di->cin_limit = IINLIM_900;
               di->currentmA = ICHG_820;
           } else {
               di->cin_limit = IINLIM_900;
               di->currentmA = ICHG_820;
           }
           break;
       case TEMP_CONFIG_5:
           if(di->calling_limit){
               di->cin_limit = IINLIM_900;
               di->currentmA = di->design_capacity / 10 * di->charge_in_temp_5;
           } else {
               di->cin_limit = di->max_cin_currentmA;
               /* battery whose max_voltage is above 4.35V is easy to broken
                  when the temperature is below 15℃.
                  So we need set the Current below 0.x * Capacity. */
               di->currentmA = di->design_capacity / 10 * di->charge_in_temp_5;
           }
           break;
       case TEMP_CONFIG_15:
           if(di->calling_limit){
               di->cin_limit = IINLIM_900;
               di->currentmA = di->design_capacity / 10 * di->charge_in_temp_15;
           } else {
               di->cin_limit = di->max_cin_currentmA;
               di->currentmA = di->design_capacity / 10 * di->charge_in_temp_15;
           }
           break;
        case NORMAL_SECOND_STAGE:
            if(di->calling_limit){
               di->cin_limit = IINLIM_900;
               di->currentmA = ICHG_820;
           } else {
               di->cin_limit = IINLIM_1200;
               di->currentmA = ICHG_1024;
           }
           break;
       case NORMAL_HIGH_TEMP_CONFIG_CP1:
             if(di->calling_limit){
               di->cin_limit = di->cin_limit;
               di->currentmA = di->currentmA;
           } else {
               di->cin_limit = di->cin_limit;
               di->currentmA = di->currentmA;
           }
           break;
       case NORMAL_HIGH_TEMP_CONFIG_CP2:
            if(di->calling_limit){
               di->cin_limit = IINLIM_900;
               di->currentmA = ICHG_820;
           } else {
               di->cin_limit = IINLIM_900;
               di->currentmA = ICHG_820;
           }
           break;
       case NORMAL_HIGH_TEMP_CONFIG_CP3:
            if(di->calling_limit){
               di->cin_limit = di->cin_limit;
               di->currentmA = di->currentmA;
           } else {
               di->cin_limit = di->cin_limit;
               di->currentmA = di->currentmA;
           }
           break;
       case NORMAL_HIGH_TEMP_CONFIG_CP4:
            if(di->calling_limit){
               di->cin_limit = IINLIM_500;
               di->currentmA = ICHG_MIN;
           } else {
               di->cin_limit = IINLIM_500;
               di->currentmA = ICHG_MIN;
           }
        }
    }
   return;
}

static void bq2419x_config_limit_temperature_parameter(struct bq2419x_device_info *di)
{
    di->coldhot_charging_flag = 0;//DIS_CHARGER;
    di->limit_charging_flag = 1;//EN_CHARGER;
    di->temperature_cold = BQ2419x_COLD_BATTERY_THRESHOLD;
    di->temperature_cool = BQ2419x_COOL_BATTERY_THRESHOLD;
    di->temperature_warm = BQ2419x_WARM_BATTERY_THRESHOLD;
    di->temperature_hot  = BQ2419x_HOT_BATTERY_THRESHOLD;
    di->temperature_5    = BQ2419x_BATTERY_THRESHOLD_5;
    di->temperature_15   = BQ2419x_BATTERY_THRESHOLD_15;
    return;
}

static int bq2419x_check_battery_temperature_japan_threshold(struct bq2419x_device_info *di)
{
    int battery_temperature = 0;
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 begin*/
    if (g_coul_ops->battery_temperature){
        battery_temperature = g_coul_ops->battery_temperature()/10; 
    } else {
        // 无接口时，默认不充电
        battery_temperature = BQ2419x_COLD_BATTERY_THRESHOLD;
        dev_notice(di->dev, "g_coul_ops->battery_temperature NULL, battery_temperature=%d\n", battery_temperature);        
    }
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 end*/

    if (battery_temperature < BQ2419x_COLD_BATTERY_THRESHOLD) {
        return BATTERY_HEALTH_TEMPERATURE_OVERLOW;

    } else if((battery_temperature >= BQ2419x_COLD_BATTERY_THRESHOLD)
        && (battery_temperature <  BQ2419x_COOL_BATTERY_THRESHOLD)){
        return BATTERY_HEALTH_TEMPERATURE_LOW;

    } else if ((battery_temperature >= BQ2419x_COOL_BATTERY_THRESHOLD)
        && (battery_temperature < BQ2419x_BATTERY_THRESHOLD_5)){
       return BATTERY_HEALTH_TEMPERATURE_5;

    } else if ((battery_temperature >= BQ2419x_BATTERY_THRESHOLD_5)
        && (battery_temperature < BQ2419x_BATTERY_THRESHOLD_15)){
       return BATTERY_HEALTH_TEMPERATURE_15;

    } else if ((battery_temperature >= BQ2419x_BATTERY_THRESHOLD_15)
        && (battery_temperature < japan_temp_cp[di->high_temp_para].cp1)){
       return BATTERY_HEALTH_TEMPERATURE_NORMAL;

    } else if ((battery_temperature >= japan_temp_cp[di->high_temp_para].cp1)
        && (battery_temperature < japan_temp_cp[di->high_temp_para].cp2)){
        return BATTERY_HEALTH_TEMPERATURE_HIGH_CP1;

    } else if ((battery_temperature >= japan_temp_cp[di->high_temp_para].cp2)
        && (battery_temperature < japan_temp_cp[di->high_temp_para].cp3)){
        return BATTERY_HEALTH_TEMPERATURE_HIGH_CP2;

    } else if ((battery_temperature >= japan_temp_cp[di->high_temp_para].cp3)
        && (battery_temperature < japan_temp_cp[di->high_temp_para].cp4)){
        return BATTERY_HEALTH_TEMPERATURE_HIGH_CP3;

    } else if ((battery_temperature >= japan_temp_cp[di->high_temp_para].cp4)
        && (battery_temperature < japan_temp_cp[di->high_temp_para].cp5)){
        return BATTERY_HEALTH_TEMPERATURE_HIGH_CP4;

    } else if ((battery_temperature >= japan_temp_cp[di->high_temp_para].cp5)
        && (battery_temperature < japan_temp_cp[di->high_temp_para].cp6)){
        return BATTERY_HEALTH_TEMPERATURE_HIGH_CP5;

    }else if (battery_temperature >=  japan_temp_cp[di->high_temp_para].cp6){
       return BATTERY_HEALTH_TEMPERATURE_OVERHIGH;

    } else {
       return BATTERY_HEALTH_TEMPERATURE_NORMAL;
    }
}

static int bq2419x_check_battery_temperature_threshold(struct bq2419x_device_info *di)
{
    int battery_temperature = 0;
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 begin*/
    if (g_coul_ops->battery_temperature){
        battery_temperature = g_coul_ops->battery_temperature()/10; 
    } else {
        // 无接口时，默认不充电
        battery_temperature = BQ2419x_COLD_BATTERY_THRESHOLD;
        dev_notice(di->dev, "g_coul_ops->battery_temperature NULL, battery_temperature=%d\n", battery_temperature);         
    }
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 end*/

    if (battery_temperature < di->temperature_cold) {
        return BATTERY_HEALTH_TEMPERATURE_OVERLOW;

    } else if((battery_temperature >= di->temperature_cold)
        && (battery_temperature <  di->temperature_cool)){
        return BATTERY_HEALTH_TEMPERATURE_LOW;

    } else if ((battery_temperature >= di->temperature_cool)
        && (battery_temperature < di->temperature_5)){
       return BATTERY_HEALTH_TEMPERATURE_5;

    } else if ((battery_temperature >= di->temperature_5)
        && (battery_temperature < di->temperature_15)){
       return BATTERY_HEALTH_TEMPERATURE_15;

    } else if ((battery_temperature >= di->temperature_15)
        && (battery_temperature < (di->temperature_warm - BQ2419x_TEMPERATURE_OFFSET))){
       return BATTERY_HEALTH_TEMPERATURE_NORMAL;
 
    } else if ((battery_temperature >= (di->temperature_warm - BQ2419x_TEMPERATURE_OFFSET))
        && (battery_temperature < di->temperature_warm)){
        return BATTERY_HEALTH_TEMPERATURE_NORMAL_HIGH;
	/* < DTS2014030407914 jiangfei 20140304 begin */
    } else if ((battery_temperature >= di->temperature_warm)
        && (battery_temperature < di->temperature_hot - BQ2419x_TEMPERATURE_OFFSET)){
        return BATTERY_HEALTH_TEMPERATURE_HIGH;

    } else if((battery_temperature >= (di->temperature_hot - BQ2419x_TEMPERATURE_OFFSET))
        && (battery_temperature < di->temperature_hot)){
        return BATTERY_HEALTH_TEMPERATURE_HIGH_HOT;
    /* DTS2014030407914 jiangfei 20140304 end > */
    } else if (battery_temperature >= di->temperature_hot){
       return BATTERY_HEALTH_TEMPERATURE_OVERHIGH;

    } else {
       return BATTERY_HEALTH_TEMPERATURE_NORMAL;
    }
}

/* <Begin DTS2014080603839 modified by l00220156 for interface of vm and charger 20140806 > */
/* <DTS2014011005894 chenyuanquan 20140110 begin */
/* get the charge type through SYS_STATUS_REG08 register */
#define BQ2419x_CHGR_STAT_MASK 0x30
int bq2419x_get_charge_type(void)
{
    int rc;
    u8 read_reg = 0;
    struct bq2419x_device_info *di = bq_device_19x;

    rc = bq2419x_read_byte(di, &read_reg, SYS_STATUS_REG08);
    if (rc) {
            pr_err("failed to read SYS_STSTUS_REG08");
            return POWER_SUPPLY_CHARGE_TYPE_NONE;
    }

    if ((read_reg & BQ2419x_CHGR_STAT_MASK) == BQ2419x_CHGR_STAT_PRE_CHARGING)
        return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
    if ((read_reg & BQ2419x_CHGR_STAT_MASK) == BQ2419x_CHGR_STAT_FAST_CHARGING)
        return POWER_SUPPLY_CHARGE_TYPE_FAST;

    return POWER_SUPPLY_CHARGE_TYPE_NONE;
}
/* DTS2014011005894 chenyuanquan 20140110 end> */
EXPORT_SYMBOL(bq2419x_get_charge_type);

int bq2419x_get_charge_status(void)
{
    if (bq_device_19x){
        return bq_device_19x->charge_status;       
    }
    else{
        pr_err("bq_device_19x NULL!!");
        return POWER_SUPPLY_STATUS_NOT_CHARGING;
    }

}
EXPORT_SYMBOL(bq2419x_get_charge_status);
/* <End DTS2014080603839 modified by l00220156 for interface of vm and charger 20140806 > */
/* <DTS2014093003947 (report SOC 100%) caiwei 20141009 begin */
int bq2419x_check_charge_finished(void)
{
    // 放电时不做此判断
    if (bq_device_19x){
        if (POWER_SUPPLY_STATUS_DISCHARGING == bq_device_19x->charge_status){
            return 0;
        }
    }

    if (g_charge_done_count >= CHARGE_DONE_COUNT_MAX){
        return 1;
    }else{
        return 0;
    }
}
EXPORT_SYMBOL(bq2419x_check_charge_finished);
/* DTS2014093003947 (report SOC 100%) caiwei 20141009 end> */
static void bq2419x_monitor_battery_ntc_japan_charging(struct bq2419x_device_info *di)
{
    int battery_status = 0;
    u8 read_reg = 0;
    bq2419x_read_byte(di, &read_reg, SYS_STATUS_REG08);
    if(!di->battery_present)
        return;

    bq2419x_reset_vindpm(di);
    if(!di->limit_charging_flag){
        di->chrg_config = EN_CHARGER & di->factory_flag;
        bq2419x_config_power_on_reg(di);
        if(di->chrg_config){
            di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
        } else {
            di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        }
        if ((read_reg & BQ2419x_CHGR_STAT_CHAEGE_DONE) == BQ2419x_CHGR_STAT_CHAEGE_DONE){
            return;
        }

        do_power_supply_update();
        return;
    }
    
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 begin*/
    if (g_coul_ops->battery_voltage_uv){
        di->battery_voltage = g_coul_ops->battery_voltage_uv(); 
    } else {
        // 无接口时，默认不充电       
        di->battery_voltage = BQ2419x_LOW_TEMP_NOT_CHARGING_VOLTAGE;
        dev_notice(di->dev, "g_coul_ops->battery_voltage_uv NULL, battery_voltage=%d\n", di->battery_voltage); 
    }
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 end*/
    
    battery_status = bq2419x_check_battery_temperature_japan_threshold(di);

    dev_info(di->dev, "%s, \n"
                         "di->battery_voltage = %d \n"
                         "battery_status = %d\n"
                         "di->two_stage_charger_status  = %d \n"
                          , __func__, di->battery_voltage, battery_status,di->two_stage_charger_status);

    switch (battery_status) {
    case BATTERY_HEALTH_TEMPERATURE_OVERLOW:
    case BATTERY_HEALTH_TEMPERATURE_LOW:
        di->chrg_config = DIS_CHARGER | di->coldhot_charging_flag;
        break;

    /* -10 ~ 0 degree is not allowed charging anymore */
    /*
    case BATTERY_HEALTH_TEMPERATURE_LOW:
        if (di->battery_voltage <= BQ2419x_LOW_TEMP_TERM_VOLTAGE)
            di->chrg_config = EN_CHARGER;
        else if (di->battery_voltage > BQ2419x_LOW_TEMP_NOT_CHARGING_VOLTAGE)
            di->chrg_config = DIS_CHARGER;
        else
            di->chrg_config =di->chrg_config;

        if(di->battery_temp_status != battery_status){
            di->currentmA = ICHG_LOW_TEMP;
            bq2419x_config_current_reg(di);
        }
        if (di->is_disable_cool_temperature_charger == 1){
            di->chrg_config = DIS_CHARGER;
        }
        break;
    */
    case BATTERY_HEALTH_TEMPERATURE_5:
        di->chrg_config = EN_CHARGER;
       if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
           if (di->battery_temp_status != battery_status){
               bq2419x_calling_limit_ac_input_current(di,TEMP_CONFIG_5);
               bq2419x_config_input_source_reg(di);
               bq2419x_config_current_reg(di);
           }
        }
        break;

    case BATTERY_HEALTH_TEMPERATURE_15:
        di->chrg_config = EN_CHARGER;
       if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
           if (di->battery_temp_status != battery_status){
               bq2419x_calling_limit_ac_input_current(di,TEMP_CONFIG_15);
               bq2419x_config_input_source_reg(di);
               bq2419x_config_current_reg(di);
           }
        }
        break;

    case BATTERY_HEALTH_TEMPERATURE_NORMAL:
        di->chrg_config = EN_CHARGER;
        if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
            if((di->is_two_stage_charger== 1)
                &&( (di->battery_voltage >= di->first_stage_voltage)
                        || ((di->battery_voltage >= di->second_stage_voltage)&&(di->two_stage_charger_status  == TWO_STAGE_CHARGE_SECOND_STAGE)))){ 
                 bq2419x_calling_limit_ac_input_current(di,NORMAL_SECOND_STAGE);
                 di->two_stage_charger_status  = TWO_STAGE_CHARGE_SECOND_STAGE;
            }else if(di->battery_voltage > BQ2419x_NORNAL_ICHRG_VOLTAGE){
                 bq2419x_calling_limit_ac_input_current(di,NORMAL_TEMP_CONFIG);
                 di->two_stage_charger_status  = TWO_STAGE_CHARGE_FIRST_STAGE;
            } else {
                 di->currentmA = ICHG_820;
            }
            bq2419x_config_input_source_reg(di);
            bq2419x_config_current_reg(di);
       }
        break;

    case BATTERY_HEALTH_TEMPERATURE_HIGH_CP1:
        di->chrg_config = EN_CHARGER;
        if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
            if (di->battery_temp_status != battery_status){
                bq2419x_calling_limit_ac_input_current(di,NORMAL_HIGH_TEMP_CONFIG_CP1);
                bq2419x_config_input_source_reg(di);
                bq2419x_config_current_reg(di);
            }
        }
        break;

     case BATTERY_HEALTH_TEMPERATURE_HIGH_CP2:
        di->chrg_config = EN_CHARGER;
        if(di->charger_source == POWER_SUPPLY_TYPE_MAINS){
            if (di->battery_temp_status != battery_status){
                bq2419x_calling_limit_ac_input_current(di,NORMAL_HIGH_TEMP_CONFIG_CP2);
                bq2419x_config_input_source_reg(di);
                bq2419x_config_current_reg(di);
            }
        }
        break;

      case BATTERY_HEALTH_TEMPERATURE_HIGH_CP3:
        di->chrg_config = EN_CHARGER;
        if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
            if (di->battery_temp_status != battery_status){
                bq2419x_calling_limit_ac_input_current(di,NORMAL_HIGH_TEMP_CONFIG_CP3);
                bq2419x_config_input_source_reg(di);
                bq2419x_config_current_reg(di);
            }
        }
        break;

     case BATTERY_HEALTH_TEMPERATURE_HIGH_CP4:
        di->chrg_config = EN_CHARGER;
        if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
            if (di->battery_temp_status != battery_status){
                bq2419x_calling_limit_ac_input_current(di,NORMAL_HIGH_TEMP_CONFIG_CP4);
                bq2419x_config_input_source_reg(di);
                bq2419x_config_current_reg(di);
            }
        }
        break;

     case BATTERY_HEALTH_TEMPERATURE_HIGH_CP5:
        di->chrg_config = di->chrg_config;
        if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
            if ((di->battery_temp_status != battery_status) && (di->chrg_config == EN_CHARGER)){
                bq2419x_calling_limit_ac_input_current(di,NORMAL_HIGH_TEMP_CONFIG_CP4);
                bq2419x_config_input_source_reg(di);
                bq2419x_config_current_reg(di);
            }
        }
        break;

    case BATTERY_HEALTH_TEMPERATURE_OVERHIGH:
        di->chrg_config = DIS_CHARGER | di->coldhot_charging_flag;
        break;
    default:
        break;
    }

    di->chrg_config = di->chrg_config & di->factory_flag;
    if(di->chrg_config){
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    } else {
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
    bq2419x_config_power_on_reg(di);
    di->battery_temp_status = battery_status;
    if ((read_reg & BQ2419x_CHGR_STAT_CHAEGE_DONE) == BQ2419x_CHGR_STAT_CHAEGE_DONE){
       return;
    }

    do_power_supply_update();
    return;
}
/* < DTS2014011515091 yuanzhen 20140115 begin*/
static void bq2419x_monitor_battery_ntc_charging(struct bq2419x_device_info *di)
{
    int battery_status = 0;
    u8 read_reg = 0;
    bq2419x_read_byte(di, &read_reg, SYS_STATUS_REG08);
    if(!di->battery_present)
        return;

    bq2419x_reset_vindpm(di);
    if(!di->limit_charging_flag){
        di->chrg_config = EN_CHARGER & di->factory_flag;
        bq2419x_config_power_on_reg(di);
        if(di->chrg_config){
            di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
        } else {
            di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        }
        if ((read_reg & BQ2419x_CHGR_STAT_CHAEGE_DONE) == BQ2419x_CHGR_STAT_CHAEGE_DONE){
            return;
        }

        do_power_supply_update();
        return;
    }
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 begin*/
    if (g_coul_ops->battery_voltage_uv){
        di->battery_voltage = g_coul_ops->battery_voltage_uv(); 
    } else {
        // 无接口时，默认不充电       
        di->battery_voltage = BQ2419x_LOW_TEMP_NOT_CHARGING_VOLTAGE;
        dev_notice(di->dev, "g_coul_ops->battery_voltage NULL, battery_voltage=%d\n", di->battery_voltage); 
    }
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 end*/
    
    battery_status = bq2419x_check_battery_temperature_threshold(di);

    switch (battery_status) {
    case BATTERY_HEALTH_TEMPERATURE_OVERLOW:
    case BATTERY_HEALTH_TEMPERATURE_LOW:
        di->chrg_config = DIS_CHARGER | di->coldhot_charging_flag;
        break;

    /* -10 ~ 0 degree is not allowed charging anymore */
    /*
    case BATTERY_HEALTH_TEMPERATURE_LOW:
        if (di->battery_voltage <= BQ2419x_LOW_TEMP_TERM_VOLTAGE)
            di->chrg_config = EN_CHARGER;
        else if (di->battery_voltage > BQ2419x_LOW_TEMP_NOT_CHARGING_VOLTAGE)
            di->chrg_config = DIS_CHARGER;
        else
            di->chrg_config =di->chrg_config;

        if(di->battery_temp_status != battery_status){
            di->currentmA = ICHG_LOW_TEMP;
            bq2419x_config_current_reg(di);
       }
        break;
    */
    case BATTERY_HEALTH_TEMPERATURE_5:
        di->chrg_config = EN_CHARGER;
        di->voltagemV = di->max_voltagemV;
        if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
           if (di->battery_temp_status != battery_status){
               bq2419x_calling_limit_ac_input_current(di,TEMP_CONFIG_5);
               bq2419x_config_input_source_reg(di);
               bq2419x_config_current_reg(di);
           }
        }
        break;
    case BATTERY_HEALTH_TEMPERATURE_15:
        di->chrg_config = EN_CHARGER;
        di->voltagemV = di->max_voltagemV;
        if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
           if (di->battery_temp_status != battery_status){
               bq2419x_calling_limit_ac_input_current(di,TEMP_CONFIG_15);
               bq2419x_config_input_source_reg(di);
               bq2419x_config_current_reg(di);
           }
        }
        break;
    case BATTERY_HEALTH_TEMPERATURE_NORMAL:
        di->chrg_config = EN_CHARGER;
        di->voltagemV = di->max_voltagemV;
        if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
            if(di->battery_voltage > BQ2419x_NORNAL_ICHRG_VOLTAGE){
                 bq2419x_calling_limit_ac_input_current(di,NORMAL_TEMP_CONFIG);
            } else {
                 di->currentmA = ICHG_820;
            }
            bq2419x_config_input_source_reg(di);
            bq2419x_config_current_reg(di);
       }
        break;
    case BATTERY_HEALTH_TEMPERATURE_NORMAL_HIGH:
	/* <DTS2014093003947 (40~45 degree charge current 1A) caiwei 20141009 begin */
        if (!di->hw_is_pad) {
            di->chrg_config = EN_CHARGER;
            if(di->coldhot_charging_flag)
            {
                di->voltagemV = di->max_voltagemV;
            }
            if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
                if (di->battery_temp_status != battery_status){
                    bq2419x_calling_limit_ac_input_current(di,NORMAL_HIGH_TEMP_CONFIG);
                    bq2419x_config_input_source_reg(di);
                    bq2419x_config_current_reg(di);
                }
            }
            break;
        }
    case BATTERY_HEALTH_TEMPERATURE_HIGH:
        if (!di->hw_is_pad) {
            di->chrg_config = EN_CHARGER;
            if(!di->coldhot_charging_flag)
            {
                /*in high temperature, the terminal voltage should be lower then normal*/
                di->voltagemV = di->max_voltagemV - BQ2419x_HIGH_TEMP_DELTA;
            }
            else
            {
                di->voltagemV = di->max_voltagemV;
            }
            if(di->charger_source == POWER_SUPPLY_TYPE_MAINS){
                if (di->battery_temp_status != battery_status){
                    bq2419x_calling_limit_ac_input_current(di,HIGH_TEMP_CONFIG);
                    bq2419x_config_input_source_reg(di);
                    bq2419x_config_current_reg(di);
                }
            }
            break;
        }
     /* < DTS2014030407914 jiangfei 20140304 begin */
    case BATTERY_HEALTH_TEMPERATURE_HIGH_HOT:
    if (!di->hw_is_pad) {
	     if(!di->coldhot_charging_flag)
	     {
            /*in high temperature, the terminal voltage should be lower then normal*/
            di->voltagemV = di->max_voltagemV - BQ2419x_HIGH_TEMP_DELTA;
        }
        else
        {
            di->voltagemV = di->max_voltagemV;
        }
        if(di->charger_source == POWER_SUPPLY_TYPE_MAINS){
            if (di->battery_temp_status != battery_status){
                bq2419x_calling_limit_ac_input_current(di,HIGH_TEMP_CONFIG);
                bq2419x_config_input_source_reg(di);
                bq2419x_config_current_reg(di);
            }
        }
        break;
    } else {
        di->chrg_config = EN_CHARGER;
        di->voltagemV = di->max_voltagemV;
        if (di->charger_source == POWER_SUPPLY_TYPE_MAINS){
            if(di->battery_voltage > BQ2419x_NORNAL_ICHRG_VOLTAGE){
                 bq2419x_calling_limit_ac_input_current(di,NORMAL_TEMP_CONFIG);
            } else {
                 di->currentmA = ICHG_820;
            }
            bq2419x_config_input_source_reg(di);
            bq2419x_config_current_reg(di);
        }
         break;
    }
	/* DTS2014093003947 (40~45 degree charge current 1A) caiwei 20141009 end> */
    /* DTS2014030407914 jiangfei 20140304 end > */
    case BATTERY_HEALTH_TEMPERATURE_OVERHIGH:
        di->chrg_config = DIS_CHARGER | di->coldhot_charging_flag;
        break;
    default:
        break;
    }

    bq2419x_config_voltage_reg(di);

    di->chrg_config = di->chrg_config & di->factory_flag;
    /* < DTS2014021400603 yuanzhen 20140218 begin */
    /* if set hz_mode, the device is in discharge mode */
    if(di->chrg_config){
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    } else if(di->hz_mode == EN_HIZ){
        di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
    } else {
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
    /* DTS2014021400603 yuanzhen 20140218 end > */
    bq2419x_config_power_on_reg(di);
    di->battery_temp_status = battery_status;
    if ((read_reg & BQ2419x_CHGR_STAT_CHAEGE_DONE) == BQ2419x_CHGR_STAT_CHAEGE_DONE){
       return;
    }

    do_power_supply_update();
    return;
}
/* DTS2014011515091 yuanzhen 20140115 end > */
static void bq2419x_start_usb_charger(struct bq2419x_device_info *di)
{
    /* <DTS2014111002912 caiwei 20141112 begin */
    last_otg_status = 0;
    /* DTS2014111002912 caiwei 20141112 end> */
    di->wakelock_enabled = 1;
    if (di->wakelock_enabled){
        wake_lock(&chrg_lock);
    }
	
/* DTS2014092405474 caiwei 20140926 delete 7 lines */

    di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    do_power_supply_update();
    di->charger_source = POWER_SUPPLY_TYPE_USB;
    di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    bq2419x_get_max_charge_voltage(di);
/* DTS2014120200663 xiongxi xwx234328 20141210 begin */
/*<DTS2014121908780 xiongxi xwx234328 20141231 begin*/
    di->cin_dpmmV = VINDPM_4520;
/*<DTS2014121908780 xiongxi xwx234328 20141231 end>*/
/* DTS2014120200663 xiongxi xwx234328 20141210 end */
    di->cin_limit = IINLIM_500;

    di->voltagemV = di->max_voltagemV;
    di->currentmA = ICHG_MIN ;
    di->term_currentmA = ITERM_MIN_128;
    di->watchdog_timer = WATCHDOG_40;
    di->chrg_timer = CHG_TIMER_12;

    
    di->hz_mode = DIS_HIZ;
    /*DTS2014122906101 xiongxi xwx234328 20150115 begin*/
    /*<DTS2014122906101 xiongxi xwx234328 20150126 begin*/
    di->chrg_config = EN_CHARGER;
    /*DTS2014122906101 xiongxi xwx234328 20150126 end>*/
    /*DTS2014122906101 xiongxi xwx234328 20150115 end*/
    di->boost_lim = BOOST_LIM_500;
    di->enable_low_chg = DIS_FORCE_20PCT;
    di->enable_iterm = DIS_TERM;
    di->enable_timer = EN_TIMER;
    di->enable_batfet = EN_BATFET;
    di->factory_flag = EN_CHARGER;
    di->calling_limit = 0;
    di->battery_temp_status = -1;
    /* <DTS2015010804547 caiwei 20150112 begin */
    di->enable_dpdm = DPDM_DIS;
    /* DTS2015010804547 caiwei 20150112 end> */
    di->charge_full_count = 0;

    bq2419x_config_power_on_reg(di);
    /* <DTS2015010804597 caiwei 20150112 begin */
    bq2419x_config_input_source_reg(di);
    msleep(500);
    /* DTS2015010804597 caiwei 20150112 end> */
    bq2419x_config_current_reg(di);
    bq2419x_config_prechg_term_current_reg(di);
    bq2419x_config_voltage_reg(di);
    bq2419x_config_term_timer_reg(di);
    bq2419x_config_thernal_regulation_reg(di);
    bq2419x_config_misc_operation_reg(di);
/* <DTS2015010804438 caiwei 20150112 begin */
/* <DTS2014092405474 caiwei 20140926 begin */
    if(BOARDID_T1_10 != di->hw_board_id)
         gpio_set_value(di->gpio, 1);
    else
         gpio_set_value(di->gpio, 0);
/* DTS2014092405474 caiwei 20140926 end> */
/* DTS2015010804438 caiwei 20150112 end> */

    /*DTS2014122906101 xiongxi xwx234328 20150115 begin*/
    /*<DTS2014122906101 xiongxi xwx234328 20150126 begin*/
    //delete by xiongxi
    /*DTS2014122906101 xiongxi xwx234328 20150126 end>*/
    /*DTS2014122906101 xiongxi xwx234328 20150115 end*/
    schedule_delayed_work(&di->bq2419x_charger_work, msecs_to_jiffies(0));

    dev_info(di->dev, "%s, ---->START USB CHARGING, \n"
                         "battery current = %d mA\n"
                         "battery voltage = %d mV\n"
                         "cin_limit_current = %d mA\n"
                         , __func__, di->currentmA, di->voltagemV,di->cin_limit);
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 begin*/
    if (g_coul_ops->is_battery_exist){
        di->battery_present = g_coul_ops->is_battery_exist(); 
    } else {
        // 无接口时，默认在位       
        di->battery_present = 1;
        dev_notice(di->dev, "g_coul_ops->is_battery_exist NULL, battery_present=%d\n", di->battery_present); 
    }
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 end*/
    
    if (!di->battery_present) {
            dev_dbg(di->dev, "BATTERY NOT DETECTED!\n");
            di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            do_power_supply_update();
    }

    return;
}

static void bq2419x_start_ac_charger(struct bq2419x_device_info *di)
{
    /* <DTS2014111002912 caiwei 20141112 begin */
    last_otg_status = 0;
    /* DTS2014111002912 caiwei 20141112 end> */
    di->wakelock_enabled = 1;
    if (di->wakelock_enabled){
        wake_lock(&chrg_lock);
    }

/* DTS2014092405474 caiwei 20140926 delete 7 lines */

    di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    do_power_supply_update();
    di->charger_source = POWER_SUPPLY_TYPE_MAINS;
    di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    bq2419x_get_max_charge_voltage(di);
	/* DTS2014120200663 xiongxi xwx234328 20141210 begin */
/*<DTS2014121908780 xiongxi xwx234328 20141231 begin*/
    di->cin_dpmmV = VINDPM_4520;
/*<DTS2014121908780 xiongxi xwx234328 20141231 end>*/
	/* DTS2014120200663 xiongxi xwx234328 20141210 end */
    di->cin_limit = di->max_cin_currentmA;
    di->voltagemV = di->max_voltagemV;
    di->currentmA = di->max_currentmA;
    di->term_currentmA = ITERM_MIN_128;
    di->watchdog_timer = WATCHDOG_40;
    di->chrg_timer = CHG_TIMER_8;

    di->hz_mode = DIS_HIZ;
    /*DTS2014122906101 xiongxi xwx234328 20150116 begin*/
    /*<DTS2014122906101 xiongxi xwx234328 20150126 begin*/
    di->chrg_config = EN_CHARGER;
    /*DTS2014122906101 xiongxi xwx234328 20150126 end>*/
    /*DTS2014122906101 xiongxi xwx234328 20150116 end*/
    di->boost_lim = BOOST_LIM_500;
    di->enable_low_chg = DIS_FORCE_20PCT;
    di->enable_iterm = DIS_TERM;
    di->enable_timer = EN_TIMER;
    di->enable_batfet = EN_BATFET;
    di->factory_flag = EN_CHARGER;
    di->enable_dpdm = DPDM_DIS;
    di->calling_limit = 0;
    di->battery_temp_status = -1;
    di->charge_full_count = 0;

    bq2419x_config_power_on_reg(di);
    msleep(500);
    bq2419x_config_input_source_reg(di);
    bq2419x_config_current_reg(di);
    bq2419x_config_prechg_term_current_reg(di);
    bq2419x_config_voltage_reg(di);
    bq2419x_config_term_timer_reg(di);
    bq2419x_config_thernal_regulation_reg(di);
    bq2419x_config_misc_operation_reg(di);
/* <DTS2015010804438 caiwei 20150112 begin */
/* <DTS2014092405474 caiwei 20140926 begin */
    if(BOARDID_T1_10 != di->hw_board_id)
         gpio_set_value(di->gpio, 1);
    else
         gpio_set_value(di->gpio, 0);
/* DTS2014092405474 caiwei 20140926 end> */
/* DTS2015010804438 caiwei 20150112 end> */

    /*DTS2014122906101 xiongxi xwx234328 20150115 begin*/
    /*<DTS2014122906101 xiongxi xwx234328 20150126 begin*/
    //delete by xiongxi
    /*DTS2014122906101 xiongxi xwx234328 20150126 end>*/
    /*DTS2014122906101 xiongxi xwx234328 20150115 end*/
    schedule_delayed_work(&di->bq2419x_charger_work, msecs_to_jiffies(0));

    dev_info(di->dev, "%s, ---->START AC CHARGING, \n"
                         "battery current = %d mA\n"
                         "battery voltage = %d mV\n"
                         "cin_limit_current = %d mA\n"
                         , __func__, di->currentmA, di->voltagemV,di->cin_limit);

    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 begin*/
    if (g_coul_ops->is_battery_exist){
        di->battery_present = g_coul_ops->is_battery_exist(); 
    } else {
        // 无接口时，默认在位       
        di->battery_present = 1;
        dev_notice(di->dev, "g_coul_ops->is_battery_exist NULL, battery_present=%d\n", di->battery_present); 
    }
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 end*/
    
    if (!di->battery_present) {
            dev_dbg(di->dev, "BATTERY NOT DETECTED!\n");
            di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
            do_power_supply_update();
    }

    return;
}

static void bq2419x_start_usb_otg(struct bq2419x_device_info *di)
{
    dev_info(di->dev, "%s,---->USB_EVENT_OTG_ID<----\n", __func__);
    /* <DTS2014102505836 caiwei 20141030 begin */
    do_power_supply_update_otg();
    /* <DTS2014111002912 caiwei 20141112 begin */
    last_otg_status = 1;
    /* DTS2014111002912 caiwei 20141112 end> */
    /* DTS2014102505836 caiwei 20141030 end> */
    if(di->charger_source != POWER_SUPPLY_TYPE_BATTERY){
/* <DTS2014092405474 caiwei 20140926 begin */
       if (di->hw_board_id != BOARDID_T1_10)
      /* <DTS2014102005025 caiwei 20141021 begin */
            gpio_set_value(di->gpio, 1);
       else
            gpio_set_value(di->gpio, 0);
      /* DTS2014102005025 caiwei 20141021 end> */
/* DTS2014092405474 caiwei 20140926 end> */
        return;
    }

    di->wakelock_enabled = 1;
    if (di->wakelock_enabled){
        wake_lock(&chrg_lock);
    }
/* <DTS2014092405474 caiwei 20140926 begin */
    if (di->hw_board_id != BOARDID_T1_10)
    /* <DTS2014102005025 caiwei 20141021 begin */
         gpio_set_value(di->gpio, 0);
    else
         gpio_set_value(di->gpio, 1);
    /* DTS2014102005025 caiwei 20141021 end> */
/* DTS2014092405474 caiwei 20140926 end> */

    di->hz_mode = DIS_HIZ;
    di->chrg_config = EN_CHARGER_OTG;
   /* <DTS2014102005025 caiwei 20141021 begin */
    di->boost_lim = BOOST_LIM_1300;
   /* DTS2014102005025 caiwei 20141021 end> */

    bq2419x_config_power_on_reg(di);
    bq2419x_config_input_source_reg(di);
    /* < DTS2014031501494 yuanzhen 20140317 begin */
    if(irq_int_active == 0) {
        enable_irq(di->irq_int);
        irq_int_active = 1;
    }
    /* DTS2014031501494 yuanzhen 20140317 end > */
    schedule_delayed_work(&di->bq2419x_usb_otg_work, msecs_to_jiffies(0));

    return;
}

static void bq2419x_stop_charger(struct bq2419x_device_info *di)
{
    if (!wake_lock_active(&chrg_lock)){
        wake_lock(&chrg_lock);
    }

    /*set gpio_074 high level for CE pin to disable bq2419x IC */
/* <DTS2014092405474 caiwei 20140926 begin */
    if (di->hw_board_id != BOARDID_T1_10)
         gpio_set_value(di->gpio, 0);
    else
         gpio_set_value(di->gpio, 1);
/* DTS2014092405474 caiwei 20140926 end> */
    /* < DTS2014031501494 yuanzhen 20140317 begin */
    if(irq_int_active == 1) {
        disable_irq(di->irq_int);
        irq_int_active = 0;
    }
    /* DTS2014031501494 yuanzhen 20140317 end > */

    cancel_delayed_work_sync(&di->bq2419x_charger_work);
    
    di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	/*< DTS2015011001210  yesiping 20150123 begin */

    dev_info(di->dev,"%s,---->STOP CHARGING\n", __func__);
    di->charger_source = POWER_SUPPLY_TYPE_BATTERY;
    di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;

    di->enable_batfet = EN_BATFET;
    di->calling_limit = 0;
    di->factory_flag = 0;
    di->hz_mode = DIS_HIZ;
    di->battery_temp_status = 0;
    di->chrg_config = DIS_CHARGER;
    di->enable_dpdm = DPDM_DIS;
    di->charge_full_count = 0;
    /*<DTS2014122906101 xiongxi xwx234328 20150126 begin*/
    di->cin_dpmmV = VINDPM_4680;
    /*DTS2014122906101 xiongxi xwx234328 20150126 end>*/
    /*<DTS2015020507957 xiongxi xwx234328 20150205 begin*/
    di->watchdog_timer = WATCHDOG_DIS;
    /* DTS2015020507957 xiongxi xwx234328 20150205 end>*/
    //delete by yesiping
    bq2419x_config_power_on_reg(di);
    bq2419x_config_input_source_reg(di);
    bq2419x_config_misc_operation_reg(di);
    /*<DTS2015020507957 xiongxi xwx234328 20150205 begin*/
    bq2419x_config_term_timer_reg(di);
    /* DTS2015020507957 xiongxi xwx234328 20150205 end>*/
    do_power_supply_update();
    /* <DTS2014122706448 xiongxi xwx234328 20150202 begin*/
    do_power_supply_update_otg();
    /* DTS2014122706448 xiongxi xwx234328 20150202 end>*/
	/* DTS2015011001210  yesiping 20150123 end >*/
    cancel_delayed_work_sync(&di->bq2419x_usb_otg_work);

    /*set gpio_074 high level for CE pin to disable bq2419x IC */
/* <DTS2014092405474 caiwei 20140926 begin */
    if (di->hw_board_id != BOARDID_T1_10)
         gpio_set_value(di->gpio, 0);
    else
         gpio_set_value(di->gpio, 1);
/* DTS2014092405474 caiwei 20140926 end> */
    msleep(1000);

    di->wakelock_enabled = 1;
    if (di->wakelock_enabled)
        wake_unlock(&chrg_lock);

    wakeup_timer_seconds = 0;
   return;
}

static void bq2419x_check_bq27510_charge_full(struct bq2419x_device_info *di)
{
    if(di->battery_present){

        if(di->battery_full){
            di->enable_iterm = EN_TERM;
            di->charge_full_count++;
            if(di->charge_full_count >= 20){
               di->charge_full_count = 20;
            }
        }else{
            di->enable_iterm = DIS_TERM;
        }
    }else{
        di->enable_iterm = EN_TERM;
    }
    /* DTS2014081800811 by l00220156 begin*/
    di->enable_iterm = EN_TERM;
    /* DTS2014081800811 by l00220156 end*/
        
    bq2419x_config_term_timer_reg(di);

    return;
}

/* DTS2014081800811 by l00220156 begin*/
#if 0
static void bq2419x_charger_done_release_wakelock(struct bq2419x_device_info *di)
{
    if(di->charger_source == POWER_SUPPLY_TYPE_MAINS){
        if(!di->battery_present)
            return;
        if(di->battery_full){
            if (wake_lock_active(&chrg_lock)){
                if(di->charge_full_count >= 20){
                    wake_unlock(&chrg_lock);
                    dev_err(di->dev, "ac charge done wakelock release\n");
                }
            }
        }else{
            if (!wake_lock_active(&chrg_lock)){
                wake_lock(&chrg_lock);
                dev_err(di->dev, "ac recharge wakelock add again\n");
            }
        }
    }
    return;
}
#endif
/* DTS2014081800811 by l00220156 end*/

static void bq2419x_config_status_reg(struct bq2419x_device_info *di)
{
    di->power_on_config_reg01 = di->power_on_config_reg01 |WATCHDOG_TIMER_RST;
    bq2419x_write_byte(di, di->power_on_config_reg01, POWER_ON_CONFIG_REG01);
    return;
}

static void
bq2419x_charger_update_status(struct bq2419x_device_info *di)
{
    u8 read_reg[11] = {0};

    di->timer_fault = 0;
    bq2419x_read_block(di, &read_reg[0], 0, 11);
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 begin*/
    bq2419x_read_byte(di, &read_reg[9], CHARGER_FAULT_REG09);
    msleep(700);
    bq2419x_read_byte(di, &read_reg[9], CHARGER_FAULT_REG09);
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 end*/    
    
    if ((read_reg[8] & BQ2419x_CHGR_STAT_CHAEGE_DONE) == BQ2419x_CHGR_STAT_CHAEGE_DONE){
        /*DTS2014091301574,begin:rm linear charger, 2014.9.30*/
		dev_dbg(di->dev, "CHARGE DONE\n");
		/*DTS2014091301574,end:rm linear charger, 2014.9.30*/
        //while battery is not present, the register show charging done.
        //avoid the issue, add condition here
        if(!di->battery_present)
        {
            di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        }
        else
        {
            if(di->battery_full)
                di->charge_status = POWER_SUPPLY_STATUS_FULL;
        }
/* <DTS2014093003947 (report SOC 100%) caiwei 20141009 begin */
        g_charge_done_count ++;
        // 防止溢出了
        if (g_charge_done_count > (CHARGE_DONE_COUNT_MAX *10)){
            g_charge_done_count = CHARGE_DONE_COUNT_MAX + 1;
        }
        do_power_supply_update();
    }
/* DTS2014093003947 (report SOC 100%) caiwei 20141009 end> */
    if ((read_reg[8] & BQ2419x_PG_STAT) == BQ2419x_NOT_PG_STAT){
        di->cfg_params = 1;
        dev_info(di->dev, "not power good\n");
    }

    if ((read_reg[9] & BQ2419x_POWER_SUPPLY_OVP) == BQ2419x_POWER_SUPPLY_OVP){
        dev_err(di->dev, "POWER_SUPPLY_OVERVOLTAGE = %x\n", read_reg[9]);
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        do_power_supply_update();
    }

    if ((read_reg[9] & BQ2419x_WATCHDOG_FAULT) == BQ2419x_WATCHDOG_FAULT)
        di->timer_fault = 1;

    if (read_reg[9] & 0xFF) {
        di->cfg_params = 1;
        dev_err(di->dev, "CHARGER STATUS %x\n", read_reg[9]);
    }

    if ((read_reg[9] & BQ2419x_BAT_FAULT_OVP) == BQ2419x_BAT_FAULT_OVP) {
        di->hz_mode = EN_HIZ;
        bq2419x_config_input_source_reg(di);
        bq2419x_write_byte(di, di->charge_voltage_reg04, CHARGE_VOLTAGE_REG04);
        dev_err(di->dev, "BATTERY OVP = %x\n", read_reg[9]);
        msleep(700);
        di->hz_mode = DIS_HIZ;
        bq2419x_config_input_source_reg(di);
    }

    if ((di->timer_fault == 1) || (di->cfg_params == 1)) {
        bq2419x_write_byte(di, di->input_source_reg00, INPUT_SOURCE_REG00);
        bq2419x_write_byte(di, di->power_on_config_reg01, POWER_ON_CONFIG_REG01);
        bq2419x_write_byte(di, di->charge_current_reg02, CHARGE_CURRENT_REG02);
        bq2419x_write_byte(di, di->prechrg_term_current_reg03, PRECHARGE_TERM_CURRENT_REG03);
        bq2419x_write_byte(di, di->charge_voltage_reg04, CHARGE_VOLTAGE_REG04);
        bq2419x_write_byte(di, di->term_timer_reg05, CHARGE_TERM_TIMER_REG05);
        bq2419x_write_byte(di, di->thermal_regulation_reg06, THERMAL_REGUALTION_REG06);
        di->cfg_params = 0;
    }

    /* reset 30 second timer */
    bq2419x_config_status_reg(di);
}
/* <DTS2015011311389 caiwei 20150122 begin */
/**********************************************************
*  Function:       bq2419x_reset_otg
*  Discription:    set the otg mode disable and enable
*  Parameters:   void
*  return value:  0-sucess or others-fail
**********************************************************/
static void bq2419x_reset_otg(struct bq2419x_device_info *di)
{

    if (BOARDID_T1_10 != di->hw_board_id)
         gpio_set_value(di->gpio, 0);
    else
         gpio_set_value(di->gpio, 1);
    /* turn off OTG */
    di->chrg_config = (DIS_CHARGER) << 1;
    bq2419x_config_power_on_reg(di);
    msleep(100);
    if (BOARDID_T1_10 != di->hw_board_id)
         gpio_set_value(di->gpio, 1);
    else
         gpio_set_value(di->gpio, 0);
    /* turn on OTG */
    di->chrg_config = EN_CHARGER_OTG;
    bq2419x_config_power_on_reg(di);
}
/* DTS2015011311389 caiwei 20150122 end> */
static void bq2419x_charger_work(struct work_struct *work)
{
    struct bq2419x_device_info *di = container_of(work,
                 struct bq2419x_device_info, bq2419x_charger_work.work);
    
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 begin*/
    if (g_coul_ops->is_battery_exist){
        di->battery_present = g_coul_ops->is_battery_exist(); 
    } else {
        // 无接口时，默认在位       
        di->battery_present = 1;
        dev_notice(di->dev, "g_coul_ops->is_battery_exist NULL, battery_present=%d\n", di->battery_present); 
    }

    if (g_coul_ops->check_eoc_condition){
        g_coul_ops->check_eoc_condition();
    } else {
        dev_notice(di->dev, "g_coul_ops->check_eoc_condition NULL");
    }
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 end*/

    if(di->japan_charger){
        bq2419x_monitor_battery_ntc_japan_charging(di);
    }
    else{
        bq2419x_monitor_battery_ntc_charging(di);
    }

    bq2419x_check_bq27510_charge_full(di);

    bq2419x_charger_update_status(di);
    
    /* DTS2014081800811 by l00220156 begin*/
    //bq2419x_charger_done_release_wakelock(di);
    /* DTS2014081800811 by l00220156 end*/

    schedule_delayed_work(&di->bq2419x_charger_work,
                 msecs_to_jiffies(BQ2419x_WATCHDOG_TIMEOUT));
}

/* < DTS2014031501494 yuanzhen 20140317 begin */
static void bq2419x_otg_int_work(struct work_struct *work)
{
    struct bq2419x_device_info *di = container_of(work,struct bq2419x_device_info, otg_int_work.work);
    u8 read_reg = 0;
    u8 init_read_reg = 0;

    if(di->event != USB_EVENT_OTG_ID)
        return;
    dev_info(di->dev, "bq2419x_otg_int_work\n");
    /* <DTS2015011311389 caiwei 20150122 begin */
    msleep(100);
    /* DTS2015011311389 caiwei 20150122 end> */
    bq2419x_read_byte(di, &init_read_reg, CHARGER_FAULT_REG09);
    /* <DTS2015011311389 caiwei 20150122 begin */
    // delete 1 line
    /* DTS2015011311389 caiwei 20150122 end> */
    bq2419x_read_byte(di, &read_reg, CHARGER_FAULT_REG09);

    if (init_read_reg != read_reg){
        dev_info(di->dev, "reg09=0x%x first, then=0x%x\n", init_read_reg, read_reg);
    }
    /* <DTS2015011311389 caiwei 20150122 begin */
#if 0
    if (di->otg_int_work_cnt == OTG_INT_WORK_CNT){
        dev_info(di->dev, "OTG_INT reg09=0x%x\n", read_reg);
    }
#endif
    /* DTS2015011311389 caiwei 20150122 end> */
    if(read_reg & BQ2419x_OTG_FAULT){
    /* <DTS2015011311389 caiwei 20150122 begin */
        bq2419x_reset_otg(di);
        msleep(100);

        bq2419x_read_byte(di, &read_reg, CHARGER_FAULT_REG09);
        bq2419x_read_byte(di, &read_reg, CHARGER_FAULT_REG09);
        if (read_reg & BQ2419x_OTG_FAULT) {
            dev_info(di->dev, "CHARGE_FAULT_REG09:OTG_FAULT twice!! \n");
	      if (BOARDID_T1_10 != di->hw_board_id)
                    gpio_set_value(di->gpio, 1);
               else
                    gpio_set_value(di->gpio, 0);
              /* turn off OTG */
               di->chrg_config = (DIS_CHARGER) << 1;
               bq2419x_config_power_on_reg(di);
               return;
        }
     /* DTS2015011311389 caiwei 20150122 end> */
    }
    /* <DTS2015011311389 caiwei 20150122 begin */
#if 0
    if (di->otg_int_work_cnt){
        di->otg_int_work_cnt --;
        schedule_delayed_work(&di->otg_int_work, msecs_to_jiffies(OTG_INT_WORK_TIMER));
    }
#endif
    if(irq_int_active == 0) {
        enable_irq(di->irq_int);
        irq_int_active = 1;
    }
     /* DTS2015011311389 caiwei 20150122 end> */
}

static irqreturn_t bq2419x_irq_int_interrupt(int irq, void *_di)
{
    struct bq2419x_device_info *di = _di;
    di->otg_int_work_cnt = 2;
    dev_err(di->dev, "OTG interrupt\n");

    schedule_delayed_work(&di->otg_int_work,0);
    return IRQ_HANDLED;
}
/* DTS2014031501494 yuanzhen 20140317 end > */

static void bq2419x_usb_otg_work(struct work_struct *work)
{
    struct bq2419x_device_info *di = container_of(work,
    struct bq2419x_device_info, bq2419x_usb_otg_work.work);
/* <DTS2014092405474 caiwei 20140926 begin */
    u8 read_reg = 0;

    bq2419x_read_byte(di, &read_reg, CHARGER_FAULT_REG09);
    msleep(700);
    bq2419x_read_byte(di, &read_reg, CHARGER_FAULT_REG09);
    if(read_reg & BQ2419x_OTG_FAULT){
/* DTS2014092405474 caiwei 20140926 end> */
        dev_err(di->dev, "VBUS overloaded in OTG, or VBUS OVP = %x\n", read_reg);
     }

        /* reset 30 second timer */
    bq2419x_config_power_on_reg(di);

    schedule_delayed_work(&di->bq2419x_usb_otg_work,
                 msecs_to_jiffies(BQ2419x_WATCHDOG_TIMEOUT));
}

static void bq2419x_usb_charger_work(struct work_struct *work)
{
    struct bq2419x_device_info	*di =
              container_of(work, struct bq2419x_device_info, usb_work);

    switch (di->event) {
    case CHARGER_TYPE_USB:
        dev_info(di->dev, "case = CHARGER_TYPE_USB-> \n");
        bq2419x_start_usb_charger(di);
        break;
    case CHARGER_TYPE_NON_STANDARD:
        dev_info(di->dev, "case = CHARGER_TYPE_NON_STANDARD -> \n");
        bq2419x_start_usb_charger(di);
        break;
    case CHARGER_TYPE_BC_USB:
        dev_info(di->dev, "case = CHARGER_TYPE_BC_USB -> \n");
        bq2419x_start_ac_charger(di);
        break;
    case CHARGER_TYPE_STANDARD:
        dev_info(di->dev, "case = CHARGER_TYPE_STANDARD\n");
        bq2419x_start_ac_charger(di);
        break;
    case CHARGER_REMOVED:
        dev_info(di->dev, "case = USB_EVENT_NONE\n");
        bq2419x_stop_charger(di);
        break;
    case USB_EVENT_OTG_ID:
        dev_info(di->dev, "case = USB_EVENT_OTG_ID\n");
        bq2419x_start_usb_otg(di);
        break;
    default:
        break;
    }
}

void notify_vubs_plug_status(unsigned long event)
{
    struct bq2419x_device_info *di = bq_device_19x;

     printk("%s:event = %lu\n",__func__,event);

     if(di == NULL)
     {
        printk("%s:device not init,do nothing!\n",__func__);
        return;
     }
     
    switch (event) {
    case POWER_SUPPLY_TYPE_USB:
    case POWER_SUPPLY_TYPE_USB_ACA:
        di->event = CHARGER_TYPE_USB;
        break;
    case POWER_SUPPLY_TYPE_USB_CDP:
    case POWER_SUPPLY_TYPE_USB_DCP:
        di->event = CHARGER_TYPE_STANDARD;
        break;
    case POWER_SUPPLY_TYPE_UNKNOWN:
        di->event = CHARGER_REMOVED;
        break;
    case POWER_SUPPLY_TYPE_OTG:
        di->event = USB_EVENT_OTG_ID;
        break;
    default:
        di->event = CHARGER_REMOVED;
    }
/* <DTS2014093003947 (report SOC 100%) caiwei 20141009 begin */
    // 当有任何USB口动作时，重新计时。
    g_charge_done_count = 0;
/* DTS2014093003947 (report SOC 100%) caiwei 20141009 end> */
    schedule_work(&di->usb_work);
}
EXPORT_SYMBOL(notify_vubs_plug_status);
 
/* DTS2014072806213 by l00220156 for factory interfaces 20140728 begin */
// 写1开始充电，写0放电
int bq2419x_factory_set_charging_enabled(int enable)
{
    struct bq2419x_device_info *di = bq_device_19x;

    if ((di == NULL) || ((enable!=0)&&(enable!=1))){
        return -1;
    }

    di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING; 
    di->chrg_config = enable;
    di->factory_flag = enable;
    if(di->factory_flag){
        switch (di->event){
        case CHARGER_TYPE_USB:
        case CHARGER_TYPE_NON_STANDARD:
            di->cin_limit = IINLIM_500;
            di->cin_dpmmV = VINDPM_4600;
            break;
            
        case CHARGER_TYPE_STANDARD:
        case CHARGER_TYPE_BC_USB:
            di->cin_limit = di->max_cin_currentmA;
            di->cin_dpmmV = VINDPM_4520;
            break;
            
        case CHARGER_REMOVED:
        case USB_EVENT_OTG_ID:
            return 0;
        default:
            return 0;
        }
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
        
    } else { 
        /*<DTS2014121703947  xiongxi xwx234328 20141217 begin*/
        di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
        /*DTS2014121703947  xiongxi xwx234328 20141217 end>*/
        di->cin_limit = IINLIM_100;
        di->cin_dpmmV = VINDPM_MAX_5080;
    }
        
    bq2419x_config_input_source_reg(di);
    bq2419x_config_power_on_reg(di);
   /* <DTS2014101701500 caiwei 20141021 begin */
    /*<DTS2014121703947  xiongxi xwx234328 20141217 begin*/
    do_power_supply_update();
    /*DTS2014121703947  xiongxi xwx234328 20141217 end>*/
   /* DTS2014101701500 caiwei 20141021 end> */
    return 0;
}
EXPORT_SYMBOL(bq2419x_factory_set_charging_enabled);

int bq2419x_equip_at_batt_charge_enable(int enable)
{
    struct bq2419x_device_info *di = bq_device_19x;

    if ((di == NULL) || ((enable!=0)&&(enable!=1))){
        return -1;
    }

    di->charge_status = POWER_SUPPLY_STATUS_CHARGING; 
    di->chrg_config = enable;
    di->factory_flag = enable;
    if(di->factory_flag){
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    } else {
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
    bq2419x_config_power_on_reg(di);

    do_power_supply_update();
    return 0;    
}
EXPORT_SYMBOL(bq2419x_equip_at_batt_charge_enable);
/* DTS2014072806213 by l00220156 for factory interfaces 20140728 end */
/* <DTS2014111002912 caiwei 20141112 begin */
int bq2419x_otg_status_check(void)
{
    struct bq2419x_device_info* di = bq_device_19x;

    u8 read_reg = 0;

    if (di->event != USB_EVENT_OTG_ID)
    { return 0; }

    bq2419x_read_byte(di, &read_reg, SYS_STATUS_REG08);
    msleep(700);
    bq2419x_read_byte(di, &read_reg, SYS_STATUS_REG08);

    if (read_reg & BQ2419x_VBUS_STAT_OTG)
    {
        return 1;
    }
    else if (last_otg_status)
    { return 1; }
    else
    { return 0; }
}
EXPORT_SYMBOL(bq2419x_otg_status_check);
/* DTS2014111002912 caiwei 20141112 end> */
/*
* set 1 --- hz_mode ; 0 --- not hz_mode
*/

/*<DTS2014121703947  xiongxi xwx234328 20141217 begin*/
int bq2419x_ovp_status_check(void)
{
	struct bq2419x_device_info* di = bq_device_19x;

    u8 read_reg = 0;

    bq2419x_read_byte(di, &read_reg, CHARGER_FAULT_REG09);
    msleep(700);
    bq2419x_read_byte(di, &read_reg, CHARGER_FAULT_REG09);

	return read_reg;
}
EXPORT_SYMBOL(bq2419x_ovp_status_check);
/*DTS2014121703947  xiongxi xwx234328 20141217 end*/

static ssize_t bq2419x_set_enable_hz_mode(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    di->hz_mode= val;
    bq2419x_config_input_source_reg(di);

    return status;
}

static ssize_t bq2419x_show_enable_hz_mode(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->hz_mode;
    return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2419x_set_dppm_voltage(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < VINDPM_MIN_3880)
                      || (val > VINDPM_MAX_5080))
         return -EINVAL;

    di->cin_dpmmV = val;
    bq2419x_config_input_source_reg(di);

    return status;
}

static ssize_t bq2419x_show_dppm_voltage(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->cin_dpmmV;
    return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2419x_set_cin_limit(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < IINLIM_100)
                || (val > IINLIM_3000))
        return -EINVAL;

    di->cin_limit = val;
    bq2419x_config_input_source_reg(di);

    return status;
}

static ssize_t bq2419x_show_cin_limit(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->cin_limit;
    return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2419x_set_regulation_voltage(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < VCHARGE_MIN_3504)
                     || (val > VCHARGE_MAX_4400))
        return -EINVAL;

    di->voltagemV = val;
    bq2419x_config_voltage_reg(di);

    return status;
}

static ssize_t bq2419x_show_regulation_voltage(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->voltagemV;
    return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2419x_set_charge_current(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < ICHG_MIN)
                   || (val > ICHG_3000))
        return -EINVAL;

    di->currentmA = val;
    bq2419x_config_current_reg(di);

    return status;
}

static ssize_t bq2419x_show_charge_current(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->currentmA;
    return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2419x_set_precharge_current(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > IPRECHRG_MAX_2048))
    return -EINVAL;

    di->prechrg_currentmA = val;
    bq2419x_config_prechg_term_current_reg(di);

    return status;
}

static ssize_t bq2419x_show_precharge_current(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->prechrg_currentmA;
    return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2419x_set_termination_current(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > ITERM_MAX_2048))
        return -EINVAL;

    di->term_currentmA = val;
    bq2419x_config_prechg_term_current_reg(di);

    return status;
}

static ssize_t bq2419x_show_termination_current(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->term_currentmA;
    return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2419x_set_enable_itermination(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    di->enable_iterm = val;
    bq2419x_config_term_timer_reg(di);

    return status;
}

static ssize_t bq2419x_show_enable_itermination(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->enable_iterm;
    return sprintf(buf, "%lu\n", val);
}

/* set 1 --- enable_charger; 0 --- disable charger */
static ssize_t bq2419x_set_enable_charger(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    u8 read_reg = 0;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    di->charge_status = POWER_SUPPLY_STATUS_CHARGING; 
    di->chrg_config = val;
    di->factory_flag = val;
    if(di->factory_flag){
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    } else {
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
    bq2419x_config_power_on_reg(di);

    bq2419x_read_byte(di, &read_reg, SYS_STATUS_REG08);
    if ((read_reg & BQ2419x_CHGR_STAT_CHAEGE_DONE) == BQ2419x_CHGR_STAT_CHAEGE_DONE){
        return status;
    }

    do_power_supply_update();
    return status;
}

static ssize_t bq2419x_show_enable_charger(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->chrg_config ;
    return sprintf(buf, "%lu\n", val);
}

/* set 1 --- enable_batfet; 0 --- disable batfet */
static ssize_t bq2419x_set_enable_batfet(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    di->enable_batfet = val ^ 0x01;
    bq2419x_config_misc_operation_reg(di);

    return status;
}

static ssize_t bq2419x_show_enable_batfet(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->enable_batfet ^ 0x01;
    return sprintf(buf, "%lu\n", val);
}

/*
* set 1 --- enable bq24192 IC; 0 --- disable bq24192 IC
*
*/
static ssize_t bq2419x_set_enable_cd(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    di->cd_active =val ^ 0x1;
    gpio_set_value(di->gpio, di->cd_active);
    return status;
}

static ssize_t bq2419x_show_enable_cd(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->cd_active ^ 0x1;
    return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2419x_set_charging(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if (strncmp(buf, "startac", 7) == 0) {
        if (di->charger_source == POWER_SUPPLY_TYPE_USB)
            bq2419x_stop_charger(di);
        bq2419x_start_ac_charger(di);
    } else if (strncmp(buf, "startusb", 8) == 0) {
        if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
            bq2419x_stop_charger(di);
        bq2419x_start_usb_charger(di);
    } else if (strncmp(buf, "stop" , 4) == 0) {
        bq2419x_stop_charger(di);
    } else if (strncmp(buf, "otg" , 3) == 0) {
        if (di->charger_source == POWER_SUPPLY_TYPE_BATTERY){
            bq2419x_stop_charger(di);
            bq2419x_start_usb_otg(di);
        } else{
            return -EINVAL;
        }
    } else
        return -EINVAL;

    return status;
}

static ssize_t bq2419x_set_wakelock_enable(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    if ((val) && (di->charger_source != POWER_SUPPLY_TYPE_BATTERY))
        wake_lock(&chrg_lock);
    else
        wake_unlock(&chrg_lock);

    di->wakelock_enabled = val;
    return status;
}

static ssize_t bq2419x_show_wakelock_enable(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned int val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->wakelock_enabled;
    return sprintf(buf, "%u\n", val);
}

static ssize_t bq2419x_show_chargelog(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    int i = 0;
    u8 read_reg[11] = {0};
    u8 buf_temp[26] = {0};
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    bq2419x_read_block(di, &read_reg[0], 0, 11);
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 begin*/
    bq2419x_read_byte(di, &read_reg[9], CHARGER_FAULT_REG09);
    /* <DTS2014073105492 modified by l00220156 20140818 for bq24296 end*/
    for(i=0;i<11;i++)
    {
        sprintf(buf_temp,"0x%-8.2x",read_reg[i]);
        strcat(buf,buf_temp);
    }
    strcat(buf,"\n");
   return strlen(buf);
}

static ssize_t bq2419x_set_calling_limit(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
       return -EINVAL;

    di->calling_limit = val;
    if (di->charger_source == POWER_SUPPLY_TYPE_MAINS)
    {
        if(di->calling_limit){
            di->cin_limit = IINLIM_900;
            di->currentmA = ICHG_820;
            bq2419x_config_input_source_reg(di);
            bq2419x_config_current_reg(di);
            dev_info(di->dev,"calling_limit_current = %d\n", di->cin_limit);
        } else {
            di->battery_temp_status = -1;
            di->cin_limit = di->max_cin_currentmA;
            di->currentmA = di->max_currentmA ;
            bq2419x_config_input_source_reg(di);
            bq2419x_config_current_reg(di);
        }
    } else {
        di->calling_limit = 0;
    }

    return status;
}

static ssize_t bq2419x_show_calling_limit(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->calling_limit;

    return sprintf(buf, "%lu\n", val);
}
static ssize_t bq2419x_set_compesation_resistor(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 70))
        return -EINVAL;

    di->bat_compohm = val;
    bq2419x_config_thernal_regulation_reg(di);

    return status;
}

static ssize_t bq2419x_show_compesation_resistor(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->bat_compohm;
    return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2419x_set_compesation_voltage(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 112))
        return -EINVAL;

    di->comp_vclampmV = val;
    bq2419x_config_thernal_regulation_reg(di);

    return status;
}

static ssize_t bq2419x_show_compesation_voltage(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->comp_vclampmV;
    return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2419x_set_limit_temp_charging(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    di->limit_charging_flag = val;
    if(di->charger_source == POWER_SUPPLY_TYPE_MAINS){
        di->cin_limit = di->max_cin_currentmA;
        di->currentmA = di->max_currentmA;
        di->enable_low_chg = DIS_FORCE_20PCT;
        bq2419x_config_input_source_reg(di);
        bq2419x_config_current_reg(di);
    }

    bq2419x_monitor_battery_ntc_charging(di);

    return status;
}

static ssize_t bq2419x_show_limit_temp_charging(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->limit_charging_flag;
    return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2419x_set_coldhot_charging(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 0) || (val > 1))
        return -EINVAL;

    di->coldhot_charging_flag = val;
    di->limit_charging_flag = 1;

    bq2419x_monitor_battery_ntc_charging(di);
    return status;
}

static ssize_t bq2419x_show_coldhot_charging(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    unsigned long val;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    val = di->coldhot_charging_flag;
    return sprintf(buf, "%lu\n", val);
}

static ssize_t bq2419x_set_temperature_parameter(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < 40)
                   || (val > 70))
        return -EINVAL;

     di->temperature_warm = val;
     di->temperature_hot  = val + 5;

    bq2419x_monitor_battery_ntc_charging(di);

    return status;
}

static ssize_t bq2419x_show_temperature_parameter(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    int read_reg[6] = {0};
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    read_reg[0] = di->temperature_cold;
    read_reg[1] = di->temperature_cool;
    read_reg[2] = di->temperature_5;
    read_reg[3] = di->temperature_15;
    read_reg[4] = di->temperature_warm;
    read_reg[5] = di->temperature_hot;

    sprintf(buf,"%-9d  %-9d  %-9d  %-9d  %-9d  %-9d",
    read_reg[0],read_reg[1],read_reg[2],read_reg[3],read_reg[4],read_reg[5]);

    return strlen(buf);
}

static ssize_t bq2419x_set_current(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    long val;
    int status = count;
    struct bq2419x_device_info *di = dev_get_drvdata(dev);

    if ((strict_strtol(buf, 10, &val) < 0) || (val < ICHG_MIN)
                   || (val > ICHG_3000))
        return -EINVAL;

    di->max_currentmA = val;
    di->max_cin_currentmA = val;

    return status;
}

/* <DTS2014012404412 sunwenyong 20140128 begin */

/*
* set 1 --- hz_mode ; 0 --- not hz_mode
*/
/* < DTS2014021400603 yuanzhen 20140218 begin */
void bq2419x_set_enable_hz_mode_for_factory_test(struct bq2419x_device_info *di, int val)
{
    u8 read_reg = 0;
    if((di == NULL) ||(val < 0) || (val > 1))
        return ;

    di->hz_mode= val;
    di->chrg_config = !val;
    di->factory_flag = !val;
    bq2419x_config_input_source_reg(di);
    bq2419x_config_power_on_reg(di);

    if(val){
        di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
    } else {
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    }

    bq2419x_read_byte(di, &read_reg, SYS_STATUS_REG08);
    if ((read_reg & BQ2419x_CHGR_STAT_CHAEGE_DONE) == BQ2419x_CHGR_STAT_CHAEGE_DONE){
        return;
    }

    do_power_supply_update();

    return ;
}
/* DTS2014021400603 yuanzhen 20140218 end > */

/* set 1 --- enable_charger; 0 --- disable charger */
void bq2419x_set_enable_charger_for_factory_test(struct bq2419x_device_info *di, int val)
{
    u8 read_reg = 0;
	if((di == NULL) ||(val < 0) || (val > 1))
		return ;

    di->charge_status = POWER_SUPPLY_STATUS_CHARGING; 
    di->chrg_config = val;
    di->factory_flag = val;
    if(di->factory_flag){
        di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
    } else {
        di->charge_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
    bq2419x_config_power_on_reg(di);

    bq2419x_read_byte(di, &read_reg, SYS_STATUS_REG08);
    if ((read_reg & BQ2419x_CHGR_STAT_CHAEGE_DONE) == BQ2419x_CHGR_STAT_CHAEGE_DONE){
        return;
    }

    do_power_supply_update();
}

/* DTS2014012404412 sunwenyong 20140128 end> */
static DEVICE_ATTR(enable_hz_mode, S_IWUSR | S_IRUGO,
                bq2419x_show_enable_hz_mode,
                bq2419x_set_enable_hz_mode);
static DEVICE_ATTR(dppm_voltage, S_IWUSR | S_IRUGO,
                bq2419x_show_dppm_voltage,
                bq2419x_set_dppm_voltage);
static DEVICE_ATTR(cin_limit, S_IWUSR | S_IRUGO,
                bq2419x_show_cin_limit,
                bq2419x_set_cin_limit);
static DEVICE_ATTR(regulation_voltage, S_IWUSR | S_IRUGO,
                bq2419x_show_regulation_voltage,
                bq2419x_set_regulation_voltage);
static DEVICE_ATTR(charge_current, S_IWUSR | S_IRUGO,
                bq2419x_show_charge_current,
                bq2419x_set_charge_current);
static DEVICE_ATTR(termination_current, S_IWUSR | S_IRUGO,
                bq2419x_show_termination_current,
                bq2419x_set_termination_current);
static DEVICE_ATTR(precharge_current, S_IWUSR | S_IRUGO,
                bq2419x_show_precharge_current,
                bq2419x_set_precharge_current);
static DEVICE_ATTR(enable_itermination, S_IWUSR | S_IRUGO,
                bq2419x_show_enable_itermination,
                bq2419x_set_enable_itermination);
static DEVICE_ATTR(enable_charger, S_IWUSR | S_IRUGO,
                bq2419x_show_enable_charger,
                bq2419x_set_enable_charger);
static DEVICE_ATTR(enable_batfet, S_IWUSR | S_IRUGO,
                bq2419x_show_enable_batfet,
                bq2419x_set_enable_batfet);
static DEVICE_ATTR(enable_cd, S_IWUSR | S_IRUGO,
                bq2419x_show_enable_cd,
                bq2419x_set_enable_cd);
static DEVICE_ATTR(charging, S_IWUSR | S_IRUGO,
                NULL,
                bq2419x_set_charging);
static DEVICE_ATTR(wakelock_enable, S_IWUSR | S_IRUGO,
                bq2419x_show_wakelock_enable,
                bq2419x_set_wakelock_enable);
static DEVICE_ATTR(chargelog, S_IWUSR | S_IRUGO,
                bq2419x_show_chargelog,
                NULL);
static DEVICE_ATTR(calling_limit, S_IWUSR | S_IRUGO,
                bq2419x_show_calling_limit,
                bq2419x_set_calling_limit);
static DEVICE_ATTR(compesation_resistor, S_IWUSR | S_IRUGO,
                bq2419x_show_compesation_resistor,
                bq2419x_set_compesation_resistor);
static DEVICE_ATTR(compesation_voltage, S_IWUSR | S_IRUGO,
                bq2419x_show_compesation_voltage,
                bq2419x_set_compesation_voltage);
static DEVICE_ATTR(limit_charging, S_IWUSR | S_IRUGO,
                bq2419x_show_limit_temp_charging,
                bq2419x_set_limit_temp_charging);
static DEVICE_ATTR(coldhot_charging, S_IWUSR | S_IRUGO,
                bq2419x_show_coldhot_charging,
                bq2419x_set_coldhot_charging);
static DEVICE_ATTR(temperature_parameter, S_IWUSR | S_IRUGO,
                bq2419x_show_temperature_parameter,
                bq2419x_set_temperature_parameter);
static DEVICE_ATTR(set_current, S_IWUSR | S_IRUGO,
                NULL,
                bq2419x_set_current);

static struct attribute *bq2419x_attributes[] = {
    &dev_attr_enable_hz_mode.attr,
    &dev_attr_dppm_voltage.attr,
    &dev_attr_cin_limit.attr,
    &dev_attr_regulation_voltage.attr,
    &dev_attr_charge_current.attr,
    &dev_attr_precharge_current.attr,
    &dev_attr_termination_current.attr,
    &dev_attr_enable_itermination.attr,
    &dev_attr_enable_charger.attr,
    &dev_attr_enable_batfet.attr,
    &dev_attr_enable_cd.attr,
    &dev_attr_charging.attr,
    &dev_attr_wakelock_enable.attr,
    &dev_attr_chargelog.attr,
    &dev_attr_calling_limit.attr,
    &dev_attr_compesation_resistor.attr,
    &dev_attr_compesation_voltage.attr,
    &dev_attr_limit_charging.attr,
    &dev_attr_coldhot_charging.attr,
    &dev_attr_temperature_parameter.attr,
    &dev_attr_set_current.attr,
    NULL,
};

static const struct attribute_group bq2419x_attr_group = {
    .attrs = bq2419x_attributes,
};

static int bq2419x_dt_parse(struct device *dev, struct bq2419x_platform_data *pdata) 
{
    struct device_node *np = dev->of_node;
    u32 temp_val = 0;
    int rc = 0;
    
    rc = of_property_read_u32(np, "max_charger_currentmA", &temp_val); 
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read max_charger_currentmA\n");
        return rc;
    } 
    else {
        pdata->max_charger_currentmA = temp_val;
        printk("max_currentmA is %d\n", pdata->max_charger_currentmA);
    }
    
    rc = of_property_read_u32(np, "max_charger_voltagemV", &temp_val);
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read max_voltagemV\n");
        return rc;
    } 
    else {
        pdata->max_charger_voltagemV = temp_val;
        printk("max_voltagemV is %d\n", pdata->max_charger_voltagemV);
    }
    
    rc = of_property_read_u32(np, "max_cin_limit_currentmA", &temp_val);   
    if (rc && (rc != -EINVAL)) {
        dev_err(dev, "Unable to read max_cin_limit_currentmA\n");
        return rc;
    } 
    else {
        pdata->max_cin_limit_currentmA = temp_val;
        printk("max_cin_currentmA is %d\n", pdata->max_cin_limit_currentmA);
    }
    
	pdata->gpio = of_get_named_gpio(np,"gpio_cd",0);
	if(!gpio_is_valid(pdata->gpio)){
		dev_err(dev, "Unable to read gpio\n");
		return -EFAULT;
	}
    printk("gpio is %d\n", pdata->gpio);

	pdata->gpio_int = of_get_named_gpio(np,"gpio_int",0);
	if(!gpio_is_valid(pdata->gpio_int)){
		dev_err(dev, "Unable to read gpio\n");
		return -EFAULT;
	}
    printk("gpio is %d\n", pdata->gpio_int);
    return rc;
}

/* < DTS2014031501494 yuanzhen 20140317 begin */
static int bq2419x_charger_probe(struct i2c_client *client,
                           const struct i2c_device_id *id)
{
    struct bq2419x_device_info *di;
    struct bq2419x_platform_data *pdata = NULL;
    int ret = 0;
    u8 read_reg = 0;

    di = kzalloc(sizeof(*di), GFP_KERNEL);
    if (!di) {
        dev_err(&client->dev, "bq2419x_device_info is NULL!\n");
        return -ENOMEM;
    }

    di->dev = &client->dev;
    di->client = client;

    i2c_set_clientdata(client, di);

    ret = bq2419x_read_byte(di, &read_reg, PART_REVISION_REG0A);

    if (ret < 0) {
        dev_err(&client->dev, "chip not present at address %x\n",
                     client->addr);
        ret = -EINVAL;
        goto err_kfree;
    }
    
    /* <DTS2014073105492 modified by l00220156 20140731 for bq24296 begin*/
    if (((read_reg & 0x38) == BQ2419x || (read_reg & 0x38) == BQ24192)
         && (client->addr == 0x6B)){
        di->bqchip_version = BQ24192;
    }
    else if (((read_reg & BQ24296) == BQ24296) && (client->addr == 0x6B)){
        di->bqchip_version = BQ24296;
    }
    /*<DTS2014122508930 added by xiongxi xwx234328 20141227 begin*/
    if(((read_reg & BQ24196) == BQ24196) && (client->addr == 0x6B)){
        di->bqchip_version = BQ24196;
    }
    /*DTS2014122508930 added by xiongxi xwx234328 20141227 end>*/

    dev_notice(&client->dev, "Chip address:%x, version: %x", client->addr, read_reg);
    /* <DTS2014073105492 modified by l00220156 20140731 for bq24296 end*/

    if (di->bqchip_version == 0) {
        dev_dbg(&client->dev, "unknown bq chip\n");
        dev_dbg(&client->dev, "Chip address %x", client->addr);
        dev_dbg(&client->dev, "bq chip version reg value %x", read_reg);
        ret = -EINVAL;
        goto err_kfree;
    }

    if (client->dev.of_node) {
        pdata = kzalloc(sizeof(struct bq2419x_platform_data), GFP_KERNEL);
        if(pdata == NULL)
        {
            ret = -EINVAL;
            dev_err(&client->dev,				
                "failed to allocate memory for pdata: %d\n", ret);
            goto err_kfree;
        }

        memset(pdata, 0 , sizeof(struct bq2419x_platform_data));
        ret = bq2419x_dt_parse(&client->dev, pdata);
        if (ret) 
        {
            dev_err(&client->dev,				
                "Unable to parse platfrom data ret=%d\n", ret);
            goto err_pdata;
        }
        else
        {
            di->max_voltagemV = pdata->max_charger_voltagemV;
            di->max_currentmA = pdata->max_charger_currentmA;
	    /* <DTS2015012010751 caiwei 20150120 begin */
	    /* <DTS2015012904724 xiongxi 20150129 begin*/
	    if (factory_flag_5V2A)
		    di->max_cin_currentmA = ICHG_2000;
	    else if(BQ24196 == di->bqchip_version)
		    di->max_cin_currentmA = ICHG_900;
	    else
		    di->max_cin_currentmA = pdata->max_cin_limit_currentmA;
	    /* DTS2015012904724 xiongxi 20150129 end>*/
            /* DTS2015012010751 caiwei 20150120 end> */
            di->gpio = pdata->gpio;//ENABLE_BQ2419x_CHARGER;
            di->gpio_int = pdata->gpio_int;
        }
    }

    /*set gpio_114 to control CD pin to disable/enable bq24192 IC*/
    ret = gpio_request(di->gpio, "gpio_114_cd");
    if (ret) {
          dev_err(&client->dev, "could not request irq\n");
          ret = -ENOMEM;
          goto err_io;
    }
    gpio_direction_output(di->gpio, 0);

    ret = gpio_request(di->gpio_int,"charger_int");
    if(ret) {
        dev_err(&client->dev, "could not request gpio_int\n");
        goto err_gpio_int;
    }
    gpio_direction_input(di->gpio_int);

    di->irq_int = gpio_to_irq(di->gpio_int);
    if(di->irq_int < 0) {
        dev_err(&client->dev, "could not map gpio_int to irq\n");
        goto err_int_map_irq;
    }
    ret = request_irq(di->irq_int,bq2419x_irq_int_interrupt,IRQF_TRIGGER_FALLING,"charger_int_irq",di);
    if(ret) {
        dev_err(&client->dev, "could not request irq_int\n");
        di->irq_int = -1;
        goto err_irq_int;
    }

    disable_irq(di->irq_int);
    irq_int_active = 0;
/* <DTS2014092405474 caiwei 20140926 begin */
    di->hw_board_id = socinfo_get_platform_type();
/* DTS2014092405474 caiwei 20140926 end> */
/* <DTS2014093003947 (40~45 degree charge current 1A) caiwei 20141009 begin */
/*<DTS2015010402454 xiongxi xwx234328 20150104 begin*/
    if ((di->hw_board_id == BOARDID_T1_10) || (di->hw_board_id == BOARDID_T1_A21L)
		 || (di->hw_board_id == BOARDID_T1_A21W) 
		 || (di->hw_board_id == BOARDID_T1_A22L) || (di->hw_board_id == BOARDID_T1_A23L) 
		 || (di->hw_board_id == BOARDID_T1_A24L) || (di->hw_board_id == BOARDID_T1_A21LT))
		 di->hw_is_pad = 1;
	else di->hw_is_pad = 0;
/*DTS2015010402454 xiongxi xwx234328 20150104 end>*/
/* DTS2014093003947 (40~45 degree charge current 1A) caiwei 20141009 end> */
    /* <DTS2014123106973 caiwei 20141231 begin */
	/* <DTS2015012904724 xiongxi 20150129 begin*/
	//delete by xiongxi
	/* DTS2015012904724 xiongxi 20150129 end>*/
    /* DTS2014123106973 caiwei 20141231 end> */
    di->voltagemV = di->max_voltagemV;
    di->currentmA = ICHG_MIN ;
    /*<DTS2014122906101 xiongxi xwx234328 20150126 begin*/
    di->cin_dpmmV = VINDPM_4680;
    /*DTS2014122906101 xiongxi xwx234328 20150126 end>*/
    di->cin_limit = IINLIM_500;
    di->sys_minmV = SYS_MIN_3500;
    di->prechrg_currentmA = IPRECHRG_256;
    di->term_currentmA = ITERM_MIN_128;
    /*<DTS2015020507957 xiongxi xwx234328 20150205 begin*/
    di->watchdog_timer = WATCHDOG_DIS;
    /* DTS2015020507957 xiongxi xwx234328 20150205 end>*/
    di->chrg_timer = CHG_TIMER_8;
    di->bat_compohm = BAT_COMP_40;
    di->comp_vclampmV = VCLAMP_48;
    /* < DTS2014011515091 yuanzhen 20140115 begin*/
    di->hot_design_current = di->max_cin_currentmA;
    /* DTS2014011515091 yuanzhen 20140115 end > */
    di->hz_mode = DIS_HIZ;
    di->chrg_config = EN_CHARGER;
    di->boost_lim = BOOST_LIM_500;
    di->enable_low_chg = DIS_FORCE_20PCT;
    di->enable_iterm = EN_TERM;
    di->enable_timer = EN_TIMER;
    di->enable_batfet = EN_BATFET;
    di->factory_flag = 0;
    di->cd_active = 0;
    di->params.enable = 1;
    di->cfg_params = 1;
    di->enable_dpdm = DPDM_DIS;
    di->battery_full = 0;
    di->charge_full_count = 0;

    di->japan_charger = 0;
    di->is_two_stage_charger = 0;
    di->two_stage_charger_status = TWO_STAGE_CHARGE_FIRST_STAGE;
    di->first_stage_voltage = 4200;
    di->second_stage_voltage = 4100;
    di->is_disable_cool_temperature_charger = 0;
    di->high_temp_para =HIGH_TEMP_CP_U9701L;

    /*if the product is for japan, should configured the follow parameters in dts
    * japan_charger,is_two_stage_charger,first_stage_voltage,second_stage_voltage,is_disable_cool_temperature_charger,high_temp_para
    * bq2419x_get_boardid_japan_charge_parameter(di) is not used anyway
    */
    /* <DTS2015010804597 caiwei 20150112 begin */
    bq2419x_config_input_source_reg(di);
    /* DTS2015010804597 caiwei 20150112 end> */
    bq2419x_config_power_on_reg(di);
    bq2419x_config_current_reg(di);
    bq2419x_config_prechg_term_current_reg(di);
    bq2419x_config_voltage_reg(di);
    bq2419x_config_term_timer_reg(di);
    bq2419x_config_thernal_regulation_reg(di);
    bq2419x_config_limit_temperature_parameter(di);

    wake_lock_init(&chrg_lock, WAKE_LOCK_SUSPEND, "bq2419x_chrg_wakelock");

    /* Configuration parameters for 0C-10C, set the default to the parameters,
       and get the values from boardid files */
    di->design_capacity = BQ2419x_DEFAULT_CAPACITY;
    di->charge_in_temp_5 = DEFAULT_CHARGE_PARAM_LOW_TEMP;
    di->charge_in_temp_15 = DEFAULT_CHARGE_PARAM_LOW_TEMP;

    INIT_DELAYED_WORK(&di->bq2419x_charger_work, bq2419x_charger_work);

    INIT_DELAYED_WORK(&di->bq2419x_usb_otg_work, bq2419x_usb_otg_work);

    INIT_DELAYED_WORK(&di->otg_int_work, bq2419x_otg_int_work);

    INIT_WORK(&di->usb_work, bq2419x_usb_charger_work);

    ret = sysfs_create_group(&client->dev.kobj, &bq2419x_attr_group);
    if (ret) {
        dev_dbg(&client->dev, "could not create sysfs files\n");
        goto err_sysfs;
    }

    di->event = CHARGER_REMOVED;
    
    schedule_work(&di->usb_work);

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_CHARGER);
#endif
    dev_err(di->dev, "bq2419x probe ok!\n");
    bq_device_19x = di;
    return 0;

err_sysfs:
    wake_lock_destroy(&chrg_lock);
    free_irq(di->irq_int,di);
err_irq_int:
err_int_map_irq:
    gpio_free(di->gpio_int);
err_gpio_int:
    gpio_free(di->gpio);
err_io:
err_pdata:
    kfree(pdata);
    pdata = NULL;
err_kfree:
    kfree(di);
    di = NULL;

    return ret;
}

static int bq2419x_charger_remove(struct i2c_client *client)
{
    struct bq2419x_device_info *di = i2c_get_clientdata(client);
    sysfs_remove_group(&client->dev.kobj, &bq2419x_attr_group);
    wake_lock_destroy(&chrg_lock);

    cancel_delayed_work(&di->bq2419x_charger_work);
    cancel_delayed_work_sync(&di->bq2419x_usb_otg_work);
    cancel_delayed_work_sync(&di->otg_int_work);
    flush_scheduled_work();
    gpio_free(di->gpio);
    free_irq(di->irq_int,di);
    gpio_free(di->gpio_int);
    kfree(di);

    return 0;
}

static void bq2419x_charger_shutdown(struct i2c_client *client)
{
    struct bq2419x_device_info *di = i2c_get_clientdata(client);
/* <DTS2014092002531 caiwei 20140923 begin*/
    di->power_on_config_reg01 = REG_RST;
    bq2419x_write_byte(di, di->power_on_config_reg01, POWER_ON_CONFIG_REG01);
/* DTS2014092002531 caiwei 20140923 end> */
    cancel_delayed_work(&di->bq2419x_charger_work);
    /* < DTS2014021400603 yuanzhen 20140218 begin */
    /* flush work queue will casue shutdown abnormal */
    //flush_scheduled_work();
    /* DTS2014021400603 yuanzhen 20140218 end > */
    cancel_delayed_work_sync(&di->bq2419x_usb_otg_work);
    cancel_delayed_work_sync(&di->otg_int_work);
    return;
}
/* DTS2014031501494 yuanzhen 20140317 end > */

static const struct i2c_device_id bq2419x_id[] = {
    { "bq2419x_charger", 0 },
    {},
};

#ifdef CONFIG_PM
static int bq2419x_charger_suspend(struct i2c_client *client,
              pm_message_t state)
{
    struct bq2419x_device_info *di = i2c_get_clientdata(client);

    if(di->charger_source == POWER_SUPPLY_TYPE_MAINS){
        if(di->battery_full){
            if (!wake_lock_active(&chrg_lock)){
                cancel_delayed_work(&di->bq2419x_charger_work);
                if((wakeup_timer_seconds > 300) || !wakeup_timer_seconds)
                    wakeup_timer_seconds = 300;
            }
        }
    }

    bq2419x_config_power_on_reg(di);
    return 0;
}

static int bq2419x_charger_resume(struct i2c_client *client)
{
    struct bq2419x_device_info *di = i2c_get_clientdata(client);

    if(di->charger_source == POWER_SUPPLY_TYPE_MAINS){
        bq2419x_config_voltage_reg(di);
        schedule_delayed_work(&di->bq2419x_charger_work, msecs_to_jiffies(0));
    }

    bq2419x_config_power_on_reg(di);
   return 0;
}
#else
#define bq2419x_charger_suspend       NULL
#define bq2419x_charger_resume        NULL
#endif /* CONFIG_PM */


static struct of_device_id bq2419x_charger_match_table[] = 
{
    {    .compatible = "ti,bq2419x_charger",},
    { },
};

MODULE_DEVICE_TABLE(i2c, bq24192);
static struct i2c_driver bq2419x_charger_driver = {
    .probe = bq2419x_charger_probe,
    .remove = bq2419x_charger_remove,
    .suspend = bq2419x_charger_suspend,
    .resume = bq2419x_charger_resume,
    .shutdown = bq2419x_charger_shutdown,
    .id_table = bq2419x_id,
    .driver = {
         .owner = THIS_MODULE,
         .name = "bq2419x_charger",
         .of_match_table = bq2419x_charger_match_table,
    },
};
/* <DTS2015012010751 caiwei 20150120 begin */
static int __init early_parse_factory_flag(char * p)
{
         if(p) {
            if(!strcmp(p,"factory")) {
                 factory_flag_5V2A = true;
	     }
         }
         return 0;
}
early_param("androidboot.huawei_swtype",early_parse_factory_flag);
/* DTS2015012010751 caiwei 20150120 end> */
static int __init bq2419x_charger_init(void)
{
    return i2c_add_driver(&bq2419x_charger_driver);
}
module_init(bq2419x_charger_init);

static void __exit bq2419x_charger_exit(void)
{
    i2c_del_driver(&bq2419x_charger_driver);
}
module_exit(bq2419x_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("HW Inc");
/* DTS2014071806762 lWX198526 l00220156 20140718 end */
