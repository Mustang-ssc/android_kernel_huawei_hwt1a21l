/*
 * bq2415x charger driver
 *
 * Copyright (C) 2011-2012  Pali Rohár <pali.rohar@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef BQ24152_CHARGER_H
#define BQ24152_CHARGER_H

/*
 * This is platform data for bq2415x chip. It contains default board
 * voltages and currents which can be also later configured via sysfs. If
 * value is -1 then default chip value (specified in datasheet) will be
 * used.
 *
 * Value resistor_sense is needed for for configuring charge and
 * termination current. It it is less or equal to zero, configuring charge
 * and termination current will not be possible.
 *
 * Function set_mode_hook is needed for automode (setting correct current
 * limit when charger is connected/disconnected or setting boost mode).
 * When is NULL, automode function is disabled. When is not NULL, it must
 * have this prototype:
 *
 *    int (*set_mode_hook)(
 *      void (*hook)(enum bq2415x_mode mode, void *data),
 *      void *data)
 *
 * hook is hook function (see below) and data is pointer to driver private
 * data
 *
 * bq2415x driver will call it as:
 *
 *    platform_data->set_mode_hook(bq2415x_hook_function, bq2415x_device);
 *
 * Board/platform function set_mode_hook return non zero value when hook
 * function was successful registered. Platform code should call that hook
 * function (which get from pointer, with data) every time when charger
 * was connected/disconnected or require to enable boost mode. bq2415x
 * driver then will set correct current limit, enable/disable charger or
 * boost mode.
 *
 * Hook function has this prototype:
 *
 *    void hook(enum bq2415x_mode mode, void *data);
 *
 * mode is bq2415x mode (charger or boost)
 * data is pointer to driver private data (which get from
 * set_charger_type_hook)
 *
 * When bq driver is being unloaded, it call function:
 *
 *    platform_data->set_mode_hook(NULL, NULL);
 *
 * (hook function and driver private data are NULL)
 *
 * After that board/platform code must not call driver hook function! It
 * is possible that pointer to hook function will not be valid and calling
 * will cause undefined result.
 */
/*<DTS2014061605512 liyuping 20140617 begin */
enum bq2415x_command {
	BQ2415X_TIMER_RESET,
	BQ2415X_OTG_STATUS,
	BQ2415X_STAT_PIN_STATUS,
	BQ2415X_STAT_PIN_ENABLE,
	BQ2415X_STAT_PIN_DISABLE,
	BQ2415X_CHARGE_STATUS,
	BQ2415X_BOOST_STATUS,
	BQ2415X_FAULT_STATUS,

	BQ2415X_CHARGE_TERMINATION_STATUS,
	BQ2415X_CHARGE_TERMINATION_ENABLE,
	BQ2415X_CHARGE_TERMINATION_DISABLE,
	BQ2415X_CHARGER_STATUS,
	BQ2415X_CHARGER_ENABLE,
	BQ2415X_CHARGER_DISABLE,
	BQ2415X_HIGH_IMPEDANCE_STATUS,
	BQ2415X_HIGH_IMPEDANCE_ENABLE,
	BQ2415X_HIGH_IMPEDANCE_DISABLE,
	BQ2415X_BOOST_MODE_STATUS,
	BQ2415X_BOOST_MODE_ENABLE,
	BQ2415X_BOOST_MODE_DISABLE,

	BQ2415X_OTG_LEVEL,
	BQ2415X_OTG_ACTIVATE_HIGH,
	BQ2415X_OTG_ACTIVATE_LOW,
	BQ2415X_OTG_PIN_STATUS,
	BQ2415X_OTG_PIN_ENABLE,
	BQ2415X_OTG_PIN_DISABLE,

	BQ2415X_VENDER_CODE,
	BQ2415X_PART_NUMBER,
	BQ2415X_REVISION,
};

enum bq2415x_chip {
	BQUNKNOWN,
	BQ24150,
	BQ24150A,
	BQ24151,
	BQ24151A,
	BQ24152,
	BQ24153,
	BQ24153A,
	BQ24155,
	BQ24156,
	BQ24156A,
	BQ24158,
};


/* Supported modes with maximal current limit */
enum bq2415x_mode {
	BQ2415X_MODE_OFF,		/* offline mode (charger disabled) */
	BQ2415X_MODE_NONE,		/* unknown charger (100mA) */
	BQ2415X_MODE_HOST_CHARGER,	/* usb host/hub charger (500mA) */
	BQ2415X_MODE_DEDICATED_CHARGER, /* dedicated charger (unlimited) */
	BQ2415X_MODE_BOOST,		/* boost mode (charging disabled) */
};

struct bq2415x_platform_data {
	int current_limit;		/* mA */
	int weak_battery_voltage;	/* mV */
	int battery_regulation_voltage;	/* mV */
	int charge_current;		/* mA */
	int termination_current;	/* mA */
	int resistor_sense;		/* m ohm */
	int (*set_mode_hook)(void (*hook)(enum bq2415x_mode mode, void *data),
			     void *data);
};

struct bq2415x_device {
	struct device *dev;
	struct bq2415x_platform_data init_data;
	struct power_supply charger;
	struct delayed_work work;
	enum bq2415x_mode reported_mode;/* mode reported by hook function */
	enum bq2415x_mode mode;		/* current configured mode */
	enum bq2415x_chip chip;
	const char *timer_error;
	char *model;
	char *name;
	int autotimer;	/* 1 - if driver automatically reset timer, 0 - not */
	int automode;	/* 1 - enabled, 0 - disabled; -1 - not supported */
	int id;
	/* <DTS2014052906550 liyuping 20140530 begin */
	int iusb_limit;
	spinlock_t ibat_change_lock;
	struct power_supply *qcom_charger_psy;
	struct power_supply *qcom_bms_psy;
	struct power_supply *ti_bms_psy;
	struct power_supply *usb_psy;
	struct work_struct iusb_work;
	/* < DTS2014080804732 yuanzhen 20140815 begin */
	/* <DTS2014073007208 jiangfei 20140801 begin */
	struct delayed_work lower_power_charger_work;
	/* DTS2014073007208 jiangfei 20140801 end> */
	/* DTS2014080804732 yuanzhen 20140815 end > */
	int charge_status;
	int charge_full_count;
	/* < DTS2014072101379  yuanzhen 20140728 begin */
	int charge_done_flag;
	bool charge_disable;
/* < DTS2015010904246  mapengfei 20150105 begin */
    int use_only_charge;
    bool soc_resume_charging;
/* DTS2015010904246  mapengfei 20150105 end > */
	/* DTS2014072101379  yuanzhen 20140728 end > */
	/* DTS2014052906550 liyuping 20140530 end> */
};

/*<DTS2014070404260 liyuping 20140704 begin */
typedef enum jeita_thermal_zone
{
	COLD,
	COOL,
	NORMAL,
	WARM,
	HOT,
	ZONE_MAX
}jeita_thermal_zone;

/*<DTS2014080704371 liyuping 20140815 begin */
typedef struct jeita_entry 
{
	int t_low;
	int t_high;
	int i_max;
	int v_max;
	int selected;
	struct jeita_entry * last_zone;
}jeita_entry;
/* DTS2014080704371 liyuping 20140815 end> */

typedef struct jeita_spec
{
	jeita_entry  cold;
	jeita_entry  cool;
	jeita_entry  normal;
	jeita_entry  warm;
	jeita_entry  hot;
}jeita_spec;

/*<DTS2014080704371 liyuping 20140815 begin */
extern jeita_spec jeita_batt_param;
/* DTS2014080704371 liyuping 20140815 end> */
#endif
/* DTS2014061605512 liyuping 20140617 end> */
