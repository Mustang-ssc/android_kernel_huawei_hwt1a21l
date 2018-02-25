/* < DTS2014071108152 yangxiaomei 20140714 begin */
/* < DTS2014042508166 yangxiaomei 20140428 begin */
/*huawei kernel driver for apds993x*/
/*
 * apds993x.c - Linux kernel modules for ambient light + proximity sensor
 *
 * Copyright (C) 2012 Lee Kai Koon <kai-koon.lee@avagotech.com>
 * Copyright (C) 2012 Avago Technologies
 * Copyright (C) 2013 LGE Inc.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/i2c/apds993x_ls.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensors.h>
#include <linux/wakelock.h>
/* < DTS2014041705903 zhangmin 20140418 begin */
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
/* DTS2014041705903 zhangmin 20140418 end > */
 /* <DTS2014112604473 yangzhonghua 20141126 begin */
#ifdef CONFIG_HUAWEI_DSM
/* < DTS2014072201656 yangzhonghua 20140722 begin */
#include 	<linux/dsm_pub.h>
/* DTS2014072201656 yangzhonghua 20140722 end > */
#endif
/* DTS2014112604473 yangzhonghua 20141126 end> */


/* < DTS2014052903087 wanghang 201400604 begin */
#include <misc/app_info.h>
/* DTS2014052903087 wanghang 201400604 end > */

/* < DTS2014061000435 wanghang 20140610 begin */
#include <linux/debugfs.h>
/* DTS2014061000435 wanghang 20140610 end > */
static int origin_prox = 822;
#define APDS993X_DRV_NAME	"apds993x"
#define DRIVER_VERSION		"1.0.0"
/* DTS2014101703500 modified by guoling WX232221 start */
#define USING_8YOUTH_ALGORITHM
/* DTS2014101703500 modified by guoling WX232221 end */

/* < DTS2014062401963  yangzhonghua 20140624 begin */
#define APDS993X_REG_LEN 0x1a
/* DTS2014062401963  yangzhonghua 20140624 end > */

/* Register Value define : CONTROL */

/* < DTS2014073005921 yangxiaomei 20140730 begin */
#define	SENSORS_I2C_SCL	909
#define	SENSORS_I2C_SDA	908
/* DTS2014073005921 yangxiaomei 20140730 end > */
#define APDS993X_I2C_RETRY_COUNT		3 	/* Number of times to retry i2c */
/* < DTS2014082603327 yangxiaomei 20140902 begin */
/*wait more time to try read or write to avoid potencial risk*/
#define APDS993X_I2C_RETRY_TIMEOUT	3	/* Timeout between retry (miliseconds) */
/* DTS2014082603327 yangxiaomei 20140902 end > */

#define APDS993X_I2C_BYTE 0
#define APDS993X_I2C_WORD 1
/* < DTS2014082603327 yangxiaomei 20140902 begin */
/*APDS993X_SUSPEND_ON:suspend already but not resume,phone is sleeping or suspending*/
/*APDS993X_SUSPEND_OFF:resume already,phone is awake*/
#define APDS993X_SUSPEND_ON 1
#define APDS993X_SUSPEND_OFF 0
#define APDS993X_ERROR_HAPPEN 1
#define APDS993X_ERROR_NOTHAPPEN 0
/* DTS2014082603327 yangxiaomei 20140902 end > */
/* < DTS2014090903105 wangcunyou 20140912 begin */
/*keep 400ms wakeup after the ps report the far or near state*/
#define PS_WAKEUP_TIME 700
/* DTS2014090903105 wangcunyou 20140912 end > */
/*it is like min_proximity_value in 8x12 and 8930,to adjust the dynamic proximity ditance*/
static int apds993x_ps_init_threshold = 960;
/* < DTS2014073005921 yangxiaomei 20140730 begin */
/*remove to .h file*/
/* DTS2014073005921 yangxiaomei 20140730 end > */
/*we use different GA for A,D and C light,so there are three GAs*/
/*pwave and pwindow are for pls to realize the adaptive algorithm*/
/* DTS2014101703500 modified by guoling WX232221 start */
#ifndef USING_8YOUTH_ALGORITHM
static int IAC_last = 0;
#endif
/* DTS2014101703500 modified by guoling WX232221 end */
/* < DTS2014073005921 yangxiaomei 20140730 begin */
/*delete it,we use data->enable to save enable state*/
/* DTS2014073005921 yangxiaomei 20140730 end > */
/*delete the wakelock,so system can sleep when the device is in the calling*/
/*als parameters,it is different for every devices*/
static uint32_t apds993x_a_ga = 200;
static uint32_t apds993x_c_ga = 400;
static uint32_t apds993x_d_ga = 300;
/*pls parameters,it is still different for every devices*/
static uint32_t apds993x_pwave_value = 100;
static uint32_t apds993x_pwindow_value = 200;
/* < DTS2014090409459 wanghang 20140904 begin */
extern bool power_key_ps ;    //the value is true means powerkey is pressed, false means not pressed
/* DTS2014090409459 wanghang 20140904 end >*/
/* < DTS2014082603327 yangxiaomei 20140902 begin */
static int apds993x_suspend_flag = APDS993X_SUSPEND_OFF;
static int apds993x_error_flag = APDS993X_ERROR_NOTHAPPEN;
/* DTS2014082603327 yangxiaomei 20140902 end > */
/*dynamic debug mask to control log print,you can echo value to apds993x_debug to control*/
static int apds993x_debug_mask= 0;
module_param_named(apds993x_debug, apds993x_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(apds993x_pwindow, apds993x_pwindow_value, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(apds993x_pwave, apds993x_pwave_value, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(a_ga, apds993x_a_ga, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(c_ga, apds993x_c_ga, int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(d_ga, apds993x_d_ga, int, S_IRUGO | S_IWUSR | S_IWGRP);
#define APDS993X_ERR(x...) do {\
    if (apds993x_debug_mask >=0) \
        printk(KERN_ERR x);\
    } while (0)
/*KERNEL_HWFLOW is for radar using to control all the log of devices*/
#define APDS993X_INFO(x...) do {\
	/* < DTS2014101407653  wuying/wx221431 20141014 begin */\
    if ((apds993x_debug_mask >=0)) \
	/* DTS2014101407653 wuying/wwx221431 20141014 end > */\
        printk(KERN_ERR x);\
    } while (0)
#define APDS993X_FLOW(x...) do {\
    if (KERNEL_HWFLOW && (apds993x_debug_mask >=1)) \
        printk(KERN_ERR x);\
    } while (0)

/* < DTS2014121803938 yangzhonghua 20141218 begin */

enum apds_check_type{
	APDS_NO_IRQ = 0,
	APDS_WRONG_THRES,
};

/*
 * Structs
 */
 struct ls_test_excep{
	int i2c_scl_val;		/* when i2c transfer err, read the gpio value*/
	int i2c_sda_val;
	/* < DTS2014072201656 yangzhonghua 20140722 begin */
	int vdd_mv;
	int vdd_status;
	int vio_mv;
	int vio_status;
	int i2c_err_num;
	int excep_num;
	/* DTS2014072201656 yangzhonghua 20140722 end > */
	atomic_t ps_enable_flag;
	int ps_enable_cnt;
	int ps_report_flag;		/*check if this is one irq occur or not after enable ps */
	int last_high_threshold;
	int last_low_threshold;
	int cur_high_threshold;
	int cur_low_threshold;
	int check_type;
	int irq_val;
	struct mutex dsm_lock;
	char *reg_buf;
};
struct apds993x_data {
	struct i2c_client *client;
	/*to protect the i2c read and write operation*/
	struct mutex update_lock;
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	/*to protect only one thread to control the device register*/
	struct mutex single_lock;
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	/* < DTS2014051304400 caowei 201400513 begin */
	struct work_struct	dwork;		/* for PS interrupt */
	struct work_struct	als_dwork;	/* for ALS polling */
	/* DTS2014051304400 caowei 201400513 end >*/
	/* < DTS2014090409459 wanghang 20140904 begin */
	struct delayed_work    powerkey_work;
	/* DTS2014090409459 wanghang 20140904 end >*/
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;

	/* regulator data */
	bool power_on;
	struct regulator *vdd;
	struct regulator *vio;

	/* pinctrl data*/
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	/*sensor class for als and ps*/
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
	struct apds993x_platform_data *platform_data;
	/*for capture the i2c and other errors*/
	struct ls_test_excep ls_test_exception;
	struct delayed_work dsm_work;
	/* < DTS2015012102106 wanghang 20150128 begin */
	struct delayed_work dsm_irq_work;
	/* DTS2015012102106 wanghang 20150128 end > */

	int irq;
	/* < DTS2014110303514 wanghang 20141104 begin */
	int count;		/* the interrupt counter*/
	/* DTS2014110303514 wanghang 20141104 end >*/
	/*hrtimer is removed from platform struct to here*/
	/* < DTS2014051304400 caowei 201400513 begin */
	struct hrtimer timer;
	/* DTS2014051304400 caowei 201400513 end >*/
	/*wake lock for not losing ps event reporting*/
	struct wake_lock ps_report_wk;
	unsigned int enable;
	unsigned int atime;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppcount;
	unsigned int control;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;
	/*to record the open or close state of als before suspend*/
	unsigned int enable_als_state;
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	/*delete it,we use data->ps_detection to save ps close or far state*/
	/* DTS2014073005921 yangxiaomei 20140730 end > */

	/* PS parameters */
	unsigned int ps_min_threshold; 	/*it is the min_proximity_value */
	unsigned int ps_detection;		/* 5 = near-to-far; 0 = far-to-near */
	unsigned int ps_data;/* to store PS data,it is pdata */

	/* ALS parameters */
	unsigned int als_threshold_l;	/* low threshold */
	unsigned int als_threshold_h;	/* high threshold */
	unsigned int als_data;		/* to store ALS data from CH0 or CH1*/
	int als_prev_lux;		/* to store previous lux value */
	unsigned int als_poll_delay;	/* needed for light sensor polling : micro-second (us) */
	/* < DTS2014082103954  yangzhonghua 20140821 begin */
	bool device_exist;
	/* DTS2014082103954  yangzhonghua 20140821 end > */
};
/* DTS2014121803938 yangzhonghua 20141218 end > */
static struct sensors_classdev sensors_light_cdev = {
	.name = "apds9930-light",
	.vendor = "avago",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "10000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
/* < DTS2014112709755 suzhanzheng 20141128 begin */
static struct sensors_classdev sensors_proximity_cdev = {
	.name = "apds9930-proximity",
	.vendor = "avago",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "1",
	.resolution = "1.0",
	.sensor_power = "3",
	.min_delay = 0, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

/* DTS2014112709755 suzhanzheng 20141128 end > */

/*
 * Global data
 */
/*delete two global data to avoid potencial risks*/
/* < DTS2014051304400 caowei 201400513 begin */
/* Proximity sensor use this work queue to report data */
/* < DTS2014052606323 wanghang 20140526 begin */
static struct workqueue_struct *apds993x_workqueue = NULL;
/* DTS2014052606323 wanghang 20140526  end > */
/* DTS2014051304400 caowei 201400513 end >*/
/*changeable als gain and ADC time,we don't use*/

/* ALS tuning,they are used to calculate lux*/
/* BEGIN PN:  DTS2014052808848,Added by z84020826, 2014/05/29*/
static int apds993x_coe_b = 2117;
static int apds993x_coe_c = 76;
static int apds993x_coe_d = 78;
static int apds993x_coe_e = 217;
static int apds993x_coe_f = 50;
/* END PN:DTS2014052808848 ,Added by z84020826, 2014/05/29*/
/* < DTS2014061000435 wanghang 20140610 begin */
static int cur_pthreshold_h=0;
static int cur_pthreshold_l=0;
/* DTS2014061000435 wanghang 20140610 end > */

/* < DTS2014061100702 wanghang 20140621 begin */
static int far_init=549;
static int near_init=550;
/* DTS2014061100702 wanghang 20140621 end > */

/*
 * Management functions,they are used to set registers in booting and enable
 */
/*init the register of device function for probe and every time the chip is powered on*/
static int apds993x_init_client(struct i2c_client *client);

/* < DTS2014110303514 wanghang 20141104 begin */
void operate_irq(struct apds993x_data *data, int enable, bool sync);
/* DTS2014110303514 wanghang 20141104 end >*/
/* < DTS2014121803938 yangzhonghua 20141218 begin */

/* <DTS2014112604473 yangzhonghua 20141126 begin */
#ifdef CONFIG_HUAWEI_DSM

/* < DTS2014072201656 yangzhonghua 20140722 begin */
static struct dsm_client *apds993x_lps_dclient = NULL;

/* dsm client for lp-sensor */
static struct dsm_dev dsm_lps_apds = {
	.name 		= CLIENT_NAME_LPS_APDS,		// dsm client name
	.fops 		= NULL,						// options
	.buff_size 	= DSM_SENSOR_BUF_MAX,			// buffer size
};


/* DTS2014072201656 yangzhonghua 20140722 end > */

static int apds993x_i2c_read(struct i2c_client*client, u8 reg,bool flag);
static ssize_t apds993x_print_reg_buf(struct device *dev,
			struct device_attribute *attr, char *buf);
static ssize_t apds993x_write_reg(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count);
static struct device_attribute apds_show_regs =
		__ATTR(dump_reg, 0440, apds993x_print_reg_buf, apds993x_write_reg);

static int apds_dsm_report_i2c(struct apds993x_data *data);
static int apds_dsm_report_err(int errno,struct apds993x_data *data);

static void apds_dsm_read_regs(struct apds993x_data *data)
{
	struct ls_test_excep *excep = &data->ls_test_exception;

	/*
	* read all regs to buf
	*/
	apds993x_print_reg_buf(&data->client->dev,&apds_show_regs, excep->reg_buf);
}

static void apds_dsm_save_threshold(struct apds993x_data *data, int high, int low)
{
	struct ls_test_excep *excep = &data->ls_test_exception;

	mutex_lock(&excep->dsm_lock);
	excep->last_high_threshold = high;
	excep->last_low_threshold = low;
	mutex_unlock(&excep->dsm_lock);
}

/* < DTS2015012102106 wanghang 20150128 begin */
/*************************************************
  Function:        apds_dsm_irq_excep_work
  Description:    this func is to report dsm err, no irq occured
                       after ps enabled
  Input:            work
  Output:          none
  Return:          0
*************************************************/
static void apds_dsm_irq_excep_work(struct work_struct *work)
{
	struct apds993x_data *data =
		container_of((struct delayed_work *)work, struct apds993x_data, dsm_irq_work);

	struct ls_test_excep *excep = &data->ls_test_exception;

			if(!excep->ps_report_flag){
				/*
				* report dsm err, no irq occured after ps enabled.
				*/
				apds_dsm_report_err(DSM_LPS_ENABLED_IRQ_ERROR,data);

			}
}
/* DTS2015012102106 wanghang 20150128 end > */

static void apds_dsm_excep_work(struct work_struct *work)
{
	struct apds993x_data *data =
		container_of((struct delayed_work *)work, struct apds993x_data, dsm_work);
	struct i2c_client *client = data->client;

	struct ls_test_excep *excep = &data->ls_test_exception;
	/* < DTS2015012102106 wanghang 20150128 begin */
	int high_threshold;
	int low_threshold;

	/*
	*	read high and low threshold, and compare them with old valus.
	*/
	low_threshold = apds993x_i2c_read(client,APDS993X_PILTL_REG,APDS993X_I2C_WORD);
	high_threshold = apds993x_i2c_read(client,APDS993X_PIHTL_REG,APDS993X_I2C_WORD);
	if(high_threshold == excep->last_high_threshold
		&& low_threshold == excep->last_low_threshold ){
		/*
		* report dsm err, high and low threshold don't changed after ps irq.
		*/
		apds_dsm_report_err(DSM_LPS_THRESHOLD_ERROR,data);
	}

	apds_dsm_save_threshold(data, high_threshold, low_threshold);
	/* DTS2015012102106 wanghang 20150128 end > */
}


static void apds_dsm_no_update_threhold_check(struct apds993x_data *data)
{
	data->ls_test_exception.check_type = APDS_WRONG_THRES;

	schedule_delayed_work(&data->dsm_work, msecs_to_jiffies(20));
}

static void apds_dsm_no_irq_check(struct apds993x_data *data)
{
	struct ls_test_excep *excep = &data->ls_test_exception;


	/* add this code segment to enable ps func
	*	irq gpio status
	*/
	atomic_set(&excep->ps_enable_flag, 1);

	data->ls_test_exception.check_type = APDS_NO_IRQ;

	/* < DTS2015012102106 wanghang 20150128 begin */
	schedule_delayed_work(&data->dsm_irq_work, HZ);
	/* DTS2015012102106 wanghang 20150128 end > */
}


static void apds_dsm_change_ps_enable_status(struct apds993x_data *data)
{
	/*
	*	add this code segment to report ps event.
	*/
	if (atomic_cmpxchg(&data->ls_test_exception.ps_enable_flag, 1, 0)){
		data->ls_test_exception.ps_report_flag = 1;
	}
}


/*
* if i2c transfer error, we check sda/scl value and regulator's value
*/
/* < DTS2014072201656 yangzhonghua 20140722 begin */
static int apds_dump_i2c_exception_status(struct apds993x_data *data)
{
	int ret = 0;
	/* print pm status and i2c gpio status*/
	struct ls_test_excep *excep = &data->ls_test_exception;

	if (data->vdd == NULL) {
		return -ENXIO;
	}

	if (data->vio == NULL) {
		return -ENXIO;
	}

	/* read i2c_sda i2c_scl gpio value*/
	mutex_lock(&data->update_lock);
	excep->i2c_scl_val = gpio_get_value(SENSORS_I2C_SCL);
	excep->i2c_sda_val = gpio_get_value(SENSORS_I2C_SDA);
	mutex_unlock(&data->update_lock);

	/* get regulator's status*/
	excep->vdd_status = regulator_is_enabled(data->vdd);
	/* < DTS2014082603327 yangxiaomei 20140902 begin */
	if(excep->vdd_status < 0){
		APDS993X_ERR("%s,line %d:regulator_is_enabled vdd failed\n",__func__,__LINE__);
	}
	/* DTS2014082603327 yangxiaomei 20140902 end > */
	excep->vio_status = regulator_is_enabled(data->vio);
	if(excep->vio_status < 0){
		APDS993X_ERR("%s,line %d:regulator_is_enabled vio failed\n",__func__,__LINE__);
	}

	/* get regulator's value*/
	excep->vdd_mv = regulator_get_voltage(data->vdd)/1000;
	if(excep->vdd_mv < 0){
		APDS993X_ERR("%s,line %d:regulator_get_voltage vdd failed\n",__func__,__LINE__);
	}

	excep->vio_mv = regulator_get_voltage(data->vio)/1000;
	if(excep->vio_mv < 0){
		APDS993X_ERR("%s,line %d:regulator_get_voltage vio failed\n",__func__,__LINE__);
	}

	/* report i2c err info */
	ret = apds_dsm_report_err(DSM_LPS_I2C_ERROR,data);

	APDS993X_INFO("%s,line %d:i2c_scl_val=%d,i2c_sda_val=%d,vdd = %d, vdd_status = %d\n"
			"vio=%d, vio_status=%d, excep_num=%d, i2c_err_num=%d",__func__,__LINE__
			,excep->i2c_scl_val,excep->i2c_sda_val,excep->vdd_mv,excep->vdd_status
			,excep->vio_mv,excep->vio_status,excep->excep_num,excep->i2c_err_num);

	excep->i2c_err_num = 0;

	return ret;

}
/* DTS2014072201656 yangzhonghua 20140722 end > */
static void apds_report_i2c_info(struct apds993x_data* data, int ret)
{
	data->ls_test_exception.i2c_err_num = ret;
	apds_dump_i2c_exception_status(data);
}

static int apds_dsm_init(struct apds993x_data *data)
{
	apds993x_lps_dclient = dsm_register_client(&dsm_lps_apds);
	if (!apds993x_lps_dclient) {
		APDS993X_ERR("%s@%d register dsm apds993x_lps_dclient failed!\n",__func__,__LINE__);
		return -ENOMEM;
	}

	apds993x_lps_dclient->driver_data = data;

	data->ls_test_exception.reg_buf = kzalloc(512, GFP_KERNEL);
	if(!data->ls_test_exception.reg_buf){
		APDS993X_ERR("%s@%d alloc dsm reg_buf failed!\n",__func__,__LINE__);
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&data->dsm_work, apds_dsm_excep_work);
	/* < DTS2015012102106 wanghang 20150128 begin */
	INIT_DELAYED_WORK(&data->dsm_irq_work, apds_dsm_irq_excep_work);
	/* DTS2015012102106 wanghang 20150128 end > */

	mutex_init(&data->ls_test_exception.dsm_lock);

	return 0;
}

static void apds_dsm_exit(void)
{
	dsm_unregister_client(apds993x_lps_dclient,&dsm_lps_apds);
}

static int apds_dsm_report_i2c(struct apds993x_data *data)
{
	struct ls_test_excep *excep = &data->ls_test_exception;

	ssize_t size = 0;

	size = dsm_client_record(apds993x_lps_dclient,
				"i2c_scl_val=%d,i2c_sda_val=%d,vdd = %d, vdd_status = %d\n"
				"vio=%d, vio_status=%d, excep_num=%d, i2c_err_num=%d\n"
				,excep->i2c_scl_val,excep->i2c_sda_val,excep->vdd_mv,excep->vdd_status
				,excep->vio_mv,excep->vio_status,excep->excep_num,excep->i2c_err_num);

	/* < DTS2014082103954  yangzhonghua 20140821 begin */
	/*if device is not probe successfully or client is null, don't notify dsm work func*/
	if(data->device_exist == false || apds993x_lps_dclient == NULL){
		return -ENODEV;
	}
	/* DTS2014082103954  yangzhonghua 20140821 end > */

	return size;

}

static int apds_dsm_report_wrong_irq(struct apds993x_data *data)
{
	struct ls_test_excep *excep = &data->ls_test_exception;
	int irq_gpio = data->platform_data->irq_gpio;
	ssize_t size;


	/*
	*	read  regs and irq gpio
	*/
	apds_dsm_read_regs(data);
	excep->irq_val = gpio_get_value(irq_gpio);


	size = dsm_client_record(apds993x_lps_dclient,"irq_pin = %d\n regs:%s \n",
		excep->irq_val, excep->reg_buf);

	APDS993X_ERR("dsm error-> irq_pin = %d\n regs:%s\n",
				excep->irq_val, excep->reg_buf);


	return 0;

}

static int apds_dsm_report_no_irq(struct apds993x_data *data)
{
	ssize_t size = 0;

	size = apds_dsm_report_wrong_irq(data);

	return size;
}

static int apds_dsm_report_not_change_threshold(struct apds993x_data *data)
{
	struct ls_test_excep *excep = &data->ls_test_exception;
	ssize_t size = 0;

	apds_dsm_read_regs(data);
	excep->irq_val = gpio_get_value(data->platform_data->irq_gpio);

	size = dsm_client_record(apds993x_lps_dclient, "irq_pin = %d high_thrhd = %d, low_thrhd = %d\n regs:%s",
							 excep->irq_val, excep->last_high_threshold,
							 excep->last_low_threshold, excep->reg_buf);

	APDS993X_ERR("dsm error->""irq_pin = %d high_thrhd = %d, low_thrhd = %d\n regs:%s",
							 excep->irq_val, excep->last_high_threshold,
							 excep->last_low_threshold, excep->reg_buf);

	return size;
}


static int apds_dsm_report_err(int errno,struct apds993x_data *data)
{
	int size = 0;

	if(dsm_client_ocuppy(apds993x_lps_dclient))
	{
		/* buffer is busy */
		APDS993X_ERR("%s: buffer is busy!, errno = %d\n", __func__,errno);
		return -EBUSY;
	}

	APDS993X_INFO("dsm error, errno = %d \n", errno);

	switch(errno){
		case DSM_LPS_I2C_ERROR:
			size = apds_dsm_report_i2c(data);
			break;

		case DSM_LPS_WRONG_IRQ_ERROR:
			size = apds_dsm_report_wrong_irq(data);
			break;

		case DSM_LPS_THRESHOLD_ERROR:
			size = apds_dsm_report_not_change_threshold(data);
			break;

		case DSM_LPS_ENABLED_IRQ_ERROR:
			size = apds_dsm_report_no_irq(data);
			break;
		default:

			break;

	}

	dsm_client_notify(apds993x_lps_dclient, errno);


	return size;
}
/* DTS2014121803938 yangzhonghua 20141218 end > */

#endif
/* DTS2014112604473 yangzhonghua 20141126 end> */


static int apds993x_set_command(struct i2c_client *client, int command)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	int clearInt;
	int loop;

	if (command == 0)
		clearInt = CMD_CLR_PS_INT;
	else if (command == 1)
		clearInt = CMD_CLR_ALS_INT;
	else
		clearInt = CMD_CLR_PS_ALS_INT;
	loop = APDS993X_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation if i2c errors happen*/
	while(loop){
		mutex_lock(&data->update_lock);
		ret = i2c_smbus_write_byte(client, clearInt);
		mutex_unlock(&data->update_lock);

		if(ret < 0){
			loop--;
			mdelay(APDS993X_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}
	/*after three times,we print the register and regulator value*/
	if(loop == 0){
		/* <DTS2014112604473 yangzhonghua 20141126 begin */
#ifdef CONFIG_HUAWEI_DSM
	apds_report_i2c_info(data,ret);
#endif
		/* DTS2014112604473 yangzhonghua 20141126 end> */
		APDS993X_ERR("%s,line %d:attention: i2c write err = %d\n",__func__,__LINE__,ret);
	}
	APDS993X_FLOW("%s,line %d:command = %d\n",__func__,__LINE__,command);
	return ret;
}
/*we use the unified the function for i2c write and read operation*/
static int apds993x_i2c_write(struct i2c_client*client, u8 reg, u16 value,bool flag)
{
	int err,loop;

	struct apds993x_data *data = i2c_get_clientdata(client);

	loop = APDS993X_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation if i2c errors happen*/
	while(loop) {
		mutex_lock(&data->update_lock);
		/*0 is i2c_smbus_write_byte_data,1 is i2c_smbus_write_word_data*/
		if(flag == APDS993X_I2C_BYTE)
		{
			err = i2c_smbus_write_byte_data(client, CMD_BYTE|reg, (u8)value);
		}
		else if(flag == APDS993X_I2C_WORD)
		{
			err = i2c_smbus_write_word_data(client, CMD_WORD|reg, value);
		}
		else
		{
			APDS993X_ERR("%s,line %d:attention: i2c write wrong flag\n",__func__,__LINE__);
			mutex_unlock(&data->update_lock);
			return -EINVAL;
		}
		mutex_unlock(&data->update_lock);
		if(err < 0){
			loop--;
			mdelay(APDS993X_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}
	/*after three times,we print the register and regulator value*/
	if(loop == 0){
		/* <DTS2014112604473 yangzhonghua 20141126 begin */
		APDS993X_ERR("%s,line %d:attention:i2c write err = %d\n",__func__,__LINE__,err);
#ifdef CONFIG_HUAWEI_DSM
		apds_report_i2c_info(data,err);
#endif
		/* DTS2014112604473 yangzhonghua 20141126 end> */

	}

	return err;
}

static int apds993x_i2c_read(struct i2c_client*client, u8 reg,bool flag)
{
	int err,loop;

	struct apds993x_data *data = i2c_get_clientdata(client);

	loop = APDS993X_I2C_RETRY_COUNT;
	/*we give three times to repeat the i2c operation if i2c errors happen*/
	while(loop) {
		mutex_lock(&data->update_lock);
		/*0 is i2c_smbus_read_byte_data,1 is i2c_smbus_read_word_data*/
		if(flag == APDS993X_I2C_BYTE)
		{
			err = i2c_smbus_read_byte_data(client, CMD_BYTE|reg);
		}
		else if(flag == APDS993X_I2C_WORD)
		{
			err = i2c_smbus_read_word_data(client, CMD_WORD|reg);
		}
		else
		{
			APDS993X_ERR("%s,line %d:attention: i2c read wrong flag\n",__func__,__LINE__);
			mutex_unlock(&data->update_lock);
			return -EINVAL;
		}
		mutex_unlock(&data->update_lock);
		if(err < 0){
			loop--;
			mdelay(APDS993X_I2C_RETRY_TIMEOUT);
		}
		else
			break;
	}
	/*after three times,we print the register and regulator value*/
	if(loop == 0){
		/* <DTS2014112604473 yangzhonghua 20141126 begin */
		APDS993X_ERR("%s,line %d:attention: i2c read err = %d,reg=0x%x\n",__func__,__LINE__,err,reg);
#ifdef CONFIG_HUAWEI_DSM
		apds_report_i2c_info(data,err);
#endif
		/* DTS2014112604473 yangzhonghua 20141126 end> */

	}

	return err;
}

/* < DTS2014062401963  yangzhonghua 20140624 begin */
/*
*	print the registers value with proper format
*/
static int dump_reg_buf(struct apds993x_data *data,char *buf, int size,int enable)
{
	int i=0;

	mutex_lock(&data->update_lock);

	if(enable)
		printk("[enable]");
	else
		printk("[disable]");
	printk(" reg_buf= ");
	for(i = 0;i < size; i++){
		printk("0x%2x  ",buf[i]);
	}
	mutex_unlock(&data->update_lock);

	printk("\n");
	return 0;
}
static int apds933x_regs_debug_print(struct apds993x_data *data,int enable)
{
	int i=0;
	char reg_buf[APDS993X_REG_LEN];
	struct i2c_client *client = data->client;
	struct apds993x_platform_data *pdata = data->platform_data;

	/* read registers[0x0~0x1a] value*/
	for(i = 0; i < APDS993X_REG_LEN; i++ )
	{

		reg_buf[i] = apds993x_i2c_read(client,i,APDS993X_I2C_BYTE);

		if(reg_buf[i] <0){
			APDS993X_ERR("%s,line %d:read %d reg failed\n",__func__,__LINE__,i);
			return reg_buf[i] ;
		}
	}

	/* print the registers[0x0~0x1a] value in proper format*/
	dump_reg_buf(data,reg_buf,APDS993X_REG_LEN,enable);

	/* print some important parameters*/
	APDS993X_INFO("parameters: a=%d, c=%d, d=%d,e=%d,f=%d\n",
		pdata->ga_a_value,pdata->ga_c_value,pdata->ga_d_value,pdata->ga_e_value,pdata->ga_f_value);

	return 0;
}

/* DTS2014062401963  yangzhonghua 20140624 end > */


static int apds993x_set_enable(struct i2c_client *client, int enable)
{
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	int ret;

	ret = apds993x_i2c_write(client, APDS993X_ENABLE_REG, enable,APDS993X_I2C_BYTE);
	if (ret < 0){
		APDS993X_ERR("%s,line %d:i2c error,enable = %d\n",__func__,__LINE__,enable);
		return ret;
	}
	/*remove it,we have save the value of enable in another way*/
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	APDS993X_FLOW("%s,line %d:apds993x enable = %d\n",__func__,__LINE__,enable);
	return ret;
}

static int apds993x_set_atime(struct i2c_client *client, int atime)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = apds993x_i2c_write(client,APDS993X_ATIME_REG, atime,APDS993X_I2C_BYTE);
	if (ret < 0){
		APDS993X_ERR("%s,line %d:i2c error,atime = %d\n",__func__,__LINE__,atime);
		return ret;
	}

	data->atime = atime;
	APDS993X_FLOW("%s,line %d:apds993x atime = %d\n",__func__,__LINE__,atime);
	return ret;
}

static int apds993x_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = apds993x_i2c_write(client,APDS993X_PTIME_REG, ptime,APDS993X_I2C_BYTE);
	if (ret < 0){
		APDS993X_ERR("%s,line %d:i2c error,ptime = %d\n",__func__,__LINE__,ptime);
		return ret;
	}

	data->ptime = ptime;
	APDS993X_FLOW("%s,line %d:apds993x ptime = %d\n",__func__,__LINE__,ptime);
	return ret;
}

static int apds993x_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = apds993x_i2c_write(client,APDS993X_WTIME_REG, wtime,APDS993X_I2C_BYTE);
	if (ret < 0){
		APDS993X_ERR("%s,line %d:i2c error,wtime = %d\n",__func__,__LINE__,wtime);
		return ret;
	}

	data->wtime = wtime;
	APDS993X_FLOW("%s,line %d:apds993x wtime = %d\n",__func__,__LINE__,wtime);
	return ret;
}

static int apds993x_set_pilt(struct i2c_client *client, int threshold)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = apds993x_i2c_write(client,APDS993X_PILTL_REG, threshold,APDS993X_I2C_WORD);
	if (ret < 0){
		APDS993X_ERR("%s,line %d:i2c error,threshold = %d\n",__func__,__LINE__,threshold);
		return ret;
	}

	data->pilt = threshold;
	/* < DTS2014061000435 wanghang 20140610 begin */
	APDS993X_INFO("%s,line %d:set apds993x pilt =%d\n", __func__, __LINE__,threshold);
	/* DTS2014061000435 wanghang 20140610 end > */

	return ret;
}

static int apds993x_set_piht(struct i2c_client *client, int threshold)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = apds993x_i2c_write(client,APDS993X_PIHTL_REG, threshold,APDS993X_I2C_WORD);
	if (ret < 0){
		APDS993X_ERR("%s,line %d:i2c error,threshold = %d\n",__func__,__LINE__,threshold);
		return ret;
	}

	data->piht = threshold;
	/* < DTS2014061000435 wanghang 20140610 begin */
	APDS993X_INFO("%s,line %d:set apds993x piht =%d\n", __func__,__LINE__,threshold);
	/* DTS2014061000435 wanghang 20140610 end > */
	return ret;
}

static int apds993x_set_pers(struct i2c_client *client, int pers)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = apds993x_i2c_write(client,APDS993X_PERS_REG, pers,APDS993X_I2C_BYTE);
	if (ret < 0){
		APDS993X_ERR("%s,line %d:i2c error,pers = %d\n",__func__,__LINE__,pers);
		return ret;
	}

	data->pers = pers;
	APDS993X_FLOW("%s,line %d:apds993x pers = %d\n",__func__,__LINE__,pers);
	return ret;
}

static int apds993x_set_config(struct i2c_client *client, int config)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = apds993x_i2c_write(client,APDS993X_CONFIG_REG, config,APDS993X_I2C_BYTE);
	if (ret < 0){
		APDS993X_ERR("%s,line %d:i2c error,config = %d\n",__func__,__LINE__,config);
		return ret;
	}

	data->config = config;
	APDS993X_FLOW("%s,line %d:apds993x config = %d\n",__func__,__LINE__,config);
	return ret;
}

static int apds993x_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = apds993x_i2c_write(client,APDS993X_PPCOUNT_REG, ppcount,APDS993X_I2C_BYTE);
	if (ret < 0){
		APDS993X_ERR("%s,line %d:i2c error,ppcount = %d\n",__func__,__LINE__,ppcount);
		return ret;
	}

	data->ppcount = ppcount;
	APDS993X_FLOW("%s,line %d:apds993x ppcount = %d\n",__func__,__LINE__,ppcount);
	return ret;
}

static int apds993x_set_control(struct i2c_client *client, int control)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	ret = apds993x_i2c_write(client,APDS993X_CONTROL_REG, control,APDS993X_I2C_BYTE);
	if (ret < 0){
		APDS993X_ERR("%s,line %d:i2c error,control = %d\n",__func__,__LINE__,control);
		return ret;
	}

	data->control = control;
	APDS993X_FLOW("%s,line %d:apds993x control = %d\n",__func__,__LINE__,control);
	return ret;
}

/* DTS2014101703500 modified by guoling WX232221 start */
#ifndef USING_8YOUTH_ALGORITHM
static int LuxCalculation(struct i2c_client *client, int ch0data, int ch1data)
{
	int luxValue=0;
	int IAC1=0;
	int IAC2=0;
	int IAC=0;

	if (ch0data >=  APDS993X_CDATA_MAX || ch1data >= APDS993X_CDATA_MAX)
	{
		luxValue = APDS993X_LUX_MAX;
		return luxValue;
	}

	/* re-adjust COE_B to avoid 2 decimal point */
	/* BEGIN PN:  DTS2014052808848,Added by z84020826, 2014/05/29*/
	IAC1 = (ch0data - (apds993x_coe_b* ch1data) / 1000);
	/* re-adjust COE_C and COE_D to void 2 decimal point */
	IAC2 = ((apds993x_coe_c * ch0data) / 1000 -(apds993x_coe_d * ch1data) / 1000);
	/* END PN:  DTS2014052808848,Added by z84020826, 2014/05/29*/

	if (IAC1 > IAC2)
	{
		IAC = IAC1;
	}
	else if (IAC1 <= IAC2)
	{
		IAC = IAC2;
	}
	else
	{
		IAC = 0;
	}

	if(IAC < 0)
	{
		IAC =IAC_last;
	}
	else
	{
		IAC_last = IAC;
	}
	/*In different color temprature light,we use different GA to calculate lux*/
	/*A light*/
	/* BEGIN PN:  DTS2014052808848,Added by z84020826, 2014/05/29*/
	if(ch0data < apds993x_coe_e*ch1data/100)
	{
		luxValue = (IAC*apds993x_a_ga*APDS993X_DF)/100;
	}
	/*D light*/
	else if((ch0data >= apds993x_coe_e*ch1data/100) && (ch0data <= apds993x_coe_f*ch1data/10))
	/* END PN:  DTS2014052808848,Added by z84020826, 2014/05/29*/
	{
		luxValue = (IAC*apds993x_d_ga*APDS993X_DF)/100;
	}
	/*C light*/
	else
	{
		luxValue = (IAC*apds993x_c_ga*APDS993X_DF)/100;
	}
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	/*we use 100ms adc time,so the corresponding again is 1x*/
	luxValue /= (272*(256-APDS993X_100MS_ADC_TIME)*APDS993X_AGAIN_1X_LUXCALCULATION/100);
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	APDS993X_FLOW("%s,line %d:ch0 = %d,ch1=%d,lux = %d\n",__func__,__LINE__,ch0data,ch1data,luxValue);
	return (luxValue);
}
#else

/* Coefficients in open air:
  * GA:Glass (or Lens) Attenuation Factor
  * DF:Device Factor
  * alsGain: ALS Gain
  * aTime: ALS Timing
  * ALSIT = 2.72ms * (256 ¡§C ATIME) = 2.72ms * (256-0xDB) =  100ms
  */
static int aTime = 0xDB;
static int alsGain = 1;
static int coe_b=1860;
static int coe_c=750;
static int coe_d=1290;
static int DF=52;

static int LuxCalculation(struct i2c_client *client, int cdata, int irdata)
{
    /* < DTS2011081702329  chenziqi 20140526 begin */
    int luxValue=0;
    /* < DTS2011050303216 wangjiongfeng 20110504 begin */
    //int first_half, sec_half;
    /* < DTS2011050303216 wangjiongfeng 20110504 end */
    int iac1=0;
    int iac2=0;
    int iac=0;
    int type=0;
    int ga=251;
    /* <DTS2014080406324 chenziqi 20140804 begin */
    if(cdata==0)
        return 0;
    /* DTS2014080406324 chenziqi 20140804 end> */
    type=(irdata*100)/cdata;
    APDS993X_FLOW("%s:enter in luxcalculation function cdata = %d ,irdata = %d\n", __func__, cdata, irdata);
    /*
     *Lux Equation:
     *       IAC1 = CH0DATA -B * CH1DATA                (IAC:IR Adjusted Count)
     *       IAC2 = C * CH0DATA - D * CH1DATA
     *       IAC = Max (IAC1, IAC2, 0)
     *       LPC = GA * DF / (ALSIT * AGAIN)               (LPC:Lux per Count)
     *       Lux = IAC * LPC
     *Coefficients in open air:
     *       GA = 0.48
     *       B = 2.23
     *       C = 0.7
     *       D = 1.42
     */
    /* < DTS2011081101306  liujinggang 20110811 begin */
    /*adjust data by parameter*/
    /* DTS2014040803232 chenziqi 20140531 begin > */
    if(type>30){
        if(irdata<1600)
            ga=251;
        else if(irdata>1800)
            ga=191;
        else
            ga=221;
    }
    else if(irdata<80)
        ga=365;
    else if (irdata>120)
        ga=565;
    else
        ga=455;
    /* DTS2014040803232 chenziqi 20140531 end > */
    iac1 = (int) (cdata - (coe_b*irdata/1000));
    iac2 = (int) ((coe_c*cdata/1000) - (coe_d*irdata/1000));

    /* < DTS2011050303216 wangjiongfeng 20110504 begin */
    if (iac1 > iac2)
    {
        iac = iac1;
    }
    else if (iac1 <= iac2)
    {
        iac = iac2;
    }
    else
    {
        iac = 0;
    }

    /* < DTS2011081702329  liujinggang 20110817 begin */
    //first_half = iac*ga*DF/100;
    /* DTS2011081702329  liujinggang 20110817 end > */
    //sec_half = ((272*(256-aTime))*alsGain)/100;
    //luxValue = first_half/sec_half;
    luxValue = ((iac*ga*DF)/100)/(((272*(256-aTime))/100) *alsGain);
    APDS993X_FLOW("iac=%d,type=%d,luxValue=%d,ga=%d\n",iac,type,luxValue,ga);
    /* < DTS2011050303216 wangjiongfeng 20110504 end */
    /* DTS2011081101306  liujinggang 20110811 end > */
    return luxValue;
}
#endif
/* DTS2014101703500 modified by guoling WX232221 end */

/* < DTS2014073005921 yangxiaomei 20140730 begin */
static void apds993x_dump_register(struct i2c_client *client,int status,int enable)
{
	int wtime,ptime,atime,ppcount,control,config,pers;
	wtime = apds993x_i2c_read(client, APDS993X_WTIME_REG,APDS993X_I2C_BYTE);
	ptime= apds993x_i2c_read(client, APDS993X_PTIME_REG,APDS993X_I2C_BYTE);
	atime =apds993x_i2c_read(client, APDS993X_ATIME_REG,APDS993X_I2C_BYTE);
	ppcount=apds993x_i2c_read(client, APDS993X_PPCOUNT_REG,APDS993X_I2C_BYTE);
	control=apds993x_i2c_read(client, APDS993X_CONTROL_REG,APDS993X_I2C_BYTE);
	config = apds993x_i2c_read(client, APDS993X_CONFIG_REG,APDS993X_I2C_BYTE);
	pers= apds993x_i2c_read(client, APDS993X_PERS_REG,APDS993X_I2C_BYTE);

	APDS993X_INFO("%s,line %d:status = 0x%x,enable=0x%x,wtime=0x%x\n",__func__,__LINE__,status,enable,wtime);
	APDS993X_INFO("%s,line %d:ptime = 0x%x,atime=0x%x,ppcount=0x%x\n",__func__,__LINE__,ptime,atime,ppcount);
	APDS993X_INFO("%s,line %d:control = 0x%x,config=0x%x,pers=0x%x\n",__func__,__LINE__,control,config,pers);
}
/* DTS2014073005921 yangxiaomei 20140730 end > */
static void apds993x_ps_report_event(struct i2c_client *client, int apds_status)
{
	/* < DTS2014090903105 wangcunyou 20140912 begin */
	int ch0data = 0;
	int sunlight_detect = 0;
	/*PS interrupt generation flags*/
	int ps_int_occur = 0;
	struct apds993x_data *data = i2c_get_clientdata(client);

	/* < DTS2014062401963  yangzhonghua 20140624 begin */
	int ret;
	ch0data = apds993x_i2c_read(client, APDS993X_CH0DATAL_REG, APDS993X_I2C_WORD);
	if(ch0data < 0)
	{
		APDS993X_ERR("%s,line %d:apds993x_i2c_read ch0data err:%d\n", __func__, __LINE__, ch0data);
	}
	else if(ch0data > APDS993X_SUNLIGHT_CHODATA)
	{
		sunlight_detect = 0x1;
	}

	APDS993X_INFO("%s, line %d: read status reg data: 0x%x; ch0data: %d, sunlight_detect:%d\n",
		__func__, __LINE__, apds_status, ch0data, sunlight_detect);

	if((apds_status >= 0) && ((apds_status & (APDS993X_PINT)) == 0x20))
	{
		ret = apds993x_i2c_read(client, APDS993X_PDATAL_REG,APDS993X_I2C_WORD);
		if( ret < 0 )
		{
		/* the number "200" is a value to make sure there is a valid value */
			data->ps_data = 200 ;
			APDS993X_ERR("%s, line %d: pdate<0, reset to %d\n", __func__, __LINE__, data->ps_data);
		}else{
			data->ps_data = ret ;
			ps_int_occur = 1;
			APDS993X_FLOW("%s, line %d:read ps data->ps_data:%d, ps_int_occur:%d\n",
				__func__, __LINE__, data->ps_data, ps_int_occur);
		}
		/* < DTS2014121803938 yangzhonghua 20141218 begin */
#ifdef CONFIG_HUAWEI_DSM
		apds_dsm_no_update_threhold_check(data);
#endif
		/* DTS2014121803938 yangzhonghua 20141218 end > */
	}
	else
	{
			/* the number "200" is a value to make sure there is a valid value */
			data->ps_data = 200 ;
			APDS993X_FLOW("%s, line %d:ps intruppt not occur data->ps_data:%d\n",  __func__, __LINE__, data->ps_data);
	}
	APDS993X_FLOW("%s, line %d: read pdata reg data: %d\n",
		__func__, __LINE__, data->ps_data);
	/* < DTS2014082603327 yangxiaomei 20140902 begin */
	/*remove to interrupt function to avoid phone suspend and pls work still running*/
	/* DTS2014082603327 yangxiaomei 20140902 end > */
	/* DTS2014062401963  yangzhonghua 20140624 end > */

	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	APDS993X_FLOW("%s,line %d:apds9930 ps_data=%d,ps_min_threshold=%d, ps_int_occur:%d\n",__func__,__LINE__,
		data->ps_data,data->ps_min_threshold, ps_int_occur);
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	if(((data->ps_data + apds993x_pwave_value) < (data->ps_min_threshold)) && ps_int_occur)
	/* DTS2014090903105 wangcunyou 20140912 end > */
	{
		data->ps_min_threshold = data->ps_data + apds993x_pwave_value;

		ret = apds993x_i2c_write(client,APDS993X_PILTL_REG,data->ps_min_threshold,APDS993X_I2C_WORD);
		ret += apds993x_i2c_write(client,APDS993X_PIHTL_REG,data->ps_min_threshold + apds993x_pwindow_value,APDS993X_I2C_WORD);
		if (ret < 0)
		{
			APDS993X_ERR("%s,line %d:data->pilt = %d,data->piht=%d, i2c wrong\n",__func__,__LINE__,data->pilt,data->piht);
			goto exit;
		}

		data->pilt = data->ps_min_threshold;
		data->piht = data->ps_min_threshold + apds993x_pwindow_value;
		APDS993X_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);
		if(data->pilt != cur_pthreshold_l || data->piht != cur_pthreshold_h)
		{
			cur_pthreshold_h = data->piht;
			cur_pthreshold_l = data->pilt;
			APDS993X_FLOW("%s,line %d:data->ps_data=%d data->pilt=%d data->piht=%d\n", __func__, __LINE__, data->ps_data, data->pilt, data->piht);
		}
	}
	/* < DTS2014090903105 wangcunyou 20140912 begin */
	/* < DTS2014121503091 wanghang 20141218 begin */
	data->pilt = apds993x_i2c_read(client, APDS993X_PILTL_REG, APDS993X_I2C_WORD);
	data->piht = apds993x_i2c_read(client, APDS993X_PIHTL_REG, APDS993X_I2C_WORD);
	if (data->pilt < 0 || data->piht < 0){
		APDS993X_ERR("%s,line %d:data->pilt = %d,data->piht=%d,read i2c wrong\n",__func__,__LINE__,data->pilt,data->piht);
		goto exit;
	}
	/* DTS2014121503091 wanghang 20141218 end > */
	if ((data->ps_data >= data->piht) && (!sunlight_detect) && ps_int_occur) {
	/* DTS2014090903105 wangcunyou 20140912 end > */
		/* < DTS2014073005921 yangxiaomei 20140730 begin */
		/* far-to-near detected */
		data->ps_detection = APDS993X_CLOSE_FLAG;

		/* FAR-to-NEAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, APDS993X_CLOSE_FLAG);
		input_sync(data->input_dev_ps);

		/* < DTS2014121803938 yangzhonghua 20141218 begin */
#ifdef CONFIG_HUAWEI_DSM
		apds_dsm_change_ps_enable_status(data);
#endif
		/* DTS2014121803938 yangzhonghua 20141218 end > */

		/* DTS2014073005921 yangxiaomei 20140730 end > */
		/* < DTS2014061000435 wanghang 20140610 begin */
		APDS993X_INFO("%s,line %d:PROXIMITY close event, data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);
		/* DTS2014061000435 wanghang 20140610 end > */

		ret = apds993x_i2c_write(client,APDS993X_PILTL_REG,data->ps_min_threshold,APDS993X_I2C_WORD);
		ret += apds993x_i2c_write(client,APDS993X_PIHTL_REG, (APDS993X_PROX_MAX_ADC_VALUE + 1),APDS993X_I2C_WORD);
		if (ret < 0){
			APDS993X_ERR("%s,line %d:data->pilt = %d,data->piht=%d, i2c wrong\n",__func__,__LINE__,data->pilt,data->piht);
			goto exit;
		}
		data->pilt = data->ps_min_threshold;
		data->piht = APDS993X_PROX_MAX_ADC_VALUE + 1;
		APDS993X_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);
		/* < DTS2014061000435 wanghang 20140610 begin */
		if(data->pilt != cur_pthreshold_l || data->piht != cur_pthreshold_h)
		{
			cur_pthreshold_h = data->piht;
			cur_pthreshold_l = data->pilt;
			APDS993X_FLOW("%s,line %d:data->ps_data=%d data->pilt=%d data->piht=%d\n", __func__, __LINE__, data->ps_data, data->pilt, data->piht);
		}
		/* DTS2014061000435 wanghang 20140610 end > */
	/* < DTS2014090903105 wangcunyou 20140912 begin */
	} else if (((data->ps_data <= data->pilt)) ||(sunlight_detect)) {
	/* DTS2014090903105 wangcunyou 20140912 end > */
		/* < DTS2014073005921 yangxiaomei 20140730 begin */
		/* near-to-far detected */
		data->ps_detection = APDS993X_FAR_FLAG;

		/* NEAR-to-FAR detection */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, APDS993X_FAR_FLAG);
		input_sync(data->input_dev_ps);
		/* DTS2014073005921 yangxiaomei 20140730 end > */
		/* < DTS2015012102106 wanghang 20150128 begin */
#ifdef CONFIG_HUAWEI_DSM
		apds_dsm_change_ps_enable_status(data);
#endif
		/* DTS2015012102106 wanghang 20150128 end > */

		/* < DTS2014061000435 wanghang 20140610 begin */
		APDS993X_INFO("%s,line %d:PROXIMITY far event, data->ps_data=%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);
		/* DTS2014061000435 wanghang 20140610 end > */

		ret = apds993x_i2c_write(client,APDS993X_PIHTL_REG,
				data->ps_min_threshold + apds993x_pwindow_value,APDS993X_I2C_WORD);
		ret += apds993x_i2c_write(client,APDS993X_PILTL_REG,0,APDS993X_I2C_WORD);
		if (ret < 0){
			APDS993X_ERR("%s,line %d:data->pilt = %d,data->piht=%d, i2c wrong\n",__func__,__LINE__,data->pilt,data->piht);
			goto exit;
		}

		data->piht = data->ps_min_threshold + apds993x_pwindow_value;
		/* < DTS2014061901549  wangcunyou 20140619 begin */
		data->pilt  = 0;
		/* DTS2014061901549 wangcunyou 20140619 end > */
		APDS993X_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);
		/* < DTS2014061000435 wanghang 20140610 begin */
		if(data->pilt != cur_pthreshold_l || data->piht != cur_pthreshold_h)
		{
			cur_pthreshold_h = data->piht;
			cur_pthreshold_l = data->pilt;
			APDS993X_FLOW("%s:data->ps_data=%d data->pilt=%d data->piht=%d\n", __func__, data->ps_data, data->pilt, data->piht);
		}
		/* DTS2014061000435 wanghang 20140610 end > */
		/* < DTS2014090903105 wangcunyou 20140912 begin */
		if(1 == sunlight_detect)
		{
			ret = apds993x_i2c_write(client,APDS993X_ENABLE_REG, data->enable,APDS993X_I2C_BYTE);
			/*To avoid Always generated light interrupt*/
			msleep(300);
		}
		/* DTS2014090903105 wangcunyou 20140912 end > */
	}
	else{
		APDS993X_ERR("%s,line %d:data->ps_data=%d,data->pilt = %d,data->piht=%d,wrong interrupts\n",__func__,__LINE__,data->ps_data, data->pilt,data->piht);
	}
	return ;
exit:
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	/*if i2c error happens,we report far event*/
	if(data->ps_detection == APDS993X_CLOSE_FLAG)
	{
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, APDS993X_FAR_FLAG);
		input_sync(data->input_dev_ps);
		data->ps_detection= APDS993X_FAR_FLAG;
		APDS993X_ERR("%s:i2c error happens, report far event, data->ps_data:%d\n", __func__,data->ps_data);
		/* < DTS2014090903105 wangcunyou 20140912 begin */
		if(1 == sunlight_detect)
		{
			ret = apds993x_i2c_write(client,APDS993X_ENABLE_REG, data->enable,APDS993X_I2C_BYTE);
			/*To avoid Always generated light interrupt*/
			msleep(300);
		}
		/* DTS2014090903105 wangcunyou 20140912 end > */
		return ;
	}
	/* DTS2014073005921 yangxiaomei 20140730 end > */
}

/* < DTS2014051304400 caowei 201400513 begin */
/* delete apds993x_reschedule_work, we use queue_work to replase queue_delayed_work, because flush_delayed_work
   may cause system stop work */
/* DTS2014051304400 caowei 201400513 end >*/
/* ALS polling routine */
static void apds993x_als_polling_work_handler(struct work_struct *work)
{
	struct apds993x_data *data = container_of(work, struct apds993x_data,als_dwork);
	struct i2c_client *client=data->client;
	int ch0data, ch1data, pdata;
	int luxValue=0;

	unsigned char lux_is_valid=1;

	ch0data = apds993x_i2c_read(client,
			APDS993X_CH0DATAL_REG,APDS993X_I2C_WORD);
	ch1data = apds993x_i2c_read(client,
			APDS993X_CH1DATAL_REG,APDS993X_I2C_WORD);
	pdata = apds993x_i2c_read(client,
			APDS993X_PDATAL_REG,APDS993X_I2C_WORD);
	if(ch0data < 0 || ch1data < 0 || pdata < 0)
	{
		/* don't report, this is invalid lux value */
		lux_is_valid = 0;
		luxValue = data->als_prev_lux;
	}
	else
	{
		luxValue = LuxCalculation(client, ch0data, ch1data);
	}

	if (luxValue >= 0)
	{

		luxValue = luxValue < APDS993X_LUX_MAX? luxValue : APDS993X_LUX_MAX;

		data->als_prev_lux = luxValue;
	}
	else
	{
		/* don't report, this is invalid lux value */
		lux_is_valid = 0;
		luxValue = data->als_prev_lux;
	}
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	/*remove it because we use other judge method to decide if pls close event is triggered by sunlight*/
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	if (lux_is_valid) {
		/* report the lux level */
		input_report_abs(data->input_dev_als, ABS_MISC, luxValue);
		input_sync(data->input_dev_als);
		APDS993X_FLOW("%s,line %d:apds9930 lux=%d\n",__func__,__LINE__,luxValue);
	}

	/* restart timer */
	/* < DTS2014051304400 caowei 201400513 begin */
	/* start a work after 200ms */
	if (0 != hrtimer_start(&data->timer,
							ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL) )
	{
		APDS993X_ERR("%s: hrtimer_start fail! nsec=%d\n", __func__, data->als_poll_delay);
	}
	/* DTS2014051304400 caowei 201400513 end >*/
}

/* < DTS2014051304400 caowei 201400513 begin */
/*****************************************************************
Parameters    :  timer
Return        :  HRTIMER_NORESTART
Description   :  hrtimer_start call back function,
				 use to report als data
*****************************************************************/
static enum hrtimer_restart apds993x_als_timer_func(struct hrtimer *timer)
{
	struct apds993x_data* data = container_of(timer,struct apds993x_data,timer);
	/* < DTS2014052606323 wanghang 20140526 begin */
	queue_work(apds993x_workqueue, &data->als_dwork);
	/* DTS2014052606323 wanghang 20140526  end > */
	return HRTIMER_NORESTART;
}
/* DTS2014051304400 caowei 201400513 end >*/

/* PS interrupt routine */

static void apds993x_work_handler(struct work_struct *work)
{
	/* < DTS2014051304400 caowei 201400513 begin */
	struct apds993x_data *data = container_of(work, struct apds993x_data, dwork);
	/* DTS2014051304400 caowei 201400513 end >*/
	struct i2c_client *client=data->client;
	int status;
	int enable;
	int ret;
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	int pdata;
	int ps_saturation_flag;/*1:pls current is over by strong sunlight*/
	mutex_lock(&data->single_lock);
	pdata = apds993x_i2c_read(client, APDS993X_PDATAL_REG,APDS993X_I2C_WORD);
	status = apds993x_i2c_read(client, APDS993X_STATUS_REG,APDS993X_I2C_BYTE);
	enable = apds993x_i2c_read(client, APDS993X_ENABLE_REG,APDS993X_I2C_BYTE);
	if((apds993x_debug_mask > 1)||(pdata == APDS993X_PROX_MAX_ADC_VALUE + 1))
	{
		apds993x_dump_register(client,status,enable);
	}

	/* disable 993x's ADC first */
	ret = apds993x_i2c_write(client, APDS993X_ENABLE_REG, 0x01,APDS993X_I2C_BYTE);
	if (ret < 0){
		APDS993X_ERR("%s,line %d:I2c write error happens\n",__func__,__LINE__);
		/* < DTS2014082603327 yangxiaomei 20140902 begin */
		/*if the I2C error happens,firstly we check if suspend is already used.*/
		apds993x_error_flag = APDS993X_ERROR_HAPPEN;
		if(APDS993X_SUSPEND_ON == apds993x_suspend_flag)
		{
			/*report the opposite pls event to wake up phone,at least we hope so*/
			if(APDS993X_CLOSE_FLAG == data->ps_detection)
			{
				input_report_abs(data->input_dev_ps, ABS_DISTANCE, APDS993X_FAR_FLAG);
				input_sync(data->input_dev_ps);
				APDS993X_ERR("%s,%d:i2c error happens, report far event\n", __func__,__LINE__);
			}
			else if(APDS993X_FAR_FLAG == data->ps_detection)
			{
				input_report_abs(data->input_dev_ps, ABS_DISTANCE, APDS993X_CLOSE_FLAG);
				input_sync(data->input_dev_ps);
				APDS993X_ERR("%s,%d:i2c error happens, report near event\n", __func__,__LINE__);
			}
			else
			{
				/*this condition hardly happens*/
				APDS993X_ERR("%s,%d:i2c error happens, data->ps_detection = %d is wrong\n", __func__,__LINE__,data->ps_detection);
			}
		}
		/* DTS2014082603327 yangxiaomei 20140902 end > */
		mutex_unlock(&data->single_lock);
		return ;
	}
	/* < DTS2014090903105 wangcunyou 20140912 begin */
	if (((status & enable & 0x20) == 0x20) || ((status & enable & 0x10) == 0x10)) {
	/* DTS2014090903105 wangcunyou 20140912 end > */
		/* only PS is interrupted */
		APDS993X_FLOW("%s,line %d:only PLS is detected.\n",__func__,__LINE__);
		/* check if this is triggered by background ambient noise */
		ps_saturation_flag = status & 0x40;
		/* < DTS2014061000435 wanghang 20140610 begin */
		APDS993X_FLOW("%s,line %d:apds993x_ps_report_event, ps_saturation_flag:%d\n", __func__,__LINE__,ps_saturation_flag);
		/* DTS2014061000435 wanghang 20140610 end > */
		/* < DTS2014071802506 wangcunyou 20140718 begin */
		/*In the case of ambient light never turned on, ch0data=0*/
		/*if strong sunlight trigger PS close event,ps_saturation_flag =1 not 0*/
		if (ps_saturation_flag == APDS993X_SATURATION_NOT_OVER)
		/* DTS2014071802506 wangcunyou 20140718 end > */
		{
			/* < DTS2014090903105 wangcunyou 20140912 begin */
			apds993x_ps_report_event(client, status);
			/* DTS2014090903105 wangcunyou 20140912 end > */
		}
		else
		{
			if (data->ps_detection == APDS993X_CLOSE_FLAG)
				/* < DTS2014090903105 wangcunyou 20140912 begin */
				apds993x_ps_report_event(client, status);
				/* DTS2014090903105 wangcunyou 20140912 end > */
			else
				APDS993X_INFO("%s: background ambient noise\n",__func__);
		}
	} else{
		APDS993X_ERR("%s,line %d:wrong interrupts,APDS993X_STATUS_REG is 0X%x\n",__func__,__LINE__,status);
	}

	/* < DTS2014090903105 wangcunyou 20140912 begin */
	/* 2 = CMD_CLR_PS_ALS_INT */
	apds993x_set_command(client, 2);
	/* DTS2014090903105 wangcunyou 20140912 end > */
	ret = apds993x_i2c_write(client,APDS993X_ENABLE_REG, data->enable,APDS993X_I2C_BYTE);
	mutex_unlock(&data->single_lock);
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	/* < DTS2014082603327 yangxiaomei 20140902 begin */
	if (ret < 0)
	{
		APDS993X_ERR("%s,%d:i2c error happens, als and pls disabled,data->enable = %d\n", __func__,__LINE__,data->enable);
		return ;
	}
	/* DTS2014082603327 yangxiaomei 20140902 end > */
	if (data->irq)
	{
		/* < DTS2014110303514 wanghang 20141104 begin */
		operate_irq(data,1,true);
		/* DTS2014110303514 wanghang 20141104 end >*/
	}
}

/* < DTS2014110303514 wanghang 20141104 begin */
void operate_irq(struct apds993x_data *data, int enable, bool sync)
{
	APDS993X_FLOW("%s,line %d:operate irq type enable:%d, data->count:%d.\n",__func__,__LINE__,enable, data->count);
	if (data->irq)
	{
		if(enable)
		{
			enable_irq(data->irq);
			data->count++;
		}
		else
		{
			if(data->count > 0)
			{
				if(sync)
				{
					disable_irq(data->irq);
				}
				else
				{
					disable_irq_nosync(data->irq);
				}
				data->count--;
			}
		}
	}
}
/* DTS2014110303514 wanghang 20141104 end >*/

/* assume this is ISR */
static irqreturn_t apds993x_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct apds993x_data *data = i2c_get_clientdata(client);
	/* < DTS2014082603327 yangxiaomei 20140902 begin */
	/*in 400ms,system keeps in wakeup state to avoid the sleeling system lose the pls event*/
	APDS993X_FLOW("%s,line %d:interrupt is coming.\n",__func__,__LINE__);
	/* < DTS2014110303514 wanghang 20141104 begin */
	operate_irq(data,0,false);
	/* DTS2014110303514 wanghang 20141104 end >*/
	wake_lock_timeout(&data->ps_report_wk, PS_WAKEUP_TIME);
	/* DTS2014082603327 yangxiaomei 20140902 end > */
	/* < DTS2014051304400 caowei 201400513 begin */
	/* and ps data report function to workqueue */
	/* < DTS2014052606323 wanghang 20140526 begin */
	queue_work(apds993x_workqueue, &data->dwork);
	/* DTS2014052606323 wanghang 20140526  end > */
	/* DTS2014051304400 caowei 201400513 end >*/

	return IRQ_HANDLED;
}

/*
 * IOCTL support
 */
static int apds993x_enable_als_sensor(struct i2c_client *client, int val)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	/* < DTS2014051304400 caowei 201400513 begin */
	struct apds993x_platform_data *pdata = data->platform_data;
	/* DTS2014051304400 caowei 201400513 end >*/
	int ret;
	/* < DTS2014071802506 wangcunyou 20140718 begin */
	/*Not in use power_value*/
	/* DTS2014071802506 wangcunyou 20140718 end > */

	APDS993X_FLOW("%s,line %d:enable als val=%d\n",__func__,__LINE__,val);
	/* < DTS2014071802506 wangcunyou 20140718 begin */
	/*Not in use power_value*/
	/* DTS2014071802506 wangcunyou 20140718 end > */
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	mutex_lock(&data->single_lock);
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	if (val == 1) {
		/* turn on light  sensor */
		if (data->enable_als_sensor == 0) {
			if(data->enable_ps_sensor == 0){
				/* Power on and initalize the device */
				if (pdata->power_on)
					pdata->power_on(true,data);

				ret = apds993x_init_client(client);
				if (ret) {
					APDS993X_ERR("%s:line:%d,Failed to init apds993x\n", __func__, __LINE__);
					/* < DTS2014073005921 yangxiaomei 20140730 begin */
					mutex_unlock(&data->single_lock);
					/* DTS2014073005921 yangxiaomei 20140730 end > */
					return ret;
				}
			}
			data->enable_als_sensor = 1;
			/* < DTS2014051304400 caowei 201400513 begin */
			/* < DTS2014071802506 wangcunyou 20140718 begin */
			/* < DTS2014073005921 yangxiaomei 20140730 begin */
			data->enable = (data->enable)|0x03;
			apds993x_set_enable(client, data->enable);
			/* DTS2014071802506 wangcunyou 20140718 end > */
			/* < DTS2014082603327 yangxiaomei 20140902 begin */
			APDS993X_INFO("%s: line:%d enable als sensor,data->enable=0x%x\n", __func__, __LINE__, data->enable);
			/* DTS2014082603327 yangxiaomei 20140902 end > */
			/* DTS2014073005921 yangxiaomei 20140730 end > */
			/* enable als sensor, start data report hrtimer */
			ret = hrtimer_start(&data->timer, ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL);
			if (ret != 0) {
				APDS993X_ERR("%s: hrtimer_start fail! nsec=%d\n", __func__, data->als_poll_delay);
			}
			/* DTS2014051304400 caowei 201400513 end >*/
		}
	} else {
		/*
		 * turn off light sensor
		 * what if the p sensor is active?
		 */
		 if(data->enable_als_sensor == 1)
		 {
			data->enable_als_sensor = 0;
			/* < DTS2014073005921 yangxiaomei 20140730 begin */
			if(data->enable_ps_sensor ==1)
			{
				/* < DTS2014090903105 wangcunyou 20140912 begin */
				/*don't disable als sensor,ps sensor need it to check Ambient light Infra-red noise*/
				APDS993X_INFO("%s: line:%d,don't disable als sensor,ps sensor need it to check Ambient light Infra-red noise\n",
						__func__, __LINE__);
				/* DTS2014090903105 wangcunyou 20140912 end > */
			}
			else
			{
				data->enable = 0x00;
				apds993x_set_enable(client, data->enable);
			}

			APDS993X_INFO("%s: line:%d,disable als sensor,data->enable = 0x%x\n", __func__, __LINE__,data->enable);
			/* < DTS2014051304400 caowei 201400513 begin */
			/* disable als sensor, cancne data report hrtimer */
			hrtimer_cancel(&data->timer);
			cancel_work_sync(&data->als_dwork);
			/* DTS2014073005921 yangxiaomei 20140730 end > */
			/* DTS2014051304400 caowei 201400513 end >*/
		 }

	}
	/* Vote off  regulators if both light and prox sensor are off */
	if ((data->enable_als_sensor == 0) &&(data->enable_ps_sensor == 0) &&(pdata->power_on)){
		pdata->power_on(false,data);
	}
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	mutex_unlock(&data->single_lock);
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	APDS993X_FLOW("%s: line:%d,enable als sensor success\n", __func__, __LINE__);
	return 0;
}
/* < DTS2014071802506 wangcunyou 20140718 begin */
static int apds993x_open_ps_sensor(struct apds993x_data *data, struct i2c_client *client)
/* DTS2014071802506 wangcunyou 20140718 end > */
{
	int ret = 0;
	/* turn on p sensor */
	if (data->enable_ps_sensor==0) {
		/* < DTS2014122400610 wanghang 20141227 begin */
		//if(data->enable_als_sensor == 0){
		/* DTS2014122400610 wanghang 20141227 end > */
			/* Power on and initalize the device */
			if (data->platform_data->power_on)
				data->platform_data->power_on(true,data);

			ret = apds993x_init_client(client);
			if (ret) {
				APDS993X_ERR("%s:line:%d,Failed to init apds993x\n", __func__, __LINE__);
				return ret;
			}
		/* < DTS2014122400610 wanghang 20141227 begin */
		//}
		/* DTS2014122400610 wanghang 20141227 end > */

		data->enable_ps_sensor= 1;
		/*initialize the ps_min_threshold,to update data->piht and data->pilt*/
		data->ps_min_threshold = origin_prox;
		APDS993X_FLOW("%s,line %d:change threshoid,data->ps_min_threshold =%d\n",__func__,__LINE__,data->ps_min_threshold);
		/* < DTS2014061100702 wanghang 20140621 begin */
		ret = apds993x_i2c_write(client,APDS993X_PILTL_REG,far_init,APDS993X_I2C_WORD);
		ret += apds993x_i2c_write(client,APDS993X_PIHTL_REG,near_init,APDS993X_I2C_WORD);
		/* < DTS2014090903105 wangcunyou 20140912 begin */
		/*we use ch0data high thresh hold to check sunlight*/
		ret += apds993x_i2c_write(client,APDS993X_AILTL_REG, 0, APDS993X_I2C_WORD);
		ret += apds993x_i2c_write(client,APDS993X_AIHTL_REG, APDS993X_SUNLIGHT_CHODATA, APDS993X_I2C_WORD);
		/* DTS2014090903105 wangcunyou 20140912 end > */
		if (ret < 0)
			return ret;
		/* DTS2014061100702 wanghang 20140621 end > */
		APDS993X_INFO("%s,line %d:change threshoid,data->ps_data =%d,data->pilt=%d,data->piht=%d,\n", __func__,__LINE__,data->ps_data, data->pilt, data->piht);

		/* < DTS2014121803938 yangzhonghua 20141218 begin */
#ifdef CONFIG_HUAWEI_DSM
		apds_dsm_save_threshold(data, far_init, near_init);
#endif
		/* DTS2014121803938 yangzhonghua 20141218 end > */

		/*we use our own calibration algorithm,more details of the algorithm you can check apds993x_ps_report_event*/
		/* < DTS2014071802506 wangcunyou 20140718 begin */
		/* < DTS2014073005921 yangxiaomei 20140730 begin */
	/* < DTS2014091802363 yangzhonghua 20140918 begin */
		/* < DTS2014092002755 wangcunyou 20140920 begin */
		/*clear als_ps interrupt before enable AP irq*/
		apds993x_set_command(client, 2);
		/* DTS2014092002755 wangcunyou 20140920 end > */
		/* < DTS2014091802363 yangzhonghua 20140918 begin */
		if (data->irq)
		{
			/* < DTS2014110303514 wanghang 20141104 begin */
			operate_irq(data,1,true);
			/* DTS2014110303514 wanghang 20141104 end >*/
			/*set the property of pls irq,so the pls irq can wake up the sleeping system */
			irq_set_irq_wake(data->irq, 1);
		}
	/* DTS2014091802363 yangzhonghua 20140918 end > */

		/* < DTS2014090903105 wangcunyou 20140912 begin */
		/* Infrared detection results under the impact of ambient light , so need open ambient light and enable irq*/
		data->enable = ((data->enable) |0x25 |0x3 | APDS993X_SUNLIGHT_AIEN);
		/* DTS2014090903105 wangcunyou 20140912 end > */
		apds993x_set_enable(client, data->enable);
		APDS993X_INFO("%s: line:%d,enable pls sensor.data->enable = 0x%x\n", __func__, __LINE__,data->enable);
		/* DTS2014071802506 wangcunyou 20140718 end > */
		/* 0 is close, 1 is far */
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, APDS993X_FAR_FLAG);
		input_sync(data->input_dev_ps);
		/* DTS2014073005921 yangxiaomei 20140730 end > */
		/* < DTS2014061000435 wanghang 20140610 begin */
		APDS993X_INFO("%s,line %d:input_report_abs report ABS_DISTANCE, far event, data->ps_data:%d\n", __func__,__LINE__,data->ps_data);
		/* DTS2014061000435 wanghang 20140610 end > */

	/* < DTS2014091802363 yangzhonghua 20140918 begin */
	/* move this codes befor apds993x_set_enable*/
	/* DTS2014091802363 yangzhonghua 20140918 end > */
	}
	return ret;
}
static int apds993x_enable_ps_sensor(struct i2c_client *client,unsigned int val)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	/* < DTS2014071802506 wangcunyou 20140718 begin */
	/*Not in use power_value*/
	/* DTS2014071802506 wangcunyou 20140718 end > */

	APDS993X_FLOW("%s,line %d:val=%d\n",__func__,__LINE__,val);
	if ((val != 0) && (val != 1)) {
		APDS993X_ERR("%s: invalid value=%d\n", __func__, val);
		return -EINVAL;
	}
	/* < DTS2014071802506 wangcunyou 20140718 begin */
	/*Not in use power_value*/
	/* DTS2014071802506 wangcunyou 20140718 end > */
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	if (val == 1) {
		mutex_lock(&data->single_lock);
		/* < DTS2014071802506 wangcunyou 20140718 begin */
		ret = apds993x_open_ps_sensor(data, client);
		/* DTS2014071802506 wangcunyou 20140718 end > */
		mutex_unlock(&data->single_lock);
		if(ret)
		{
			APDS993X_ERR("%s,line %d:read power_value failed,open ps fail\n",__func__,__LINE__);
			return ret;
		}
		/* < DTS2014090409459 wanghang 20140904 begin */

		/* < DTS2014121803938 yangzhonghua 20141218 begin */
#ifdef CONFIG_HUAWEI_DSM
		apds_dsm_no_irq_check(data);
#endif
		/* DTS2014121803938 yangzhonghua 20141218 end > */

		power_key_ps = false;
		schedule_delayed_work(&data->powerkey_work, msecs_to_jiffies(100));
		/* DTS2014090409459 wanghang 20140904 end >*/
	} else {
		/*
		 * turn off p sensor - kk 25 Apr 2011
		 * we can't turn off the entire sensor,
		 * the light sensor may be needed by HAL
		 */
		if (data->enable_ps_sensor==1) {
			mutex_lock(&data->single_lock);
			data->enable_ps_sensor = 0;
			if(data->enable_als_sensor ==1)
			{
				data->enable = (data->enable)&0xcb;
				apds993x_set_enable(client, data->enable);
			}
			else
			{
				data->enable= 0x00;
				apds993x_set_enable(client, data->enable);
			}
			mutex_unlock(&data->single_lock);
			APDS993X_INFO("%s: line:%d,disable pls sensor,data->enable = 0x%x\n", __func__, __LINE__,data->enable);
			/* < DTS2015012103110 wanghang 20150127 begin */
#ifdef CONFIG_HUAWEI_DSM
			apds_dsm_change_ps_enable_status(data);
#endif
			/* DTS2015012103110 wanghang 20150127 end > */
			cancel_work_sync(&data->dwork);
			/* < DTS2014090409459 wanghang 20140904 begin */
			cancel_delayed_work(&data->powerkey_work);
			/* DTS2014090409459 wanghang 20140904 end >*/
			if (data->irq)
			{
				/*when close the pls,make the wakeup property diabled*/
				irq_set_irq_wake(data->irq, 0);
				/* < DTS2014110303514 wanghang 20141104 begin */
				operate_irq(data,0,true);
				/* DTS2014110303514 wanghang 20141104 end >*/
			}
		}
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	}
	/* Vote off  regulators if both light and prox sensor are off */
	if ((data->enable_als_sensor == 0) &&(data->enable_ps_sensor == 0) &&(data->platform_data->power_on)){
		data->platform_data->power_on(false,data);
	}
	return 0;
}
/*
 * SysFS support
 */
 static int apds993x_als_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	/* < DTS2014062401963  yangzhonghua 20140624 begin */
	int ret = 0;
	static int als_enalbe_count=0;
	/* DTS2014062401963  yangzhonghua 20140624 end > */

	struct apds993x_data *data = container_of(sensors_cdev,struct apds993x_data, als_cdev);
	if ((enable != 0) && (enable != 1)) {
		APDS993X_ERR("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}
	APDS993X_FLOW("%s,line %d:apds9930 als enable=%d\n",__func__,__LINE__,enable);

	/* < DTS2014062401963  yangzhonghua 20140624 begin */
	/*for debug and print registers value when enable/disable the als every time*/
	if(enable == 0)
	{
		if(apds993x_debug_mask >= 1){
		APDS993X_FLOW("attention:before als_disable %d times\n", als_enalbe_count);
			apds933x_regs_debug_print(data,enable);
		}
		apds993x_enable_als_sensor(data->client, enable);

	}else{

		apds993x_enable_als_sensor(data->client, enable);

		if(apds993x_debug_mask >= 1){
		APDS993X_FLOW("attention: after als_enable %d times\n",++als_enalbe_count);
			apds933x_regs_debug_print(data,enable);
		}
	 }
	 return ret;
	/* DTS2014062401963  yangzhonghua 20140624 end > */
}

static int apds993x_ps_set_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct apds993x_data *data = container_of(sensors_cdev,
			struct apds993x_data, ps_cdev);
	if ((enable != 0) && (enable != 1)) {
		APDS993X_ERR("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}
	APDS993X_FLOW("%s,line %d:apds9930 enable ps value=(%d)\n",__func__,__LINE__, enable);
	return apds993x_enable_ps_sensor(data->client, enable);
}
/*use this function to reset the poll_delay time(ms),val is the time parameter*/
static int apds993x_set_als_poll_delay(struct i2c_client *client,
		unsigned int val)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	int atime_index;

	/* minimum 10ms */
	if (val < 10)
		val = 10;
	/* < DTS2014071902196 wangcunyou 20140719 begin */
	data->als_poll_delay = 200;
	/*atime value,0xDB is 100.64ms;0xED is 51.68ms;0xF6 is 27.2ms*/
	atime_index = APDS993X_100MS_ADC_TIME;
	/* DTS2014071902196 wangcunyou 20140719 end > */
	ret = apds993x_set_atime(client, atime_index);
	if (ret >= 0) {
		APDS993X_INFO("%s,line %d:poll delay %d, atime_index %d",__func__,__LINE__,
				data->als_poll_delay, atime_index);
	} else {
		APDS993X_ERR("%s,line %d:SET ATIME FAIL,poll delay %d, atime_index %d",__func__,__LINE__,
				data->als_poll_delay, atime_index);
		return ret;
	}
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->als_dwork);
	ret = hrtimer_start(&data->timer, ktime_set(0, data->als_poll_delay * 1000000), HRTIMER_MODE_REL);
	if (ret != 0) {
		APDS993X_ERR("%s,line%d: hrtimer_start fail! nsec=%d\n", __func__, __LINE__,data->als_poll_delay);
		return ret;
	}
	return 0;
}

static int apds993x_als_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct apds993x_data *data = container_of(sensors_cdev,
			struct apds993x_data, als_cdev);
	apds993x_set_als_poll_delay(data->client, delay_msec);
	return 0;
}
static ssize_t apds993x_show_ch0data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ch0data;

	ch0data = apds993x_i2c_read(client,APDS993X_CH0DATAL_REG,APDS993X_I2C_WORD);

/* < DTS2014062401963  yangzhonghua 20140624 begin */
	return snprintf(buf,32,"%d\n", ch0data);
/* DTS2014062401963  yangzhonghua 20140624 end > */
}

static DEVICE_ATTR(ch0data, S_IRUGO, apds993x_show_ch0data, NULL);

static ssize_t apds993x_show_ch1data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ch1data;

	ch1data = apds993x_i2c_read(client,APDS993X_CH1DATAL_REG,APDS993X_I2C_WORD);

/* < DTS2014062401963  yangzhonghua 20140624 begin */
	return snprintf(buf,32, "%d\n", ch1data);
/* DTS2014062401963  yangzhonghua 20140624 end > */
}

static DEVICE_ATTR(ch1data, S_IRUGO, apds993x_show_ch1data, NULL);

static ssize_t apds993x_show_pdata(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int pdata;

	pdata = apds993x_i2c_read(client, APDS993X_PDATAL_REG,APDS993X_I2C_WORD);
	if(pdata <0){
		APDS993X_ERR("%s,line %d:read pdata failed\n",__func__,__LINE__);
	}

/* < DTS2014062401963  yangzhonghua 20140624 begin */
	return snprintf(buf,32, "%d\n", pdata);
/* DTS2014062401963  yangzhonghua 20140624 end > */
}

static DEVICE_ATTR(pdata, S_IRUGO, apds993x_show_pdata, NULL);


/* < DTS2014062401963  yangzhonghua 20140624 begin */
/*
* set the register's value from userspace
* Usage: echo "0x08|0x12" > dump_reg
*			"reg_address|reg_value"
*/
static ssize_t apds993x_write_reg(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds993x_data *data = i2c_get_clientdata(client);
	int val_len_max = 4;
	char* input_str =NULL;
	char reg_addr_str[10]={'\0'};
	char reg_val_str[10]={'\0'};
	long reg_addr,reg_val;
	int addr_lenth=0,value_lenth=0,buf_len=0,ret = -1;
	char* strtok=NULL;

	buf_len = strlen(buf);
	input_str = kzalloc(buf_len, GFP_KERNEL);
	if (!input_str)
	{
		pr_err("%s:kmalloc fail!\n",__func__);
		return -ENOMEM;
	}

	mutex_lock(&data->update_lock);
	snprintf(input_str, 10,"%s", buf);
	/*Split the string when encounter "|", for example "0x08|0x12" will be splited "0x18" "0x12" */
	strtok=strsep(&input_str, "|");
	if(strtok!=NULL)
	{
		addr_lenth = strlen(strtok);
		memcpy(reg_addr_str,strtok,((addr_lenth > (val_len_max))?(val_len_max):addr_lenth));
	}
	else
	{
		pr_err("%s: buf name Invalid:%s", __func__,buf);
		goto parse_fail_exit;
	}
	strtok=strsep(&input_str, "|");
	if(strtok!=NULL)
	{
		value_lenth = strlen(strtok);
		memcpy(reg_val_str,strtok,((value_lenth > (val_len_max))?(val_len_max):value_lenth));
	}
	else
	{
		pr_err("%s: buf value Invalid:%s", __func__,buf);
		goto parse_fail_exit;
	}

	/* transform string to long int */
	ret = kstrtol(reg_addr_str,16,&reg_addr);
	if(ret)
		goto parse_fail_exit;

	ret = kstrtol(reg_val_str,16,&reg_val);
	if(ret)
		goto parse_fail_exit;

	/* write the parsed value in the register*/
	ret = apds993x_i2c_write(client,(char)reg_addr,(char)reg_val,APDS993X_I2C_BYTE);
	/* < DTS2014082603327 yangxiaomei 20140902 begin */
	if (ret < 0){
		goto parse_fail_exit;
	}
	mutex_unlock(&data->update_lock);
	/* DTS2014082603327 yangxiaomei 20140902 end > */
	return count;

parse_fail_exit:
	mutex_unlock(&data->update_lock);
	if (input_str)
		kfree(input_str);

	return ret;
}

/*
* show all registers' value to userspace
*/
static ssize_t apds993x_print_reg_buf(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int i;
	char reg[APDS993X_REG_LEN];
	struct i2c_client *client = to_i2c_client(dev);

	/* read all register value and print to user*/
	for(i = 0; i < APDS993X_REG_LEN; i++ )
	{
		reg[i] = apds993x_i2c_read(client,i,APDS993X_I2C_BYTE);
		if(reg[i] <0){
			APDS993X_ERR("%s,line %d:read %d reg failed\n",__func__,__LINE__,i);
			return reg[i] ;
		}
	}

	return snprintf(buf,512,"reg[0x0~0x8]=0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n"
			"reg[0x09~0x11]0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n"
			"reg[0x12~0x19]0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x, 0x%2x\n",
			reg[0x00],reg[0x01],reg[0x02],reg[0x03],reg[0x04],reg[0x05],reg[0x06],reg[0x07],reg[0x08],
			reg[0x09],reg[0x0a],reg[0x0b],reg[0x0c],reg[0x0d],reg[0x0e],reg[0x0f],reg[0x10],reg[0x11],
			reg[0x12],reg[0x13],reg[0x14],reg[0x15],reg[0x16],reg[0x17],reg[0x18],reg[0x19]);
}

static DEVICE_ATTR(dump_reg ,S_IRUGO|S_IWUSR|S_IWGRP, apds993x_print_reg_buf, apds993x_write_reg);
/* DTS2014062401963  yangzhonghua 20140624 end > */
/* <DTS2014112604473 yangzhonghua 20141126 begin */
#ifdef CONFIG_HUAWEI_DSM

/* < DTS2014072201656 yangzhonghua 20140722 begin */
/*
*  test data or i2c error interface for device monitor
*/
static ssize_t apds993x_sysfs_dsm_test(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds993x_data *data = i2c_get_clientdata(client);
	long mode;
	int ret = 0;

	if (strict_strtol(buf, 10, &mode))
			return -EINVAL;

	/* < DTS2014121803938 yangzhonghua 20141218 begin */
	switch(mode){
		case DSM_LPS_I2C_ERROR:
			ret = apds_dump_i2c_exception_status(data);
			break;
		case DSM_LPS_WRONG_IRQ_ERROR:
		case DSM_LPS_THRESHOLD_ERROR:
		case DSM_LPS_ENABLED_IRQ_ERROR:
			ret = apds_dsm_report_err(mode, data);
			break;

		default:
			APDS993X_ERR("%s unsupport err_no = %ld \n", __func__, mode);
			break;

	}
	/* DTS2014121803938 yangzhonghua 20141218 end > */

	return ret;
}

static DEVICE_ATTR(dsm_excep,S_IWUSR|S_IWGRP, NULL, apds993x_sysfs_dsm_test);
/* DTS2014072201656 yangzhonghua 20140722 end > */
#endif
/* DTS2014112604473 yangzhonghua 20141126 end> */

static struct attribute *apds993x_attributes[] = {
	&dev_attr_ch0data.attr,
	&dev_attr_ch1data.attr,
	&dev_attr_pdata.attr,
	/* < DTS2014062401963  yangzhonghua 20140624 begin */
	&dev_attr_dump_reg.attr,
	/* <DTS2014112604473 yangzhonghua 20141126 begin */
#ifdef CONFIG_HUAWEI_DSM
	/* < DTS2014072201656 yangzhonghua 20140722 begin */
	&dev_attr_dsm_excep.attr,
	/* DTS2014072201656 yangzhonghua 20140722 end > */
#endif
	/* DTS2014112604473 yangzhonghua 20141126 end> */
	/* DTS2014062401963  yangzhonghua 20140624 end > */
	NULL
};

static const struct attribute_group apds993x_attr_group = {
	.attrs = apds993x_attributes,
};

/*
 * Initialization function
 */
static int apds993x_read_device_id(struct i2c_client *client)
{
	int id;
	int err;
	id = apds993x_i2c_read(client, APDS993X_ID_REG,APDS993X_I2C_BYTE);
	if (id == 0x30) {
		APDS993X_INFO("%s: APDS9931\n", __func__);
		/* < DTS2014052903087 wanghang 201400604 begin */
		err = app_info_set("LP-Sensor", "APS9931/TMD27723T");
		if (err < 0)/*failed to add app_info*/
		{
		    APDS993X_ERR("%s %d:failed to add app_info\n", __func__, __LINE__);
		}
		/* DTS2014052903087 wanghang 201400604 end > */
	} else if (id == 0x39) {
		APDS993X_INFO("%s: APDS9930\n", __func__);
		/* < DTS2014052903087 wanghang 201400604 begin */
		err = app_info_set("LP-Sensor", "APS9930/TMD27723T");
		if (err < 0)/*failed to add app_info*/
		{
		    APDS993X_ERR("%s %d:failed to add app_info\n", __func__, __LINE__);
		}
		/* DTS2014052903087 wanghang 201400604 end > */
	} else {
		APDS993X_INFO("%s: Neither APDS9931 nor APDS9930\n", __func__);
		return -ENODEV;
	}
	return 0;
}
static int apds993x_init_client(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int err;
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	data->enable = 0x00;
	err = apds993x_set_enable(client, data->enable);
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	if (err < 0)
	{
		APDS993X_ERR("%s,line%d:apds993x_set_enable FAIL ",__func__,__LINE__);
		return err;
	}

	/* 100.64ms ALS integration time */
	/* < DTS2014071902196 wangcunyou 20140719 begin */
	err = apds993x_set_atime(client,APDS993X_100MS_ADC_TIME);
	/* DTS2014071902196 wangcunyou 20140719 end > */
	if (err < 0)
	{
		APDS993X_ERR("%s,line%d:apds993x_set_atime FAIL ",__func__,__LINE__);
		return err;
	}

	/* 2.72ms Prox integration time */
	err = apds993x_set_ptime(client, 0xFF);
	if (err < 0)
	{
		APDS993X_ERR("%s,line%d:apds993x_set_ptime FAIL ",__func__,__LINE__);
		return err;
	}

	/* 2.72ms Wait time the value is 0xff,the value is changeable*/
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	/* < DTS2014071902196 wangcunyou 20140719 begin */
	err = apds993x_set_wtime(client, 0xb6);
	/* DTS2014071902196 wangcunyou 20140719 end > */
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	if (err < 0)
	{
		APDS993X_ERR("%s,line%d:apds993x_set_wtime FAIL ",__func__,__LINE__);
		return err;
	}

	err = apds993x_set_ppcount(client, 0xC);
	if (err < 0)
	{
		APDS993X_ERR("%s,line%d:apds993x_set_ppcount FAIL ",__func__,__LINE__);
		return err;
	}

	/* no long wait */
	err = apds993x_set_config(client, 0);
	if (err < 0)
	{
		APDS993X_ERR("%s,line%d:apds993x_set_config FAIL ",__func__,__LINE__);
		return err;
	}
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	err = apds993x_set_control(client,
			APDS993X_PDRVIE_50MA |
			APDS993X_PRX_IR_DIOD |
			APDS993X_PGAIN_1X|APDS993X_AGAIN_1X);
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	if (err < 0)
	{
		APDS993X_ERR("%s,line%d:apds993x_set_control FAIL ",__func__,__LINE__);
		return err;
	}

	/* init threshold for proximity */
	err = apds993x_set_pilt(client, apds993x_ps_init_threshold-1);
	if (err < 0)
	{
		APDS993X_ERR("%s,line%d:apds993x_set_pilt FAIL ",__func__,__LINE__);
		return err;
	}

	err = apds993x_set_piht(client, apds993x_ps_init_threshold);
	if (err < 0)
	{
		APDS993X_ERR("%s,line%d:apds993x_set_piht FAIL ",__func__,__LINE__);
		return err;
	}
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	data->ps_detection = APDS993X_FAR_FLAG; /* initial value = far*/
	/* DTS2014073005921 yangxiaomei 20140730 end > */

	/* 2 consecutive Interrupt persistence */
	err = apds993x_set_pers(client, APDS993X_PPERS_1|APDS993X_APERS_2);
	if (err < 0)
	{
		APDS993X_ERR("%s,line%d:apds993x_set_pers FAIL ",__func__,__LINE__);
		return err;
	}

	/* sensor is in disabled mode but all the configurations are preset */
	return 0;
}
/*qualcom updated the regulator configure functions and we add them all*/
static int sensor_regulator_configure(struct apds993x_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				APDS993X_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				APDS993X_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			APDS993X_ERR("%s,line%d:Regulator get failed vdd rc=%d\n",__func__,__LINE__, rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				APDS993X_VDD_MIN_UV, APDS993X_VDD_MAX_UV);
			if (rc) {
				APDS993X_ERR("%s,line%d:Regulator set failed vdd rc=%d\n",__func__,__LINE__,rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			APDS993X_ERR("%s,line%d:Regulator get failed vio rc=%d\n",__func__,__LINE__, rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				APDS993X_VIO_MIN_UV, APDS993X_VIO_MAX_UV);
			if (rc) {
				APDS993X_ERR("%s,line%d:Regulator set failed vio rc=%d\n",__func__,__LINE__, rc);
				goto reg_vio_put;
			}
		}

	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, APDS993X_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}
/*In suspend and resume function,we only control the als,leave pls alone*/
static int apds993x_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int rc;
	/* < DTS2014082603327 yangxiaomei 20140902 begin */
	APDS993X_INFO("%s,line%d:APDS993X SUSPEND\n",__func__,__LINE__);
	/* < DTS2014051304400 caowei 201400513 begin */
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	/*hrtimer and work will canceled at apds993x_enable_als_sensor*/
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	/* DTS2014051304400 caowei 201400513 end >*/
	/*
	* Save sensor state and disable them,
	* this is to ensure internal state flags are set correctly.
	* device will power off after both sensors are disabled.
	* P sensor will not be disabled because it  is a wakeup sensor.
	*/
	data->enable_als_state = data->enable_als_sensor;
	apds993x_suspend_flag = APDS993X_SUSPEND_ON;
	if(data->enable_als_sensor){
		APDS993X_INFO("%s,line%d:APDS993X SUSPEND and disable als\n",__func__,__LINE__);
		rc = apds993x_enable_als_sensor(data->client, 0);
		if (rc){
			APDS993X_ERR("%s,line%d:Disable light sensor fail! rc=%d\n",__func__,__LINE__, rc);
		}
	/* DTS2014082603327 yangxiaomei 20140902 end > */
	}

	return 0;
}

static int apds993x_resume(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	/* < DTS2014051304400 caowei 201400513 begin */
	int ret = 0;
	/* DTS2014051304400 caowei 201400513 end >*/
	/* < DTS2014082603327 yangxiaomei 20140902 begin */
	apds993x_suspend_flag = APDS993X_SUSPEND_OFF;
	APDS993X_INFO("%s,line%d:APDS993X RESUME\n",__func__,__LINE__);
	if(APDS993X_ERROR_HAPPEN == apds993x_error_flag)
	{
		/* < DTS2014090903105 wangcunyou 20140912 begin */
		/* 2 = CMD_CLR_PS_INT */
		apds993x_set_command(client, 2);
		/* DTS2014090903105 wangcunyou 20140912 end > */
		if (data->irq)
		{
			/* < DTS2014110303514 wanghang 20141104 begin */
			operate_irq(data,1,true);
			/* DTS2014110303514 wanghang 20141104 end >*/
		}
		apds993x_error_flag = APDS993X_ERROR_NOTHAPPEN;
	}
	/* DTS2014082603327 yangxiaomei 20140902 end > */
	if (data->enable_als_state) {
		ret = apds993x_enable_als_sensor(data->client, 1);
		if (ret){
			APDS993X_ERR("%s,line%d:Disable light sensor fail! rc=%d\n",__func__,__LINE__, ret);
		}
	}

	return 0;
}
/*pamameter subfunction of probe to reduce the complexity of probe function*/
static int apds993x_sensorclass_init(struct apds993x_data *data,struct i2c_client* client)
{
	int err;
	/* Register to sensors class */
	data->als_cdev = sensors_light_cdev;
	data->als_cdev.sensors_enable = apds993x_als_set_enable;
	data->als_cdev.sensors_poll_delay = apds993x_als_poll_delay;

	data->ps_cdev = sensors_proximity_cdev;
	data->ps_cdev.sensors_enable = apds993x_ps_set_enable;
	data->ps_cdev.sensors_poll_delay = NULL;

	err = sensors_classdev_register(&client->dev, &data->als_cdev);
	if (err) {
		APDS993X_ERR("%s: Unable to register to sensors class: %d\n",__func__, err);
		goto exit;
	}
	err = sensors_classdev_register(&client->dev, &data->ps_cdev);
	if (err) {
		APDS993X_ERR("%s: Unable to register to sensors class: %d\n",__func__, err);
		goto exit_unregister_als_class;
	}
	goto exit;
exit_unregister_als_class:
	sensors_classdev_unregister(&data->als_cdev);
exit:
	return err;
}
static void apds993x_parameter_init(struct apds993x_data *data)
{
	/* Set the default parameters */
	apds993x_a_ga= data->platform_data->ga_a_value;
	apds993x_c_ga= data->platform_data->ga_c_value;
	apds993x_d_ga= data->platform_data->ga_d_value;
	apds993x_pwave_value= data->platform_data->pwave;
	apds993x_pwindow_value= data->platform_data->pwindow;
	/* BEGIN PN:  DTS2014052808848,Added by z84020826, 2014/05/29*/
	apds993x_coe_e = data->platform_data->ga_e_value;
	apds993x_coe_f = data->platform_data->ga_f_value;
	/* END PN:  DTS2014052808848,Added by z84020826, 2014/05/29*/
	/* < DTS2014061901549  wangcunyou 20140619 begin */
	apds993x_coe_b = data->platform_data->als_B;
	apds993x_coe_c = data->platform_data->als_C;
	apds993x_coe_d = data->platform_data->als_D;
	/* DTS2014061901549 wangcunyou 20140619 end > */
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	data->enable = 0x00;	/* default mode is standard */
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	data->ps_min_threshold = origin_prox;
	/* < DTS2014061000435 wanghang 20140610 begin */
	APDS993X_FLOW("%s:set origin_prox to data->ps_min_threshold=%d\n", __func__, data->ps_min_threshold);
	/* DTS2014061000435 wanghang 20140610 end > */
	data->enable_als_sensor = 0;	// default to 0
	data->enable_ps_sensor = 0;	// default to 0
	/* < DTS2014071902196 wangcunyou 20140719 begin */
	data->als_poll_delay = 200;	// default to 200ms
	/* DTS2014071902196 wangcunyou 20140719 end > */
	data->als_prev_lux = 300;
	/* < DTS2014082302937 wanghang 20140823 begin */
	data->count = 1;	// disable_irq is before enable_irq, so the initial value should more than zero
	/* DTS2014110303514 wanghang 20141104 end >*/
}
/*input init subfunction of probe to reduce the complexity of probe function*/
static int apds993x_input_init(struct apds993x_data *data)
{
	int err = 0;
	/* Register to Input Device */
	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als) {
		err = -ENOMEM;
		APDS993X_ERR("%s: Failed to allocate input device als\n", __func__);
		goto exit;
	}

	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		input_free_device(data->input_dev_als);
		APDS993X_ERR("%s: Failed to allocate input device ps\n", __func__);
		goto exit;
	}

	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 10000, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 5, 0, 0);

	data->input_dev_als->name = "light";
	data->input_dev_ps->name = "proximity";

	err = input_register_device(data->input_dev_als);
	if (err) {
		err = -ENOMEM;
		input_free_device(data->input_dev_als);
		input_free_device(data->input_dev_ps);
		APDS993X_ERR("%s: Unable to register input device als: %s\n",
				__func__, data->input_dev_als->name);
		goto exit;
	}

	err = input_register_device(data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		APDS993X_ERR("%s: Unable to register input device ps: %s\n",
				__func__, data->input_dev_ps->name);
		goto unregister_als;
	}
	goto exit;
unregister_als:
	input_unregister_device(data->input_dev_als);
	input_free_device(data->input_dev_ps);
exit:
	return err;
}
/*irq request subfunction of probe to reduce the complexity of probe function*/
static int apds993x_irq_init(struct apds993x_data *data,struct i2c_client *client)
{
	int ret = 0;
	if (data->platform_data->irq_gpio)
	{
		ret = gpio_request(data->platform_data->irq_gpio,"aps9930_irq_gpio");
		if (ret)
		{
			APDS993X_ERR("%s, line %d:unable to request gpio [%d]\n", __func__, __LINE__,data->platform_data->irq_gpio);
			return ret;
		}
		else
		{
			ret = gpio_direction_input(data->platform_data->irq_gpio);
			if(ret)
			{
				APDS993X_ERR("%s, line %d: Failed to set gpio %d direction\n", __func__, __LINE__,data->platform_data->irq_gpio);
				return ret;
			}
		}
	}
	client->irq = gpio_to_irq(data->platform_data->irq_gpio);
	if (client->irq < 0) {
		ret = -EINVAL;
		APDS993X_ERR("%s, line %d:gpio_to_irq FAIL! IRQ=%d\n", __func__, __LINE__,data->platform_data->irq_gpio);
		return ret;
	}
	data->irq = client->irq;
	if (client->irq)
	{
		/* < DTS2014092002755 wangcunyou 20140920 begin */
		/*AP examination of low level to prevent lost interrupt*/
		if (request_irq(data->irq, apds993x_interrupt,IRQF_TRIGGER_LOW|IRQF_ONESHOT|IRQF_NO_SUSPEND, APDS993X_DRV_NAME, (void *)client) >= 0)
		/* DTS2014092002755 wangcunyou 20140920 end > */
		{
			APDS993X_FLOW("%s, line %d:Received IRQ!\n", __func__, __LINE__);
			/* < DTS2014110303514 wanghang 20141104 begin */
			operate_irq(data,0,true);
			/* DTS2014110303514 wanghang 20141104 end >*/
		}
		else
		{
			APDS993X_ERR("%s, line %d:Failed to request IRQ!\n", __func__, __LINE__);
			ret = -EINVAL;
			return ret;
		}
	}
	return ret;
}
static int sensor_regulator_power_on(struct apds993x_data *data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			APDS993X_ERR("%s: Regulator vdd disable failed rc=%d\n", __func__, rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			APDS993X_ERR("%s: Regulator vdd disable failed rc=%d\n", __func__, rc);
			rc = regulator_enable(data->vdd);
			APDS993X_ERR("%s:Regulator vio re-enabled rc=%d\n",__func__, rc);
			/*
			 * Successfully re-enable regulator.
			 * Enter poweron delay and returns error.
			 */
			if (!rc) {
				rc = -EBUSY;
				goto enable_delay;
			}
		}
		return rc;
	} else {
		rc = regulator_enable(data->vdd);
		if (rc) {
			APDS993X_ERR("%s:Regulator vdd enable failed rc=%d\n",__func__, rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			APDS993X_ERR("%s:Regulator vio enable failed rc=%d\n", __func__,rc);
			rc = regulator_disable(data->vdd);
			return rc;
		}
	}
enable_delay:
	msleep(130);
	APDS993X_FLOW("%s:Sensor regulator power on =%d\n",__func__, on);
	return rc;
}

static int sensor_platform_hw_power_on(bool on,struct apds993x_data *data)
{
	int err = 0;

	if (data->power_on != on) {
		if (!IS_ERR_OR_NULL(data->pinctrl)) {
			if (on)
				/*after poweron,set the INT pin the default state*/
				err = pinctrl_select_state(data->pinctrl,
					data->pin_default);
			if (err)
				APDS993X_ERR("%s,line%d:Can't select pinctrl state\n", __func__, __LINE__);
		}

		err = sensor_regulator_power_on(data, on);
		if (err)
			APDS993X_ERR("%s,line%d:Can't configure regulator!\n", __func__, __LINE__);
		else
			data->power_on = on;
	}

	return err;
}
static int sensor_platform_hw_init(struct apds993x_data *data)
{
	int error;

	error = sensor_regulator_configure(data, true);
	if (error < 0) {
		APDS993X_ERR("%s,line %d:unable to configure regulator\n",__func__,__LINE__);
		return error;
	}

	return 0;
}

static void sensor_platform_hw_exit(struct apds993x_data *data)
{
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	int error;
	error = sensor_regulator_configure(data, false);
	if (error < 0) {
		APDS993X_ERR("%s,line %d:unable to configure regulator\n",__func__,__LINE__);
	}
	/*remove to the error handler program in probe because the gpio is not requested.*/
	/* DTS2014073005921 yangxiaomei 20140730 end > */
}
static int apds993x_pinctrl_init(struct apds993x_data *data)
{
	struct i2c_client *client = data->client;

	data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		APDS993X_ERR("%s,line %d:Failed to get pinctrl\n",__func__,__LINE__);
		return PTR_ERR(data->pinctrl);
	}
	/*we have not set the sleep state of INT pin*/
	data->pin_default =
		pinctrl_lookup_state(data->pinctrl, "default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		APDS993X_ERR("%s,line %d:Failed to look up default state\n",__func__,__LINE__);
		return PTR_ERR(data->pin_default);
	}

	return 0;
}

static int sensor_parse_dt(struct device *dev,
		struct apds993x_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	unsigned int tmp;
	int rc = 0;

	/* set functions of platform data */
	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;
	pdata->power_on = sensor_platform_hw_power_on;

	/* irq gpio */
	rc = of_get_named_gpio_flags(dev->of_node,
			"avago,irq-gpio", 0, NULL);
	if (rc < 0) {
		APDS993X_ERR("%s,line %d:Unable to read irq gpio\n",__func__,__LINE__);
		return rc;
	}
	pdata->irq_gpio = rc;

	rc = of_property_read_u32(np, "avago,ga_a_value", &tmp);
	if (rc) {
		APDS993X_ERR("%s,line %d:Unable to read ga_a_value\n",__func__,__LINE__);
		return rc;
	}
	pdata ->ga_a_value= tmp;

	rc = of_property_read_u32(np, "avago,ga_c_value", &tmp);
	if (rc) {
		APDS993X_ERR("%s,line %d:Unable to read ga_c_value\n",__func__,__LINE__);
		return rc;
	}
	pdata ->ga_c_value= tmp;

	rc = of_property_read_u32(np, "avago,ga_d_value", &tmp);
	if (rc) {
		APDS993X_ERR("%s,line %d:Unable to read ga_d_value\n",__func__,__LINE__);
		return rc;
	}
	pdata ->ga_d_value= tmp;

	rc = of_property_read_u32(np, "avago,wave", &tmp);
	if (rc) {
		APDS993X_ERR("%s,line %d:Unable to read pwave_value\n",__func__,__LINE__);
		return rc;
	}
	pdata ->pwave= tmp;

	rc = of_property_read_u32(np, "avago,window", &tmp);
	if (rc) {
		APDS993X_ERR("%s,line %d:Unable to read pwindow_value\n",__func__,__LINE__);
		return rc;
	}
	pdata ->pwindow= tmp;
	/* BEGIN PN:  DTS2014052808848,Added by z84020826, 2014/05/29*/
	rc = of_property_read_u32(np, "avago,ga_e_value", &tmp);
	if (rc) {
		APDS993X_ERR("%s,line %d:Unable to read ga_e_value\n",__func__,__LINE__);
		return rc;
	}
	pdata ->ga_e_value= tmp;
	rc = of_property_read_u32(np, "avago,ga_f_value", &tmp);
	if (rc) {
		APDS993X_ERR("%s,line %d:Unable to read ga_f_value\n",__func__,__LINE__);
		return rc;
	}
	pdata ->ga_f_value= tmp;
	/* END PN:  DTS2014052808848,Added by z84020826, 2014/05/29*/
	/* < DTS2014061901549  wangcunyou 20140619 begin */
	rc = of_property_read_u32(np, "avago,als_B", &tmp);
	if (rc) {
		APDS993X_ERR("%s,line %d:Unable to read als_B\n",__func__,__LINE__);
		return rc;
	}
	pdata ->als_B= tmp;
	rc = of_property_read_u32(np, "avago,als_C", &tmp);
	if (rc) {
		APDS993X_ERR("%s,line %d:Unable to read als_C\n",__func__,__LINE__);
		return rc;
	}
	pdata ->als_C= tmp;
	rc = of_property_read_u32(np, "avago,als_D", &tmp);
	if (rc) {
		APDS993X_ERR("%s,line %d:Unable to read als_D\n",__func__,__LINE__);
		return rc;
	}
	pdata ->als_D= tmp;
	/* DTS2014061901549 wangcunyou 20140619 end > */
	return 0;
}

/* < DTS2014061000435 wanghang 20140610 begin */
/*************************************************
  Function:        debug_log_set
  Description:     this func is to set the value to control log enable or disable
  Input:           data:address of register
                   val:to control log enable or disable
  Output:          none
  Return:          0
*************************************************/
static int debug_log_set(void *data, u64 val)
{
    apds993x_debug_mask = (val >= 1) ? (int)val : 1;
    return 0;
}

/*************************************************
  Function:        debug_log_get
  Description:     this func is to get the value of control log enable or disable
  Input:           data:address of register
  Output:          *val:to control log enable or disable
  Return:          0
*************************************************/
static int debug_log_get(void *data, u64 *val)
{
    *val = (u64)apds993x_debug_mask;
    return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_mask_fops, debug_log_get, debug_log_set, "%llu\n");
/* DTS2014061000435 wanghang 20140610 end > */

/* < DTS2014090409459 wanghang 20140904 begin */
static void apds993x_powerkey_screen_handler(struct work_struct *work)
{
	struct apds993x_data *data = container_of((struct delayed_work *)work, struct apds993x_data, powerkey_work);
	if(power_key_ps)
	{
		APDS993X_INFO("%s : power_key_ps (%d) press\n",__func__, power_key_ps);
		power_key_ps=false;
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, APDS993X_FAR_FLAG);
		input_sync(data->input_dev_ps);
	}
	schedule_delayed_work(&data->powerkey_work, msecs_to_jiffies(500));
}
/* DTS2014090409459 wanghang 20140904 end >*/

/*
 * I2C init/probing/exit functions
 */
static struct i2c_driver apds993x_driver;
static int apds993x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds993x_data *data;
	struct apds993x_platform_data *pdata;
	int err = 0;
	/* < DTS2014061000435 wanghang 20140610 begin */
	struct dentry *dentry_proximitylog = NULL;
	struct dentry *dentry_file = NULL;
	/* DTS2014061000435 wanghang 20140610 end > */

	APDS993X_INFO("%s,line %d:PROBE START.\n",__func__,__LINE__);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct apds993x_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			APDS993X_ERR("%s,line %d:Failed to allocate memory\n",__func__,__LINE__);
			err =-ENOMEM;
			goto exit;
		}

		client->dev.platform_data = pdata;
		err = sensor_parse_dt(&client->dev, pdata);
		if (err) {
			APDS993X_ERR("%s: sensor_parse_dt() err\n", __func__);
			goto exit_parse_dt;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			APDS993X_ERR("%s,line %d:No platform data\n",__func__,__LINE__);
			err = -ENODEV;
			goto exit;
		}
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		APDS993X_ERR("%s,line %d:Failed to i2c_check_functionality\n",__func__,__LINE__);
		err = -EIO;
		goto exit_parse_dt;
	}

	data = kzalloc(sizeof(struct apds993x_data), GFP_KERNEL);
	if (!data) {
		APDS993X_ERR("%s,line %d:Failed to allocate memory\n",__func__,__LINE__);
		err = -ENOMEM;
		goto exit_parse_dt;
	}

	data->platform_data = pdata;
	data->client = client;
	data->device_exist = false;

	/* h/w initialization */
	if (pdata->init)
		err = pdata->init(data);

	if (pdata->power_on)
		err = pdata->power_on(true,data);
	/* <DTS2014112604473 yangzhonghua 20141126 begin */
#ifdef CONFIG_HUAWEI_DSM
	/* < DTS2014072201656 yangzhonghua 20140722 begin */
	err = apds_dsm_init(data);
	if(err < 0)
		goto exit_uninit;
	/* DTS2014072201656 yangzhonghua 20140722 end > */
#endif
	/* DTS2014112604473 yangzhonghua 20141126 end> */

	i2c_set_clientdata(client, data);
	apds993x_parameter_init(data);
	/* initialize pinctrl */
	err = apds993x_pinctrl_init(data);
	if (err) {
		APDS993X_ERR("%s,line %d:Can't initialize pinctrl\n",__func__,__LINE__);
			goto exit_unregister_dsm;
	}
	err = pinctrl_select_state(data->pinctrl, data->pin_default);
	if (err) {
		APDS993X_ERR("%s,line %d:Can't select pinctrl default state\n",__func__,__LINE__);
		goto exit_unregister_dsm;
	}

	mutex_init(&data->update_lock);
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	mutex_init(&data->single_lock);
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	/* < DTS2014051304400 caowei 201400513 begin */
	INIT_WORK(&data->dwork, apds993x_work_handler);

	INIT_WORK(&data->als_dwork, apds993x_als_polling_work_handler);
	/* DTS2014051304400 caowei 201400513 end >*/
	/* < DTS2014090409459 wanghang 20140904 begin */
	INIT_DELAYED_WORK(&data->powerkey_work, apds993x_powerkey_screen_handler);
	/* DTS2014090409459 wanghang 20140904 end >*/
	/* Initialize the APDS993X chip and judge who am i*/
	err = apds993x_read_device_id(client);
	if (err) {
		APDS993X_ERR("%s: Failed to read apds993x\n", __func__);
		goto exit_unregister_dsm;
	}
	err = apds993x_init_client(client);
	if (err) {
		APDS993X_ERR("%s: Failed to init apds993x\n", __func__);
		goto exit_unregister_dsm;
	}

	err = apds993x_input_init(data);
	if(err)
		goto exit_unregister_dsm;

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &apds993x_attr_group);
	if (err)
		goto exit_unregister_dev_ps;

	wake_lock_init(&data->ps_report_wk, WAKE_LOCK_SUSPEND, "psensor_wakelock");

	err=apds993x_irq_init(data,client);
	if(err)
		goto exit_remove_sysfs_group;

	device_init_wakeup(&(client->dev), true);

	err = apds993x_sensorclass_init(data,client);
	if (err) {
		APDS993X_ERR("%s: Unable to register to sensors class: %d\n",
	__func__, err);
		goto exit_free_irq;
	}

	/* < DTS2014051304400 caowei 201400513 begin */
	/* < DTS2014052606323 wanghang 20140526 begin */
	apds993x_workqueue = create_workqueue("apds993x_work_queue");
	if (!apds993x_workqueue)
	{
		APDS993X_ERR("%s: Create ps_workqueue fail.\n", __func__);
		/* < DTS2014073005921 yangxiaomei 20140730 begin */
		goto exit_unregister_sensorclass;
		/* DTS2014073005921 yangxiaomei 20140730 end > */
	/* DTS2014052606323 wanghang 20140526  end > */
	}

	/* init hrtimer and call back function */
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->timer.function = apds993x_als_timer_func;
	/* DTS2014051304400 caowei 201400513 end >*/
	/* < DTS2014041705903 zhangmin 20140418 begin */
	set_sensors_list(L_SENSOR + P_SENSOR);
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	set_hw_dev_flag(DEV_I2C_APS);
	set_hw_dev_flag(DEV_I2C_L_SENSOR);
#endif
	err = set_sensor_input(PS, data->input_dev_ps->dev.kobj.name);
	if (err) {
		APDS993X_ERR("%s set_sensor_input PS failed\n", __func__);
	}
	err = set_sensor_input(ALS, data->input_dev_als->dev.kobj.name);
	if (err) {
		APDS993X_ERR("%s set_sensor_input ALS failed\n", __func__);
	}

	/* < DTS2014061000435 wanghang 20140610 begin */
	/* create directory "hw_apds993x_log" */
	dentry_proximitylog = debugfs_create_dir("hw_apds993x_log", NULL);
	if(dentry_proximitylog)
	{
		/* create file named debug_mask at /sys/kernel/debug/hw_Proximity_log */
		dentry_file = debugfs_create_file("debug_mask", 0664, dentry_proximitylog, NULL, &debug_mask_fops);
		if (!dentry_file)
		{
			APDS993X_ERR("%s %d:Create log file error! \n", __func__, __LINE__);
			debugfs_remove(dentry_file);
		}
	}
	else
	{
		APDS993X_ERR("%s %d:Create log director error! \n", __func__, __LINE__);
	}
	/* DTS2014061000435 wanghang 20140610 end > */
	if (pdata->power_on)
		err = pdata->power_on(false,data);
	/* DTS2014041705903 zhangmin 20140418 end > */
	APDS993X_INFO("%s: Support ver. %s enabled\n", __func__, DRIVER_VERSION);
	data->device_exist = true;
	return 0;
	/* < DTS2014052606323 wanghang 20140526 begin */
	/* delete this,use one work queue*/
	/* DTS2014052606323 wanghang 20140526  end > */
/* < DTS2014073005921 yangxiaomei 20140730 begin */
exit_unregister_sensorclass:
	sensors_classdev_unregister(&data->als_cdev);
	sensors_classdev_unregister(&data->ps_cdev);
exit_free_irq:
	if (gpio_is_valid(data->platform_data->irq_gpio))
		gpio_free(data->platform_data->irq_gpio);
/* DTS2014073005921 yangxiaomei 20140730 end > */
	free_irq(data->irq, client);
exit_remove_sysfs_group:
	wake_lock_destroy(&data->ps_report_wk);
	sysfs_remove_group(&client->dev.kobj, &apds993x_attr_group);
exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);
	input_unregister_device(data->input_dev_als);
/* <DTS2014112604473 yangzhonghua 20141126 begin */
#ifdef CONFIG_HUAWEI_DSM
/* < DTS2014072201656 yangzhonghua 20140722 begin */
exit_unregister_dsm:
	apds_dsm_exit();
exit_uninit:
/* DTS2014072201656 yangzhonghua 20140722 end > */
#else
exit_unregister_dsm:
#endif
	if (pdata->power_on)
		pdata->power_on(false,data);
	if (pdata->exit)
		pdata->exit(data);
	kfree(data);

/* DTS2014112604473 yangzhonghua 20141126 end> */
exit_parse_dt:
	/* < DTS2014072107286 yangxiaomei 20140723 begin */
	/*remove it,because devm_kzalloc will release the memory automatically
	if don't remove,there will be null pointer which lead to booting crush*/
	/* DTS2014072107286 yangxiaomei 20140723 end >*/
exit:
	return err;
}

static int apds993x_remove(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	struct apds993x_platform_data *pdata = data->platform_data;

	/* Power down the device */
	/* < DTS2014073005921 yangxiaomei 20140730 begin */
	data->enable = 0x00;
	apds993x_set_enable(client, data->enable);
	/* DTS2014073005921 yangxiaomei 20140730 end > */
	wake_lock_destroy(&data->ps_report_wk);
	sysfs_remove_group(&client->dev.kobj, &apds993x_attr_group);

	input_unregister_device(data->input_dev_ps);
	input_unregister_device(data->input_dev_als);

	free_irq(client->irq, data);
	/* < DTS2014051304400 caowei 201400513 begin */
	hrtimer_cancel(&data->timer);
	/* DTS2014051304400 caowei 201400513 end >*/
	/* <DTS2014112604473 yangzhonghua 20141126 begin */
#ifdef CONFIG_HUAWEI_DSM
	/* < DTS2014072201656 yangzhonghua 20140722 begin */
	apds_dsm_exit();
	/* DTS2014072201656 yangzhonghua 20140722 end > */
#endif
	/* DTS2014112604473 yangzhonghua 20141126 end> */

	if (pdata->power_on)
		pdata->power_on(false,data);

	if (pdata->exit)
		pdata->exit(data);

	kfree(data);

	return 0;
}

static const struct i2c_device_id apds993x_id[] = {
	{ "apds993x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds993x_id);

static struct of_device_id apds993X_match_table[] = {
	{ .compatible = "avago,apds9930",},
	{ },
};

static struct i2c_driver apds993x_driver = {
	.driver = {
		.name   = APDS993X_DRV_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = apds993X_match_table,
	},
	.probe  = apds993x_probe,
	.remove = apds993x_remove,
	.suspend = apds993x_suspend,
	.resume = apds993x_resume,
	.id_table = apds993x_id,
};

static int __init apds993x_init(void)
{
	return i2c_add_driver(&apds993x_driver);
}

static void __exit apds993x_exit(void)
{
	/* < DTS2014051304400 caowei 201400513 begin */
	/* destroy als and ps work queue */
	/* < DTS2014052606323 wanghang 20140526 begin */
	if (apds993x_workqueue) {
		destroy_workqueue(apds993x_workqueue);
		apds993x_workqueue = NULL;
	}

	/* DTS2014052606323 wanghang 20140526  end > */
	/* DTS2014051304400 caowei 201400513 end >*/
	i2c_del_driver(&apds993x_driver);
}

MODULE_DESCRIPTION("APDS993X ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds993x_init);
module_exit(apds993x_exit);
/* DTS2014042508166 yangxiaomei 20140428 end > */
/* DTS2014071108152 yangxiaomei 20140714 end >*/


