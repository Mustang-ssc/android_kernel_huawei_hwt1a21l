/* < DTS2014042402686 sunlibin 20140424 begin */
/*Add for huawei TP*/
/*
 * Copyright (c) 2014 Huawei Device Company
 *
 * This file provide common requeirment for different touch IC.
 * 
 * 2014-01-04:Add "tp_get_touch_screen_obj" by sunlibin
 *
 */
#ifndef __HW_TP_COMMON__
#define __HW_TP_COMMON__
/* < DTS2014052402264 shenjinming 20140526 begin */
/* delete 2 lines */
/* DTS2014052402264 shenjinming 20140526 end > */
/*IC type*/
#define IC_TYPE_3207 3207

#define FW_OFILM_STR "000"
#define FW_EELY_STR "001"
#define FW_TRULY_STR "002"
#define FW_JUNDA_STR "005"
/* < DTS2014062403301 wanglongfei 20140624 begin */
#define FW_GIS_STR  "004"
#define FW_YASSY_STR  "007"
/* DTS2014062403301 wanglongfei 20140624 end> */
#define FW_LENSONE_STR "006"

#define MODULE_STR_LEN 3

/* < DTS2014052402264 shenjinming 20140526 begin */
/* buffer size for dsm tp client */
#define TP_RADAR_BUF_MAX	4096
/* DTS2014052402264 shenjinming 20140526 end > */

enum f54_product_module_name {
	FW_OFILM = 0,
	FW_EELY = 1,
	FW_TRULY = 2,
	/* < DTS2014070819376 sunlibin 20140709 begin */
	/*Modify G760L tp_cap threshold get from V3*/
	/* < DTS2014062403301 wanglongfei 20140624 begin */
	FW_GIS = 4,
	FW_JUNDA = 5,
	FW_LENSONE = 6,
	FW_YASSY = 7,
	/* DTS2014062403301 wanglongfei 20140624 end> */
	/* DTS2014070819376 sunlibin 20140709 end> */
	
	UNKNOW_PRODUCT_MODULE = 0xff,
};
struct holster_mode{
	unsigned long holster_enable;
	int top_left_x0;
	int top_left_y0;
	int bottom_right_x1;
	int bottom_right_y1;
};

struct kobject* tp_get_touch_screen_obj(void);
/* <DTS2014090905785 wwx203500 20141009 begin */
struct kobject* tp_get_virtual_key_obj(char *name);
/* DTS2014090905785 wwx203500 20141009 end> */
struct kobject* tp_get_glove_func_obj(void);

/* < DTS2014052402264 shenjinming 20140526 begin */
/* delete the func declare */
/* DTS2014052402264 shenjinming 20140526 end > */

/* < DTS2014071601734 yanghaizhou 20140715 begin */
/* add phone name so that a tp-driver can behave differentlly
accroding to different products*/
#define PHONE_NAME_Y550      "Y550"
#define PHONE_NAME_ULC02    "ULC02"
/* < DTS2014112404549 caowei 20141128 begin */
#define PHONE_NAME_ALICE	"Alice"
/* DTS2014112404549 caowei 20141128 end > */
/* DTS2014071601734 yanghaizhou 20140715 end > */
/* BEGIN PN: DTS2014101601174,Added by yuanbo, 2014/10/16*/
/* < DTS2014082603541 yanghaizhou 20140826 begin */
unsigned char already_has_tp_driver_running(void);
void set_tp_driver_running(void);
/* DTS2014082603541 yanghaizhou 20140826 end > */
/* END   PN: DTS2014101601174,Added by yuanbo, 2014/10/16*/
/* < DTS2015012000564 zhangyonggang 20150120 begin */
int get_tp_type(void);
void set_tp_type(int type);
/* DTS2015012000564 zhangyonggang 20150120 end > */
#endif

/* DTS2014042402686 sunlibin 20140424 end> */
