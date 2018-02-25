/* <DTS2014051302540 zhuchengming 20140513 begin */
/* < DTS2014072601546 yanhuiwen/0083550 20140723 begin */
/*************************************************
  Copyright (C), 1988-1999, Huawei Tech. Co., Ltd.
  File name        : Sensor_otp_common_if.c
  Author            : z00278703
  Version           : Initial Draft
  Date               : 2014/05/14
  Description   : this file is used by driver to check if the otp
            is supported by the driver.
  Function List    :
            is_exist_otp_function
  History            :
  1.Date            : 2014/05/14
     Author   : z00278703
     Modification  : Created File

*************************************************/

//#define HW_CMR_LOGSWC 0   //file log switch set 0 off,default is 1 on
#define HW_CMR_LOG_TAG "sensor_otp_common_if"

#include <linux/hw_camera_common.h>
#include "msm_sensor.h"
#include "sensor_otp_common_if.h"

#define OV5648_SUNNY_P5V18G_RG_RATIO_TYPICAL 0x2eb
#define OV5648_SUNNY_P5V18G_BG_RATIO_TYPICAL 0x2dc

/* <DTS2014071005860 jiweifeng/jwx206032 20140711 begin */
#define OV5648_SUNNY_P5V36D_RG_RATIO_TYPICAL 0x2e8
#define OV5648_SUNNY_P5V36D_BG_RATIO_TYPICAL 0x2dd
/* DTS2014071005860  jiweifeng/jwx206032 20140711 end >*/

#define S5K4E1_LITEON_13P1_RG_RATIO_TYPICAL 0x2d0
#define S5K4E1_LITEON_13P1_BG_RATIO_TYPICAL 0x287

#define S5K4E1_FOXCONN_DC0301A_RG_RATIO_TYPICAL 0x34c
#define S5K4E1_FOXCONN_DC0301A_BG_RATIO_TYPICAL 0x2f3

/* <DTS2014061204421 yangzhenxi/WX221546 20140612 begin */
#define OV8858_FOXCONN_RG_RATIO_TYPICAL 0x2FC
#define OV8858_FOXCONN_BG_RATIO_TYPICAL 0x2B5
/*<DTS2014061904065 Zhangbo 20140619 begin*/
//5M OV5648 Foxconn
/*<DTS2014080508165 jiweifeng/jwx206032 20140805 begin*/
#define OV5648_FOXCONN_SC0602_RG_RATIO_TYPICAL 0x2da
#define OV5648_FOXCONN_SC0602_BG_RATIO_TYPICAL 0x2c3
/* DTS2014080508165 jiweifeng/jwx206032 20140805 end> */
//5M s5k4e1 Sunny
#define S5K4E1_SUNNY_P5S07A_RG_RATIO_TYPICAL 0x312
#define S5K4E1_SUNNY_P5S07A_BG_RATIO_TYPICAL 0x2DE
/* DTS2014061904065 Zhangbo 20140619 end >*/

/* <DTS2014070903091 jiweifeng/jwx206032 20140709 begin */
#define OV13850_SUNNY_P13V01H_RG_RATIO_TYPICAL  0x248
#define OV13850_SUNNY_P13V01H_BG_RATIO_TYPICAL  0x257
/* DTS2014070903091 jiweifeng/jwx206032 20140709 end> */

/* <DTS2014081404502 wangguoying 20140814 begin */
/* Not used*/
#define IMX328_SUNNY_P13N10A_RG_RATIO_TYPICAL  0x01ED
#define IMX328_SUNNY_P13N10A_BG_RATIO_TYPICAL  0x0172
/* DTS2014081404502 wangguoying 20140814 end> */

/* <DTS2014101602303  jiweifeng/jwx206032 20141016 begin */
#define OV5670_BYD_CHT854B_RG_RATIO_TYPICAL         0x0275  
#define OV5670_BYD_CHT854B_BG_RATIO_TYPICAL         0x02a3 
/* DTS2014101602303 jiweifeng/jwx206032 20141016 end > */

/* <DTS2014101105981 jiweifeng/jwx206032 20141011 begin */
#define S5K5E2_OFILM_OHW5F02_RG_RATIO_TYPICAL         0x035c
#define S5K5E2_OFILM_OHW5F02_BG_RATIO_TYPICAL         0x02b8
/* DTS2014101105981 jiweifeng/jwx206032 20141011 end> */

/*< DTS2014112103818 gwx229921 20141211 begin */
/*< DTS2014112103803  gwx229921 20141211 begin */
/*< DTS2014121706384 gwx229921 20141217 begin */
#define OV5648_OFILM_OHW5F03_RG_RATIO_TYPICAL         0x02e8
#define OV5648_OFILM_OHW5F03_BG_RATIO_TYPICAL         0x0312
/* DTS2014121706384 gwx229921 20141217 end >*/

#define S5K4E1_SUNNY_132_RG_RATIO_TYPICAL            0x30c
#define S5K4E1_SUNNY_132_BG_RATIO_TYPICAL            0x2b1

#define OV5648_FOXCONN_132_RG_RATIO_TYPICAL          0x278
#define OV5648_FOXCONN_132_BG_RATIO_TYPICAL          0x2d2

#define OV13850_LITEON_P13V01H_RG_RATIO_TYPICAL  	0x248
#define OV13850_LITEON_P13V01H_BG_RATIO_TYPICAL  	0x257

/*< DTS2015021200322  jwx206032 20150215 begin */
#define OV13850_OFILM_OHWBA03_RG_RATIO_TYPICAL  0x248
#define OV13850_OFILM_OHWBA03_BG_RATIO_TYPICAL  0x257
/* DTS2015021200322  jwx206032 20150215 end >*/

/*DTS2014112103803 gwx229921 20141211 end >*/
/* DTS2014112103818 gwx229921 20141211 end >*/

/* < DTS2014073004411 yangzhenxi 20140731 begin */

/* DTS2015011906877 shaohongyuan swx234330 Add OTP function -- Begin*/
/* < DTS2015020209292 l00271394 20150202 begin */
#define S5K5E2_FOXCONN_HC0806_PAD_RG_RATIO_TYPICAL		0x354
#define S5K5E2_FOXCONN_HC0806_PAD_BG_RATIO_TYPICAL		0x28d
/* DTS2015020209292 l00271394 20150202 end > */
/* < DTS2015022605541 l00271394 20150226 begin */
/* < DTS2015030906532 l00271394 20150309 begin */
#define OV5670_OFILM_OHW5A03_PAD_RG_RATIO_TYPICAL         0x294  
#define OV5670_OFILM_OHW5A03_PAD_BG_RATIO_TYPICAL         0x299 
/* DTS2015030906532 l00271394 20150309 end > */
/* DTS2015022605541 l00271394 20150226 end > */
/* DTS2015011906877 shaohongyuan swx234330 Add OTP function -- End*/

struct otp_function_t otp_function_lists []=
{
	{
		"ov5648_sunny_p5v18g",
		ov5648_sunny_p5v18g_otp_func,
		OV5648_SUNNY_P5V18G_RG_RATIO_TYPICAL,
		OV5648_SUNNY_P5V18G_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov5648_sunny_p5v36d",
		ov5648_sunny_p5v18g_otp_func,
		OV5648_SUNNY_P5V36D_RG_RATIO_TYPICAL,
		OV5648_SUNNY_P5V36D_BG_RATIO_TYPICAL,
		false,
	},
	{
		"s5k4e1_liteon_13p1",
		s5k4e1_liteon_13p1_otp_func,
		S5K4E1_LITEON_13P1_RG_RATIO_TYPICAL,
		S5K4E1_LITEON_13P1_BG_RATIO_TYPICAL,
		false,
	},
	{
		"s5k4e1_foxconn_dc0301a",
		s5k4e1_liteon_13p1_otp_func,
		S5K4E1_FOXCONN_DC0301A_RG_RATIO_TYPICAL,
		S5K4E1_FOXCONN_DC0301A_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov8858_foxconn",
		ov8858_foxconn_otp_func,
		OV8858_FOXCONN_RG_RATIO_TYPICAL,
		OV8858_FOXCONN_BG_RATIO_TYPICAL,
		true,
	},

	/*<DTS2014061904065 Zhangbo 20140619 begin*/
	//5M OV5648 Foxconn
	{
		"ov5648_foxconn",  //Temp
		ov5648_foxconn_sc0602_otp_func,
		OV5648_FOXCONN_SC0602_RG_RATIO_TYPICAL,
		OV5648_FOXCONN_SC0602_BG_RATIO_TYPICAL,
		false,
	},

	/* <DTS2014070903091 jiweifeng/jwx206032 20140709 begin */
	{
		"ov13850_sunny_p13v01h",
		ov13850_sunny_p13v01h_otp_func,
		OV13850_SUNNY_P13V01H_RG_RATIO_TYPICAL,
		OV13850_SUNNY_P13V01H_BG_RATIO_TYPICAL,
		false,
	},
	/* DTS2014070903091 jiweifeng/jwx206032 20140709 end> */

	//5M s5k4e1 Sunny
	{
		"s5k4e1_sunny",  //Temp
		s5k4e1_sunny_p5s07a_otp_func,
		S5K4E1_SUNNY_P5S07A_RG_RATIO_TYPICAL,
		S5K4E1_SUNNY_P5S07A_BG_RATIO_TYPICAL,
		false,
	},
	/* DTS2014061904065 Zhangbo 20140619 end >*/
	/* <DTS2014091504893 xiongtao/wx217212 20140915 begin*/
	/*close otp function for imx328*/
    /* <DTS2014081404502 wangguoying 20140814 begin */
	#if 0
	{
		"imx328_sunny_p13n10a",
		imx328_sunny_p13n10a_otp_func,
		IMX328_SUNNY_P13N10A_RG_RATIO_TYPICAL,
		IMX328_SUNNY_P13N10A_BG_RATIO_TYPICAL,
		false,
	},
	#endif
    /* DTS2014081404502 wangguoying 20140814 end> */
	/* <DTS2014101105981  jiweifeng/jwx206032 20141017 begin */
	{
	    "s5k5e2_ofilm_ohw5f02",
		s5k5e2_otp_func,
		S5K5E2_OFILM_OHW5F02_RG_RATIO_TYPICAL,
		S5K5E2_OFILM_OHW5F02_BG_RATIO_TYPICAL,
		false,
	},
/* DTS2014101105981 jiweifeng/jwx206032 20141017 end > */
/* <DTS2014101602303  jiweifeng/jwx206032 20141016 begin */
	{
		"ov5670_byd_cht854b",
		ov5670_otp_func,
		OV5670_BYD_CHT854B_RG_RATIO_TYPICAL,
		OV5670_BYD_CHT854B_BG_RATIO_TYPICAL,
		false,
	},
/* DTS2014101602303 jiweifeng/jwx206032 20141016 end > */
	/* DTS2014082203425 lwx223669 20140822 end > */
    /*< DTS2014112103803  gwx229921 20141211 begin */
        {
		"s5k4e1_sunny_chm",
		s5k4e1_sunny_132_otp_func,
		S5K4E1_SUNNY_132_RG_RATIO_TYPICAL,
		S5K4E1_SUNNY_132_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov5648_foxconn_chm",
		ov5648_foxconn_132_otp_func,
		OV5648_FOXCONN_132_RG_RATIO_TYPICAL,
		OV5648_FOXCONN_132_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov5648_ofilm_ohw5f03_chm",
		ov5648_foxconn_132_otp_func,
		OV5648_OFILM_OHW5F03_RG_RATIO_TYPICAL,
		OV5648_OFILM_OHW5F03_BG_RATIO_TYPICAL,
		false,
	},
	{
		"imx328_sunny_p13n10a_chm",
		imx328_sunny_p13n10a_otp_func,
		IMX328_SUNNY_P13N10A_RG_RATIO_TYPICAL,
		IMX328_SUNNY_P13N10A_BG_RATIO_TYPICAL,
		false,
	},
	/*< DTS2015021200322  jwx206032 20150215 begin */
	{
		"ov13850_liteon_p13v01h_chm",
		ov13850_otp_func,
		OV13850_LITEON_P13V01H_RG_RATIO_TYPICAL,
		OV13850_LITEON_P13V01H_BG_RATIO_TYPICAL,
		false,
	},
	{
	    "ov13850_ofilm_ohwba03_ale",
		ov13850_otp_func,
		OV13850_OFILM_OHWBA03_RG_RATIO_TYPICAL,
		OV13850_OFILM_OHWBA03_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov13850_ofilm_ohwba03_chm",
		ov13850_otp_func,
		OV13850_OFILM_OHWBA03_RG_RATIO_TYPICAL,
		OV13850_OFILM_OHWBA03_BG_RATIO_TYPICAL,
		false,
	},
	/* DTS2015021200322  jwx206032 20150215 end >*/
	/* DTS2014112103803 gwx229921 20141211 end >*/
	/*< DTS2014112103818 gwx229921 20141211 begin */
	{
		"s5k4e1_sunny_ale",
		s5k4e1_sunny_132_otp_func,
		S5K4E1_SUNNY_132_RG_RATIO_TYPICAL,
		S5K4E1_SUNNY_132_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov5648_foxconn_ale",
		ov5648_foxconn_132_otp_func,
		OV5648_FOXCONN_132_RG_RATIO_TYPICAL,
		OV5648_FOXCONN_132_BG_RATIO_TYPICAL,
		false,
	},
	{
		"ov5648_ofilm_ohw5f03_ale",
		ov5648_foxconn_132_otp_func,
		OV5648_OFILM_OHW5F03_RG_RATIO_TYPICAL,
		OV5648_OFILM_OHW5F03_BG_RATIO_TYPICAL,
		false,
	},
	{
		"imx328_sunny_p13n10a_ale",
		imx328_sunny_p13n10a_otp_func,
		IMX328_SUNNY_P13N10A_RG_RATIO_TYPICAL,
		IMX328_SUNNY_P13N10A_BG_RATIO_TYPICAL,
		false,
	},
	/*< DTS2015021200322  jwx206032 20150215 begin */
	{
		"ov13850_liteon_p13v01h_ale",
		ov13850_otp_func,
		OV13850_LITEON_P13V01H_RG_RATIO_TYPICAL,
		OV13850_LITEON_P13V01H_BG_RATIO_TYPICAL,
		false,
	},
	/* DTS2015021200322  jwx206032 20150215 end >*/
	/* DTS2014112103818 gwx229921 20141211 end >*/

	/* DTS2015011906877 shaohongyuan swx234330 Add OTP function -- Begin*/
	{
		"ov5670_ofilm_ohw5a03_pad",
		ov5670_otp_func,
		OV5670_OFILM_OHW5A03_PAD_RG_RATIO_TYPICAL,
		OV5670_OFILM_OHW5A03_PAD_BG_RATIO_TYPICAL,
		false,
	},
	{
		"s5k5e2_foxconn_hc0806_pad",
		s5k5e2_otp_func,
		S5K5E2_FOXCONN_HC0806_PAD_RG_RATIO_TYPICAL,
		S5K5E2_FOXCONN_HC0806_PAD_BG_RATIO_TYPICAL,
		false,
	},
	/* DTS2015011906877 shaohongyuan swx234330 Add OTP function -- End*/
};
/* DTS2014073004411 yangzhenxi 20140731 end >*/
/* <DTS2014061204421 yangzhenxi/WX221546 20140612 end */

/*************************************************
  Function    : is_exist_otp_function
  Description: Detect the otp we support
  Calls:
  Called By  : msm_sensor_config
  Input       : s_ctrl
  Output     : index
  Return      : true describe the otp we support
                false describe the otp we don't support

*************************************************/
bool is_exist_otp_function( struct msm_sensor_ctrl_t *s_ctrl, int32_t *index)
{
	int32_t i = 0;
	/*< DTS2014121706384 gwx229921 20141217 begin */
	s_ctrl->afc_otp_info.starting_dac = 0;
	s_ctrl->afc_otp_info.infinity_dac = 0;
	s_ctrl->afc_otp_info.macro_dac = 0;
	/* DTS2014121706384 gwx229921 20141217 end >*/
	for (i=0; i<(sizeof(otp_function_lists)/sizeof(otp_function_lists[0])); ++i)
	{
		/* <DTS2014092901923 ganfan 20140929 begin */
        if(strlen(s_ctrl->sensordata->sensor_name) != strlen(otp_function_lists[i].sensor_name))
            continue;
		/* DTS2014092901923 ganfan 20140929 end> */
		if (0 == strncmp(s_ctrl->sensordata->sensor_name, otp_function_lists[i].sensor_name, strlen(s_ctrl->sensordata->sensor_name)))
		{
			*index = i;
			CMR_LOGI("is_exist_otp_function success i = %d\n", i);
			return true;
		}
	}
	return false;
}
/* DTS2014072601546 yanhuiwen/00283550 20140723 end > */

/* DTS2014051302540 zhuchengming 20140513 end> */
