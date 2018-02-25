/* <DTS2014051302540 zhuchengming 20140513 begin */
/*************************************************
  Copyright (C), 1988-1999, Huawei Tech. Co., Ltd.
  File name        : Sensor_otp_common_if.h 
  Author            : z00278703
  Version           : Initial Draft  
  Date               : 2014/05/14
  Description   : this file contions otp function we used. We 
               define a struct that contion otp we support
               and the otp function.
  Function List    :  
           is_exist_otp_function
           s5k4e1_liteon_13p1_otp_func
           ov5648_sunny_p5v18g_otp_func
  History            : 
  1.Date            : 2014/05/14
     Author   : z00278703
     Modification  : Created File

*************************************************/



#ifndef _SENSOR_OTP_COMMON_IF_H
#define _SENSOR_OTP_COMMON_IF_H


struct otp_function_t {
	char sensor_name[32];
	int (*sensor_otp_function) (struct msm_sensor_ctrl_t *s_ctrl, int index);
	uint32_t rg_ratio_typical;//r/g ratio
	uint32_t bg_ratio_typical;//b/g ratio
/* < DTS2014073004411 yangzhenxi 20140731 begin */
	bool is_boot_load; //whether load otp info when booting.
/* DTS2014073004411 yangzhenxi 20140731 end >*/
};


extern bool is_exist_otp_function(struct msm_sensor_ctrl_t *sctl, int32_t *index);
extern int s5k4e1_liteon_13p1_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index);
extern int ov5648_sunny_p5v18g_otp_func(struct msm_sensor_ctrl_t * s_ctrl, int index);
/* <DTS2014061204421 yangzhenxi/WX221546 20140612 begin */
extern int ov8858_foxconn_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index);
/* <DTS2014061204421 yangzhenxi/WX221546 20140612 end */
/*<DTS2014061904065 Zhangbo 20140619 begin*/
extern int ov5648_foxconn_sc0602_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index);
extern int s5k4e1_sunny_p5s07a_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index);
/* DTS2014061904065 Zhangbo 20140619 end >*/

/* <DTS2014070903091 jiweifeng/jwx206032 20140709 begin */
extern int ov13850_sunny_p13v01h_otp_func(struct msm_sensor_ctrl_t * s_ctrl, int index);
/* DTS2014070903091 jiweifeng/jwx206032 20140709 end> */
/* <DTS2014081404502 wangguoying 20140814 begin */
extern int imx328_sunny_p13n10a_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index);
/* DTS2014081404502 wangguoying 20140814 end> */

/* <DTS2014101602303  jiweifeng/jwx206032 20141016 begin */
extern int ov5670_otp_func(struct msm_sensor_ctrl_t * s_ctrl, int index);
/* DTS2014101602303 jiweifeng/jwx206032 20141016 end > */

/* <DTS2014101105981 jiweifeng/jwx206032 20141011 begin */
extern int s5k5e2_otp_func(struct msm_sensor_ctrl_t * s_ctrl, int index);
/* DTS2014101105981 jiweifeng/jwx206032 20141011 end> */

/*< DTS2014112103803  gwx229921 20141211 begin */
/*< DTS2014112103818 gwx229921 20141211 begin */
/*< DTS2015021200322  jwx206032 20150215 begin */
extern int ov13850_otp_func(struct msm_sensor_ctrl_t * s_ctrl, int index);
/* DTS2015021200322  jwx206032 20150215 end >*/
extern int ov5648_foxconn_132_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index);
extern int s5k4e1_sunny_132_otp_func(struct msm_sensor_ctrl_t *s_ctrl, int index);
/* DTS2014112103803 gwx229921 20141211 end >*/
/* DTS2014112103818 gwx229921 20141211 end >*/



extern struct otp_function_t otp_function_lists [];


#endif
/* DTS2014051302540 zhuchengming 20140513 end> */
