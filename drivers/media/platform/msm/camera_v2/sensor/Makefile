ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
obj-$(CONFIG_MSMB_CAMERA) += cci/ io/ csiphy/ csid/ actuator/ flash/ eeprom/

#/* <DTS2014051302540 zhuchengming 20140513 begin */
#/*<DTS2014061904065 Zhangbo 20140619 begin*/
obj-$(CONFIG_MSM_CAMERA_SENSOR) += msm_sensor_init.o msm_sensor_driver.o msm_sensor.o sensor_otp_common_if.o sensor_otp_ov5648_sunny_p5v18g.o sensor_otp_s5k4e1_liteon_13p1.o sensor_otp_ov5648_foxconn_sc0602.o sensor_otp_s5k4e1_sunny_p5s07a.o
#/* DTS2014061904065 Zhangbo 20140619 end >*/
#/* DTS2014051302540 zhuchengming 20140513 end> */
#/* <DTS2014061204421 yangzhenxi/WX221546 20140612 begin */
obj-$(CONFIG_MSM_CAMERA_SENSOR) += sensor_otp_ov8858_foxconn.o
#/* <DTS2014061204421 yangzhenxi/WX221546 20140612 end */

#/* <DTS2014101602303  jiweifeng/jwx206032 20141016 begin */
obj-$(CONFIG_MSM_CAMERA_SENSOR) += sensor_otp_ov5670.o
#/* DTS2014101602303 jiweifeng/jwx206032 20141016 end > */

#/* <DTS2014101105981 jiweifeng/jwx206032 20141011 begin */
obj-$(CONFIG_MSM_CAMERA_SENSOR) += sensor_otp_s5k5e2.o
#/* DTS2014101105981 jiweifeng/jwx206032 20141011 end> */

#/* <DTS2014070903091 jiweifeng/jwx206032 20140709 begin */
obj-$(CONFIG_MSM_CAMERA_SENSOR) += sensor_otp_ov13850_sunny_p13v01h.o
#/* DTS2014070903091 jiweifeng/jwx206032 20140709 end> */
#/* <DTS2014081404502 wangguoying 20140814 begin */
obj-$(CONFIG_MSM_CAMERA_SENSOR) += sensor_otp_imx328_sunny_p13n10a.o
#/* DTS2014081404502 wangguoying 20140814 end >*/

#/*< DTS2014112103803  gwx229921 20141211 begin */
#/*< DTS2014112103818 gwx229921 20141211 begin */
obj-$(CONFIG_MSM_CAMERA_SENSOR) += sensor_otp_ov5648_foxconn_132.o
obj-$(CONFIG_MSM_CAMERA_SENSOR) += sensor_otp_s5k4e1_sunny_132.o
obj-$(CONFIG_MSM_CAMERA_SENSOR) += sensor_otp_ov13850_liteon_p13v01h.o
#/* DTS2014112103818 gwx229921 20141211 end >*/
#/*DTS2014112103803 gwx229921 20141211 end >*/

obj-$(CONFIG_IMX132) += imx132.o
obj-$(CONFIG_IMX134) += imx134.o
obj-$(CONFIG_OV8825) += ov8825.o
obj-$(CONFIG_OV8865) += ov8865.o
obj-$(CONFIG_s5k4e1) += s5k4e1.o
obj-$(CONFIG_OV12830) += ov12830.o
obj-$(CONFIG_OV9724) += ov9724.o
obj-$(CONFIG_HI256) += hi256.o
obj-$(CONFIG_OV5648) += ov5648.o
obj-$(CONFIG_MT9M114) += mt9m114.o
obj-$(CONFIG_MT9M114) += ov5645.o
obj-$(CONFIG_MT9M114) += ov7695.o
obj-$(CONFIG_SP1628) += sp1628.o
obj-$(CONFIG_GC0339) += gc0339.o
