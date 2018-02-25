ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor
ccflags-y += -Idrivers/media/platform/msm/camera_v2/codecs
ccflags-y += -Idrivers/media/platform/msm/camera_v2/isps
ccflags-y += -Idrivers/media/platform/msm/camera_v2/pproc
ccflags-y += -Idrivers/media/platform/msm/camera_v2/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2/jpeg_10

obj-$(CONFIG_MSMB_CAMERA) += msm.o
#/* < DTS2014072601546 yanhuiwen/0083550 20140723 begin */
obj-$(CONFIG_MSMB_CAMERA) += hw_camera_common.o
#/* DTS2014072601546 yanhuiwen/00283550 20140723 end > */
#/* < DTS2014081209165 Zhangbo 00166742 20140812 begin */
#/* < DTS2014111105636 Houzhipeng hwx231787 20141111 begin */
obj-$(CONFIG_HUAWEI_DSM) += msm_camera_dsm.o
#/* DTS2014111105636 Houzhipeng hwx231787 20141111 end > */
#/* DTS2014081209165 Zhangbo 00166742 20140812 end > */
obj-$(CONFIG_MSMB_CAMERA) += camera/
obj-$(CONFIG_MSMB_CAMERA) += msm_vb2/
obj-$(CONFIG_MSMB_CAMERA) += sensor/
obj-$(CONFIG_MSMB_CAMERA) += pproc/
obj-$(CONFIG_MSMB_CAMERA) += isp/
obj-$(CONFIG_MSMB_CAMERA) += ispif/
obj-$(CONFIG_MSMB_JPEG) += jpeg_10/
obj-$(CONFIG_MSMB_CAMERA) += msm_buf_mgr/
#/*<DTS2014060405107 luozhi/wx217218 20140604 begin*/
obj-$(CONFIG_MSMB_CAMERA) += hw_camera_log.o
#/*DTS2014060405107 luozhi/wx217218 20140604 end>*/

