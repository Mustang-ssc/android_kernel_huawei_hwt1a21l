#/* < DTS2014020700267  caowei 20140210 begin */
# /* < DTS2013123002532  weiqiangqiang  20131219 begin*/
#
# Makefile for the touchscreen drivers.
#

# Each configuration option enables a list of files.


#focaltech   
obj-$(CONFIG_FOCALTECH_FT6X06_UPDATA_FW)    += ft6x06_ex_fun.o 
#/* < DTS2014050708300 caowei 201400507 begin */
extlibs :=$(PWD)/drivers/input/touchscreen/focaltech/focal_scap_test_lib.a
ldflags-$(CONFIG_FOCALTECH_FT6X06_UPDATA_FW) +=--whole-archive $(extlibs) --no-whole-archive
#/* DTS2014050708300 caowei 201400507 end >*/
obj-$(CONFIG_FOCALTECH_FOCALTECH_USERDATA)	+= focaltech_ctl.o
obj-$(CONFIG_FOCALTECH_FT6X06_SYSDEBUG)  	+= ft6x06_sysdebug.o
obj-$(CONFIG_FOCALTECH_FT6X06_TS)    		+= ft6x06_ts.o 

#/* < DTS2014050708300 caowei 201400507 begin */
#delete scap test file, use lib to test
#/* DTS2014050708300 caowei 201400507 end >*/

#/*DTS2013123002532  weiqiangqiang  20131219 end >*/
#/* DTS2014020700267  caowei 20140210 end >*/
