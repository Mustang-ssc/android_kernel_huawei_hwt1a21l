#
# Makefile for MMC/SD card drivers
#

obj-$(CONFIG_MMC_BLOCK)		+= mmc_block.o
mmc_block-objs			:= block.o queue.o
obj-$(CONFIG_MMC_TEST)		+= mmc_test.o

obj-$(CONFIG_SDIO_UART)		+= sdio_uart.o

obj-$(CONFIG_MMC_BLOCK_TEST)		+= mmc_block_test.o

#/* < DTS2014042606672 gaoxu 20140424 begin */
obj-$(CONFIG_HW_MMC_TEST_MODULE)	+= hw_mmc_test.o
#/* < DTS2014111202482 duanhuan 20141118 begin */
obj-$(CONFIG_HW_MMC_AGING_TEST_MODULE)	+= hw_mmc_aging_test.o
#/* DTS2014111202482 duanhuan 20141118 end > */
#/* DTS2014041506599 gaoxu 20140424 end > */
#/* < DTS2014072602310 zhaoyingchun 20140726 begin */ 
obj-$(CONFIG_MMC_FFU)		+= ffu.o
#/* DTS2014072602310 zhaoyingchun 20140726 end > */ 

