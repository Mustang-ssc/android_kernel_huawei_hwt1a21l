#
# Makefile for the kernel mmc device drivers.
#

subdir-ccflags-$(CONFIG_MMC_DEBUG) := -DDEBUG

obj-$(CONFIG_MMC)		+= core/
obj-$(CONFIG_MMC)		+= card/
obj-$(subst m,y,$(CONFIG_MMC))	+= host/
#/* < DTS2014062704895 zhaoyingchun 20140628 begin */
obj-$(CONFIG_MMC_SWRM)          += swrm/
#/* DTS2014062704895 zhaoyingchun 20140628 end > */

