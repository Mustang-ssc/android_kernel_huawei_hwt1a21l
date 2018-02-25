/* < DTS2014080506603  caowei 201400806 begin */
/*******************************************************************
                   Copyright 2008 - 2013, Huawei Tech. Co., Ltd.
                             ALL RIGHTS RESERVED

Filename      : synaptics_dsx_esd.h
Author        : c00248442
Creation time : 2014/8/2
Description   : 

Version       : 1.0
********************************************************************/
#include <linux/workqueue.h>
#include <linux/synaptics_dsx_i2c.h>
/* < DTS2014090101734  caowei 201400902 begin */
#include <linux/hw_tp_common.h>
/* DTS2014083005839  caowei 201400902 end >*/
#define SYNAPTICS_ESD_CHECK_TIME 1500
#define SYNAPTICS_STATUS_REG 0x0013
#define SYNAPTICS_ESD_RETRY_TIMES 3

/* < DTS2014090101734  caowei 201400902 begin */
#define SYNAPTICS_HOLSTER_INFO_LENGTH (64) 

struct synaptics_esd {
	atomic_t esd_check_status;
	atomic_t irq_status;
	struct workqueue_struct * esd_work_queue;
	struct delayed_work esd_work;
};

typedef enum esd_status {
	ESD_CHECK_NOT_READY,
	ESD_CHECK_STOPED,
	ESD_CHECK_START,
} ESD_STATUS;
/* DTS2014090101734  caowei 201400902 end >*/

typedef int (* synaptics_read)(struct synaptics_rmi4_data *, unsigned short, unsigned char *, unsigned short);
typedef int (* synaptics_write)(struct synaptics_rmi4_data *, unsigned short, unsigned char *, unsigned short);

int synaptics_dsx_esd_init(struct synaptics_rmi4_data *rmi4_data, synaptics_read read, synaptics_write write);
int synaptics_dsx_esd_start(void);
/* < DTS2014090101734  caowei 201400902 begin */
int synaptics_dsx_esd_stop(void);
void synaptics_dsx_esd_suspend(void);
void synaptics_dsx_esd_resume(void);
/* DTS2014090101734  caowei 201400902 end >*/
/* < DTS2015012105205  zhoumin wx222300 20150121 begin */
#ifdef CONFIG_HUAWEI_DSM
ssize_t synaptics_dsm_record_esd_err_info( int err_num );
#endif
/* DTS2015012105205  zhoumin wx222300 20150121 end > */
/* DTS2014080506603  caowei 201400806 end >*/


