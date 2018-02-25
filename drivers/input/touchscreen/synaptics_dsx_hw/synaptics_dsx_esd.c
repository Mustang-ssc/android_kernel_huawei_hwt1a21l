/* < DTS2014080506603  caowei 201400806 begin */
/*******************************************************************
                   Copyright 2008 - 2013, Huawei Tech. Co., Ltd.
                             ALL RIGHTS RESERVED

Filename      : synaptics_dsx_esd.c
Author        : c00248442
Creation time : 2014/8/2
Description   : 

Version       : 1.0
********************************************************************/

#include <linux/init.h>
#include <linux/printk.h>
/* < DTS2015012105205  zhoumin wx222300 20150121 begin */
#ifdef CONFIG_HUAWEI_DSM
#include <linux/dsm_pub.h>
#endif
/* DTS2015012105205  zhoumin wx222300 20150121 end > */
#include <linux/delay.h>
#include <linux/synaptics_dsx_i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/synaptics_dsx.h>
#include <linux/huawei_tp_adapter.h>
#include <linux/synaptics_dsx_i2c.h>
#include <linux/of_gpio.h>
#include <linux/hw_tp_common.h>
#include <linux/pm_runtime.h>
/* < DTS2015012105205  zhoumin wx222300 20150121 begin */
//delete one line
/* DTS2015012105205  zhoumin wx222300 20150121 end > */

#include "synaptics_dsx_esd.h"

static struct synaptics_esd synaptics_dsx_esd;
static struct synaptics_rmi4_data *g_rmi4_data = NULL;

synaptics_read synaptics_esd_read = NULL;
/* < DTS2014090101734  caowei 201400902 begin */
/* move ic reset to i2c.c */
/* DTS2014090101734  caowei 201400902 end >*/
/*****************************************************************
Parameters    :  work
Return        :    
Description   :  check if is running
*****************************************************************/
static void synaptics_esd_work(struct work_struct *work)
{
	int i = 0;
	int ret = 0;
	unsigned char data = 0x00;
	
	/* < DTS2014090101734  caowei 201400902 begin */
	tp_log_debug("%s %d:synaptics esd check is working\n", __func__, __LINE__);
	/* if irq is be handled, cancle esd check */
	ret = atomic_read(&(synaptics_dsx_esd.irq_status));
	if (ret != 0) 
	{
		tp_log_err("%s %d:synaptics ic is handle irq, count = %d.\n", 
					__func__, __LINE__, ret);
		goto exit;
	}

	/* DTS2014090101734  caowei 201400902 end >*/
	/* read 3 times, if success, ic is working, if all 3 times read 
		fail, ic is dead */
	for (i = 0; i < SYNAPTICS_ESD_RETRY_TIMES; i++) 
	{
		ret = synaptics_esd_read(g_rmi4_data, SYNAPTICS_STATUS_REG, 
									&data, sizeof(data));
		if (ret > 0) 
		{
			break;
		}
	}

	/* < DTS2014090101734  caowei 201400902 begin */
	if (ret <= 0 && i == SYNAPTICS_ESD_RETRY_TIMES)
	{
		tp_log_err("%s %d:synaptics ic is dead\n", __func__, __LINE__);
/* < DTS2015012105205  zhoumin wx222300 20150121 begin */
#ifdef CONFIG_HUAWEI_DSM
		synp_tp_report_dsm_err(DSM_TP_ESD_ERROR_NO, ret);
#endif
/* DTS2015012105205  zhoumin wx222300 20150121 end > */
		synaptics_dsx_hardware_reset(g_rmi4_data);
	}
	else
	{
		tp_log_info("%s %d:synaptics ic is working\n", __func__, __LINE__);
	}

exit:

	tp_log_debug("%s %d:synaptics data = %d\n", __func__, __LINE__, data);
	/* DTS2014090101734  caowei 201400902 end >*/
	queue_delayed_work(synaptics_dsx_esd.esd_work_queue, &synaptics_dsx_esd.esd_work, msecs_to_jiffies(SYNAPTICS_ESD_CHECK_TIME));
}


/*****************************************************************
Parameters    :  rmi4_data
                 read     
                 write    
Return        :  ok:0, error:-1
Description   :  init ESD parameter
*****************************************************************/
int synaptics_dsx_esd_init(struct synaptics_rmi4_data *rmi4_data, 
							synaptics_read read, synaptics_write write)
{
	/* < DTS2014090101734  caowei 201400902 begin */
	if (NULL == rmi4_data || NULL == read || NULL == write) 
	{
		tp_log_err("%s %d:input parameter is NULL\n", __func__, __LINE__);
		return -1;
	}

	tp_log_debug("%s %d:synaptics esd init\n", __func__, __LINE__);
	/* DTS2014090101734  caowei 201400902 end >*/
	g_rmi4_data = rmi4_data;
	synaptics_esd_read = read;
	
	INIT_DELAYED_WORK(&synaptics_dsx_esd.esd_work, synaptics_esd_work);
	
	synaptics_dsx_esd.esd_work_queue = 
				create_singlethread_workqueue("synaptics_esd_workqueue");
	if (!synaptics_dsx_esd.esd_work_queue)
	{
		/* < DTS2014090101734  caowei 201400902 begin */
		tp_log_err("%s %d:synaptics esd workqueue alloc failed\n", __func__, __LINE__);
		/* DTS2014090101734  caowei 201400902 end >*/
		return -1;
	}
	
	/* < DTS2014090101734  caowei 201400902 begin */
	/* set esd check thread state */
	atomic_set(&(synaptics_dsx_esd.esd_check_status), ESD_CHECK_STOPED);
	atomic_set(&(synaptics_dsx_esd.irq_status), 0);
	/* DTS2014090101734  caowei 201400902 end >*/
	return 0;
}

/*****************************************************************
Parameters    :  void
Return        :  success return work number,fail return 0; 
Description   :  
*****************************************************************/
int synaptics_dsx_esd_start(void) 
{
	int ret = 0;
	
	/* < DTS2014090101734  caowei 201400902 begin */
	tp_log_info("%s %d:start synaptics esd check\n", __func__, __LINE__);	
	if (ESD_CHECK_STOPED == atomic_read(&synaptics_dsx_esd.esd_check_status))
	{
		ret = queue_delayed_work(synaptics_dsx_esd.esd_work_queue, &synaptics_dsx_esd.esd_work, msecs_to_jiffies(SYNAPTICS_ESD_CHECK_TIME));
		if (!ret) {
			tp_log_err("%s %d:queue_delayed_work fail\n", __func__, __LINE__);
			return ret;
		}
		
		atomic_set(&(synaptics_dsx_esd.esd_check_status), ESD_CHECK_START);
	}
	else
	{
		tp_log_err("%s %d:synaptics esd check is not ready\n", __func__, __LINE__);
		ret = 0;
	}
	/* DTS2014090101734  caowei 201400902 end >*/
	
	return ret;
}

/*****************************************************************
Parameters    :  void
Return        :    
Description   :  suspend esd check
*****************************************************************/
int synaptics_dsx_esd_stop(void)
{
	int ret = 0;
	
	/* < DTS2014090101734  caowei 201400902 begin */
	tp_log_err("%s %d:stop synaptics esd check\n", __func__, __LINE__);
	if (ESD_CHECK_START == atomic_read(&synaptics_dsx_esd.esd_check_status))
	{
		ret = cancel_delayed_work(&synaptics_dsx_esd.esd_work);
		if (!ret) {
			tp_log_err("%s %d:stop synaptics esd fail.\n", __func__, __LINE__);
			return -1;
		}
		flush_delayed_work(&synaptics_dsx_esd.esd_work);
		atomic_set(&(synaptics_dsx_esd.esd_check_status), ESD_CHECK_STOPED);
	}
	else
	{
		tp_log_err("%s %d:synaptics esd check is not running\n", __func__, __LINE__);
	}
	/* DTS2014090101734  caowei 201400902 end >*/
	
	return ret;
}

/* < DTS2014090101734  caowei 201400902 begin */
/*****************************************************************
Parameters    :  void
Return        :  void  
Description   :  set irq_handle flag, esd work will not read ic
*****************************************************************/
void synaptics_dsx_esd_suspend(void)
{
	int resume_count = atomic_read(&synaptics_dsx_esd.irq_status);
	atomic_set(&(synaptics_dsx_esd.irq_status), resume_count - 1);		
	tp_log_debug("%s %d:synaptics esd check suspend, count = %d\n", 
				__func__, __LINE__, resume_count - 1);
}

/*****************************************************************
Parameters    :  void
Return        :    
Description   :  set irq_handle flag, esd work will read ic
*****************************************************************/
void synaptics_dsx_esd_resume(void)
{
	int resume_count = atomic_read(&synaptics_dsx_esd.irq_status);
	atomic_set(&(synaptics_dsx_esd.irq_status), resume_count + 1);
	tp_log_debug("%s %d:synaptics esd check resume, count = %d\n", 
				__func__, __LINE__, resume_count + 1);
}
/* DTS2014090101734  caowei 201400902 end >*/
/* < DTS2015012105205  zhoumin wx222300 20150121 begin */
#ifdef CONFIG_HUAWEI_DSM
/*****************************************************************
Parameters    :  err_num
Return        :    
Description   :  save esd error to dsm
*****************************************************************/
ssize_t synaptics_dsm_record_esd_err_info( int err_num )
{

	ssize_t size = 0;
	struct dsm_client *tp_dclient = tp_dsm_get_client();

	/* esd err number */
	size =dsm_client_record(tp_dclient, "esd err number:%d\n", err_num);

	return size;
}
#endif
/* DTS2015012105205  zhoumin wx222300 20150121 end > */
/* DTS2014080506603  caowei 201400806 end >*/

