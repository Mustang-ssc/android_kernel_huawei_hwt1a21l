/**************************************************
 Copyright (C), 2012-2015, Huawei Tech. Co., Ltd.
 File Name: kernel/drivers/huawei/device/equip/equip.c
 Author:  n00164272       Version:  1   Date:20121208
 Description:equip for AT normalize 
 Version: V1.0
 Function List:
 History:
 
**************************************************/

/*Begin <DTS2012120510674>  add by nielimin/00164272 for normalizing internal interface of AT command  2012/12/5*/
#include <linux/miscdevice.h>
#include <linux/kobject.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/equip.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/module.h>

#define DEVICE_EQUIP_NAME "equip"

struct EQUIP_FUNC_ARRARY equip_func;

struct EQUIP_PARAM args;

void register_equip_func(enum EQUIP_DEVICE equip_dev, enum EQUIP_OPS ops, PFunc pfunc)
{
	int oper = ops;
	equip_func.pfunc[equip_dev][oper] = pfunc;
}
EXPORT_SYMBOL(register_equip_func);

static void equip_set_index(unsigned long index)
{	
	enum EQUIP_OPS ops = index%2;
	equip_func.equip_index = index/2;	
	equip_func.equip_ops = ops;	
	printk(KERN_INFO "--[%s]----equip_func.equip_index = %d equip_func.equip_ops = %d\n", __func__,equip_func.equip_index, equip_func.equip_ops);	
}

static unsigned long equip_get_index(void)
{
	return equip_func.equip_index;
}

static ssize_t equip_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{	
	int oper = equip_func.equip_ops;
	if( equip_func.pfunc[equip_func.equip_index][oper] == NULL)
	{
		printk(KERN_INFO "[%s]-equip function index %d has not registered-\n", __func__, equip_func.equip_index);
		return -EFAULT;
	}
	(*equip_func.pfunc[equip_func.equip_index][oper])((void*)(&args));
	printk(KERN_INFO "--[%s]--args->out_str = %s--\n", __func__, args.str_out);	
	if (copy_to_user(buf, args.str_out, strlen(args.str_out)))
	{
		printk(KERN_INFO "--[%s]--copy data to user error--\n", __func__);	
		return -EFAULT;
	}
	return strlen(args.str_out);
}

static ssize_t equip_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{		
	int oper = equip_func.equip_ops;
	if( equip_func.pfunc[equip_func.equip_index][oper] == NULL)
	{
		printk(KERN_INFO "[%s]-equip function index %d has not registered-\n", __func__, equip_func.equip_index);
		return -EFAULT;
	}
	printk(KERN_INFO "[%s]----buf = %s--\n",__func__, buf);		
	if(copy_from_user(args.str_in, buf, count))
	{
		printk(KERN_INFO "--[%s]--copy data from user error--\n", __func__);	
		return -EFAULT;
	}	
	printk(KERN_INFO "[%s]----args.str_in = %s--\n",__func__, args.str_in);	
	(*equip_func.pfunc[equip_func.equip_index][oper])((void*)(&args));				
	return count;
}


static long equip_ioctl(struct file *filp, unsigned int cmd, unsigned long param)
{
	int ret = 0;
	switch(cmd)
	{
		case IOCTL_EQUIP_SET_INDEX :
			equip_set_index(param);
			break;
		case IOCTL_EQUIP_GET_INDEX:
			ret = equip_get_index();
			break;		
		default:
			ret = -EINVAL;
			break;		
	}
	return ret;
}

static struct file_operations dev_equip_fops = 
{  
	.owner   		  = 	THIS_MODULE,
	.unlocked_ioctl   = 	equip_ioctl,
	.read 	 		  = 	equip_read,
	.write	 		  = 	equip_write,
};  
  
static struct miscdevice misc_equip = 
{  
    .minor = MISC_DYNAMIC_MINOR,  
    .name = DEVICE_EQUIP_NAME,  
    .fops = &dev_equip_fops,  
};  

static int __init dev_equip_init(void)
{
	int ret;	
	if ((ret = misc_register(&misc_equip)))
	{		
		printk(KERN_ERR "[%s], misc_register error!", __func__);
		goto equip_fail_1;
	}
	return 0;
equip_fail_1:		
	return ret;
}

static void __exit dev_equip_exit(void)
{	
	misc_deregister(&misc_equip);
}

module_init(dev_equip_init);
module_exit(dev_equip_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("nielmin/00164272");
/*End <DTS2012120510674>  add by nielimin/00164272 for normalizing internal interface of AT command  2012/12/5*/
