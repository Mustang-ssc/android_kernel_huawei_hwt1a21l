/*DTS2014091104672 xiongxi xwx234328 begin*/
#include <linux/module.h>
#include <linux/gfp.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <linux/iopoll.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>


struct msm_sar
{
    struct platform_device *pdev;
    struct timer_list  timer_detect;
    unsigned int irq;
    unsigned int status;
    unsigned int gpio_pin;
};
static struct wake_lock sar_wake_lock;
struct msm_sar *sar_host = NULL;
struct work_struct sar_work; 
struct workqueue_struct *sar_wq = NULL;

unsigned int get_sar_status(unsigned int sar_detection_gpio)
{
    int electrical_lvl;
	
    electrical_lvl = gpio_get_value(sar_detection_gpio);
    pr_info(" electrical_lvl = %d  \n", electrical_lvl);
    return electrical_lvl;
}


static void sar_detect_work(struct work_struct *work)
{
    unsigned int new_status;
    char *sar_on_duty[2]    = { "SAR_STATE=ON DUTY", NULL };
    char *sar_off_duty[2]    = { "SAR_STATE=OFF DUTY", NULL };
    
    new_status = get_sar_status(sar_host->gpio_pin);
    pr_info(KERN_INFO "sar card status change from %s to %s\n",
           sar_host->status == 1 ? "on duty":"off duty",
           new_status== 1 ? "on duty":"off duty" );

    if( new_status != sar_host->status )
    {
        sar_host->status = new_status; 
        kobject_uevent_env(&sar_host->pdev->dev.kobj, KOBJ_CHANGE,( sar_host->status ? sar_on_duty : sar_off_duty));  
        pr_info("send sar state change uevent........................end\n");
    }
    
    wake_unlock(&sar_wake_lock);
    return;
}

static void timer_detect_func(unsigned long arg)
{
    pr_info("sar -%s[%d]: %s\n", __FILE__, __LINE__, __FUNCTION__);  
    queue_work(sar_wq, &sar_work);
}

irqreturn_t sar_gpio_irq_handler(int irq, void *dev_id)
{
    struct msm_sar *sar_host = (struct msm_sar *)dev_id;

    if (irq != sar_host->irq)
        return IRQ_NONE;
    
    wake_lock(&sar_wake_lock);
    mod_timer(&sar_host->timer_detect, jiffies+HZ/2);

    return IRQ_HANDLED;
}

static int msm_sar_gpio_init(void)
{
    int irq;
    int err = 0;
    enum   of_gpio_flags flags = OF_GPIO_ACTIVE_LOW;
 
    sar_host->gpio_pin = of_get_named_gpio_flags(sar_host->pdev->dev.of_node, "qcom,sar-detect", 0, &flags);
    if (!gpio_is_valid(sar_host->gpio_pin))
    {
        dev_err(&sar_host->pdev->dev, "%s: Failed to parser dt cd-gpio\n", __func__);
        goto get_gpio_err;
    }
   
    err = gpio_request_one(sar_host->gpio_pin, GPIOF_DIR_IN, "sar gpio");
    if (err)
        goto request_gpio_err;  
    
    irq = gpio_to_irq(sar_host->gpio_pin);
     if (irq < 0)
    {
        pr_err("msm_sar: probe gpio_to_irq  error!\n");
        goto  get_irq_err;
    }
    sar_host->irq = irq;
		
    err = request_irq(irq, sar_gpio_irq_handler,
                IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "sar", sar_host);
    if (err < 0)
    {
        pr_err("request_irq  error!\n");
        goto request_irq_err;
    }

    init_timer(&sar_host->timer_detect);
    sar_host->timer_detect.function = &timer_detect_func;

    return 0;    

request_irq_err:
get_irq_err:
    gpio_free(sar_host->gpio_pin);    
request_gpio_err:
get_gpio_err:
    return err;
}

static ssize_t online_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{	
    return sprintf(buf, "%d\n", get_sar_status(sar_host->gpio_pin));
}

static ssize_t online_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	return 0;
}

static DEVICE_ATTR(online, S_IRUGO | S_IWUSR, online_show, online_store);

static int sar_msm_probe(struct platform_device  *pdev)
{
    int ret = 0;
    
    sar_host = devm_kzalloc(&pdev->dev, sizeof(struct msm_sar), GFP_KERNEL);
    if (!sar_host)
        return -ENOMEM;

    memset(sar_host, 0, sizeof(struct msm_sar));
    
    sar_host->pdev = pdev;
    
    ret = msm_sar_gpio_init();
    if(ret)
        goto err_sar_init;          

    sar_host->status = get_sar_status(sar_host->gpio_pin);
    
    sar_wq = create_workqueue("sar_workqueue");
    if(!sar_wq)
        goto err_create_workqueue;
    
    INIT_WORK(&sar_work, sar_detect_work);
    wake_lock_init(&sar_wake_lock, WAKE_LOCK_SUSPEND, "sar detect"); 
    ret = device_create_file(&pdev->dev, &dev_attr_online);
    if(ret)
        goto err_create_file;
    
    return 0;

err_create_file:
    destroy_workqueue(sar_wq);
	printk("sar create file failed!\n");
err_create_workqueue:
err_sar_init:
	wake_lock_destroy(&sar_wake_lock);
    kfree(sar_host);
    return ret;
		
}

static int sar_msm_remove(struct platform_device *pdev)
{
    if (gpio_is_valid(sar_host->gpio_pin))
        gpio_free(sar_host->gpio_pin);
    free_irq(sar_host->irq, NULL);
    destroy_workqueue(sar_wq);
    wake_lock_destroy(&sar_wake_lock);
    kfree(sar_host);

    return 0;
}

static const struct of_device_id sar_msm_dt_match[] = {
    { .compatible = "qcom,msm-sar", },
    { },
};


static struct platform_driver sar_msm_driver = {
    .probe     = sar_msm_probe,
    .remove    = sar_msm_remove,
    .driver    = {
        .name  = "qcom,msm-sar",
	.owner = THIS_MODULE,
	.of_match_table = sar_msm_dt_match,
     },
};

static int __init msm_sar_init(void)
{
	
    return platform_driver_register(&sar_msm_driver);	
	
}

static void __exit msm_sar_exit(void)
{

    platform_driver_unregister(&sar_msm_driver);

}


module_init(msm_sar_init);
module_exit(msm_sar_exit);

MODULE_AUTHOR("zhangshaoqi");
MODULE_DESCRIPTION("sar detect driver");
MODULE_LICENSE("GPL");
/*DTS2014091104672 xiongxi xwx234328 end*/
