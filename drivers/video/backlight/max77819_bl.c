/* <DTS2014080900552 chenyuanquan 20140825 begin */
/*
 * max77819.c
 *
 * Copyright 2013 Maxim Integrated Products, Inc.
 * Gyungoh Yoo <jack.yoo@maximintegrated.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/mfd/max77819.h>
/*< DTS2014111007403 tianye/293347 20141115 begin*/
/* Reduce the voltage gradually when the system is in the process of shutting down*/
#include <linux/delay.h>
/*DTS2014111007403 tianye/293347 20141115 end >*/
#define MAX_BRIGHTNESS     0xFF

/* registers */
#define MAX77819_WLED_INT		0x9B
#define MAX77819_WLED_INT_MASK	0x9C
#define MAX77819_WLEDBSTCNTL	0x98
#define MAX77819_IWLED			0x99

/* MAX77819_WLED_INT */
#define MAX77819_WLEDOL			0x10
#define MAX77819_WLEDOVP_I		0x80

/* MAX77819_WLED_INT_MASK */
#define MAX77819_WLEDOL_M		0x10
#define MAX77819_WLEDOVP_M		0x80

/* MAX77819_WLEDBSTCNTL */
#define MAX77819_WLEDOVP		0x02
#define MAX77819_WLEDFOSC		0x0C
#define MAX77819_WLEDPWM2EN		0x10
#define MAX77819_WLEDPWM1EN		0x20
#define MAX77819_WLED2EN		0x40
#define MAX77819_WLED1EN		0x80
#define MAX77819_WLED_EN		0xF0

/* MAX77819_IWLED */
#define MAX77819_CUR			0xFF

struct max77819_wled {
    struct max77819_dev                    *chip;
    struct max77819_io                     *io;
    struct device                          *dev;
    struct backlight_device                *bl_dev;
    int                                     brightness;
};

static struct backlight_device *gMax77819bl = NULL;
/*< DTS2014111007403 tianye/293347 20141115 begin*/
/* Reduce the voltage gradually when the system is in the process of shutting down*/
static int max77819_bl_off(struct backlight_device *bl_dev)
{
    struct max77819_wled *me = bl_get_data(bl_dev);
/*< DTS2015012007445  tianye/293347 20150120 begin*/
/* coverity scans camera-LCD code modify  */
	struct regmap *regmap = NULL;
	int ret;
	int curr_val = 0;
		if(!(me&&me->io&&me->io->regmap)){
		pr_err("The existence of null pointer from bl_get_data in max77819_bl_off\n");
		 return 0;
		}
	regmap = me->io->regmap;
/*DTS2015012007445 tianye/293347 20150120 end >*/
	/*get current backlight current setting*/
	ret = regmap_read(regmap, MAX77819_IWLED, &curr_val);
	/*gradually adjust the current to 0*/
	do{
		curr_val = curr_val - 21; //reduce 2 mA
		if(curr_val <= 0)
			curr_val = 0;

	ret |= regmap_write(regmap, MAX77819_IWLED, curr_val);
			udelay(200);
	}while(curr_val > 0);
	ret |= regmap_update_bits(regmap, MAX77819_WLEDBSTCNTL, MAX77819_WLED1EN | MAX77819_WLED2EN | MAX77819_WLEDPWM1EN | MAX77819_WLEDPWM2EN, 0); //disable dual wled
	if (IS_ERR_VALUE(ret))
		dev_err(&bl_dev->dev, "failed to power off backlight : %d\n", ret);
		
	return ret;
}
/*DTS2014111007403 tianye/293347 20141115 end >*/
static int max77819_bl_update_status(struct backlight_device *bl_dev)
{
    struct max77819_wled *me = bl_get_data(bl_dev);
/*< DTS2015012007445  tianye/293347 20150120 begin*/
/* coverity scans camera-LCD code modify  */
	struct regmap *regmap = NULL;
	int brightness = bl_dev->props.brightness;
	int ret = 0;
	unsigned int value = 0;
	if(!(me&&me->io&&me->io->regmap)){
		pr_err("the existence of null pointer in max77819_bl_update_status.\n ");
		return 0;
	}
	regmap = me->io->regmap;
/*DTS2015012007445 tianye/293347 20150120 end >*/
	printk("max77819 update brightness\n");
	if (brightness == 0)
		ret = max77819_bl_off(bl_dev);
	else
	{
		if (brightness > MAX_BRIGHTNESS)
		brightness = MAX_BRIGHTNESS;

		ret = regmap_read(regmap, MAX77819_WLEDBSTCNTL, &value);
        if (IS_ERR_VALUE(ret)){
			dev_err(&bl_dev->dev, "can't read WLEDBSTCNTL : %d\n", ret);
			return ret;
		}
		
		if((value&(MAX77819_WLED1EN|MAX77819_WLED2EN)) != 0xc0){
			/*Clear status register before enable the device*/
/*< DTS2015012007445  tianye/293347 20150120 begin*/
/* coverity scans camera-LCD code modify  */
		ret = regmap_read(regmap, MAX77819_WLED_INT, &value);
		if (IS_ERR_VALUE(ret)){
			dev_err(&bl_dev->dev, "can't read WLED_INT : %d\n", ret);
		}
/*DTS2015012007445 tianye/293347 20150120 end >*/
			ret = regmap_write(regmap, MAX77819_IWLED, 0);
			if (IS_ERR_VALUE(ret))
			{
				dev_err(&bl_dev->dev, "can't write IWLED : %d\n", ret);
				return ret;
			}
/*< DTS2014091701123 huangli 20140917 begin */
			//change 0x98 = 0xF8: 28v 1.47MHZ the same wiht lk.
			ret = regmap_update_bits(regmap, MAX77819_WLEDBSTCNTL, MAX77819_WLEDOVP | MAX77819_WLEDFOSC | MAX77819_WLED_EN,
					0x00 | 0x08 | MAX77819_WLED_EN);
/* DTS2014091701123 huangli 20140917 end > */
			if (IS_ERR_VALUE(ret))
				dev_err(&bl_dev->dev, "can't write WLEDBSTCNTL : %d\n", ret);
		}
		
		ret = regmap_write(regmap, MAX77819_IWLED, (unsigned int)brightness);
		if (IS_ERR_VALUE(ret))
		{
			dev_err(&bl_dev->dev, "can't write IWLED : %d\n", ret);
			return ret;
		}
	}

	return ret;
}
/*< DTS2014111007403 tianye/293347 20141115 begin*/
/* Reduce the voltage gradually when the system is in the process of shutting down*/
void max77819_update_brightness(int brightness)
{
	if (gMax77819bl){
	  if(gMax77819bl->props.brightness != brightness)
/*< DTS2014122508475 tianye/293347 20141225 begin*/
/* optimize platform code,remove the superfluous log information */
	  {
		gMax77819bl->props.brightness = brightness;		
		backlight_update_status(gMax77819bl);
	  }
/*DTS2014122508475 tianye/293347 20141225 end >*/
	}
}
/*DTS2014111007403 tianye/293347 20141115 end >*/
static int max77819_bl_get_brightness(struct backlight_device *bl_dev)
{
    struct max77819_wled *me = bl_get_data(bl_dev);
/*< DTS2015012007445  tianye/293347 20150120 begin*/
/* coverity scans camera-LCD code modify  */
	struct regmap *regmap = NULL;
/*DTS2015012007445 tianye/293347 20150120 end >*/
	unsigned int value;
	int ret;
/*< DTS2015012007445  tianye/293347 20150120 begin*/
/* coverity scans camera-LCD code modify  */
	if(!(me&&me->io&&me->io->regmap)){
		pr_err("existence of null pointer from in max77819_bl_get_brightness\n");
		return 0;
   }
	regmap = me->io->regmap;
/*DTS2015012007445 tianye/293347 20150120 end >*/
	ret = regmap_read(regmap, MAX77819_WLEDBSTCNTL, &value);
	if (IS_ERR_VALUE(ret))
	{
		dev_err(&bl_dev->dev, "can't read WLEDBSTCNTL : %d\n", ret);
		return ret;
	}
	else if ((value & (MAX77819_WLED2EN | MAX77819_WLED1EN)) == 0)
		return 0;

	ret = regmap_read(regmap, MAX77819_IWLED, &value);
	if (IS_ERR_VALUE(ret))
	{
		dev_err(&bl_dev->dev, "can't read IWLED : %d\n", ret);
		return ret;
	}
	
	return value;
}

static const struct backlight_ops max77819_bl_ops = {
	.update_status  = max77819_bl_update_status,
	.get_brightness = max77819_bl_get_brightness,
};

static int max77819_bl_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct max77819_dev *chip = dev_get_drvdata(dev->parent);
/*< DTS2015012007445  tianye/293347 20150120 begin*/
/* coverity scans camera-LCD code modify  */
    struct max77819_wled *me = NULL;
/*DTS2015012007445 tianye/293347 20150120 end >*/
    struct backlight_properties bl_props;
    int rc;

    
	printk("backlight attached\n");
	me = devm_kzalloc(dev, sizeof(*me), GFP_KERNEL);
	if (unlikely(!me)) {
		dev_err(dev, "out of memory (%uB requested)\n", sizeof(*me));
		return -ENOMEM;
	}
	dev_set_drvdata(dev, me);

	me->io	 = max77819_get_io(chip);
	me->dev  = dev;
/*< DTS2015012007445  tianye/293347 20150120 begin*/
/* coverity scans camera-LCD code modify  */
    if(!(me->io&&me->io->regmap)){
		pr_err("get me->io->regmap addr fail\n ");
		return -ENOMEM;
	}
/*DTS2015012007445 tianye/293347 20150120 end >*/
	/*Mask all interrupts*/
	regmap_write(me->io->regmap, MAX77819_WLED_INT_MASK, 0xff);
	
	/* brightness is initially MAX */
	me->brightness = MAX_BRIGHTNESS;

	memset(&bl_props, 0x00, sizeof(bl_props));
	bl_props.type			= BACKLIGHT_RAW;
	bl_props.brightness = MAX_BRIGHTNESS;
	bl_props.max_brightness = MAX_BRIGHTNESS;

	me->bl_dev = backlight_device_register("max77819-bl", dev, me,
		&max77819_bl_ops, &bl_props);
	if (unlikely(IS_ERR(me->bl_dev))) {
		rc = PTR_ERR(me->bl_dev);
		me->bl_dev = NULL;
		goto abort;
	}

	gMax77819bl = me->bl_dev;
	
	return 0;

abort:
    devm_kfree(dev, me);	
	dev_set_drvdata(dev, NULL);
	return rc;
}
/*< DTS2014111801969 tianye/293347 20141118 begin*/
/* The pointer type of function returned is inconsistent.*/
static int max77819_bl_remove(struct platform_device *pdev)
{
	struct max77819_wled *bl = platform_get_drvdata(pdev);
       if(bl&&(bl->bl_dev))
	{
	bl->bl_dev->props.brightness = 0;
	backlight_update_status(bl->bl_dev);
	backlight_device_unregister(bl->bl_dev);
       }
	return 0;
}
/*DTS2014111801969 tianye/293347 20141118 end >*/
/*< DTS2014111007403 tianye/293347 20141115 begin*/
/* Reduce the voltage gradually when the system is in the process of shutting down*/
static void max77819_bl_shutdown(struct platform_device *pdev)
{
	max77819_update_brightness(0);
	printk("exit %s \n",__func__);
}
/*DTS2014111007403 tianye/293347 20141115 end >*/
#ifdef CONFIG_OF
static struct of_device_id max77819_bl_of_ids[] = {
    { .compatible = "maxim,"MAX77819_WLED_NAME },
    { },
};
MODULE_DEVICE_TABLE(of, max77819_bl_of_ids);
#endif /* CONFIG_OF */
/*< DTS2014111007403 tianye/293347 20141115 begin*/
/* Reduce the voltage gradually when the system is in the process of shutting down*/
static struct platform_driver max77819_bl_driver = {
	.driver	= {
		.name = MAX77819_WLED_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
    		.of_match_table  = max77819_bl_of_ids,
#endif /* CONFIG_OF */			
	},
	.probe = max77819_bl_probe,
	.remove = max77819_bl_remove,
	.shutdown = max77819_bl_shutdown,
};
/*DTS2014111007403 tianye/293347 20141115 end >*/
static int __init max77819_bl_init(void)
{
	return platform_driver_register(&max77819_bl_driver);
}
module_init(max77819_bl_init);

static void __exit max77819_bl_exit(void)
{
	platform_driver_unregister(&max77819_bl_driver);
}
module_exit(max77819_bl_exit);

MODULE_ALIAS("platform:max77819-backlight");
MODULE_AUTHOR("Gyungoh Yoo <jack.yoo@maximintegrated.com>");
MODULE_DESCRIPTION("MAX77819 backlight");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
/* DTS2014080900552 chenyuanquan 20140825 end> */
