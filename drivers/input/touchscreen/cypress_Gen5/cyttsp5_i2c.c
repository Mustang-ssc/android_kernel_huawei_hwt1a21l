/*
 * cyttsp5_i2c.c
 * Cypress TrueTouch(TM) Standard Product V5 I2C Driver module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2012-2013 Cypress Semiconductor
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 * Modified by: Cypress Semiconductor for test with device
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */
#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>

#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include "cyttsp5_bus.h"
#include "cyttsp5_core.h"
#include "cyttsp5_i2c.h"
#include "cyttsp5_devtree.h"


#define SYNAPTCS_IRQ_CFG "irq_config"
#define SYNAPTICS_ALGO_ID "algo_id"
#define SYNAPTICS_VDD    "synaptics-vdd"
#define SYNAPTICS_VBUS   "synaptics-io"

//extern int get_boot_into_recovery_flag(void);
u8 g_ts_log_level = 0;

static int cyttsp5_i2c_read_default_(struct cyttsp5_adapter *adap,
	void *buf, int size)
{
	struct cyttsp5_i2c *ts = dev_get_drvdata(adap->dev);
	struct i2c_client *client = ts->client;
	int rc;

	if (!buf || !size)
		return -EINVAL;

	rc = i2c_master_recv(client, buf, size);

	return (rc < 0) ? rc : rc != size ? -EIO : 0;
}

static int cyttsp5_i2c_read_default(struct cyttsp5_adapter *adap,
	void *buf, int size)
{
	struct cyttsp5_i2c *ts = dev_get_drvdata(adap->dev);
	int rc;

	mutex_lock(&ts->lock);
	rc = cyttsp5_i2c_read_default_(adap, buf, size);
	mutex_unlock(&ts->lock);

	return rc;
}

static int cyttsp5_i2c_read_default_nosize_(struct cyttsp5_adapter *adap,
	u8 *buf, u32 max)
{
	struct cyttsp5_i2c *ts = dev_get_drvdata(adap->dev);
	struct i2c_client *client = ts->client;
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;
	u32 size;

	if (!buf)
		return -EINVAL;

	msgs[0].addr = ts->client->addr;
	msgs[0].flags = (client->flags & I2C_M_TEN) | I2C_M_RD;
	msgs[0].len = 2;
	msgs[0].buf = buf;
	rc = i2c_transfer(client->adapter, msgs, msg_count);
	if (rc < 0 || rc != msg_count)
		return (rc < 0) ? rc : -EIO;

	size = get_unaligned_le16(&buf[0]);
	if (!size)
		return 0;

	if (size > max)
		return -EINVAL;

	rc = i2c_master_recv(client, buf, size);

	return (rc < 0) ? rc : rc != (int)size ? -EIO : 0;
}

static int cyttsp5_i2c_read_default_nosize(struct cyttsp5_adapter *adap,
	void *buf, int max)
{
	struct cyttsp5_i2c *ts = dev_get_drvdata(adap->dev);
	int rc;

	mutex_lock(&ts->lock);

	rc = cyttsp5_i2c_read_default_nosize_(adap, buf, max);

	mutex_unlock(&ts->lock);

	return rc;
}

static int cyttsp5_i2c_write_read_specific_(struct cyttsp5_adapter *adap,
		u8 write_len, u8 *write_buf, u8 *read_buf)
{
	struct cyttsp5_i2c *ts = dev_get_drvdata(adap->dev);
	struct i2c_client *client = ts->client;
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;

	if (!write_buf || !write_len)
		return -EINVAL;

	msgs[0].addr = ts->client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = write_len;
	msgs[0].buf = write_buf;
	rc = i2c_transfer(client->adapter, msgs, msg_count);

	if (rc < 0 || rc != msg_count)
		return (rc < 0) ? rc : -EIO;
	else
		rc = 0;

	if (read_buf)
		rc = cyttsp5_i2c_read_default_nosize_(adap, read_buf, 512);

	return rc;
}

static int cyttsp5_i2c_write_read_specific(struct cyttsp5_adapter *adap,
		u8 write_len, u8 *write_buf, u8 *read_buf)
{
	struct cyttsp5_i2c *ts = dev_get_drvdata(adap->dev);
	int rc;

	mutex_lock(&ts->lock);

	rc = cyttsp5_i2c_write_read_specific_(adap, write_len, write_buf,
			read_buf);

	mutex_unlock(&ts->lock);

	return rc;
}

static struct cyttsp5_ops ops = {
	.read_default = cyttsp5_i2c_read_default,
	.read_default_nosize = cyttsp5_i2c_read_default_nosize,
	.write_read_specific = cyttsp5_i2c_write_read_specific,
};

static struct of_device_id cyttsp5_i2c_of_match[] = {
	{ .compatible = "cy,cyttsp5_i2c_adapter", },
	{ }
};
MODULE_DEVICE_TABLE(of, cyttsp5_i2c_of_match);

struct cyttsp5_i2c *g_ts_i2c = NULL;

static int cyttsp5_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *i2c_id)
{
	struct cyttsp5_i2c *ts_i2c;
	struct device *dev = &client->dev;
	const struct of_device_id *match;
	char const *adap_id;
	struct pinctrl *pinctrl;
	struct pinctrl_state *int_pin_default;
	int rc;
	int vbus_gpio;
	int vbus_gpio_1;

	struct regulator *vdd;
	
	printk("%s start\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TS_LOG_ERR( "%s: fail check I2C functionality\n", __func__);
		rc = -EIO;
		goto error_alloc_data_failed;
	}

	ts_i2c = kzalloc(sizeof(struct cyttsp5_i2c), GFP_KERNEL);
	if (ts_i2c == NULL) {
		TS_LOG_ERR( "%s: Error, kzalloc.\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}

	match = of_match_device(of_match_ptr(cyttsp5_i2c_of_match), dev);
	if (match) {
		rc = of_property_read_string(dev->of_node, "cy,adapter_id",
				&adap_id);
		if (rc) {
			TS_LOG_ERR( "%s: OF error rc=%d\n", __func__, rc);
			goto error_free_data;
		}
        TS_LOG_INFO( "%s: start to register devices here.\n", __func__);
		cyttsp5_devtree_register_devices(dev);
	} else {
		adap_id = dev_get_platdata(dev);
	}
    
	vdd = regulator_get(dev, "cyttsp5-vdd");
	if (IS_ERR(vdd)) {
		TS_LOG_ERR("%s: failed to get cypress vdd\n", __func__);
		goto error_free_data;
	}
    
	ts_i2c->vdd = vdd;
    
	rc = regulator_set_voltage(vdd,1850000,1850000);
	if(rc < 0){
		TS_LOG_ERR("%s: failed to set cypress vdd\n", __func__);
		goto error_free_data;
	}
	
	rc = regulator_enable(vdd);
	if (rc < 0) {
		TS_LOG_ERR("%s: failed to enable cypress vdd\n", __func__);
		goto error_free_data;
	}
	
	vbus_gpio = of_get_named_gpio_flags(dev->of_node, "cy,vbus_gpio", 0, NULL);
	if (!gpio_is_valid(vbus_gpio)) {
		TS_LOG_ERR("%s :[TP] get cy,vbus_gpio %d failed.\n",__func__, vbus_gpio);
		//goto error_free_data;
	}
	rc = gpio_request(vbus_gpio, "vbus_gpio");
	if (rc < 0) {
		TS_LOG_ERR("%s: Fail request gpio=%d\n", __func__, vbus_gpio);
		goto error_free_data;
	}

	ts_i2c->vbus_gpio = vbus_gpio;
	rc = gpio_direction_output(vbus_gpio, 1);
	if (rc < 0) {
		TS_LOG_ERR("%s: Fail set output gpio=%d\n",__func__, vbus_gpio);
		goto error_free_data;
	}

	vbus_gpio_1 = of_get_named_gpio_flags(dev->of_node, "cy,vbus_gpio_1", 0, NULL);
	if (!gpio_is_valid(vbus_gpio_1)) {
		TS_LOG_ERR("[TP] get cy,vbus_gpio_1  %d failed.\n", vbus_gpio_1);
		//goto error_free_data;
	}
	rc = gpio_request(vbus_gpio_1, "vbus_gpio_1");
	if (rc < 0) {
		TS_LOG_ERR("%s: Fail request gpio=%d\n", __func__, vbus_gpio_1);
		goto error_free_data;
	}
    
    ts_i2c->vbus_gpio_en = vbus_gpio_1;
    
	rc = gpio_direction_output(vbus_gpio_1, 1);
	if (rc < 0) {
		TS_LOG_ERR("%s: Fail set output gpio=%d\n",__func__, vbus_gpio_1);
		goto error_free_data;
	}
     
	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		TS_LOG_ERR("%s:get pinctrl failed\n",__func__);
		rc = -EIO;
		goto error_pinctrl_get;
	}
	int_pin_default = pinctrl_lookup_state(pinctrl, "int_default");
	if (IS_ERR_OR_NULL(int_pin_default)) {
		TS_LOG_ERR("%s:get int pin default failed\n",__func__);
		rc = -EIO;
		goto error_pinctrl_lookup_state;
	}
	rc = pinctrl_select_state(pinctrl, int_pin_default);
	if (rc) {
		TS_LOG_ERR("%s:set int pin default state failed\n",__func__);
		goto error_pinctrl_select_state;
	}

	msleep(100);
    
	mutex_init(&ts_i2c->lock);
	ts_i2c->client = client;
	ts_i2c->id = (adap_id) ? adap_id : CYTTSP5_I2C_NAME;
	client->dev.bus = &i2c_bus_type;
	i2c_set_clientdata(client, ts_i2c);
	dev_set_drvdata(&client->dev, ts_i2c);

	/*DTS2014121002592 init g_ts_i2c before add_adapter by x00267953 begin*/
	g_ts_i2c = ts_i2c;
	/*DTS2014121002592 init g_ts_i2c before add_adapter by x00267953 end*/

	TS_LOG_DEBUG( "%s: add adap='%s' (CYTTSP5_I2C_NAME=%s)\n", __func__,
		ts_i2c->id, CYTTSP5_I2C_NAME);


	rc = cyttsp5_add_adapter(ts_i2c->id, &ops, dev);
	if (rc) {
		TS_LOG_ERR( "%s: Error on probe %s\n", __func__,
			CYTTSP5_I2C_NAME);
		goto add_adapter_err;
	}
    
	TS_LOG_INFO( "%s probe successful\n", __func__);

	return 0;

add_adapter_err:
	dev_set_drvdata(&client->dev, NULL);
	i2c_set_clientdata(client, NULL);
error_pinctrl_select_state:
error_pinctrl_lookup_state:
error_pinctrl_get:
	gpio_free(vbus_gpio_1);
	gpio_free(vbus_gpio);
error_free_data:
	kfree(ts_i2c);
error_alloc_data_failed:
	return rc;
}

/* registered in driver struct */
static int cyttsp5_i2c_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cyttsp5_i2c *ts_i2c = dev_get_drvdata(dev);

	cyttsp5_del_adapter(ts_i2c->id);
	dev_set_drvdata(&client->dev, NULL);
	i2c_set_clientdata(client, NULL);
	kfree(ts_i2c);
	return 0;
}

static const struct i2c_device_id cyttsp5_i2c_id[] = {
	{ CYTTSP5_I2C_NAME, 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cyttsp5_i2c_id);

static struct i2c_driver cyttsp5_i2c_driver = {
	.driver = {
		.name = CYTTSP5_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cyttsp5_i2c_of_match),
	},
	.probe = cyttsp5_i2c_probe,
	.remove = cyttsp5_i2c_remove,
	.id_table = cyttsp5_i2c_id,
};

static int __init cyttsp5_i2c_init(void)
{
	int rc = 0;
/*
	if(get_boot_into_recovery_flag())
		return 0;
*/
	rc = i2c_add_driver(&cyttsp5_i2c_driver);

	pr_info("%s: Cypress TTSP I2C Touchscreen Driver (Built %s) rc=%d\n",
		 __func__, CY_DRIVER_DATE, rc);
	return rc;
}
module_init(cyttsp5_i2c_init);

static void __exit cyttsp5_i2c_exit(void)
{
	i2c_del_driver(&cyttsp5_i2c_driver);
}
module_exit(cyttsp5_i2c_exit);

MODULE_ALIAS(CYTTSP5_I2C_NAME);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) I2C driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");
