/*
 * cyttsp5_btn.c
 * Cypress TrueTouch(TM) Standard Product V5 CapSense touch reports module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2012-2013 Cypress Semiconductor
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
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



#include <linux/delay.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "cyttsp5_bus.h"
#include "cyttsp5_btn.h"
#include "cyttsp5_core.h"
#include "cyttsp5_regs.h"

struct cyttsp5_btn_data {
	struct cyttsp5_device *ttsp;
	struct cyttsp5_btn_platform_data *pdata;
	struct cyttsp5_sysinfo *si;
	struct input_dev *input;
	struct mutex btn_lock;
#if defined(CONFIG_FB)
	struct notifier_block fb_notify;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend es;
#endif
	bool is_suspended;
	bool input_device_registered;
	char phys[NAME_MAX];
	u8 pr_buf[CY_MAX_PRBUF_SIZE];
};

static void cyttsp5_btn_key_action(struct cyttsp5_btn_data *bd,
	int cur_btn, u8 cur_btn_mask, int num_btns, int new_btn_state)
{
	struct device *dev = &bd->ttsp->dev;
	struct cyttsp5_sysinfo *si = bd->si;
	int i;
	int btn;
	int cur_btn_state;
	int state_from_ic;

	cur_btn_state = new_btn_state == CY_BTN_PRESSED ? CY_BTN_RELEASED :
		CY_BTN_PRESSED;

	for (i = 0; i < num_btns; i++) {
		btn = cur_btn + i;
		if (!si->btn[btn].enabled)
			continue;
		state_from_ic = (cur_btn_mask >> (i * CY_BITS_PER_BTN))
				& CY_NUM_BTN_EVENT_ID;
		if (state_from_ic == new_btn_state &&
				si->btn[btn].state == cur_btn_state) {
			input_report_key(bd->input, si->btn[btn].key_code,
					new_btn_state);
			si->btn[btn].state = new_btn_state;
			input_sync(bd->input);
			TS_LOG_DEBUG("%s: btn=%d key_code=%d %s\n", __func__,
				btn, si->btn[btn].key_code,
				new_btn_state == CY_BTN_PRESSED ?
				"PRESSED" : "RELEASED");
		}
	}
	return;
}

static void cyttsp5_get_btn_touches(struct cyttsp5_btn_data *bd)
{
	enum cyttsp5_btn_state btn_state = CY_BTN_RELEASED;
	struct cyttsp5_sysinfo *si = bd->si;
	int num_cur_btn;
	int cur_btn;
	u8 cur_btn_mask;

	for (btn_state = CY_BTN_RELEASED; btn_state < CY_BTN_NUM_STATE;
		btn_state++) {
		num_cur_btn = si->num_btns;
		cur_btn = 0;
		if (num_cur_btn > 0) {
			cur_btn_mask = si->xy_data[0];
			cyttsp5_btn_key_action(bd, cur_btn,
				cur_btn_mask, num_cur_btn, btn_state);
		}
	}
	return;
}

static void cyttsp5_btn_lift_all(struct cyttsp5_btn_data *bd)
{
	struct cyttsp5_sysinfo *si = bd->si;
	if (si->num_btns == 0)
		return;

	si->xy_data[0] = 0;

	cyttsp5_get_btn_touches(bd);
}

#ifdef VERBOSE_DEBUG
static void cyttsp5_log_btn_data(struct cyttsp5_btn_data *bd)
{
	struct device *dev = &bd->ttsp->dev;
	struct cyttsp5_sysinfo *si = bd->si;
	int cur;
	int value;

	for (cur = 0; cur < si->num_btns; cur++) {
		bd->pr_buf[0] = 0;
		if (si->xy_data[0] & (1 << cur))
			value = 1;
		else
			value = 0;
		snprintf(bd->pr_buf, CY_MAX_PRBUF_SIZE, "btn_rec[%d]=0x", cur);
		snprintf(bd->pr_buf, CY_MAX_PRBUF_SIZE, "%s%X (%02X)",
			bd->pr_buf, value,
			le16_to_cpu(si->xy_data[1 + cur * 2]));

		TS_LOG_DEBUG("%s: %s\n", __func__, bd->pr_buf);
	}
	return;
}
#endif

/* read xy_data for all current CapSense button touches */
static int cyttsp5_xy_worker(struct cyttsp5_btn_data *bd)
{
	struct cyttsp5_sysinfo *si = bd->si;

	/* extract button press/release touch information */
	if (si->num_btns > 0) {
		cyttsp5_get_btn_touches(bd);
#ifdef VERBOSE_DEBUG
		/* log button press/release touch information */
		cyttsp5_log_btn_data(bd);
#endif
	}

	return 0;
}

static int cyttsp5_btn_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_btn_data *bd = dev_get_drvdata(dev);
	int rc = 0;

	if (bd->si->xy_mode[2] !=  bd->si->desc.btn_report_id)
		return 0;

	/* core handles handshake */
	mutex_lock(&bd->btn_lock);
	rc = cyttsp5_xy_worker(bd);
	mutex_unlock(&bd->btn_lock);
	if (rc < 0)
		TS_LOG_ERR("%s: xy_worker error r=%d\n", __func__, rc);

	return rc;
}

static int cyttsp5_startup_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_btn_data *bd = dev_get_drvdata(dev);
	struct cyttsp5_sysinfo *si = bd->si;
	int btn;

	mutex_lock(&bd->btn_lock);
	if (bd->si)
		cyttsp5_btn_lift_all(bd);

	for (btn = 0; btn < si->num_btns; btn++)
		bd->si->btn[btn].state = CY_BTN_RELEASED;
	mutex_unlock(&bd->btn_lock);

	return 0;
}

static int cyttsp5_btn_open(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct cyttsp5_device *ttsp =
		container_of(dev, struct cyttsp5_device, dev);


	TS_LOG_DEBUG("%s: setup subscriptions\n", __func__);

	/* set up touch call back */
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp5_btn_attention, CY_MODE_OPERATIONAL);

	/* set up startup call back */
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_startup_attention, 0);

	return 0;
}

static void cyttsp5_btn_close(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct cyttsp5_device *ttsp =
		container_of(dev, struct cyttsp5_device, dev);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp5_btn_attention, CY_MODE_OPERATIONAL);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_startup_attention, 0);

}

#if defined(CONFIG_FB)
static int fb_notifier_btn_callback(struct notifier_block *self, 
		unsigned long event, void *data)
{
	int i;
	struct fb_event *fb_event = data;
	int *blank = fb_event->data;
	struct cyttsp5_btn_data *bd =
		container_of(self, struct cyttsp5_btn_data, fb_notify);

	for (i = 0 ; i < FB_MAX; i++) {
		if (registered_fb[i] == fb_event->info) {
			if (i == 0) {
				TS_LOG_INFO("%s: cyttsp5_btn:%d, go on !\n", __func__,i);
				break;
			} else {
				TS_LOG_INFO("%s: cyttsp5_btn:%d, exit !\n", __func__,i);
				return -1;
			}
		}
	}

	switch(*blank){
	case FB_BLANK_UNBLANK:
		/*resume device*/
		switch(event) {
		case FB_EARLY_EVENT_BLANK:
			bd->is_suspended = false;
			break;
		case FB_EVENT_BLANK:
			break;
		default:
			break;
		}
		break;
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
	case FB_BLANK_POWERDOWN:
	default:
		/*suspend device*/
		switch(event) {
		case FB_EARLY_EVENT_BLANK:
			bd->is_suspended = true;
			break;
		case FB_EVENT_BLANK:
			break;
		default:
			break;
		}
		break;
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void cyttsp5_btn_early_suspend(struct early_suspend *h)
{
	struct cyttsp5_btn_data *bd =
		container_of(h, struct cyttsp5_btn_data, es);
	struct device *dev = &bd->ttsp->dev;


	bd->is_suspended = true;
}

static void cyttsp5_btn_late_resume(struct early_suspend *h)
{
	struct cyttsp5_btn_data *bd =
		container_of(h, struct cyttsp5_btn_data, es);
	struct device *dev = &bd->ttsp->dev;


	bd->is_suspended = false;
}
#endif

#if defined(CONFIG_PM_RUNTIME)
static int cyttsp5_btn_rt_suspend(struct device *dev)
{
	struct cyttsp5_btn_data *bd = dev_get_drvdata(dev);

	mutex_lock(&bd->btn_lock);
	if (bd->si)
		cyttsp5_btn_lift_all(bd);
	mutex_unlock(&bd->btn_lock);

	return 0;
}

static int cyttsp5_btn_rt_resume(struct device *dev)
{
	return 0;
}
#endif

#if defined(CONFIG_PM_SLEEP)
static int cyttsp5_btn_suspend(struct device *dev)
{
	struct cyttsp5_btn_data *bd = dev_get_drvdata(dev);

	mutex_lock(&bd->btn_lock);
	if (bd->si)
		cyttsp5_btn_lift_all(bd);
	mutex_unlock(&bd->btn_lock);

	return 0;
}

static int cyttsp5_btn_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops cyttsp5_btn_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cyttsp5_btn_suspend, cyttsp5_btn_resume)
	SET_RUNTIME_PM_OPS(cyttsp5_btn_rt_suspend, cyttsp5_btn_rt_resume, NULL)
};

static int cyttsp5_setup_input_device(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_btn_data *bd = dev_get_drvdata(dev);
	int i;
	int rc;

	TS_LOG_DEBUG( "%s: Initialize event signals\n", __func__);
	__set_bit(EV_KEY, bd->input->evbit);
	TS_LOG_DEBUG( "%s: Number of buttons %d\n", __func__, bd->si->num_btns);
	for (i = 0; i < bd->si->num_btns; i++) {
		TS_LOG_DEBUG( "%s: btn:%d keycode:%d\n",
			__func__, i, bd->si->btn[i].key_code);
		__set_bit(bd->si->btn[i].key_code, bd->input->keybit);
	}

	rc = input_register_device(bd->input);
	if (rc < 0)
		TS_LOG_ERR( "%s: Error, failed register input device r=%d\n",
			__func__, rc);
	else
		bd->input_device_registered = true;

	return rc;
}

static int cyttsp5_setup_input_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_btn_data *bd = dev_get_drvdata(dev);
	int rc;

	bd->si = cyttsp5_request_sysinfo(ttsp);
	if (!bd->si)
		return -1;

	rc = cyttsp5_setup_input_device(ttsp);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_setup_input_attention, 0);

	return rc;
}

static int cyttsp5_btn_probe(struct cyttsp5_device *ttsp)
{
	struct cyttsp5_btn_data *bd;
	struct device *dev = &ttsp->dev;
	struct cyttsp5_btn_platform_data *pdata = dev_get_platdata(dev);
	int rc = 0;

	if (pdata == NULL) {
		TS_LOG_ERR( "%s: Missing platform data\n", __func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}

	bd = kzalloc(sizeof(*bd), GFP_KERNEL);
	if (bd == NULL) {
		TS_LOG_ERR( "%s: Error, kzalloc\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}

	mutex_init(&bd->btn_lock);
	bd->ttsp = ttsp;
	bd->pdata = pdata;
	dev_set_drvdata(dev, bd);
	/* Create the input device and register it. */
	TS_LOG_DEBUG( "%s: Create the input device and register it\n",
		__func__);
	bd->input = input_allocate_device();
	if (bd->input == NULL) {
		TS_LOG_ERR( "%s: Error, failed to allocate input device\n",
			__func__);
		rc = -ENOSYS;
		goto error_alloc_failed;
	}

	if (pdata->inp_dev_name)
		bd->input->name = pdata->inp_dev_name;
	else
		bd->input->name = ttsp->name;
	scnprintf(bd->phys, sizeof(bd->phys)-1, "%s", dev_name(dev));
	bd->input->phys = bd->phys;
	bd->input->dev.parent = &bd->ttsp->dev;
	bd->input->open = cyttsp5_btn_open;
	bd->input->close = cyttsp5_btn_close;
	input_set_drvdata(bd->input, bd);


	/* get sysinfo */
	bd->si = cyttsp5_request_sysinfo(ttsp);

	if (bd->si) {
		rc = cyttsp5_setup_input_device(ttsp);
		if (rc)
			goto error_init_input;
	} else {
		TS_LOG_ERR( "%s: Fail get sysinfo pointer from core p=%p\n",
			__func__, bd->si);
		cyttsp5_subscribe_attention(ttsp, CY_ATTEN_STARTUP,
			cyttsp5_setup_input_attention, 0);
	}

#if defined (CONFIG_FB)
	bd->fb_notify.notifier_call = fb_notifier_btn_callback;
	fb_register_client(&bd->fb_notify);
#ifdef CONFIG_HAS_EARLYSUSPEND
	bd->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	bd->es.suspend = cyttsp5_btn_early_suspend;
	bd->es.resume = cyttsp5_btn_late_resume;
	register_early_suspend(&bd->es);
#endif

	return 0;

error_init_input:
	input_free_device(bd->input);
error_alloc_failed:
	dev_set_drvdata(dev, NULL);
	kfree(bd);
error_alloc_data_failed:
error_no_pdata:
	TS_LOG_ERR( "%s failed.\n", __func__);
	return rc;
}

static int cyttsp5_btn_release(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_btn_data *bd = dev_get_drvdata(dev);

#if defined(CONFIG_FB)
	fb_unregister_client(&bd->fb_notify);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&bd->es);
#endif

	if (bd->input_device_registered) {
		input_unregister_device(bd->input);
	} else {
		input_free_device(bd->input);
		cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
			cyttsp5_setup_input_attention, 0);
	}

	dev_set_drvdata(dev, NULL);
	kfree(bd);
	return 0;
}

static struct cyttsp5_driver cyttsp5_btn_driver = {
	.probe = cyttsp5_btn_probe,
	.remove = cyttsp5_btn_release,
	.driver = {
		.name = CYTTSP5_BTN_NAME,
		.bus = &cyttsp5_bus_type,
		.owner = THIS_MODULE,
		.pm = &cyttsp5_btn_pm_ops,
	},
};

static int __init cyttsp5_btn_init(void)
{
	int rc = 0;
	rc = cyttsp5_register_driver(&cyttsp5_btn_driver);
	pr_info("%s: Cypress TTSP MT v5 CapSense BTN (Built %s), rc=%d\n",
		 __func__, CY_DRIVER_DATE, rc);
	return rc;
}
module_init(cyttsp5_btn_init);

static void __exit cyttsp5_btn_exit(void)
{
	cyttsp5_unregister_driver(&cyttsp5_btn_driver);
}
module_exit(cyttsp5_btn_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TTSP 2D multi-touch CapSense BTN driver");
MODULE_AUTHOR("Cypress Semiconductor");
