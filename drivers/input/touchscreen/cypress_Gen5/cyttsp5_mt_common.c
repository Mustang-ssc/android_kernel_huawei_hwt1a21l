/*
 * cyttsp5_mt_common.c
 * Cypress TrueTouch(TM) Standard Product V5 Multi-touch module.
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
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/atomic.h>
#include "cyttsp5_core.h"
#include "cyttsp5_device_access.h"
#include "cyttsp5_bus.h"
#include "cyttsp5_mt_common.h"


#define CY_MAX_BUF_SIZE 128


static const char* cover_command[] = {
	"0x04 0x00 0x08 0x00 0x2f 0x00 0x06 0x5a 0x01 0x01", //cover on
	"0x04 0x00 0x08 0x00 0x2f 0x00 0x06 0x5a 0x01 0x02", //cover off
	"0x04 0x00 0x06 0x00 0x2f 0x00 0x05 0x5a"            //read cover mode
};

static const char* cover_response_state[] = {
	"07 00 1F 00 06 5A 01",      //set cover on/off success
	"07 00 1F 00 86 5A 01",      //set cover on/off success
	"08 00 1F 00 05 5A 01 01",   //cover mode on
	"08 00 1F 00 85 5A 01 01",   //cover mode on
	"08 00 1F 00 05 5A 01 02",    //cover mode off
	"08 00 1F 00 85 5A 01 02"    //cover mode off
};

struct cyttsp5_cover_mode_flags_ {
	int set_cover;
};

struct cyttsp5_mt_data *mt_data;
static struct timer_list cover_mode_timer;
static struct work_struct cover_mode_work;


static atomic_t touch_mode_f = ATOMIC_INIT(0);
static atomic_t work_is_busy = ATOMIC_INIT(0);

static struct cyttsp5_cover_mode_flags_ cyttsp5_cover_mode_flags = {
    .set_cover = 3,
};

#define COVER_RESPONSE_MAX_LEN	30
int cyttsp5_get_cover_mode(void)
{
	int rc;
	int i = 0;
	int index = 0;
	int num_read = 0;
	u8 response_buf[CY_MAX_BUF_SIZE] = {0};
	char response_str[COVER_RESPONSE_MAX_LEN] = {0};

	printk(KERN_INFO "Enter %s.\n", __func__);

	memset(response_buf, 0, sizeof(response_buf));

	rc = cyttsp5_send_command(cover_command[2]);
	if (rc || !cyttsp5_command_status()) {
		printk(KERN_ERR "%s: Send read TP cover mode command failed!\n", __func__);
		return -1;
	}

	msleep(10);

	num_read = cyttsp5_command_response(response_buf);
	if (!cyttsp5_command_status() || num_read < 0) {
		printk(KERN_ERR "%s: Get read TP cover mode command response failed!\n", __func__);
		return -1;
	}

	for (i = 0; i < num_read; i++) {
		index += scnprintf(response_str + index, (size_t)(COVER_RESPONSE_MAX_LEN - index),
			"%02X ", response_buf[i]);
	}

	if ((strncmp(cover_response_state[2], response_str, strlen(cover_response_state[2])) == 0)
	|| (strncmp(cover_response_state[3], response_str, strlen(cover_response_state[3])) == 0)) {
		printk(KERN_INFO "Exit %s.\n", __func__);
		return 1;
	} else if ((strncmp(cover_response_state[4], response_str, strlen(cover_response_state[4])))
	|| (strncmp(cover_response_state[5], response_str, strlen(cover_response_state[5])))) {
		printk(KERN_INFO "Exit %s.\n", __func__);
		return 0;
	}

	return -1;
}

static int cyttsp5_set_cover_mode(int on)
{
	u8 response_buf[CY_MAX_BUF_SIZE] = {0};
	char response_str[COVER_RESPONSE_MAX_LEN] = {0};
	u32 touch_mode;
	int i =0;
	int num_read = 0;
	int index = 0;
	int rc;
	int flag;
	int retry_times = 5;

	TS_LOG_DEBUG( "Enter %s.\n", __func__);

	memset(response_buf, 0, sizeof(response_buf));

	flag = cyttsp5_get_cover_mode();
	if ((on == 1 && flag == 1) || (on == 0 && flag == 0)) {
		TS_LOG_DEBUG( "TP already in %s mode!\n", (flag == 1) ? "cover ON" : "cover OFF");
		return 0;
	} else if (flag < 0) {
		TS_LOG_ERR( "get TP cover mode ERROR!\n");
		return -1;
	}

retry:
	if (retry_times--) {
		if (on == 1) {
			TS_LOG_DEBUG( "%s:Set cover mode ON retry_times.....\n", __func__);
			touch_mode = cyttsp5_get_touch_mode();
			if (touch_mode < 0) {
				TS_LOG_ERR( "%s: Failed to get touch mode.\n", __func__);
				goto retry;
			}

			if (HOVER_MODE == touch_mode) {
				TS_LOG_INFO( "%s: Change TP mode to normal mode\n", __func__);
				rc = cyttsp5_set_touch_mode(NORMAL_MODE);
				if (rc < 0) {
					TS_LOG_ERR( "%s: Error set or get touch mode.\n", __func__);
					goto retry;
				}
				atomic_set(&touch_mode_f, 1);
			}

			rc = cyttsp5_send_command(cover_command[0]);
			if (rc || !cyttsp5_command_status()) {
				TS_LOG_ERR(  "%s: Send read TP cover mode command failed!\n", __func__);
				goto retry;
			}
            

		} else {
			TS_LOG_DEBUG( "%s:Set cover mode OFF retry_times.....\n", __func__);
			if (atomic_read(&touch_mode_f)) {
				atomic_set(&touch_mode_f, 0);

				TS_LOG_DEBUG( "%s: Change TP mode to hover mode\n", __func__);

				rc  = cyttsp5_set_touch_mode(HOVER_MODE);
				if (rc  < 0) {
					TS_LOG_ERR( "%s: Error set or get touch mode.\n", __func__);
					goto retry;
				}
			}

			rc = cyttsp5_send_command(cover_command[1]);
			if (rc || !cyttsp5_command_status()) {
				TS_LOG_ERR(  "%s: Send read TP cover mode command failed!\n", __func__);
				goto retry;
			}

		}
	} else {
		TS_LOG_ERR( "%s Failed to set cover mode @ line = %d\n",__func__,__LINE__);

		if (atomic_read(&touch_mode_f)) {
			atomic_set(&touch_mode_f, 0);

			TS_LOG_DEBUG( "%s: Change TP mode to hover mode\n", __func__);

			rc  = cyttsp5_set_touch_mode(HOVER_MODE);
			if (rc  < 0) {
				TS_LOG_ERR( "%s: Error set or get touch mode.\n", __func__);
				return -1;
			}
		}
	}

	num_read = cyttsp5_command_response(response_buf);
	if (!cyttsp5_command_status() || num_read < 0) {
		TS_LOG_ERR( "%s: Get 'set TP cover mode' command response failed!\n", __func__);
		return -1;
	}
	
	for (i = 0; i < num_read; i++) {
		index += scnprintf(response_str + index, (size_t)(COVER_RESPONSE_MAX_LEN - index),
			"%02X ", response_buf[i]);
	}

	if ((strncmp(cover_response_state[0], response_str, strlen(cover_response_state[0])) == 0)
	|| (strncmp(cover_response_state[1], response_str, strlen(cover_response_state[1])) == 0)){
		TS_LOG_INFO( "Set TP cover mode successful!\n");
	} else {
		TS_LOG_ERR( "%s Failed to set TP cover mode!\n",__func__);
		return -1;
	}

	TS_LOG_INFO( "Exit %s.\n", __func__);

	return 0;
}

static void cyttsp5_cover_mode_work_function(struct work_struct *work)
{
	int rc = -1;
	int flag_mode = -1;

	TS_LOG_DEBUG( "Enter %s.\n", __func__);

	atomic_set(&work_is_busy, 1);

	mutex_lock(&mt_data->mt_lock);
	flag_mode = mt_data->cover_info.cover_mode;
	mutex_unlock(&mt_data->mt_lock);

	rc = cyttsp5_set_cover_mode(flag_mode);
	if (rc < 0) {
		TS_LOG_INFO( "%s:set tp cover mode %s failed!\n",
			__func__, (flag_mode == 0 ? "OFF" : "ON"));
	}

	if(cyttsp5_cover_mode_flags.set_cover > 0){
	    cyttsp5_cover_mode_flags.set_cover -= 1;
	    mod_timer(&cover_mode_timer, jiffies +
	                msecs_to_jiffies(CY_COVER_MODE_TIMEOUT));
	}

	atomic_set(&work_is_busy, 0);

	TS_LOG_DEBUG( "Exit %s.\n", __func__);
}


static void cyttsp5_cover_mode_timer(unsigned long handle)
{
	TS_LOG_DEBUG( "Enter %s.\n", __func__);

	if(atomic_read(&work_is_busy)){
		TS_LOG_DEBUG( "%s: work_is_busy = %d.\n", __func__,
			atomic_read(&work_is_busy));
		mod_timer(&cover_mode_timer, jiffies +
			msecs_to_jiffies(CY_COVER_MODE_TIMEOUT));

	}else{
		TS_LOG_DEBUG( "%s: work_is_busy = %d.\n", __func__,
			atomic_read(&work_is_busy));
		schedule_work(&cover_mode_work);
	}

	TS_LOG_DEBUG( "Exit %s.\n", __func__);
}

void cyttsp5_cover_mode_settings(int cover_mode)
{

	if (!mt_data) {
		pr_info("Enter %s and TP not ready.\n", __func__);
		return;
	}

    TS_LOG_DEBUG( "Enter %s.\n", __func__);

    if(mt_data->cover_info.enable == 0){
        TS_LOG_DEBUG( " %s: Cover mode is disable.\n", __func__);
        return;
    }

	mutex_lock(&mt_data->mt_lock);

	mt_data->cover_info.cover_mode = cover_mode;
	
	if (mt_data->is_suspended == false) {
		TS_LOG_INFO("%s,mt_data->is_suspended = %d,\n",__func__,mt_data->is_suspended);
		mod_timer(&cover_mode_timer, jiffies +
		        msecs_to_jiffies(CY_COVER_MODE_TIMEOUT));
	}

	cyttsp5_cover_mode_flags.set_cover = 3;
    
	TS_LOG_DEBUG("%s,set_cover_f = %d,\n",__func__, cyttsp5_cover_mode_flags.set_cover);

	TS_LOG_DEBUG( "Exit %s.\n", __func__);

	mutex_unlock(&mt_data->mt_lock);
}



static int cyttsp5_in_available_area(struct cyttsp5_mt_data *md, int x_axis, int y_axis)
{
	TS_LOG_DEBUG( "%s:cover value is %d, (%d,%d), (%d,%d)\n",
		__func__, md->cover_info.cover_mode, md->cover_info.x0, md->cover_info.y0,
		md->cover_info.x1, md->cover_info.y1);

	if  ((x_axis > md->cover_info.x0 && x_axis < md->cover_info.x1)
		&& (y_axis > md->cover_info.y0 && y_axis < md->cover_info.y1)) {
		return 1;
	}

	return 0;
}
static void cyttsp5_mt_lift_all(struct cyttsp5_mt_data *md)
{
	int max = md->si->tch_abs[CY_TCH_T].max;
	/* Ouput the max value */
	TS_LOG_DEBUG( " %s,max = %d\n", __func__, max);

	if (md->num_prv_tch != 0) {
		if (md->mt_function.report_slot_liftoff)
			md->mt_function.report_slot_liftoff(md, max);
		input_sync(md->input);
		md->num_prv_tch = 0;
	}
}

static void cyttsp5_get_touch_axis(struct cyttsp5_mt_data *md,
	int *axis, int size, int max, u8 *xy_data, int bofs)
{
	int nbyte;
	int next;

	for (nbyte = 0, *axis = 0, next = 0; nbyte < size; nbyte++) {
		TS_LOG_DEBUG(
			"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p"
			" xy_data[%d]=%02X(%d) bofs=%d\n",
			__func__, *axis, *axis, size, max, xy_data, next,
			xy_data[next], xy_data[next], bofs);
		*axis = *axis + ((xy_data[next] >> bofs) << (nbyte * 8));
		next++;
	}

	*axis &= max - 1;

	TS_LOG_DEBUG(
		"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p"
		" xy_data[%d]=%02X(%d)\n",
		__func__, *axis, *axis, size, max, xy_data, next,
		xy_data[next], xy_data[next]);
}

static void cyttsp5_get_touch_hdr(struct cyttsp5_mt_data *md,
	struct cyttsp5_touch *touch, u8 *xy_mode)
{
	struct cyttsp5_sysinfo *si = md->si;
	enum cyttsp5_tch_hdr hdr;

	for (hdr = CY_TCH_TIME; hdr < CY_TCH_NUM_HDR; hdr++) {
		if (!si->tch_hdr[hdr].report)
			continue;
		cyttsp5_get_touch_axis(md, &touch->hdr[hdr],
			si->tch_hdr[hdr].size,
			si->tch_hdr[hdr].max,
			xy_mode + si->tch_hdr[hdr].ofs,
			si->tch_hdr[hdr].bofs);
		TS_LOG_DEBUG( "%s: get %s=%04X(%d)\n", __func__,
			cyttsp5_tch_hdr_string[hdr],
			touch->hdr[hdr], touch->hdr[hdr]);
	}

	TS_LOG_DEBUG(
		"%s: time=%X tch_num=%d lo=%d noise=%d counter=%d\n",
		__func__,
		touch->hdr[CY_TCH_TIME],
		touch->hdr[CY_TCH_NUM],
		touch->hdr[CY_TCH_LO],
		touch->hdr[CY_TCH_NOISE],
		touch->hdr[CY_TCH_COUNTER]);
}

struct cyttsp5_touch print_touch_xydata;

static void cyttsp5_get_touch(struct cyttsp5_mt_data *md,
	struct cyttsp5_touch *touch, u8 *xy_data)
{
	struct cyttsp5_sysinfo *si = md->si;
	enum cyttsp5_tch_abs abs;
	int tmp;
	bool flipped;

	for (abs = CY_TCH_X; abs < CY_TCH_NUM_ABS; abs++) {
		if (!si->tch_abs[abs].report)
			continue;
		cyttsp5_get_touch_axis(md, &touch->abs[abs],
			si->tch_abs[abs].size,
			si->tch_abs[abs].max,
			xy_data + si->tch_abs[abs].ofs,
			si->tch_abs[abs].bofs);
		TS_LOG_DEBUG( "%s: get %s=%04X(%d)\n", __func__,
			cyttsp5_tch_abs_string[abs],
			touch->abs[abs], touch->abs[abs]);
	}

	if (md->pdata->flags & CY_MT_FLAG_FLIP) {
		tmp = touch->abs[CY_TCH_X];
		touch->abs[CY_TCH_X] = touch->abs[CY_TCH_Y];
		touch->abs[CY_TCH_Y] = tmp;
		flipped = true;
	} else
		flipped = false;

	if (md->pdata->flags & CY_MT_FLAG_INV_X) {
		if (flipped)
			touch->abs[CY_TCH_X] = si->sensing_conf_data.res_y -
				touch->abs[CY_TCH_X];
		else
			touch->abs[CY_TCH_X] = si->sensing_conf_data.res_x -
				touch->abs[CY_TCH_X];
	}
	if (md->pdata->flags & CY_MT_FLAG_INV_Y) {
		if (flipped)
			touch->abs[CY_TCH_Y] = si->sensing_conf_data.res_x -
				touch->abs[CY_TCH_Y];
		else
			touch->abs[CY_TCH_Y] = si->sensing_conf_data.res_y -
				touch->abs[CY_TCH_Y];
	}

	TS_LOG_DEBUG( "%s: flip=%s inv-x=%s inv-y=%s x=%04X(%d) y=%04X(%d)\n",
		__func__, flipped ? "true" : "false",
		md->pdata->flags & CY_MT_FLAG_INV_X ? "true" : "false",
		md->pdata->flags & CY_MT_FLAG_INV_Y ? "true" : "false",
		touch->abs[CY_TCH_X], touch->abs[CY_TCH_X],
		touch->abs[CY_TCH_Y], touch->abs[CY_TCH_Y]);
}

static void cyttsp5_get_mt_touches(struct cyttsp5_mt_data *md,
		struct cyttsp5_touch *tch, int num_cur_tch)
{
	struct cyttsp5_sysinfo *si = md->si;
	int sig;
	int i, j, t = 0;
	DECLARE_BITMAP(ids, md->si->tch_abs[CY_TCH_T].max);
	int mt_sync_count = 0;

	bitmap_zero(ids, md->si->tch_abs[CY_TCH_T].max);
	memset(tch->abs, 0, sizeof(tch->abs));

	for (i = 0; i < num_cur_tch; i++) {
		cyttsp5_get_touch(md, tch, si->xy_data +
			(i * si->desc.tch_record_size));


		if (md->cover_info.cover_mode) {
			if(!cyttsp5_in_available_area(md, tch->abs[CY_TCH_X], tch->abs[CY_TCH_Y])) {
				memset(tch->abs, 0, sizeof(tch->abs));
				continue;
			}
		}

		/*  Discard proximity event */
		if (tch->abs[CY_TCH_O] == CY_OBJ_PROXIMITY) {
			TS_LOG_DEBUG( "%s: Discarding proximity event\n",
					__func__);
			continue;
		} else if (tch->abs[CY_TCH_O] == CY_OBJ_HOVER) {
			tch->abs[CY_TCH_P] = 0;
		}

		if ((tch->abs[CY_TCH_T] < md->pdata->frmwrk->abs
			[(CY_ABS_ID_OST * CY_NUM_ABS_SET) + CY_MIN_OST]) ||
			(tch->abs[CY_TCH_T] > md->pdata->frmwrk->abs
			[(CY_ABS_ID_OST * CY_NUM_ABS_SET) + CY_MAX_OST])) {
			TS_LOG_ERR( "%s: tch=%d -> bad trk_id=%d max_id=%d\n",
				__func__, i, tch->abs[CY_TCH_T],
				md->pdata->frmwrk->abs[(CY_ABS_ID_OST *
				CY_NUM_ABS_SET) + CY_MAX_OST]);
			if (md->mt_function.input_sync)
				md->mt_function.input_sync(md->input);
			mt_sync_count++;
			continue;
		}

		/* use 0 based track id's */
		sig = md->pdata->frmwrk->abs
			[(CY_ABS_ID_OST * CY_NUM_ABS_SET) + 0];
		if (sig != CY_IGNORE_VALUE) {
			t = tch->abs[CY_TCH_T] - md->pdata->frmwrk->abs
				[(CY_ABS_ID_OST * CY_NUM_ABS_SET) + CY_MIN_OST];
			if (tch->abs[CY_TCH_E] == CY_EV_LIFTOFF) {
				TS_LOG_DEBUG( "%s: t=%d e=%d lift-off\n",
					__func__, t, tch->abs[CY_TCH_E]);
				goto cyttsp5_get_mt_touches_pr_tch;
			}
			if (md->mt_function.input_report)
				md->mt_function.input_report(md->input, sig,
						t, tch->abs[CY_TCH_O]);
			__set_bit(t, ids);
		}

		/* all devices: position and pressure fields */
		for (j = 0; j <= CY_ABS_W_OST; j++) {
			if (!si->tch_abs[j].report)
				continue;
			sig = md->pdata->frmwrk->abs[((CY_ABS_X_OST + j) *
				CY_NUM_ABS_SET) + 0];
			if (sig != CY_IGNORE_VALUE)
				input_report_abs(md->input, sig,
					tch->abs[CY_TCH_X + j]);
		}

		/* Get the extended touch fields */
		for (j = 0; j < CY_NUM_EXT_TCH_FIELDS; j++) {
			if (!si->tch_abs[j].report)
				continue;
			sig = md->pdata->frmwrk->abs
				[((CY_ABS_MAJ_OST + j) *
				CY_NUM_ABS_SET) + 0];
			if (sig != CY_IGNORE_VALUE)
				input_report_abs(md->input, sig,
					tch->abs[CY_TCH_MAJ + j]);
		}
		if (md->mt_function.input_sync)
			md->mt_function.input_sync(md->input);
		mt_sync_count++;

cyttsp5_get_mt_touches_pr_tch:
		TS_LOG_DEBUG(
			"%s: t=%d x=%d y=%d z=%d M=%d m=%d o=%d e=%d obj=%d tip=%d\n",
			__func__, t,
			tch->abs[CY_TCH_X],
			tch->abs[CY_TCH_Y],
			tch->abs[CY_TCH_P],
			tch->abs[CY_TCH_MAJ],
			tch->abs[CY_TCH_MIN],
			tch->abs[CY_TCH_OR],
			tch->abs[CY_TCH_E],
			tch->abs[CY_TCH_O],
			tch->abs[CY_TCH_TIP]);
	}

	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input, md->si->tch_abs[CY_TCH_T].max,
				mt_sync_count, ids);

	md->num_prv_tch = num_cur_tch;

    memcpy(&print_touch_xydata,tch,sizeof(struct cyttsp5_touch));

	return;
}

/* read xy_data for all current touches */
static int cyttsp5_xy_worker(struct cyttsp5_mt_data *md)
{
	struct cyttsp5_sysinfo *si = md->si;
	struct cyttsp5_touch tch;
	u8 num_cur_tch;

	cyttsp5_get_touch_hdr(md, &tch, si->xy_mode + 3);

	num_cur_tch = tch.hdr[CY_TCH_NUM];
	if (num_cur_tch > MAX_TOUCH_NUMBER) {
		TS_LOG_ERR( "%s: Num touch err detected (n=%d)\n",
			__func__, num_cur_tch);
		num_cur_tch = MAX_TOUCH_NUMBER;
	}

	if (tch.hdr[CY_TCH_LO]) {
		TS_LOG_DEBUG( "%s: Large area detected\n", __func__);
		if (md->pdata->flags & CY_MT_FLAG_NO_TOUCH_ON_LO)
			num_cur_tch = 0;
	}

	/* extract xy_data for all currently reported touches */
	TS_LOG_DEBUG( "%s: extract data num_cur_tch=%d\n", __func__,
		num_cur_tch);
	if (num_cur_tch)
		cyttsp5_get_mt_touches(md, &tch, num_cur_tch);
	else{
		cyttsp5_mt_lift_all(md);
        memset(&print_touch_xydata,0,sizeof(struct cyttsp5_touch));
	}

	return 0;
}

static void cyttsp5_mt_send_dummy_event(struct cyttsp5_mt_data *md)
{

    if (!md && !md->input)
        return;

    /* for easy wakeup */
    TS_LOG_DEBUG( "Enter %s\n", __func__);
    input_report_key(md->input, KEY_WAKEUP, 1);
    input_sync(md->input);
    input_report_key(md->input, KEY_WAKEUP, 0);
    input_sync(md->input);
}

static int cyttsp5_mt_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	int rc;

	if (md->si->xy_mode[2] !=  md->si->desc.tch_report_id)
		return 0;

	/* core handles handshake */
	mutex_lock(&md->mt_lock);
	rc = cyttsp5_xy_worker(md);
	mutex_unlock(&md->mt_lock);
	if (rc < 0)
		TS_LOG_ERR( "%s: xy_worker error r=%d\n", __func__, rc);

	return rc;
}

static int cyttsp5_mt_wake_attention(struct cyttsp5_device *ttsp)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(&ttsp->dev);

	mutex_lock(&md->mt_lock);
	cyttsp5_mt_send_dummy_event(md);
	mutex_unlock(&md->mt_lock);
	return 0;
}

static int cyttsp5_startup_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	int rc = 0;

	mutex_lock(&md->mt_lock);
	cyttsp5_mt_lift_all(md);
	mutex_unlock(&md->mt_lock);

	return rc;
}

static int cyttsp5_mt_open(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct cyttsp5_device *ttsp =
		container_of(dev, struct cyttsp5_device, dev);

	TS_LOG_DEBUG( "%s: setup subscriptions\n", __func__);

	/* set up touch call back */
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp5_mt_attention, CY_MODE_OPERATIONAL);

	/* set up startup call back */
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_startup_attention, 0);

	/* set up wakeup call back */
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_WAKE,
		cyttsp5_mt_wake_attention, 0);

	return 0;
}

static void cyttsp5_mt_close(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct cyttsp5_device *ttsp =
		container_of(dev, struct cyttsp5_device, dev);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp5_mt_attention, CY_MODE_OPERATIONAL);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_startup_attention, 0);
}

static void cyttsp5_mt_early_suspend_(struct cyttsp5_mt_data *md)
{
	//struct device *dev = &md->ttsp->dev;
	TS_LOG_DEBUG( "Enter %s.\n", __func__);

#ifdef CY_TOUCH_RUNTIME_NOTUSED
	cyttsp5_core_early_suspend();
#endif
	mutex_lock(&md->mt_lock);
	md->is_suspended = true;
	mutex_unlock(&md->mt_lock);

	/*
	 * Ensure we wait until the cover_mode_timer
	 * running on a different CPU finishes
	 */
	/*
	del_timer_sync(&cover_mode_timer);
	cancel_work_sync(&cover_mode_work);
	del_timer_sync(&cover_mode_timer);
	*/
	msleep(50);
	TS_LOG_DEBUG( "Exit %s.\n", __func__);
}

static void cyttsp5_mt_late_resume_(struct cyttsp5_mt_data *md)
{
	//struct device *dev = &md->ttsp->dev;

	TS_LOG_DEBUG( "Enter %s.\n", __func__);

#ifdef CY_TOUCH_RUNTIME_NOTUSED
	cyttsp5_core_late_rusume();
#endif

	mutex_lock(&md->mt_lock);
	if (md->si)
		cyttsp5_mt_lift_all(md);

	md->is_suspended = false;
	/*
	if (cyttsp5_cover_mode_flags.set_cover > 0) {
        cyttsp5_cover_mode_flags.set_cover -= 1;
		TS_LOG_DEBUG( "%s:Enter later resume to set/check cover mode.\n", __func__);
		mod_timer(&cover_mode_timer, jiffies +
		        msecs_to_jiffies(CY_COVER_MODE_TIMEOUT));
	}
	*/
	mutex_unlock(&md->mt_lock);

	TS_LOG_DEBUG( "Exit %s.\n", __func__);
}

#if defined(CONFIG_FB)
static int fb_notifier_mt_callback(struct notifier_block *self, 
		unsigned long event, void *data)
{
	struct fb_event *fb_event = data;
	int *blank = fb_event->data;
	struct cyttsp5_mt_data *md = container_of(self, struct cyttsp5_mt_data, fb_notify);
	
	if (!data) {
		TS_LOG_ERR("data is NULL %s.\n", __func__);
		return 0;
	}
	blank = fb_event->data;
	switch(event) {
	case FB_EARLY_EVENT_BLANK:
		switch(*blank){
		case FB_BLANK_UNBLANK:/*resume device*/
			if(md->is_suspended) {
				cyttsp5_mt_late_resume_(md);
			}
			break;
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
		case FB_BLANK_POWERDOWN:
		default:/*suspend device*/ 
			if(!md->is_suspended) {
				cyttsp5_mt_early_suspend_(md);
			}
			break;
		}
	case FB_EVENT_BLANK:
	default:
		break;
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void cyttsp5_mt_early_suspend(struct early_suspend *h)
{
	struct cyttsp5_mt_data *md =
		container_of(h, struct cyttsp5_mt_data, es);
	
	cyttsp5_mt_early_suspend_(md);
}

static void cyttsp5_mt_late_resume(struct early_suspend *h)
{
	struct cyttsp5_mt_data *md =
		container_of(h, struct cyttsp5_mt_data, es);
	cyttsp5_mt_late_resume_(md);
}

static void cyttsp5_setup_early_suspend(struct cyttsp5_mt_data *md)
{
	md->es.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	md->es.suspend = cyttsp5_mt_early_suspend;
	md->es.resume = cyttsp5_mt_late_resume;

	register_early_suspend(&md->es);
}
#endif

#ifndef CY_TOUCH_RUNTIME_NOTUSED
#if defined(CONFIG_PM_RUNTIME)
static int cyttsp5_mt_rt_suspend(struct device *dev)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);

	mutex_lock(&md->mt_lock);
	if (md->si)
		cyttsp5_mt_lift_all(md);
	mutex_unlock(&md->mt_lock);

	return 0;
}

static int cyttsp5_mt_rt_resume(struct device *dev)
{
	return 0;
}
#endif

#if defined(CONFIG_PM_SLEEP)
static int cyttsp5_mt_suspend(struct device *dev)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);

	mutex_lock(&md->mt_lock);
	if (md->si)
		cyttsp5_mt_lift_all(md);
	mutex_unlock(&md->mt_lock);

	return 0;
}

static int cyttsp5_mt_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops cyttsp5_mt_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cyttsp5_mt_suspend, cyttsp5_mt_resume)
	SET_RUNTIME_PM_OPS(cyttsp5_mt_rt_suspend, cyttsp5_mt_rt_resume, NULL)
};
#endif

static int cyttsp5_setup_input_device(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	int signal = CY_IGNORE_VALUE;
	int max_x, max_y, max_p, min, max;
	int max_x_tmp, max_y_tmp;
	int i;
	int rc;

	TS_LOG_DEBUG( "%s: Initialize event signals\n", __func__);
	__set_bit(EV_ABS, md->input->evbit);
	__set_bit(EV_REL, md->input->evbit);
	__set_bit(EV_KEY, md->input->evbit);
#ifdef INPUT_PROP_DIRECT
	__set_bit(INPUT_PROP_DIRECT, md->input->propbit);
#endif
    __set_bit(KEY_WAKEUP, md->input->keybit);


	/* If virtualkeys enabled, don't use all screen */
	if (md->pdata->flags & CY_MT_FLAG_VKEYS) {
		max_x_tmp = md->pdata->vkeys_x;
		max_y_tmp = md->pdata->vkeys_y;
	} else {
		max_x_tmp = md->si->sensing_conf_data.res_x;
		max_y_tmp = md->si->sensing_conf_data.res_y;
	}

	/* get maximum values from the sysinfo data */
	if (md->pdata->flags & CY_MT_FLAG_FLIP) {
		max_x = max_y_tmp - 1;
		max_y = max_x_tmp - 1;
	} else {
		max_x = max_x_tmp - 1;
		max_y = max_y_tmp - 1;
	}
	max_p = md->si->sensing_conf_data.max_z;

	/* set event signal capabilities */
	for (i = 0; i < (md->pdata->frmwrk->size / CY_NUM_ABS_SET); i++) {
		signal = md->pdata->frmwrk->abs
			[(i * CY_NUM_ABS_SET) + CY_SIGNAL_OST];
		if (signal != CY_IGNORE_VALUE) {
			__set_bit(signal, md->input->absbit);
			min = md->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_MIN_OST];
			max = md->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_MAX_OST];
			if (i == CY_ABS_ID_OST) {
				/* shift track ids down to start at 0 */
				max = max - min;
				min = min - min;
			} else if (i == CY_ABS_X_OST)
				max = max_x;
			else if (i == CY_ABS_Y_OST)
				max = max_y;
			else if (i == CY_ABS_P_OST)
				max = max_p;
			input_set_abs_params(md->input, signal, min, max,
				md->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_FUZZ_OST],
				md->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_FLAT_OST]);
			TS_LOG_DEBUG( "%s: register signal=%02X min=%d max=%d\n",
				__func__, signal, min, max);
		}
	}

	rc = md->mt_function.input_register_device(md->input,
			md->si->tch_abs[CY_TCH_T].max);
	if (rc < 0)
		TS_LOG_ERR( "%s: Error, failed register input device r=%d\n",
			__func__, rc);
	else
		md->input_device_registered = true;

	return rc;
}

static int cyttsp5_setup_input_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	int rc;

	md->si = cyttsp5_request_sysinfo(ttsp);
	if (!md->si)
		return -EINVAL;

	rc = cyttsp5_setup_input_device(ttsp);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_setup_input_attention, 0);

	return rc;
}

static int cyttsp5_mt_release(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);

#if defined(CONFIG_FB)
	fb_unregister_client(&md->fb_notify);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&md->es);
#endif

	if (md->input_device_registered) {
		input_unregister_device(md->input);
	} else {
		input_free_device(md->input);
		cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
			cyttsp5_setup_input_attention, 0);
	}

	dev_set_drvdata(dev, NULL);
	kfree(md);
	return 0;
}

static int cyttsp5_mt_probe(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md;
	struct cyttsp5_mt_platform_data *pdata = dev_get_platdata(dev);
	int rc = 0;
	
	if (pdata == NULL) {
		TS_LOG_ERR( "%s: Missing platform data\n", __func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}

	md = kzalloc(sizeof(*md), GFP_KERNEL);
	if (md == NULL) {
		TS_LOG_ERR( "%s: Error, kzalloc\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}

	cyttsp5_init_function_ptrs(md);

	mutex_init(&md->mt_lock);
	md->ttsp = ttsp;
	md->pdata = pdata;
	dev_set_drvdata(dev, md);
	/* Create the input device and register it. */
	TS_LOG_INFO( "%s: Create the input device and register it\n",
		__func__);
	md->input = input_allocate_device();
	if (md->input == NULL) {
		TS_LOG_ERR( "%s: Error, failed to allocate input device\n",
			__func__);
		rc = -ENOSYS;
		goto error_alloc_failed;
	}

	if (pdata->inp_dev_name)
		md->input->name = pdata->inp_dev_name;
	else
		md->input->name = ttsp->name;
	scnprintf(md->phys, sizeof(md->phys)-1, "%s", dev_name(dev));
	md->input->phys = md->phys;
	md->input->dev.parent = &md->ttsp->dev;
	md->input->open = cyttsp5_mt_open;
	md->input->close = cyttsp5_mt_close;
	input_set_drvdata(md->input, md);
    
    md->cover_info.enable = 1;

	/* get sysinfo */
	md->si = cyttsp5_request_sysinfo(ttsp);

	if (md->si) {
		rc = cyttsp5_setup_input_device(ttsp);
		if (rc){
			TS_LOG_ERR( "%s: Fail setup input device\n",__func__);
			goto error_init_input;
		}
	} else {
		TS_LOG_ERR( "%s: Fail get sysinfo pointer from core p=%p\n",
			__func__, md->si);
		cyttsp5_subscribe_attention(ttsp, CY_ATTEN_STARTUP,
			cyttsp5_setup_input_attention, 0);
	}

	md->is_suspended = false;
#if defined(CONFIG_FB)
	md->fb_notify.notifier_call = fb_notifier_mt_callback;
	rc = fb_register_client(&md->fb_notify);
	if (rc) {
		TS_LOG_ERR( "%s: Fail to register fb_notifier %d\n",__func__, rc);
		goto err_register_fb_notifier;
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	cyttsp5_setup_early_suspend(md);
#endif

	mt_data = md;

	INIT_WORK(&cover_mode_work, cyttsp5_cover_mode_work_function);
	setup_timer(&cover_mode_timer, cyttsp5_cover_mode_timer,(unsigned long)mt_data);

	return 0;
#if defined(CONFIG_FB)
err_register_fb_notifier:
#endif
error_init_input:
	input_free_device(md->input);
error_alloc_failed:
	dev_set_drvdata(dev, NULL);
	kfree(md);
error_alloc_data_failed:
error_no_pdata:
	TS_LOG_ERR( "%s failed.\n", __func__);
	return rc;
}

struct cyttsp5_driver cyttsp5_mt_driver = {
	.probe = cyttsp5_mt_probe,
	.remove = cyttsp5_mt_release,
	.driver = {
		.name = CYTTSP5_MT_NAME,
		.bus = &cyttsp5_bus_type,
#ifndef CY_TOUCH_RUNTIME_NOTUSED
		.pm = &cyttsp5_mt_pm_ops,
#endif
	},
};

