/*
 * cyttsp5_core.c
 * Cypress TrueTouch(TM) Standard Product V5 Core driver module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2012-2013 Cypress Semiconductor
 * Copyright (C) 2011 Sony Ericsson Mobile Communications AB.
 *
 * Author: Aleksej Makarov <aleksej.makarov@sonyericsson.com>
 * Modified by: Cypress Semiconductor to add device functions
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



#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/hw_dev_dec.h>
#ifdef CONFIG_HUAWEI_KERNEL
#include <misc/app_info.h>
#endif
#include "cyttsp5_core.h"
#include "cyttsp5_i2c.h"
#include "cyttsp5_mt_common.h"
#include "cyttsp5_bus.h"

#ifdef CONFIG_TOUCHSCREEN_MMI_EQUIP
#include "cyttsp5_mmi_test.h"
#endif

MODULE_FIRMWARE(CY_FW_FILE_NAME);
static const char *cy_driver_core_name = CYTTSP5_CORE_NAME;
static const char *cy_driver_core_version = CY_DRIVER_VERSION;
static const char *cy_driver_core_date = CY_DRIVER_DATE;

//extern int get_boot_into_recovery_flag(void); 

#ifdef VERBOSE_DEBUG
void cyttsp5_pr_buf(struct device *dev, u8 *pr_buf, u8 *dptr, int size,
		const char *data_name)
{
	int i, k;
	const char fmt[] = "%02X ";
	int max;

	if (!size)
		return;

	max = (CY_MAX_PRBUF_SIZE - 1) - sizeof(CY_PR_TRUNCATED);

	pr_buf[0] = 0;
	for (i = k = 0; i < size && k < max; i++, k += 3)
		scnprintf(pr_buf + k, CY_MAX_PRBUF_SIZE, fmt, dptr[i]);

	if (size)
		TS_LOG_DEBUG( "%s:  %s[0..%d]=%s%s\n", __func__, data_name,
			size - 1, pr_buf, size <= max ? "" : CY_PR_TRUNCATED);
	else
		TS_LOG_DEBUG( "%s:  %s[]\n", __func__, data_name);
}
EXPORT_SYMBOL(cyttsp5_pr_buf);
#endif

#ifdef TTHE_TUNER_SUPPORT
static int tthe_print(struct cyttsp5_core_data *cd, u8 *buf, int buf_len,
		const u8 *data_name)
{
	int len = strlen(data_name);
	int i, n;
	u8 *p;
	int remain;

	mutex_lock(&cd->tthe_lock);
	if (!cd->tthe_buf)
		goto exit;

	if (cd->tthe_buf_len + (len + buf_len) > CY_MAX_PRBUF_SIZE)
		goto exit;

	if (len + buf_len == 0)
		goto exit;

	remain = CY_MAX_PRBUF_SIZE - cd->tthe_buf_len;
	if (remain < len)
		len = remain;

	p = cd->tthe_buf + cd->tthe_buf_len;
	memcpy(p, data_name, len);
	cd->tthe_buf_len += len;
	p += len;
	remain -= len;

	*p = 0;
	for (i = 0; i < buf_len; i++) {
		n = scnprintf(p, remain, "%02X ", buf[i]);
		if (!n)
			break;
		p += n;
		remain -= n;
		cd->tthe_buf_len += n;
	}

	n = scnprintf(p, remain, "\n");
	if (!n)
		cd->tthe_buf[cd->tthe_buf_len] = 0;
	cd->tthe_buf_len += n;
	wake_up(&cd->wait_q);
exit:
	mutex_unlock(&cd->tthe_lock);
	return 0;
}

static int _cyttsp5_request_tthe_print(struct cyttsp5_device *ttsp, u8 *buf,
		int buf_len, const u8 *data_name)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);
	return tthe_print(cd, buf, buf_len, data_name);
}
#endif

/* Must be called with cd->hid_report_lock acquired */
static struct cyttsp5_hid_report *cyttsp5_get_hid_report_(
		struct cyttsp5_core_data *cd, u8 report_type, u8 report_id,
		bool create)
{
	struct cyttsp5_hid_report *report = NULL;
	int i;

	/* Look for created reports */
	for (i = 0; i < cd->num_hid_reports; i++) {
		if (cd->hid_reports[i]->type == report_type
				&& cd->hid_reports[i]->id == report_id) {
			return cd->hid_reports[i];
		}
	}

	/* Create a new report */
	if (create && cd->num_hid_reports < CY_HID_MAX_REPORTS) {
		report = kzalloc(sizeof(struct cyttsp5_hid_report),
				GFP_KERNEL);
		if (!report)
			return NULL;

		report->type = report_type;
		report->id = report_id;
		cd->hid_reports[cd->num_hid_reports++] = report;
	}

	return report;
}

/* Must be called with cd->hid_report_lock acquired */
static void cyttsp5_free_hid_reports_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_report *report;
	int i, j;

	for (i = 0; i < cd->num_hid_reports; i++) {
		report = cd->hid_reports[i];
		for (j = 0; j < report->num_fields; j++)
			kfree(report->fields[j]);
		kfree(report);
		cd->hid_reports[i] = NULL;
	}

	cd->num_hid_reports = 0;
}

static void cyttsp5_free_hid_reports(struct cyttsp5_core_data *cd)
{
	mutex_lock(&cd->hid_report_lock);
	cyttsp5_free_hid_reports_(cd);
	mutex_unlock(&cd->hid_report_lock);
}

/* Must be called with cd->hid_report_lock acquired */
static struct cyttsp5_hid_field *cyttsp5_create_hid_field_(
		struct cyttsp5_hid_report *report)
{
	struct cyttsp5_hid_field *field;

	if (!report)
		return NULL;

	if (report->num_fields == CY_HID_MAX_FIELDS)
		return NULL;

	field = kzalloc(sizeof(struct cyttsp5_hid_field), GFP_KERNEL);
	if (!field)
		return NULL;

	field->report = report;

	report->fields[report->num_fields++] = field;

	return field;
}

static int request_exclusive(struct cyttsp5_core_data *cd, void *ownptr,
		int timeout_ms)
{
	int t = msecs_to_jiffies(timeout_ms);
	bool with_timeout = (timeout_ms != 0);

	mutex_lock(&cd->system_lock);
	if (!cd->exclusive_dev && cd->exclusive_waits == 0) {
		cd->exclusive_dev = ownptr;
		goto exit;
	}

	cd->exclusive_waits++;
wait:
	mutex_unlock(&cd->system_lock);
	if (with_timeout) {
		t = wait_event_timeout(cd->wait_q, !cd->exclusive_dev, t);
		if (IS_TMO(t)) {
			TS_LOG_ERR( "%s: tmo waiting exclusive access\n",
				__func__);
			return -ETIME;
		}
	} else {
		wait_event(cd->wait_q, !cd->exclusive_dev);
	}
	mutex_lock(&cd->system_lock);
	if (cd->exclusive_dev)
		goto wait;
	cd->exclusive_dev = ownptr;
	cd->exclusive_waits--;
exit:
	mutex_unlock(&cd->system_lock);
	TS_LOG_DEBUG( "%s: request_exclusive ok=%p\n",
		__func__, ownptr);

	return 0;
}

static int release_exclusive_(struct cyttsp5_core_data *cd, void *ownptr)
{
	if (cd->exclusive_dev != ownptr)
		return -EINVAL;

	TS_LOG_DEBUG( "%s: exclusive_dev %p freed\n",
		__func__, cd->exclusive_dev);
	cd->exclusive_dev = NULL;
	wake_up(&cd->wait_q);
	return 0;
}

/*
 * returns error if was not owned
 */
static int release_exclusive(struct cyttsp5_core_data *cd, void *ownptr)
{
	int rc;

	mutex_lock(&cd->system_lock);
	rc = release_exclusive_(cd, ownptr);
	mutex_unlock(&cd->system_lock);

	return rc;
}

static int cyttsp5_hid_exec_cmd_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_cmd *hid_cmd)
{
	int rc;
	u8 *cmd;
	u8 cmd_length;
	u8 cmd_offset = 0;

	cmd_length = 2 /* command register */
		+ 2    /* command */
		+ (hid_cmd->report_id >= 0XF ? 1 : 0)   /* Report ID */
		+ (hid_cmd->has_data_register ? 2 : 0)	/* Data register */
		+ hid_cmd->write_length;                /* Data length */

	cmd = kzalloc(cmd_length, GFP_KERNEL);
	if (!cmd) {
		TS_LOG_ERR( "%s: Fail alloc hid cmd\n", __func__);
		return -ENOMEM;
	}

	/* Set Command register */
	memcpy(&cmd[cmd_offset], &cd->hid_desc.command_register,
			sizeof(cd->hid_desc.command_register));
	cmd_offset += sizeof(cd->hid_desc.command_register);

	/* Set Command */
	SET_CMD_REPORT_TYPE(cmd[cmd_offset], hid_cmd->report_type);

	if (hid_cmd->report_id >= 0XF)
		SET_CMD_REPORT_ID(cmd[cmd_offset], 0xF);
	else
		SET_CMD_REPORT_ID(cmd[cmd_offset], hid_cmd->report_id);
	cmd_offset++;

	SET_CMD_OPCODE(cmd[cmd_offset], hid_cmd->opcode);
	cmd_offset++;

	if (hid_cmd->report_id >= 0XF) {
		cmd[cmd_offset] = hid_cmd->report_id;
		cmd_offset++;
	}

	/* Set Data register */
	if (hid_cmd->has_data_register) {
		memcpy(&cmd[cmd_offset], &cd->hid_desc.data_register,
				sizeof(cd->hid_desc.data_register));
		cmd_offset += sizeof(cd->hid_desc.data_register);
	}

	/* Set Data */
	if (hid_cmd->write_length && hid_cmd->write_buf) {
		memcpy(&cmd[cmd_offset], hid_cmd->write_buf,
				hid_cmd->write_length);
		cmd_offset += hid_cmd->write_length;
	}

	rc = cyttsp5_adap_write_read_specific(cd->core->adap, cmd_length, cmd,
			hid_cmd->read_buf);
	if (rc)
		TS_LOG_ERR( "%s: Fail cyttsp5_adap_transfer\n", __func__);

	kfree(cmd);
	return rc;
}

static int cyttsp5_hid_exec_cmd_and_wait_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_cmd *hid_cmd)
{
	int rc;
	int t;
	u16 timeout_ms;
	int *cmd_state;

	if (hid_cmd->reset_cmd)
		cmd_state = &cd->hid_reset_cmd_state;
	else
		cmd_state = &cd->hid_cmd_state;

	if (hid_cmd->wait_interrupt) {
		mutex_lock(&cd->system_lock);
		*cmd_state = 1;
		mutex_unlock(&cd->system_lock);
	}

	rc = cyttsp5_hid_exec_cmd_(cd, hid_cmd);
	if (rc) {
		if (hid_cmd->wait_interrupt)
			goto error;

		goto exit;
	}

	if (!hid_cmd->wait_interrupt)
		goto exit;

	if (hid_cmd->timeout_ms)
		timeout_ms = hid_cmd->timeout_ms;
	else
		timeout_ms = CY_HID_RESET_TIMEOUT;

	t = wait_event_timeout(cd->wait_q, (*cmd_state == 0),
			msecs_to_jiffies(timeout_ms));
	if (IS_TMO(t)) {
		TS_LOG_ERR( "%s: HID output cmd execution timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	*cmd_state = 0;
	mutex_unlock(&cd->system_lock);

exit:
	return rc;
}

static int cyttsp5_hid_cmd_reset_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_cmd hid_cmd = {
		.opcode = HID_CMD_RESET,
		.wait_interrupt = 1,
		.reset_cmd = 1,
		.timeout_ms = CY_HID_RESET_TIMEOUT,
	};

	return cyttsp5_hid_exec_cmd_and_wait_(cd, &hid_cmd);
}

static int cyttsp5_hid_cmd_reset(struct cyttsp5_core_data *cd)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_cmd_reset_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_cmd_get_report_(struct cyttsp5_core_data *cd,
		u8 report_id, u8 *read_buf)
{
	struct cyttsp5_hid_cmd hid_cmd = {
		.opcode = HID_CMD_GET_REPORT,
		.report_type = 1,
		.has_data_register = 1,
		.read_buf = read_buf,
	};
	hid_cmd.report_id = report_id;

	return cyttsp5_hid_exec_cmd_and_wait_(cd, &hid_cmd);
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_cmd_get_report(struct cyttsp5_core_data *cd,
		u8 report_id, u8 *read_buf)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_cmd_get_report_(cd, report_id, read_buf);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_cmd_set_report_(struct cyttsp5_core_data *cd,
		u8 report_id, u8 write_len, u8 *write_buf)
{
	struct cyttsp5_hid_cmd hid_cmd = {
		.opcode = HID_CMD_SET_REPORT,
		.has_data_register = 1,
		.write_length = write_len,
		.write_buf = write_buf,
	};
	hid_cmd.report_id = report_id;

	return cyttsp5_hid_exec_cmd_and_wait_(cd, &hid_cmd);
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_cmd_set_report(struct cyttsp5_core_data *cd,
		u8 report_id, u8 write_len, u8 *write_buf)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_cmd_set_report_(cd, report_id, write_len, write_buf);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_cmd_set_power_(struct cyttsp5_core_data *cd,
		u8 power_state)
{
	int rc;
    int retry = 3;
	struct cyttsp5_hid_cmd hid_cmd = {
		.opcode = HID_CMD_SET_POWER,
		.wait_interrupt = 1,
		.timeout_ms = CY_HID_SET_POWER_TIMEOUT,
	};
	hid_cmd.power_state = power_state;

again:
	rc =  cyttsp5_hid_exec_cmd_and_wait_(cd, &hid_cmd);
	if (rc) {
		TS_LOG_ERR( "%s: Failed to set power to state:%d\n",
				__func__, power_state);
        //return rc;
        if(retry-- > 0)
            goto again;
        else
            return rc;
	}

	/* validate */
	if ((cd->response_buf[2] != HID_RESPONSE_REPORT_ID)
			|| ((cd->response_buf[3] & 0x3) != power_state)
			|| ((cd->response_buf[4] & 0xF) != HID_CMD_SET_POWER)) {
		TS_LOG_ERR( "%s:"
				"cd->response_buf[2] = %d"
				"cd->response_buf[3] = %d"
				"cd->response_buf[4] = %d"
				"power_state = %d",
				__func__,
				cd->response_buf[2],
				cd->response_buf[3],
				cd->response_buf[4],
				power_state);
        if(retry-- > 0)
            goto again;
        else
		    rc = -EINVAL;
	}

	return rc;
}

static int cyttsp5_hid_cmd_set_power(struct cyttsp5_core_data *cd,
		u8 power_state)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_cmd_set_power_(cd, power_state);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static const u16 crc_table[16] = {
	0x0000, 0x1021, 0x2042, 0x3063,
	0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b,
	0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
};

static u16 _cyttsp5_compute_crc(u8 *buf, u32 size)
{
	u16 remainder = 0xFFFF;
	u16 xor_mask = 0x0000;
	u32 index;
	u32 byte_value;
	u32 table_index;
	u32 crc_bit_width = sizeof(u16) * 8;

	/* Divide the message by polynomial, via the table. */
	for (index = 0; index < size; index++) {
		byte_value = buf[index];
		table_index = ((byte_value >> 4) & 0x0F)
			^ (remainder >> (crc_bit_width - 4));
		remainder = crc_table[table_index] ^ (remainder << 4);
		table_index = (byte_value & 0x0F)
			^ (remainder >> (crc_bit_width - 4));
		remainder = crc_table[table_index] ^ (remainder << 4);
	}

	/* Perform the final remainder CRC. */
	return remainder ^ xor_mask;
}

static int cyttsp5_hid_output_validate_bl_response(
		struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	u16 size;
	u16 crc;
	u8 status;

	if (cd->response_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET]
			!= HID_BL_RESPONSE_REPORT_ID) {
		TS_LOG_ERR( "%s: HID output response, wrong report_id\n",
			__func__);
		return -EPROTO;
	}

	if (cd->response_buf[4] != HID_OUTPUT_BL_SOP) {
		TS_LOG_ERR( "%s: HID output response, wrong SOP\n",
			__func__);
		return -EPROTO;
	}

	size = get_unaligned_le16(&cd->response_buf[0]);
	if (cd->response_buf[size - 1] != HID_OUTPUT_BL_EOP) {
		TS_LOG_ERR( "%s: HID output response, wrong EOP\n",
			__func__);
		return -EPROTO;
	}

	crc = _cyttsp5_compute_crc(&cd->response_buf[4], size - 7);
	if (cd->response_buf[size - 3] != LOW_BYTE(crc)
			|| cd->response_buf[size - 2] != HI_BYTE(crc)) {
		TS_LOG_ERR( "%s: HID output response, wrong CRC 0x%X\n",
			__func__, crc);
		return -EPROTO;
	}

	status = cd->response_buf[5];
	if (status) {
		TS_LOG_ERR( "%s: HID output response, ERROR:%d\n",
			__func__, status);
		return -EPROTO;
	}

	return 0;
}

static int cyttsp5_hid_output_validate_app_response(
		struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	int command_code;

	if (cd->response_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET]
			!= HID_APP_RESPONSE_REPORT_ID) {
		TS_LOG_ERR( "%s: HID output response, wrong report_id:%X\n",
			__func__, cd->response_buf[HID_OUTPUT_RESPONSE_REPORT_OFFSET]);
		return -EPROTO;
	}

	command_code = cd->response_buf[HID_OUTPUT_RESPONSE_CMD_OFFSET]
		& HID_OUTPUT_RESPONSE_CMD_MASK;
	if (command_code != hid_output->command_code) {
		TS_LOG_ERR(
			"%s: HID output response, wrong command_code:%X\n",
			__func__, command_code);
		return -EPROTO;
	}

	return 0;
}

static int cyttsp5_hid_output_validate_response(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	if (hid_output->cmd_type == HID_OUTPUT_CMD_BL)
		return cyttsp5_hid_output_validate_bl_response(cd, hid_output);

	return cyttsp5_hid_output_validate_app_response(cd, hid_output);

}

static int cyttsp5_hid_send_output_user_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	int rc;

	if (!hid_output->length || !hid_output->write_buf)
		return -EINVAL;

	rc = cyttsp5_adap_write_read_specific(cd->core->adap,
			hid_output->length, hid_output->write_buf, NULL);
	if (rc)
		TS_LOG_ERR( "%s: Fail cyttsp5_adap_transfer\n", __func__);

	return rc;
}

static int cyttsp5_hid_send_output_user_and_wait_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	int rc;
	int t;

	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = HID_OUTPUT_USER_CMD + 1;
	mutex_unlock(&cd->system_lock);

	rc = cyttsp5_hid_send_output_user_(cd, hid_output);
	if (rc)
		goto error;

	t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
			msecs_to_jiffies(CY_HID_OUTPUT_USER_TIMEOUT));
	if (IS_TMO(t)) {
		TS_LOG_ERR( "%s: HID output cmd execution timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);

exit:
	return rc;
}

static int cyttsp5_hid_send_output_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	int rc;
	u8 *cmd;
	u16 length;
	u8 report_id;
	u8 cmd_offset = 0;
	u16 crc;

	switch (hid_output->cmd_type) {
	case HID_OUTPUT_CMD_APP:
		report_id = HID_APP_OUTPUT_REPORT_ID;
		length = 5;
		break;
	case HID_OUTPUT_CMD_BL:
		report_id = HID_BL_OUTPUT_REPORT_ID;
		length = 11 /* 5 + SOP + LEN(2) + CRC(2) + EOP */;
		break;
	default:
		return -EINVAL;
	}

	length += hid_output->write_length;

	cmd = kzalloc(length + 2, GFP_KERNEL);
	if (!cmd) {
		TS_LOG_ERR( "%s: Fail alloc hid cmd\n", __func__);
		return -ENOMEM;
	}

	/* Set Output register */
	memcpy(&cmd[cmd_offset], &cd->hid_desc.output_register,
			sizeof(cd->hid_desc.output_register));
	cmd_offset += sizeof(cd->hid_desc.output_register);

	cmd[cmd_offset++] = LOW_BYTE(length);
	cmd[cmd_offset++] = HI_BYTE(length);
	cmd[cmd_offset++] = report_id;
	cmd[cmd_offset++] = 0x0; /* reserved */
	if (hid_output->cmd_type == HID_OUTPUT_CMD_BL)
		cmd[cmd_offset++] = HID_OUTPUT_BL_SOP;
	cmd[cmd_offset++] = hid_output->command_code;

	/* Set Data Length for bootloader */
	if (hid_output->cmd_type == HID_OUTPUT_CMD_BL) {
		cmd[cmd_offset++] = LOW_BYTE(hid_output->write_length);
		cmd[cmd_offset++] = HI_BYTE(hid_output->write_length);
	}
	/* Set Data */
	if (hid_output->write_length && hid_output->write_buf) {
		memcpy(&cmd[cmd_offset], hid_output->write_buf,
				hid_output->write_length);
		cmd_offset += hid_output->write_length;
	}
	if (hid_output->cmd_type == HID_OUTPUT_CMD_BL) {
		crc = _cyttsp5_compute_crc(&cmd[6],
				hid_output->write_length + 4);
		cmd[cmd_offset++] = LOW_BYTE(crc);
		cmd[cmd_offset++] = HI_BYTE(crc);
		cmd[cmd_offset++] = HID_OUTPUT_BL_EOP;
	}

	cyttsp5_pr_buf(cd->dev, cd->pr_buf, cmd, length + 2,
			"command");
	rc = cyttsp5_adap_write_read_specific(cd->core->adap, length + 2,
			cmd, NULL);
	if (rc)
		TS_LOG_ERR( "%s: Fail cyttsp5_adap_transfer\n", __func__);

	kfree(cmd);
	return rc;
}

static int cyttsp5_hid_send_output_and_wait_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_output *hid_output)
{
	int rc;
	int t;
#ifdef VERBOSE_DEBUG
	u16 size;
#endif
	u16 timeout_ms;

	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = hid_output->command_code + 1;
	mutex_unlock(&cd->system_lock);

	if (hid_output->timeout_ms)
		timeout_ms = hid_output->timeout_ms;
	else
		timeout_ms = CY_HID_OUTPUT_TIMEOUT;

	rc = cyttsp5_hid_send_output_(cd, hid_output);
	if (rc)
		goto error;

	/* Workaround for FW defect, CDT165308-	 * bl_launch app creates a glitch in IRQ line */
   /*
	if (hid_output->command_code == HID_OUTPUT_BL_LAUNCH_APP) {
        disable_irq(cd->irq);
        msleep(200);
        enable_irq(cd->irq);
    }
	*/

	t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
			msecs_to_jiffies(timeout_ms));
	if (IS_TMO(t)) {
		TS_LOG_ERR( "%s: HID output cmd execution timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	if (!hid_output->novalidate)
		rc = cyttsp5_hid_output_validate_response(cd, hid_output);

#ifdef VERBOSE_DEBUG
	size = get_unaligned_le16(&cd->response_buf[0]);
	cyttsp5_pr_buf(cd->dev, cd->pr_buf, cd->response_buf, size,
			"return_buf");
#endif

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
exit:
	return rc;
}

static int cyttsp5_hid_output_null_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_NULL),
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_null(struct cyttsp5_core_data *cd)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_null_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_start_bootloader_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_START_BOOTLOADER),
		.timeout_ms = CY_HID_OUTPUT_START_BOOTLOADER_TIMEOUT,
		.novalidate = 1,
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_start_bootloader(struct cyttsp5_core_data *cd)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_start_bootloader_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_start_bl(struct cyttsp5_device *ttsp,
		int protect)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_start_bootloader(cd);
	else
		return cyttsp5_hid_output_start_bootloader_(cd);
}

static void cyttsp5_si_get_cydata(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_cydata *cydata = &cd->sysinfo.cydata;
	struct cyttsp5_cydata_dev *cydata_dev =
		(struct cyttsp5_cydata_dev *)
		&cd->response_buf[HID_SYSINFO_CYDATA_OFFSET];

	cydata->pip_ver_major = cydata_dev->pip_ver_major;
	cydata->pip_ver_minor = cydata_dev->pip_ver_minor;
	cydata->bl_ver_major = cydata_dev->bl_ver_major;
	cydata->bl_ver_minor = cydata_dev->bl_ver_minor;
	cydata->fw_ver_major = cydata_dev->fw_ver_major;
	cydata->fw_ver_minor = cydata_dev->fw_ver_minor;

	cydata->fw_pid = get_unaligned_le16(&cydata_dev->fw_pid);
	cydata->fw_ver_conf = get_unaligned_le16(&cydata_dev->fw_ver_conf);
	cydata->post_code = get_unaligned_le16(&cydata_dev->post_code);
	cydata->revctrl = get_unaligned_le32(&cydata_dev->revctrl);
	cydata->jtag_id_l = get_unaligned_le16(&cydata_dev->jtag_si_id_l);
	cydata->jtag_id_h = get_unaligned_le16(&cydata_dev->jtag_si_id_h);

	memcpy(cydata->mfg_id, cydata_dev->mfg_id, CY_NUM_MFGID);

	cyttsp5_pr_buf(cd->dev, cd->pr_buf, (u8 *)cydata_dev,
			sizeof(struct cyttsp5_cydata_dev), "sysinfo_cydata");
}

static void cyttsp5_si_get_sensing_conf_data(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sensing_conf_data *scd = &cd->sysinfo.sensing_conf_data;
	struct cyttsp5_sensing_conf_data_dev *scd_dev =
		(struct cyttsp5_sensing_conf_data_dev *)
		&cd->response_buf[HID_SYSINFO_SENSING_OFFSET];

	scd->electrodes_x = scd_dev->electrodes_x;
	scd->electrodes_y = scd_dev->electrodes_y;
	scd->origin_x = scd_dev->origin_x;
	scd->origin_y = scd_dev->origin_y;
	scd->panel_id = scd_dev->panel_id;
	scd->btn = scd_dev->btn;
	scd->scan_mode = scd_dev->scan_mode;
	scd->max_num_of_tch_per_refresh_cycle =
		scd_dev->max_num_of_tch_per_refresh_cycle;

	scd->res_x = get_unaligned_le16(&scd_dev->res_x);
	scd->res_y = get_unaligned_le16(&scd_dev->res_y);
	scd->max_z = get_unaligned_le16(&scd_dev->max_z);
	scd->len_x = get_unaligned_le16(&scd_dev->len_x);
	scd->len_y = get_unaligned_le16(&scd_dev->len_y);

	cyttsp5_pr_buf(cd->dev, cd->pr_buf, (u8 *)scd_dev,
			sizeof(struct cyttsp5_sensing_conf_data_dev),
			"sensing_conf_data");
}

static int cyttsp5_si_setup(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;

	if (si->xy_data == NULL)
		si->xy_data = kzalloc(MAX_TOUCH_NUMBER
				* si->desc.tch_record_size, GFP_KERNEL);
	if (si->xy_data == NULL) {
		TS_LOG_ERR( "%s: fail alloc xy_data\n",
			__func__);
		return -ENOMEM;
	}

	if (si->xy_mode == NULL)
		si->xy_mode = kzalloc(si->desc.tch_header_size, GFP_KERNEL);
	if (si->xy_mode == NULL) {
		TS_LOG_ERR( "%s: fail alloc xy_mode\n",
			__func__);
		kfree(si->xy_data);
		return -ENOMEM;
	}

	return 0;
}

static int cyttsp5_si_get_btn_data(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int num_btns = 0;
	int num_defined_keys;
	u16 *key_table;
	int btn;
	int i;
	int rc = 0;
	unsigned int btns = cd->response_buf[HID_SYSINFO_BTN_OFFSET]
		& HID_SYSINFO_BTN_MASK;
	size_t btn_keys_size;

	TS_LOG_DEBUG( "%s: get btn data\n", __func__);

	for (i = 0; i < HID_SYSINFO_MAX_BTN; i++) {
		if (btns & (1 << i))
			num_btns++;
	}
	si->num_btns = num_btns;

	if (num_btns) {
		btn_keys_size = num_btns * sizeof(struct cyttsp5_btn);
		if (si->btn == NULL)
			si->btn = kzalloc(btn_keys_size, GFP_KERNEL);
		if (si->btn == NULL) {
			TS_LOG_ERR( "%s: %s\n", __func__,
				"fail alloc btn_keys memory");
			return -ENOMEM;
		}

		if (cd->pdata->sett[CY_IC_GRPNUM_BTN_KEYS] == NULL)
			num_defined_keys = 0;
		else if (cd->pdata->sett[CY_IC_GRPNUM_BTN_KEYS]->data == NULL)
			num_defined_keys = 0;
		else
			num_defined_keys = cd->pdata->sett
				[CY_IC_GRPNUM_BTN_KEYS]->size;

		for (btn = 0; btn < num_btns && btn < num_defined_keys; btn++) {
			key_table = (u16 *)cd->pdata->sett
				[CY_IC_GRPNUM_BTN_KEYS]->data;
			si->btn[btn].key_code = key_table[btn];
			si->btn[btn].enabled = true;
		}
		for (; btn < num_btns; btn++) {
			si->btn[btn].key_code = KEY_RESERVED;
			si->btn[btn].enabled = true;
		}

		return rc;
	}

	kfree(si->btn);
	si->btn = NULL;
	return rc;
}

static void cyttsp5_si_put_log_data(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	struct cyttsp5_cydata *cydata = &si->cydata;
	struct cyttsp5_sensing_conf_data *scd = &si->sensing_conf_data;
	int i;

	TS_LOG_DEBUG( "%s: pip_ver_major =0x%02X (%d)\n", __func__,
		cydata->pip_ver_major, cydata->pip_ver_major);
	TS_LOG_DEBUG( "%s: pip_ver_minor =0x%02X (%d)\n", __func__,
		cydata->pip_ver_minor, cydata->pip_ver_minor);
	TS_LOG_DEBUG( "%s: fw_pid =0x%04X (%d)\n", __func__,
		cydata->fw_pid, cydata->fw_pid);
	TS_LOG_DEBUG( "%s: fw_ver_major =0x%02X (%d)\n", __func__,
		cydata->fw_ver_major, cydata->fw_ver_major);
	TS_LOG_DEBUG( "%s: fw_ver_minor =0x%02X (%d)\n", __func__,
		cydata->fw_ver_minor, cydata->fw_ver_minor);
	TS_LOG_DEBUG( "%s: revctrl =0x%08X (%d)\n", __func__,
		cydata->revctrl, cydata->revctrl);
	TS_LOG_DEBUG( "%s: fw_ver_conf =0x%04X (%d)\n", __func__,
		cydata->fw_ver_conf, cydata->fw_ver_conf);
	TS_LOG_DEBUG( "%s: bl_ver_major =0x%02X (%d)\n", __func__,
		cydata->bl_ver_major, cydata->bl_ver_major);
	TS_LOG_DEBUG( "%s: bl_ver_minor =0x%02X (%d)\n", __func__,
		cydata->bl_ver_minor, cydata->bl_ver_minor);
	TS_LOG_DEBUG( "%s: jtag_id_h =0x%04X (%d)\n", __func__,
		cydata->jtag_id_h, cydata->jtag_id_h);
	TS_LOG_DEBUG( "%s: jtag_id_l =0x%04X (%d)\n", __func__,
		cydata->jtag_id_l, cydata->jtag_id_l);
	for (i = 0; i < CY_NUM_MFGID; i++)
		TS_LOG_DEBUG( "%s: mfg_id[%d] =0x%02X (%d)\n", __func__, i,
			cydata->mfg_id[i], cydata->mfg_id[i]);
	TS_LOG_DEBUG( "%s: post_code =0x%04X (%d)\n", __func__,
		cydata->post_code, cydata->post_code);

	TS_LOG_DEBUG( "%s: electrodes_x =0x%02X (%d)\n", __func__,
		scd->electrodes_x, scd->electrodes_x);
	TS_LOG_DEBUG( "%s: electrodes_y =0x%02X (%d)\n", __func__,
		scd->electrodes_y, scd->electrodes_y);
	TS_LOG_DEBUG( "%s: len_x =0x%04X (%d)\n", __func__,
		scd->len_x, scd->len_x);
	TS_LOG_DEBUG( "%s: len_y =0x%04X (%d)\n", __func__,
		scd->len_y, scd->len_y);
	TS_LOG_DEBUG( "%s: res_x =0x%04X (%d)\n", __func__,
		scd->res_x, scd->res_x);
	TS_LOG_DEBUG( "%s: res_y =0x%04X (%d)\n", __func__,
		scd->res_y, scd->res_y);
	TS_LOG_DEBUG( "%s: max_z =0x%04X (%d)\n", __func__,
		scd->max_z, scd->max_z);
	TS_LOG_DEBUG( "%s: origin_x =0x%02X (%d)\n", __func__,
		scd->origin_x, scd->origin_x);
	TS_LOG_DEBUG( "%s: origin_y =0x%02X (%d)\n", __func__,
		scd->origin_y, scd->origin_y);
	TS_LOG_DEBUG( "%s: panel_id =0x%02X (%d)\n", __func__,
		scd->panel_id, scd->panel_id);
	TS_LOG_DEBUG( "%s: btn =0x%02X (%d)\n", __func__,
		scd->btn, scd->btn);
	TS_LOG_DEBUG( "%s: scan_mode =0x%02X (%d)\n", __func__,
		scd->scan_mode, scd->scan_mode);
	TS_LOG_DEBUG( "%s: max_num_of_tch_per_refresh_cycle =0x%02X (%d)\n",
		__func__, scd->max_num_of_tch_per_refresh_cycle,
		scd->max_num_of_tch_per_refresh_cycle);

	TS_LOG_DEBUG( "%s: xy_mode =%p\n", __func__,
		si->xy_mode);
	TS_LOG_DEBUG( "%s: xy_data =%p\n", __func__,
		si->xy_data);
}

static int cyttsp5_get_sysinfo_regs(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int rc;

	rc = cyttsp5_si_get_btn_data(cd);
	if (rc < 0)
		return rc;

	cyttsp5_si_get_cydata(cd);

	cyttsp5_si_get_sensing_conf_data(cd);

	cyttsp5_si_setup(cd);

	cyttsp5_si_put_log_data(cd);

	si->ready = true;
	return rc;
}

static void cyttsp5_free_si_ptrs(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;

	kfree(si->btn);
	kfree(si->xy_mode);
	kfree(si->xy_data);
}

static int cyttsp5_hid_output_get_sysinfo_(struct cyttsp5_core_data *cd)
{
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_SYSINFO),
		.timeout_ms = CY_HID_OUTPUT_GET_SYSINFO_TIMEOUT,
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	rc = cyttsp5_get_sysinfo_regs(cd);
	if (rc)
		cyttsp5_free_si_ptrs(cd);

	return rc;
}

static int cyttsp5_hid_output_get_sysinfo(struct cyttsp5_core_data *cd)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_get_sysinfo_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_suspend_scanning_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_SUSPEND_SCANNING),
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_suspend_scanning(struct cyttsp5_core_data *cd)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_suspend_scanning_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_suspend_scanning(
		struct cyttsp5_device *ttsp, int protect)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_suspend_scanning(cd);
	else
		return cyttsp5_hid_output_suspend_scanning_(cd);
}

static int cyttsp5_hid_output_resume_scanning_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_RESUME_SCANNING),
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_resume_scanning(struct cyttsp5_core_data *cd)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_resume_scanning_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_resume_scanning(
		struct cyttsp5_device *ttsp, int protect)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_resume_scanning(cd);
	else
		return cyttsp5_hid_output_resume_scanning_(cd);
}

static int cyttsp5_hid_output_get_param_(struct cyttsp5_core_data *cd,
		u8 param_id, u32 *value)
{
	int write_length = 1;
	u8 param[1] = { param_id };
	u8 read_param_id;
	int param_size;
	u8 *ptr;
	int rc;
	int i;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_PARAM),
		.write_length = write_length,
		.write_buf = param,
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	read_param_id = cd->response_buf[5];
	if (read_param_id != param_id)
		return -EPROTO;

	param_size = cd->response_buf[6];
	ptr = &cd->response_buf[7];
	*value = 0;
	for (i = 0; i < param_size; i++)
		*value += ptr[i] << (i * 8);
	return 0;
}

static int cyttsp5_hid_output_get_param(struct cyttsp5_core_data *cd,
		u8 param_id, u32 *value)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_get_param_(cd, param_id, value);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_get_param(struct cyttsp5_device *ttsp,
		int protect, u8 param_id, u32 *value)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_get_param(cd, param_id, value);
	else
		return cyttsp5_hid_output_get_param_(cd, param_id, value);
}

static int cyttsp5_hid_output_set_param_(struct cyttsp5_core_data *cd,
		u8 param_id, u32 value)
{
	u8 write_buf[6];
	u8 *ptr = &write_buf[1];
	u8 size = 0;
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_SET_PARAM),
		.write_buf = write_buf,
	};

	write_buf[0] = param_id;
	while (value) {
		size++;
		ptr[size] = value & 0xFF;
		value = value >> 8;
	}
	if (size == 0) {
		size = 1;
		ptr[size] = 0;
	}

	write_buf[1] = size;
	hid_output.write_length = 2 + size;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	if (param_id != cd->response_buf[5] || size != cd->response_buf[6])
		return -EPROTO;

	return 0;
}

static int cyttsp5_hid_output_set_param(struct cyttsp5_core_data *cd,
		u8 param_id, u32 value)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_set_param_(cd, param_id, value);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_set_param(struct cyttsp5_device *ttsp,
		int protect, u8 param_id, u32 value)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_set_param(cd, param_id, value);
	else
		return cyttsp5_hid_output_set_param_(cd, param_id, value);
}
static ssize_t cyttsp5_touch_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
    u8 param_id = 0xd0;
    u32 mode = 0;
    int rc = -1;

    rc = cyttsp5_hid_output_get_param(cd, param_id, &mode);
    if (rc < 0)
        TS_LOG_ERR( "%s: cyttsp5_hid_output_get_param failed r=%d\n",
            __func__, rc);

    return snprintf(buf, CY_MAX_PRBUF_SIZE, "0x%04X\n", mode);
}

extern struct cyttsp5_mt_data *mt_data;
static ssize_t cyttsp5_touch_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    int rc;
    unsigned long value;

    rc = kstrtoul(buf, 10, &value);
    if (rc < 0) {
        TS_LOG_ERR( "%s: Invalid value\n", __func__);
        return rc;
    }

    if (mt_data && mt_data->cover_info.cover_mode && HOVER_MODE == value) {
        return rc;
    }

    TS_LOG_INFO( "%s: value = %ld\n",__func__, value);

	switch (value) {
	case FINGER_ONLY_MODE:
	case NORMAL_MODE:
	case STYLUS_ONLY:
	case FINGER_STYLUS:
	case FINGER_STYLUS_ONLY:
		rc = cyttsp5_set_touch_mode(value);
		if (rc < 0){
			TS_LOG_ERR( "%s: cyttsp5_hid_output_set_param failed r=%d\n",
				__func__, rc);
			return rc;
		}
		break;
	default:
		TS_LOG_ERR( "%s: not support mode set!\n", __func__);
		break;
	}

    return size;
}

struct cyttsp5_core_data *globe_cd;

unsigned int cyttsp5_get_touch_mode(void)
{
	u8 param_id = 0xd0;
	u32 mode = 0;
	int rc = -1;

	rc = cyttsp5_hid_output_get_param(globe_cd, param_id, &mode);
	if (rc < 0) {
		TS_LOG_ERR( "%s: cyttsp5_hid_output_get_param failed r=%d\n",
	        	__func__, rc);
		return rc;
	}

	return mode;
}

int cyttsp5_set_touch_mode(u32 value)
{
	int rc = -1;
	u8 param_id = 0xd0;

	TS_LOG_INFO( "%s: value = %d\n",__func__, value);

	rc = cyttsp5_hid_output_set_param(globe_cd, param_id, value);
	if (rc < 0){
	    TS_LOG_ERR( "%s: cyttsp5_hid_output_set_param failed r=%d\n",
	        __func__, rc);
	    return rc;
	}

	return rc;
}

void cyttsp5_set_proximity_flag(bool flag)
{
	if (!globe_cd) {
		printk(KERN_INFO"cyttsp5 not ready!\n");
		return;
	}

	TS_LOG_INFO( "%s: flag=%d\n",__func__, flag);
	if (globe_cd->proximity_flag != flag) 
		globe_cd->proximity_flag = flag;
}
EXPORT_SYMBOL(cyttsp5_set_proximity_flag);

int cyttsp5_get_panel_id(void)
{
	if ((!globe_cd) || (!globe_cd->sysinfo.ready)) {
		printk(KERN_INFO"cyttsp5 SysInfo not ready!\n");
		return -1;
	}

	if (0 == globe_cd->sysinfo.sensing_conf_data.panel_id) {	
		return 0;/*0 = ofilm*/
	} else if (1 == globe_cd->sysinfo.sensing_conf_data.panel_id) {
		return 1;/*1 = lens*/
	}

	return -1;
}
EXPORT_SYMBOL(cyttsp5_get_panel_id);

static int cyttsp5_hid_output_get_noise_metrics_(struct cyttsp5_core_data *cd,
		u16 *noise_level, u8 *metric1, u8 *metric2, u8 *metric3,
		u8 *percent)
{
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_NOISE_METRICS),
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	*noise_level = get_unaligned_le16(&cd->response_buf[5]);
	*metric1 = cd->response_buf[7];
	*metric2 = cd->response_buf[8];
	*metric3 = cd->response_buf[9];
	*percent = cd->response_buf[10];

	return rc;
}

static int cyttsp5_hid_output_get_noise_metrics(struct cyttsp5_core_data *cd,
		u16 *noise_level, u8 *metric1, u8 *metric2, u8 *metric3,
		u8 *percent)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_get_noise_metrics_(cd, noise_level,
			metric1, metric2, metric3, percent);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

/* TODO: Remove inline when function used */
static inline int _cyttsp5_request_hid_output_get_noise_metrics(
		struct cyttsp5_device *ttsp, int protect, u16 *noise_level,
		u8 *metric1, u8 *metric2, u8 *metric3, u8 *percent)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_get_noise_metrics(cd, noise_level,
				metric1, metric2, metric3, percent);
	else
		return cyttsp5_hid_output_get_noise_metrics_(cd, noise_level,
				metric1, metric2, metric3, percent);
}

static int cyttsp5_hid_output_enter_easywake_state_(
		struct cyttsp5_core_data *cd, u8 data, u8 *return_data)
{
	int write_length = 1;
	u8 param[1] = { data };
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_ENTER_EASYWAKE_STATE),
		.write_length = write_length,
		.write_buf = param,
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	*return_data = cd->response_buf[5];
	return rc;
}

static int cyttsp5_hid_output_enter_easywake_state(struct cyttsp5_core_data *cd,
		u8 data, u8 *return_data)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_enter_easywake_state_(cd, data, return_data);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

/* TODO: Remove inline when function used */
static inline int _cyttsp5_request_hid_output_enter_easywake_state(
		struct cyttsp5_device *ttsp, int protect,
		u8 data, u8 *return_data)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_enter_easywake_state(cd, data,
				return_data);
	else
		return cyttsp5_hid_output_enter_easywake_state_(cd, data,
				return_data);
}

static int cyttsp5_hid_output_verify_config_block_crc_(
		struct cyttsp5_core_data *cd, u8 ebid, u8 *status,
		u16 *calculated_crc, u16 *stored_crc)
{
	int write_length = 1;
	u8 param[1] = { ebid };
	u8 *ptr;
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_VERIFY_CONFIG_BLOCK_CRC),
		.write_length = write_length,
		.write_buf = param,
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	ptr = &cd->response_buf[5];
	*status = ptr[0];
	*calculated_crc = get_unaligned_le16(&ptr[1]);
	*stored_crc = get_unaligned_le16(&ptr[3]);
	return 0;
}

static int cyttsp5_hid_output_verify_config_block_crc(
		struct cyttsp5_core_data *cd, u8 ebid, u8 *status,
		u16 *calculated_crc, u16 *stored_crc)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_verify_config_block_crc_(cd, ebid, status,
			calculated_crc, stored_crc);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_verify_config_block_crc(
		struct cyttsp5_device *ttsp, int protect, u8 ebid, u8 *status,
		u16 *calculated_crc, u16 *stored_crc)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_verify_config_block_crc(cd, ebid,
			status, calculated_crc, stored_crc);
	else
		return cyttsp5_hid_output_verify_config_block_crc_(cd, ebid,
			status, calculated_crc, stored_crc);
}

static int cyttsp5_hid_output_get_config_row_size_(struct cyttsp5_core_data *cd,
		u16 *row_size)
{
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_CONFIG_ROW_SIZE),
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	*row_size = get_unaligned_le16(&cd->response_buf[5]);
	return 0;
}

static int cyttsp5_hid_output_get_config_row_size(struct cyttsp5_core_data *cd,
		u16 *row_size)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_get_config_row_size_(cd, row_size);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_get_config_row_size(
		struct cyttsp5_device *ttsp, int protect, u16 *row_size)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_get_config_row_size(cd, row_size);
	else
		return cyttsp5_hid_output_get_config_row_size_(cd, row_size);
}

static int cyttsp5_hid_output_read_conf_block_(struct cyttsp5_core_data *cd,
		u16 row_number, u16 length, u8 ebid, u8 *read_buf, u16 *crc)
{
	int read_ebid;
	int read_length;
	int status;
	int rc;
	int write_length = 5;
	u8 write_buf[5];
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_READ_CONF_BLOCK),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[cmd_offset++] = LOW_BYTE(row_number);
	write_buf[cmd_offset++] = HI_BYTE(row_number);
	write_buf[cmd_offset++] = LOW_BYTE(length);
	write_buf[cmd_offset++] = HI_BYTE(length);
	write_buf[cmd_offset++] = ebid;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	status = cd->response_buf[5];
	if (status)
		return -EINVAL;

	read_ebid = cd->response_buf[6];
	if ((read_ebid != ebid) || (cd->response_buf[9] != 0))
		return -EPROTO;

	read_length = get_unaligned_le16(&cd->response_buf[7]);
	if (length < read_length)
		length = read_length;

	memcpy(read_buf, &cd->response_buf[10], length);
	*crc = get_unaligned_le16(&cd->response_buf[read_length + 10]);

	return 0;
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_output_read_conf_block(
		struct cyttsp5_core_data *cd, u16 row_number, u16 length,
		u8 ebid, u8 *read_buf, u16 *crc)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_read_conf_block_(cd, row_number, length,
			ebid, read_buf, crc);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_write_conf_block_(struct cyttsp5_core_data *cd,
		u16 row_number, u16 write_length, u8 ebid, u8 *write_buf,
		u8 *security_key, u16 *actual_write_len)
{
	/* row_number + write_len + ebid + security_key + crc */
	int full_write_length = 2 + 2 + 1 + write_length + 8 + 2 ;
	u8 *full_write_buf;
	u8 cmd_offset = 0;
	u16 crc;
	int status;
	int rc;
	int read_ebid;
	u8 *data;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_WRITE_CONF_BLOCK),
		.write_length = full_write_length,
	};

	full_write_buf = kzalloc(full_write_length, GFP_KERNEL);
	if (!full_write_buf) {
		TS_LOG_ERR( "%s: Fail alloc write buffer\n", __func__);
		return -ENOMEM;
	}

	hid_output.write_buf = full_write_buf;
	full_write_buf[cmd_offset++] = LOW_BYTE(row_number);
	full_write_buf[cmd_offset++] = HI_BYTE(row_number);
	full_write_buf[cmd_offset++] = LOW_BYTE(write_length);
	full_write_buf[cmd_offset++] = HI_BYTE(write_length);
	full_write_buf[cmd_offset++] = ebid;
	data = &full_write_buf[cmd_offset];
	memcpy(data, write_buf, write_length);
	cmd_offset += write_length;
	memcpy(&full_write_buf[cmd_offset], security_key, 8);
	cmd_offset += 8;
	crc = _cyttsp5_compute_crc(data, write_length);
	full_write_buf[cmd_offset++] = LOW_BYTE(crc);
	full_write_buf[cmd_offset++] = HI_BYTE(crc);

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		goto exit;

	status = cd->response_buf[5];
	if (status) {
		rc = -EINVAL;
		goto exit;
	}

	read_ebid = cd->response_buf[6];
	if (read_ebid != ebid) {
		rc = -EPROTO;
		goto exit;
	}

	*actual_write_len = get_unaligned_le16(&cd->response_buf[7]);

exit:
	kfree(full_write_buf);
	return rc;
}

static int cyttsp5_hid_output_write_conf_block(struct cyttsp5_core_data *cd,
		u16 row_number, u16 write_length, u8 ebid, u8 *write_buf,
		u8 *security_key, u16 *actual_write_len)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_write_conf_block_(cd, row_number, write_length,
			ebid, write_buf, security_key, actual_write_len);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_write_conf_block(
		struct cyttsp5_device *ttsp, int protect,
		u16 row_number, u16 write_length, u8 ebid, u8 *write_buf,
		u8 *security_key, u16 *actual_write_len)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_write_conf_block(cd, row_number,
			write_length, ebid, write_buf, security_key,
			actual_write_len);
	else
		return cyttsp5_hid_output_write_conf_block_(cd, row_number,
			write_length, ebid, write_buf, security_key,
			actual_write_len);
}

static int cyttsp5_hid_output_get_data_structure_(struct cyttsp5_core_data *cd,
		u16 read_offset, u16 read_len, u8 data_id, u16 *act_read_len,
		u8 *read_buf)
{
	int status;
	u8 read_data_id;
	int rc;
	int write_length = 5;
	u8 write_buf[5];
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_DATA_STRUCTURE),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[cmd_offset++] = LOW_BYTE(read_offset);
	write_buf[cmd_offset++] = HI_BYTE(read_offset);
	write_buf[cmd_offset++] = LOW_BYTE(read_len);
	write_buf[cmd_offset++] = HI_BYTE(read_len);
	write_buf[cmd_offset++] = data_id;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	status = cd->response_buf[5];
	if (status)
		return -EINVAL;

	read_data_id = cd->response_buf[6];
	if (read_data_id != data_id)
		return -EPROTO;

	*act_read_len = get_unaligned_le16(&cd->response_buf[7]);
	memcpy(read_buf, &cd->response_buf[9], *act_read_len + 1);

	return 0;
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_output_get_data_structure(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_len,
		u8 data_id, u16 *act_read_len, u8 *read_buf)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_get_data_structure_(cd, read_offset, read_len,
			data_id, act_read_len, read_buf);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_load_self_test_param_(
		struct cyttsp5_core_data *cd, u16 load_offset, u16 load_len,
		u8 test_id, u8 *param_buf, u16 *actual_load_len)
{
	int status;
	u8 read_test_id;
	int rc;
	int write_length = load_len + 5;
	u8 *write_buf;
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_LOAD_SELF_TEST_PARAM),
		.write_length = write_length,
	};

	write_buf = kzalloc(write_length, GFP_KERNEL);
	if (!write_buf) {
		TS_LOG_ERR( "%s: Fail alloc write buffer\n", __func__);
		return -ENOMEM;
	}

	hid_output.write_buf = write_buf;
	write_buf[cmd_offset++] = LOW_BYTE(load_offset);
	write_buf[cmd_offset++] = HI_BYTE(load_offset);
	write_buf[cmd_offset++] = LOW_BYTE(load_len);
	write_buf[cmd_offset++] = HI_BYTE(load_len);
	write_buf[cmd_offset++] = test_id;
	memcpy(&write_buf[cmd_offset], param_buf, load_len);

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		goto exit;

	status = cd->response_buf[5];
	if (status) {
		rc = -EINVAL;
		goto exit;
	}

	read_test_id = cd->response_buf[6];
	if (read_test_id != test_id) {
		rc = -EPROTO;
		goto exit;
	}

	*actual_load_len = get_unaligned_le16(&cd->response_buf[7]);

exit:
	kfree(write_buf);
	return rc;
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_output_load_self_test_param(
		struct cyttsp5_core_data *cd, u16 load_offset, u16 load_len,
		u8 test_id, u8 *param_buf, u16 *actual_load_len)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_load_self_test_param_(cd, load_offset,
			load_len, test_id, param_buf, actual_load_len);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_run_self_test_(struct cyttsp5_core_data *cd,
		u8 test_id, u8 write_idac, u8 *result, u8 *available)
{
	int status;
	int rc;
	int write_length = 2;
	u8 write_buf[2];
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_RUN_SELF_TEST),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[cmd_offset++] = test_id;
	write_buf[cmd_offset++] = write_idac;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	status = cd->response_buf[5];
	if (status)
		return -EINVAL;

	*result = cd->response_buf[6];
	*available = cd->response_buf[7];

	return 0;
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_output_run_self_test(
		struct cyttsp5_core_data *cd, u8 test_id, u8 write_idac,
		u8 *result, u8 *available)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_run_self_test_(cd, test_id, write_idac,
			result, available);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_get_self_test_result_(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_length,
		u8 test_id, u16 *actual_read_len, u8 *read_buf)
{
	int status;
	u8 read_test_id;
	int rc;
	int write_length = 5;
	u8 write_buf[5];
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_GET_SELF_TEST_RESULT),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[cmd_offset++] = LOW_BYTE(read_offset);
	write_buf[cmd_offset++] = HI_BYTE(read_offset);
	write_buf[cmd_offset++] = LOW_BYTE(read_length);
	write_buf[cmd_offset++] = HI_BYTE(read_length);
	write_buf[cmd_offset++] = test_id;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	status = cd->response_buf[5];
	if (status)
		return -EINVAL;

	read_test_id = cd->response_buf[6];
	if (read_test_id != test_id)
		return -EPROTO;

	*actual_read_len = get_unaligned_le16(&cd->response_buf[7]);

	memcpy(read_buf, &cd->response_buf[10], *actual_read_len);
	return rc;
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_output_get_self_test_result(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_length,
		u8 test_id, u16 *actual_read_len, u8 *read_buf)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_get_self_test_result_(cd, read_offset,
			read_length, test_id, actual_read_len, read_buf);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_calibrate_idacs_(struct cyttsp5_core_data *cd,
		u8 mode)
{
	int status;
	int rc;
	int write_length = 1;
	u8 write_buf[1];
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_CALIBRATE_IDACS),
		.write_length = write_length,
		.write_buf = write_buf,
		.timeout_ms = CY_HID_OUTPUT_CALIBRATE_IDAC_TIMEOT,
	};

	write_buf[cmd_offset++] = mode;
	rc =  cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	status = cd->response_buf[5];
	if (status)
		return -EINVAL;

	return 0;
}

static int cyttsp5_hid_output_calibrate_idacs(struct cyttsp5_core_data *cd,
		u8 mode)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_calibrate_idacs_(cd, mode);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_calibrate_idacs(
		struct cyttsp5_device *ttsp, int protect, u8 mode)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_calibrate_idacs(cd, mode);
	else
		return cyttsp5_hid_output_calibrate_idacs_(cd, mode);
}

static int cyttsp5_hid_output_initialize_baselines_(
		struct cyttsp5_core_data *cd, u8 test_id)
{
	int status;
	int rc;
	int write_length = 1;
	u8 write_buf[1];
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_INITIALIZE_BASELINES),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[cmd_offset++] = test_id;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	status = cd->response_buf[5];
	if (status)
		return -EINVAL;

	return rc;
}

static int cyttsp5_hid_output_initialize_baselines(struct cyttsp5_core_data *cd,
		u8 test_id)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_initialize_baselines_(cd, test_id);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_initialize_baselines(
		struct cyttsp5_device *ttsp, int protect, u8 test_id)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_initialize_baselines(cd, test_id);
	else
		return cyttsp5_hid_output_initialize_baselines_(cd, test_id);
}

static int cyttsp5_hid_output_exec_panel_scan_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_EXEC_PANEL_SCAN),
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_exec_panel_scan(struct cyttsp5_core_data *cd)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_exec_panel_scan_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_exec_panel_scan(
		struct cyttsp5_device *ttsp, int protect)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_exec_panel_scan(cd);
	else
		return cyttsp5_hid_output_exec_panel_scan_(cd);
}

/* @response: set none NULL only if all response required including header */
static int cyttsp5_hid_output_retrieve_panel_scan_(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_count,
		u8 data_id, u8 *response, u8 *config, u16 *actual_read_len,
		u8 *read_buf)
{
	int status;
	u8 read_data_id;
	int rc;
	int write_length = 5;
	u8 write_buf[5];
	u8 cmd_offset = 0;
	u8 data_elem_size;
	int size;
	int data_size;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_RETRIEVE_PANEL_SCAN),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[cmd_offset++] = LOW_BYTE(read_offset);
	write_buf[cmd_offset++] = HI_BYTE(read_offset);
	write_buf[cmd_offset++] = LOW_BYTE(read_count);
	write_buf[cmd_offset++] = HI_BYTE(read_count);
	write_buf[cmd_offset++] = data_id;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	status = cd->response_buf[5];
	if (status)
		return -EINVAL;

	read_data_id = cd->response_buf[6];
	if (read_data_id != data_id)
		return -EPROTO;

	size = get_unaligned_le16(&cd->response_buf[0]);
	*actual_read_len = get_unaligned_le16(&cd->response_buf[7]);
	*config = cd->response_buf[9];

	data_elem_size = *config & 0x07;
	data_size = *actual_read_len * data_elem_size;

	if (read_buf)
		memcpy(read_buf, &cd->response_buf[10], data_size);
	if (response)
		memcpy(response, cd->response_buf, size);
	return rc;
}

static int cyttsp5_hid_output_retrieve_panel_scan(
		struct cyttsp5_core_data *cd, u16 read_offset, u16 read_count,
		u8 data_id, u8 *response, u8 *config, u16 *actual_read_len,
		u8 *read_buf)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_retrieve_panel_scan_(cd, read_offset,
			read_count, data_id, response, config,
			actual_read_len, read_buf);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_retrieve_panel_scan(
		struct cyttsp5_device *ttsp, int protect,
		u16 read_offset, u16 read_count, u8 data_id, u8 *response,
		u8 *config, u16 *actual_read_len, u8 *read_buf)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_retrieve_panel_scan(cd,
				read_offset, read_count, data_id, response,
				config, actual_read_len, read_buf);
	else
		return cyttsp5_hid_output_retrieve_panel_scan_(cd,
				read_offset, read_count, data_id, response,
				config, actual_read_len, read_buf);
}

static int cyttsp5_hid_output_start_sensor_data_mode_(
		struct cyttsp5_core_data *cd, u8 data_point_desc_count,
		u8 *write_buf)
{
	int write_length = data_point_desc_count * 2;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_START_SENSOR_DATA_MODE),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_output_start_sensor_data_mode(
		struct cyttsp5_core_data *cd, u8 data_point_desc_count,
		u8 *write_buf)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_start_sensor_data_mode_(cd,
			data_point_desc_count, write_buf);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_stop_sensor_data_mode_(
		struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_STOP_SENSOR_DATA_MODE),
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_output_stop_sensor_data_mode(
		struct cyttsp5_core_data *cd)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_stop_sensor_data_mode_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_start_tracking_heatmap_mode_(
		struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_START_TRACKING_HEATMAP_MODE),
	};

	if (cd->sysinfo.ready && IS_PIP_VER_GE(&cd->sysinfo, 1, 2))
		return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	else
		return -EINVAL;
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_output_start_tracking_heatmap_mode(
		struct cyttsp5_core_data *cd)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_start_tracking_heatmap_mode_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_int_pin_override_(struct cyttsp5_core_data *cd,
		u8 bit_mask)
{
	int status;
	int rc;
	int write_length = 1;
	u8 write_buf[1];
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_INT_PIN_OVERRIDE),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[0] = bit_mask;

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	status = cd->response_buf[5];
	if (status)
		return -EINVAL;

	return rc;
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_output_int_pin_override(
		struct cyttsp5_core_data *cd, u8 bit_mask)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_int_pin_override_(cd, bit_mask);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_store_panel_scan_(
		struct cyttsp5_core_data *cd, u16 write_offset, u16 write_len,
		u16 elem_size, u8 data_id, u8 *write_buf)
{
	int write_length = write_len * elem_size;
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_STORE_PANEL_SCAN),
		.write_length = write_length,
		.write_buf = write_buf,
	};

	write_buf[cmd_offset++] = LOW_BYTE(write_offset);
	write_buf[cmd_offset++] = HI_BYTE(write_offset);
	write_buf[cmd_offset++] = LOW_BYTE(write_len);
	write_buf[cmd_offset++] = HI_BYTE(write_len);
	write_buf[cmd_offset++] = LOW_BYTE(elem_size);
	write_buf[cmd_offset++] = HI_BYTE(elem_size);
	write_buf[cmd_offset++] = data_id;

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_output_store_panel_scan(
		struct cyttsp5_core_data *cd, u16 write_offset, u16 write_len,
		u16 elem_size, u8 data_id, u8 *write_buf)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_store_panel_scan_(cd, write_offset,
			write_len, elem_size, data_id, write_buf);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_process_panel_scan_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_PROCESS_PANEL_SCAN),
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_output_process_panel_scan(
		struct cyttsp5_core_data *cd)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_process_panel_scan_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_discard_input_report_(
		struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_APP_COMMAND(HID_OUTPUT_DISCARD_INPUT_REPORT),
	};

	return cyttsp5_hid_send_output_(cd, &hid_output);
}

/* TODO: Remove inline when function used */
static inline int cyttsp5_hid_output_discard_input_report(
		struct cyttsp5_core_data *cd)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_discard_input_report_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_hid_output_user_cmd_(struct cyttsp5_core_data *cd,
		u16 read_len, u8 *read_buf, u16 write_len, u8 *write_buf,
		u16 *actual_read_len)
{
	int rc;
	u16 size;
#ifdef TTHE_TUNER_SUPPORT
	int command_code = 0;
	int len;
#endif
	struct cyttsp5_hid_output hid_output = {
		.length = write_len,
		.write_buf = write_buf,
	};

	rc = cyttsp5_hid_send_output_user_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	size = get_unaligned_le16(&cd->response_buf[0]);
	if (size == 0)
		size = 2;

	if (size > read_len) {
		*actual_read_len = 0;
		return -EINVAL;
	}

	memcpy(read_buf, cd->response_buf, size);
	*actual_read_len = size;

#ifdef TTHE_TUNER_SUPPORT
	/* print up to cmd code */
	len = HID_OUTPUT_CMD_OFFSET + 1;
	if (write_len < len)
		len = write_len;
	else
		command_code = write_buf[HID_OUTPUT_CMD_OFFSET]
			& HID_OUTPUT_CMD_MASK;

	/* Do not print for EXEC_PANEL_SCAN & RETRIEVE_PANEL_SCAN commands */
	if (command_code != HID_OUTPUT_EXEC_PANEL_SCAN
			&& command_code != HID_OUTPUT_RETRIEVE_PANEL_SCAN)
		tthe_print(cd, write_buf, len, "CMD=");
#endif

	return 0;
}

static int cyttsp5_hid_output_user_cmd(struct cyttsp5_core_data *cd,
		u16 read_len, u8 *read_buf, u16 write_len, u8 *write_buf,
		u16 *actual_read_len)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_user_cmd_(cd, read_len, read_buf,
			write_len, write_buf, actual_read_len);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_user_cmd(struct cyttsp5_device *ttsp,
		int protect, u16 read_len, u8 *read_buf,
		u16 write_len, u8 *write_buf, u16 *actual_read_len)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_user_cmd(cd, read_len, read_buf,
			write_len, write_buf, actual_read_len);
	else
		return cyttsp5_hid_output_user_cmd_(cd, read_len, read_buf,
			write_len, write_buf, actual_read_len);
}

static int cyttsp5_hid_output_bl_get_information_(struct cyttsp5_core_data *cd,
		u8 *return_data)
{
	int rc;
	int data_len;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_GET_INFO),
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;

	data_len = get_unaligned_le16(&cd->input_buf[6]);
	if (!data_len)
		return -EPROTO;

	memcpy(return_data, &cd->response_buf[8], data_len);

	return 0;
}

static int cyttsp5_hid_output_bl_get_information(struct cyttsp5_core_data *cd,
		u8 *return_data)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_get_information_(cd, return_data);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_bl_get_information(
		struct cyttsp5_device *ttsp, int protect, u8 *return_data)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_bl_get_information(cd, return_data);
	else
		return cyttsp5_hid_output_bl_get_information_(cd, return_data);
}

static int cyttsp5_hid_output_bl_initiate_bl_(struct cyttsp5_core_data *cd,
		u16 key_size, u8 *key_buf, u16 row_size, u8 *metadata_row_buf)
{
	u16 write_length = key_size + row_size;
	u8 *write_buf;
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_INITIATE_BL),
		.write_length = write_length,
		.timeout_ms = CY_HID_OUTPUT_BL_INITIATE_BL_TIMEOUT,
	};

	write_buf = kzalloc(write_length, GFP_KERNEL);
	if (!write_buf) {
		TS_LOG_ERR( "%s: Fail alloc write buffer\n", __func__);
		return -ENOMEM;
	}
	hid_output.write_buf = write_buf;

	if (key_size)
		memcpy(write_buf, key_buf, key_size);

	if (row_size)
		memcpy(&write_buf[key_size], metadata_row_buf, row_size);

	rc =  cyttsp5_hid_send_output_and_wait_(cd, &hid_output);

	kfree(write_buf);
	return rc;
}

static int cyttsp5_hid_output_bl_initiate_bl(struct cyttsp5_core_data *cd,
		u16 key_size, u8 *key_buf, u16 row_size, u8 *metadata_row_buf)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_initiate_bl_(cd, key_size, key_buf,
			row_size, metadata_row_buf);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_bl_initiate_bl(
		struct cyttsp5_device *ttsp, int protect,
		u16 key_size, u8 *key_buf, u16 row_size, u8 *metadata_row_buf)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_bl_initiate_bl(cd, key_size, key_buf,
				row_size, metadata_row_buf);
	else
		return cyttsp5_hid_output_bl_initiate_bl_(cd, key_size, key_buf,
				row_size, metadata_row_buf);
}

static int cyttsp5_hid_output_bl_append_data_buff_(struct cyttsp5_core_data *cd,
		u16 data_len, u8 *data_buf)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_APPEND_DATA_BUFF),
		.write_length = data_len,
		.write_buf = data_buf,
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_bl_append_data_buff(struct cyttsp5_core_data *cd,
		u16 data_len, u8 *data_buf)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_append_data_buff_(cd, data_len, data_buf);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

/* TODO: Remove inline when function used */
static inline int _cyttsp5_request_hid_output_bl_append_data_buff(
		struct cyttsp5_device *ttsp, int protect, u16 data_len,
		u8 *data_buf)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_bl_append_data_buff(cd, data_len,
				data_buf);
	else
		return cyttsp5_hid_output_bl_append_data_buff_(cd, data_len,
				data_buf);
}

static int cyttsp5_hid_output_bl_program_and_verify_(
		struct cyttsp5_core_data *cd, u16 data_len, u8 *data_buf)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_PROGRAM_AND_VERIFY),
		.write_length = data_len,
		.write_buf = data_buf,
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_bl_program_and_verify(
		struct cyttsp5_core_data *cd, u16 data_len, u8 *data_buf)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_program_and_verify_(cd, data_len, data_buf);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_bl_program_and_verify(
		struct cyttsp5_device *ttsp, int protect,
		u16 data_len, u8 *data_buf)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_bl_program_and_verify(cd, data_len,
				data_buf);
	else
		return cyttsp5_hid_output_bl_program_and_verify_(cd, data_len,
				data_buf);
}

static int cyttsp5_hid_output_bl_verify_app_integrity_(
		struct cyttsp5_core_data *cd, u8 *result)
{
	int rc;
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_VERIFY_APP_INTEGRITY),
	};

	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc) {
		*result = 0;
		return rc;
	}

	*result = cd->response_buf[8];
	return 0;
}

static int cyttsp5_hid_output_bl_verify_app_integrity(
		struct cyttsp5_core_data *cd, u8 *result)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_verify_app_integrity_(cd, result);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_bl_verify_app_integrity(
		struct cyttsp5_device *ttsp, int protect, u8 *result)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_bl_verify_app_integrity(cd, result);
	else
		return cyttsp5_hid_output_bl_verify_app_integrity_(cd, result);
}

static int cyttsp5_hid_output_bl_launch_app_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_hid_output hid_output = {
		HID_OUTPUT_BL_COMMAND(HID_OUTPUT_BL_LAUNCH_APP),
		.novalidate = 1,
	};

	return cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
}

static int cyttsp5_hid_output_bl_launch_app(struct cyttsp5_core_data *cd)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_hid_output_bl_launch_app_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_hid_output_launch_app(struct cyttsp5_device *ttsp,
		int protect)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_hid_output_bl_launch_app(cd);
	else
		return cyttsp5_hid_output_bl_launch_app_(cd);
}

static inline int cyttsp5_bits_2_bytes(int nbits, int *max)
{
	*max = 1 << nbits;
	return (nbits + 7) / 8;
}

static int cyttsp5_get_hid_descriptor_(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_desc *desc)
{
	struct cyttsp5_core *core = cd->core;
	int rc;
	int t;
	u8 cmd[2];

	/* Read HID descriptor length and version */
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 1;
	mutex_unlock(&cd->system_lock);

	/* Set HID descriptor register */
	memcpy(cmd, &cd->hid_core.hid_desc_register,
		sizeof(cd->hid_core.hid_desc_register));

	rc = cyttsp5_adap_write_read_specific(core->adap, 2,
			cmd, NULL);
	if (rc) {
		TS_LOG_ERR( "%s: failed to get HID descriptor length and version, rc=%d\n",
			__func__, rc);
		goto error;
	}

	t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
			msecs_to_jiffies(CY_HID_GET_HID_DESCRIPTOR_TIMEOUT));
	if (IS_TMO(t)) {
		TS_LOG_ERR( "%s: HID get descriptor timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	memcpy((u8 *)desc, cd->response_buf, sizeof(struct cyttsp5_hid_desc));

	/* Check HID descriptor length and version */
	TS_LOG_DEBUG( "%s: HID len:%X HID ver:%X\n", __func__,
		le16_to_cpu(desc->hid_desc_len),
		le16_to_cpu(desc->bcd_version));

	if (le16_to_cpu(desc->hid_desc_len) != sizeof(*desc) ||
		le16_to_cpu(desc->bcd_version) != CY_HID_VERSION) {
		TS_LOG_ERR( "%s: Unsupported HID version\n", __func__);
		return -ENOSYS;
	}

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
exit:
	return rc;
}

static int cyttsp5_get_hid_descriptor(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_desc *desc)
{
	int rc;
	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return rc;
	}

	rc = cyttsp5_get_hid_descriptor_(cd, desc);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

	return rc;
}

static int _cyttsp5_request_get_hid_desc(struct cyttsp5_device *ttsp,
		int protect)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (protect)
		return cyttsp5_get_hid_descriptor(cd, &cd->hid_desc);
	else
		return cyttsp5_get_hid_descriptor_(cd, &cd->hid_desc);
}

static int cyttsp5_hw_soft_reset(struct cyttsp5_core_data *cd)
{
	int rc;
	if (cd->hid_desc.hid_desc_len == 0) {
		rc = cyttsp5_get_hid_descriptor_(cd, &cd->hid_desc);
		if (rc < 0)
			return rc;
	}

	rc = cyttsp5_hid_cmd_reset_(cd);
	if (rc < 0) {
		TS_LOG_ERR( "%s: FAILED to execute SOFT reset\n",
				__func__);
		return rc;
	}
	TS_LOG_DEBUG( "%s: execute SOFT reset\n", __func__);
	return 0;
}

static int cyttsp5_hw_hard_reset(struct cyttsp5_core_data *cd)
{
	if (cd->pdata->xres) {
		cd->pdata->xres(cd->pdata, cd->dev);
		TS_LOG_DEBUG( "%s: execute HARD reset\n", __func__);
		return 0;
	}
	TS_LOG_ERR( "%s: FAILED to execute HARD reset\n", __func__);
	return -ENOSYS;
}

static int cyttsp5_hw_reset(struct cyttsp5_core_data *cd)
{
	int rc;

	mutex_lock(&cd->system_lock);
	rc = cyttsp5_hw_hard_reset(cd);
	mutex_unlock(&cd->system_lock);
	if (rc == -ENOSYS)
		rc = cyttsp5_hw_soft_reset(cd);
	return rc;
}

int cyttsp5_hw_reset_export(void)
{
	return cyttsp5_hw_reset(globe_cd);
}

static inline int get_hid_item_data(u8 *data, int item_size)
{
	if (item_size == 1)
		return (int)*data;
	else if (item_size == 2)
		return (int)get_unaligned_le16(data);
	else if (item_size == 4)
		return (int)get_unaligned_le32(data);
	return 0;
}

static int parse_report_descriptor(struct cyttsp5_core_data *cd,
		u8 *report_desc, size_t len)
{
	struct cyttsp5_hid_report *report;
	struct cyttsp5_hid_field *field;
	u8 *buf = report_desc;
	u8 *end = buf + len;
	int rc = 0;
	int offset = 0;
	int i;
	u8 report_type;
	u32 up_usage;
	/* Global items */
	u8 report_id = 0;
	u16 usage_page = 0;
	int report_count = 0;
	int report_size = 0;
	int logical_min = 0;
	int logical_max = 0;
	/* Local items */
	u16 usage = 0;
	/* Main items - Collection stack */
	u32 collection_usages[CY_HID_MAX_NESTED_COLLECTIONS];
	u8 collection_types[CY_HID_MAX_NESTED_COLLECTIONS];
	/* First collection for header, second for report */
	int logical_collection_count = 0;
	int collection_nest = 0;

	TS_LOG_DEBUG( "%s: Report descriptor length: %d\n",
		__func__, len);

	mutex_lock(&cd->hid_report_lock);
	cyttsp5_free_hid_reports_(cd);

	while (buf < end) {
		int item_type;
		int item_size;
		int item_tag;
		u8 *data;

		/* Get Item */
		item_size = HID_GET_ITEM_SIZE(buf[0]);
		if (item_size == 3)
			item_size = 4;
		item_type = HID_GET_ITEM_TYPE(buf[0]);
		item_tag = HID_GET_ITEM_TAG(buf[0]);

		data = ++buf;
		buf += item_size;

		/* Process current item */
		switch (item_type) {
		case HID_ITEM_TYPE_GLOBAL:
			switch (item_tag) {
			case HID_GLOBAL_ITEM_TAG_REPORT_ID:
				if (item_size != 1) {
					rc = -EINVAL;
					goto exit;
				}
				report_id = get_hid_item_data(data, item_size);
				offset = 0;
				logical_collection_count = 0;
				break;
			case HID_GLOBAL_ITEM_TAG_USAGE_PAGE:
				if (item_size == 0 || item_size == 4) {
					rc = -EINVAL;
					goto exit;
				}
				usage_page = (u16)get_hid_item_data(data,
						item_size);
				break;
			case HID_GLOBAL_ITEM_TAG_LOGICAL_MINIMUM:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				logical_min = get_hid_item_data(data,
						item_size);
				break;
			case HID_GLOBAL_ITEM_TAG_LOGICAL_MAXIMUM:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				logical_max = get_hid_item_data(data,
						item_size);
				break;
			case HID_GLOBAL_ITEM_TAG_REPORT_COUNT:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				report_count = get_hid_item_data(data,
						item_size);
				break;
			case HID_GLOBAL_ITEM_TAG_REPORT_SIZE:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				report_size = get_hid_item_data(data,
						item_size);
				break;
			default:
				TS_LOG_INFO(
					"%s: Unrecognized Global tag %d\n",
					__func__, item_tag);
			}
			break;
		case HID_ITEM_TYPE_LOCAL:
			switch (item_tag) {
			case HID_LOCAL_ITEM_TAG_USAGE:
				if (item_size == 0 || item_size == 4) {
					rc = -EINVAL;
					goto exit;
				}
				usage = (u16)get_hid_item_data(data,
						item_size);
				break;
			case HID_LOCAL_ITEM_TAG_USAGE_MINIMUM:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				/* usage_min = */
				get_hid_item_data(data, item_size);
				break;
			case HID_LOCAL_ITEM_TAG_USAGE_MAXIMUM:
				if (item_size == 0) {
					rc = -EINVAL;
					goto exit;
				}
				/* usage_max = */
				get_hid_item_data(data, item_size);
				break;
			default:
				TS_LOG_INFO(
					"%s: Unrecognized Local tag %d\n",
					__func__, item_tag);
			}
			break;
		case HID_ITEM_TYPE_MAIN:
			switch (item_tag) {
			case HID_MAIN_ITEM_TAG_BEGIN_COLLECTION:
				if (item_size != 1) {
					rc = -EINVAL;
					goto exit;
				}
				if (CY_HID_MAX_NESTED_COLLECTIONS ==
						collection_nest) {
					rc = -EINVAL;
					goto exit;
				}

				up_usage = usage_page << 16 | usage;

				/* Update collection stack */
				collection_usages[collection_nest] = up_usage;
				collection_types[collection_nest] =
					get_hid_item_data(data, item_size);

				if (collection_types[collection_nest] ==
						HID_COLLECTION_LOGICAL)
					logical_collection_count++;

				collection_nest++;
				break;
			case HID_MAIN_ITEM_TAG_END_COLLECTION:
				if (item_size != 0) {
					rc = -EINVAL;
					goto exit;
				}
				if (--collection_nest < 0) {
					rc = -EINVAL;
					goto exit;
				}
				break;
			case HID_MAIN_ITEM_TAG_INPUT:
				report_type = HID_INPUT_REPORT;
				goto continue_main_item;
			case HID_MAIN_ITEM_TAG_OUTPUT:
				report_type = HID_OUTPUT_REPORT;
				goto continue_main_item;
			case HID_MAIN_ITEM_TAG_FEATURE:
				report_type = HID_FEATURE_REPORT;
continue_main_item:
				if (item_size != 1) {
					rc = -EINVAL;
					goto exit;
				}

				up_usage = usage_page << 16 | usage;

				/* Get or create report */
				report = cyttsp5_get_hid_report_(cd,
						report_type, report_id, true);
				if (!report) {
					rc = -ENOMEM;
					goto exit;
				}
				if (!report->usage_page)
					report->usage_page =
						collection_usages
							[collection_nest - 1];

				/* Create field */
				field = cyttsp5_create_hid_field_(report);
				if (!field) {
					rc = -ENOMEM;
					goto exit;
				}

				field->report_count = report_count;
				field->report_size = report_size;
				field->size = report_count * report_size;
				field->offset = offset;
				field->data_type =
					get_hid_item_data(data, item_size);
				field->logical_min = logical_min;
				field->logical_max = logical_max;
				field->usage_page = up_usage;

				for (i = 0; i < collection_nest; i++) {
					field->collection_usage_pages
							[collection_types[i]] =
						collection_usages[i];
				}

				/* Update report's header or record size */
				if (logical_collection_count == 1) {
					report->header_size += field->size;
				} else if (logical_collection_count == 2) {
					field->record_field = true;
					field->offset -= report->header_size;
					/* Set record field index */
					if (report->record_field_index == 0)
						report->record_field_index =
							report->num_fields - 1;
					report->record_size += field->size;
				}

				report->size += field->size;

				offset += field->size;
				break;
			default:
				TS_LOG_INFO( "%s: Unrecognized Main tag %d\n",
					__func__, item_tag);
			}

			/* Reset all local items */
			usage = 0;
			break;
		}
	}

	if (buf != end) {
		TS_LOG_ERR( "%s: Report descriptor length invalid\n",
			__func__);
		rc = -EINVAL;
		goto exit;
	}

	if (collection_nest) {
		TS_LOG_ERR( "%s: Unbalanced collection items (%d)\n",
			__func__, collection_nest);
		rc = -EINVAL;
		goto exit;
	}

exit:
	if (rc)
		cyttsp5_free_hid_reports_(cd);
	mutex_unlock(&cd->hid_report_lock);
	return rc;
}

static struct cyttsp5_hid_field *find_report_desc_field(
		struct cyttsp5_core_data *cd, u32 usage_page,
		u32 collection_usage_page)
{
	struct cyttsp5_hid_report *report = NULL;
	struct cyttsp5_hid_field *field = NULL;
	int i;
	int j;
	u32 field_cup;
	u32 field_up;

	for (i = 0; i < cd->num_hid_reports; i++) {
		report = cd->hid_reports[i];
		for (j = 0; j < report->num_fields; j++) {
			field = report->fields[j];
			field_cup = field->collection_usage_pages
				[HID_COLLECTION_LOGICAL];
			field_up = field->usage_page;
			if (field_cup == collection_usage_page
					&& field_up == usage_page) {
				return field;
			}
		}
	}

	return NULL;
}

static int fill_tch_abs(struct cyttsp5_tch_abs_params *tch_abs,
		struct cyttsp5_hid_field *field)
{
	tch_abs->ofs = field->offset / 8;
	tch_abs->size = field->report_size / 8;
	if (field->report_size % 8)
		tch_abs->size += 1;
	tch_abs->min = 0;
	tch_abs->max = 1 << field->report_size;
	tch_abs->bofs = field->offset - (tch_abs->ofs << 3);

	return 0;
}

static struct cyttsp5_hid_report *find_report_desc(struct cyttsp5_core_data *cd,
		u32 usage_page)
{
	struct cyttsp5_hid_report *report = NULL;
	int i;

	for (i = 0; i < cd->num_hid_reports; i++) {
		if (cd->hid_reports[i]->usage_page == usage_page) {
			report = cd->hid_reports[i];
			break;
		}
	}

	return report;
}

static int setup_report_descriptor(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	struct cyttsp5_hid_report *report;
	struct cyttsp5_hid_field *field;
	int i;
	u32 tch_collection_usage_page = HID_CY_TCH_COL_USAGE_PG;
	u32 btn_collection_usage_page = HID_CY_BTN_COL_USAGE_PG;

	for (i = CY_TCH_X; i < CY_TCH_NUM_ABS; i++) {
		field = find_report_desc_field(cd,
				cyttsp5_tch_abs_field_map[i],
				tch_collection_usage_page);
		if (field) {
			TS_LOG_DEBUG(
				" Field %p: rep_cnt:%d rep_sz:%d off:%d data:%02X min:%d max:%d usage_page:%08X\n",
				field, field->report_count, field->report_size,
				field->offset, field->data_type,
				field->logical_min, field->logical_max,
				field->usage_page);
			fill_tch_abs(&si->tch_abs[i], field);
			si->tch_abs[i].report = 1;
			TS_LOG_DEBUG( "%s: ofs:%d size:%d min:%d max:%d bofs:%d report:%d",
				cyttsp5_tch_abs_string[i],
				si->tch_abs[i].ofs, si->tch_abs[i].size,
				si->tch_abs[i].min, si->tch_abs[i].max,
				si->tch_abs[i].bofs, si->tch_abs[i].report);

		} else {
			si->tch_abs[i].report = 0;
		}
	}
	for (i = CY_TCH_TIME; i < CY_TCH_NUM_HDR; i++) {
		field = find_report_desc_field(cd,
				cyttsp5_tch_hdr_field_map[i],
				tch_collection_usage_page);
		if (field) {
			TS_LOG_DEBUG(
				" Field %p: rep_cnt:%d rep_sz:%d off:%d data:%02X min:%d max:%d usage_page:%08X\n",
				field, field->report_count, field->report_size,
				field->offset, field->data_type,
				field->logical_min, field->logical_max,
				field->usage_page);
			fill_tch_abs(&si->tch_hdr[i], field);
			si->tch_hdr[i].report = 1;
			TS_LOG_DEBUG( "%s: ofs:%d size:%d min:%d max:%d bofs:%d report:%d",
				cyttsp5_tch_hdr_string[i],
				si->tch_abs[i].ofs, si->tch_abs[i].size,
				si->tch_abs[i].min, si->tch_abs[i].max,
				si->tch_abs[i].bofs, si->tch_abs[i].report);

		} else {
			si->tch_hdr[i].report = 0;
		}
	}

	report = find_report_desc(cd, tch_collection_usage_page);
	if (report) {
		si->desc.tch_report_id = report->id;
		si->desc.tch_record_size = report->record_size / 8;
		si->desc.tch_header_size = (report->header_size / 8) + 3;
	} else {
		si->desc.tch_report_id = HID_TOUCH_REPORT_ID;
		si->desc.tch_record_size = TOUCH_REPORT_SIZE;
		si->desc.tch_header_size = TOUCH_INPUT_HEADER_SIZE;
	}

	report = find_report_desc(cd, btn_collection_usage_page);
	if (report)
		si->desc.btn_report_id = report->id;
	else
		si->desc.tch_report_id = HID_BTN_REPORT_ID;

	return 0;
}

static int cyttsp5_get_report_descriptor_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_core *core = cd->core;
	u8 cmd[2];
	int rc;
	int t;

	/* Read report descriptor length and version */
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 1;
	mutex_unlock(&cd->system_lock);

	/* Set report descriptor register */
	memcpy(cmd, &cd->hid_desc.report_desc_register,
		sizeof(cd->hid_desc.report_desc_register));

	rc = cyttsp5_adap_write_read_specific(core->adap, 2,
			cmd, NULL);
	if (rc) {
		TS_LOG_ERR( "%s: failed to get HID descriptor length and version, rc=%d\n",
			__func__, rc);
		goto error;
	}

	t = wait_event_timeout(cd->wait_q, (cd->hid_cmd_state == 0),
		msecs_to_jiffies(CY_HID_GET_REPORT_DESCRIPTOR_TIMEOUT));
	if (IS_TMO(t)) {
		TS_LOG_ERR( "%s: HID get descriptor timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	cyttsp5_pr_buf(cd->dev, cd->pr_buf, cd->response_buf,
		cd->hid_core.hid_report_desc_len,
		"Report Desc");

	rc = parse_report_descriptor(cd, cd->response_buf + 3,
		get_unaligned_le16(&cd->response_buf[0]) - 3);
	if (rc) {
		TS_LOG_ERR( "%s: Error parsing report descriptor r=%d\n",
			__func__, rc);
	}

	TS_LOG_DEBUG( "%s: %d reports found in descriptor\n", __func__,
		cd->num_hid_reports);

	for (t = 0; t < cd->num_hid_reports; t++) {
		struct cyttsp5_hid_report *report = cd->hid_reports[t];
		int j;

		TS_LOG_DEBUG(
			"Report %d: type:%d id:%02X size:%d fields:%d rec_fld_index:%d hdr_sz:%d rec_sz:%d usage_page:%08X\n",
			t, report->type, report->id,
			report->size, report->num_fields,
			report->record_field_index, report->header_size,
			report->record_size, report->usage_page);

		for (j = 0; j < report->num_fields; j++) {
			struct cyttsp5_hid_field *field = report->fields[j];

			TS_LOG_DEBUG(
				" Field %d: rep_cnt:%d rep_sz:%d off:%d data:%02X min:%d max:%d usage_page:%08X\n",
				j, field->report_count, field->report_size,
				field->offset, field->data_type,
				field->logical_min, field->logical_max,
				field->usage_page);

			TS_LOG_DEBUG( "  Collections Phys:%08X App:%08X Log:%08X\n",
				field->collection_usage_pages
					[HID_COLLECTION_PHYSICAL],
				field->collection_usage_pages
					[HID_COLLECTION_APPLICATION],
				field->collection_usage_pages
					[HID_COLLECTION_LOGICAL]);
		}
	}

	rc = setup_report_descriptor(cd);

	/* Free it for now */
	cyttsp5_free_hid_reports_(cd);

	TS_LOG_DEBUG( "%s: %d reports found in descriptor\n", __func__,
		cd->num_hid_reports);

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
exit:
	return rc;
}

static int cyttsp5_get_mode(struct cyttsp5_core_data *cd,
		struct cyttsp5_hid_desc *desc)
{
	if (CY_HID_APP_REPORT_ID == desc->packet_id)
		return CY_MODE_OPERATIONAL;
	else if (CY_HID_BL_REPORT_ID == desc->packet_id)
		return CY_MODE_BOOTLOADER;

	return CY_MODE_UNKNOWN;
}

static int _cyttsp5_request_get_mode(struct cyttsp5_device *ttsp,
		int protect, u8 *mode)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);
	int rc;

	if (protect)
		rc = cyttsp5_get_hid_descriptor(cd, &cd->hid_desc);
	else
		rc = cyttsp5_get_hid_descriptor_(cd, &cd->hid_desc);

	if (rc)
		*mode = CY_MODE_UNKNOWN;
	else
		*mode = cyttsp5_get_mode(cd, &cd->hid_desc);

	return rc;
}

static void cyttsp5_queue_startup_(struct cyttsp5_core_data *cd)
{
	if (cd->startup_state == STARTUP_NONE) {
		cd->startup_state = STARTUP_QUEUED;
		schedule_work(&cd->startup_work);
		TS_LOG_INFO( "%s: cyttsp5_startup queued\n", __func__);
	} else {
		TS_LOG_DEBUG( "%s: startup_state = %d\n", __func__,
			cd->startup_state);
	}
}

static void cyttsp5_queue_startup(struct cyttsp5_core_data *cd)
{
	mutex_lock(&cd->system_lock);
	cyttsp5_queue_startup_(cd);
	mutex_unlock(&cd->system_lock);
}

static void call_atten_cb(struct cyttsp5_core_data *cd,
		enum cyttsp5_atten_type type, int mode)
{
	struct atten_node *atten, *atten_n;

	TS_LOG_DEBUG( "%s: check list type=%d mode=%d\n",
		__func__, type, mode);
	spin_lock(&cd->spinlock);
	list_for_each_entry_safe(atten, atten_n,
			&cd->atten_list[type], node) {
		if (!mode || atten->mode & mode) {
			spin_unlock(&cd->spinlock);
			TS_LOG_DEBUG( "%s: attention for '%s'", __func__,
				dev_name(&atten->ttsp->dev));
			atten->func(atten->ttsp);
			spin_lock(&cd->spinlock);
		}
	}
	spin_unlock(&cd->spinlock);
}

static void cyttsp5_start_wd_timer(struct cyttsp5_core_data *cd)
{
	if (!CY_WATCHDOG_TIMEOUT)
		return;

	mod_timer(&cd->watchdog_timer, jiffies +
			msecs_to_jiffies(CY_WATCHDOG_TIMEOUT));
	return;
}

static void cyttsp5_stop_wd_timer(struct cyttsp5_core_data *cd)
{
	if (!CY_WATCHDOG_TIMEOUT)
		return;

	/*
	 * Ensure we wait until the watchdog timer
	 * running on a different CPU finishes
	 */
	del_timer_sync(&cd->watchdog_timer);
	cancel_work_sync(&cd->watchdog_work);
	del_timer_sync(&cd->watchdog_timer);
	return;
}

static void cyttsp5_watchdog_work(struct work_struct *work)
{
	struct cyttsp5_core_data *cd =
			container_of(work, struct cyttsp5_core_data,
					watchdog_work);
	int rc;
    //static int ping_count = 0;
    
	rc = request_exclusive(cd, cd->core,
			CY_WATCHDOG_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		goto queue_startup;
	}

	rc = cyttsp5_hid_output_null_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);

queue_startup:
	if (rc) {
		TS_LOG_ERR(
			"%s: failed to access device in watchdog timer r=%d\n",
			__func__, rc);
      	/*  
        if(ping_count++ > 3){
            return;
        }
        */
		cyttsp5_queue_startup(cd);
		return;
	}
    
    //ping_count = 0;

	cyttsp5_start_wd_timer(cd);
}

static void cyttsp5_watchdog_timer(unsigned long handle)
{
	struct cyttsp5_core_data *cd = (struct cyttsp5_core_data *)handle;

	if (!cd){
        printk(KERN_ERR "%s: cd is NULL.\n", __func__);
		return;
	}

	TS_LOG_DEBUG( "%s: Watchdog timer triggered\n", __func__);

	if (!work_pending(&cd->watchdog_work))
		schedule_work(&cd->watchdog_work);
}

static int cyttsp5_put_device_into_easy_wakeup_(struct cyttsp5_core_data *cd)
{
	int rc;
	u8 status = 0;

	rc = cyttsp5_hid_output_enter_easywake_state_(cd,
			cd->easy_wakeup_gesture, &status);
	if (rc || status == 0) {
		TS_LOG_ERR( "%s: fail set devices into easy_wakeup mode,rc = %d,status = %d.\n",
              	__func__, rc,status);
		return -EBUSY;
	}

	return rc;
}

static int cyttsp5_put_device_into_deep_sleep_(struct cyttsp5_core_data *cd)
{
	int rc;
	rc = cyttsp5_hid_cmd_set_power_(cd, HID_POWER_SLEEP);
	if (rc)
		rc = -EBUSY;
	return rc;
}

static int cyttsp5_put_device_into_sleep_(struct cyttsp5_core_data *cd)
{
	int rc;
	if ((!cd->wakeup_system_enabled) || (IS_DEEP_SLEEP_CONFIGURED(cd->easy_wakeup_gesture))) {
		rc = cyttsp5_put_device_into_deep_sleep_(cd);
	} else {
		if (cd->proximity_flag) {
			rc = cyttsp5_put_device_into_deep_sleep_(cd);
		} else {
			rc = cyttsp5_put_device_into_easy_wakeup_(cd);
		}
	}
	return rc;
}

static int cyttsp5_core_sleep_(struct cyttsp5_core_data *cd)
{
	int rc;
	cyttsp5_stop_wd_timer(cd);
	rc = cyttsp5_put_device_into_sleep_(cd);

	mutex_lock(&cd->system_lock);
	cd->sleep_state = SS_SLEEP_ON;
	mutex_unlock(&cd->system_lock);

	return rc;
}

static int cyttsp5_core_sleep(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		return 0;
	}

	rc = cyttsp5_core_sleep_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);
	else
		TS_LOG_DEBUG( "%s: pass release exclusive\n", __func__);

	return rc;
}

static int cyttsp5_wakeup_host(struct cyttsp5_core_data *cd)
{
	int rc = 0;
	int event_id;
	int size = get_unaligned_le16(&cd->input_buf[0]);

	/* Validate report */
	if (size != 4 || cd->input_buf[2] != 4){
        TS_LOG_INFO( "%s: size=%d, cd->input_buf[2]=%d\n", __func__, size, cd->input_buf[2]);
		rc = -EINVAL;
	}

	cd->wake_initiated_by_device = 1;
	event_id = cd->input_buf[3];

	TS_LOG_INFO( "%s: event_id=%d, rc=%d\n", __func__, event_id, rc);

	if (rc) {
		cyttsp5_core_sleep_(cd);
        TS_LOG_ERR( "%s: event_id=%d, rc=%d,exit here.\n", __func__, event_id, rc);
		goto exit;
	}

	/* attention WAKE */
	call_atten_cb(cd, CY_ATTEN_WAKE, 0);
exit:
	return rc;
}

static void cyttsp5_get_touch_axis(struct cyttsp5_core_data *cd,
	int *axis, int size, int max, u8 *data, int bofs)
{
	int nbyte;
	int next;

	for (nbyte = 0, *axis = 0, next = 0; nbyte < size; nbyte++) {
		*axis = *axis + ((data[next] >> bofs) << (nbyte * 8));
		next++;
	}

	*axis &= max - 1;
}

static int move_tracking_hetmap_data(struct cyttsp5_core_data *cd,
	struct cyttsp5_sysinfo *si)
{
#ifdef TTHE_TUNER_SUPPORT
	int size = get_unaligned_le16(&cd->input_buf[0]);
	if (size)
		tthe_print(cd, cd->input_buf, size, "tracking_heatmap=");
#endif
	memcpy(si->xy_mode, cd->input_buf, SENSOR_HEADER_SIZE);
	return 0;
}

static int move_sensor_data(struct cyttsp5_core_data *cd,
	struct cyttsp5_sysinfo *si)
{
#ifdef TTHE_TUNER_SUPPORT
	int size = get_unaligned_le16(&cd->input_buf[0]);
	if (size)
		tthe_print(cd, cd->input_buf, size, "sensor_monitor=");
#endif
	memcpy(si->xy_mode, cd->input_buf, SENSOR_HEADER_SIZE);
	return 0;
}

static int move_button_data(struct cyttsp5_core_data *cd,
	struct cyttsp5_sysinfo *si)
{
#ifdef TTHE_TUNER_SUPPORT
	int size = get_unaligned_le16(&cd->input_buf[0]);
	if (size)
		tthe_print(cd, cd->input_buf, size, "OpModeData=");
#endif
	memcpy(si->xy_mode, cd->input_buf, BTN_INPUT_HEADER_SIZE);
	cyttsp5_pr_buf(cd->dev, cd->pr_buf, (u8 *)si->xy_mode,
			BTN_INPUT_HEADER_SIZE, "xy_mode");

	memcpy(si->xy_data, &cd->input_buf[BTN_INPUT_HEADER_SIZE],
			BTN_REPORT_SIZE);
	cyttsp5_pr_buf(cd->dev, cd->pr_buf, (u8 *)si->xy_data,
			BTN_REPORT_SIZE, "xy_data");
	return 0;
}

static int move_touch_data(struct cyttsp5_core_data *cd,
	struct cyttsp5_sysinfo *si)
{
	int num_cur_tch;
	int length;
	struct cyttsp5_tch_abs_params *tch = &si->tch_hdr[CY_TCH_NUM];
#ifdef TTHE_TUNER_SUPPORT
	int size = get_unaligned_le16(&cd->input_buf[0]);
	if (size)
		tthe_print(cd, cd->input_buf, size, "OpModeData=");
#endif

	memcpy(si->xy_mode, cd->input_buf, si->desc.tch_header_size);
	cyttsp5_pr_buf(cd->dev, cd->pr_buf, (u8 *)si->xy_mode,
			si->desc.tch_header_size, "xy_mode");

	cyttsp5_get_touch_axis(cd, &num_cur_tch, tch->size,
			tch->max, si->xy_mode + 3 + tch->ofs, tch->bofs);
	if (unlikely(num_cur_tch > MAX_TOUCH_NUMBER))
		num_cur_tch = MAX_TOUCH_NUMBER;

	length = num_cur_tch * si->desc.tch_record_size;

	memcpy(si->xy_data, &cd->input_buf[si->desc.tch_header_size], length);
	cyttsp5_pr_buf(cd->dev, cd->pr_buf, (u8 *)si->xy_data, length,
			"xy_data");
	return 0;
}

static int parse_touch_input(struct cyttsp5_core_data *cd, int size)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int report_id = cd->input_buf[2];
	int rc = -EINVAL;

	TS_LOG_DEBUG( "%s: Received touch report\n", __func__);
	if (!si->ready) {
		TS_LOG_ERR(
			"%s: Need system information to parse touches\n",
			__func__);
		return 0;
	}

	if (!si->xy_mode || !si->xy_data)
		return rc;

	if (report_id == si->desc.tch_report_id)
		rc = move_touch_data(cd, si);
	else if (report_id == si->desc.btn_report_id)
		rc = move_button_data(cd, si);
	else if (report_id == HID_SENSOR_DATA_REPORT_ID)
		rc = move_sensor_data(cd, si);
	else if (report_id == HID_TRACKING_HEATMAP_REPOR_ID)
		rc = move_tracking_hetmap_data(cd, si);

	if (rc)
		return rc;

	/* attention IRQ */
	call_atten_cb(cd, CY_ATTEN_IRQ, cd->mode);

	return 0;
}

static int parse_command_input(struct cyttsp5_core_data *cd, int size)
{
	TS_LOG_DEBUG( "%s: Received cmd interrupt\n", __func__);

	memcpy(cd->response_buf, cd->input_buf, size);

	mutex_lock(&cd->system_lock);
	cd->hid_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
	wake_up(&cd->wait_q);

	return 0;
}

static int cyttsp5_parse_input(struct cyttsp5_core_data *cd)
{
	int report_id;
	int is_command = 0;
	int size;
    //u32 size;

	size = get_unaligned_le32(&cd->input_buf[0]);
    TS_LOG_DEBUG( "%s: 32 size:%X\n", __func__, size);
    size = get_unaligned_le16(&cd->input_buf[0]);
    TS_LOG_DEBUG( "%s: 16 size:%X\n", __func__, size);

	/* check reset */
	if (size == 0) {
		TS_LOG_DEBUG( "%s: Reset complete\n", __func__);
		mutex_lock(&cd->system_lock);
		if (!cd->hid_reset_cmd_state && !cd->hid_cmd_state) {
			mutex_unlock(&cd->system_lock);
			TS_LOG_DEBUG( "%s: Device Initiated Reset\n",
					__func__);
			return 0;
		} else {
			cd->hid_reset_cmd_state = 0;
			if (cd->hid_cmd_state == HID_OUTPUT_START_BOOTLOADER + 1
				|| cd->hid_cmd_state ==
					HID_OUTPUT_BL_LAUNCH_APP + 1
				|| cd->hid_cmd_state ==
					HID_OUTPUT_USER_CMD + 1)
				cd->hid_cmd_state = 0;
			wake_up(&cd->wait_q);
		}
		mutex_unlock(&cd->system_lock);
		return 0;
	}
	
	report_id = cd->input_buf[2];
	TS_LOG_DEBUG( "%s: report_id:%X\n", __func__, report_id);

	/* Check wake-up report */
	if (report_id == HID_WAKEUP_REPORT_ID && cd->wakeup_system_enabled) {
        TS_LOG_INFO( "%s: report_id:%X\n", __func__, report_id);
		cyttsp5_wakeup_host(cd);
		return 0;
	}

	/* update watchdog expire time */
	mod_timer_pending(&cd->watchdog_timer, jiffies +
			msecs_to_jiffies(CY_WATCHDOG_TIMEOUT));

	if (report_id != cd->sysinfo.desc.tch_report_id
			&& report_id != cd->sysinfo.desc.btn_report_id
			&& report_id != HID_SENSOR_DATA_REPORT_ID
			&& report_id != HID_TRACKING_HEATMAP_REPOR_ID)
		is_command = 1;

	if (unlikely(is_command)) {
		parse_command_input(cd, size);
		return 0;
	}
	parse_touch_input(cd, size);
	return 0;
}

static int cyttsp5_read_input(struct cyttsp5_core_data *cd)
{
	int rc;

	rc = cyttsp5_adap_read_default_nosize(cd->core->adap, cd->input_buf,
			CY_MAX_INPUT);
	if (rc) {
		TS_LOG_ERR( "%s: Error getting report, r=%d\n", __func__, rc);
		return rc;
	}
	TS_LOG_DEBUG( "%s: Read input successfully\n", __func__);
	return rc;
}

static bool cyttsp5_check_irq_asserted(struct cyttsp5_core_data *cd)
{
#ifdef ENABLE_WORKAROUND_FOR_GLITCH_AFTER_BL_LAUNCH_APP
	/* Workaround for FW defect, CDT165308
	* bl_launch app creates a glitch in IRQ line */
	if (cd->hid_cmd_state == HID_OUTPUT_BL_LAUNCH_APP + 1
		&& cd->pdata->irq_stat) {
		/*
		in X1S panel and GC1546 panel, the width for the INT 
		glitch is about 4us,the normal INT width of response
		will last more than 200us, so use 10us delay
		for distinguish the glitch the normal INT is enough.
		*/
		udelay(20);
		if (cd->pdata->irq_stat(cd->pdata, cd->dev)
			!= CY_IRQ_ASSERTED_VALUE)
			return false;
	}
#endif
	return true;
}

static irqreturn_t cyttsp5_irq(int irq, void *handle)
{
	struct cyttsp5_core_data *cd = handle;
	int rc;

	TS_LOG_DEBUG("Enter %s\n", __func__);
	if (!cyttsp5_check_irq_asserted(cd))
		return IRQ_HANDLED;

	rc = cyttsp5_read_input(cd);
	if (!rc)
		cyttsp5_parse_input(cd);

	TS_LOG_DEBUG("Exit %s\n", __func__);
	return IRQ_HANDLED;
}

static int _cyttsp5_subscribe_attention(struct cyttsp5_device *ttsp,
	enum cyttsp5_atten_type type,
	int (*func)(struct cyttsp5_device *), int mode)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);
	struct atten_node *atten, *atten_new;

	atten_new = kzalloc(sizeof(*atten_new), GFP_KERNEL);
	if (!atten_new) {
		TS_LOG_ERR( "%s: Fail alloc atten node\n", __func__);
		return -ENOMEM;
	}

	TS_LOG_DEBUG( "%s from '%s'\n", __func__, dev_name(cd->dev));

	spin_lock(&cd->spinlock);
	list_for_each_entry(atten, &cd->atten_list[type], node) {
		if (atten->ttsp == ttsp && atten->mode == mode) {
			spin_unlock(&cd->spinlock);
			kfree(atten_new);
			TS_LOG_DEBUG( "%s: %s=%p %s=%d\n",
				 __func__,
				 "already subscribed attention",
				 ttsp, "mode", mode);

			return 0;
		}
	}

	atten_new->ttsp = ttsp;
	atten_new->mode = mode;
	atten_new->func = func;

	list_add(&atten_new->node, &cd->atten_list[type]);
	spin_unlock(&cd->spinlock);

	return 0;
}

static int _cyttsp5_unsubscribe_attention(struct cyttsp5_device *ttsp,
	enum cyttsp5_atten_type type, int (*func)(struct cyttsp5_device *),
	int mode)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);
	struct atten_node *atten, *atten_n;

	spin_lock(&cd->spinlock);
	list_for_each_entry_safe(atten, atten_n, &cd->atten_list[type], node) {
		if (atten->ttsp == ttsp && atten->mode == mode) {
			list_del(&atten->node);
			spin_unlock(&cd->spinlock);
			kfree(atten);
			TS_LOG_DEBUG( "%s: %s=%p %s=%d\n",
				__func__,
				"unsub for atten->ttsp", atten->ttsp,
				"atten->mode", atten->mode);
			return 0;
		}
	}
	spin_unlock(&cd->spinlock);

	return -ENODEV;
}
static int cyttsp5_hid_output_read_conf_ver_(struct cyttsp5_core_data *cd,
					u16 row_number, u16 length, u8 ebid, u16 *config_ver)
{
	int read_ebid;
	int read_length;
	int status;
	int rc;
	int write_length = 5;
	u8 write_buf[5];
	u8 cmd_offset = 0;
	struct cyttsp5_hid_output hid_output = {
			HID_OUTPUT_APP_COMMAND(HID_OUTPUT_READ_CONF_BLOCK),
			.write_length = write_length,
			.write_buf = write_buf,
		};
	write_buf[cmd_offset++] = LOW_BYTE(row_number);
	write_buf[cmd_offset++] = HI_BYTE(row_number);
	write_buf[cmd_offset++] = LOW_BYTE(length);
	write_buf[cmd_offset++] = HI_BYTE(length);
	write_buf[cmd_offset++] = ebid;
	rc = cyttsp5_hid_send_output_and_wait_(cd, &hid_output);
	if (rc)
		return rc;
	status = cd->response_buf[5];
	if (status)
		return -EINVAL;
	read_ebid = cd->response_buf[6];
	if ((read_ebid != ebid) || (cd->response_buf[9] != 0))
		return -EPROTO;
	read_length = get_unaligned_le16(&cd->response_buf[7]);
	if (length < read_length)
		length = read_length;
	*config_ver = get_unaligned_le16(&cd->response_buf[18]);
	return 0;
}
static int cyttsp5_get_config_ver_(struct cyttsp5_core_data *cd)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int rc;
	u16 config_ver;

	rc = cyttsp5_hid_output_suspend_scanning_(cd);
	if (rc) {
		TS_LOG_DEBUG( "%s: cyttsp5_hid_output_suspend_scanning_ ERROR.\n",	__func__);
		goto error;
	}
	rc = cyttsp5_hid_output_read_conf_ver_(cd, 0, 10, 0, &config_ver);
	if (rc)
		goto exit;
	si->cydata.fw_ver_conf= config_ver;
exit:
	cyttsp5_hid_output_resume_scanning_(cd);
error:
	return rc;
}


static int _cyttsp5_request_exclusive(struct cyttsp5_device *ttsp,
		int timeout_ms)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);
	return request_exclusive(cd, (void *)ttsp, timeout_ms);
}

static int _cyttsp5_release_exclusive(struct cyttsp5_device *ttsp)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);
	return release_exclusive(cd, (void *)ttsp);
}

static int cyttsp5_reset(struct cyttsp5_core_data *cd)
{
	int rc;

	/* reset hardware */
	TS_LOG_DEBUG( "%s: reset hw...\n", __func__);
	rc = cyttsp5_hw_reset(cd);
	if (rc < 0)
		TS_LOG_ERR( "%s: %s adap='%s' r=%d\n", __func__,
			"Fail hw reset", cd->core->adap->id, rc);
	return rc;
}

static int cyttsp5_reset_and_wait(struct cyttsp5_core_data *cd)
{
	int rc;
	int t;

	mutex_lock(&cd->system_lock);
	cd->hid_reset_cmd_state = 1;
	mutex_unlock(&cd->system_lock);

	rc = cyttsp5_reset(cd);
	if (rc < 0)
		goto error;

	t = wait_event_timeout(cd->wait_q, (cd->hid_reset_cmd_state == 0),
			msecs_to_jiffies(CY_HID_RESET_TIMEOUT));
	if (IS_TMO(t)) {
		TS_LOG_ERR( "%s: reset timed out\n",
			__func__);
		rc = -ETIME;
		goto error;
	}

	goto exit;

error:
	mutex_lock(&cd->system_lock);
	cd->hid_reset_cmd_state = 0;
	mutex_unlock(&cd->system_lock);
exit:
	return rc;
}

/*
 * returns err if refused or timeout(core uses fixed timeout period) occurs;
 * blocks until ISR occurs
 */
static int _cyttsp5_request_reset(struct cyttsp5_device *ttsp)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);
	int rc;

	mutex_lock(&cd->system_lock);
	cd->hid_reset_cmd_state = 1;
	mutex_unlock(&cd->system_lock);

	rc = cyttsp5_reset(cd);
	if (rc < 0) {
		TS_LOG_ERR( "%s: Error on h/w reset r=%d\n",
			__func__, rc);
		mutex_lock(&cd->system_lock);
		cd->hid_reset_cmd_state = 0;
		mutex_unlock(&cd->system_lock);
	}

	return rc;
}

/*
 * returns err if refused ; if no error then restart has completed
 * and system is in normal operating mode
 */
static int _cyttsp5_request_restart(struct cyttsp5_device *ttsp)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	cyttsp5_queue_startup(cd);

	return 0;
}

/*
 * returns NULL if sysinfo has not been acquired from the device yet
 */
static struct cyttsp5_sysinfo *_cyttsp5_request_sysinfo(
		struct cyttsp5_device *ttsp)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);

	if (cd->sysinfo.ready)
		return &cd->sysinfo;

	TS_LOG_ERR( "%s: sysinfo is NULL.\n", __func__);
	return NULL;
}

static struct cyttsp5_loader_platform_data *_cyttsp5_request_loader_pdata(
		struct cyttsp5_device *ttsp)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);
	return cd->pdata->loader_pdata;
}

static int _cyttsp5_request_stop_wd(struct cyttsp5_device *ttsp)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);
	cyttsp5_stop_wd_timer(cd);
	return 0;
}

static int _cyttsp5_request_enable_scan_type(struct cyttsp5_device *ttsp,
		u8 scan_type)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);
	cd = cd; /*0, to remove copiler warning */
	return 0;
}

static int _cyttsp5_request_disable_scan_type(struct cyttsp5_device *ttsp,
		u8 scan_type)
{
	struct cyttsp5_core *core = ttsp->core;
	struct cyttsp5_core_data *cd = dev_get_drvdata(&core->dev);
	cd = cd; /*0, to remove copiler warning */
	return 0;
}

static int cyttsp5_core_wake_device_from_deep_sleep_(
		struct cyttsp5_core_data *cd)
{
	int rc;
	rc = cyttsp5_hid_cmd_set_power_(cd, HID_POWER_ON);
	if (rc){
		TS_LOG_ERR( "%s: error,rc = %d\n", __func__,rc);
		rc =  -EAGAIN;
	}
	return rc;
}

static int cyttsp5_core_wake_device_(struct cyttsp5_core_data *cd)
{
	if (cd->wake_initiated_by_device) {
		cd->wake_initiated_by_device = 0;
		/* To prevent sequential wake/sleep caused by ttsp modules */
		msleep(30);
		return 0;
	}

	return cyttsp5_core_wake_device_from_deep_sleep_(cd);
}

static int cyttsp5_core_wake_(struct cyttsp5_core_data *cd)
{
	int rc;
	rc = cyttsp5_core_wake_device_(cd);

	mutex_lock(&cd->system_lock);
	cd->sleep_state = SS_SLEEP_OFF;
	mutex_unlock(&cd->system_lock);

	cyttsp5_start_wd_timer(cd);
	return rc;
}

static int cyttsp5_core_wake(struct cyttsp5_core_data *cd)
{
	int rc;

    rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
    if (rc < 0)
    {
        TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
                __func__, cd->exclusive_dev, cd->core);
        return 0;
    }

    rc = cyttsp5_core_wake_(cd);

    if (release_exclusive(cd, cd->core) < 0)
    { TS_LOG_ERR( "%s: fail to release exclusive\n", __func__); }
    else
    { TS_LOG_DEBUG( "%s: pass release exclusive\n", __func__); }

    return rc;
}

static int cyttsp5_get_ic_crc_(struct cyttsp5_core_data *cd, u8 ebid)
{
	struct cyttsp5_sysinfo *si = &cd->sysinfo;
	int rc;
	u8 status;
	u16 calculated_crc = 0;
	u16 stored_crc = 0;

	rc = cyttsp5_hid_output_suspend_scanning_(cd);
	if (rc){
		TS_LOG_ERR( "%s: 1 - error,rc = %d\n", __func__,rc);
		goto error;
	}

	rc = cyttsp5_hid_output_verify_config_block_crc_(cd, ebid, &status,
			&calculated_crc, &stored_crc);
	if (rc){
		TS_LOG_ERR( "%s: 2 - error,rc = %d\n", __func__,rc);
		goto exit;
	}

	if (status) {
		TS_LOG_ERR( "%s: 3 - error,status = %d\n", __func__,status);
		rc = -EINVAL;
		goto exit;
	}

	si->ttconfig.crc = stored_crc;

exit:
	cyttsp5_hid_output_resume_scanning_(cd);
error:
	TS_LOG_DEBUG( "%s: CRC: ebid:%d, crc:0x%04X\n",
			__func__, ebid, si->ttconfig.crc);
	return rc;
}

static int cyttsp5_check_and_deassert_int(struct cyttsp5_core_data *cd)
{
	u16 size;
	u8 buf[2];
	u8 *p;
	u8 retry = 3;
	int rc;

	do {
		if (cd->pdata->xres) {
			cd->pdata->xres(cd->pdata, cd->dev);
			mdelay(1000);
		}

		rc = cyttsp5_adap_read_default(cd->core->adap, buf, 2);
		if (rc < 0) {
			TS_LOG_ERR( "%s: Error on read_i2c r=%d\n",
				__func__, rc);
			return rc;
		}
		size = get_unaligned_le16(&buf[0]);

		if (size == 2 || size == 0)
			return 0;

		p = kzalloc(size, GFP_KERNEL);
		if (!p) {
			TS_LOG_ERR( "%s: Error on kzalloc size.\n",
				__func__);
			return -ENOMEM;
		}
		rc = cyttsp5_adap_read_default(cd->core->adap, p, size);
		kfree(p);
		if (rc < 0) {
			TS_LOG_ERR( "%s: Error on read_i2c again r=%d\n",
				__func__, rc);
			return rc;
		}
	} while (retry--);

	return 0;
}
extern struct cyttsp5_i2c *g_ts_i2c;

static int cyttsp5_power_on_try(struct cyttsp5_core_data *cd)
{
	int rc = 0;
	
	if(g_ts_i2c != NULL){
		rc = regulator_disable(g_ts_i2c->vdd);
		if (rc < 0) {
			TS_LOG_ERR("%s: Error,regulator_disable vdd,rc = %d.\n",__func__, rc);
		}
		gpio_direction_output(g_ts_i2c->vbus_gpio_en, 0);
		gpio_direction_output(g_ts_i2c->vbus_gpio, 0);
		mdelay(100);
		rc = regulator_enable(g_ts_i2c->vdd);
		if (rc < 0) {
			TS_LOG_ERR("%s: Error,regulator_enable vdd,rc = %d.\n",__func__, rc);
		}
		gpio_direction_output(g_ts_i2c->vbus_gpio_en, 1);
		gpio_direction_output(g_ts_i2c->vbus_gpio, 1);
		mdelay(200);
	}
	return rc;
}
#ifdef CONFIG_HUAWEI_KERNEL
static char * get_sensor_name(int id)
{
	switch (id) {
	case PANEL_ID_EELYEE:
		return "EELYEE";
	case PANEL_ID_MUTTO:
		return "Mutto";
	default:
		break;
	}

	return "unknow";
}
static int cyttsp5_set_appinfo(struct cyttsp5_core_data *cd)
{
	char touch_info[50] = { 0 };
	int panel_id = 0;
	int fw_ver = 0;
	int ret = 0;
	
	if (cd == NULL)
		return -EINVAL;

	panel_id = cd->sysinfo.sensing_conf_data.panel_id;
	fw_ver =  cd->sysinfo.cydata.fw_ver_conf;
	TS_LOG_INFO("%s: Sensor ID:%d; FW version:%d\n", __func__, panel_id, fw_ver);

	snprintf(touch_info,sizeof(touch_info),"%s_%s_%d\n",CYTTSP5_TOUCHINFO,
			get_sensor_name(panel_id), fw_ver);
	ret = app_info_set("touch_panel", touch_info);
	if (ret < 0) {
		TS_LOG_ERR("%s(line %d): error,ret=%d\n",__func__,__LINE__,ret);
		return ret;
	}

	return 0;
}
#endif
static int cyttsp5_startup_(struct cyttsp5_core_data *cd)
{
	int rc;
#ifdef TTHE_TUNER_SUPPORT
	tthe_print(cd, NULL, 0, "enter startup");
#endif

	cyttsp5_stop_wd_timer(cd);

	/* reset hardware */
	rc = cyttsp5_reset_and_wait(cd);
	if (rc < 0){
		TS_LOG_ERR( "%s: Error on h/w reset r=%d\n",__func__, rc);
		cyttsp5_power_on_try(cd);
		goto exit;
	}

	rc = cyttsp5_get_hid_descriptor_(cd, &cd->hid_desc);
	if (rc < 0) {
		TS_LOG_ERR( "%s: Error on getting HID descriptor r=%d\n",
			__func__, rc);
		goto exit;
	}
	cd->mode = cyttsp5_get_mode(cd, &cd->hid_desc);

	if (cd->mode == CY_MODE_BOOTLOADER) {
		TS_LOG_INFO( "%s: Bootloader mode\n", __func__);
		rc = cyttsp5_hid_output_bl_launch_app_(cd);
		if (rc < 0) {
			TS_LOG_ERR( "%s: Error on launch app r=%d\n",
				__func__, rc);
			goto exit;
		}
		rc = cyttsp5_get_hid_descriptor_(cd, &cd->hid_desc);
		if (rc < 0) {
			TS_LOG_ERR(
				"%s: Error on getting HID descriptor r=%d\n",
				__func__, rc);
			goto exit;
		}
		cd->mode = cyttsp5_get_mode(cd, &cd->hid_desc);
		if (cd->mode == CY_MODE_BOOTLOADER){
			TS_LOG_INFO("%s: Here Bootloader mode,then exit.\n",__func__);
			goto exit;
		}
	}

	mutex_lock(&cd->system_lock);
	/* Read descriptor lengths */
	cd->hid_core.hid_report_desc_len =
		le16_to_cpu(cd->hid_desc.report_desc_len);
	cd->hid_core.hid_max_input_len =
		le16_to_cpu(cd->hid_desc.max_input_len);
	cd->hid_core.hid_max_output_len =
		le16_to_cpu(cd->hid_desc.max_output_len);

	cd->mode = cyttsp5_get_mode(cd, &cd->hid_desc);
	if (cd->mode == CY_MODE_OPERATIONAL)
		TS_LOG_INFO( "%s: Operational mode\n", __func__);
	else if (cd->mode == CY_MODE_BOOTLOADER)
		TS_LOG_INFO( "%s: Bootloader mode\n", __func__);
	else if (cd->mode == CY_MODE_UNKNOWN) {
		TS_LOG_ERR( "%s: Unknown mode\n", __func__);
		rc = -ENODEV;
		mutex_unlock(&cd->system_lock);
		goto exit;
	}
	mutex_unlock(&cd->system_lock);

	TS_LOG_DEBUG( "%s: Reading report descriptor\n", __func__);
	rc = cyttsp5_get_report_descriptor_(cd);
	if (rc < 0) {
		TS_LOG_ERR( "%s: Error on getting report descriptor r=%d\n",
			__func__, rc);
		goto exit;
	}

	rc = cyttsp5_hid_output_get_sysinfo_(cd);
	if (rc) {
		TS_LOG_ERR( "%s: Error on getting sysinfo r=%d\n",
			__func__, rc);
		goto exit;
	}
	rc = cyttsp5_get_config_ver_(cd);
	if (rc)	{
		TS_LOG_ERR( "%s: failed to read config version rc=%d\n",__func__, rc);
	}

	rc = cyttsp5_get_ic_crc_(cd, CY_TCH_PARM_EBID);
	if (rc){
        cd->sysinfo.ready = false;
		TS_LOG_ERR( "%s: failed to crc data rc=%d\n",
			__func__, rc);
	}
	
#ifdef CONFIG_HUAWEI_KERNEL
	cyttsp5_set_appinfo(cd);
#endif
	/* attention startup */
	call_atten_cb(cd, CY_ATTEN_STARTUP, 0);

exit:
	cyttsp5_start_wd_timer(cd);

#ifdef TTHE_TUNER_SUPPORT
	tthe_print(cd, NULL, 0, "exit startup");
#endif

	return rc;
}

static int cyttsp5_startup(struct cyttsp5_core_data *cd)
{
	int rc;

	mutex_lock(&cd->system_lock);
	cd->startup_state = STARTUP_RUNNING;
	mutex_unlock(&cd->system_lock);

	rc = request_exclusive(cd, cd->core, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		TS_LOG_ERR( "%s: fail get exclusive ex=%p own=%p\n",
				__func__, cd->exclusive_dev, cd->core);
		goto exit;
	}

	rc = cyttsp5_startup_(cd);

	if (release_exclusive(cd, cd->core) < 0)
		/* Don't return fail code, mode is already changed. */
		TS_LOG_ERR( "%s: fail to release exclusive\n", __func__);
	else
		TS_LOG_DEBUG( "%s: pass release exclusive\n", __func__);

exit:
	mutex_lock(&cd->system_lock);
	cd->startup_state = STARTUP_NONE;
	mutex_unlock(&cd->system_lock);

	return rc;
}

static void cyttsp5_startup_work_function(struct work_struct *work)
{
	struct cyttsp5_core_data *cd =  container_of(work,
		struct cyttsp5_core_data, startup_work);
	int rc;

	rc = cyttsp5_startup(cd);
	if (rc < 0)
		TS_LOG_ERR( "%s: Fail queued startup r=%d\n",
			__func__, rc);
}
#ifdef CY_TOUCH_RUNTIME_NOTUSED
static int sleep_error_flag = 0;
int cyttsp5_core_early_suspend()
{
	struct cyttsp5_core_data *cd = globe_cd;
	int rc;
	TS_LOG_INFO( "Enter %s\n", __func__);

	rc = cyttsp5_core_sleep(cd);
	if (rc < 0) {
		TS_LOG_ERR( "%s: Error on sleep\n", __func__);
		sleep_error_flag = 1;
		return -EAGAIN;
	}
	return 0;
}
int cyttsp5_core_late_rusume()
{
	struct cyttsp5_core_data *cd = globe_cd;
	int rc;

	TS_LOG_INFO( "Enter %s\n", __func__);

	if(1 == sleep_error_flag) {
		sleep_error_flag = 0;

		mutex_lock(&cd->system_lock);
		cyttsp5_hw_hard_reset(cd);
		mutex_unlock(&cd->system_lock);
	}

	cd->proximity_flag = 0;//clear proximity flag

	rc = cyttsp5_core_wake(cd);
	if (rc < 0) {
		TS_LOG_ERR( "%s: Error on wake\n", __func__);
		return -EAGAIN;
	}

	return 0;
}
#endif

#ifndef CY_TOUCH_RUNTIME_NOTUSED
#if defined(CONFIG_PM_RUNTIME)
static int cyttsp5_core_rt_suspend(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rc;

	rc = cyttsp5_core_sleep(cd);
	if (rc < 0) {
		TS_LOG_ERR( "%s: Error on sleep\n", __func__);
		return -EAGAIN;
	}
	return 0;
}

static int cyttsp5_core_rt_resume(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rc;

	rc = cyttsp5_core_wake(cd);
	if (rc < 0) {
		TS_LOG_ERR( "%s: Error on wake\n", __func__);
		return -EAGAIN;
	}

	return 0;
}
#endif

#if defined(CONFIG_PM_SLEEP)
static int cyttsp5_core_suspend(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if ((cd->pdata->flags & CY_CORE_FLAG_WAKE_ON_GESTURE))
		return 0;

	/*
	 * This will not prevent resume
	 * Required to prevent interrupts before i2c awake
	 */
	disable_irq(cd->irq);

	if (device_may_wakeup(dev)) {
		TS_LOG_DEBUG( "%s Device MAY wakeup\n", __func__);
		if (!enable_irq_wake(cd->irq))
			cd->irq_wake = 1;
	} else {
		TS_LOG_DEBUG( "%s Device NOT wakeup\n", __func__);
	}

	return 0;
}

static int cyttsp5_core_resume(struct device *dev)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if ((cd->pdata->flags & CY_CORE_FLAG_WAKE_ON_GESTURE))
		return 0;

	enable_irq(cd->irq);

	if (device_may_wakeup(dev)) {
		TS_LOG_DEBUG( "%s Device MAY wakeup\n", __func__);
		if (cd->irq_wake) {
			disable_irq_wake(cd->irq);
			cd->irq_wake = 0;
		}
	} else {
		TS_LOG_DEBUG( "%s Device NOT wakeup\n", __func__);
	}

	return 0;
}
#endif

static const struct dev_pm_ops cyttsp5_core_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cyttsp5_core_suspend, cyttsp5_core_resume)
	SET_RUNTIME_PM_OPS(cyttsp5_core_rt_suspend, cyttsp5_core_rt_resume,
			NULL)
};
#endif
/*
 * Show Firmware version via sysfs
 */
static ssize_t cyttsp5_ic_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	struct cyttsp5_cydata *cydata = &cd->sysinfo.cydata;

	return sprintf(buf,
		"%s: 0x%02X\n"
		"%s: 0x%02X\n"
		"%s: 0x%08X\n"
		"%s: 0x%04X\n"
		"%s: 0x%02X\n"
		"%s: 0x%02X\n",
		"Firmware Major Version", cydata->fw_ver_major,
		"Firmware Minor Version", cydata->fw_ver_minor,
		"Revision Control Number", cydata->revctrl,
		"Firmware Configuration Version", cydata->fw_ver_conf,
		"Bootloader Major Version", cydata->bl_ver_major,
		"Bootloader Minor Version", cydata->bl_ver_minor);
}

/*
 * Show Driver version via sysfs
 */
static ssize_t cyttsp5_drv_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Driver: %s\nVersion: %s\nDate: %s\n",
		cy_driver_core_name, cy_driver_core_version,
		cy_driver_core_date);
}

/*
 * HW reset via sysfs
 */
static ssize_t cyttsp5_hw_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int rc;

	rc = cyttsp5_startup(cd);
	if (rc < 0)
		TS_LOG_ERR( "%s: HW reset failed r=%d\n",
			__func__, rc);

	return size;
}

/*
 * Show IRQ status via sysfs
 */
static ssize_t cyttsp5_hw_irq_stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	int retval;

	if (cd->pdata->irq_stat) {
		retval = cd->pdata->irq_stat(cd->pdata, dev);
		switch (retval) {
		case 0:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is LOW.\n");
		case 1:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Interrupt line is HIGH.\n");
		default:
			return snprintf(buf, CY_MAX_PRBUF_SIZE,
				"Function irq_stat() returned %d.\n", retval);
		}
	}

	return snprintf(buf, CY_MAX_PRBUF_SIZE,
		"Function irq_stat() undefined.\n");
}

/*
 * Show IRQ enable/disable status via sysfs
 */
static ssize_t cyttsp5_drv_irq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	if (cd->irq_enabled)
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Driver interrupt is ENABLED\n");
	else
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE,
			"Driver interrupt is DISABLED\n");
	mutex_unlock(&cd->system_lock);

	return ret;
}

/*
 * Enable/disable IRQ via sysfs
 */
static ssize_t cyttsp5_drv_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int retval = 0;

	retval = kstrtoul(buf, 10, &value);
	if (retval < 0) {
		TS_LOG_ERR( "%s: Invalid value\n", __func__);
		goto cyttsp5_drv_irq_store_error_exit;
	}

	mutex_lock(&cd->system_lock);
	switch (value) {
	case 0:
		if (cd->irq_enabled) {
			cd->irq_enabled = false;
			/* Disable IRQ */
			disable_irq_nosync(cd->irq);
			TS_LOG_INFO( "%s: Driver IRQ now disabled\n",
				__func__);
		} else
			TS_LOG_INFO( "%s: Driver IRQ already disabled\n",
				__func__);
		break;

	case 1:
		if (cd->irq_enabled == false) {
			cd->irq_enabled = true;
			/* Enable IRQ */
			enable_irq(cd->irq);
			TS_LOG_INFO( "%s: Driver IRQ now enabled\n",
				__func__);
		} else
			TS_LOG_INFO( "%s: Driver IRQ already enabled\n",
				__func__);
		break;

	default:
		TS_LOG_ERR( "%s: Invalid value\n", __func__);
	}
	mutex_unlock(&(cd->system_lock));

cyttsp5_drv_irq_store_error_exit:

	return size;
}

/*
 * Debugging options via sysfs
 */
static ssize_t cyttsp5_drv_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int rc;
	u8 return_data[8];

	rc = kstrtoul(buf, 10, &value);
	if (rc < 0) {
		TS_LOG_ERR( "%s: Invalid value\n", __func__);
		goto cyttsp5_drv_debug_store_exit;
	}

	switch (value) {
	case CY_DBG_SUSPEND:
		TS_LOG_INFO( "%s: SUSPEND (cd=%p)\n", __func__, cd);
		rc = cyttsp5_core_sleep(cd);
		if (rc)
			TS_LOG_ERR( "%s: Suspend failed rc=%d\n",
				__func__, rc);
		else
			TS_LOG_INFO( "%s: Suspend succeeded\n", __func__);
		break;

	case CY_DBG_RESUME:
		TS_LOG_INFO( "%s: RESUME (cd=%p)\n", __func__, cd);
		rc = cyttsp5_core_wake(cd);
		if (rc)
			TS_LOG_ERR( "%s: Resume failed rc=%d\n",
				__func__, rc);
		else
			TS_LOG_INFO( "%s: Resume succeeded\n", __func__);
		break;
	case CY_DBG_SOFT_RESET:
		TS_LOG_INFO( "%s: SOFT RESET (cd=%p)\n", __func__, cd);
		rc = cyttsp5_hw_soft_reset(cd);
		break;
	case CY_DBG_RESET:
		TS_LOG_INFO( "%s: HARD RESET (cd=%p)\n", __func__, cd);
		rc = cyttsp5_hw_hard_reset(cd);
		break;
	case CY_DBG_HID_RESET:
		TS_LOG_INFO( "%s: hid_reset (cd=%p)\n", __func__, cd);
		cyttsp5_hid_cmd_reset(cd);
		break;
	case CY_DBG_HID_SET_POWER_ON:
		TS_LOG_INFO( "%s: hid_set_power_on (cd=%p)\n", __func__, cd);
		cyttsp5_hid_cmd_set_power(cd, HID_POWER_ON);
		cyttsp5_start_wd_timer(cd);
		break;
	case CY_DBG_HID_SET_POWER_SLEEP:
		TS_LOG_INFO( "%s: hid_set_power_off (cd=%p)\n", __func__, cd);
		cyttsp5_stop_wd_timer(cd);
		cyttsp5_hid_cmd_set_power(cd, HID_POWER_SLEEP);
		break;
	case CY_DBG_HID_NULL:
		TS_LOG_INFO( "%s: hid_null (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_null(cd);
		break;
	case CY_DBG_HID_ENTER_BL:
		TS_LOG_INFO( "%s: start_bootloader (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_start_bootloader(cd);
		break;
	case CY_DBG_HID_SYSINFO:
		TS_LOG_INFO( "%s: get_sysinfo (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_get_sysinfo(cd);
		break;
	case CY_DBG_HID_SUSPEND_SCAN:
    case CY_DBG_HDMI_DISABLE:
		TS_LOG_INFO( "%s: suspend_scanning (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_suspend_scanning(cd);
		break;
	case CY_DBG_HID_RESUME_SCAN:
    case CY_DBG_HDMI_ENABLE:
		TS_LOG_INFO( "%s: resume_scanning (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_resume_scanning(cd);
		break;
	case CY_DBG_HID_STOP_WD:
		TS_LOG_INFO( "%s: stop watchdog (cd=%p)\n", __func__, cd);
		cyttsp5_stop_wd_timer(cd);
		break;
	case CY_DBG_HID_START_WD:
		TS_LOG_INFO( "%s: start watchdog (cd=%p)\n", __func__, cd);
		cyttsp5_start_wd_timer(cd);
		break;
	case HID_OUTPUT_BL_VERIFY_APP_INTEGRITY:
		TS_LOG_INFO( "%s: verify app integ (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_bl_verify_app_integrity(cd, &return_data[0]);
		break;
	case HID_OUTPUT_BL_APPEND_DATA_BUFF:
		TS_LOG_INFO( "%s: append data buffer (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_bl_append_data_buff(cd, 0, NULL);
		break;
	case HID_OUTPUT_BL_GET_INFO:
		TS_LOG_INFO( "%s: bl get info (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_bl_get_information(cd, return_data);
		break;
	case HID_OUTPUT_BL_PROGRAM_AND_VERIFY:
		TS_LOG_INFO( "%s: program and verify (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_bl_program_and_verify(cd, 0, NULL);
		break;
	case HID_OUTPUT_BL_LAUNCH_APP:
		TS_LOG_INFO( "%s: launch app (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_bl_launch_app(cd);
		break;
	case HID_OUTPUT_BL_INITIATE_BL:
		TS_LOG_INFO( "%s: initiate bl (cd=%p)\n", __func__, cd);
		cyttsp5_hid_output_bl_initiate_bl(cd, 0, NULL, 0, NULL);
		break;
    case CY_DBG_COVER_MODE_ON:
        TS_LOG_INFO( "%s: cover on.\n", __func__);
		cyttsp5_cover_mode_settings(1);
		break;
    case CY_DBG_COVER_MODE_OFF:
        TS_LOG_INFO( "%s: cover off.\n", __func__);
		cyttsp5_cover_mode_settings(0);
		break;
	default:
		TS_LOG_ERR( "%s: Invalid value\n", __func__);
	}

cyttsp5_drv_debug_store_exit:
	return size;
}

/*
 * Show system status on deep sleep status via sysfs
 */
static ssize_t cyttsp5_sleep_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	if (cd->sleep_state == SS_SLEEP_ON)
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "off\n");
	else
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "on\n");
	mutex_unlock(&cd->system_lock);

	return ret;
}

static ssize_t cyttsp5_easy_wakeup_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&cd->system_lock);
	ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "0x%02X\n",
			cd->easy_wakeup_gesture);
	mutex_unlock(&cd->system_lock);
	return ret;
}

static ssize_t cyttsp5_easy_wakeup_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0)
		return ret;

	if (value > 0xFF && value < 0)
		return -EINVAL;

	mutex_lock(&cd->system_lock);
	if (cd->sysinfo.ready && IS_PIP_VER_GE(&cd->sysinfo, 1, 2))
		cd->easy_wakeup_gesture = (u8)value;
	else
		ret = -ENODEV;
	mutex_unlock(&cd->system_lock);

	if (ret)
		return ret;

	return size;
}

extern struct cyttsp5_touch print_touch_xydata;

/*
 * Show system status with debug mode via sysfs
 */
static ssize_t cyttsp5_print_xydata_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int len = 0;

    len += snprintf(buf + len, CY_MAX_PRBUF_SIZE, "X = %d ",print_touch_xydata.abs[CY_TCH_X]);
    len += snprintf(buf + len, CY_MAX_PRBUF_SIZE, "Y = %d\n",print_touch_xydata.abs[CY_TCH_Y]);
#if 0
    len += snprintf(buf + len, CY_MAX_PRBUF_SIZE, "Z   = %d\n",print_touch_xydata.abs[CY_TCH_P]);
    len += snprintf(buf + len, CY_MAX_PRBUF_SIZE, "TID = %d\n",print_touch_xydata.abs[CY_TCH_T]);
    len += snprintf(buf + len, CY_MAX_PRBUF_SIZE, "EID = %d\n",print_touch_xydata.abs[CY_TCH_E]);
    len += snprintf(buf + len, CY_MAX_PRBUF_SIZE, "OID = %d\n",print_touch_xydata.abs[CY_TCH_O]);
    len += snprintf(buf + len, CY_MAX_PRBUF_SIZE, "W   = %d\n",print_touch_xydata.abs[CY_TCH_W]);
    len += snprintf(buf + len, CY_MAX_PRBUF_SIZE, "MAJ = %d\n",print_touch_xydata.abs[CY_TCH_MAJ]);
    len += snprintf(buf + len, CY_MAX_PRBUF_SIZE, "MIN = %d\n",print_touch_xydata.abs[CY_TCH_MIN]);
    len += snprintf(buf + len, CY_MAX_PRBUF_SIZE, "OR  = %d\n",print_touch_xydata.abs[CY_TCH_OR]);
#endif
	return len;
}
/*
 * Show system touchinfo via sysfs
 */
static ssize_t cyttsp5_touchinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int len = 0;

    len += snprintf(buf + len, CY_MAX_PRBUF_SIZE, "%s\n",CYTTSP5_TOUCHINFO);
    return len;
}
/*
 * Show Panel ID via sysfs
 */
static ssize_t cyttsp5_panel_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	if (!cd->sysinfo.ready)
		return sprintf(buf, "SysInfo not ready!\n");

	return sprintf(buf, "%d\n",
			cd->sysinfo.sensing_conf_data.panel_id);
}

/*
 * Show ttconfig version via sysfs
 */
static ssize_t cyttsp5_ttconfig_ver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
    struct cyttsp5_cydata *cydata = &cd->sysinfo.cydata;

	if (!cd->sysinfo.ready)
		return sprintf(buf, "SysInfo not ready!\n");

	return sprintf(buf, "%d\n",cydata->fw_ver_conf);
}

static ssize_t cyttsp5_wakeup_system_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	
	if (cd->wakeup_system_enabled)
		return sprintf(buf, "%d\n",1);
	else
		return sprintf(buf, "%d\n",0);
}
static ssize_t cyttsp5_wakeup_system_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0)
		return ret;

	if (value > 1)
		return -EINVAL;
	
	mutex_lock(&cd->system_lock);
	if (value == 1)
		cd->wakeup_system_enabled = true;
	else
		cd->wakeup_system_enabled = false;
	mutex_unlock(&cd->system_lock);

	return size;
}

static ssize_t cyttsp5_cover_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned int cover_mode;
    unsigned int x0,y0,x1,y1;
    int ret = -1;

	ret = sscanf(buf, "%u %u %u %u %u", &cover_mode,&x0,&y0,&x1,&y1);
    if(ret != 5){
        TS_LOG_ERR( "%s:input value is error.\n", __func__);
		return -EINVAL; 
    }

    mt_data->cover_info.x0 = x0;
    mt_data->cover_info.y0 = y0;
    mt_data->cover_info.x1 = x1;
    mt_data->cover_info.y1 = y1;

	TS_LOG_INFO( "%s:cover_mode = %d,x0 = %d,y0 = %d,x1 = %d,y1 = %d\n", __func__,cover_mode,x0,y0,x1,y1);

	return size;
}

static ssize_t cyttsp5_cover_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	
	return sprintf(buf, "%u %u %u %u %u %u\n",mt_data->cover_info.enable,
                                              mt_data->cover_info.cover_mode,
                                              mt_data->cover_info.x0,
                                              mt_data->cover_info.y0,
                                              mt_data->cover_info.x1,
                                              mt_data->cover_info.y1);
}

static ssize_t cyttsp5_glove_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",mt_data->glove_mode_enabled);
}

static ssize_t cyttsp5_enable_cover_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
    int enable;
    int ret = -1;

	ret = sscanf(buf, "%u", &enable);
    if(ret != 1){
        TS_LOG_ERR( "%s:input value is error.\n", __func__);
		return -EINVAL; 
    }
    
    mt_data->cover_info.enable = enable;

	TS_LOG_INFO( "%s:enable = %d.\n", __func__,enable);

	return size;
}

static ssize_t cyttsp5_enable_cover_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	
	return sprintf(buf, "mt_data->cover_info.enable = %d.\n",mt_data->cover_info.enable);
}


static ssize_t cyttsp5_glove_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int glove_mode;
	int rc = 0;
	struct cyttsp5_mt_data *md = mt_data;

	if (NULL == md) {
		TS_LOG_ERR( "%s:md is null\n", __func__);
		return -EINVAL;
	}

	sscanf(buf, "%d", &glove_mode);

	TS_LOG_INFO( "%s:set glove_mode  = %d.\n", __func__, glove_mode);
	if (md->glove_mode_enabled == glove_mode) {
		TS_LOG_INFO( "%s:TP is already in glove_mode  = %d.\n", __func__, glove_mode);
		return size;
	}
	
	if (glove_mode) {
		rc = cyttsp5_set_touch_mode(NORMAL_MODE);
	} else {
		rc = cyttsp5_set_touch_mode(FINGER_ONLY_MODE);
	}
	
	if (rc < 0) {
		TS_LOG_INFO( "%s: set mode %d failed.\n", __func__, glove_mode);
		return size;
	} else {
		md->glove_mode_enabled = glove_mode;
	}

	TS_LOG_DEBUG( "%s:set TP glove mode done!\n", __func__);
	return size;
}

static ssize_t cyttsp5_loglevel_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	
	return sprintf(buf, "g_ts_log_level = %d.\n",g_ts_log_level);
}


static ssize_t cyttsp5_loglevel_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
    int ret  = -1;
    unsigned long value;
    
	ret = kstrtoul(buf, 10, &value);
	if (ret < 0)
		return ret;

	if (value != 0x01 && value != 0x00)
		return -EINVAL;

    g_ts_log_level = value;

	return size;
}



static struct device_attribute attributes[] = {
	__ATTR(ic_ver, 0444, cyttsp5_ic_ver_show, NULL),
	__ATTR(drv_ver, 0444, cyttsp5_drv_ver_show, NULL),
	__ATTR(hw_reset, 0220, NULL, cyttsp5_hw_reset_store),
	__ATTR(hw_irq_stat, 0444, cyttsp5_hw_irq_stat_show, NULL),
	__ATTR(drv_irq, 0664, cyttsp5_drv_irq_show,
		cyttsp5_drv_irq_store),
	__ATTR(drv_debug, 0220, NULL, cyttsp5_drv_debug_store),
	__ATTR(sleep_status, 0444, cyttsp5_sleep_status_show, NULL),
	__ATTR(easy_wakeup_gesture, 0664,
		cyttsp5_easy_wakeup_gesture_show,
		cyttsp5_easy_wakeup_gesture_store),
	__ATTR(print_xydata, 0444, cyttsp5_print_xydata_show,NULL),
	__ATTR(touchinfo, 0444, cyttsp5_touchinfo_show,NULL),
	__ATTR(panel_id, 0444, cyttsp5_panel_id_show,NULL),
	__ATTR(ttconfig_ver, 0444, cyttsp5_ttconfig_ver_show,NULL),
	__ATTR(touch_mode, 0664,cyttsp5_touch_mode_show,
		cyttsp5_touch_mode_store),
	__ATTR(wakeup_system, 0664,cyttsp5_wakeup_system_show,
		cyttsp5_wakeup_system_store),
	__ATTR(glove_mode, 0664,cyttsp5_glove_mode_show,
		cyttsp5_glove_mode_store),
	__ATTR(cover_mode, 0664,cyttsp5_cover_mode_show,
		cyttsp5_cover_mode_store),
	__ATTR(enable_cover, 0664,cyttsp5_enable_cover_mode_show,
		cyttsp5_enable_cover_mode_store),
    __ATTR(loglevel, 0664,cyttsp5_loglevel_show,
            cyttsp5_loglevel_store),

};

static int add_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto undo;
	return 0;
undo:
	for (i--; i >= 0 ; i--)
		device_remove_file(dev, attributes + i);
	TS_LOG_ERR( "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	u32 i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

#ifdef TTHE_TUNER_SUPPORT
static int tthe_debugfs_open(struct inode *inode, struct file *filp)
{
	struct cyttsp5_core_data *cd = inode->i_private;
	filp->private_data = inode->i_private;

	if (cd->tthe_buf)
		return -EBUSY;

	cd->tthe_buf = kzalloc(CY_MAX_PRBUF_SIZE, GFP_KERNEL);
	if (!cd->tthe_buf)
		return -ENOMEM;

	return 0;
}

static int tthe_debugfs_close(struct inode *inode, struct file *filp)
{
	struct cyttsp5_core_data *cd = filp->private_data;
	filp->private_data = NULL;

	kfree(cd->tthe_buf);
	cd->tthe_buf = NULL;

	return 0;
}

static ssize_t tthe_debugfs_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct cyttsp5_core_data *cd = filp->private_data;
	int size;
	int ret;

	wait_event_interruptible(cd->wait_q,
			cd->tthe_buf_len != 0 || cd->tthe_exit);
	mutex_lock(&cd->tthe_lock);
	if (cd->tthe_exit) {
		mutex_unlock(&cd->tthe_lock);
		return 0;
	}
	if (count > cd->tthe_buf_len)
		size = cd->tthe_buf_len;
	else
		size = count;
	if (!size) {
		mutex_unlock(&cd->tthe_lock);
		return 0;
	}

	ret = copy_to_user(buf, cd->tthe_buf, cd->tthe_buf_len);
	if (ret == size)
		return -EFAULT;
	size -= ret;
	cd->tthe_buf_len -= size;
	mutex_unlock(&cd->tthe_lock);
	*ppos += size;
	return size;
}

static const struct file_operations tthe_debugfs_fops = {
	.open = tthe_debugfs_open,
	.release = tthe_debugfs_close,
	.read = tthe_debugfs_read,
};
#endif

static int cyttsp5_core_probe(struct cyttsp5_core *core)
{
	struct cyttsp5_core_data *cd;
	struct device *dev = &core->dev;
	struct cyttsp5_core_platform_data *pdata = dev_get_platdata(dev);
	enum cyttsp5_atten_type type;
	unsigned int irq_flags;
	int rc = 0;
	int retry = 3;

	if (pdata == NULL) {
		TS_LOG_ERR( "%s: Missing platform data\n", __func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}

	/* get context and debug print buffers */
	cd = kzalloc(sizeof(*cd), GFP_KERNEL);
	if (cd == NULL) {
		TS_LOG_ERR( "%s: Error, kzalloc cd\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data;
	}
	
	TS_LOG_INFO( "%s: initialize core data\n", __func__);

    //pdata->irq_gpio = g_ts_i2c->power_ctrl.irq_gpio;
    //pdata->reset_gpio = g_ts_i2c->power_ctrl.reset_gpio;

	/* Initialize device info */
	cd->core = core;
	cd->dev = dev;
	cd->pdata = pdata;

	/* Initialize mutexes and spinlocks */
	mutex_init(&cd->system_lock);
	mutex_init(&cd->adap_lock);
	mutex_init(&cd->hid_report_lock);
	spin_lock_init(&cd->spinlock);

	/* Initialize attention lists */
	for (type = 0; type < CY_ATTEN_NUM_ATTEN; type++)
		INIT_LIST_HEAD(&cd->atten_list[type]);

	/* Initialize wait queue */
	init_waitqueue_head(&cd->wait_q);

	/* Initialize works */
	INIT_WORK(&cd->startup_work, cyttsp5_startup_work_function);
	INIT_WORK(&cd->watchdog_work, cyttsp5_watchdog_work);

	/* Initialize HID specific data */
	cd->hid_core.hid_vendor_id = (pdata->vendor_id) ?
		pdata->vendor_id : CY_HID_VENDOR_ID;
	cd->hid_core.hid_product_id = (pdata->product_id) ?
		pdata->product_id : CY_HID_APP_PRODUCT_ID;
	cd->hid_core.hid_desc_register =
		cpu_to_le16(pdata->hid_desc_register);

	/* Initialize IRQ */
	cd->irq = gpio_to_irq(pdata->irq_gpio);
	if (cd->irq < 0) {
		rc = -EINVAL;
		goto error_gpio_irq;
	}
	cd->irq_enabled = true;

	dev_set_drvdata(dev, cd);

	/* Call platform init function */
	if (cd->pdata->init) {
		TS_LOG_DEBUG( "%s: Init HW\n", __func__);
		rc = cd->pdata->init(cd->pdata, 1, cd->dev);
	} else {
		TS_LOG_INFO( "%s: No HW INIT function\n", __func__);
		rc = 0;
	}
	if (rc < 0)
		TS_LOG_ERR( "%s: HW Init fail r=%d\n", __func__, rc);

	msleep(200);

	TS_LOG_DEBUG( "%s: initialize threaded irq=%d\n", __func__, cd->irq);
	if (cd->pdata->level_irq_udelay > 0)
		/* use level triggered interrupts */
		irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_NO_SUSPEND;
	else
		/* use edge triggered interrupts */
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND;

	rc = request_threaded_irq(cd->irq, NULL, cyttsp5_irq, irq_flags,
		dev_name(dev), cd);
	if (rc < 0) {
		TS_LOG_ERR( "%s: Error, could not request irq\n", __func__);
		goto error_request_irq;
	}
	disable_irq(cd->irq);
	/* Setup watchdog timer */
	setup_timer(&cd->watchdog_timer, cyttsp5_watchdog_timer,
			(unsigned long)cd);

	TS_LOG_DEBUG( "%s: add sysfs interfaces\n", __func__);
	rc = add_sysfs_interfaces(dev);
	if (rc < 0) {
		TS_LOG_ERR( "%s: Error, fail sysfs init\n", __func__);
		goto error_attr_create;
	}

#ifdef TTHE_TUNER_SUPPORT
	mutex_init(&cd->tthe_lock);
	cd->tthe_debugfs = debugfs_create_file(CYTTSP5_TTHE_TUNER_FILE_NAME,
			0644, NULL, cd, &tthe_debugfs_fops);
#endif
#ifndef CY_TOUCH_RUNTIME_NOTUSED
	device_init_wakeup(dev, 1);
#endif
	/*
	 * call startup directly to ensure that the device
	 * is tested before leaving the probe
	 */
	rc = cyttsp5_check_and_deassert_int(cd);
	if (rc < 0) {
		TS_LOG_INFO( "%s:i2c communication failed and try poweron again= %d\n", __func__, rc);
		while (retry-- > 0) {
			cyttsp5_power_on_try(cd);
			rc = cyttsp5_check_and_deassert_int(cd);
			if (rc >= 0) {
				TS_LOG_INFO( "%s:i2c communication retry ok\n", __func__);
				break;
			}
		}
		if (retry == 0) {
			TS_LOG_INFO( "%s:i2c communication failed and probe exit\n", __func__);
			goto error_check_i2c;
		}
	}
	enable_irq(cd->irq);
	TS_LOG_DEBUG( "%s: call startup\n", __func__);
	rc = cyttsp5_startup(cd);
	if (rc < 0) {
		TS_LOG_ERR( "%s: Fail initial startup r=%d\n",
			__func__, rc);
	}
	TS_LOG_INFO( "%s successful.\n", __func__);

	rc = sysfs_create_link(NULL,&dev->kobj,"touchscreen");
	if (rc) {
		TS_LOG_ERR( "%s: Fail create link rc=%d\n", __func__, rc);
		goto error_attr_create;
	}

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	/* detect current device successful, set the flag as present */
	set_hw_dev_flag(DEV_I2C_TOUCH_PANEL);
#endif
	cd->easy_wakeup_gesture = 0x00;
#ifdef CONFIG_TOUCHSCREEN_MMI_EQUIP
	register_equip_touchscreen_mmi_tests(cd);
#endif
	globe_cd = cd;

	return 0;

#ifdef CONFIG_HUAWEI_TOUCH_INPUT_INFO
error_alloc_touch_info:
#endif
error_attr_create:
error_check_i2c:
	free_irq(cd->irq, cd);
	del_timer(&cd->watchdog_timer);
error_request_irq:
	if (pdata->init)
		pdata->init(pdata, 0, dev);
	dev_set_drvdata(dev, NULL);
error_gpio_irq:
	kfree(cd);
error_alloc_data:
error_no_pdata:
	TS_LOG_ERR( "%s failed.\n", __func__);
	return rc;
}

static int cyttsp5_core_release(struct cyttsp5_core *core)
{
	struct device *dev = &core->dev;
	struct cyttsp5_core_data *cd = dev_get_drvdata(dev);

	cancel_work_sync(&cd->startup_work);

	cyttsp5_stop_wd_timer(cd);

#ifndef CY_TOUCH_RUNTIME_NOTUSED
	device_init_wakeup(dev, 0);
#endif

#ifdef TTHE_TUNER_SUPPORT
	mutex_lock(&cd->tthe_lock);
	cd->tthe_exit = 1;
	wake_up(&cd->wait_q);
	mutex_unlock(&cd->tthe_lock);
	debugfs_remove(cd->tthe_debugfs);
#endif
	remove_sysfs_interfaces(dev);
	free_irq(cd->irq, cd);
	if (cd->pdata->init)
		cd->pdata->init(cd->pdata, 0, dev);
	dev_set_drvdata(dev, NULL);
	cyttsp5_free_si_ptrs(cd);
	cyttsp5_free_hid_reports(cd);
	kfree(cd);
	return 0;
}

static struct cyttsp5_core_nonhid_cmd _cyttsp5_core_nonhid_cmd = {
	.start_bl = _cyttsp5_request_hid_output_start_bl,
	.suspend_scanning = _cyttsp5_request_hid_output_suspend_scanning,
	.resume_scanning = _cyttsp5_request_hid_output_resume_scanning,
	.get_param = _cyttsp5_request_hid_output_get_param,
	.set_param = _cyttsp5_request_hid_output_set_param,
	.verify_config_block_crc =
		_cyttsp5_request_hid_output_verify_config_block_crc,
	.get_config_row_size = _cyttsp5_request_hid_output_get_config_row_size,
	.calibrate_idacs = _cyttsp5_request_hid_output_calibrate_idacs,
	.initialize_baselines =
		_cyttsp5_request_hid_output_initialize_baselines,
	.exec_panel_scan = _cyttsp5_request_hid_output_exec_panel_scan,
	.retrieve_panel_scan = _cyttsp5_request_hid_output_retrieve_panel_scan,
	.write_conf_block = _cyttsp5_request_hid_output_write_conf_block,
	.user_cmd = _cyttsp5_request_hid_output_user_cmd,
	.get_bl_info = _cyttsp5_request_hid_output_bl_get_information,
	.initiate_bl = _cyttsp5_request_hid_output_bl_initiate_bl,
	.launch_app = _cyttsp5_request_hid_output_launch_app,
	.prog_and_verify = _cyttsp5_request_hid_output_bl_program_and_verify,
	.verify_app_integrity =
		_cyttsp5_request_hid_output_bl_verify_app_integrity,
};

static struct cyttsp5_core_driver cyttsp5_core_driver = {
	.probe = cyttsp5_core_probe,
	.remove = cyttsp5_core_release,
	.subscribe_attention = _cyttsp5_subscribe_attention,
	.unsubscribe_attention = _cyttsp5_unsubscribe_attention,
	.request_exclusive = _cyttsp5_request_exclusive,
	.release_exclusive = _cyttsp5_release_exclusive,
	.request_reset = _cyttsp5_request_reset,
	.request_restart = _cyttsp5_request_restart,
	.request_sysinfo = _cyttsp5_request_sysinfo,
	.request_loader_pdata = _cyttsp5_request_loader_pdata,
	.request_stop_wd = _cyttsp5_request_stop_wd,
	.request_get_hid_desc = _cyttsp5_request_get_hid_desc,
	.request_get_mode = _cyttsp5_request_get_mode,
	.request_enable_scan_type = _cyttsp5_request_enable_scan_type,
	.request_disable_scan_type = _cyttsp5_request_disable_scan_type,
#ifdef TTHE_TUNER_SUPPORT
	.request_tthe_print = _cyttsp5_request_tthe_print,
#endif
	.cmd = &_cyttsp5_core_nonhid_cmd,
	.driver = {
		.name = CYTTSP5_CORE_NAME,
		.bus = &cyttsp5_bus_type,
		.owner = THIS_MODULE,
#ifndef CY_TOUCH_RUNTIME_NOTUSED
		.pm = &cyttsp5_core_pm_ops,
#endif
	},
};

static int __init cyttsp5_core_init(void)
{
	int rc;

/*	
	if(get_boot_into_recovery_flag())
		return -1;
*/
	rc = cyttsp5_register_core_driver(&cyttsp5_core_driver);
	TS_LOG_INFO("%s: Cypress TTSP v5 core driver (Built %s) rc=%d\n",
		 __func__, CY_DRIVER_DATE, rc);
	return rc;
}
module_init(cyttsp5_core_init);

static void __exit cyttsp5_core_exit(void)
{
	cyttsp5_unregister_core_driver(&cyttsp5_core_driver);
}
module_exit(cyttsp5_core_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen core driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");
