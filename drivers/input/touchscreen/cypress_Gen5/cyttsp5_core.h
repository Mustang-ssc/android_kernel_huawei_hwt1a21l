/*
 * cyttsp5_core.h
 * Cypress TrueTouch(TM) Standard Product V5 Core driver module.
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

#ifndef _LINUX_CYTTSP5_CORE_H
#define _LINUX_CYTTSP5_CORE_H

#include <linux/stringify.h>
#include "cyttsp5_regs.h"
#include "cyttsp5_bus.h"
#include "cyttsp5_i2c.h"


#define CYTTSP5_CORE_NAME "cyttsp5_core"
#define CYTTSP5_TOUCHINFO "Cypress_TMA448"

#define CY_DRIVER_NAME TTDA
#define CY_DRIVER_MAJOR 03
#define CY_DRIVER_MINOR 01
#define ENABLE_ESD_POWEROFF

#define CY_DRIVER_REVCTRL 547877

#define CY_DRIVER_VERSION		    \
__stringify(CY_DRIVER_NAME)		    \
"." __stringify(CY_DRIVER_MAJOR)	    \
"." __stringify(CY_DRIVER_MINOR)	    \
"." __stringify(CY_DRIVER_REVCTRL)

#define CY_DRIVER_DATE "20131009"	/* YYYYMMDD */

/* x-axis resolution of panel in pixels */
#define CY_PCFG_RESOLUTION_X_MASK 0x7F

/* y-axis resolution of panel in pixels */
#define CY_PCFG_RESOLUTION_Y_MASK 0x7F

/* x-axis, 0:origin is on left side of panel, 1: right */
#define CY_PCFG_ORIGIN_X_MASK 0x80

/* y-axis, 0:origin is on top side of panel, 1: bottom */
#define CY_PCFG_ORIGIN_Y_MASK 0x80

#define CY_TOUCH_SETTINGS_MAX 32
#define CY_TOUCH_RUNTIME_NOTUSED

#define TOUCH_INFO_CYPRESS568 "cypress_CY8CTMA568"

#define CY_HID_VENDOR_ID	0x04B4
#define CY_HID_BL_PRODUCT_ID	0xC100
#define CY_HID_APP_PRODUCT_ID	0xC101
#define CY_HID_VERSION		0x0100
#define CY_HID_APP_REPORT_ID	0xF7
#define CY_HID_BL_REPORT_ID	0xFF

/* Timeout in ms. */
#define CY_CORE_RESET_AND_WAIT_TIMEOUT		500
#define CY_CORE_WAKEUP_TIMEOUT			500
#define CY_HID_RESET_TIMEOUT			2000
#define CY_HID_SET_POWER_TIMEOUT		500
#define CY_HID_GET_HID_DESCRIPTOR_TIMEOUT	500
#define CY_HID_GET_REPORT_DESCRIPTOR_TIMEOUT	500
#ifdef VERBOSE_DEBUG
#define CY_HID_OUTPUT_TIMEOUT			2000
#else
#define CY_HID_OUTPUT_TIMEOUT			200
#endif
#define CY_HID_OUTPUT_START_BOOTLOADER_TIMEOUT	2000
#define CY_HID_OUTPUT_USER_TIMEOUT		3000
#define CY_HID_OUTPUT_GET_SYSINFO_TIMEOUT	3000
#define CY_HID_OUTPUT_CALIBRATE_IDAC_TIMEOT	5000
#define CY_HID_OUTPUT_BL_INITIATE_BL_TIMEOUT	8000

#define CY_MAX_INPUT		512

#define HID_OUTPUT_RESPONSE_REPORT_OFFSET	2
#define HID_OUTPUT_RESPONSE_CMD_OFFSET		4
#define HID_OUTPUT_RESPONSE_CMD_MASK		0x7F
#define HID_OUTPUT_CMD_OFFSET			6
#define HID_OUTPUT_CMD_MASK			0x7F

#define HID_SYSINFO_CYDATA_OFFSET	5
#define HID_SYSINFO_SENSING_OFFSET	33
#define HID_SYSINFO_BTN_OFFSET		48
#define HID_SYSINFO_BTN_MASK		0xF
#define HID_SYSINFO_MAX_BTN		4

#define HID_APP_OUTPUT_REPORT_ID	0x2F
#define HID_APP_RESPONSE_REPORT_ID	0x1F
#define HID_BL_OUTPUT_REPORT_ID		0x40
#define HID_BL_RESPONSE_REPORT_ID	0x30
#define HID_RESPONSE_REPORT_ID		0xF0

#define HID_POWER_ON			0x0
#define HID_POWER_SLEEP			0x1
#define HID_LENGTH_BYTES		2
#define HID_LENGTH_AND_REPORT_ID_BYTES	3

#define CY_HID_MAX_REPORTS		8
#define CY_HID_MAX_FIELDS		128
#define CY_HID_MAX_COLLECTIONS		3
#define CY_HID_MAX_NESTED_COLLECTIONS	CY_HID_MAX_COLLECTIONS

#ifdef TTHE_TUNER_SUPPORT
#define CYTTSP5_TTHE_TUNER_FILE_NAME "tthe_tuner"
#endif


#define SET_CMD_OPCODE(byte, opcode) SET_CMD_LOW(byte, opcode)
#define SET_CMD_REPORT_TYPE(byte, type) SET_CMD_HIGH(byte, ((type) << 4))
#define SET_CMD_REPORT_ID(byte, id) SET_CMD_LOW(byte, id)

#define HID_OUTPUT_APP_COMMAND(command) \
	.cmd_type = HID_OUTPUT_CMD_APP, \
	.command_code = command

#define HID_OUTPUT_BL_COMMAND(command) \
	.cmd_type = HID_OUTPUT_CMD_BL, \
	.command_code = command


enum cyttsp5_core_platform_flags {
	CY_CORE_FLAG_NONE,
	CY_CORE_FLAG_WAKE_ON_GESTURE,
};

enum cyttsp5_loader_platform_flags {
	CY_LOADER_FLAG_NONE,
	CY_LOADER_FLAG_CALIBRATE_AFTER_FW_UPGRADE,
};

struct touch_settings {
	const uint8_t   *data;
	uint32_t         size;
	const uint8_t   *ver;
	uint32_t         vsize;
	uint8_t         tag;
};

struct cyttsp5_touch_firmware {
	const uint8_t *img;
	uint32_t size;
	const uint8_t *ver;
	uint8_t vsize;
};

struct cyttsp5_touch_config {
	struct touch_settings *param_regs;
	struct touch_settings *param_size;
	const uint8_t *fw_ver;
	uint8_t fw_vsize;
};

struct cyttsp5_loader_platform_data {
	struct cyttsp5_touch_firmware *fw;
	struct cyttsp5_touch_config *ttconfig;
	u32 flags;
};

struct cyttsp5_core_platform_data {
	int irq_gpio;
	int rst_gpio;
	int level_irq_udelay;
	u16 hid_desc_register;
	u16 vendor_id;
	u16 product_id;
	int (*xres)(struct cyttsp5_core_platform_data *pdata,
		struct device *dev);
	int (*init)(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev);
	int (*power)(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq);
	int (*irq_stat)(struct cyttsp5_core_platform_data *pdata,
		struct device *dev);
	struct touch_settings *sett[CY_TOUCH_SETTINGS_MAX];
	struct cyttsp5_loader_platform_data *loader_pdata;
	u32 flags;
};

struct cyttsp5_hid_desc {
	__le16 hid_desc_len;
	u8 packet_id;
	u8 reserved_byte;
	__le16 bcd_version;
	__le16 report_desc_len;
	__le16 report_desc_register;
	__le16 input_register;
	__le16 max_input_len;
	__le16 output_register;
	__le16 max_output_len;
	__le16 command_register;
	__le16 data_register;
	__le16 vendor_id;
	__le16 product_id;
	__le16 version_id;
	u8 reserved[4];
} __packed;

enum cyttsp5_sleep_state {
	SS_SLEEP_OFF,
	SS_SLEEP_ON,
	SS_SLEEPING,
	SS_WAKING,
};

enum cyttsp5_startup_state {
	STARTUP_NONE,
	STARTUP_QUEUED,
	STARTUP_RUNNING,
	STARTUP_ILLEGAL,
};

struct cyttsp5_hid_core {
	u16 hid_vendor_id;
	u16 hid_product_id;
	__le16 hid_desc_register;
	u16 hid_report_desc_len;
	u16 hid_max_input_len;
	u16 hid_max_output_len;
};

struct cyttsp5_hid_field {
	int report_count;
	int report_size;
	int size; /* report_count * report_size */
	int offset;
	int data_type;
	int logical_min;
	int logical_max;
	/* Usage Page (Hi 16 bit) + Usage (Lo 16 bit) */
	u32 usage_page;
	u32 collection_usage_pages[CY_HID_MAX_COLLECTIONS];
	struct cyttsp5_hid_report *report;
	bool record_field;
};

struct cyttsp5_hid_report {
	u8 id;
	u8 type;
	int size;
	struct cyttsp5_hid_field *fields[CY_HID_MAX_FIELDS];
	int num_fields;
	int record_field_index;
	int header_size;
	int record_size;
	u32 usage_page;
};

struct cyttsp5_core_data {
	struct device *dev;
	struct cyttsp5_core *core;
	struct list_head atten_list[CY_ATTEN_NUM_ATTEN];
	struct mutex system_lock;
	struct mutex adap_lock;
	struct mutex hid_report_lock;
	enum cyttsp5_mode mode;
	spinlock_t spinlock;
	struct cyttsp5_core_platform_data *pdata;
	wait_queue_head_t wait_q;
	enum cyttsp5_sleep_state sleep_state;
	enum cyttsp5_startup_state startup_state;
	int irq;
	bool irq_enabled;
	bool irq_wake;
	bool wakeup_system_enabled;
	bool proximity_flag;
	u8 easy_wakeup_gesture;
	bool wake_initiated_by_device;
	struct work_struct startup_work;
	struct cyttsp5_sysinfo sysinfo;
	void *exclusive_dev;
	int exclusive_waits;
	struct work_struct watchdog_work;
	struct timer_list watchdog_timer;
	struct cyttsp5_hid_core hid_core;
	int hid_cmd_state;
	int hid_reset_cmd_state; /* reset can happen any time */
	struct cyttsp5_hid_desc hid_desc;
	struct cyttsp5_hid_report *hid_reports[CY_HID_MAX_REPORTS];
	int num_hid_reports;
#ifdef TTHE_TUNER_SUPPORT
	struct dentry *tthe_debugfs;
	u8 *tthe_buf;
	u32 tthe_buf_len;
	struct mutex tthe_lock;
	u8 tthe_exit;
#endif
	u8 input_buf[CY_MAX_INPUT];
	u8 response_buf[CY_MAX_INPUT];
#ifdef VERBOSE_DEBUG
	u8 pr_buf[CY_MAX_PRBUF_SIZE];
#endif
    //struct cyttsp5_power_control power_ctrl;
};

struct atten_node {
	struct list_head node;
	int (*func)(struct cyttsp5_device *);
	struct cyttsp5_device *ttsp;
	int mode;
};

struct cyttsp5_hid_cmd {
	u8 opcode;
	u8 report_type;
	union {
		u8 report_id;
		u8 power_state;
	};
	u8 has_data_register;
	size_t write_length;
	u8 *write_buf;
	u8 *read_buf;
	u8 wait_interrupt;
	u8 reset_cmd;
	u16 timeout_ms;
};

struct cyttsp5_hid_output {
	u8 cmd_type;
	u16 length;
	u8 command_code;
	size_t write_length;
	u8 *write_buf;
	u8 novalidate;
	u16 timeout_ms;
};

int cyttsp5_hw_reset_export(void);

#ifdef VERBOSE_DEBUG
extern void cyttsp5_pr_buf(struct device *dev, u8 *pr_buf, u8 *dptr, int size,
			   const char *data_name);
#else
#define cyttsp5_pr_buf(a, b, c, d, e) do { } while (0)
#endif

unsigned int cyttsp5_get_touch_mode(void);
int cyttsp5_set_touch_mode(u32 value);
#ifdef CY_TOUCH_RUNTIME_NOTUSED
int cyttsp5_core_early_suspend(void);
int cyttsp5_core_late_rusume(void);
#endif
#endif /* _LINUX_CYTTSP5_CORE_H */
