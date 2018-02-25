/*
 * cyttsp5_device_access.h
 * Cypress TrueTouch(TM) Standard Product V5 Device Access module.
 * Configuration and Test command/status user interface.
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

#ifndef _LINUX_CYTTSP5_DEVICE_ACCESS_H
#define _LINUX_CYTTSP5_DEVICE_ACCESS_H
#include "cyttsp5_bus.h"
#include "cyttsp5_regs.h"

#define CYTTSP5_DEVICE_ACCESS_NAME "cyttsp5_device_access"

#define CYTTSP5_INPUT_ELEM_SZ (sizeof("0xHH") + 1)


#define CY_READ_DATA_LEN 100   //read 100 bytes(hex) once,8 times;
#define CY_READ_TIMES 8   //read 100 bytes(hex) once,8 times;

#define CY_TX_NUMBER 36
#define CY_RX_NUMBER 21
#define CY_DATA_HEAD 10

#define CY_MAX_RAW_DATA (4000)
#define CY_MIN_RAW_DATA (-4000)

#define CY_MAX_LPWC_DATA 58
#define CY_MIN_LPWC_DATA 38

#define CHECK_RETRY_TIMES	3

#define CYTTSP5_GET_OPENS 1

#define CYTTSP5_DATA_ONE_BYTE 1
#define CYTTSP5_DATA_TWO_BYTE 2




#define CY_MAX_CONFIG_BYTES    256
#define CYTTSP5_TTHE_TUNER_GET_PANEL_DATA_FILE_NAME "get_panel_data"
#define TTHE_TUNER_MAX_BUF	(CY_MAX_PRBUF_SIZE * 3)

enum cyttsp5_sensor_check {
	CY_I2C_CHECK,
    CY_RAW_CHECK,
    CY_OPEN_CHECK, 
    CY_LPWC_CHECK,
    CY_OLPWC_CHECK,
    CY_SOLPWC_CHECK,
	CY_SELFRAW_CHECK,
	CY_SHORT_CHECK,
	CY_MUT_BASE_CHECK,
	CY_SELF_BASE_CHECK,
	CY_MUT_DIFF_CHECK,
	CY_SELF_DIFF_CHECK,
	CY_MUT_NOISE_CHECK,
	CY_SELF_NOISE_CHECK,
	CY_SYSTEM_INFO_CHECK,
    CY_CHECK_MAX,
};

#ifdef TTHE_TUNER_SUPPORT

#define CY_CMD_RET_PANEL_IN_DATA_OFFSET	0
#define CY_CMD_RET_PANEL_ELMNT_SZ_MASK	0x07
#define CY_CMD_RET_PANEL_HDR		0x0A
#define CY_CMD_RET_PANEL_ELMNT_SZ_MAX	0x2

enum scan_data_type_list {
	CY_MUT_RAW,
	CY_MUT_BASE,
	CY_MUT_DIFF,
	CY_SELF_RAW,
	CY_SELF_BASE,
	CY_SELF_DIFF,
	CY_BAL_RAW,
	CY_BAL_BASE,
	CY_BAL_DIFF,
};

struct heatmap_param {
	bool scan_start;
	enum scan_data_type_list data_type; /* raw, base, diff */
	int num_element;
};
#endif

struct cyttsp5_device_access_platform_data {
	char const *device_access_dev_name;
};

struct cyttsp5_device_access_data {
	struct cyttsp5_device *ttsp;
	struct cyttsp5_device_access_platform_data *pdata;
	struct cyttsp5_sysinfo *si;
	struct mutex sysfs_lock;
	u8 status;
	u16 response_length;
	bool sysfs_nodes_created;
#ifdef TTHE_TUNER_SUPPORT
	struct heatmap_param heatmap;
	struct dentry *tthe_get_panel_data_debugfs;
	struct mutex debugfs_lock;
	u8 tthe_get_panel_data_buf[TTHE_TUNER_MAX_BUF];
	u8 tthe_get_panel_data_is_open;
#endif
#ifdef VERBOSE_DEBUG
	u8 pr_buf[CY_MAX_PRBUF_SIZE];
#endif
	u8 ic_buf[CY_MAX_PRBUF_SIZE];
	u8 response_buf[CY_MAX_PRBUF_SIZE];
    int check_index;
    int check_flag;
    int data_buf[CY_TX_NUMBER * CY_RX_NUMBER + (CY_TX_NUMBER + 2)];
    char read_cmd[8][128];
};

struct cyttsp5_check_func_ {
	int index;
	int (* func)(void);
};

unsigned char cyttsp5_command_status(void);
int cyttsp5_command_response(u8 *buf);
int cyttsp5_send_command(const char *buf);

int cyttsp5_set_check_flag(int flag);
int cyttsp5_check_items(enum cyttsp5_sensor_check item, bool reset);
#endif /* _LINUX_CYTTSP5_DEVICE_ACCESS_H */
