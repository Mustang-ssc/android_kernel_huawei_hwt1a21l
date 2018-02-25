#ifndef __CYTTSP5_LOADER_H__
#define __CYTTSP5_LOADER_H__
#include <linux/slab.h>
#include "cyttsp5_bus.h"
#include "cyttsp5_regs.h"

#define CYTTSP5_LOADER_NAME "cyttsp5_loader"
#define CYTTSP5_AUTO_LOAD_FOR_CORRUPTED_FW 1
#define CYTTSP5_LOADER_FW_UPGRADE_RETRY_COUNT 3

#define CYTTSP5_FW_UPGRADE 1

/* Timeout values in ms. */
#define CY_LDR_REQUEST_EXCLUSIVE_TIMEOUT		500
#define CY_LDR_SWITCH_TO_APP_MODE_TIMEOUT		300

#define CY_MAX_STATUS_SIZE				32

#define CY_DATA_MAX_ROW_SIZE				256
#define CY_DATA_ROW_SIZE				128

#define CY_ARRAY_ID_OFFSET				0
#define CY_ROW_NUM_OFFSET				1
#define CY_ROW_SIZE_OFFSET				3
#define CY_ROW_DATA_OFFSET				5

#define CY_POST_TT_CFG_CRC_MASK				0x2

#ifndef CONFIG_TOUCHSCREEN_FW_FORCE_UPGRADE
#define CONFIG_TOUCHSCREEN_FW_FORCE_UPGRADE		1
#endif

struct cyttsp5_loader_data {
	struct cyttsp5_device *ttsp;
	struct cyttsp5_sysinfo *si;
	u8 status_buf[CY_MAX_STATUS_SIZE];
	struct completion int_running;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BINARY_FW_UPGRADE
	struct completion builtin_bin_fw_complete;
	int builtin_bin_fw_status;
	bool is_manual_upgrade_enabled;
#endif
	struct work_struct fw_and_config_upgrade;
	struct work_struct calibration_work;
	struct cyttsp5_loader_platform_data *loader_pdata;
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MANUAL_TTCONFIG_UPGRADE
	struct mutex config_lock;
	u8 *config_data;
	int config_size;
	bool config_loading;
#endif
};

struct cyttsp5_dev_id {
	u32 silicon_id;
	u8 rev_id;
	u32 bl_ver;
};

struct cyttsp5_hex_image {
	u8 array_id;
	u16 row_num;
	u16 row_size;
	u8 row_data[CY_DATA_ROW_SIZE];
} __packed;

void cyttsp5_fw_calibrate(struct cyttsp5_loader_data *data);

#endif
