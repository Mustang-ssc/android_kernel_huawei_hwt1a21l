#ifndef __CYTTSP5_MMI_TEST_H__
#define __CYTTSP5_MMI_TEST_H__

#include "cyttsp5_core.h"
#include "cyttsp5_device_access.h"
#include "cyttsp5_loader.h"


#define I2C_CHECK_FAIL_MASK				1 << 0
#define MUTUA_CAP_CHECK_FAIL_MASK		1 << 1
#define SELF_CAP_CHECK_FAIL_MASK		1 << 2
#define OPEN_CHECK_FAIL_MASK			1 << 3
#define SHORT_CHECK_FAIL_MASK			1 << 4
#define LPWC_CHECK_FAIL_MASK			1 << 5
#define OLPWC_CHECK_FAIL_MASK			1 << 6
#define SOLPWC_CHECK_FAIL_MASK			1 << 7


#ifdef CONFIG_TOUCHSCREEN_MMI_EQUIP
void register_equip_touchscreen_mmi_tests(struct cyttsp5_core_data *cd);
void register_equip_loader_data(struct cyttsp5_loader_data *ld);
void register_equip_device_access_data(struct cyttsp5_device_access_data *dad);
#endif

#endif
