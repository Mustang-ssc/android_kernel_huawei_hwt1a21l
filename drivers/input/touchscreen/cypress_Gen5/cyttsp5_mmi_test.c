#include <linux/slab.h>
#include "cyttsp5_mmi_test.h"
#include "cyttsp5_bus.h"

#ifdef CONFIG_TOUCHSCREEN_MMI_EQUIP
#include <linux/equip.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_MMI_EQUIP

#define TOUCHINFO_MAX_LENGTH	100
struct tp_mmi_test_struct {
	struct cyttsp5_core_data *cd;
	struct cyttsp5_loader_data *ld;
	struct cyttsp5_device_access_data *dad;
};

static struct tp_mmi_test_struct t_data;

static void equip_get_touchinfo(void *arg_out)
{
	struct EQUIP_PARAM* arg = (struct EQUIP_PARAM*)arg_out;	
	char touchinfo_str[TOUCHINFO_MAX_LENGTH] = { 0 };
	int index = 0;
	int panel_id = t_data.cd->sysinfo.sensing_conf_data.panel_id;
	int fw_ver = t_data.cd->sysinfo.cydata.fw_ver_conf;

	//FIXME
	index = snprintf(touchinfo_str, TOUCHINFO_MAX_LENGTH, 
					"Chip ID:%s ;", CYTTSP5_TOUCHINFO);
	index += snprintf(touchinfo_str+index, TOUCHINFO_MAX_LENGTH - index,
					 "Sensor ID:%d ;", panel_id);
	index += snprintf(touchinfo_str+index, TOUCHINFO_MAX_LENGTH - index,
					 "FW version:%d", fw_ver);
	
	snprintf(arg->str_out, EQUIP_STR_LEN, "%s\n", touchinfo_str);
	
	printk("--[%s]--arg->str_out = %s---\n", __func__,  arg->str_out);
	return;

}

static void equip_set_tptest_flag(void *arg_in)
{
	int ret = 0;
	int flag = 0;
	struct EQUIP_PARAM* arg = (struct EQUIP_PARAM*)arg_in;
	
	ret = sscanf(arg->str_in, "%d", &flag);	
	if (flag < 0 || flag > 3) {
		printk("flag = %d is invalid input", flag);
		return;
	}
	cyttsp5_set_check_flag(flag);
	return;
}

static void equip_get_tptest_result(void *arg_out)
{
	struct EQUIP_PARAM* arg = (struct EQUIP_PARAM*)arg_out;
	int check_flag = -1;
	u32 result_bits = 0x0;
	/*result_bits for X1 or X1S'cypress TP 
	*bits 0    1         2         3       4        5      6       7          8             9....
	*    I2C  MUTUA_CAP SELF_CAP  OPEN    SHORT    LPWC   OLPWC   SOLPWC     UD(undefine)  UD
	*/
	
	//i2c check
	check_flag = cyttsp5_check_items(CY_I2C_CHECK, 1);
	if (check_flag >= 0) {
		printk("%s: i2c_check ok\n", __func__);
	} else {
		printk("%s: i2c_check fail\n", __func__);
		result_bits |= I2C_CHECK_FAIL_MASK;
	}
	
	//lpwc check
	check_flag = cyttsp5_check_items(CY_LPWC_CHECK, 0);
	if (check_flag >= 0) {
		printk("%s: lpwc check ok\n", __func__);
	} else {
		printk("%s: lpwc check fail\n", __func__);
		result_bits |= LPWC_CHECK_FAIL_MASK;
	}

	//olpwc check
	check_flag = cyttsp5_check_items(CY_OLPWC_CHECK, 0);
	if (check_flag >= 0) {
		printk("%s: olpwc check ok\n", __func__);
	} else {
		printk("%s: olpwc check fail\n", __func__);
		result_bits |= OLPWC_CHECK_FAIL_MASK;
	}

	//solpwc check
	check_flag = cyttsp5_check_items(CY_SOLPWC_CHECK, 0);
	if (check_flag >= 0) {
		printk("%s: solpwc check ok\n", __func__);
	} else {
		printk("%s: solpwc check fail\n", __func__);
		result_bits |= SOLPWC_CHECK_FAIL_MASK;
	}

	//mutual cap check
	check_flag = cyttsp5_check_items(CY_RAW_CHECK, 1);
	if (check_flag >= 0) {
		printk("%s: mutual cap check ok\n", __func__);
	} else {
		printk("%s: mutual cap check fail\n", __func__);
		result_bits |= MUTUA_CAP_CHECK_FAIL_MASK;
	}

	//self cap check
	check_flag = cyttsp5_check_items(CY_SELFRAW_CHECK,1);
	if (check_flag >= 0) {
		printk("%s: self cap check ok\n", __func__);
	} else {
		printk("%s: self cap check fail\n", __func__);
		result_bits |= SELF_CAP_CHECK_FAIL_MASK;
	}

	//open check
	check_flag = cyttsp5_check_items(CY_OPEN_CHECK, 1);
	if (check_flag >= 0) {
		printk("%s: open check ok\n", __func__);
	} else {
		printk("%s: open check fail\n", __func__);
		result_bits |= OPEN_CHECK_FAIL_MASK;
	}

	//short check
	check_flag = cyttsp5_check_items(CY_SHORT_CHECK, 1);
	if (check_flag >= 0) {
		printk("%s: short check ok\n", __func__);
	} else {
		printk("%s: short check fail\n", __func__);
		result_bits |= SHORT_CHECK_FAIL_MASK;
	}
	
	printk("%s: test relust is %x\n", __func__, result_bits);
	snprintf(arg->str_out, EQUIP_STR_LEN, "%d\n", result_bits);
	return;
}

static void register_equip_tp_test(void)
{
	enum EQUIP_DEVICE equip_dev;	
	enum EQUIP_OPS ops;

	equip_dev = TP_TOUCHINFO;
	ops = EP_READ;
	register_equip_func(equip_dev, ops, equip_get_touchinfo);

	equip_dev = TP_TEST;
	ops = EP_WRITE;
	register_equip_func(equip_dev, ops, equip_set_tptest_flag);
	ops = EP_READ;
	register_equip_func(equip_dev, ops, equip_get_tptest_result);
}


void register_equip_touchscreen_mmi_tests(struct cyttsp5_core_data *cd)
{
	t_data.cd = cd;

	register_equip_tp_test();
}

void register_equip_loader_data(struct cyttsp5_loader_data *ld)
{
	t_data.ld = ld;
}

void register_equip_device_access_data(struct cyttsp5_device_access_data *dad)
{
	t_data.dad = dad;
}
#endif

