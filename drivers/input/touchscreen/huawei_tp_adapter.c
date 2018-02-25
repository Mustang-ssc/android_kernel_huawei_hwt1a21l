/* < DTS2014062006092  songrongyuan 20140620 begin */
/*< DTS2014011007929 songrongyuan 20140110 begin*/
#include <linux/huawei_tp_adapter.h>
/* DTS2014011007929 songrongyuan 20140110 end >*/

atomic_t touch_detected_flag = ATOMIC_INIT(0);

static void set_touch_probe_flag(int detected)
{
	if(detected >= 0)
	{
		atomic_set(&touch_detected_flag, 1);
	}
	else
	{
		atomic_set(&touch_detected_flag, 0);
	}

	return;
}
static int read_touch_probe_flag(void)
{
	return atomic_read(&touch_detected_flag);
}


struct touch_hw_platform_data touch_hw_data =
{
	.set_touch_probe_flag = set_touch_probe_flag,
	.read_touch_probe_flag = read_touch_probe_flag,
};
/* DTS2014062006092  songrongyuan 20140620 end >*/
