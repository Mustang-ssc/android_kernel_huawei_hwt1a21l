/*< DTS2014041500980 renlipeng 20140419 begin*/
/*< DTS2013081305942 mazhenhua 20130909 begin*/
/* < DTS2013080201182 wuzhihui 20130802 begin */
#ifndef __HUAWEI_APANIC_H__
#define __HUAWEI_APANIC_H__

//#include <mach/msm_iomap.h>
#include <linux/types.h>
#include <asm/sizes.h>
#include <linux/of.h>

#define HUAWEI_PANIC_TAG "[HUAWEI_PANIC]"

#define CRASH_LOG_MAGIC_NUM               (0xCACADEAD)
/* < DTS2013092801324 roopesh 20130927 begin */
//#define CRASH_LOG_MAGIC_NUM_ADDR          (MSM_IMEM_BASE + 0xB18)
//#define CRASH_LOG_LENGTH_ADDR             (MSM_IMEM_BASE + 0xB1C)
/* < DTS2013092801324 roopesh 20130927 end */

extern void *hw_reset_magic_addr;
#define HW_RESET_LOG_MAGIC_NUM               (0xCACADEAD)
//#define HW_RESET_LOG_MAGIC_NUM_ADDR          (MSM_IMEM_BASE + 0xB20)
#define HW_RESET_LOG_MAGIC_NUM_LEN   (4)

#define DEFAULT_PANIC_LOG_LEN             (1024)
#define MAX_LOG_BUF_LENGTH                (0x800000) /* 8M */

#ifdef CONFIG_PRINTK
void* huawei_get_log_buf_addr(void);
int huawei_get_log_buf_len(void);
#endif
/* < DTS2014040103298 zhaoyingchun 20140401 begin */
void clear_hw_reset(void);
/* DTS2014040103298 zhaoyingchun 20140401 end > */
#endif
/* DTS2013080201182 wuzhihui 20130802 end > */
/*DTS2013081305942 mazhenhua 20130909 end >*/
/*DTS2014041500980 renlipeng 20140419 end >*/
