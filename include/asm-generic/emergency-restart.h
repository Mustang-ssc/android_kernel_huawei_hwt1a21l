#ifndef _ASM_GENERIC_EMERGENCY_RESTART_H
#define _ASM_GENERIC_EMERGENCY_RESTART_H

static inline void machine_emergency_restart(void)
{
/*< DTS2015012007753 wangyuantao 20150120 begin*/
#ifdef CONFIG_HUAWEI_KERNEL
	machine_restart("emergency_restart");
#else
	machine_restart(NULL);
#endif
/*DTS2015012007753 wangyuantao 20150120 end >*/
}

#endif /* _ASM_GENERIC_EMERGENCY_RESTART_H */
