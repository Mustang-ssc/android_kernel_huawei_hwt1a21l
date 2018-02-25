/* < DTS2014042905347 zhaoyuxia 20140429 begin */
/* Copyright (c), Code HUAWEI. All rights reserved.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef HW_LCD_COMMON_H
#define HW_LCD_COMMON_H

/* < DTS2014111001776 zhaoyuxia 20141114 begin */
/* remove to hw_lcd_debug.h */
/* DTS2014111001776 zhaoyuxia 20141114 end > */
#include <linux/dsm_pub.h>
/* DTS2014051603610 zhaoyuxia 20140516 end > */

/* Add dynamic_log interface */
#define LCD_ERR  1
#define LCD_INFO 2
#define LCD_DBG  3

#define OPER_READ  (1)
#define OPER_WRITE (2)
#define MIPI_DCS_COMMAND (1<<0)
#define MIPI_GEN_COMMAND 4
#define MIPI_PATH_OPEN		1
#define MIPI_PATH_CLOSE	0
#ifdef CONFIG_DEBUG_FS
extern atomic_t mipi_path_status;
#endif
/* DTS2014050808504 daiyuhong 20140508 end > */

extern int lcd_debug_mask ;
/* < DTS2014080106240 renxigang 20140801 begin */
/*< DTS2015012205825  tianye/293347 20150122 begin*/
/* remove APR web LCD report log information  */
#ifdef CONFIG_HUAWEI_DSM
/*DTS2015012205825 tianye/293347 20150122 end >*/
struct lcd_pwr_status_t
{
	int panel_power_on;
	int lcd_dcm_pwr_status;
	struct timer_list lcd_dsm_t;
	struct tm tm_unblank;
	struct timeval tvl_unblank;
	struct tm tm_lcd_on;
	struct timeval tvl_lcd_on;
	struct tm tm_set_frame;
	struct timeval tvl_set_frame;
	struct tm tm_backlight;
	struct timeval tvl_backlight;
};
extern int lcd_dcm_pwr_status;
extern struct lcd_pwr_status_t lcd_pwr_status;
#define  LCD_PWR_STAT_GOOD 0x000f
/*< DTS2015012205825  tianye/293347 20150122 begin*/
/* remove APR web LCD report log information  */
#endif
/*DTS2015012205825 tianye/293347 20150122 end >*/
/* DTS2014080106240 renxigang 20140801 end > */

#ifndef LCD_LOG_ERR
#define LCD_LOG_ERR( x...)					\
do{											\
	if( lcd_debug_mask >= LCD_ERR )			\
	{										\
		printk(KERN_ERR "[LCD_ERR] " x);	\
	}										\
											\
}while(0)
#endif

#ifndef LCD_LOG_INFO
#define LCD_LOG_INFO( x...)					\
do{											\
	if( lcd_debug_mask >= LCD_INFO )		\
	{										\
		printk(KERN_ERR "[LCD_INFO] " x);	\
	}										\
											\
}while(0)
#endif

/* < DTS2014071607596 renxigang 20140717 begin */
/*KERNEL_HWFLOW is for controlling all the log of devices*/
#ifndef LCD_LOG_DBG
#define LCD_LOG_DBG( x...)					\
do{											\
	if( (KERNEL_HWFLOW) && (lcd_debug_mask >= LCD_DBG) )			\
	{										\
		printk(KERN_ERR "[LCD_DBG] " x);	\
	}										\
											\
}while(0)
#endif
/* DTS2014071607596 renxigang 20140717 end >*/

/* LCD_MDELAY will select mdelay or msleep according value */
#define LCD_MDELAY(time_ms)   	\
	do							\
	{ 							\
		if (time_ms>10)			\
			msleep(time_ms);	\
		else					\
			mdelay(time_ms);	\
	}while(0)	

#endif
/* DTS2014042905347 zhaoyuxia 20140429 end >*/
/* < DTS2014050808504 daiyuhong 20140508 begin */
/* < DTS2014111001776 zhaoyuxia 20141114 begin */
/* remove to hw_lcd_debug.h */
/* < DTS2014051603610 zhaoyuxia 20140516 begin */
extern struct dsm_client *lcd_dclient;
int lcd_report_dsm_err(int type,  int err_value,int add_value);
int mdss_record_dsm_err(u32 *dsi_status);
/* DTS2014111001776 zhaoyuxia 20141114 end > */
/* DTS2014051603610 zhaoyuxia 20140516 end > */
/* < DTS2014080106240 renxigang 20140801 begin */
/*< DTS2015012205825  tianye/293347 20150122 begin*/
/* remove APR web LCD report log information  */
#ifdef CONFIG_HUAWEI_DSM
void lcd_dcm_pwr_status_handler(unsigned long data);
#endif
/*DTS2015012205825 tianye/293347 20150122 end >*/
/* DTS2014080106240 renxigang 20140801 end > */
/* < DTS2014101301850 zhoujian 20141013 begin */
void mdp_underrun_dsm_report(unsigned long num,unsigned long underrun_cnt,int cpu_freq,unsigned long mdp_clk_rate,unsigned long clk_axi,unsigned long clk_ahb);
/* DTS2014101301850 zhoujian 20141013 end > */
/* DTS2014050808504 daiyuhong 20140508 end > */
