/* < DTS2014041202666  yangzicheng 20140412 begin */
#ifndef _I2C_QUP_H_
#define _I2C_QUP_H_

void qup_set_clk_freq(struct i2c_adapter *adap, int clk_freq) ;
int qup_get_clk_freq(struct i2c_adapter *adap);

#endif
/* DTS2014041202666  yangzicheng 20140412 end >*/
