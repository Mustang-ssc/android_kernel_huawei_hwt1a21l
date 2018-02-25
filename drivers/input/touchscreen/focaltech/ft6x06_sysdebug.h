/* < DTS2014020700267  caowei 20140210 begin */
#ifndef __FT6X06_SYSDEBUG_H__
#define __FT6X06_SYSDEBUG_H__
int ft6x06_create_sysfs(struct i2c_client *client);

/* < DTS2014021201959 caowei 20140213 begin */
void ft6x06_release_sysfs(struct i2c_client *client);
/* DTS2014021201959 caowei 20140213 end >*/

#endif
/* DTS2014020700267  caowei 20140210 end >*/
