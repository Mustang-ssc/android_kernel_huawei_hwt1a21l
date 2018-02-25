/*
 * cyttsp5_i2c.h
 * Cypress TrueTouch(TM) Standard Product V5 I2C driver module.
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

#ifndef _LINUX_CYTTSP5_I2C_H
#define _LINUX_CYTTSP5_I2C_H

#define CYTTSP5_I2C_NAME "cyttsp5_i2c_adapter"

#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mutex.h>

#define CY_I2C_DATA_SIZE  (3 * 256)

struct cyttsp5_i2c {
	struct i2c_client *client;
	u8 wr_buf[CY_I2C_DATA_SIZE];
	char const *id;
	struct mutex lock;
    struct regulator *vdd;
    int vbus_gpio;
    int vbus_gpio_en;
};

#endif /* _LINUX_CYTTSP5_I2C_H */
