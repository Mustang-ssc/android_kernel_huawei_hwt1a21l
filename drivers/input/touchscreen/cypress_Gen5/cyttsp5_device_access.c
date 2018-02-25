/*
 * cyttsp5_device_access.c
 * Cypress TrueTouch(TM) Standard Product V5 Device Access module.
 * Configuration and Test command/status user interface.
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
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include "cyttsp5_core.h"
#include "cyttsp5_device_access.h"
#include "cyttsp5_bus.h"

#ifdef CONFIG_TOUCHSCREEN_MMI_EQUIP
#include "cyttsp5_mmi_test.h"
#endif

char command_buf[CY_READ_TIMES][CY_READ_DATA_LEN];
static int read_times = 0;
static int read_times_OLPWC = 0;


//extern int get_boot_into_recovery_flag(void);

static ssize_t cyttsp5_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad = dev_get_drvdata(dev);
	u8 val;

	mutex_lock(&dad->sysfs_lock);
	val = dad->status;
	mutex_unlock(&dad->sysfs_lock);

	return scnprintf(buf, CY_MAX_PRBUF_SIZE, "%d\n", val);
}
static DEVICE_ATTR(status, 0444,
           cyttsp5_status_show, NULL);

struct cyttsp5_device_access_data *globe_dad;

unsigned char cyttsp5_command_status(void)
{
	return globe_dad->status;
}

static ssize_t cyttsp5_response_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad = dev_get_drvdata(dev);
	int i;
	ssize_t num_read;
	int index;

	mutex_lock(&dad->sysfs_lock);
	index = scnprintf(buf, CY_MAX_PRBUF_SIZE,
			"Status %d\n", dad->status);
	if (!dad->status)
		goto error;

	num_read = dad->response_length;

	for (i = 0; i < num_read; i++)
		index += scnprintf(buf + index, CY_MAX_PRBUF_SIZE - index,
				"0x%02X\n", dad->response_buf[i]);

	index += scnprintf(buf + index, CY_MAX_PRBUF_SIZE - index,
			"(%d bytes)\n", num_read);

error:
	mutex_unlock(&dad->sysfs_lock);
	return index;
}
static DEVICE_ATTR(response, 0444,
           cyttsp5_response_show, NULL);


int cyttsp5_command_response(u8 *buf)
{
	int ret = -1;
	ssize_t num_read;

	mutex_lock(&globe_dad->sysfs_lock);
	if (!globe_dad->status) {
		goto error;
	}

	num_read = globe_dad->response_length;
    
    memcpy(buf,globe_dad->response_buf,num_read);
	if (num_read > 2)/* when length <= 2 we can't get any data*/
		ret = num_read;
error:
	mutex_unlock(&globe_dad->sysfs_lock);
    return ret;
}


/*
 * Gets user input from sysfs and parse it
 * return size of parsed output buffer
 */
static int cyttsp5_ic_parse_input(struct device *dev, const char *buf,
		size_t buf_size, u8 *ic_buf, size_t ic_buf_size)
{
	const char *pbuf = buf;
	unsigned long value;
	char scan_buf[CYTTSP5_INPUT_ELEM_SZ];
	u32 i = 0;
	u32 j;
	int last = 0;
	int ret;

	TS_LOG_DEBUG( "%s: pbuf=%p buf=%p size=%d %s=%d buf=%s\n", __func__,
			pbuf, buf, (int) buf_size, "scan buf size",
			CYTTSP5_INPUT_ELEM_SZ, buf);

	while (pbuf <= (buf + buf_size)) {
		if (i >= CY_MAX_CONFIG_BYTES) {
			TS_LOG_ERR( "%s: %s size=%d max=%d\n", __func__,
					"Max cmd size exceeded", i,
					CY_MAX_CONFIG_BYTES);
			return -EINVAL;
		}
		if (i >= ic_buf_size) {
			TS_LOG_ERR( "%s: %s size=%d buf_size=%d\n", __func__,
					"Buffer size exceeded", i, ic_buf_size);
			return -EINVAL;
		}
		while (((*pbuf == ' ') || (*pbuf == ','))
				&& (pbuf < (buf + buf_size))) {
			last = *pbuf;
			pbuf++;
		}

		if (pbuf >= (buf + buf_size))
			break;

		memset(scan_buf, 0, CYTTSP5_INPUT_ELEM_SZ);
		if ((last == ',') && (*pbuf == ',')) {
			TS_LOG_ERR( "%s: %s \",,\" not allowed.\n", __func__,
					"Invalid data format.");
			return -EINVAL;
		}
		for (j = 0; j < (CYTTSP5_INPUT_ELEM_SZ - 1)
				&& (pbuf < (buf + buf_size))
				&& (*pbuf != ' ')
				&& (*pbuf != ','); j++) {
			last = *pbuf;
			scan_buf[j] = *pbuf++;
		}

		ret = kstrtoul(scan_buf, 16, &value);
		if (ret < 0) {
			TS_LOG_ERR( "%s: %s '%s' %s%s i=%d r=%d\n", __func__,
					"Invalid data format. ", scan_buf,
					"Use \"0xHH,...,0xHH\"", " instead.",
					i, ret);
			return ret;
		}

		ic_buf[i] = value;
		i++;
	}

	return i;
}

static ssize_t cyttsp5_command_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_device_access_data *dad = dev_get_drvdata(dev);
	ssize_t length;
	int rc;

	mutex_lock(&dad->sysfs_lock);
	dad->status = 0;
	dad->response_length = 0;
	length = cyttsp5_ic_parse_input(dev, buf, size, dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length <= 0) {
		TS_LOG_ERR( "%s: %s Group Data store\n", __func__,
				"Malformed input for");
		goto exit;
	}

	/* write ic_buf to log */
	cyttsp5_pr_buf(dev, dad->pr_buf, dad->ic_buf, length, "ic_buf");


	rc = cyttsp5_request_nonhid_user_cmd(dad->ttsp, 1, CY_MAX_PRBUF_SIZE,
			dad->response_buf, length, dad->ic_buf,
			&dad->response_length);
	if (rc) {
		dad->response_length = 0;
		TS_LOG_ERR( "%s: Failed to store command\n", __func__);
	} else {
		dad->status = 1;
	}

exit:
	mutex_unlock(&dad->sysfs_lock);
	TS_LOG_DEBUG( "%s: return size=%d\n", __func__, size);
	return size;
}
static DEVICE_ATTR(command, 0220,
    NULL, cyttsp5_command_store);


int cyttsp5_send_command(const char *buf)
{
	struct device *dev = &globe_dad->ttsp->dev;
	ssize_t length;
	int rc = 0;
	
	mutex_lock(&globe_dad->sysfs_lock);
	globe_dad->status = 0;
	globe_dad->response_length = 0;
	length = cyttsp5_ic_parse_input(dev, buf, strlen(buf), globe_dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length <= 0) {
		TS_LOG_ERR( "%s: %s Group Data store\n", __func__,
				"Malformed input for");
		rc = -EIO;
		goto exit;
	}

	/* write ic_buf to log */
	cyttsp5_pr_buf(dev, globe_dad->pr_buf, globe_dad->ic_buf, length, "ic_buf");


	rc = cyttsp5_request_nonhid_user_cmd(globe_dad->ttsp, 1, CY_MAX_PRBUF_SIZE,
			globe_dad->response_buf, length, globe_dad->ic_buf,
			&globe_dad->response_length);
	if (rc) {
		globe_dad->response_length = 0;
		TS_LOG_ERR( "%s: Failed to store command\n", __func__);
	} else {
		globe_dad->status = 1;
	}

exit:
	mutex_unlock(&globe_dad->sysfs_lock);
	return rc;
}



static int cyttsp5_change_scan_status(int value)
{
    int ret = -1;
    
    const char *Command_function[] = {
        "0x04 0x00 0x05 0x00 0x2F 0x00 0x03",  //Suspend Scan
        "0x04 0x00 0x05 0x00 0x2F 0x00 0x2A",  //Panel Scan
        "0x04 0x00 0x05 0x00 0x2F 0x00 0x04"   //Resume scan
    };

    if(value != 0 && value != 1 && value != 2){
         TS_LOG_ERR( "%s Input value is error,value = %d.\n",__func__,value);
         return -1;
    }

    ret = cyttsp5_send_command(Command_function[value]);
    if (ret < 0) {
        TS_LOG_ERR( "%s Failed to change scan status.\n",__func__);
        return -1;
    }
    
    return 0;
}

static void cyttsp5_build_test_command(char *cmd,char *data_id,int data_len)
{
    int i,j;
    int readlength = 0;
    char *cmd_head = "0x04 0x00 0x0A 0x00 0x2F 0x00 ";
    char tmp_cmd_str[CY_READ_DATA_LEN] = {0};
    char tmp_cmd[32] = {0};
    /*
    Data ID As show:
    0x00 Mutual-cap Raw Data
    0x01 Mutual-cap Baseline Data
    0x02 Mutual-cap Difference/Signal Data
    0x03 Self-cap Raw Data
    0x04 Self-cap Baseline Data
    0x05 Self-cap Difference/Signal Data
    */
    
    /*read 0x64 bytes once*/

    /* "0x04 0x00 0x0A 0x00 0x2F 0x00 0x2B|0x00 0x00|0x64 0x00|0x00" */
    /*************************************|read len |offset   |*******/
    /**************Command Head **********|lsb, msb |lsb, msb | Data ID|*/

    /* read raw data command
    "0x04 0x00 0x0A 0x00 0x2F 0x00 0x2B 0x00 0x00 0x64 0x00 0x00",
    "0x04 0x00 0x0A 0x00 0x2F 0x00 0x2B 0x64 0x00 0x64 0x00 0x00",
    "0x04 0x00 0x0A 0x00 0x2F 0x00 0x2B 0xc8 0x00 0x64 0x00 0x00",
    "0x04 0x00 0x0A 0x00 0x2F 0x00 0x2B 0x2C 0x01 0x64 0x00 0x00",
    "0x04 0x00 0x0A 0x00 0x2F 0x00 0x2B 0x90 0x01 0x64 0x00 0x00",
    "0x04 0x00 0x0A 0x00 0x2F 0x00 0x2B 0xF4 0x01 0x64 0x00 0x00",
    "0x04 0x00 0x0A 0x00 0x2F 0x00 0x2B 0x58 0x02 0x64 0x00 0x00",
    "0x04 0x00 0x0A 0x00 0x2F 0x00 0x2B 0xBC 0x02 0x0E 0x00 0x00"
    */ 
    memset(tmp_cmd,0,sizeof(tmp_cmd));
    memset(tmp_cmd_str,0,sizeof(tmp_cmd_str));
    
    for(i = 0,j = 0;i < data_len;i += CY_READ_DATA_LEN,j++){
        if(data_len - i > CY_READ_DATA_LEN){
            readlength = CY_READ_DATA_LEN;
        }else{
            readlength = data_len - i;
        }
        
        strcpy(tmp_cmd_str,cmd_head);
        strcat(tmp_cmd_str,cmd);
        sprintf(tmp_cmd," 0x%02X 0x%02X 0x%02X 0x%02X ", i%256,i/256,readlength,0);
        strcat(tmp_cmd_str,tmp_cmd);
        strcat(tmp_cmd_str,data_id);
        
        strcpy(globe_dad->read_cmd[j],tmp_cmd_str);
        
        
        memset(tmp_cmd,0,sizeof(tmp_cmd));
        memset(tmp_cmd_str,0,sizeof(tmp_cmd_str));
    }
}

static int cyttsp5_get_SensorData(char *id,char *cmd,int data_len,int data_byte,int other)
{
    int i,j;
    int ret = -1;
    int read_times = 0;
    u8 tmp_buf[CY_READ_DATA_LEN * 2 + CY_DATA_HEAD] = { 0 };
    const char *open_test_command =  "0x04 0x00 0x07 0x00 0x2F 0x00 0x26 0x03 0x00";


    memset(globe_dad->read_cmd,0,sizeof(globe_dad->read_cmd));
    memset(globe_dad->data_buf,0,sizeof(globe_dad->data_buf));

    cyttsp5_build_test_command(cmd,id,data_len);

    //Suspend Scan
    ret = cyttsp5_change_scan_status(0);
    if (ret < 0 ) {
        TS_LOG_ERR( "%s Failed to suspend scan.\n",__func__);
        return -1;
    }
    
    mdelay(30);

    //Panel Scan
    ret = cyttsp5_change_scan_status(1);
    if (ret < 0 ) {
        TS_LOG_ERR( "%s Failed to scan panel once.\n",__func__);
        goto exit;
    }
    
    mdelay(30);

    read_times = (data_len > CY_READ_DATA_LEN) ? CY_READ_TIMES : 1;

    //OPEN TEST
    if(other == CYTTSP5_GET_OPENS){
        ret = cyttsp5_send_command(open_test_command);
        if(ret < 0){
            TS_LOG_ERR("%s Failed to send command.\n",__func__);
            goto exit;
        }
    }

    for(i = 0;i < read_times;i++){
		memset(tmp_buf,0,sizeof(tmp_buf));
        mdelay(30);
        ret = cyttsp5_send_command(globe_dad->read_cmd[i]);
        if(ret < 0 ){
            TS_LOG_ERR("%s Failed to send command.\n",__func__);
            goto exit;
        }

        mdelay(30);
        ret = cyttsp5_command_response(tmp_buf);
        if(ret < 0){
            TS_LOG_ERR( "%s Failed to get response.\n",__func__);
            goto exit;
        }
		for(j = CY_DATA_HEAD;j < globe_dad->response_length;j += data_byte){
			globe_dad->data_buf[i * CY_READ_DATA_LEN + (j - CY_DATA_HEAD)/data_byte] 
				= (data_byte -1) * tmp_buf[j + 1] * 256 +  tmp_buf[j];
			if(globe_dad->data_buf[i * CY_READ_DATA_LEN + (j - CY_DATA_HEAD)/data_byte] > 32767){
				globe_dad->data_buf[i * CY_READ_DATA_LEN + (j - CY_DATA_HEAD)/data_byte] -= 65535;
			}
		}
    }
    
    //Print data
    for(i = 0; i < data_len; i++) {
		if((i+1)%CY_RX_NUMBER == 0) {
			printk("%6d\n", globe_dad->data_buf[i]);
		} else {
			printk("%6d", globe_dad->data_buf[i]);
		}
	}
    
exit:
    //Resume scan
    cyttsp5_change_scan_status(2);

	return ret;
}


static int cyttsp5_get_rawdata(void)
{
    int ret  = -1;
    int i = 0;
    int check_flag = 0;
    
    ret = cyttsp5_get_SensorData("0x00","0x2B",CY_TX_NUMBER * CY_RX_NUMBER,CYTTSP5_DATA_TWO_BYTE,0);
    if(ret < 0){
        TS_LOG_ERR("%s error.\n",__func__);
        goto exit;
    }
    
    for(i = 0;i < CY_TX_NUMBER * CY_RX_NUMBER;i++){
        if(globe_dad->data_buf[i] > CY_MAX_RAW_DATA || globe_dad->data_buf[i] < CY_MIN_RAW_DATA){
            TS_LOG_ERR( "%s check is failed.\n",__func__);
            check_flag = -1;
            break;
        }
    }
    
    return check_flag;
    
exit:
    return -1;
}

static int cyttsp5_get_self_rawdata(void)
{
    int ret  = -1;
    int i = 0;
    int check_flag = 0;
    
    
    ret = cyttsp5_get_SensorData("0x03","0x2B",CY_TX_NUMBER + CY_RX_NUMBER,CYTTSP5_DATA_TWO_BYTE,0);
    if(ret < 0){
        TS_LOG_ERR("%s error.\n",__func__);
        goto exit;
    }
    
    for(i = 0;i < CY_TX_NUMBER * CY_RX_NUMBER;i++){
        if(globe_dad->data_buf[i] > CY_MAX_RAW_DATA || globe_dad->data_buf[i] < CY_MIN_RAW_DATA){
            TS_LOG_ERR( "%s check is failed.\n",__func__);
            check_flag = -1;
            break;
        }
    }
    
    return check_flag;
    
exit:
    return -1;

}

static int cyttsp5_get_mutual_noise(void)
{
	int i,k;
	int ret = -1;
    int * min_raw = NULL;
    int * max_raw = NULL;

    max_raw = kzalloc(sizeof(int) * CY_TX_NUMBER * CY_RX_NUMBER,GFP_KERNEL);
    if(max_raw == NULL){
        TS_LOG_ERR( "%s: Error, kzalloc\n", __func__);
        return -1;
    }
    
    min_raw = kzalloc(sizeof(int) * CY_TX_NUMBER * CY_RX_NUMBER,GFP_KERNEL);
    if(min_raw == NULL){
        TS_LOG_ERR( "%s: Error, kzalloc\n", __func__);
        kfree(max_raw);
        return -1;
    }

	for (k = 0; k < 5; k++) {
		ret = cyttsp5_get_rawdata();
		if (ret < 0) {
			TS_LOG_ERR( "%s get rawdata failed.\n",__func__);
			goto exit;
		}
		if (k ==0 ) {
			for (i = 0; i < CY_TX_NUMBER * CY_RX_NUMBER; i++) {
		    	max_raw[i] = globe_dad->data_buf[i];
				min_raw[i] = globe_dad->data_buf[i];
    		}
		} else {
			for (i = 0; i < CY_TX_NUMBER * CY_RX_NUMBER; i++) {
		    	max_raw[i] = max(globe_dad->data_buf[i],max_raw[i]);
				min_raw[i] = min(globe_dad->data_buf[i],min_raw[i]);
    		}
		}
	}
    
	for (i = 0; i < CY_TX_NUMBER * CY_RX_NUMBER; i++) {
		globe_dad->data_buf[i] = max_raw[i] - min_raw[i];
	}
    
exit:
    kfree(max_raw);
    kfree(min_raw);
	max_raw = min_raw = NULL;
	return ret;
}


static int cyttsp5_get_self_noise(void)
{
	int i,k;
	int ret = 0;
    int * min_raw = NULL;
    int * max_raw = NULL;


    max_raw = kzalloc(sizeof(int) * CY_TX_NUMBER * CY_RX_NUMBER,GFP_KERNEL);
    if(max_raw == NULL){
        TS_LOG_ERR( "%s: Error, kzalloc\n", __func__);
        return -1;
    }
    
    min_raw = kzalloc(sizeof(int) * CY_TX_NUMBER * CY_RX_NUMBER,GFP_KERNEL);
    if(min_raw == NULL){
        TS_LOG_ERR( "%s: Error, kzalloc\n", __func__);
        kfree(max_raw);
        return -1;
    }

	for (k = 0; k < 5; k++) {
		ret = cyttsp5_get_self_rawdata();
		if (ret < 0) {
			TS_LOG_ERR( "%s get self rawdata failed.\n",__func__);
			goto exit;
		}
		if (k == 0) {
			for (i = 0; i < CY_TX_NUMBER + CY_RX_NUMBER; i++) {
				max_raw[i] = globe_dad->data_buf[i];
				min_raw[i] = globe_dad->data_buf[i];
			}
		} else {
			for (i = 0; i < CY_TX_NUMBER + CY_RX_NUMBER; i++) {
				max_raw[i] = max(globe_dad->data_buf[i],max_raw[i]);
				min_raw[i] = min(globe_dad->data_buf[i],min_raw[i]);
			}
		}
	}
    
	for (i = 0; i < CY_TX_NUMBER + CY_RX_NUMBER; i++) {
		globe_dad->data_buf[i] = max_raw[i] - min_raw[i];
	}
    
exit:
	kfree(max_raw);
	kfree(min_raw);
	max_raw = min_raw = NULL;
	return ret;
}

static int cyttsp5_get_LocalPWC(void)
{
    int ret  = -1;
    int i = 0;
    int check_flag = 0;
    
    ret = cyttsp5_get_SensorData("0x00","0x24",CY_TX_NUMBER * CY_RX_NUMBER + CY_TX_NUMBER + 4,
                                    CYTTSP5_DATA_ONE_BYTE,0);
    
    if(ret < 0){
        TS_LOG_ERR("%s error.\n",__func__);
        goto exit;
    }
    
    for(i = CY_TX_NUMBER + 4;i < CY_TX_NUMBER * CY_RX_NUMBER + CY_TX_NUMBER + 4;i++){
        if(globe_dad->data_buf[i] > CY_MAX_LPWC_DATA || globe_dad->data_buf[i] < CY_MIN_LPWC_DATA){
            TS_LOG_ERR( "%s check is failed.\n",__func__);
            check_flag = -1;
            break;
        }
    }
    
    return check_flag;
    
exit:
    return -1;
}

static int cyttsp5_get_GlobalGIDAC(void)
{
    int ret  = -1;
    int i = 0;
    int range_flag = 0;
    int delta_flag = 0;
    int GiDAC_MAX  = 55;
    int GiDAC_MIN  = 22;
    int GiDAC_diff  = 6;
    
    ret = cyttsp5_get_SensorData("0x00","0x24",CY_TX_NUMBER + 4,CYTTSP5_DATA_ONE_BYTE,0);
    if(ret < 0){
        TS_LOG_ERR("%s error.\n",__func__);
        goto exit;
    }
    
    for(i = 0;i < CY_TX_NUMBER + 4; i++){
        if(globe_dad->data_buf[i] > GiDAC_MAX || globe_dad->data_buf[i] < GiDAC_MIN){
            TS_LOG_ERR( "%s check is failed.\n",__func__);
            range_flag = -1;
            break;
        }
    }

    for(i = 0;i < CY_TX_NUMBER;i++){
        if(abs(globe_dad->data_buf[i] - globe_dad->data_buf[i+1]) > GiDAC_diff){ // MMI is 6
            TS_LOG_ERR( "%s uniformity check is failed.\n",__func__);
            delta_flag = -1;
            break;
        }
    }

    ret = (range_flag < 0 ||  delta_flag < 0)  ? -1  : 0;
    
    return ret;
    
exit:
    return -1;
}

static int cyttsp5_get_SelfLocalPWC(void)
{
    int ret  = -1;
    int i = 0;
    int tx_flag = 0;
    int rx_flag = 0;
    int rx_tx_uniformity_max = 35;

    ret = cyttsp5_get_SensorData("0x01","0x24",CY_TX_NUMBER + CY_RX_NUMBER,CYTTSP5_DATA_ONE_BYTE,0);
    if(ret < 0){
        TS_LOG_ERR("%s error.\n",__func__);
        goto exit;
    }


    for(i = 0;i < CY_TX_NUMBER - 1;i++){
        if(abs(globe_dad->data_buf[i] - globe_dad->data_buf[i+1])> rx_tx_uniformity_max){
            TS_LOG_ERR( "%s tx uniformity check is failed.\n",__func__);
            tx_flag = -1;
            goto exit;
        }
    }

    for(i = CY_TX_NUMBER;i < CY_TX_NUMBER + CY_RX_NUMBER - 1;i++){
        if(abs(globe_dad->data_buf[i] - globe_dad->data_buf[i+1])> rx_tx_uniformity_max){
            TS_LOG_ERR( "%s rx uniformity check is failed.\n",__func__);
            rx_flag = -1;
            break;
        }
    }

    ret = (tx_flag < 0 ||  rx_flag < 0)  ? -1  : 0;
    
    return ret;
    
exit:
    return -1;
}

static int cyttsp5_get_OPENS(void)
{
    int ret  = -1;
    int i = 0;
    int tx_check_count  = 0;
    int rx_check_count  = 0;
    int tx_rx_diff_max[4]  = {6, 7, 8, 9};
    int tx_rx_E_by_E[4]    = {9, 10, 11, 12};
    int tx_rx_count_max = 6;
    int * opens_diff = NULL;
    
    opens_diff = kzalloc(sizeof(int) * CY_TX_NUMBER * CY_RX_NUMBER,GFP_KERNEL);
    if(opens_diff == NULL){
        TS_LOG_ERR( "%s: Error, kzalloc\n", __func__);
        return -1;
    }

    ret = cyttsp5_get_SensorData("0x03","0x27",CY_TX_NUMBER * CY_RX_NUMBER, CYTTSP5_DATA_ONE_BYTE, 1);
    if(ret < 0){
        TS_LOG_ERR("%s error.\n",__func__);
        goto exit;
    }

    //rx uniformity
    for(i = 0;i< CY_RX_NUMBER * CY_TX_NUMBER -1 ;i++) {
        if((i+1)%CY_RX_NUMBER != 0){
            opens_diff[i] = (int)(globe_dad->data_buf[i] - globe_dad->data_buf[i+1]);//左右相减
        }else{
            opens_diff[i] = 0;
        }
    }

    for(i = 0;i< CY_RX_NUMBER * CY_TX_NUMBER ;i++) {
        //uniformity E-By-E
        if(abs(opens_diff[i]) > tx_rx_E_by_E[globe_dad->check_flag]){
            TS_LOG_ERR( "%s rx uniformity E-By-E check is failed.\n",__func__);
            goto exit;
        }
        
        if((i % CY_RX_NUMBER) == 10){//skip rx10-rx11
            continue;
        }else{
            //uniformity
            if(abs(opens_diff[i]) > tx_rx_diff_max[globe_dad->check_flag]){
                rx_check_count++;
            }
        }
    }

    //tx uniformity
    for(i = 0;i< CY_RX_NUMBER * (CY_TX_NUMBER -1); i++) {
		opens_diff[i] = (int)(globe_dad->data_buf[i] - globe_dad->data_buf[i + CY_RX_NUMBER]);//上下相减
    }

    for(i = 0;i< CY_RX_NUMBER * CY_TX_NUMBER ;i++) {
        //uniformity E-By-E
        if(abs(opens_diff[i]) > tx_rx_E_by_E[globe_dad->check_flag]){
            TS_LOG_ERR( "%s tx uniformity E-By-E check is failed.\n",__func__);
            goto exit;
        }

        if((i / CY_RX_NUMBER) == 16){//skip tx16-tx17
            continue;
        }else{
            //uniformity
            if(abs(opens_diff[i]) > tx_rx_diff_max[globe_dad->check_flag]){
                tx_check_count++;
            }
        }
    }

    if(rx_check_count > tx_rx_count_max){
        TS_LOG_ERR( "%s rx uniformity check is failed. globe_dad->check_flag=%d\n",
			__func__, globe_dad->check_flag);
        goto exit;
    }
    
    if(tx_check_count > tx_rx_count_max){
        TS_LOG_ERR( "%s tx uniformity check is failed. globe_dad->check_flag=%d\n",
			__func__, globe_dad->check_flag);
        goto exit;
    }
 	
	kfree(opens_diff);
	opens_diff = NULL;
    return 0;
    
exit:
	kfree(opens_diff);
	opens_diff = NULL;
    return -1;
}

static int cyttsp5_get_mutual_baseline(void)
{
    int ret  = -1;
    
    ret = cyttsp5_get_SensorData("0x01","0x2B",CY_TX_NUMBER * CY_RX_NUMBER,CYTTSP5_DATA_TWO_BYTE,0);
    if(ret < 0){
        TS_LOG_ERR("%s error.\n",__func__);
        goto exit;
    }
    
    return ret;
    
exit:
    return -1;
}

static int cyttsp5_get_self_baseline(void)
{
    int ret  = -1;
    
    ret = cyttsp5_get_SensorData("0x04","0x2B",CY_TX_NUMBER + CY_RX_NUMBER,CYTTSP5_DATA_TWO_BYTE,0);
    if(ret < 0){
        TS_LOG_ERR("%s error.\n",__func__);
        goto exit;
    }
    
    return ret;
    
exit:
    return -1;
}

static int cyttsp5_get_mutual_diff(void)
{
    int ret  = -1;
    
    ret = cyttsp5_get_SensorData("0x02","0x2B",CY_TX_NUMBER * CY_RX_NUMBER,CYTTSP5_DATA_TWO_BYTE,0);
    if(ret < 0){
        TS_LOG_ERR("%s error.\n",__func__);
        goto exit;
    }
    
    return ret;
    
exit:
    return -1;
}

static int cyttsp5_get_self_diff(void)
{
    int ret  = -1;
    
    ret = cyttsp5_get_SensorData("0x05","0x2B",CY_TX_NUMBER + CY_RX_NUMBER,CYTTSP5_DATA_TWO_BYTE,0);
    if(ret < 0){
        TS_LOG_ERR("%s error.\n",__func__);
        goto exit;
    }
    
    return ret;
    
exit:
    return -1;
}

static int cyttsp5_short_test(void)
{
    int ret = -1;
    int i = 0;
    int check_flag = 1;
    u8 data_buf[100] = {0};
    const char* short_command = "0x04 0x00 0x07 0x00 0x2F 0x00 0x26 0x04 0x00";

	memset(globe_dad->data_buf, 0, sizeof(globe_dad->data_buf));
    //Suspend Scan
    ret = cyttsp5_change_scan_status(0);
    if(ret < 0 ){
        TS_LOG_ERR( "%s Failed to suspend scan.\n",__func__);
        return -1;
	}

    //SHORT TEST
    mdelay(30);
    ret = cyttsp5_send_command(short_command);
    if(ret < 0){
        TS_LOG_ERR("%s Failed to send command.\n",__func__);
        goto exit;
	}

	mdelay(30);
    ret = cyttsp5_command_response(data_buf);
    if(ret < 0){
        TS_LOG_ERR( "%s Failed to get response.\n",__func__);
        goto exit;
    }

	for (i = 0; i < 7; i++) {
		globe_dad->data_buf[i] = data_buf[i];
	}
	if (data_buf[0] == 0x07
		&& data_buf[2] == 0x1F
		&& (data_buf[4] == 0xA6 || data_buf[4] == 0x26)) {
			if (data_buf[5] != 0x0 || data_buf[6] != 0x0) {
				TS_LOG_ERR("%s Short test failed.\n",__func__);
				check_flag = 0;
		}		
	} else {
		TS_LOG_ERR("%s Short test failed.\n",__func__);
		check_flag = 0;
	}

exit:
    //Resume scan
    cyttsp5_change_scan_status(2);
	
	if (check_flag == 0) {
		for (i = 0; i < 7; i++) {
			printk("0x%2x ", data_buf[i]);
		}
		printk("\n");

		return -2;
	}

	return ret;
}

static int cyttsp5_get_system_info(void)
{
    int ret = -1;
    int i = 0;
    u8 data_buf[51] = {0};/*short need read 51*/
    const char* systeminfo_command = "0x04 0x00 0x05 0x00 0x2F 0x00 0x02";
	
    memset(data_buf, 0, sizeof(data_buf));
    memset(globe_dad->data_buf, 0, sizeof(globe_dad->data_buf));

    //Send Opens Test command to command sysfs node.
    ret = cyttsp5_send_command(systeminfo_command);
    if (ret < 0) {
        TS_LOG_ERR("%s Failed to send command.\n",__func__);
        goto exit;
    }
    
	mdelay(30);
    
    ret = cyttsp5_command_response(data_buf);
    if(ret < 0){
        TS_LOG_ERR( "%s Failed to get response.\n",__func__);
        goto exit;
    }


	for (i = 0; i < 51; i++) {
		globe_dad->data_buf[i] = data_buf[i];
	}

exit:
    
	return ret;	
}

static int cyttsp5_i2c_check(void)
{
	//struct cyttsp5_device *ttsp = globe_dad->ttsp;
	int rc = 0;
    const char *ping_cmd = "0x04 0x00 0x05 0x00 0x2F 0x00 0x00";
    char data_buf[5] = {0};
	/*
	rc = cyttsp5_request_get_hid_desc(ttsp, 0);
	if (rc == -EIO) {
		TS_LOG_ERR("%s I2C test failed.\n",__func__);
		return rc;
	}
    */
    rc = cyttsp5_send_command(ping_cmd);
    if(rc < 0){
        TS_LOG_ERR("%s Failed to send command.\n",__func__);
        goto exit;
    }

    rc = cyttsp5_command_response(data_buf);
    if(rc < 0){
        TS_LOG_ERR( "%s Failed to get response.\n",__func__);
        goto exit;
    }

    if(  data_buf[0] == 0x05 && data_buf[1] == 0x00 
     &&  data_buf[2] == 0x1F && data_buf[3] == 0x00 
     && (data_buf[4] == 0x80 || data_buf[4] == 0x00)){
        rc = 0;
    }else{
        rc  = -1;
    }

exit:

	return rc;
}

static struct cyttsp5_check_func_ cyttsp5_check_func[] = {
	{CY_I2C_CHECK, cyttsp5_i2c_check},
	{CY_RAW_CHECK, cyttsp5_get_rawdata},
	{CY_OPEN_CHECK,cyttsp5_get_GlobalGIDAC},
	{CY_LPWC_CHECK,cyttsp5_get_LocalPWC},
	{CY_OLPWC_CHECK,cyttsp5_get_OPENS},
	{CY_SOLPWC_CHECK,cyttsp5_get_SelfLocalPWC},
	{CY_SELFRAW_CHECK,cyttsp5_get_self_rawdata},
	{CY_SHORT_CHECK, cyttsp5_short_test},
    {CY_MUT_BASE_CHECK, cyttsp5_get_mutual_baseline},
    {CY_SELF_BASE_CHECK, cyttsp5_get_self_baseline},
    {CY_MUT_DIFF_CHECK, cyttsp5_get_mutual_diff},
    {CY_SELF_DIFF_CHECK, cyttsp5_get_self_diff},
    {CY_MUT_NOISE_CHECK, cyttsp5_get_mutual_noise},
    {CY_SELF_NOISE_CHECK, cyttsp5_get_self_noise},
    {CY_SYSTEM_INFO_CHECK, cyttsp5_get_system_info},
    {CY_CHECK_MAX, NULL},
};

int cyttsp5_set_check_flag(int flag)
{
	//0==>TP Check 1==>MMI1  2==>MMI2 3==>Factory Menu
	globe_dad->check_flag = flag;
	return 0;
}

int cyttsp5_check_items(enum cyttsp5_sensor_check item, bool reset)
{
	int retry = 0;
	int check_flag = -1;
	if (item < CY_I2C_CHECK || item > CY_CHECK_MAX) {
		printk("%s: invalid input item\n", __func__);
		return -EINVAL;
	}
	if (cyttsp5_check_func[item].func) {
		for(retry = 0; retry < CHECK_RETRY_TIMES; retry++) {
			check_flag = cyttsp5_check_func[item].func();
			if (check_flag >= 0) {
				break;
			} else {
				printk("%s:check item:%d, %dtimes failed\n",__func__, item, retry+1);
				if (reset) {
					cyttsp5_hw_reset_export();
					msleep(200);
				}
			}
		}
	}
	
	return check_flag;
}

static int cyttsp5_data_show(char *buf, int num, int offset, bool long_data)
{
	int index = 0;
	int i = 0;
	if (long_data) {
		for (i = 0; i < num; i++) {
			if((i+1) % CY_RX_NUMBER == 0){
				index += scnprintf(buf + index, (size_t)(CY_MAX_PRBUF_SIZE - index),
					"%6d\n", globe_dad->data_buf[i + offset]);
			}else{
				index += scnprintf(buf + index, (size_t)(CY_MAX_PRBUF_SIZE - index), 
					"%6d ", globe_dad->data_buf[i + offset]);
			}	
		}
	} else {
		for (i = 0; i < num; i++) {
			if((i+1) % CY_RX_NUMBER == 0){
				index += scnprintf(buf + index, (size_t)(CY_MAX_PRBUF_SIZE - index),
					"%4d\n", globe_dad->data_buf[i + offset]);
			}else{
				index += scnprintf(buf + index, (size_t)(CY_MAX_PRBUF_SIZE - index), 
					"%4d ", globe_dad->data_buf[i + offset]);
			}	
		}
	}
	return index;
}

/*
  print olpwc txdiff
  This func need called follow cyttsp5_get_OPENS 
*/
static int cyttsp5_olpwc_txdiff_show(char *buf)
{
	int *tx_diff = NULL;
	int i = 0;
	int index = 0;

	tx_diff = kzalloc(sizeof(int) * CY_TX_NUMBER * CY_RX_NUMBER,GFP_KERNEL);
	if (tx_diff == NULL) {
		printk("%s: malloc txdiff buf failed\n", __func__);
		return -1;
	}
	
	for(i = 0;i< CY_RX_NUMBER * (CY_TX_NUMBER -1); i++) {
		tx_diff[i] = (int)(globe_dad->data_buf[i] - globe_dad->data_buf[i + CY_RX_NUMBER]);//上下相减
	}

	for (i = 0; i < CY_RX_NUMBER * CY_TX_NUMBER; i++) {
		if((i+1) % CY_RX_NUMBER == 0){
			index += scnprintf(buf + index, (size_t)(CY_MAX_PRBUF_SIZE - index),
				"%4d\n", tx_diff[i]);
		}else{
			index += scnprintf(buf + index, (size_t)(CY_MAX_PRBUF_SIZE - index), 
				"%4d ", tx_diff[i]);
		}
	}
	kfree(tx_diff);
	tx_diff = NULL;
	
	return index;	
}

/*
  print olpwc rxdiff
  This func need called follow cyttsp5_get_OPENS 
*/
static int cyttsp5_olpwc_rxdiff_show(char *buf)
{
	int *rx_diff = NULL;
	int i = 0;
	int index = 0;

	rx_diff = kzalloc(sizeof(int) * CY_TX_NUMBER * CY_RX_NUMBER,GFP_KERNEL);
	if (rx_diff == NULL) {
		printk("%s: malloc rxdiff buf failed\n", __func__);
		return -1;
	}

	for(i = 0;i< CY_RX_NUMBER * CY_TX_NUMBER -1 ;i++) {
		if((i+1)%CY_RX_NUMBER != 0){
			rx_diff[i] = (int)(globe_dad->data_buf[i]  - globe_dad->data_buf[i+1]);//左右相减
		} else {
			rx_diff[i] = 0;
		}
 	}

	for (i = 0; i < CY_RX_NUMBER * CY_TX_NUMBER; i++) {
		if((i+1) % CY_RX_NUMBER == 0){
			index += scnprintf(buf + index, (size_t)(CY_MAX_PRBUF_SIZE - index),
				"%4d\n", rx_diff[i]);
		}else{
			index += scnprintf(buf + index, (size_t)(CY_MAX_PRBUF_SIZE - index), 
				"%4d ", rx_diff[i]);
		}	
	}
	kfree(rx_diff);
	rx_diff = NULL;
	return index;	
	
}

static ssize_t cyttsp5_rawdata_check_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_device_access_data *dad = dev_get_drvdata(dev);
    
	int data_off = 0;
	int data_num = 0;
	bool long_data = false;
	
	switch (dad ->check_index) {
	case CY_RAW_CHECK:
	case CY_MUT_DIFF_CHECK:
	case CY_MUT_NOISE_CHECK:
		long_data = true;/*long data print we need read 2 times*/
		data_num = CY_TX_NUMBER * CY_RX_NUMBER / 2;
		data_off = read_times ? data_num : 0;
        	read_times = !read_times;
		break;
	case CY_MUT_BASE_CHECK:
		data_num = CY_TX_NUMBER * CY_RX_NUMBER;
		break;
	case CY_LPWC_CHECK:
		data_num = CY_TX_NUMBER * CY_RX_NUMBER;
		data_off = 40;
		break;
	case CY_OPEN_CHECK:
		data_num = CY_TX_NUMBER;
		break;
	case CY_SELFRAW_CHECK:
	case CY_SELF_BASE_CHECK:
	case CY_SELF_DIFF_CHECK:
	case CY_SELF_NOISE_CHECK:
		long_data = true;
		data_num = CY_TX_NUMBER + CY_RX_NUMBER;
		break;
	case CY_SHORT_CHECK:
		data_num = CY_TX_NUMBER + CY_RX_NUMBER;
		break;
	case CY_SOLPWC_CHECK:
		data_num = CY_TX_NUMBER + CY_RX_NUMBER;
		break;
	case CY_OLPWC_CHECK:
		if (read_times_OLPWC == 0) {
			data_num = CY_TX_NUMBER * CY_RX_NUMBER;
			read_times_OLPWC = 1;
		} else if(read_times_OLPWC == 1) {
			read_times_OLPWC = 2;
			return cyttsp5_olpwc_rxdiff_show(buf);
		} else {
			read_times_OLPWC = 0;
			return cyttsp5_olpwc_txdiff_show(buf);
		}
		break;
	case CY_SYSTEM_INFO_CHECK:
		data_num = 51;
		break;
	default:
		return -1;
	}

	return cyttsp5_data_show(buf, data_num, data_off, long_data);
}


static ssize_t cyttsp5_rawdata_check_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned int value = 0;
	read_times = 0;
	read_times_OLPWC = 0;
	
    value = simple_strtoul(buf,NULL,10);
    if(value >= CY_CHECK_MAX || value == 0){
        TS_LOG_ERR( "%s input value is error.\n",__func__);
        return -EINVAL;
    }
    
    globe_dad->check_index = value;

	cyttsp5_check_items(value, 1);

    return size;
}


static DEVICE_ATTR(rawdata_check, 0664,
    cyttsp5_rawdata_check_show, cyttsp5_rawdata_check_store);



#ifdef TTHE_TUNER_SUPPORT
/*
 * Execute scan command
 */
static int cyttsp5_exec_scan_cmd_(struct cyttsp5_device *ttsp)
{
	int rc;

	rc =  cyttsp5_request_nonhid_exec_panel_scan(ttsp, 0);
	if (rc < 0)
		TS_LOG_ERR( "%s: Heatmap start scan failed r=%d\n",
			__func__, rc);
	return rc;
}

/*
 * Retrieve panel data command
 */
static int cyttsp5_ret_scan_data_cmd_(struct cyttsp5_device *ttsp,
		u16 read_offset, u16 read_count, u8 data_id, u8 *response,
		u8 *config, u16 *actual_read_len, u8 *return_buf)
{
	int rc;

	rc = cyttsp5_request_nonhid_retrieve_panel_scan(ttsp, 0, read_offset,
			read_count, data_id, response, config, actual_read_len,
			return_buf);
	if (rc < 0)
		TS_LOG_ERR( "%s: Retrieve scan data failed r=%d\n",
				__func__, rc);
	return rc;
}

#define MAX_ELEM 100
static ssize_t tthe_get_panel_data_debugfs_read(struct file *filp,
		char __user *buf, size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_data *dad = filp->private_data;
	struct cyttsp5_device *ttsp;
	struct device *dev;
	u8 config;
	u16 actual_read_len;
	u16 length;
	u8 element_size = 0;
	u8 *buf_offset;
	u8 *buf_out;
	int elem;
	int elem_offset = 0;
	int print_idx = 0;
	int rc;
	int rc1;
	int i;

	mutex_lock(&dad->debugfs_lock);
	ttsp = dad->ttsp;
	dev = &ttsp->dev;
	buf_out = dad->tthe_get_panel_data_buf;
	if (!buf_out)
		goto release_mutex;


	rc = cyttsp5_request_exclusive(ttsp, CY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0)
		goto put_runtime;

	if (dad->heatmap.scan_start) {
		/* Start scan */
		rc = cyttsp5_exec_scan_cmd_(ttsp);
		if (rc < 0)
			goto release_exclusive;
	}

	elem = dad->heatmap.num_element;

	if(elem > MAX_ELEM) {
		rc = cyttsp5_ret_scan_data_cmd_(ttsp, elem_offset, MAX_ELEM,
			dad->heatmap.data_type, dad->ic_buf, &config,
			&actual_read_len, NULL);		
	} else{
	rc = cyttsp5_ret_scan_data_cmd_(ttsp, elem_offset, elem,
			dad->heatmap.data_type, dad->ic_buf, &config,
			&actual_read_len, NULL);
	}
	if (rc < 0)
		goto release_exclusive;

	length = get_unaligned_le16(&dad->ic_buf[0]);
	buf_offset = dad->ic_buf + length;

	element_size = config & CY_CMD_RET_PANEL_ELMNT_SZ_MASK;

	elem -= actual_read_len;
	elem_offset = actual_read_len;
	while (elem > 0) {
		if(elem > MAX_ELEM) {
			rc = cyttsp5_ret_scan_data_cmd_(ttsp, elem_offset, MAX_ELEM,
				dad->heatmap.data_type, NULL, &config,
				&actual_read_len, buf_offset);			
		} else {
		rc = cyttsp5_ret_scan_data_cmd_(ttsp, elem_offset, elem,
				dad->heatmap.data_type, NULL, &config,
				&actual_read_len, buf_offset);
		}
		if (rc < 0)
			goto release_exclusive;

		if (!actual_read_len)
			break;

		length += actual_read_len * element_size;
		buf_offset = dad->ic_buf + length;
		elem -= actual_read_len;
		elem_offset += actual_read_len;
	}

	/* Reconstruct cmd header */
	put_unaligned_le16(length, &dad->ic_buf[0]);
	put_unaligned_le16(elem_offset, &dad->ic_buf[7]);

release_exclusive:
	rc1 = cyttsp5_release_exclusive(ttsp);
put_runtime:

	if (rc < 0)
		goto release_mutex;

	print_idx += scnprintf(buf_out, TTHE_TUNER_MAX_BUF, "CY_DATA:");
	for (i = 0; i < length; i++)
		print_idx += scnprintf(buf_out + print_idx,
				TTHE_TUNER_MAX_BUF - print_idx,
				"%02X ", dad->ic_buf[i]);
	print_idx += scnprintf(buf_out + print_idx,
			TTHE_TUNER_MAX_BUF - print_idx,
			":(%d bytes)\n", length);
	rc = simple_read_from_buffer(buf, count, ppos, buf_out, print_idx);
	print_idx = rc;

release_mutex:
	mutex_unlock(&dad->debugfs_lock);
	return print_idx;
}

static ssize_t tthe_get_panel_data_debugfs_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *ppos)
{
	struct cyttsp5_device_access_data *dad = filp->private_data;
	struct device *dev = &dad->ttsp->dev;
	ssize_t length;
	int max_read;
	u8 *buf_in = dad->tthe_get_panel_data_buf;
	int ret;

	mutex_lock(&dad->debugfs_lock);
	ret = copy_from_user(buf_in + (*ppos), buf, count);
	if (ret)
		goto exit;
	buf_in[count] = 0;

	length = cyttsp5_ic_parse_input(dev, buf_in, count, dad->ic_buf,
			CY_MAX_PRBUF_SIZE);
	if (length <= 0) {
		TS_LOG_ERR( "%s: %s Group Data store\n", __func__,
				"Malformed input for");
		goto exit;
	}

	/* update parameter value */
	dad->heatmap.num_element = get_unaligned_le16(&dad->ic_buf[3]);
	dad->heatmap.data_type = dad->ic_buf[5];

	if (dad->ic_buf[6] > 0)
		dad->heatmap.scan_start = true;
	else
		dad->heatmap.scan_start = false;

	/* elem can not be bigger then buffer size */
	max_read = CY_CMD_RET_PANEL_HDR;
	max_read += dad->heatmap.num_element * CY_CMD_RET_PANEL_ELMNT_SZ_MAX;

	if (max_read >= CY_MAX_PRBUF_SIZE) {
		dad->heatmap.num_element =
			(CY_MAX_PRBUF_SIZE - CY_CMD_RET_PANEL_HDR)
			/ CY_CMD_RET_PANEL_ELMNT_SZ_MAX;
		TS_LOG_ERR( "%s: Will get %d element\n", __func__,
				dad->heatmap.num_element);
	}

exit:
	mutex_unlock(&dad->debugfs_lock);
	TS_LOG_DEBUG( "%s: return count=%d\n", __func__, count);
	return count;
}

static int tthe_get_panel_data_debugfs_open(struct inode *inode,
		struct file *filp)
{
	struct cyttsp5_device_access_data *dad = inode->i_private;

	mutex_lock(&dad->debugfs_lock);

	if (dad->tthe_get_panel_data_is_open) {
		mutex_unlock(&dad->debugfs_lock);
		return -EBUSY;
	}

	filp->private_data = inode->i_private;

	dad->tthe_get_panel_data_is_open = 1;
	mutex_unlock(&dad->debugfs_lock);
	return 0;
}

static int tthe_get_panel_data_debugfs_close(struct inode *inode,
		struct file *filp)
{
	struct cyttsp5_device_access_data *dad = filp->private_data;

	mutex_lock(&dad->debugfs_lock);
	filp->private_data = NULL;
	dad->tthe_get_panel_data_is_open = 0;
	mutex_unlock(&dad->debugfs_lock);

	return 0;
}

static const struct file_operations tthe_get_panel_data_fops = {
	.open = tthe_get_panel_data_debugfs_open,
	.release = tthe_get_panel_data_debugfs_close,
	.read = tthe_get_panel_data_debugfs_read,
	.write = tthe_get_panel_data_debugfs_write,
};
#endif

#ifdef CONFIG_PM_SLEEP
static int cyttsp5_device_access_suspend(struct device *dev)
{
	struct cyttsp5_device_access_data *dad = dev_get_drvdata(dev);

	if (!mutex_trylock(&dad->sysfs_lock))
		return -EBUSY;

	mutex_unlock(&dad->sysfs_lock);
	return 0;
}

static int cyttsp5_device_access_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops cyttsp5_device_access_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cyttsp5_device_access_suspend,
			cyttsp5_device_access_resume)
};

static int cyttsp5_setup_sysfs(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_device_access_data *dad = dev_get_drvdata(dev);
	int rc = 0;

	rc = device_create_file(dev, &dev_attr_command);
	if (rc) {
		TS_LOG_ERR( "%s: Error, could not create command\n",
				__func__);
		goto exit;
	}

	rc = device_create_file(dev, &dev_attr_status);
	if (rc) {
		TS_LOG_ERR( "%s: Error, could not create status\n",
				__func__);
		goto unregister_command;
	}

	rc = device_create_file(dev, &dev_attr_response);
	if (rc) {
		TS_LOG_ERR( "%s: Error, could not create response\n",
				__func__);
		goto unregister_status;
	}

    rc = device_create_file(dev, &dev_attr_rawdata_check);
	if (rc) {
		TS_LOG_ERR( "%s: Error, could not create rawdata_check\n",
				__func__);
		goto unregister_response;
	}

#ifdef TTHE_TUNER_SUPPORT
	dad->tthe_get_panel_data_debugfs = debugfs_create_file(
			CYTTSP5_TTHE_TUNER_GET_PANEL_DATA_FILE_NAME,
			0644, NULL, dad, &tthe_get_panel_data_fops);
	if (IS_ERR_OR_NULL(dad->tthe_get_panel_data_debugfs)) {
		TS_LOG_ERR( "%s: Error, could not create get_panel_data\n",
				__func__);
		dad->tthe_get_panel_data_debugfs = NULL;
		goto unregister_rawdata_check;
	}
#endif

	dad->sysfs_nodes_created = true;
	return rc;

#ifdef TTHE_TUNER_SUPPORT
unregister_rawdata_check:
    device_remove_file(dev, &dev_attr_rawdata_check);
#endif
unregister_response:
	device_remove_file(dev, &dev_attr_response);
unregister_status:
	device_remove_file(dev, &dev_attr_status);
unregister_command:
	device_remove_file(dev, &dev_attr_command);
exit:
	return rc;
}

static int cyttsp5_setup_sysfs_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_device_access_data *dad = dev_get_drvdata(dev);
	int rc = 0;

	dad->si = cyttsp5_request_sysinfo(ttsp);
	if (!dad->si)
		return -1;

	rc = cyttsp5_setup_sysfs(ttsp);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_setup_sysfs_attention, 0);

	return rc;

}

static int cyttsp5_device_access_probe(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_device_access_data *dad;
	struct cyttsp5_device_access_platform_data *pdata =
			dev_get_platdata(dev);
	int rc = 0;

	dad = kzalloc(sizeof(*dad), GFP_KERNEL);
	if (dad == NULL) {
		TS_LOG_ERR( "%s: Error, kzalloc\n", __func__);
		rc = -ENOMEM;
		goto cyttsp5_device_access_probe_data_failed;
	}

	mutex_init(&dad->sysfs_lock);
	dad->ttsp = ttsp;
	dad->pdata = pdata;
	dev_set_drvdata(dev, dad);
#ifdef TTHE_TUNER_SUPPORT
	mutex_init(&dad->debugfs_lock);
	dad->heatmap.num_element = 200;
#endif

	/* get sysinfo */
	dad->si = cyttsp5_request_sysinfo(ttsp);
	if (dad->si) {
		rc = cyttsp5_setup_sysfs(ttsp);
		if (rc){
			TS_LOG_ERR( "%s: Fail setup sysfs\n",__func__);
			goto cyttsp5_device_access_setup_sysfs_failed;
		}
	} else {
		TS_LOG_ERR( "%s: Fail get sysinfo pointer from core p=%p\n",
				__func__, dad->si);
		cyttsp5_subscribe_attention(ttsp, CY_ATTEN_STARTUP,
			cyttsp5_setup_sysfs_attention, 0);
	}

	globe_dad = dad;
#ifdef CONFIG_TOUCHSCREEN_MMI_EQUIP
	register_equip_device_access_data(dad);
#endif
	return 0;

 cyttsp5_device_access_setup_sysfs_failed:
	dev_set_drvdata(dev, NULL);
	kfree(dad);
 cyttsp5_device_access_probe_data_failed:
	TS_LOG_ERR( "%s failed.\n", __func__);
	return rc;
}

static int cyttsp5_device_access_release(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_device_access_data *dad = dev_get_drvdata(dev);

	if (dad->sysfs_nodes_created) {
		device_remove_file(dev, &dev_attr_command);
		device_remove_file(dev, &dev_attr_status);
		device_remove_file(dev, &dev_attr_response);
#ifdef TTHE_TUNER_SUPPORT
		debugfs_remove(dad->tthe_get_panel_data_debugfs);
#endif
	} else {
		cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
			cyttsp5_setup_sysfs_attention, 0);
	}

	dev_set_drvdata(dev, NULL);
	kfree(dad);
	return 0;
}

static struct cyttsp5_driver cyttsp5_device_access_driver = {
	.probe = cyttsp5_device_access_probe,
	.remove = cyttsp5_device_access_release,
	.driver = {
		.name = CYTTSP5_DEVICE_ACCESS_NAME,
		.bus = &cyttsp5_bus_type,
		.owner = THIS_MODULE,
		.pm = &cyttsp5_device_access_pm_ops,
	},
};

static struct cyttsp5_device_access_platform_data
	_cyttsp5_device_access_platform_data = {
	.device_access_dev_name = CYTTSP5_DEVICE_ACCESS_NAME,
};

static const char cyttsp5_device_access_name[] = CYTTSP5_DEVICE_ACCESS_NAME;
static struct cyttsp5_device_info
	cyttsp5_device_access_infos[CY_MAX_NUM_CORE_DEVS];

static char *core_ids[CY_MAX_NUM_CORE_DEVS] = {
	CY_DEFAULT_CORE_ID,
	NULL,
	NULL,
	NULL,
	NULL
};

static int num_core_ids = 1;

module_param_array(core_ids, charp, &num_core_ids, 0);
MODULE_PARM_DESC(core_ids,
	"Core id list of cyttsp5 core devices for device access module");

static int __init cyttsp5_device_access_init(void)
{
	int rc = 0;
	int i, j;
	/*
	if(get_boot_into_recovery_flag())
		return 0;
	*/
	/* Check for invalid or duplicate core_ids */
	for (i = 0; i < num_core_ids; i++) {
		if (!strlen(core_ids[i])) {
			TS_LOG_ERR("%s: core_id %d is empty\n",
				__func__, i+1);
			return -EINVAL;
		}
		for (j = i+1; j < num_core_ids; j++)
			if (!strcmp(core_ids[i], core_ids[j])) {
				TS_LOG_ERR("%s: core_ids %d and %d are same\n",
					__func__, i+1, j+1);
				return -EINVAL;
			}
	}

	for (i = 0; i < num_core_ids; i++) {
		cyttsp5_device_access_infos[i].name =
			cyttsp5_device_access_name;
		cyttsp5_device_access_infos[i].core_id = core_ids[i];
		cyttsp5_device_access_infos[i].platform_data =
			&_cyttsp5_device_access_platform_data;
		TS_LOG_INFO("%s: Registering device access device for core_id: %s\n",
			__func__, cyttsp5_device_access_infos[i].core_id);
		rc = cyttsp5_register_device(&cyttsp5_device_access_infos[i]);
		if (rc < 0) {
			TS_LOG_ERR("%s: Error, failed registering device\n",
				__func__);
			goto fail_unregister_devices;
		}
	}
	rc = cyttsp5_register_driver(&cyttsp5_device_access_driver);
	if (rc) {
		TS_LOG_ERR("%s: Error, failed registering driver\n", __func__);
		goto fail_unregister_devices;
	}

	TS_LOG_INFO("%s: Cypress TTSP Device Access (Built %s) rc=%d\n",
		 __func__, CY_DRIVER_DATE, rc);
	return 0;

fail_unregister_devices:
	for (i--; i <= 0; i--) {
		cyttsp5_unregister_device(cyttsp5_device_access_infos[i].name,
			cyttsp5_device_access_infos[i].core_id);
		TS_LOG_INFO("%s: Unregistering device access device for core_id: %s\n",
			__func__, cyttsp5_device_access_infos[i].core_id);
	}
	return rc;
}
module_init(cyttsp5_device_access_init);

static void __exit cyttsp5_device_access_exit(void)
{
	int i;

	cyttsp5_unregister_driver(&cyttsp5_device_access_driver);
	for (i = 0; i < num_core_ids; i++) {
		cyttsp5_unregister_device(cyttsp5_device_access_infos[i].name,
			cyttsp5_device_access_infos[i].core_id);
		TS_LOG_INFO("%s: Unregistering device access device for core_id: %s\n",
			__func__, cyttsp5_device_access_infos[i].core_id);
	}
}
module_exit(cyttsp5_device_access_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product Device Access Driver");
MODULE_AUTHOR("Cypress Semiconductor");
