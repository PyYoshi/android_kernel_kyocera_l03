/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 *
 * drivers/input/touchscreen/pixart.c
 *
 * Copyright (C) 2012 Pixart Imaging Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/time.h>
#include <asm/irq.h>

#include <linux/i2c/pixart.h>
#include "pixart-fw.h"

#include <linux/input/mt.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#ifdef CONFIG_OF
#include "board-8974-touch.h"
#endif

#define PIXART_DRIVER_VERSION "Pixart AMRI/PAP1100 v2.10"

#define MODULE_NAME "pixart: "
#define FIRST	1
#define SECOND	2

#define TMP_FACTORY_TEST

static uint8_t pid = 0;

static int pixart_init_panel(struct kc_ts_data *ts, int force_dl);
static void pixart_error_check_process(u8 val, struct kc_ts_data *ts);
static int pixart_set_nv(struct kc_ts_data *ts);
static void pixart_report_clear(struct kc_ts_data *ts);
static int pixart_switch_config(struct kc_ts_data *ts);
static long pixart_pixel_dump(struct kc_ts_data *ts, u8 *p_pixel_dump);

static int pixart_r_reg_byte(struct i2c_client *client, uint8_t reg, uint8_t *val)
{
	int i;
	unsigned char data[1];

	if (!client->adapter)
		return -ENODEV;

	for(i = 0; i < PIXART_RW_RETRY; i++){
		if(i2c_smbus_read_i2c_block_data(client, reg, 1, (uint8_t *) data) == 1){
			KC_TS_DEV_I2C("%s: add: %02x, val: %02x\n",__func__, reg , data[0]);
			*val = data[0];
			return 0;
		}
		msleep(PIXART_I2C_WAIT);
	}
	pr_err("%s:Err add = 0x%02x, val = 0x%02x\n",__func__ , reg, data[0]);
	return -EIO;
}

static int pixart_r_reg_nbytes(struct i2c_client *client,
		uint8_t readAddress, uint8_t *pdata, uint8_t len)
{
	uint8_t i;

	if (!client->adapter)
		return -ENODEV;

	for(i = 0; i < PIXART_RW_RETRY; i++){
		if(i2c_smbus_read_i2c_block_data(client, readAddress, len, pdata)){
			if(ts_log_level & 0x08){
				for(i=0; i<len; i++){
					pr_notice("%s: add: %02x, val: %02x\n",__func__, readAddress, *pdata);
					pdata++;
				}
			}
			return 0;
		}
		msleep(PIXART_I2C_WAIT);
	}
	pr_err("%s:Err add = 0x%02x\n",__func__ , readAddress);
	return -EIO;

}

static int pixart_w_reg_byte(struct i2c_client *client,
		uint8_t reg, uint8_t val)
{
	int i;

	if (!client->adapter)
		return -ENODEV;

	for(i = 0; i < PIXART_RW_RETRY; i++){
		if(i2c_smbus_write_i2c_block_data(client, reg, 1, &val) == 0){
			KC_TS_DEV_I2C("%s: add: %02x, val: %02x\n",__func__, reg , val);
			return 0;
		}
		msleep(PIXART_I2C_WAIT);
	}
	pr_err("%s:Err add = 0x%02x, val = 0x%02x\n",__func__ , reg, val);
	return -EIO;
}

static void pixart_enable_irq(struct kc_ts_data *ts)
{
	KC_TS_DEV_DBG("%s() is called.\n",__func__);
	if (ts->is_enable)
		goto done;

	ts->is_enable = true;
	enable_irq(ts->vdata->client->irq);
done:
	return;
}

static void pixart_disable_irq(struct kc_ts_data *ts)
{
	KC_TS_DEV_DBG("%s() is called.\n",__func__);
	if (!ts->is_enable)
		goto done;

	disable_irq_nosync(ts->vdata->client->irq);
	ts->is_enable = false;
done:
	return;
}

static int pixart_check_bit_true(struct kc_ts_data *ts,
						int sleep, int loop, u8 reg, u8 bit)
{
	uint8_t val;
	int err, retry;

	while(1){
		err = pixart_r_reg_byte(ts->vdata->client, reg, &val);
		if(err){
			pr_err("%s:Error Read 0x%02x\n", __func__, reg);
			return err;
		}
		if(val & bit)
			break;
		msleep(sleep);
		retry++;
		if(retry > loop){
			pr_err("%s:TimeOut\n",__func__);
			return -1;
		}
	}
	return 0;
}

static int pixart_check_bit_false(struct kc_ts_data *ts,
						int sleep, int loop, u8 reg, u8 bit)
{
	uint8_t val;
	int err, retry;

	while(1){
		err = pixart_r_reg_byte(ts->vdata->client, reg, &val);
		if(err){
			pr_err("%s:Error Read 0x%02x\n", __func__, reg);
			return err;
		}
		if((val & bit) == 0)
			break;
		msleep(sleep);
		retry++;
		if(retry > loop){
			pr_err("%s:TimeOut\n",__func__);
			return -1;
		}
	}
	return 0;
}

static int pixart_set_orient_rate(struct kc_ts_data *ts)
{
	int err = 0;
	uint8_t val;

	KC_TS_DEV_DBG("%s() is called.\n",__func__);
	err |= pixart_r_reg_byte(ts->vdata->client, PIXART_REG_ORIENTATION_GEN2, &val);
	val = val & 0xF8;
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_ORIENTATION_GEN2, 0x03 | val);
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_REPORT_RATE_GEN2, 0x5A);
	if(err){
		pr_err("%s: Error Write Report_Rate\n",__func__);
		return err;
	}
	return 0;
}

static int pixart_check_config(struct kc_ts_data *ts, int *flg)
{
	const struct kc_ts_platform_data *pdata = ts->pdata;
	struct ts_config_nv *config_nv;
	int err, i;
	uint8_t val;

	KC_TS_DEV_DBG("%s() is called.\n",__func__);

	err = pixart_r_reg_byte(ts->vdata->client, PIXART_REG_FW_REVID_GEN2, &val);
	if(err){
		pr_err("%s: Error Read FW_REVID\n", __func__);
		return err;
	}
	if(val == 0xFF){
		pr_notice("%s:This IC is Test-FW\n",__func__);
		return 0;
	}

	for(i=0; i<pdata->config_length; ){
		err = pixart_r_reg_byte(ts->vdata->client, pdata->config[i], &val);
		if(err){
			pr_err("%s: Error Read pdata->config[%d] = %x\n", __func__, i, pdata->config[i]);
			return err;
		}
		if(val != pdata->config[i+2]){
			pr_notice("%s: Add->%02x, IC = %02x, NV = %02x\n", __func__, pdata->config[i], val, pdata->config[i+2]);
			*flg = 1;
			return 0;
		}
		i += 3;
	}

	config_nv = &ts->config_nv[TS_EXTENDED_NV];
	if(!config_nv->data){
		pr_err("%s: No nv Extended data.\n",__func__);
		return 0;
	}
	for(i = 0; i < config_nv->size; ){
		err |= pixart_w_reg_byte(ts->vdata->client, config_nv->data[i],   config_nv->data[i+2]);
		err |= pixart_w_reg_byte(ts->vdata->client, config_nv->data[i+3], config_nv->data[i+5]);
		err |= pixart_r_reg_byte(ts->vdata->client, config_nv->data[i+6], &val);
		if(val != config_nv->data[i+8]){
			pr_notice("%s: Add->%02x, IC = %02x, NV = %02x\n",
					 __func__, config_nv->data[i+5], val, config_nv->data[i+8]);
			*flg = 1;
			return 0;
		}
		if(err){
			pr_err("%s: %d: I2C Access Error. Addr 0x%02x\n", __func__, __LINE__, config_nv->data[i+5]);
			return err;
		}
		i += 9;
	}

	config_nv = &ts->config_nv[TS_CHARGE_CABLE];
	if(!config_nv->data){
		pr_err("%s: No nv data.\n",__func__);
		return 0;
	}
	for(i = 0; i < config_nv->size; ){
		err = pixart_r_reg_byte(ts->vdata->client, config_nv->data[i], &val);
		if(err){
			pr_err("%s: Error Read pdata->config[%d] = %x\n", __func__, i, config_nv->data[i]);
			return err;
		}
		if(val != config_nv->data[i+2]){
			pr_notice("%s: Add->%02x, IC = %02x, NV = %02x\n", __func__, config_nv->data[i], val, config_nv->data[i+2]);
			*flg = 1;
			return 0;
		}
		i += 3;
	}

	err = pixart_set_orient_rate(ts);
	if(err){
		pr_err("%s: Error Write  fixed value\n",__func__);
		return err;
	}

	return 0;
}

static int pixart_charge_config(struct kc_ts_data *ts)
{
	struct ts_config_nv *config_nv;
	int err = 0, i;
	uint8_t val;

	pr_notice("%s is called\n",__func__);

	config_nv = &ts->config_nv[TS_CHARGE_CABLE];
	if(!config_nv->data){
		pr_err("%s: No nv data.\n",__func__);
		return 0;
	}

	for(i = 0; i < config_nv->size; ){
		err |= pixart_r_reg_byte(ts->vdata->client, config_nv->data[i], &val);
		if(val != config_nv->data[i+2]){
			pr_notice("%s: Add->%02x, IC = %02x, NV = %02x\n", __func__, config_nv->data[i], val, config_nv->data[i+2]);
			err |= pixart_w_reg_byte(ts->vdata->client, config_nv->data[i], config_nv->data[i+2]);
		}
		if(err){
			pr_err("%s: %d: I2C Access Error. Addr 0x%02x\n", __func__, __LINE__, config_nv->data[i]);
			return err;
		}
		i += 3;
	}

	return 0;
}

static int pixart_set_initial(struct kc_ts_data *ts)
{
	struct ts_config_nv *config_nv;
	int err = 0, i;
	uint8_t val;

	KC_TS_DEV_DBG("%s() is called.\n",__func__);
	config_nv = &ts->config_nv[TS_INITIAL_VALUE_NV];
	if(!config_nv->data){
		pr_err("%s: No nv data.\n",__func__);
		return 0;
	}

	for(i = 0; i < config_nv->size; ){
		err |= pixart_r_reg_byte(ts->vdata->client, config_nv->data[i], &val);
		if(val != config_nv->data[i+2]){
			err |= pixart_w_reg_byte(ts->vdata->client, config_nv->data[i], config_nv->data[i+2]);
			pr_notice("%s: Add->%02x, IC = %02x, NV = %02x\n", __func__, config_nv->data[i], val, config_nv->data[i+2]);
		}
		if(err){
			pr_err("%s: %d: I2C Access Error. Addr 0x%02x\n", __func__, __LINE__, config_nv->data[i]);
			return err;
		}
		i += 3;
	}

	return 0;
}

static int pixart_set_extended(struct kc_ts_data *ts)
{
	struct ts_config_nv *config_nv;
	int err = 0, i;
	uint8_t val;

	KC_TS_DEV_DBG("%s() is called.\n",__func__);
	config_nv = &ts->config_nv[TS_EXTENDED_NV];
	if(!config_nv->data){
		pr_err("%s: No nv Extended data.\n",__func__);
		return 0;
	}

	for(i = 0; i < config_nv->size; ){
		if (!i)
			err |= pixart_w_reg_byte(ts->vdata->client, config_nv->data[i],   config_nv->data[i+2]);
		err |= pixart_w_reg_byte(ts->vdata->client, config_nv->data[i+3], config_nv->data[i+5]);
		err |= pixart_r_reg_byte(ts->vdata->client, config_nv->data[i+6], &val);
		if(val != config_nv->data[i+8]){
			err |= pixart_w_reg_byte(ts->vdata->client, config_nv->data[i+6], config_nv->data[i+8]);
			pr_notice("%s: Add->%02x, IC = %02x, NV = %02x\n",
					 __func__, config_nv->data[i+5], val, config_nv->data[i+8]);
		}
		if(err){
			pr_err("%s: %d: I2C Access Error. Addr 0x%02x\n", __func__, __LINE__, config_nv->data[i+5]);
			return err;
		}
		i += 9;
	}

	return 0;
}

static int pixart_set_config(struct kc_ts_data *ts)
{
	int err = 0;

	pr_notice("%s is called\n",__func__);

	err |= pixart_w_reg_byte(ts->vdata->client, 0x7C, 0xFE);
	usleep_range(1000, 1000);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x27, 0x42);
	usleep_range(1000, 1000);
	if(err){
		pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
		return -1;
	}

	err |= pixart_set_initial(ts);
	err |= pixart_set_extended(ts);
	err |= pixart_charge_config(ts);
	err |= pixart_set_orient_rate(ts);
	usleep_range(1000, 1000);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7C, 0xFE);
	usleep_range(1000, 1000);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x27, 0x82);
	if(err){
		pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
		return -1;
	}

	msleep(300);

	err = pixart_check_bit_false(ts, 20, 10, 0x27, 0x01);
	if(err){
		pr_err("%s:Fail FLASH-SAVE\n",__func__);
		return -1;
	}

	pr_notice("%s is complete\n",__func__);

	return 0;
}

static int pixart_reset_and_wait(struct kc_ts_data *ts)
{
	const struct kc_ts_platform_data *pdata = ts->pdata;
	int retry, err;
	u8 val = 0;

	KC_TS_DEV_DBG("%s() is called.\n",__func__);
	if (pdata->reset_hw)
		pdata->reset_hw();
	else {
		pr_notice("%s: reset_hw is NULL\n",__func__);
		return -1;
	}

	for(retry = 0; retry < 11; retry++){
		err = pixart_r_reg_byte(ts->vdata->client, PIXART_REG_PID, &pid);
		if(err){
			pr_err("%s: Error Read PRODUCT_ID\n",__func__);
			return err;
		}
		if(pid == PID_PAP1110){
			break;
		}
		if(retry == 10) {
			pr_err("%s: No hardware detected.\n",__func__);
			return -1;
		}
	}

	retry = 50;
	while (retry > 0) {
		err = pixart_r_reg_byte(ts->vdata->client, PIXART_REG_BOOT_STAT_GEN2, &val);
		if(err){
			pr_err("%s: Error Read BOOT Status\n",__func__);
			return err;
		}
		if (val & 0x01)
			break;

		msleep(10);
		retry--;
	}

	if (retry == 0) {
		pr_err("%s: Reset failed\n",__func__);
		return 1;
	}

	return 0;
}

static int pixart_check_info_block(struct kc_ts_data *ts, int *flg)
{
	uint8_t trim_fine = 0;
	uint8_t trim_coarse = 0;
	uint8_t info_fine = 0;
	uint8_t info_coarse = 0;
	int err = 0;

	KC_TS_DEV_DBG("%s is called\n",__func__);

	err |= pixart_w_reg_byte(ts->vdata->client, 0x7A, 0xAA);
	usleep_range(1000, 1000);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7A, 0xCC);
	usleep_range(10000, 10000);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x07);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7D, 0xAD);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x48);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x13);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x30, 0x01);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x01);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x01, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x00, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x04, 0x80);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x05, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x06, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x07, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x00, 0xA1);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x01, 0x01);
	if(err){
		pr_err("%s: I2C Write Error\n", __func__);
		return -1;
	}

	err |= pixart_r_reg_byte(ts->vdata->client, 0x03, &info_fine);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x04, 0x81);
	err |= pixart_r_reg_byte(ts->vdata->client, 0x03, &info_coarse);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x00, 0x00);
	if(err){
		pr_err("%s: Get INFO_Value Error\n", __func__);
		return -1;
	}

	err = pixart_reset_and_wait(ts);
	if (err){
		pr_err("%s: Failed to restart!\n",__func__);
		return -1;
	}
	err |= pixart_r_reg_byte(ts->vdata->client, 0x1C, &trim_fine);
	err |= pixart_r_reg_byte(ts->vdata->client, 0x1D, &trim_coarse);
	if(err){
		pr_err("%s: Get OSC_TRIM Error\n", __func__);
		return -1;
	}

	if((trim_fine == info_fine) && (trim_coarse == info_coarse)){
		pr_notice("%s: info_block is %02x %02x\n",__func__, info_fine, info_coarse);
		return 0;
	}

	pr_err("%s: OSC_Trim is %02x %02x, Info_Block is %02x %02x\n"
			, __func__, trim_fine, trim_coarse, info_fine, info_coarse);
	*flg = 1;
	return 0;
}

static int pixart_dl_info_block(struct kc_ts_data *ts, int fw_dl)
{
	uint8_t info_fine = 0, info_coarse = 0;
	int err = 0;

	pr_notice("%s is called\n",__func__);

	err |= pixart_w_reg_byte(ts->vdata->client, 0x7A, 0xAA);
	usleep_range(1000, 1000);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7A, 0xCC);
	usleep_range(10000, 10000);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x07);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7D, 0xAD);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7C, 0xFE);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x48);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x13);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x30, 0x01);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x01);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x01, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x00, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x04, 0x80);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x05, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x06, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x07, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x00, 0xA1);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x01, 0x01);
	if(err){
		pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
		return -1;
	}
	err |= pixart_r_reg_byte(ts->vdata->client, 0x03, &info_fine);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x04, 0x81);
	err |= pixart_r_reg_byte(ts->vdata->client, 0x03, &info_coarse);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x00, 0x00);
	if(err){
		pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
		return -1;
	}

	if(fw_dl == SECOND) {
		pr_notice("%s is INFO_BLOCK Read, FW_DL root\n",__func__);
	} else if(fw_dl == FIRST){
		pr_notice("%s is INFO_BLOCK Read, Not FW_DL root\n",__func__);
		usleep_range(10000, 10000);

		err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x01);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x01, 0x00);
		if(err){
			pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
			return -1;
		}
		usleep_range(2000, 2000);

		err |= pixart_check_bit_false(ts, 10, 20, 0x02, 0x01);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x04, 0x5C);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x00, 0x08);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x01, 0x01);
		if(err){
			pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
			return -1;
		}
		msleep(60);

		err |= pixart_check_bit_true(ts, 10, 20, 0x02, 0x01);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x01, 0x00);
		if(err){
			pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
			return -1;
		}
		usleep_range(2000, 2000);

		err |= pixart_check_bit_false(ts, 10, 20, 0x02, 0x01);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x04, 0x5D);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x00, 0x08);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x01, 0x01);
		if(err){
			pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
			return -1;
		}
		msleep(60);

		err |= pixart_check_bit_true(ts, 10, 20, 0x02, 0x01);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x01, 0x00);
		if(err){
			pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
			return -1;
		}
		usleep_range(2000, 2000);

		err |= pixart_check_bit_false(ts, 10, 20, 0x02, 0x01);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x04, 0x5E);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x00, 0x08);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x01, 0x01);
		if(err){
			pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
			return -1;
		}
		msleep(60);

		err |= pixart_check_bit_true(ts, 10, 20, 0x02, 0x01);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x01, 0x00);
		if(err){
			pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
			return -1;
		}
		usleep_range(2000, 2000);

		err |= pixart_check_bit_false(ts, 10, 20, 0x02, 0x01);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x04, 0x5F);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x00, 0x08);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x01, 0x01);
		if(err){
			pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
			return -1;
		}
		msleep(60);

		err |= pixart_check_bit_true(ts, 10, 20, 0x02, 0x01);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x01, 0x00);
		if(err){
			pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
			return -1;
		}
		usleep_range(2000, 2000);

		err |= pixart_check_bit_false(ts, 10, 20, 0x02, 0x01);
		err |= pixart_w_reg_byte(ts->vdata->client, 0x00, 0x00);
		if(err){
			pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
			return -1;
		}
	}
	err = pixart_reset_and_wait(ts);
	if (err){
		pr_err("%s: Failed to restart!\n",__func__);
		return -1;
	}

	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x48);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x13);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x04);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x20, info_fine);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x21, info_coarse);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7F, 0x07);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7D, 0xAD);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7C, 0xFE);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x27, 0xC0);
	if(err){
		pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
		return -1;
	}
	err = pixart_check_bit_false(ts, 10, 20, 0x27, 0x01);
	if(err){
		pr_err("%s:Failed Flash CTL\n",__func__);
		return -1;
	}

	err = pixart_reset_and_wait(ts);
	if (err){
		pr_err("%s: Failed to restart!\n",__func__);
		return -1;
	}

	return 0;
}

static int pixart_check_fw_crc(struct kc_ts_data *ts)
{
	int err = 0;
	uint8_t hi_val, lo_val;

	KC_TS_DEV_DBG("%s is called.\n",__func__);

	err |= pixart_w_reg_byte(ts->vdata->client, 0x30, 0x02);
	usleep_range(1000, 1000);
	err |= pixart_check_bit_true(ts, 10, 50, 0x30, 0x80);
	if(err){
		pr_err("%s:Failed Check CRC\n",__func__);
		return err;
	}

	err |= pixart_r_reg_byte(ts->vdata->client, 0x32, &hi_val);
	err |= pixart_r_reg_byte(ts->vdata->client, 0x33, &lo_val);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x30, 0x00);
	if(err){
		pr_err("%s: Error Read CRC / Write disable\n",__func__);
		return err;
	}

	if((hi_val == PIXART_FF_FW_CRC_HIGH) && (lo_val == PIXART_FF_FW_CRC_LOW)){
		pr_notice("%s:This FW is WS1_FW\n",__func__);
		return -1;
	}

	return 0;
}

static int pixart_check_and_load_fw(struct kc_ts_data *ts, int *cnt)
{
	int i, len, retry, err;
	uint8_t val;
	uint8_t *pfw;

	err = pixart_r_reg_byte(ts->vdata->client, PIXART_REG_FW_REVID_GEN2, &val);
	if(err){
		pr_err("%s: Error Read FW_REVID\n", __func__);
		return err;
	}

	if(val >= PIXART_LATEST_FW && *cnt == FIRST && val != 0xFF){
		pr_notice("%s:This FW is latest 0x%02x\n",__func__, val);
		return 0;
	}
	if(val == 0xff){
		err = pixart_check_fw_crc(ts);
		if(!err){
			pr_notice("%s:This FW is Test_FW:0xFF\n",__func__);
			return 0;
		}
		pr_notice("%s: FW CRC Error\n",__func__);
	}

	*cnt = SECOND;
	err = 0;
	len = sizeof (firmware0E);
	pfw = firmware0E;

	retry = 0;

	if(val == 0){
		err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_SHUTDOWN, PIXART_SHUTDOWN);
		usleep_range(1000, 1000);
		err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_SHUTDOWN, PIXART_RESET);
		err |= pixart_check_bit_true(ts, 10, 100, PIXART_REG_BOOT_STAT_GEN2, 0x01);
		msleep(30);
		err |= pixart_r_reg_byte(ts->vdata->client, PIXART_REG_FW_REVID_GEN2, &val);
		if(err){
			pr_err("%s: %d: I2C Access Error. shutdown part.\n", __func__, __LINE__);
			return err;
		}
		if(val == 0){
			pr_notice("%s:FW_REV_ID = 0x00\n",__func__);
			goto boot_status;
		}
	}
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_FLASH_ENABLE, PIXART_FLASH_ENABLE);
	usleep_range(1000, 1000);
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_FLASH_CTL_GEN2, PIXART_REG_FLASH_NO_SAVE_CUST_REGS);
	msleep(300);
	err |= pixart_check_bit_false(ts, 10, 20, PIXART_REG_FLASH_CTL_GEN2, 0x01);
	if(err){
		pr_err("%s: %d: I2C Access Error. Flash CTL part.\n", __func__, __LINE__);
		return err;
	}

boot_status:
	err |= pixart_check_bit_true(ts, 10, 100, PIXART_REG_BOOT_STAT_GEN2, 0x01);
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_SHUTDOWN, PIXART_SHUTDOWN);
	usleep_range(1000, 1000);
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_SHUTDOWN, PIXART_RESET_TO_ROM_GEN2);
	if(err){
		pr_err("%s: %d: I2C Access Error. shutdown part.\n", __func__, __LINE__);
		return err;
	}
	msleep(10);

	err |= pixart_check_bit_true(ts, 10, 100, PIXART_REG_BOOT_STAT_GEN2, 0x01);
	msleep(30);
	err |= pixart_check_bit_false(ts, 10, 50, PIXART_REG_FW_REVID_GEN2, 0xFF);
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_WD_DISABLE, PIXART_WD_DISABLE);
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_FLASH_ENABLE, PIXART_FLASH_ENABLE);
	usleep_range(1000, 1000);
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_BOOT_STAT_GEN2, 0x00);
	usleep_range(1000, 1000);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x34, 0x08);
	msleep(10);
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_IODL_CTL_GEN2, PIXART_ENABLE_DL_GEN2);
	msleep(10);
	err |= pixart_check_bit_false(ts, 10, 200, PIXART_REG_IODL_CTL_GEN2, 0x80);
	if(err){
		pr_err("%s: %d: I2C Access Error. Boot->Flash CTL part.\n", __func__, __LINE__);
		return err;
	}

	err = pixart_r_reg_byte(ts->vdata->client, PIXART_REG_ERROR_ID_GEN2, &val);
	if(err){
		pr_err("%s:Error Read ERROR_ID\n", __func__);
		return err;
	}
	if(val)
		pr_notice("%s:Check only ERROR_ID = %x\n", __func__, val);

	for(i = 0; i < len; i++){
		err = pixart_w_reg_byte(ts->vdata->client, PIXART_REG_IODL_DATA_GEN2, *(pfw + i));
		if(err){
			pr_err("%s: Error Write IODL_DATA\n",__func__);
			return err;
		}
	}

	retry = 0;
	while(1){
		msleep(10);
		err = pixart_r_reg_byte(ts->vdata->client, PIXART_REG_BOOT_STAT_GEN2, &val);
		if(err){
			pr_err("%s:Error Read BOOT_Stat\n", __func__);
			return err;
		}
		if((val & 0x40) && (val & 0x01))
			break;
		retry++;
		if(retry > 100){
			pr_err("%s:Error Boot Status = 0x%x\n",__func__, val);
			return -1;
		}
	}

	msleep(70);
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_WD_DISABLE, PIXART_WD_ENABLE);
	err |= pixart_r_reg_byte(ts->vdata->client, PIXART_REG_FW_REVID_GEN2, &val);
	if(err){
		pr_err("%s:Error Read FW_REVID\n", __func__);
		return err;
	}
	if(val != PIXART_LATEST_FW){
		pr_err("%s:Not latest FW. This FW is 0x%x\n",__func__, val);
		return -1;
	}

	pr_notice("%s:FW Update is complete\n",__func__);
	return 0;
}

static int pixart_check_ds_map(struct kc_ts_data *ts, int *flg)
{
	int err, i;
	uint8_t hi_crc, lo_crc, val;

	KC_TS_DEV_DBG("%s is called.\n",__func__);

	err = pixart_w_reg_byte(ts->vdata->client, 0x4E, 0x04);
	if(err){
		pr_err("%s: Error Write enable\n",__func__);
		return err;
	}
	usleep_range(1000, 1000);

	for(i = 0; i < 50; i++){
		err |= pixart_r_reg_byte(ts->vdata->client, 0x4E, &val);
		if((val & 0x81) == 0x80){
			KC_TS_DEV_DBG("%s: Success, retry[%02d], val = %02x\n",__func__, i, val);
			break;
		}
		usleep_range(10000, 10000);
	}
	if((i >= 50) || (err)){
		pr_err("%s: Flash Error\n",__func__);
		return -1;
	}

	err  = pixart_r_reg_byte(ts->vdata->client, 0x32, &hi_crc);
	err |= pixart_r_reg_byte(ts->vdata->client, 0x33, &lo_crc);
	if(err){
		pr_err("%s: Error read DS-Map CRC\n",__func__);
		return err;
	}
	err = pixart_w_reg_byte(ts->vdata->client, 0x4E, 0x00);
	if(err){
		pr_err("%s: Error Write disable\n",__func__);
		return err;
	}

	if((hi_crc == PIXART_CRC_HIGH) && (lo_crc == PIXART_CRC_LOW)){
		KC_TS_DEV_DBG("%s:DS-Map is good\n",__func__);
		return 0;
	}

	pr_err("%s:DS-Map Error! CRC = 0x%02x%02x\n",__func__, hi_crc, lo_crc);
	*flg = 1;
	return 0;
}

static int pixart_dl_ds_map(struct kc_ts_data *ts)
{
	int i,retry, len, err = 0;
	uint8_t *pmap, val;

	pr_notice("%s is called\n",__func__);
	
	usleep_range(10000, 10000);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7D, 0xAD);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x03, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x0A, 0x2C);
	usleep_range(1000, 1000);
	err |= pixart_r_reg_byte(ts->vdata->client, PIXART_REG_ERROR_ID_GEN2, &val);
	if((err) || (val == 0x11)){
		pr_err("%s: %d: I2C Access Error or Error ID = %02x\n",
				__func__, __LINE__, val);
		return -1;
	}

	len = sizeof (ds_map);
	pmap = ds_map;
	for(i = 0; i < len; i++){
		err = pixart_w_reg_byte(ts->vdata->client, 0x0B, *(pmap + i));
		if(err){
			pr_err("%s: Error Write IODL_DATA\n",__func__);
			return err;
		}
	}

	retry = 25;
	do {
		pixart_r_reg_byte(ts->vdata->client, 0x03, &val);
		if ((val & 0x40) && !(val & 0x20))
			break;
		msleep(20);
	} while (retry-- > 0);
	if(retry < 1){
		pr_err("%s:Fail BOOT_STAT check!\n",__func__);
		return -1;
	}

	err |= pixart_w_reg_byte(ts->vdata->client, 0x7D, 0x00);
	err |= pixart_r_reg_byte(ts->vdata->client, PIXART_REG_ERROR_ID_GEN2, &val);
	if(err){
		pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
		return -1;
	}
	if(val){
		pr_err("%s: Error ID = 0x%02x\n", __func__, val);
		return -1;
	}
	usleep_range(2000, 2000);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x7C, 0xFE);
	usleep_range(1000, 1000);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x27, 0x84);
	msleep(300);
	if(err){
		pr_err("%s: %d: I2C Access Error\n", __func__, __LINE__);
		return -1;
	}

	err = pixart_check_bit_false(ts, 20, 25, 0x27, 0x01);
	if(err){
		pr_err("%s:Failed Flash CTL\n",__func__);
		return -1;
	}

	pr_notice("%s is complete\n",__func__);
	return 0;
}

static int pixart_init_panel(struct kc_ts_data *ts, int force_dl)
{
	int fw_cnt = FIRST;
	int user_cnt = FIRST;
	int check_flg = 0;

	if(force_dl)
		goto all_write;

	msleep(30);
	if (pixart_check_and_load_fw(ts, &fw_cnt)){
		pr_err("%s: Fail Load FW\n",__func__);
		goto all_write;
	}
	if (fw_cnt == SECOND){
		pr_notice("%s: FW_DL -> Write the INFO_BLOCK, D/S_Map, IO_Reg\n", __func__);
		goto user_write;
	}

	if (pixart_check_info_block(ts, &check_flg)){
		pr_err("%s: Fail Check Info Block\n",__func__);
		goto all_write;
	}
	if(check_flg){
		pr_err("%s: Error Info Block\n", __func__);
		goto user_write;
	}

	if (pixart_check_ds_map(ts, &check_flg)){
		pr_err("%s: Fail Check Drive_Sense Map\n",__func__);
		goto all_write;
	}
	if(check_flg){
		pr_err("%s: Error DS-Map\n", __func__);
		goto user_write;
	}

	if (pixart_check_config(ts, &check_flg)){
		pr_err("%s: Fail Check config\n",__func__);
		goto all_write;
	}
	if(check_flg){
		pr_err("%s: Difference Config\n", __func__);
		goto user_write;
	}

	goto done;

all_write:
	if (pixart_reset_and_wait(ts)){
		pr_err("%s: Failed to restart!\n",__func__);
		return -1;
	}

	fw_cnt = SECOND;
	if (pixart_check_and_load_fw(ts, &fw_cnt)){
		pr_err("%s: Second Fail Load FW\n",__func__);
		return -1;
	}

user_write:
	if (pixart_dl_info_block(ts, fw_cnt)){
		pr_err("%s: Fail DL Info Block\n",__func__);
		if(user_cnt == SECOND){
			pr_err("%s: Second Fail Info Block\n",__func__);
			return -1;
		}
		user_cnt = SECOND;
		goto all_write;
	}

	if (pixart_dl_ds_map(ts)){
		pr_err("%s: Fail DL Drive_Sense Map\n",__func__);
		if(user_cnt == SECOND){
			pr_err("%s: Second Fail Drive_Sense Map\n",__func__);
			return -1;
		}
		user_cnt = SECOND;
		goto all_write;
	}

	if (pixart_set_config(ts)){
		pr_err("%s: Fail DL config\n",__func__);
		if(user_cnt == SECOND){
			pr_err("%s: Second Fail config\n",__func__);
			return -1;
		}
		user_cnt = SECOND;
		goto all_write;
	}

done:
	return 0;
}

static void pixart_calc_resolution(struct kc_ts_data *data)
{
	unsigned int max_x = data->pdata->x_size - 1;
	unsigned int max_y = data->pdata->y_size - 1;

	if (data->pdata->orient & PIX_XY_SWITCH) {
		data->max_x = max_y;
		data->max_y = max_x;
	} else {
		data->max_x = max_x;
		data->max_y = max_y;
	}
}

static int pixart_reset_process(struct kc_ts_data *ts)
{
	int err = 0;

	pr_err("%s: IC Reset\n",__func__);
	pixart_disable_irq(ts);

	err = pixart_reset_and_wait(ts);
	if (err){
		pr_err("%s: Failed to restart!\n",__func__);
		goto done;
	}

	err = pixart_init_panel(ts, 0);
	if (err)
		pr_err("%s: Fail init process\n",__func__);

done:
	pixart_enable_irq(ts);
	return err;
}

static void pixart_error_check_process(u8 val, struct kc_ts_data *ts)
{
	if(ts->vdata->err_cnt > 5){
		pr_err("%s: Error Status. ID = 0x%02x\n",__func__, val);
		pr_err("%s: many error\n",__func__);
		if (ts->is_enable) {
			disable_irq_nosync(ts->irq);
			ts->is_enable = false;
		}
		return;
	}
	switch (val) {
	case 0x01:
	case 0x02:
	case 0x06:
	case 0x07:
	case 0x08:
	case 0x09:
	case 0x0A:
	case 0x0B:
	case 0x0C:
	case 0x0D:
	case 0x0E:
	case 0x0F:
	case 0x10:
	case 0x11:
	case 0x12:
	case 0x23:
		pr_err("%s: IC Error. ID = 0x%02x\n",__func__, val);
		ts->vdata->err_cnt++;
		pixart_reset_process(ts);
		break;
	case 0x13:
		KC_TS_DEV_DBG("%s: Completion notice of initialization.\n",__func__);
		break;
	default:
		pr_err("%s: IC Error. ID = 0x%02x\n",__func__, val);
		ts->vdata->err_cnt++;
		break;
	}
}

static int pixart_interrupt(struct kc_ts_data *ts)
{
	int i, j, err;
	struct kc_ts_pix_data *touch_data = ts->pix_data;
	uint8_t *pdata, status, val;
	uint8_t move = 0;

	ts->vdata->touch_number = 0;

	if (ts->is_suspended){
		pr_err("%s: Abnormal Status\n", __func__);
		goto end_of_interrupt;
	}

	err = pixart_r_reg_byte(ts->vdata->client, PIXART_REG_STATUS_GEN2, &status);
	if(err){
		pr_err("%s:Error Read Status\n", __func__);
		goto end_of_interrupt;
	}
	if (status & PIXART_REG_STATUS_GEN2_ERROR){
		err = pixart_r_reg_byte(ts->vdata->client, PIXART_REG_ERROR_ID_GEN2, &val);
		if(err){
			pr_err("%s:Error Read ERROR_ID\n", __func__);
			goto end_of_interrupt;
		}
		pixart_error_check_process(val, ts);
		if(val != 0x03)
			goto end_of_interrupt;
	}
	if (!(status & PIXART_REG_STATUS_GEN2_DATA_READY)){
		pr_err("%s: Error End. Status = %x\n",__func__, status);
		goto end_of_interrupt;
	}
	if (!(status & PIXART_REG_STATUS_GEN2_TOUCH))
		KC_TS_DEV_TOUCH("%s: decision No Touch\n",__func__);

	err = pixart_w_reg_byte(ts->vdata->client, PIXART_REG_MOTION_REPORT_CTL_GEN2,
		PIXART_MOTION_MARK_READ_GEN2 | PIXART_MOTION_DISABLE_HOVER_GEN2);
	if(err){
		pr_err("%s: Error Write MOTION_REPORT_CTL 01\n",__func__);
		goto end_of_interrupt;
	}

	pdata = (uint8_t *)touch_data;
	err = pixart_r_reg_nbytes(ts->vdata->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, pdata, 2);
	if(err){
		pr_err("%s: Error READ MOTION_REPORT_DATA 01\n",__func__);
		goto end_of_interrupt;
	}	
	if (*pdata == 0xff) {
		pr_err("%s: ignoring report w/ status = 0xff.\n",__func__);
		err = pixart_w_reg_byte(ts->vdata->client, PIXART_REG_MOTION_REPORT_CTL_GEN2,0x04);
		if(err){
			pr_err("%s: Error Write MOTION_REPORT_CTL 02\n",__func__);
			goto end_of_interrupt;
		}
		goto end_of_interrupt;
	}
	pdata += 2;
	err = pixart_r_reg_nbytes(ts->vdata->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, pdata, 9);
	if(err){
		pr_err("%s: Error READ MOTION_REPORT_DATA 02\n",__func__);
		goto end_of_interrupt;
	}
	pdata += 9;
	if (touch_data->total_touch > PIXART_MAX_FINGER)
		touch_data->total_touch = PIXART_MAX_FINGER;

	for (i = 1; i < touch_data->total_touch; i++) {
		err = pixart_r_reg_nbytes(ts->vdata->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, pdata, 9);
		if(err){
			pr_err("%s: Error READ MOTION_REPORT_DATA 03\n",__func__);
			goto end_of_interrupt;
		}
		pdata += 9;
	}

	err = pixart_w_reg_byte(ts->vdata->client, PIXART_REG_MOTION_REPORT_CTL_GEN2,0x04);
	if(err){
		pr_err("%s: Error Write MOTION_REPORT_CTL 03\n",__func__);
		goto end_of_interrupt;
	}

	if(ts_event_control)
		goto end_of_interrupt;

	for(j = 0; j < PIXART_MAX_FINGER; j++){
		if(touch_data->slot[j].id == 0)
			continue;
		val = j;
		for(i = 0; i < touch_data->total_touch; i++){
			if(touch_data->slot[j].id == touch_data->point_data[i].id){
				val = 255;
				break;
			}
		}
		if(val != 255){
			KC_TS_DEV_TOUCH("%s RELEASE j=%d\n",__func__,j);
			input_mt_slot(ts->input_dev, j);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
			touch_data->slot[j].id = 0;
			touch_data->slot[j].tool_finger = 0;
		}
	}

	for (i = 0; i < touch_data->total_touch; i++) {
		for(j = 0; j < PIXART_MAX_FINGER; j++){
			if(touch_data->slot[j].id == touch_data->point_data[i].id){
				KC_TS_DEV_TOUCH("%s: MOVE j=%d , touch_id = %d\n",
							__func__,j,touch_data->point_data[i].id);
				move = 1;
				break;
			}
		}
		if(!move){
			for(j = 0; j < PIXART_MAX_FINGER; j++){
				if(touch_data->slot[j].id == 0){
					touch_data->slot[j].id = touch_data->point_data[i].id;
					KC_TS_DEV_TOUCH("%s: PUSH j=%d , touch_id = %d\n",
							__func__,j,touch_data->point_data[i].id);
					break;
				}
			}
		}
		move = 0;

		input_mt_slot(ts->input_dev, j);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, touch_data->point_data[i].id != 0);
		touch_data->slot[j].tool_finger = (touch_data->point_data[i].id != 0);

		ts->vdata->touch_number++;
		if(touch_data->point_data[i].pressure != 1){
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,	touch_data->point_data[i].x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,	touch_data->point_data[i].y);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE,	touch_data->point_data[i].pressure);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,	touch_data->point_data[i].area);
			KC_TS_DEV_TOUCH("%s:[%02d] id:%02d, (x,y) = (%03d, %04d) area=%d, pressure=%d, num=%x\n"
				, __func__, j, touch_data->point_data[i].id
				, touch_data->point_data[i].x, touch_data->point_data[i].y
				, touch_data->point_data[i].area, touch_data->point_data[i].pressure
				, ts->vdata->touch_number);
		}
	}

	input_report_key(ts->input_dev, BTN_TOUCH, ts->vdata->touch_number > 0);
	input_sync(ts->input_dev);

	return err;

end_of_interrupt:
	KC_TS_DEV_TOUCH("%s: end of work func\n",__func__);
	return err;
}

static int pixart_set_nv(struct kc_ts_data *ts)
{
	int err = 0;

	KC_TS_DEV_DBG("%s() is called.\n",__func__);

	if(ts->config_nv[TS_INITIAL_VALUE_NV].ver & 0xFF00)
		err |= pixart_set_initial(ts);
	
	if(ts->config_nv[TS_EXTENDED_NV].ver & 0xFF00)
		err |= pixart_set_extended(ts);

	ts->config_status_last = TS_INITIAL;
	if(ts->config_nv[TS_CHARGE_C_NV].ver & 0xFF00)
		err |= pixart_switch_config(ts);

	if(err)
		pr_err("%s: Failed set nv\n",__func__);
	return err;
}

static int pixart_switch_config(struct kc_ts_data *ts)
{
	struct ts_config_nv *config_nv;
	enum ts_nv_type nv_type;
	int i, err;

	nv_type = ts->config_status;

	config_nv = &ts->config_nv[nv_type];
	KC_TS_DEV_DBG("%s: To become the [%d]\n",__func__, nv_type);

	if (!ts_config_switching) {
		KC_TS_DEV_DBG("%s: Skip. Disabled config switching.\n", __func__);
		ts->config_status = ts->config_status_last;
		return 0;
	}
	if(ts->config_status_last == ts->config_status){
		KC_TS_DEV_DBG("%s: Skip. Same status.\n",__func__);
		return 0;
	}

	if(!config_nv->data){
		pr_err("%s: No nv data.\n",__func__);
		return 0;
	}
	for(i = 0; i < config_nv->size; ){
		if(config_nv->data[i+1] == 0){
			KC_TS_DEV_DBG("%s: config is over\n",__func__);
			break;
		}
		err = pixart_w_reg_byte(ts->vdata->client, config_nv->data[i], config_nv->data[i+2]);
		i += 3;
		if(err){
			pr_err("%s: Error Write register\n",__func__);
			return err;
		}
	}

	ts->config_status_last = ts->config_status;
	return 0;
}

static int pixart_read_summary_report(struct kc_ts_data *ts)
{
	const struct kc_ts_platform_data *pdata = ts->pdata;
	int count = 0;
	int retry = 0, err;
	u8 val;
	u8 *p_motion;

	KC_TS_DEV_DBG("%s() is called.\n",__func__);
	if (!ts->vdata->motion_data)
		ts->vdata->motion_data = kcalloc(MOTION_DATA_SIZE * MOTION_DATA_PAGE_MAX,
												 sizeof(u8), GFP_KERNEL);
	else
		KC_TS_DEV_DBG("%s: motion_data has been allocated.\n",__func__);

	if (!ts->vdata->motion_data)  {
		pr_err("%s: Failed to allocate memory!\n",__func__);
		return -ENOMEM;
	}
	p_motion = ts->vdata->motion_data;

	while(count < MOTION_DATA_PAGE_MAX && retry < 1000){
		val = gpio_get_value(pdata->irq_gpio);
		if(val != 1){
			retry++;
			usleep_range(1000,1000);
			continue;
		}

		err = pixart_r_reg_byte(ts->vdata->client, PIXART_REG_STATUS_STATUS_CHANGE, &val);
		if(err){
			pr_err("%s:Error Read CHANGE_STATUS\n", __func__);
			return err;
		}
		if(val == 0x00){
			pr_err("%s:Retry count = %d\n",__func__, retry);
			retry++;
			usleep_range(1000,1000);
			continue;
		}

		if(val & 0x01){
			pr_err("%s:Read error val = %02x\n",__func__, val);
			err = pixart_r_reg_byte(ts->vdata->client, PIXART_REG_ERROR_ID_GEN2, &val);
			if(err){
				pr_err("%s:Error Read ERROR_ID\n", __func__);
				return err;
			}
			pr_err("%s:Error Status. ID = 0x%02x\n",__func__, val);
			if(val != 0x03){
				pixart_error_check_process(val, ts);
			}
			retry++;
			continue;
		}

		if((val & 0x10) != 0x10){
			pr_err("%s:Error status Bit4 = 0x%02x\n",__func__,val);
			retry++;
			continue;
		}

		if((val & 0x02) != 0x02){
			pr_err("%s:Error status Bit1 = 0x%02x\n",__func__,val);
			retry++;
			continue;
		}

		err = pixart_r_reg_byte(ts->vdata->client, PIXART_REG_MOTION_REPORT_CTL_GEN2, &val);
		if(err){
			pr_err("%s:Error Read MOTION_REPORT\n", __func__);
			return err;
		}
		val |= 0x80;
		err = pixart_w_reg_byte(ts->vdata->client, PIXART_REG_MOTION_REPORT_CTL_GEN2, val);
		if(err){
			pr_err("%s: Error Write MOTION_REPORT_CTL 01\n",__func__);
			return err;
		}

		err |= pixart_r_reg_nbytes(ts->vdata->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 9);
		p_motion += 9;
		err |= pixart_r_reg_nbytes(ts->vdata->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 9);
		p_motion += 9;
		err |= pixart_r_reg_nbytes(ts->vdata->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 9);
		p_motion += 9;
		err |= pixart_r_reg_nbytes(ts->vdata->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 9);
		p_motion += 9;
		err |= pixart_r_reg_nbytes(ts->vdata->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 9);
		p_motion += 9;
		err |= pixart_r_reg_nbytes(ts->vdata->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 9);
		p_motion += 9;
		err |= pixart_r_reg_nbytes(ts->vdata->client, PIXART_REG_MOTION_REPORT_DATA_GEN2, p_motion, 2);
		p_motion += 38;
		if(err){
			pr_err("%s:Error Read MOTION_REPORT_DATA\n", __func__);
			return -1;
		}

		val &= 0x7F;
		err = pixart_w_reg_byte(ts->vdata->client, PIXART_REG_MOTION_REPORT_CTL_GEN2, val);
		if(err){
			pr_err("%s: Error Write MOTION_REPORT_CTL 02\n",__func__);
			return err;
		}

		retry = 0;
		count++;
		KC_TS_DEV_DBG("%s: count %d\n",__func__ ,count);
	}
	if(retry == 20){
		pr_err("%s:Error Retry = %d\n",__func__, retry);
		return -1;
	}
	return 0;

}

static int pixart_open_short(struct kc_ts_data *ts, u8 *p_motin_fail)
{
	u8 val;
	int err = 0, i;
	int retry = 0;
	int err_flg = 0;

	KC_TS_DEV_DBG("%s() is called.\n",__func__);

	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_SHUTDOWN, PIXART_SHUTDOWN);
	msleep(1);
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_SHUTDOWN, PIXART_RESET);
	msleep(10);
	for (i = 0; i < 100; i++) {
		err |= pixart_r_reg_byte(ts->vdata->client, 0x03, &val);
		if ((val & 0x81) == 0x81){
			KC_TS_DEV_DBG("%s: Success, retry[%02d], val = %02x\n",__func__, i, val);
			break;
		}
		msleep(10);
	}
	if((i >= 100) || (err)){
		pr_err("%s: Error Resume\n",__func__);
		if(pixart_reset_process(ts))
			pr_err("%s: Failed to restart\n", __func__);
		return -1;
	}

	for (i = 0; i < 100; i++) {
		err |= pixart_r_reg_byte(ts->vdata->client, 0x04, &val);
		if (!(val & 0x01)){
			KC_TS_DEV_DBG("%s: Success, retry[%02d], val = %02x\n",__func__, i, val);
			break;
		}
		err |= pixart_r_reg_byte(ts->vdata->client, 0x0D, &val);
		msleep(10);
	}
	if((i >= 100) || (err)){
		pr_err("%s: Error status\n",__func__);
		if(pixart_reset_process(ts))
			pr_err("%s: Failed to restart\n", __func__);
		return -1;
	}

	err = pixart_w_reg_byte(ts->vdata->client, 0x31, 0xFC);
	if(err){
		pr_err("%s: Error Write TEST_CTL\n",__func__);
		return err;
	}

	retry = 0;
	while(1){
		err = pixart_r_reg_byte(ts->vdata->client, PIXART_REG_STATUS, &val);
		if(err){
			pr_err("%s:Error Read STATUS\n", __func__);
			return err;
		}
		if(val == 0xFC)
			break;
		retry++;
		msleep(10);
		if(retry > 10){
			pr_err("%s:Error02 Retry count = %d\n",__func__, retry);
			return -1;
		}
	}

	err |= pixart_w_reg_byte(ts->vdata->client, 0x4A, 0x06);
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_OPEN_SHORT_SELECT, 0x10);
	err |= pixart_check_bit_false(ts, 10, 10, PIXART_REG_OPEN_SHORT_SELECT, 0xFF);
	if(err){
		pr_err("%s:Error Read OPEN_SHORT_SELECT 01\n", __func__);
		return err;
	}

	pr_notice("%s Touch clear process_1\n", __func__);
	pixart_interrupt(ts);
	usleep_range(20000,20000);
	val = gpio_get_value(ts->pdata->irq_gpio);
	if(val == 1){
		pr_notice("%s Touch clear process_2\n", __func__);
		pixart_interrupt(ts);
	}

	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_OPEN_SHORT_SELECT, 0x65);
	msleep(1000);
	err |= pixart_check_bit_true(ts, 100, 40, PIXART_REG_OPEN_SHORT_RESULT, 0x01);
	if(err){
		pr_err("%s:Error Read OPEN_SHORT_RESULT\n", __func__);
		return err;
	}

	err = pixart_read_summary_report(ts);
	if(err){
		err_flg = 1;
		goto done;
	}

	err = pixart_r_reg_byte(ts->vdata->client, PIXART_REG_OPEN_SHORT_RESULT, &val);
	if(err){
		pr_err("%s:Error Read OPEN_SHORT_RESULT\n", __func__);
		return err;
	}
	if((val & 0x02) != 0x02){
		KC_TS_DEV_DBG("%s: Test PASS\n",__func__);
		goto done;
	}

	pr_err("%s:Error OPEN_SHORT\n", __func__);
	err = pixart_r_reg_nbytes(ts->vdata->client, 0x92, p_motin_fail, 11);
	if(err){
		pr_err("%s: Error READ Motion_fail 03\n",__func__);
		return err;
	}

done:
	err = pixart_w_reg_byte(ts->vdata->client, 0x31, 0x00);
	if(err)
		pr_err("%s: Error Write TEST_CTL\n",__func__);

	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_SHUTDOWN, PIXART_SHUTDOWN);
	msleep(1);
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_SHUTDOWN, PIXART_RESET);
	msleep(10);
	for (i = 0; i < 100; i++) {
		err |= pixart_r_reg_byte(ts->vdata->client, 0x03, &val);
		if ((val & 0x81) == 0x81){
			KC_TS_DEV_DBG("%s: Success, retry[%02d], val = %02x\n",__func__, i, val);
			break;
		}
		msleep(10);
	}
	if((i >= 100) || (err)){
		pr_err("%s: Error Resume\n",__func__);
		if(pixart_reset_process(ts))
			pr_err("%s: Failed to restart\n", __func__);
		return -1;
	}

	for (i = 0; i < 100; i++) {
		err |= pixart_r_reg_byte(ts->vdata->client, 0x04, &val);
		if (!(val & 0x01)){
			KC_TS_DEV_DBG("%s: Success, retry[%02d], val = %02x\n",__func__, i, val);
			break;
		}
		err |= pixart_r_reg_byte(ts->vdata->client, 0x0D, &val);
		msleep(10);
	}
	if((i >= 100) || (err)){
		pr_err("%s: Error status\n",__func__);
		if(pixart_reset_process(ts))
			pr_err("%s: Failed to restart\n", __func__);
		return -1;
	}

	if((*p_motin_fail) || (err_flg))
		err = -1;
	return err;

}

static long pixart_pixel_dump(struct kc_ts_data *ts, u8 *p_pixel_dump)
{
	int i = 0;
	long err = 0;

	KC_TS_DEV_DBG("%s() is called.\n",__func__);

	if (ts->is_suspended){
		pr_err("%s: Failed. IC is Suspend.\n",__func__);
		return -1;
	}

	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_PIXEL_CONFIG, 0x00);
	err |= pixart_w_reg_byte(ts->vdata->client, 0x4B, 0xA8);
	err |= pixart_w_reg_byte(ts->vdata->client, PIXART_REG_PIXEL_CONFIG, 0x01);
	err |= pixart_check_bit_true(ts, 10, 10, 0x49, 0x02);
	if(err){
		pr_err("%s:Error RawData Config\n", __func__);
		return err;
	}

	for(i = 0; i < PIXEL_DUMP/2; i++){
		err |= pixart_r_reg_byte(ts->vdata->client, PIXART_REG_PIXEL_DATA_LO, p_pixel_dump);
		p_pixel_dump++;
		err |= pixart_r_reg_byte(ts->vdata->client, PIXART_REG_PIXEL_DATA_HI, p_pixel_dump);
		p_pixel_dump++;
		if(err){
			pr_err("%s:Error Read PIXEL_DATA_HI-LO\n", __func__);
			return err;
		}
	}

	err = pixart_w_reg_byte(ts->vdata->client, PIXART_REG_PIXEL_CONFIG, 0x00);
	if(err)
		pr_err("%s:Error Write PIXEL_CONFIG\n",__func__);
	return err;

}

static long pixart_ts_ioctl(struct kc_ts_data *ts, unsigned int cmd, unsigned long arg)
{
	long err = 0;
	char count;
	u8 *p_motion;
	u8 motion_fail_data[11];

	switch (cmd) {
	case IOCTL_SET_CONF_STAT:
		KC_TS_DEV_DBG("%s: IOCTL_SET_CONF_STAT\n", __func__);
		if (copy_from_user(&ts->config_status, (void __user *)arg, sizeof(ts->config_status))) {
			err = -EFAULT;
			pr_err("%s: copy_from_user error\n", __func__);
			goto done;
		}
		err = (long)pixart_switch_config(ts);
		break;
	case IOCTL_DIAG_GET_C_REFERENCE:
		KC_TS_DEV_DBG("%s: Open Short Test\n",__func__);
		err = copy_from_user(&count, (void __user *)arg, sizeof(unsigned char));
		if (err){
			pr_err("%s: copy_from_user error\n", __func__);
			return -EFAULT;
		}
		switch(count){
		case 0:
			memset(motion_fail_data, 0, sizeof(motion_fail_data));

			err = pixart_open_short(ts, motion_fail_data);
			if(err){
				pr_err("%s: Failed Open Short Test\n", __func__);
				if(motion_fail_data[0]){
					pr_err("%s: Touch Panel NG\n", __func__);
					err = copy_to_user((void __user *)arg, motion_fail_data, 12);
					if (err) {
						pr_err("%s: copy_to_user error\n", __func__);
					}
				}
			}
			break;
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
			if(!ts->vdata->motion_data){
				pr_err("%s: motion data is NULL\n",__func__);
				return -ENOMEM;
			}
			p_motion = ts->vdata->motion_data + (count - 1) * MOTION_DATA_SIZE * MOTION_DATA_1BLOCK;
			err = copy_to_user((void __user *)arg, p_motion,
								 MOTION_DATA_SIZE * MOTION_DATA_1BLOCK);
			if (err) {
				pr_err("%s: copy_to_user error\n", __func__);
				goto done;
			}
			break;
		case 7:
			if(ts->vdata->motion_data){
				KC_TS_DEV_DBG("%s: motion data release\n",__func__);
				kfree(ts->vdata->motion_data);
				ts->vdata->motion_data = NULL;
			}
			break;
		default:
			pr_err("%s: wrong value\n",__func__);
			break;
		}

		KC_TS_DEV_DBG("%s: Open Short Test is completed\n",__func__);
		break;
	case IOCTL_DIAG_GET_DELTA:
		KC_TS_DEV_DBG("%s: Pixel Dump\n",__func__);
		p_motion = kcalloc(PIXEL_DUMP, sizeof(u8), GFP_KERNEL);
		if (!p_motion) {
			pr_err("%s: Failed to allocate memory!\n",__func__);
			return -ENOMEM;
		}

		err = pixart_pixel_dump(ts, p_motion);
		if(err){
			pr_err("%s: Failed Pixel Dump\n", __func__);
			kfree(p_motion);
			p_motion = NULL;
			goto done;
		}

		err = copy_to_user((void __user *)arg, p_motion, PIXEL_DUMP);

		kfree(p_motion);
		p_motion = NULL;

		if (err) {
			pr_err("%s: copy_to_user error\n", __func__);
			goto done;
		}

		KC_TS_DEV_DBG("%s: Pixel Dump completed\n",__func__);
		break;
	default:
		return -EINVAL;
		break;
	}
done:
	return err;
}

static int pixart_input_open(struct kc_ts_data *ts)
{
	int error = 0;

	KC_TS_DEV_DBG("%s() is called.\n",__func__);
	pixart_enable_irq(ts);
	return error;

}

static void pixart_input_close(struct kc_ts_data *ts)
{
	KC_TS_DEV_DBG("%s() is called.\n",__func__);
	mutex_lock(&ts->lock);
	pixart_disable_irq(ts);
	mutex_unlock(&ts->lock);
}

static int pixart_suspend(struct kc_ts_data *ts)
{
	int rc = 0;

	KC_TS_DEV_DBG("%s() is called.\n",__func__);
	pixart_disable_irq(ts);

	if(pixart_w_reg_byte(ts->vdata->client, 0x7a, 0xaa)){
		pr_err("%s:Error Write Suspend\n",__func__);
		return -1;
	}

#ifdef CONFIG_KC_TOUCH_REGULATOR
	usleep_range(1000, 1000);

	if(ts->reg_touch)
		rc = regulator_set_optimum_mode(ts->reg_touch, KC_TS_LPM_LOAD_UA);
	if(rc < 0)
		pr_err("%s regulator set error\n",__func__);
	else
		KC_TS_DEV_DBG("%s regulator set 0x%02x\n",__func__,rc);
#endif

	KC_TS_DEV_DBG("%s() is completed.\n",__func__);
	return 0;
}

static void pixart_report_clear(struct kc_ts_data *ts)
{
	int i;

	KC_TS_DEV_DBG("%s() is called.\n",__func__);
	for(i = 0; i < KC_TS_MAX_FINGER; i++){
		if(ts->pix_data->slot[i].id == 0)
			continue;

		KC_TS_DEV_DBG("%s: [%d] released\n",__func__, i);
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		ts->pix_data->slot[i].id = 0;
	}

	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);
}

static int pixart_resume(struct kc_ts_data *ts)
{
	int i, err = 0, rc = 0;
	u8 val;

	KC_TS_DEV_DBG("%s is called.\n",__func__);
	ts->vdata->err_cnt = 0;

#ifdef CONFIG_KC_TOUCH_REGULATOR
	if(ts->reg_touch)
		rc = regulator_set_optimum_mode(ts->reg_touch, KC_TS_ACTIVE_LOAD_UA);
	if(rc < 0){
		pr_err("%s regulator set error\n",__func__);
		err = -1;
		goto done;
	} else
		KC_TS_DEV_DBG("%s regulator set 0x%02x\n",__func__,rc);

	usleep_range(1000, 1000);
#endif

	err = pixart_w_reg_byte(ts->vdata->client, 0x7a, 0xdd);
	if(err){
		pr_err("%s:Error Write Resume\n",__func__);
		goto done;
	}

	for (i = 0; i < 75; i++) {
		err = pixart_r_reg_byte(ts->vdata->client, 0x03, &val);
		if(err){
			pr_err("%s:Error Read resume\n", __func__);
			goto done;
		}
		if ((val & 0x91) == 0x91){
			KC_TS_DEV_DBG("%s: Success, retry[%02d]\n",__func__, i);
			break;
		}
		msleep(10);
	}
	if(i >= 75){
		pr_err("%s: Error Status\n",__func__);
		if(pixart_reset_process(ts)){
			pr_err("%s: Failed to restart\n", __func__);
			goto done;
		}
	}

	pixart_enable_irq(ts);
done:
	KC_TS_DEV_DBG("%s() is completed.\n",__func__);
	return err;
}

static int pixart_esd_work(struct kc_ts_data *ts)
{
	u8 val;
	int err = 0;
	struct i2c_client *client = ts->vdata->client;

	err = pixart_r_reg_byte(client, PIXART_REG_PID, &val);
	if(err){
		pr_err("%s: Error Read PRODUCT_ID\n", __func__);
	} else if(pid != val){
		pr_err("%s: IC Error, Recovery Start.\n",__func__);
		err = pixart_reset_process(ts);
		if(err){
			pr_err("%s: Failed to restart\n", __func__);
			goto ic_none;
		}
	} else
		KC_TS_DEV_DBG("%s: Correct PROD_ID [0x%02x]\n",__func__, val);

ic_none:
	return err;
}

static const struct kc_ts_operations pixart_operations = {
	.bustype		= BUS_I2C,
	.read			= pixart_r_reg_byte,
	.multi_read		= pixart_r_reg_nbytes,
	.write			= pixart_w_reg_byte,
#ifdef FEATURE_TOUCH_TEST
	.write_log		= mxt_write_log,
#endif
	.interrupt		= pixart_interrupt,
	.power_off		= pixart_suspend,
	.power_on		= pixart_resume,
	.clear			= pixart_report_clear,
	.input_open		= pixart_input_open,
	.input_close	= pixart_input_close,
	.esd_proc		= pixart_esd_work,
	.ioctl			= pixart_ts_ioctl,
	.reference		= pixart_pixel_dump,
	.reset			= pixart_reset_and_wait,
	.initialize		= pixart_init_panel,
	.resolution		= pixart_calc_resolution,
	.check_reg_nv	= pixart_set_nv,
	.enable			= pixart_enable_irq,
	.disable		= pixart_disable_irq,
};

static struct kc_ts_platform_data pixart_platform_data = {
	.config			= pixart_config_data,
	.config_length	= ARRAY_SIZE(pixart_config_data),
	.x_size			= PIXART_PANEL_WIDTH,
	.y_size			= PIXART_PANEL_HEIGHT,
	.irq_gpio		= PIXART_TS_GPIO_IRQ,
	.reset_gpio		= PIXART_TS_RESET_GPIO,
	.orient			= PIXART_DIAGONAL,
	.irqflags		= IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
	.init_hw		= pixart_init_hw,
	.reset_hw		= pixart_reset_hw,
	.shutdown		= pixart_shutdown_hw,
};

static int __devinit pixart_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	const struct kc_ts_platform_data *pdata;
	struct kc_ts_data *data;
	struct kc_ts_vendor_data *vdata;
	struct kc_ts_pix_data *pix_data;
	int err = 0, i;

	KC_TS_DEV_DBG("%s is called.\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: need I2C_FUNC_I2C\n",__func__);
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct kc_ts_data), GFP_KERNEL);
	if (!data) {
		pr_err("%s: Failed to allocate memory for ts_data\n",__func__);
		return -ENOMEM;
	}

	vdata = kzalloc(sizeof(struct kc_ts_vendor_data), GFP_KERNEL);
	if (!vdata) {
		pr_err("%s: Failed to allocate memory for vdata.\n",__func__);
		err = -ENOMEM;
		goto err_free_mem;
	}

	pix_data = kzalloc(sizeof(struct kc_ts_pix_data), GFP_KERNEL);
	if (!pix_data) {
		pr_err("%s: Failed to allocate memory for pix_data.\n",__func__);
		err = -ENOMEM;
		goto err_free_mem;
	}

	memset(data, 0, sizeof(struct kc_ts_data));
	memset(vdata, 0, sizeof(struct kc_ts_vendor_data));

#ifdef CONFIG_OF
	client->dev.platform_data = &pixart_platform_data;
#endif
	pdata = client->dev.platform_data;
	if (!pdata)
		return -EINVAL;

	data->tops = &pixart_operations;
	data->pdata = pdata;
	data->vdata = vdata;
	data->pix_data = pix_data;
	data->dev = &client->dev;
	data->irq = client->irq;
	data->vdata->client = client;
	data->vdata->touch_number = 0;

	for(i = 0; i < KC_TS_MAX_FINGER; i++){
		data->pix_data->slot[i].id = 0;
		data->pix_data->slot[i].tool_finger = 0;
		data->pix_data->point_data[i].id = 0;
		data->pix_data->point_data[i].x = 0;
		data->pix_data->point_data[i].y = 0;
		data->pix_data->point_data[i].pressure = 0;
		data->pix_data->point_data[i].area = 0;
	}

	data->config_nv[TS_CHARGE_C_NV].data		= pixart_config_charging;
	data->config_nv[TS_CHARGE_C_NV].size		= ARRAY_SIZE(pixart_config_charging);
	data->config_nv[TS_DISCHARGE_NV].data		= pixart_config_discharging;
	data->config_nv[TS_DISCHARGE_NV].size		= ARRAY_SIZE(pixart_config_discharging);
	data->config_nv[TS_CHARGE_A_S1_NV].data		= pixart_config_charging_s1;
	data->config_nv[TS_CHARGE_A_S1_NV].size		= ARRAY_SIZE(pixart_config_charging_s1);
	data->config_nv[TS_CHARGE_A_S2_NV].data		= pixart_config_charging_s2;
	data->config_nv[TS_CHARGE_A_S2_NV].size		= ARRAY_SIZE(pixart_config_charging_s2);
	data->config_nv[TS_WIRELESS_NV].data		= pixart_config_charging_wireless;
	data->config_nv[TS_WIRELESS_NV].size		= ARRAY_SIZE(pixart_config_charging_wireless);
	data->config_nv[TS_INITIAL_VALUE_NV].data	= pixart_config_data;
	data->config_nv[TS_INITIAL_VALUE_NV].size	= ARRAY_SIZE(pixart_config_data);
	data->config_nv[TS_EXTENDED_NV].data		= pixart_extended_data;
	data->config_nv[TS_EXTENDED_NV].size		= ARRAY_SIZE(pixart_extended_data);

	data->ts_sequence = TS_SEQ_PROBE_START;
	err = kc_ts_probe(data);
	if (err)
		goto done;

	KC_TS_DEV_DBG("%s is completed.\n", __func__);
	return 0;

err_free_mem:
	kfree(data);
done:
	return err;
}

static void pixart_shutdown(struct i2c_client *client)
{
	struct kc_ts_data *ts = i2c_get_clientdata(client);

	KC_TS_DEV_DBG("%s is called.\n", __func__);
	kc_ts_shutdown(ts);
	KC_TS_DEV_DBG("%s is completed.\n", __func__);
}

static int pixart_remove(struct i2c_client *client)
{
	struct kc_ts_data *ts = i2c_get_clientdata(client);

	KC_TS_DEV_DBG("%s() is called.\n",__func__);
	kc_ts_remove(ts);
	KC_TS_DEV_DBG("%s is completed\n",__func__);
	return 0;
}

static const struct i2c_device_id pixart_id[] = {
	{ KC_TS_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, pixart_id);

#ifdef CONFIG_OF
static struct of_device_id pixart_match_table[] = {
	{ .compatible = "pixart,kc_touch",},
	{ },
};
#else
#define pixart_match_table NULL
#endif

static struct i2c_driver pixart_driver = {
	.probe		= pixart_probe,
	.remove		= pixart_remove,
	.shutdown	= pixart_shutdown,
	.id_table	= pixart_id,
	.driver = {
		.name	= KC_TS_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = pixart_match_table,
	},
};

static int __init pixart_init(void)
{
	return i2c_add_driver(&pixart_driver);
}

static void __exit pixart_exit(void)
{
	KC_TS_DEV_DBG("%s() is called.\n",__func__);
	i2c_del_driver(&pixart_driver);
}
module_init(pixart_init);
module_exit(pixart_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("Pixart Touchscreen Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("pixart");
