/* 
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"OEM_CHG_WLC_I2C %s: " fmt, __func__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <oem-charger_wlc_i2c.h>
#include <oem-charger_parm.h>

#define OEM_CHG_WLC_I2C_ERR pr_err
//#define FEATURE_CHG_WLC_I2C_DEBUG
#ifdef FEATURE_CHG_WLC_I2C_DEBUG
#define OEM_CHG_WLC_I2C_DBG pr_err
#else
#define OEM_CHG_WLC_I2C_DBG pr_debug
#endif

#define OEM_CHG_WLC_I2C_DEV_NAME "oem_chg_wlc_i2c"

#define I2C_READ_MSG_NUM	2
#define I2C_WRITE_MSG_NUM	1
#define REG_ADR_MAX			0xFF

#define I2C_2_1_DATA_SIZE	2
#define I2C_PROTECT_DATA_SIZE	2

static uint8_t wlc_i2c_protect_reg[I2C_PROTECT_DATA_SIZE] = {
	0xB6,
	0xB6
};
static uint8_t wlc_i2c_protect_data[I2C_PROTECT_DATA_SIZE] = {
	0x5A,
	0xA5
};

static uint8_t wlc_i2c_rw_1_1_reg = {
	0x44
};
static uint8_t wlc_i2c_rw_1_1_data = {
	0x09
};

static uint8_t wlc_i2c_rw_1_2_reg = {
	0x94
};

static uint8_t wlc_i2c_rw_2_1_reg[I2C_2_1_DATA_SIZE] = {
	0x43,
	0x44
};
static uint8_t wlc_i2c_rw_2_2_data[I2C_2_1_DATA_SIZE] = {
	0x0B,
	0x0F
};

static uint8_t wlc_i2c_rw_2_2_reg = {
	0x94
};

static struct i2c_client *client_hold = NULL;
static unsigned int suspend_state = 0;

static
int __oem_chg_wlc_i2c_write(u8 reg, u8 *wbuf, int len)
{
	int ret = 0;
	struct i2c_msg i2cMsg[I2C_WRITE_MSG_NUM];
	static u8 buff[REG_ADR_MAX];
	int i;

	OEM_CHG_WLC_I2C_DBG("reg[%02X] len[%d]",reg, len );

	if (suspend_state)
	{
		return -EBUSY;
	}

	if (client_hold == NULL)
	{
		return -ENODEV;
	}

	buff[0] = reg;
	memcpy(&buff[1], wbuf, len);

	i2cMsg[0].addr = client_hold->addr;
	i2cMsg[0].flags = 0;
	i2cMsg[0].len = len + 1;
	i2cMsg[0].buf = (u8 *)buff;

	ret = i2c_transfer(client_hold->adapter, &i2cMsg[0], I2C_WRITE_MSG_NUM);
	OEM_CHG_WLC_I2C_DBG("i2c_transfer() called. ret[%d][0x%02X]",ret, buff[1]);

	if (ret == I2C_WRITE_MSG_NUM)
	{
		OEM_CHG_WLC_I2C_DBG("end. exec mesg[%d]",ret);
		for (i = 0; i < len; i++)
		{
			OEM_CHG_WLC_I2C_DBG("i2c write reg[%02X] value[%02X]",
						 (unsigned int)(reg + i), (unsigned int)*(wbuf + i));
		}
		OEM_CHG_WLC_I2C_DBG("end - return0");
		return 0;
	}
	OEM_CHG_WLC_I2C_ERR("i2c transfer error[%d]",ret );

	return -1;
}

int oem_chg_wlc_i2c_write_1st(void)
{
	int i;
	int rc = 0;

	for(i=0; i<I2C_PROTECT_DATA_SIZE; i++) {
		rc = __oem_chg_wlc_i2c_write(wlc_i2c_protect_reg[i], &wlc_i2c_protect_data[i], 1);
		if(rc)
			break;
	}
	if(!rc) {
		rc = __oem_chg_wlc_i2c_write(wlc_i2c_rw_1_1_reg, &wlc_i2c_rw_1_1_data, 1);
	}
	if(!rc) {
		rc = __oem_chg_wlc_i2c_write(wlc_i2c_rw_1_2_reg, &oem_param_charger.i2c_1st_data, 1);
	}
	return rc;
}

int oem_chg_wlc_i2c_write_2nd(void)
{
	int i;
	int rc = 0;

	for(i=0; i<I2C_2_1_DATA_SIZE; i++) {
		rc = __oem_chg_wlc_i2c_write(wlc_i2c_rw_2_1_reg[i], &wlc_i2c_rw_2_2_data[i], 1);
		if(rc) {
			break;
		}
	}
	if(!rc) {
		rc = __oem_chg_wlc_i2c_write(wlc_i2c_rw_2_2_reg, &oem_param_charger.i2c_2nd_data, 1);
	}
	return rc;
}

static
int __devinit oem_chg_wlc_i2c_probe(struct i2c_client *client,
									const struct i2c_device_id *id)
{
	OEM_CHG_WLC_I2C_DBG("is called.\n");

	client_hold = client;

	return 0;
}

static
int oem_chg_wlc_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static
void oem_chg_wlc_i2c_shutdown(struct i2c_client *client)
{
	return;
}

static
int oem_chg_wlc_i2c_suspend(struct device *dev)
{
	suspend_state = 1;
	return 0;
}

static
int oem_chg_wlc_i2c_resume(struct device *dev)
{
	suspend_state = 0;
	return 0;
}

static const struct i2c_device_id oem_chg_wlc_i2c_id[] = {
	{ OEM_CHG_WLC_I2C_DEV_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, oem_chg_wlc_i2c_id);

static struct of_device_id oem_chg_wlc_i2c_match_table[] = {
	{ .compatible = "kc,oem_chg_wlc_i2c", },
	{ },
};

static const struct dev_pm_ops oem_chg_wlc_i2c_pm_ops = {
	.suspend	= oem_chg_wlc_i2c_suspend,
	.resume		= oem_chg_wlc_i2c_resume,
};

static struct i2c_driver oem_chg_wlc_i2c_driver = {
	.probe		= oem_chg_wlc_i2c_probe,
	.remove		= oem_chg_wlc_i2c_remove,
	.shutdown	= oem_chg_wlc_i2c_shutdown,
	.id_table	= oem_chg_wlc_i2c_id,
	.driver = {
		.name	= OEM_CHG_WLC_I2C_DEV_NAME,
		.owner	= THIS_MODULE,
		.pm 	= &oem_chg_wlc_i2c_pm_ops,
		.of_match_table = oem_chg_wlc_i2c_match_table,
	},
};

static int __init oem_chg_wlc_i2c_init(void)
{
	return i2c_add_driver(&oem_chg_wlc_i2c_driver);
}

static void __exit oem_chg_wlc_i2c_exit(void)
{
	i2c_del_driver(&oem_chg_wlc_i2c_driver);
}
module_init(oem_chg_wlc_i2c_init);
module_exit(oem_chg_wlc_i2c_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("OEM Charger i2c Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("oem_chg_wlc_i2c");
