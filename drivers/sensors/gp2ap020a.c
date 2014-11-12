/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include "sensor_driver.h"
#include "gp2ap020a.h"
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/vreg.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/sensor_power.h>
#include <linux/wakelock.h>
#include <mach/gpio.h>

#define DBG_PSALS                           (0)
#define DEVICE_NAME                         "GP2AP020A00F"
#define DEVICE_FILE_NAME                    "psals_sensor"
#define LIGHT_SENSOR_NAME                   "light sensor"
#define PROXIMITY_SENSOR_NAME               "proximity sensor"

#define REG_ADR_00                          0x00
#define REG_ADR_01                          0x01
#define REG_ADR_02                          0x02
#define REG_ADR_03                          0x03
#define REG_ADR_04                          0x04
#define REG_ADR_05                          0x05
#define REG_ADR_06                          0x06
#define REG_ADR_07                          0x07
#define REG_ADR_08                          0x08
#define REG_ADR_09                          0x09
#define REG_ADR_0A                          0x0A
#define REG_ADR_0B                          0x0B
#define REG_ADR_0C                          0x0C
#define REG_ADR_0D                          0x0D
#define REG_ADR_0E                          0x0E
#define REG_ADR_0F                          0x0F
#define REG_ADR_10                          0x10
#define REG_ADR_11                          0x11
#define REG_ADR_MAX                         0x12

#define ABS_LUX_REPORT                      (ABS_MISC)
#define ABS_DISTANCE_REPORT                 (ABS_DISTANCE)

#define GP2AP_DEV_STATUS_INIT               0x00000000
#define GP2AP_DEV_STATUS_SUSPEND            0x00000001
#define GP2AP_DEV_STATUS_SUSPEND_INT        0x00000002
#define GP2AP_DEV_STATUS_RESUME             0x00000004

#define ALS_POLLING_CNT_RESET_NONE          0x00000000
#define ALS_POLLING_CNT_RESET_MAX_DATA      0x00000001
#define ALS_POLLING_CNT_RESET_DISABLE       0x00000002
#define ALS_POLLING_CNT_RESET_STORE_POLL    0x00000004
#define ALS_POLLING_CNT_RESET_STORE_TIME    0x00000008
#define ALS_POLLING_CNT_RESET_INIT          0x00000010
#define ALS_POLLING_CNT_RESET_RESUME        0x00000020

#define GP2AP_OP_MODE_NONE                  0x00000000
#define GP2AP_OP_MODE_PS                    0x00000001
#define GP2AP_OP_MODE_ALS                   0x00000002
#define GP2AP_OP_MODE_PS_ALS                0x00000003
#define GP2AP_OP_MODE_NONE_MASK             0x08

#define GP2AP_SENSOR_NONE                   0x00000000
#define GP2AP_SENSOR_PS                     0x00000001
#define GP2AP_SENSOR_ALS                    0x00000002

#define GP2AP_WAKE_LOCK_TIME                (HZ * 10)
#define GP2AP_WAKE_LOCK_INPUT_TIME          (HZ)
#define GP2AP_ERR_MAX_CNT                   5
#define GP2AP_COLOR_VARI                    10
#define GPIO_PROX_INT                       74
#define GPIO_PROX_EN                        76

#define VPRO_VREG_NAME                      "8941_l1"
#define VPRO_UV                             2950000

#define ALS_ON_WAIT_MS                      210
#define ALS_ON_DELAY_TIMER_MS               (ALS_ON_WAIT_MS + 10)
#define PS_ON_WAIT_MS                       5
#define PS_OFF_WAIT_MS                      10

#define ALS_IRDATA_MAX                      0x3FFF
#define ALS_CDATA_MAX                       0x3FFF
#define ALS_GET_DATA_INTERVAL_MS            1000
#define ALS_POLL_DELAY_MS_DEF               250
#define ALS_MEAN_TIME_DEF                   4

#define PS_JUDGE_COUNT                      4
#define I2C_WRITE_MSG_NUM                   1
#define I2C_READ_MSG_NUM                    2

#define LUX_CALC_GAMMA_ALS                  2
#define LUX_CALC_GAMMA_PSALS                8
#define LUX_CALC_GAMMA_UNIT                 1

#define GPIO_ON                             (1)
#define GPIO_OFF                            (0)

#define PS_DETECTION_MASK                   0x08

enum vsensor_power
{
    VSENSOR_OFF = 0,
    VSENSOR_ON,
};

enum vpro_power
{
    VPRO_OFF = 0,
    VPRO_ON,
};

enum op_flag
{
    OP_OFF = 0,
    OP_ON,
};

static struct sensor_power_callback gp2ap_power_cb;

static struct i2c_client *client_gp2ap = NULL;

static struct wake_lock gp2ap_wake_lock;
static struct wake_lock gp2ap_wake_lock_input;

static struct reg_data reg_init_data[] =
{
    {REG_ADR_00, 0x00},
    {REG_ADR_01, 0x5B},
    {REG_ADR_02, 0x72},
    {REG_ADR_03, 0x38},
};

static u16 nv_proximity_sensor_near[10] =
    {0x0025,0x0025,0x0025,0x0025,0x0025,0x0025,0x0025,0x0025,0x0025,0x0025};
static u16 nv_proximity_sensor_far[10] =
    {0x0020,0x0020,0x0020,0x0020,0x0020,0x0020,0x0020,0x0020,0x0020,0x0020};
static u16 nv_photo_sensor_beamish[1] = {0x0064};
static u16 nv_photo_sensor_a_035[10] =
    {0x0D20,0x0D20,0x0D20,0x0D20,0x0D20,0x0D20,0x0D20,0x0D20,0x0D20,0x0D20};
static u16 nv_photo_sensor_a_067[10] =
    {0x251C,0x251C,0x251C,0x251C,0x251C,0x251C,0x251C,0x251C,0x251C,0x251C};
static u16 nv_photo_sensor_a_093[10] =
    {0x03FB,0x03FB,0x03FB,0x03FB,0x03FB,0x03FB,0x03FB,0x03FB,0x03FB,0x03FB};
static u16 nv_photo_sensor_a_max[10] =
    {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
static u16 nv_photo_sensor_b_035[10] =
    {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
static u16 nv_photo_sensor_b_067[10] =
    {0x391C,0x391C,0x391C,0x391C,0x391C,0x391C,0x391C,0x391C,0x391C,0x391C};
static u16 nv_photo_sensor_b_093[10] =
    {0x0410,0x0410,0x0410,0x0410,0x0410,0x0410,0x0410,0x0410,0x0410,0x0410};
static u16 nv_photo_sensor_b_max[10] =
    {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
static u16 nv_photo_sensor_a25ms_035[10] =
    {0x05B4,0x05B4,0x05B4,0x05B4,0x05B4,0x05B4,0x05B4,0x05B4,0x05B4,0x05B4};
static u16 nv_photo_sensor_a25ms_067[10] =
    {0x05B4,0x05B4,0x05B4,0x05B4,0x05B4,0x05B4,0x05B4,0x05B4,0x05B4,0x05B4};
static u16 nv_photo_sensor_a25ms_093[10] =
    {0x03FB,0x03FB,0x03FB,0x03FB,0x03FB,0x03FB,0x03FB,0x03FB,0x03FB,0x03FB};
static u16 nv_photo_sensor_a25ms_max[10] =
    {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
static u16 nv_photo_sensor_b25ms_035[10] =
    {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
static u16 nv_photo_sensor_b25ms_067[10] =
    {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
static u16 nv_photo_sensor_b25ms_093[10] =
    {0x0410,0x0410,0x0410,0x0410,0x0410,0x0410,0x0410,0x0410,0x0410,0x0410};
static u16 nv_photo_sensor_b25ms_max[10] =
    {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
static u16 nv_prox_photo_colvar[1] = {0x0000};
static u16 nv_proximity_sensor_temp[3] = {0x000A,0x0000,0x000A};

static u32 nv_status = 0;
static atomic_t g_dev_status;
static atomic_t gp2ap_resetstatus;
static atomic_t g_update_threshold_flg;
static struct workqueue_struct *als_polling_wq;
static struct workqueue_struct *ps_polling_wq;
static s32 gp2ap_ps_irq_cnt = 0;
static u32 gp2ap_initialize = 0;

static u32 gpio_config_prox_int_pd[] = {
    GPIO_CFG(GPIO_PROX_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};
static u32 gpio_config_prox_int_pu[] = {
    GPIO_CFG(GPIO_PROX_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static s32 gp2ap_i2c_read(struct i2c_client *client, u8 reg, u8 *rbuf, int len);
static s32 gp2ap_i2c_write(struct i2c_client *client, u8 reg, u8 *wbuf, int len);
static void gp2ap_enable_ps_irq(struct gp2ap_data *data);
static void gp2ap_disable_ps_irq(struct gp2ap_data *data);
static u32 LuxCalculation(u32 cdata, u32 irdata, struct gp2ap_data *data);
static int gp2ap_get_als_data(struct i2c_client *client, u32 *cdata, u32 *irdata);
static void gp2ap_vsensor_onoff(enum vsensor_power onoff);
static void gp2ap_vpro_onoff(struct gp2ap_data *data, enum vpro_power onoff);
static u32 RatioCalculation(struct i2c_client *client, struct gp2ap_data *data);
static int gp2ap_set_op_mode(struct i2c_client *client, u32 sensor_kind, enum op_flag onoff);
static int gp2ap_init_client(struct i2c_client *client);
static int gp2ap_fin_client(struct i2c_client *client);
static void gp2ap_power_on(void);
static void gp2ap_power_off(void);
static void gp2ap_reschedule_work(struct gp2ap_data *data, unsigned long delay);
static void gp2ap_put_luxValue(struct gp2ap_data *data, u32 luxValue);
static void gp2ap_als_on_work_handler(struct work_struct *work);
static void gp2ap_als_data_work_handler(struct work_struct *work);
static void gp2ap_work_handler(struct work_struct *work);
static irqreturn_t gp2ap_interrupt(int vec, void *info);
static int gp2ap_set_ps_threshold(struct i2c_client *client, u16 low, u16 high);
static void gp2ap_set_als_mean_times(struct gp2ap_data *data, u32 mean_times);
static void gp2ap_enable_als_sensor(struct i2c_client *client, enum sensor_enable enable);
static void gp2ap_enable_ps_sensor(struct i2c_client *client, enum sensor_enable enable);

static int gp2ap_resume(struct i2c_client *client);
static int gp2ap_suspend(struct i2c_client *client, pm_message_t mesg);
static void gp2ap020a_shutdown(struct i2c_client *client);
static int __devexit gp2ap_remove(struct i2c_client *client);
static int __devinit gp2ap_probe(
    struct i2c_client *client,
    const struct i2c_device_id *id);

static s32 gp2ap_i2c_read(struct i2c_client *client, u8 reg, u8 *rbuf, int len)
{
    int ret = 0;
    struct i2c_msg i2cMsg[I2C_READ_MSG_NUM];
    u8 buff;
    int retry = GP2AP_ERR_MAX_CNT;
    int i;

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("reg[%02X] len[%d]",reg, len );

    if (client == NULL)
    {
        return -ENODEV;
    }

    while (retry--)
    {
        i2cMsg[0].addr = client->addr;
        i2cMsg[0].flags = 0;
        i2cMsg[0].len = 1;
        i2cMsg[0].buf = &buff;
        buff = reg;
        i2cMsg[1].addr = client->addr;
        i2cMsg[1].flags = I2C_M_RD;
        i2cMsg[1].len = len;
        i2cMsg[1].buf = rbuf;

        ret = i2c_transfer(client->adapter, &i2cMsg[0], I2C_READ_MSG_NUM);
        SENSOR_N_LOG("i2c_transfer() called. ret[%d]",ret);
        if (ret == I2C_READ_MSG_NUM)
        {
            SENSOR_N_LOG("end. exec mesg[%d]",(int)ret);
            for (i = 0; i < len; i++)
            {
                SENSOR_N_LOG("i2c read reg[%02X] value[%02X]",
                             (unsigned int)(reg + i), (unsigned int)*(rbuf + i));
            }
            SENSOR_N_LOG("end - return0");
            return 0;
        }
    }

    SENSOR_ERR_LOG("i2c transfer error[%d]",ret );
    return -1;
}

static s32 gp2ap_i2c_write(struct i2c_client *client, u8 reg, u8 *wbuf, int len)
{
    int ret = 0;
    struct i2c_msg i2cMsg[I2C_WRITE_MSG_NUM];
    static u8 buff[REG_ADR_MAX];
    int retry = GP2AP_ERR_MAX_CNT;
    int i;

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("reg[%02X] len[%d]",reg, len );

    if (client == NULL)
    {
         return -ENODEV;
    }

    while (retry--)
    {
        buff[0] = reg;
        memcpy(&buff[1], wbuf, len);

        i2cMsg[0].addr = client->addr;
        i2cMsg[0].flags = 0;
        i2cMsg[0].len = len + 1;
        i2cMsg[0].buf = (u8 *)buff;

        ret = i2c_transfer(client->adapter, &i2cMsg[0], I2C_WRITE_MSG_NUM);
        SENSOR_N_LOG("i2c_transfer() called. ret[%d]",ret);

        if (ret == I2C_WRITE_MSG_NUM)
        {
            SENSOR_N_LOG("end. exec mesg[%d]",ret);
            for (i = 0; i < len; i++)
            {
                SENSOR_N_LOG("i2c write reg[%02X] value[%02X]",
                             (unsigned int)(reg + i), (unsigned int)*(wbuf + i));
            }
            SENSOR_N_LOG("end - return0");
            return 0;
        }
    }
    SENSOR_ERR_LOG("i2c transfer error[%d]",ret );

    return -1;
}

static void gp2ap_enable_ps_irq(struct gp2ap_data *data)
{

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("before gp2ap_ps_irq_cnt[%d]",gp2ap_ps_irq_cnt );

    if (gp2ap_ps_irq_cnt <= 0)
    {
        SENSOR_N_LOG("enable_irq");
        enable_irq(data->ps_irq);
        gp2ap_ps_irq_cnt++;
    }
    SENSOR_N_LOG("after gp2ap_ps_irq_cnt[%d]",gp2ap_ps_irq_cnt );
    SENSOR_N_LOG("end");
}

static void gp2ap_disable_ps_irq(struct gp2ap_data *data)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("before gp2ap_ps_irq_cnt[%d]",gp2ap_ps_irq_cnt );
    if (gp2ap_ps_irq_cnt > 0)
    {
        gp2ap_ps_irq_cnt--;
        disable_irq(data->ps_irq);
        SENSOR_N_LOG("disable_irq");
    }
    SENSOR_N_LOG("after gp2ap_ps_irq_cnt[%d]",gp2ap_ps_irq_cnt );
    SENSOR_N_LOG("end");
}

static u32 LuxCalculation(u32 cdata, u32 irdata, struct gp2ap_data *data)
{
    u32 lux;
    u32 ratio;
    u16 alpha = 0;
    u16 beta = 0;
    u16 gamma = LUX_CALC_GAMMA_ALS;
    u16 colvar = nv_prox_photo_colvar[0];

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("irdata[%d] cdata[%d] op_mode[%08X]",
                  irdata, cdata, (unsigned int)data->op_mode);

    if((data->op_mode == GP2AP_OP_MODE_ALS) &&
                         ( (cdata >= 0xFDE8) || (irdata >= 0xFDE8)) )
    {
        SENSOR_ERR_LOG("GP2AP_OP_MODE_ALS cdata[%d] irdata[%d]", cdata, irdata );
        return GP2AP_LUXVALUE_MAX;
    }
    else if((data->op_mode == GP2AP_OP_MODE_PS_ALS) &&
            ( (cdata >= 0x3E80) || (irdata >= 0x3E80)) )
    {
        SENSOR_ERR_LOG("GP2AP_OP_MODE_PS_ALS cdata[%d] irdata[%d]", cdata, irdata );
        return GP2AP_LUXVALUE_MAX;
    }

    if (cdata == 0)
    {
         ratio = 100;
    }
    else
    {
        ratio = (irdata * 100) / cdata;
    }

    if ((irdata == 0) || (cdata == 0))
    {
        lux = 0;
        SENSOR_N_LOG("set lux[%d]", lux);
    }
    else
    {
        if(data->op_mode == GP2AP_OP_MODE_PS_ALS)
        {
            gamma = LUX_CALC_GAMMA_PSALS;
        }
        else
        {
            gamma = LUX_CALC_GAMMA_ALS;
        }

        if (ratio <= 48)
        {
            alpha = nv_photo_sensor_a_035[colvar];
            beta = nv_photo_sensor_b_035[colvar];
        }
        else if (ratio <= 55)
        {
            alpha = nv_photo_sensor_a_067[colvar];
            beta = nv_photo_sensor_b_067[colvar];
        }
        else if (ratio <= 60)
        {
            alpha = nv_photo_sensor_a25ms_035[colvar];
            beta = nv_photo_sensor_b25ms_035[colvar];
        }
        else if (ratio <= 81)
        {
            alpha = nv_photo_sensor_a25ms_067[colvar];
            beta = nv_photo_sensor_b25ms_067[colvar];
        }
        else if (ratio <= 97)
        {
            alpha = nv_photo_sensor_a_093[colvar];
            beta = nv_photo_sensor_b_093[colvar];
        }
        else
        {
            SENSOR_N_LOG("ratio > 97[%d]", data->als_lux);
            return data->als_lux;
        }
        SENSOR_N_LOG("ratio[%d] alpha[%04X] beta[%04X] gamma[%04X]",
                      ratio, alpha, beta, gamma);
        lux = (gamma * ((alpha * cdata - beta * irdata) / 100))/100
                                             / LUX_CALC_GAMMA_UNIT;
    }
    SENSOR_N_LOG("end. calc lux[%d]",lux);
    SENSOR_N_LOG("end");

    return lux;
}

static int gp2ap_get_als_data(struct i2c_client *client, u32 *cdata, u32 *irdata)
{
    int ret = 0;
    u8 buf[4];

    SENSOR_N_LOG("start");

    ret = gp2ap_i2c_read(client, REG_ADR_0C, buf, 4);
    if (ret < 0)
    {
        goto exit;
    }

    *cdata = (buf[1] << 8) | buf[0];
    *irdata = (buf[3] << 8) | buf[2];

    SENSOR_N_LOG("cdata[%04X] irdata[%04X]", *cdata, *irdata);

exit:
    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

static void gp2ap_vsensor_onoff(enum vsensor_power onoff)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("onoff[%d]",onoff);

    if (onoff == VSENSOR_ON)
    {
        sensor_power_reg_cbfunc(&gp2ap_power_cb);
        SENSOR_N_LOG("sensor_power_on(SENSOR_INDEX_PSALS) call.");
    }
    else
    {
        sensor_power_unreg_cbfunc(&gp2ap_power_cb);
        SENSOR_N_LOG("sensor_power_off(SENSOR_INDEX_PSALS) call.");
    }
    SENSOR_N_LOG("end");
}

static void gp2ap_vpro_onoff(struct gp2ap_data *data, enum vpro_power onoff)
{
    s32 rc = 0;

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("onoff[%d] ps_en_gpio[%d]",onoff ,data->ps_en_gpio);

    if (onoff == VPRO_ON)
    {
        if (data->ps_en_gpio >= 0)
        {
            SENSOR_N_LOG("VPRO[%d]",data->ps_en_gpio);
            gpio_set_value_cansleep(data->ps_en_gpio, GPIO_ON);
            SENSOR_N_LOG("msleep[%d]",PS_ON_WAIT_MS);
            msleep(PS_ON_WAIT_MS);
            SENSOR_N_LOG("msleep[%d]",PS_ON_WAIT_MS);
        }
        else
        {
            if (data->vpro_vreg)
            {
                SENSOR_N_LOG("VPRO[%s]",VPRO_VREG_NAME);
                rc = regulator_set_voltage(data->vpro_vreg, VPRO_UV, VPRO_UV);
                if (rc)
                    SENSOR_ERR_LOG("regulator_set_voltage[%s] error[%d]", VPRO_VREG_NAME, rc);
                rc = regulator_enable(data->vpro_vreg);
                if (rc)
                    SENSOR_ERR_LOG("regulator_enable[%s] error[%d]", VPRO_VREG_NAME, rc);
                SENSOR_N_LOG("msleep[%d]",PS_ON_WAIT_MS);
                msleep(PS_ON_WAIT_MS);
                SENSOR_N_LOG("msleep[%d]",PS_ON_WAIT_MS);
            }
        }
    }
    else
    {
        if (data->ps_en_gpio >= 0)
        {
            SENSOR_N_LOG("VPRO[%d]",data->ps_en_gpio);
            gpio_set_value_cansleep(data->ps_en_gpio, GPIO_OFF);
            SENSOR_N_LOG("msleep[%d]",PS_OFF_WAIT_MS);
            msleep(PS_OFF_WAIT_MS);
            SENSOR_N_LOG("msleep[%d]",PS_OFF_WAIT_MS);
        }
        else
        {
            if (data->vpro_vreg)
            {
                SENSOR_N_LOG("VPRO[%s]",VPRO_VREG_NAME);
                rc = regulator_disable(data->vpro_vreg);
                if (rc)
                    SENSOR_ERR_LOG("regulator_disable[%s] error[%d]", VPRO_VREG_NAME, rc);
            }
        }
    }
    SENSOR_N_LOG("end");
}

static int gp2ap_set_ps_threshold(struct i2c_client *client, u16 low, u16 high)
{
    struct gp2ap_data *data = i2c_get_clientdata(client);
    int ret = 0;
    u8 buf[4];
    u8 mode[1];
    u8 s_mode[1];

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("low[%04X] high[%04X]",low, high);

    ret = gp2ap_i2c_read(client, REG_ADR_00, mode, 1);
    if (ret < 0)
    {
        SENSOR_ERR_LOG("gp2ap_i2c_read error[%d]", ret);
        return ret;
    }

    s_mode[0] = mode[0] & GP2AP_OP_MODE_NONE_MASK;
    gp2ap_i2c_write(client, REG_ADR_00, s_mode, 1);

    if( (mode[0] & 0xF0) == 0xC0)
    {
        gp2ap_vpro_onoff(data, VPRO_OFF);
    }

    buf[0] = (u8)(0x00FF & low);
    buf[1] = (u8)(0x00FF & (low >> 8));
    buf[2] = (u8)(0x00FF & high);
    buf[3] = (u8)(0x00FF & (high >> 8));
    ret = gp2ap_i2c_write(client, REG_ADR_08, buf, sizeof(buf));

    if( (mode[0] & 0xf0) == 0xC0)
    {
        gp2ap_vpro_onoff(data, VPRO_ON);
    }
    ret = gp2ap_i2c_write(client, REG_ADR_00, mode, 1);

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

int gp2ap_set_ioctl_ps_threshold(void)
{
    int colvar = nv_prox_photo_colvar[0];
    struct gp2ap_data *data = i2c_get_clientdata(client_gp2ap);
    u32 dev_status = 0;
    int ret = 0;

    SENSOR_N_LOG("start");

    if(data->enable_als_sensor > 0){
        atomic_set(&g_update_threshold_flg, true);
    } else {
        dev_status = atomic_read(&g_dev_status);
        dev_status &= GP2AP_DEV_STATUS_SUSPEND_INT;
        if(dev_status){
            atomic_set(&g_update_threshold_flg, true);
        }else{
            atomic_set(&g_update_threshold_flg, false);
            ret = gp2ap_set_ps_threshold(client_gp2ap, nv_proximity_sensor_far[colvar],
                                   nv_proximity_sensor_near[colvar]);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

static u32 RatioCalculation(struct i2c_client *client, struct gp2ap_data *data)
{
    u32 cdata;
    u32 irdata;
    u32 ratio;
    int ret;

    SENSOR_N_LOG("start");

    ret = gp2ap_get_als_data(client, &cdata, &irdata);

    if (ret < 0)
    {
        SENSOR_ERR_LOG("gp2ap_get_als_data error.[%d]", ret);
        return ret;
    }

    data->cdata_reg = cdata;
    data->irdata_reg = irdata;

    if (cdata == 0)
    {
        ratio = 100;
    }
    else
    {
        ratio = (irdata * 100) / cdata;
    }

    SENSOR_N_LOG("ratio[%d]",ratio);
    SENSOR_N_LOG("end");

    return ratio;
}

static int gp2ap_set_op_mode(struct i2c_client *client, u32 sensor_kind, enum op_flag onoff)
{
    struct gp2ap_data *data = i2c_get_clientdata(client);
    int ret = 0;
    u8 buf[1];
    u32 pre_op_mode = data->op_mode;

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("sensor_kind[%08X] onoff[%d]",sensor_kind, onoff);
    SENSOR_N_LOG("before op_mode[%08X] op_sensor[%08X]",
                  (unsigned int)data->op_mode, (unsigned int)data->op_sensor);

    if (onoff == OP_ON)
        data->op_sensor |= sensor_kind;
    else
        data->op_sensor &= ~sensor_kind;

    switch (data->op_sensor)
    {
        case GP2AP_SENSOR_NONE:
            data->op_mode = GP2AP_OP_MODE_NONE;
            break;
        case GP2AP_SENSOR_PS:
        case GP2AP_SENSOR_PS | GP2AP_SENSOR_ALS:
            data->op_mode = GP2AP_OP_MODE_PS_ALS;
            break;
        case GP2AP_SENSOR_ALS:
            data->op_mode = GP2AP_OP_MODE_ALS;
            break;
        default:
            SENSOR_ERR_LOG("illegal status. op_sensor[%08X]",
                            (unsigned int)data->op_sensor);
            data->op_mode = GP2AP_OP_MODE_NONE;
            break;
    }

    if (pre_op_mode != data->op_mode)
    {
        if(pre_op_mode == GP2AP_OP_MODE_PS_ALS)
        {
            buf[0] = 0x00;
            ret = gp2ap_i2c_write(client, REG_ADR_00, buf, 1);
            if (ret < 0)
            {
                SENSOR_ERR_LOG("gp2ap_i2c_write error.[%d]", ret);
            }
        }

        switch (data->op_mode)
        {
            case GP2AP_OP_MODE_NONE:
                buf[0] = 0x00;
                break;
            case GP2AP_OP_MODE_ALS:
                buf[0] = 0x5B;
                break;
            case GP2AP_OP_MODE_PS_ALS:
                buf[0] = 0x63;
                break;
            case GP2AP_OP_MODE_PS:
            default:
                SENSOR_ERR_LOG("illegal status. op_mode[%08X]",
                                (unsigned int)data->op_mode);
            break;
        }

        if(data->op_mode == GP2AP_OP_MODE_ALS)
        {
            ret = gp2ap_i2c_write(client, REG_ADR_01, buf, 1);
            if (ret < 0)
            {
                SENSOR_ERR_LOG("gp2ap_i2c_write error[%d]", ret);
            }
            SENSOR_N_LOG("op_mode=GP2AP_OP_MODE_ALS");
            buf[0] = 0x90;
        }
        else if(data->op_mode == GP2AP_OP_MODE_PS_ALS)
        {
            gp2ap_vpro_onoff(data, VPRO_ON);
            ret = gp2ap_i2c_write(client, REG_ADR_01, buf, 1);
            if (ret < 0)
            {
                SENSOR_ERR_LOG("gp2ap_i2c_write error[%d]", ret);
            }
            SENSOR_N_LOG("op_mode=GP2AP_OP_MODE_PS_ALS");
            buf[0] = 0xC0;
        }

        ret = gp2ap_i2c_write(client, REG_ADR_00, buf, 1);
        if (ret < 0)
        {
            SENSOR_ERR_LOG("gp2ap_i2c_write error[%d]", ret);
        }
    }

    SENSOR_N_LOG("after op_mode[%08X] op_sensor[%08X]",
                 (unsigned int)data->op_mode, (unsigned int)data->op_sensor);
    SENSOR_N_LOG("end");

    return ret;
}

static void gp2ap_enable_als_sensor(struct i2c_client *client, enum sensor_enable enable)
{
    struct gp2ap_data *data = i2c_get_clientdata(client);
    int ret = 0;

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("enable[%08X] enable_ps_sensor[%d]",enable, data->enable_ps_sensor);

    if (enable == SENSOR_ENABLE)
    {
        SENSOR_N_LOG("queue_delayed_work(als_on_dwork) call. ");
        queue_delayed_work(als_polling_wq, &data->als_on_dwork,
                           msecs_to_jiffies(0));
    }
    else
    {
        mutex_unlock(&data->psals_mutex);
        cancel_delayed_work_sync(&data->als_on_dwork);
        cancel_delayed_work_sync(&data->als_data_dwork);
        mutex_lock(&data->psals_mutex);

        ret = gp2ap_set_op_mode(client, GP2AP_SENSOR_ALS, OP_OFF);
        if (ret < 0)
        {
            SENSOR_ERR_LOG("gp2ap_set_op_mode error.[%d]", ret);
        }

        data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_DISABLE;
        SENSOR_N_LOG("als_polling_cnt_reset[%08X]",data->als_polling_cnt_reset );
    }

    SENSOR_N_LOG("end");
}

void gp2ap_als_sensor_activate(enum sensor_enable enable)
{
    struct gp2ap_data *data = i2c_get_clientdata(client_gp2ap);

    mutex_lock(&data->psals_mutex);

    SENSOR_N_LOG("start");

    if (enable == SENSOR_ENABLE) {
        if (data->enable_als_sensor <= 0) {
            data->enable_als_sensor = 1;
            gp2ap_enable_als_sensor(client_gp2ap, enable);
        } else {
            data->enable_als_sensor++;
        }
    } else {
        data->enable_als_sensor--;
        if (data->enable_als_sensor > 0) {
            SENSOR_N_LOG("no transaction.");
        } else {
            gp2ap_enable_als_sensor(client_gp2ap, enable);
        }
    }

    SENSOR_N_LOG("end");

    mutex_unlock(&data->psals_mutex);
}

static void gp2ap_enable_ps_sensor(struct i2c_client *client, enum sensor_enable enable)
{
    struct gp2ap_data *data = i2c_get_clientdata(client);
    int ret = 0;
    u16 colvar = nv_prox_photo_colvar[0];

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("enable[%d] enable_als_sensor[%d]",enable, data->enable_als_sensor);

    if (enable == 1)
    {
        if (data->enable_als_sensor > 0)
            gp2ap_enable_als_sensor(client_gp2ap, SENSOR_DISABLE);

        ret = gp2ap_set_ps_threshold(client, nv_proximity_sensor_far[colvar],
                  nv_proximity_sensor_near[colvar]);
        if (ret < 0)
        {
            SENSOR_ERR_LOG("gp2ap_set_ps_threshold error.[%d]", ret);
            goto exit;
        }

        ret = gp2ap_set_op_mode(client, GP2AP_SENSOR_PS, OP_ON);
        if (ret < 0)
        {
            SENSOR_ERR_LOG("gp2ap_set_op_mode error.[%d]", ret);
            goto exit;
        }

        if(data->op_mode == GP2AP_OP_MODE_PS_ALS)
        {
            msleep(30);
            data->ratio_reg = RatioCalculation(client, data);
        }

        if (data->enable_als_sensor > 0)
            gp2ap_enable_als_sensor(client_gp2ap, SENSOR_ENABLE);
        gp2ap_enable_ps_irq(data);
    }
    else
    {
        gp2ap_disable_ps_irq(data);

        if (data->enable_als_sensor > 0)
        {
            gp2ap_enable_als_sensor(client_gp2ap, SENSOR_DISABLE);
        }

        ret = gp2ap_set_op_mode(client, GP2AP_SENSOR_PS, OP_OFF);
        if (ret < 0)
        {
            SENSOR_ERR_LOG("gp2ap_set_op_mode error.[%d]", ret);
            goto exit;
        }

        gp2ap_vpro_onoff(data, VPRO_OFF);
        if (data->enable_als_sensor > 0)
        {
            gp2ap_enable_als_sensor(client_gp2ap, SENSOR_ENABLE);
        }
    }

exit:
    SENSOR_N_LOG("end");
}

void gp2ap_ps_sensor_activate(enum sensor_enable enable)
{
    struct gp2ap_data *data = i2c_get_clientdata(client_gp2ap);

    mutex_lock(&data->psals_mutex);

    SENSOR_N_LOG("start");

    if (enable == SENSOR_ENABLE) {
        if (data->enable_ps_sensor <= 0) {
            data->enable_ps_sensor=1;
            gp2ap_enable_ps_sensor(client_gp2ap, enable);
        } else {
            data->enable_ps_sensor++;
        }
    } else {
        data->enable_ps_sensor--;
        if (data->enable_ps_sensor > 0) {
            SENSOR_N_LOG("no transaction.");
        } else {
            gp2ap_enable_ps_sensor(client_gp2ap, enable);
            if (data->ps_detection != 0)
            {
                data->ps_detection = 0;
            }
        }
    }

    SENSOR_N_LOG("end");

    mutex_unlock(&data->psals_mutex);
}

void gp2ap_set_als_poll_delay(u32 als_poll_delay)
{
    struct gp2ap_data *data = i2c_get_clientdata(client_gp2ap);

    SENSOR_N_LOG("start");

    mutex_lock(&data->psals_mutex);

    data->als_poll_delay = als_poll_delay;

    data->als_mean_times = ALS_GET_DATA_INTERVAL_MS / data->als_poll_delay;
    data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_STORE_POLL;

    if (data->enable_als_sensor > 0)
    {
        mutex_unlock(&data->psals_mutex);
        cancel_delayed_work_sync(&data->als_on_dwork);
        cancel_delayed_work_sync(&data->als_data_dwork);
        mutex_lock(&data->psals_mutex);
        queue_delayed_work(als_polling_wq, &data->als_on_dwork,
                            msecs_to_jiffies(0));
    }

    mutex_unlock(&data->psals_mutex);

    SENSOR_N_LOG("end");
}

static int gp2ap_init_client(struct i2c_client *client)
{
    int ret = 0;
    int i;

    SENSOR_N_LOG("start");

    ret = gpio_tlmm_config(gpio_config_prox_int_pu[0], GPIO_CFG_ENABLE);
    SENSOR_N_LOG("gpio_tlmm_config(prox_int_pu) called.[%d]",ret);
    if (ret < 0)
    {
        SENSOR_ERR_LOG("gpio_tlmm_config error.[%d]", ret);
        goto exit;
    }

    for (i = 0; i < sizeof(reg_init_data)/sizeof(reg_init_data[0]); i++)
    {
        ret = gp2ap_i2c_write(client, reg_init_data[i].reg,
                              &reg_init_data[i].data, 1);
        if (ret < 0)
        {
            SENSOR_ERR_LOG("gp2ap_i2c_write error.[%d]", ret);
            goto exit;
        }
    }

exit:
    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

static int gp2ap_fin_client(struct i2c_client *client)
{
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = gpio_tlmm_config(gpio_config_prox_int_pd[0], GPIO_CFG_ENABLE);
    SENSOR_N_LOG("gpio_tlmm_config(prox_int_pd) called.[%d]",ret);
    if (ret < 0)
    {
        SENSOR_ERR_LOG("gpio_tlmm_config error. [%d]", ret);
        goto exit;
    }
exit:
    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

static void gp2ap_power_on(void)
{
    struct gp2ap_data *data = i2c_get_clientdata(client_gp2ap);
    u32 enable_als_sensor_backup;

    mutex_lock(&data->psals_mutex);

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("enable_ps_sensor[%d] enable_als_sensor[%d]",
                  data->enable_ps_sensor, data->enable_als_sensor);

    gp2ap_init_client(client_gp2ap);

    enable_als_sensor_backup = data->enable_als_sensor;
    data->enable_als_sensor = 0;

    if (data->enable_ps_sensor > 0)
        gp2ap_enable_ps_sensor(client_gp2ap, SENSOR_ENABLE);

    data->enable_als_sensor = enable_als_sensor_backup;

    if (data->enable_als_sensor > 0)
        gp2ap_enable_als_sensor(client_gp2ap, SENSOR_ENABLE);

    SENSOR_N_LOG("end");
    mutex_unlock(&data->psals_mutex);
}

static void gp2ap_power_off(void)
{
    struct gp2ap_data *data = i2c_get_clientdata(client_gp2ap);
    u32 enable_als_sensor_backup;

    atomic_set(&gp2ap_resetstatus,true);
    mutex_lock(&data->psals_mutex);

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("enable_ps_sensor[%d] enable_als_sensor[%d]",
                  data->enable_ps_sensor, data->enable_als_sensor);

    if (data->enable_als_sensor > 0)
        gp2ap_enable_als_sensor(client_gp2ap, SENSOR_DISABLE);

    enable_als_sensor_backup = data->enable_als_sensor;
    data->enable_als_sensor = 0;

    if (data->enable_ps_sensor > 0)
        gp2ap_enable_ps_sensor(client_gp2ap, SENSOR_DISABLE);

    data->enable_als_sensor = enable_als_sensor_backup;

    gp2ap_fin_client(client_gp2ap);

    SENSOR_N_LOG("end");

    mutex_unlock(&data->psals_mutex);
    atomic_set(&gp2ap_resetstatus,false);
}

static void gp2ap_reschedule_work(struct gp2ap_data *data, unsigned long delay)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("queue_delayed_work(dwork) call[%lu]",delay);

    queue_delayed_work(ps_polling_wq, &data->dwork, msecs_to_jiffies(delay));

    SENSOR_N_LOG("end");
}

static void gp2ap_put_luxValue(struct gp2ap_data *data, u32 luxValue)
{
    u32 cnt = 1;
    u32 mean_temp = 0;
    u32 mean_times = 0;

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("luxValue[%d] als_polling_cnt_reset[%08X]",
                  luxValue, data->als_polling_cnt_reset);
    SENSOR_N_LOG("before als_polling_cnt[%d]",data->als_polling_cnt);

    if (data == NULL)
    {
        SENSOR_N_LOG("end");
        return;
    }

    data->als_lux = luxValue;
    SENSOR_N_LOG("data->als_lux[%d]",data->als_lux);

    if (data->als_polling_cnt_reset)
    {
        data->als_polling_cnt_reset = ALS_POLLING_CNT_RESET_NONE;
        SENSOR_N_LOG("als_polling_cnt_reset[%08X]",data->als_polling_cnt_reset);

        data->als_lux_ave = luxValue;
        data->als_polling_cnt = 0;

        sensor_report_data(SENSOR_LIGHT, data->als_lux_ave);

        SENSOR_N_LOG("ABS_LUX_REPORT[%d]",data->als_lux_ave);
    }
    else
    {
        data->luxValue_table[data->als_polling_cnt] = luxValue;
        data->als_polling_cnt++;

        if (data->als_polling_cnt >= data->als_mean_times)
        {
            for(cnt = 0; cnt < data->als_mean_times; cnt++)
            {
                SENSOR_N_LOG("luxValue_table[%d][%d]",
                              cnt, data->luxValue_table[cnt]);
                mean_temp += data->luxValue_table[cnt] * (cnt + 1);
                mean_times += cnt + 1;
            }
            data->als_lux_ave = mean_temp / mean_times;
            data->als_polling_cnt = 0;

            sensor_report_data(SENSOR_LIGHT, data->als_lux_ave);

            SENSOR_N_LOG("ABS_LUX_REPORT[%d]",data->als_lux_ave);
        }
    }

    SENSOR_N_LOG("after als_polling_cnt[%d]",data->als_polling_cnt);
    SENSOR_N_LOG("end");
}

static void gp2ap_als_on_work_handler(struct work_struct *work)
{
    struct gp2ap_data *data = container_of(work, struct gp2ap_data,
                                           als_on_dwork.work);
    struct i2c_client *client = data->client;
    int ret = 0;
    int als_on_wait_ms = 0;

    SENSOR_N_LOG("start");

    mutex_lock(&data->psals_mutex);

    SENSOR_N_LOG("enable_ps_sensor[%d]",data->enable_ps_sensor);

    ret = gp2ap_set_op_mode(client, GP2AP_SENSOR_ALS, OP_ON);
    if (ret < 0)
    {
        SENSOR_ERR_LOG("gp2ap_set_op_mode error.[%d]", ret);
        goto exit;
    }
    als_on_wait_ms = ALS_ON_DELAY_TIMER_MS;

    SENSOR_N_LOG("queue_delayed_work[%d]",als_on_wait_ms);

    queue_delayed_work(als_polling_wq, &data->als_data_dwork,
                       msecs_to_jiffies(als_on_wait_ms));

exit:
    SENSOR_N_LOG("end");
    mutex_unlock(&data->psals_mutex);
}

static void gp2ap_als_data_work_handler(struct work_struct *work)
{
    struct gp2ap_data *data = container_of(work, struct gp2ap_data,
                                           als_data_dwork.work);
    struct i2c_client *client = data->client;

    u32 irdata;
    u32 cdata;
    int ret = 0;
    int colvar = nv_prox_photo_colvar[0];
    u32 luxValue = 0;
    int als_on_wait_ms = 0;

    SENSOR_N_LOG("start");

    if(atomic_read(&gp2ap_resetstatus) == true){
        SENSOR_ERR_LOG("Sensor Reset Now");
        return;
    }

    mutex_lock(&data->psals_mutex);
    if(atomic_read(&g_update_threshold_flg) == true)
    {
        atomic_set(&g_update_threshold_flg, false);
        gp2ap_set_ps_threshold(client,nv_proximity_sensor_far[colvar], nv_proximity_sensor_near[colvar]);
    }
    SENSOR_N_LOG("enable_ps_sensor[%d]",data->enable_ps_sensor);

    ret = gp2ap_get_als_data(client, &cdata, &irdata);
    if (ret < 0)
    {
        SENSOR_ERR_LOG("gp2ap_get_als_data error. [%d]", ret);
        goto exit;
    }

    data->cdata = cdata;
    data->irdata = irdata;
    luxValue = LuxCalculation(cdata, irdata, data);

    luxValue = luxValue < GP2AP_LUXVALUE_MAX ? luxValue : GP2AP_LUXVALUE_MAX;
    gp2ap_put_luxValue(data, luxValue);

    if (data->enable_ps_sensor <= 0)
    {
        ret = gp2ap_set_op_mode(client, GP2AP_SENSOR_ALS, OP_OFF);
        if (ret < 0)
        {
            SENSOR_ERR_LOG("gp2ap_set_op_mode error. [%d]", ret);
            goto exit;
        }

        als_on_wait_ms = data->als_poll_delay - ALS_ON_WAIT_MS;
        SENSOR_N_LOG("queue_delayed_work(als_on_dwork) call[%d]",als_on_wait_ms);

        queue_delayed_work(als_polling_wq, &data->als_on_dwork,
                           msecs_to_jiffies(als_on_wait_ms));
    }
    else
    {
        als_on_wait_ms = data->als_poll_delay;
        SENSOR_N_LOG("queue_delayed_work(als_data_dwork) call[%d]",als_on_wait_ms);

        queue_delayed_work(als_polling_wq, &data->als_data_dwork,
                           msecs_to_jiffies(als_on_wait_ms));
    }

exit:
    SENSOR_N_LOG("end");
    mutex_unlock(&data->psals_mutex);
}

static void gp2ap_work_handler(struct work_struct *work)
{
    struct gp2ap_data *data = container_of(work, struct gp2ap_data, dwork.work);
    struct i2c_client *client = data->client;
    int ret = 0;
    u32 dev_status_tmp = 0;
    u16 irdata;
    int colvar = nv_prox_photo_colvar[0];
    u8 detection[1] = {0};
    u8 buf[2];

    SENSOR_N_LOG("start");

    mutex_lock(&data->psals_mutex);
    msleep(30);
    data->ratio_reg = RatioCalculation(client, data);
    ret = gp2ap_i2c_read(client, REG_ADR_00, detection, 1);
    if(ret < 0)
    {
        SENSOR_ERR_LOG("gp2ap_i2c_read error. [%d]", ret);
    }

    detection[0] &= PS_DETECTION_MASK;
    if(detection[0])
    {
        if((data->cdata_reg < 0x3E80) || (data->irdata_reg < 0x3E80))
        {
            if(data->ratio_reg < 80)
            {
                SENSOR_N_LOG("detection");
                buf[0] = 0x23;
                ret = gp2ap_i2c_write(client, REG_ADR_01, buf, 1);
                if(ret < 0)
                {
                    SENSOR_ERR_LOG("gp2ap_i2c_write error. [%d]", ret);
                }
            }
            else
            {
                msleep(30);
                ret = gp2ap_i2c_read(client, REG_ADR_0E, buf, 2);
                if(ret < 0)
                {
                    SENSOR_ERR_LOG("gp2ap_i2c_read error. [%d]", ret);
                }

                irdata = (buf[1] << 8) | buf[0];
                SENSOR_N_LOG("irdata[%04X]", irdata);

                if(irdata >= 0x44C)
                {
                    SENSOR_N_LOG("no detection");
                    buf[0] = 0xC0;
                    ret = gp2ap_i2c_write(client, REG_ADR_00, buf, 1);
                    if(ret < 0)
                    {
                        SENSOR_ERR_LOG("gp2ap_i2c_write error. [%d]", ret);
                    }
                    detection[0] = 0x00;
                }
                else
                {
                    SENSOR_N_LOG("detection");
                    buf[0] = 0x23;
                    ret = gp2ap_i2c_write(client, REG_ADR_01, buf, 1);
                    if(ret < 0)
                    {
                        SENSOR_ERR_LOG("gp2ap_i2c_write error. [%d]", ret);
                    }
                }
            }
        }
        else
        {
            buf[0] = 0xC0;
            ret = gp2ap_i2c_write(client, REG_ADR_00, buf, 1);
            if(ret < 0)
            {
                SENSOR_ERR_LOG("gp2ap_i2c_write error. [%d]", ret);
            }
            detection[0] = 0x00;
        }
    }
    else
    {
        SENSOR_N_LOG("no detection");
        buf[0] = 0x63;
        ret = gp2ap_i2c_write(client, REG_ADR_01, buf, 1);
        if(ret < 0)
        {
            SENSOR_ERR_LOG("gp2ap_i2c_write error. [%d]", ret);
        }
    }

    if(detection[0])
    {
        data->ps_detection = 1;
    }
    else
    {
        data->ps_detection = 0;
        wake_lock_timeout(&gp2ap_wake_lock_input, GP2AP_WAKE_LOCK_INPUT_TIME);
    }

    sensor_interrupt(SENSOR_PROX, data->ps_detection);

    dev_status_tmp = atomic_read(&g_dev_status);

    SENSOR_N_LOG("dev_status_tmp[%08X]",dev_status_tmp);
    SENSOR_N_LOG("enable_irq[%d]",data->ps_irq);

    enable_irq(data->ps_irq);

    if(atomic_read(&g_update_threshold_flg) == true)
    {
        atomic_set(&g_update_threshold_flg, false);
        gp2ap_set_ps_threshold(client,nv_proximity_sensor_far[colvar], nv_proximity_sensor_near[colvar]);
    }
    dev_status_tmp &= ~(GP2AP_DEV_STATUS_RESUME | GP2AP_DEV_STATUS_SUSPEND_INT);
    SENSOR_N_LOG("dev_status_tmp[%08X]",dev_status_tmp);
    atomic_set(&g_dev_status, dev_status_tmp );

    SENSOR_N_LOG("end");
    mutex_unlock(&data->psals_mutex);
}

static irqreturn_t gp2ap_interrupt(int vec, void *info)
{
    struct i2c_client *client=(struct i2c_client *)info;
    struct gp2ap_data *data = i2c_get_clientdata(client);
    u32 dev_status_tmp = 0;

    SENSOR_N_LOG("start");

    disable_irq_nosync(data->ps_irq);
    dev_status_tmp = atomic_read(&g_dev_status);

    SENSOR_N_LOG("dev_status_tmp[%08X]", dev_status_tmp);
    if (dev_status_tmp & GP2AP_DEV_STATUS_SUSPEND)
    {
        SENSOR_N_LOG("set_status[%d]", GP2AP_DEV_STATUS_SUSPEND_INT);
        atomic_set(&g_dev_status, dev_status_tmp | GP2AP_DEV_STATUS_SUSPEND_INT);
        wake_lock_timeout(&gp2ap_wake_lock, GP2AP_WAKE_LOCK_TIME);
    }
    else
    {
        gp2ap_reschedule_work(data, 0);
    }

    SENSOR_N_LOG("end");

	return IRQ_HANDLED;
}

static void gp2ap_set_als_mean_times(struct gp2ap_data *data, u32 mean_times)
{
    SENSOR_N_LOG("start");

    SENSOR_N_LOG("mean_times[%lu]", (unsigned long int)mean_times);
    if ((mean_times == 0) ||
        ((ALS_GET_DATA_INTERVAL_MS / mean_times) < ALS_ON_WAIT_MS))
    {
        SENSOR_ERR_LOG("bad param. mean_times[%lu]", (unsigned long int)mean_times);
        goto exit;
    }

    if (mean_times > GP2AP_LUXVALUE_TABLE_MAX)
    {
        data->als_mean_times = GP2AP_LUXVALUE_TABLE_MAX;
    }
    else
    {
        data->als_mean_times = mean_times;
    }
    data->als_poll_delay = ALS_GET_DATA_INTERVAL_MS / data->als_mean_times;
    data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_STORE_TIME;

    SENSOR_N_LOG("als_polling_cnt_reset[%08X]", data->als_polling_cnt_reset);

    if (data->enable_als_sensor > 0)
    {
        mutex_unlock(&data->psals_mutex);
        cancel_delayed_work_sync(&data->als_on_dwork);
        cancel_delayed_work_sync(&data->als_data_dwork);
        mutex_lock(&data->psals_mutex);

        SENSOR_N_LOG("queue_delayed_work(als_on_dwork) call.");

        queue_delayed_work(als_polling_wq, &data->als_on_dwork,
                           msecs_to_jiffies(0));
    }

exit:
    SENSOR_N_LOG("als_poll_delay[%d] als_mean_times[%d]",
                  data->als_poll_delay, data->als_mean_times);
    SENSOR_N_LOG("end");
}

void gp2ap_set_ioctl_als_mean_times(u32 mean_times)
{
    struct gp2ap_data *data = i2c_get_clientdata(client_gp2ap);

    SENSOR_N_LOG("start");

    mutex_lock(&data->psals_mutex);

    gp2ap_set_als_mean_times(data, mean_times);

    mutex_unlock(&data->psals_mutex);

    SENSOR_N_LOG("end");
}

u32 gp2ap_get_initialize_state(void)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end gp2ap_initialize:%d",gp2ap_initialize);

    return gp2ap_initialize;
}

void gp2ap_set_ioctl_sensor_nv(unsigned long ulArg)
{
    T_PSALS_IOCTL_NV* nv_data_type 
                        = (T_PSALS_IOCTL_NV*)ulArg;
    int i;

    SENSOR_N_LOG("start");

    switch (nv_data_type->ulItem)
    {
    case en_NV_PROXIMITY_SENSOR_NEAR_I:
        SENSOR_N_LOG("en_NV_PROXIMITY_SENSOR_NEAR_I");
        nv_status |= (0x01<<en_NV_PROXIMITY_SENSOR_NEAR_I);
        memcpy(nv_proximity_sensor_near,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_proximity_sensor_near)/
             sizeof(nv_proximity_sensor_near[0]); i++)
        {
            SENSOR_N_LOG("nv_proximity_sensor_near[%d][%04X]",
                           i, nv_proximity_sensor_near[i]);
        }
        break;

    case en_NV_PROXIMITY_SENSOR_FAR_I:
        SENSOR_N_LOG("en_NV_PROXIMITY_SENSOR_FAR_I");
        nv_status |= (0x01<<en_NV_PROXIMITY_SENSOR_FAR_I);
        memcpy(nv_proximity_sensor_far,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_proximity_sensor_far)/
             sizeof(nv_proximity_sensor_far[0]); i++)
        {
            SENSOR_N_LOG("nv_proximity_sensor_far[%d][%04X]",
                           i, nv_proximity_sensor_far[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_BEAMISH_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_BEAMISH_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_BEAMISH_I);
        memcpy(nv_photo_sensor_beamish,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_beamish)/
             sizeof(nv_photo_sensor_beamish[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_beamish[%d][%04X]",
                           i, nv_photo_sensor_beamish[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_A_035_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_A_035_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_A_035_I);
        memcpy(nv_photo_sensor_a_035,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_a_035)/
             sizeof(nv_photo_sensor_a_035[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_a_035[%d][%04X]",
                           i, nv_photo_sensor_a_035[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_A_067_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_A_067_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_A_067_I);
        memcpy(nv_photo_sensor_a_067,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_a_067)/
             sizeof(nv_photo_sensor_a_067[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_a_067[%d][%04X]",
                           i, nv_photo_sensor_a_067[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_A_093_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_A_093_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_A_093_I);
        memcpy(nv_photo_sensor_a_093,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_a_093)/
             sizeof(nv_photo_sensor_a_093[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_a_093[%d][%04X]",
                          i, nv_photo_sensor_a_093[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_A_MAX_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_A_MAX_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_A_MAX_I);
        memcpy(nv_photo_sensor_a_max,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_a_max)/
             sizeof(nv_photo_sensor_a_max[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_a_max[%d][%04X]",
                           i, nv_photo_sensor_a_max[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_B_035_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_B_035_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B_035_I);
        memcpy(nv_photo_sensor_b_035,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_b_035)/
             sizeof(nv_photo_sensor_b_035[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_b_035[%d][%04X]",
                           i, nv_photo_sensor_b_035[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_B_067_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_B_067_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B_067_I);
        memcpy(nv_photo_sensor_b_067,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_b_067)/
             sizeof(nv_photo_sensor_b_067[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_b_067[%d][%04X]",
                           i, nv_photo_sensor_b_067[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_B_093_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_B_093_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B_093_I);
        memcpy(nv_photo_sensor_b_093,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_b_093)/
             sizeof(nv_photo_sensor_b_093[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_b_093[%d][%04X]",
                           i, nv_photo_sensor_b_093[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_B_MAX_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_B_MAX_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B_MAX_I);
        memcpy(nv_photo_sensor_b_max,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_b_max)/
             sizeof(nv_photo_sensor_b_max[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_b_max[%d][%04X]",
                           i, nv_photo_sensor_b_max[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_A25MS_035_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_A25MS_035_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_A25MS_035_I);
        memcpy(nv_photo_sensor_a25ms_035,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_a25ms_035)/
             sizeof(nv_photo_sensor_a25ms_035[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_a25ms_035[%d][%04X]",
                           i, nv_photo_sensor_a25ms_035[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_A25MS_067_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_A25MS_067_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_A25MS_067_I);
        memcpy(nv_photo_sensor_a25ms_067,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_a25ms_067)/
             sizeof(nv_photo_sensor_a25ms_067[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_a25ms_067[%d][%04X]",
                           i, nv_photo_sensor_a25ms_067[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_A25MS_093_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_A25MS_093_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_A25MS_093_I);
        memcpy(nv_photo_sensor_a25ms_093,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_a25ms_093)/
             sizeof(nv_photo_sensor_a25ms_093[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_a25ms_093[%d][%04X]",
                           i, nv_photo_sensor_a25ms_093[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_A25MS_MAX_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_A25MS_MAX_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_A25MS_MAX_I);
        memcpy(nv_photo_sensor_a25ms_max,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_a25ms_max)/
             sizeof(nv_photo_sensor_a25ms_max[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_a25ms_max[%d][%04X]",
                           i, nv_photo_sensor_a25ms_max[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_B25MS_035_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_B25MS_035_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B25MS_035_I);
        memcpy(nv_photo_sensor_b25ms_035,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_b25ms_035)/
             sizeof(nv_photo_sensor_b25ms_035[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_b25ms_035[%d][%04X]",
                           i, nv_photo_sensor_b25ms_035[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_B25MS_067_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_B25MS_067_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B25MS_067_I);
        memcpy(nv_photo_sensor_b25ms_067,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_b25ms_067)/
             sizeof(nv_photo_sensor_b25ms_067[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_b25ms_067[%d][%04X]",
                           i, nv_photo_sensor_b25ms_067[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_B25MS_093_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_B25MS_093_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B25MS_093_I);
        memcpy(nv_photo_sensor_b25ms_093,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_b25ms_093)/
             sizeof(nv_photo_sensor_b25ms_093[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_b25ms_093[%d][%04X]",
                           i, nv_photo_sensor_b25ms_093[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_B25MS_MAX_I:
        SENSOR_N_LOG("en_NV_PHOTO_SENSOR_B25MS_MAX_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B25MS_MAX_I);
        memcpy(nv_photo_sensor_b25ms_max,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_photo_sensor_b25ms_max)/
             sizeof(nv_photo_sensor_b25ms_max[0]); i++)
        {
            SENSOR_N_LOG("nv_photo_sensor_b25ms_max[%d][%04X]",
                           i, nv_photo_sensor_b25ms_max[i]);
        }
        break;

    case en_NV_PROX_PHOTO_COLVAR_I:
        SENSOR_N_LOG("en_NV_PROX_PHOTO_COLVAR_I");
        nv_status |= (0x01<<en_NV_PROX_PHOTO_COLVAR_I);
        memcpy(nv_prox_photo_colvar,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        if (nv_prox_photo_colvar[0] >= GP2AP_COLOR_VARI)
        {
            nv_prox_photo_colvar[0] = 0;
        }
        for (i = 0; i < sizeof(nv_prox_photo_colvar)/
             sizeof(nv_prox_photo_colvar[0]); i++)
        {
            SENSOR_N_LOG("nv_prox_photo_colvar[%d][%04X]",
                           i, nv_prox_photo_colvar[i]);
        }
        break;

    case en_NV_PROXIMITY_SENSOR_TEMP_I:
        SENSOR_N_LOG("en_NV_PROXIMITY_SENSOR_TEMP_I");
        nv_status |= (0x01<<en_NV_PROXIMITY_SENSOR_TEMP_I);
        memcpy(nv_proximity_sensor_temp,
               nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        for (i = 0; i < sizeof(nv_proximity_sensor_temp)/
             sizeof(nv_proximity_sensor_temp[0]); i++)
        {
            SENSOR_N_LOG("nv_proximity_sensor_temp[%d][%04X]",
                           i,nv_proximity_sensor_temp[i]);
        }
        break;

    default :
        SENSOR_ERR_LOG("set_sensor_nv: Can't set nv data");
        break;
    }

    SENSOR_N_LOG("end");
}

void gp2ap_get_ioctl_lux_ave(u32 *als_lux_ave, s32 *cdata, s32 *irdata)
{
    struct gp2ap_data *data = i2c_get_clientdata(client_gp2ap);

    SENSOR_N_LOG("start");

    *als_lux_ave = data->als_lux_ave;
    *cdata = data->cdata;
    *irdata = data->irdata;

    SENSOR_N_LOG("end");
}

static const struct i2c_device_id gp2ap_id[] = {
    {DEVICE_NAME, 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, gp2ap_id);

static struct i2c_driver gp2ap_driver = {
    .driver = {
        .name  = DEVICE_NAME,
        .owner = THIS_MODULE,
    },
    .suspend     = gp2ap_suspend,
    .resume      = gp2ap_resume,
    .probe       = gp2ap_probe,
    .shutdown    = gp2ap020a_shutdown,
    .remove = __devexit_p(gp2ap_remove),
    .id_table = gp2ap_id,
};

static int gp2ap_resume(struct i2c_client *client)
{
    struct gp2ap_data *data = i2c_get_clientdata(client);
    u32 dev_status_tmp = 0;
    uint32_t ps_status = sensor_get_status(SENSOR_PROX);

    mutex_lock(&data->psals_mutex);
    SENSOR_N_LOG("start");

    SENSOR_N_LOG("data->enable_ps_sensor[%d]",data->enable_ps_sensor);

    if (device_may_wakeup(&client->dev))
    {
        if( SENSOR_ON == ps_status ){
            SENSOR_N_LOG("disable_irq_wake !!");
            disable_irq_wake(data->ps_irq);
        }
    }
    else
    {
        SENSOR_ERR_LOG("failed device_may_wakeup");
    }

    data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_RESUME;

    SENSOR_N_LOG("als_polling_cnt_reset[%08X]",data->als_polling_cnt_reset);

    if (data->enable_ps_sensor > 0)
    {
        dev_status_tmp = (atomic_read(&g_dev_status) & 
                                  GP2AP_DEV_STATUS_SUSPEND_INT);
        atomic_set(&g_dev_status, GP2AP_DEV_STATUS_INIT | 
                                  GP2AP_DEV_STATUS_RESUME | dev_status_tmp);
        dev_status_tmp = atomic_read(&g_dev_status);

        SENSOR_N_LOG("dev_status_tmp[%08X]",dev_status_tmp);

        if( atomic_read(&g_dev_status) & GP2AP_DEV_STATUS_SUSPEND_INT ) {
            gp2ap_reschedule_work(data, 0);
        }
    }
    else
    {
        atomic_set(&g_dev_status, GP2AP_DEV_STATUS_INIT);	
    }

    if (data->enable_als_sensor > 0)
    {
        mutex_unlock(&data->psals_mutex);
        cancel_delayed_work_sync(&data->als_on_dwork);
        cancel_delayed_work_sync(&data->als_data_dwork);
        mutex_lock(&data->psals_mutex);

        SENSOR_N_LOG("queue_delayed_work()call.[%d]",0);

        queue_delayed_work(als_polling_wq, &data->als_on_dwork,
                                           msecs_to_jiffies(0));
    }

    wake_unlock(&gp2ap_wake_lock);

    SENSOR_N_LOG("end");

    mutex_unlock(&data->psals_mutex);
    return 0;
}

static int gp2ap_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct gp2ap_data *data = i2c_get_clientdata(client);
    uint32_t ps_status = sensor_get_status(SENSOR_PROX);

    mutex_lock(&data->psals_mutex);
    SENSOR_N_LOG("start");

    if (device_may_wakeup(&client->dev))
    {
        if( SENSOR_ON == ps_status ){
            SENSOR_N_LOG("enable_irq_wake !!");
            enable_irq_wake(data->ps_irq);
        }
    }
    else
    {
        SENSOR_N_LOG("failed device_may_wakeup");
    }

    mutex_unlock(&data->psals_mutex);
    cancel_delayed_work_sync(&data->als_on_dwork);
    cancel_delayed_work_sync(&data->als_data_dwork);
    mutex_lock(&data->psals_mutex);
    SENSOR_N_LOG("cancel_delayed_work(als_on_dwork) call.");
    SENSOR_N_LOG("cancel_delayed_work(als_data_dwork) call.");
    atomic_set(&g_dev_status, GP2AP_DEV_STATUS_SUSPEND);

    SENSOR_N_LOG("end");
    mutex_unlock(&data->psals_mutex);

    return 0;
}

static void gp2ap020a_shutdown(struct i2c_client *client)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return;
}

static int __devexit gp2ap_remove(struct i2c_client *client)
{
    struct gp2ap_data *data = i2c_get_clientdata(client);

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("data->vpro_vreg[%08X]",(unsigned int)data->vpro_vreg);

    mutex_lock(&data->psals_mutex);

    wake_lock_destroy(&gp2ap_wake_lock);
    wake_lock_destroy(&gp2ap_wake_lock_input);

    free_irq(data->ps_irq, client);

    if (data->vpro_vreg)
    {
        regulator_put(data->vpro_vreg);
        data->vpro_vreg = NULL;
    }

    data->enable_ps_sensor = 0;
    data->enable_als_sensor = 0;

    gp2ap_enable_als_sensor(client, SENSOR_DISABLE);
    gp2ap_enable_ps_sensor(client, SENSOR_DISABLE);

    mutex_unlock(&data->psals_mutex);
    mutex_destroy(&data->psals_mutex);

    kfree(data);

    SENSOR_N_LOG("end");
    return 0;
}

static int __devinit gp2ap_probe(
    struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct gp2ap_data *data;
    int err = 0;
    struct gp2ap020_platform_data  *pdata;

    SENSOR_N_LOG("start");

    pdata = client->dev.platform_data;
    data = kzalloc(sizeof(struct gp2ap_data), GFP_KERNEL);
    if (!data)
    {
        err = -ENOMEM;
        goto exit;
    }
    data->client = client;

    i2c_set_clientdata(client, data);

    gp2ap_initialize = 0;
    data->enable_als_sensor = 0;
    data->enable_ps_sensor = 0;
    data->ps_detection = 0;
    data->ps_data = 0;
    data->als_poll_delay = ALS_POLL_DELAY_MS_DEF;
    data->cdata = 0;
    data->irdata = 0;
    data->als_lux = 0;
    memset(data->luxValue_table, 0, sizeof(data->luxValue_table));
    data->als_lux_ave = 0;
    data->als_polling_cnt = 0;
    data->als_mean_times = ALS_MEAN_TIME_DEF;
    data->als_polling_cnt_reset = ALS_POLLING_CNT_RESET_INIT;

    SENSOR_N_LOG("als_polling_cnt_reset[%08X]",data->als_polling_cnt_reset);

    data->ps_en_gpio = GPIO_PROX_EN;
    data->vpro_vreg = NULL;
    data->op_mode = GP2AP_OP_MODE_NONE;
    data->op_sensor = GP2AP_SENSOR_NONE;
    data->ratio_reg = 0;
    data->cdata_reg = 0;
    data->irdata_reg = 0;
    gp2ap_power_cb.power_on = gp2ap_power_on;
    gp2ap_power_cb.power_off = gp2ap_power_off;
    atomic_set(&gp2ap_resetstatus,false);
    atomic_set(&g_update_threshold_flg,false);

    mutex_init(&data->psals_mutex);
    wake_lock_init(&gp2ap_wake_lock, WAKE_LOCK_SUSPEND, "gp2ap_ps");
    wake_lock_init(&gp2ap_wake_lock_input, WAKE_LOCK_SUSPEND, "gp2ap_ps_input");

    err = gpio_request(GPIO_PROX_INT, DEVICE_NAME);

    SENSOR_N_LOG("gpio_request[%d] err[%d]",GPIO_PROX_INT, err);

    if (err < 0)
    {
        SENSOR_ERR_LOG("failed to request GPIO[%d] ret[%d]",GPIO_PROX_INT,err);
        goto exit_kfree;
    }

    if (data->ps_en_gpio >= 0)
    {
        err = gpio_request(data->ps_en_gpio, DEVICE_NAME);

        SENSOR_N_LOG("gpio_request[%d] err[%d]",data->ps_en_gpio, err);

        if (err < 0)
        {
            SENSOR_ERR_LOG("failed to request GPIO[%d] ret[%d]",data->ps_en_gpio,err);
            goto exit_gpio_free1;
        }
    }
    else
    {
        data->vpro_vreg = regulator_get(NULL, VPRO_VREG_NAME);
        if (IS_ERR(data->vpro_vreg))
        {
            SENSOR_ERR_LOG("regulator_get[%s] error[%ld]",
                            VPRO_VREG_NAME, IS_ERR(data->vpro_vreg));
            data->vpro_vreg = NULL;
            goto exit_gpio_free1;
        }
    }

    data->ps_irq = gpio_to_irq(GPIO_PROX_INT);
    err = request_any_context_irq(data->ps_irq, gp2ap_interrupt,
            IRQ_TYPE_EDGE_FALLING, DEVICE_NAME, (void *)client);

    SENSOR_N_LOG("request_any_context_irq() called. err[%d]", err);

    if (err < 0)
    {
        SENSOR_ERR_LOG("Could not allocate GPIO_PROX_INT[%d] err[%d]",GPIO_PROX_INT,err);

        goto exit_gpio_free2;
    }
    gp2ap_ps_irq_cnt++;
    gp2ap_disable_ps_irq(data);

    INIT_DELAYED_WORK(&data->dwork, gp2ap_work_handler);
    INIT_DELAYED_WORK(&data->als_on_dwork, gp2ap_als_on_work_handler);
    INIT_DELAYED_WORK(&data->als_data_dwork, gp2ap_als_data_work_handler);

    err = gp2ap_init_client(client);
    if (err)
    {
        SENSOR_ERR_LOG("Failed gp2ap_init_client");
        goto exit_gpio_free2;
    }

    gp2ap_vsensor_onoff(VSENSOR_ON);

    device_init_wakeup(&client->dev, 1);
    atomic_set(&g_dev_status, GP2AP_DEV_STATUS_INIT);

    client_gp2ap = client;

    gp2ap_initialize = 1;

    SENSOR_N_LOG("end");
    return 0;

exit_gpio_free2:
    if (data->ps_en_gpio >= 0)
        gpio_free(data->ps_en_gpio);
    else
        regulator_put(data->vpro_vreg);
exit_gpio_free1:
    gpio_free(GPIO_PROX_INT);
exit_kfree:
    kfree(data);
exit:
    SENSOR_N_LOG("[OUT] err[%d]", err);
    return err;
}

void gp2ap_init(void)
{
    int ret = 0;

    SENSOR_N_LOG("start");

    als_polling_wq = create_singlethread_workqueue("als_polling_wq");
    if (!als_polling_wq)
    {
        SENSOR_ERR_LOG("can't create queue : als_polling_wq");
        return;
    }

    ps_polling_wq = create_singlethread_workqueue("ps_polling_wq");
    if (!ps_polling_wq)
    {
        SENSOR_ERR_LOG("can't create queue : ps_polling_wq");
        goto REGIST_ERR1;
    }

    ret = i2c_add_driver(&gp2ap_driver);
    if(ret != 0){
        SENSOR_ERR_LOG("fail:spi_register_driver()->ret[%d]",ret);
        goto REGIST_ERR2;
    }

    SENSOR_N_LOG("end");
    return;

REGIST_ERR2:
    if (ps_polling_wq != NULL)
    {
        flush_workqueue(ps_polling_wq);
        destroy_workqueue(ps_polling_wq);
        ps_polling_wq = NULL;
    }

REGIST_ERR1:
    if (als_polling_wq != NULL)
    {
        flush_workqueue(als_polling_wq);
        destroy_workqueue(als_polling_wq);
        als_polling_wq = NULL;
    }
}

void gp2ap_exit(void)
{
    SENSOR_N_LOG("start");

    if (als_polling_wq != NULL)
    {
        SENSOR_N_LOG("als polling wq delete");
        flush_workqueue(als_polling_wq);
        destroy_workqueue(als_polling_wq);
        als_polling_wq = NULL;
    }

    if (ps_polling_wq != NULL)
    {
        SENSOR_N_LOG("ps polling wq delete");
        flush_workqueue(ps_polling_wq);
        destroy_workqueue(ps_polling_wq);
        ps_polling_wq = NULL;
    }

    i2c_del_driver(&gp2ap_driver);

    i2c_unregister_device(client_gp2ap);
    client_gp2ap = NULL;

    SENSOR_N_LOG("end");
}


