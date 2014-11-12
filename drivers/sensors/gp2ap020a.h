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
#ifndef GP2AP020A_H
#define GP2AP020A_H

#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#define GP2AP_LUXVALUE_MAX                  10000
#define GP2AP_LUXVALUE_TABLE_MAX            50

#define D_NV_DATA_MAX                           (0x55)

enum {
    en_NV_PROXIMITY_SENSOR_NEAR_I = 0,
    en_NV_PROXIMITY_SENSOR_FAR_I,
    en_NV_PHOTO_SENSOR_BEAMISH_I,
    en_NV_PROX_PHOTO_COLVAR_I,
    en_NV_PHOTO_SENSOR_A_035_I,
    en_NV_PHOTO_SENSOR_A_067_I,
    en_NV_PHOTO_SENSOR_A_093_I,
    en_NV_PHOTO_SENSOR_A_MAX_I,
    en_NV_PHOTO_SENSOR_B_035_I,
    en_NV_PHOTO_SENSOR_B_067_I,
    en_NV_PHOTO_SENSOR_B_093_I,
    en_NV_PHOTO_SENSOR_B_MAX_I,
    en_NV_PHOTO_SENSOR_A25MS_035_I,
    en_NV_PHOTO_SENSOR_A25MS_067_I,
    en_NV_PHOTO_SENSOR_A25MS_093_I,
    en_NV_PHOTO_SENSOR_A25MS_MAX_I,
    en_NV_PHOTO_SENSOR_B25MS_035_I,
    en_NV_PHOTO_SENSOR_B25MS_067_I,
    en_NV_PHOTO_SENSOR_B25MS_093_I,
    en_NV_PHOTO_SENSOR_B25MS_MAX_I,
    en_NV_PROXIMITY_SENSOR_TEMP_I
};

enum sensor_enable
{
    SENSOR_DISABLE = 0,
    SENSOR_ENABLE,
};

struct gp2ap_data
{
    struct i2c_client *client;
    struct mutex psals_mutex;
    struct delayed_work dwork;
    struct delayed_work als_on_dwork;
    struct delayed_work als_data_dwork;
    struct input_dev *input_dev_als;
    struct input_dev *input_dev_ps;
    int enable_ps_sensor;
    int enable_als_sensor;
    u32 ps_detection;
    u32 ps_data;
    u32 als_poll_delay;
    s32 ps_irq;
    s32 cdata;
    s32 irdata;
    u32 als_lux;
    u32 luxValue_table[GP2AP_LUXVALUE_TABLE_MAX];
    u32 als_lux_ave;
    u32 als_polling_cnt;
    u32 als_mean_times;
    u32 als_polling_cnt_reset;
    int ps_en_gpio;
    struct regulator *vpro_vreg;
    u32 op_mode;
    u32 op_sensor;
    u32 ratio_reg;
    s32 cdata_reg;
    s32 irdata_reg;
};

struct reg_data
{
    u8 reg;
    u8 data;
};

struct gp2ap020_platform_data
{
    int    gpio;
    int    ps_en_gpio;
};



typedef struct _t_psals_ioctl_nv
{
    unsigned long ulLength;
    unsigned char ucData[D_NV_DATA_MAX];
    unsigned long ulItem;
}T_PSALS_IOCTL_NV;


int gp2ap_set_ioctl_ps_threshold(void);
void gp2ap_set_ioctl_als_mean_times(u32 mean_times);
void gp2ap_set_ioctl_sensor_nv(unsigned long ulArg);
void gp2ap_get_ioctl_lux_ave(u32 *als_lux_ave, s32 *cdata, s32 *irdata);
void gp2ap_ps_sensor_activate(enum sensor_enable enable);
void gp2ap_als_sensor_activate(enum sensor_enable enable);
void gp2ap_set_als_poll_delay(u32 als_poll_delay);
u32 gp2ap_get_initialize_state(void);

void gp2ap_init(void);
void gp2ap_exit(void);

#endif /* GP2AP020A_H */
