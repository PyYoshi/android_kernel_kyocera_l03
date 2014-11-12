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

#include <linux/module.h>
#include <linux/kernel.h>

#include "sensor_driver.h"
#include "sensor_com.h"

#define SNESOR_COM_FW_VER_CHK_OK    (0x00)
#define SNESOR_COM_FW_VER_CHK_NG    (0xFF)
#define SENSOR_NOT_MICON_TYPE ( (1<<SENSOR_LIGHT)\
                                |(1<<SENSOR_PROX) )

#define SENSOR_NOT_OFF_SUSPEND_TYPE ( (1<<SENSOR_PROX)\
                                      | (1<<SENSOR_EXT_PEDO)\
                                      | (1<<SENSOR_EXT_VEHI)\
                                      | (1<<SENSOR_EXT_IWIFI)\
                                      | (1<<SENSOR_SGNFCNT_MTN)\
                                      | (1<<SENSOR_STEP_CNT)\
                                      | (1<<SENSOR_STEP_DTC)\
                                      | (1<<SENSOR_KC_MOTION_WALK_START)\
                                      | (1<<SENSOR_KC_MOTION_WALK_STOP)\
                                      | (1<<SENSOR_KC_MOTION_TRAIN)\
                                      | (1<<SENSOR_KC_MOTION_VEHICLE)\
                                      | (1<<SENSOR_KC_MOTION_BRINGUP) )

#define SENSOR_NOT_SUPPORT_TYPE ( (1<<SENSOR_RH)\
                                  | (1<<SENSOR_TEMP)\
                                  | (1<<SENSOR_TEMP_AMBIENT)\
                                  | (1<<SENSOR_PRESSURE)  )

#define MICON_FIFO_SIZE 3072

static ssize_t sensor_com_info_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_com_fw_update_sq_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count
);
static ssize_t sensor_com_fw_update_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t sensor_com_test0_ctl_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t sensor_com_fw_version_chk_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_com_fw_version_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static void sensor_com_set_input_params( struct input_dev *dev );
static uint32_t sensor_get_status_local(enum sensor_e_type type);
static void sensor_broadcast_batch( enum sensor_batch_report_e_type repo_type );
static uint32_t sensor_sort_batch_data(uint8_t* buffer_p);
static void sensor_drv_set_status_local( enum sensor_drv_status_e_type next_status );
static enum sensor_drv_status_e_type sensor_drv_get_status_local(void);
static void sensor_set_batch_status(
    enum sensor_e_type type,
    enum sensor_batch_status_e_type on);
static int32_t sensor_type_enable(enum sensor_e_type type,bool enable);
static int32_t sensor_application_enable(enum sensor_e_type type,bool enable);
static void sensor_suspend_ctl( void );
static void sensor_resume_ctl( void );
static void sensor_shutdown_ctl( void );
static void sensor_reset_ctl( void );
static enum sensor_micon_polling_e_type sensor_get_poll_type(
    enum sensor_e_type type);
static enum sensor_chk_status_e_type sensor_check_status(
    enum sensor_e_type type,
    bool enable );
static int32_t sensor_poll_on_ctrl( enum sensor_e_type type );
static int32_t sensor_poll_off_ctrl( enum sensor_e_type type );

static DEVICE_ATTR(fw_update_sq,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    NULL,
    sensor_com_fw_update_sq_store
);
static DEVICE_ATTR(test0_ctl,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    NULL,
    sensor_com_test0_ctl_store
);
static DEVICE_ATTR(fw_version_chk,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_com_fw_version_chk_show,
    NULL
);
static DEVICE_ATTR(fw_version,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_com_fw_version_show,
    NULL
);
static DEVICE_ATTR(fw_update,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    NULL,
    sensor_com_fw_update_store
);
static DEVICE_ATTR(info,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_com_info_show,
    NULL
);

static struct attribute *sensor_com_attributes[] = {
    &dev_attr_fw_update_sq.attr,
    &dev_attr_test0_ctl.attr,
    &dev_attr_fw_version_chk.attr,
    &dev_attr_fw_version.attr,
    &dev_attr_fw_update.attr,
    &dev_attr_info.attr,
    NULL
};

static struct attribute_group sensor_com_attr_grp = {
    .attrs = sensor_com_attributes
};


struct sensor_input_info_str sensor_com_input_info =
{
    NULL,
    sensor_com_set_input_params,
    &sensor_com_attr_grp,
};

static struct sensor_batch_data_info_str sensor_micon_batch_info_tbl[] = {
    { SENSOR_BATCH_ID_ACC, SENSOR_BATCH_SIZE_ACC, 0, {0,0,0} },
    { SENSOR_BATCH_ID_MAG_UNCAL, SENSOR_BATCH_SIZE_MAG_UNCAL, 0, {0,0,0} },
    { SENSOR_BATCH_ID_GYRO_UNCAL, SENSOR_BATCH_SIZE_GYRO_UNCAL, 0, {0,0,0} },
    { SENSOR_BATCH_ID_MAG, SENSOR_BATCH_SIZE_MAG, 0, {0,0,0} },
    { SENSOR_BATCH_ID_GYRO, SENSOR_BATCH_SIZE_GYRO, 0, {0,0,0} },
    { SENSOR_BATCH_ID_ORTN, SENSOR_BATCH_SIZE_ORTN, 0, {0,0,0} },
    { SENSOR_BATCH_ID_GRV, SENSOR_BATCH_SIZE_GRV, 0, {0,0,0} },
    { SENSOR_BATCH_ID_ACC_LNR, SENSOR_BATCH_SIZE_ACC_LNR, 0, {0,0,0} },
    { SENSOR_BATCH_ID_ROT_VCTR, SENSOR_BATCH_SIZE_ROT_VCTR, 0, {0,0,0} },
    { SENSOR_BATCH_ID_GAME_ROT_VCTR, SENSOR_BATCH_SIZE_GAME_ROT_VCTR, 0, {0,0,0} },
    { SENSOR_BATCH_ID_MAG_ROT_VCTR, SENSOR_BATCH_SIZE_MAG_ROT_VCTR, 0, {0,0,0} },
    { SENSOR_BATCH_ID_ABS_STEP, SENSOR_BATCH_SIZE_ABS_STEP, 0, {0,0,0} },
    { SENSOR_BATCH_ID_REL_STEP_2, SENSOR_BATCH_SIZE_REL_STEP_2, 0, {0,0,0} },
    { SENSOR_BATCH_ID_REL_STEP_1, SENSOR_BATCH_SIZE_REL_STEP_1, 0, {0,0,0} },
};
static uint32_t sensor_micon_batch_info_tbl_size = sizeof(sensor_micon_batch_info_tbl)
                                                   / sizeof(struct sensor_batch_data_info_str);

static struct sensor_poll_ctl_str sensor_poll_ctl_tbl[] = {
    { SENSOR_ACC, NULL },
    { SENSOR_ACC_LNR, NULL },
    { SENSOR_GAME_ROT_VCTR, NULL },
    { SENSOR_GRV, NULL },
    { SENSOR_GYRO, NULL },
    { SENSOR_GYRO_UNCAL, NULL },
    { SENSOR_MAG, NULL },
    { SENSOR_MAG_ROT_VCTR, NULL },
    { SENSOR_MAG_UNCAL, NULL },
    { SENSOR_ORTN, NULL },
    { SENSOR_ROT_VCTR, NULL },
    { SENSOR_STEP_CNT, NULL },
    { SENSOR_COM, NULL },
};
static uint32_t sensor_poll_ctl_tbl_size = sizeof(sensor_poll_ctl_tbl)
                                           / sizeof(struct sensor_poll_ctl_str);
static struct mutex sensor_com_mutex;
struct mutex sensor_batch_mutex;
static int32_t sensor_type_status = 0;
static int32_t sensor_batch_status = 0;
static enum sensor_drv_status_e_type sensor_drv_status = 0;
enum sensor_micon_polling_e_type sensor_poll_status = SENSOR_POLLING_OFF;
enum sensor_micon_polling_e_type sensor_before_poll_status = SENSOR_POLLING_OFF;

static uint8_t gyro_idx[500];
static uint8_t mag_idx[500];
static uint16_t gyro_idx_num =0;
static uint16_t mag_idx_num = 0;
static uint16_t gyro_uncal_offset[3] ={0};
static uint16_t mag_uncal_offset[3] ={0};

uint16_t batch_app_task_flag;

static ssize_t sensor_com_info_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int ret;

    ret = scnprintf(buf, PAGE_SIZE,
                    "type[0x%08X] batch[0x%08X] drv[0x%08X] poll[0x%08X]\n",
                    (int)sensor_type_status,
                    (int)sensor_batch_status,
                    (int)sensor_drv_status,
                    (int)sensor_poll_status);

    return ret;
}

static ssize_t sensor_com_fw_update_sq_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count
)
{
    uint32_t data[2];
    uint8_t *data_addr = NULL;
    uint32_t len;
    uint8_t *fw_data = NULL;
    uint32_t ret;

    SENSOR_N_LOG("start");

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        return count;
    }

    sscanf(buf, "%x %d", &data[0],&data[1]);
    data_addr = (uint8_t*)data[0];
    len = data[1];

    if((data_addr == NULL) || (len == 0)){
        SENSOR_ERR_LOG("bad param-->data_addr[%X] len[%d]",
                       (int)data_addr,
                       (int)len);
        return count;
    }
    if((len % 4) != 0){
        SENSOR_ERR_LOG("bad param-->len[%d]:is not a multiple of 4",
                       (int)len);
        return count;
    }

    fw_data = (uint8_t *)kmalloc( len, GFP_KERNEL );

    if(fw_data == NULL){
        SENSOR_ERR_LOG("fail kmalloc--> NULL");
        return -ENOMEM;
    }

    ret = copy_from_user( fw_data, data_addr, len );
    if( ret != 0 )
    {
        SENSOR_ERR_LOG("fail copy_from_user-->ret[%d]",
                       (int)ret);
        kfree( fw_data );
        return count;
    }

    sensor_drv_set_status_local( SENSOR_FW_UPDATE );
    ret = sns_update_fw_seq(fw_data, len);
    sensor_drv_set_status_local( SENSOR_RESUME );
    kfree( fw_data );

    if(ret != 0){
        SENSOR_ERR_LOG("fail sns_update_fw_seq-->ret[%d]",
                       (int)ret);
        return count;
    }

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t sensor_com_fw_update_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint8_t *fw_data = NULL;
    uint8_t *data_addr = NULL;
    uint32_t len;
    uint32_t data[2];
    uint32_t ret;

    SENSOR_N_LOG("start");

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        return count;
    }

    sscanf(buf, "%x %d", &data[0],&data[1]);
    data_addr = (uint8_t*)data[0];
    len = data[1];

    if((data_addr == NULL) || (len == 0)){
        SENSOR_ERR_LOG("bad param-->data_addr[%X] len[%d]",
                       (int)data_addr,
                       (int)len);
        return count;
    }
    if((len % 4) != 0){
        SENSOR_ERR_LOG("bad param-->len[%d]:is not a multiple of 4",
                       (int)len);
        return count;
    }

    fw_data = (uint8_t *)kmalloc( len, GFP_KERNEL );

    if(fw_data == NULL){
        SENSOR_ERR_LOG("fail kmalloc--> NULL");
        return -ENOMEM;
    }

    ret = copy_from_user( fw_data, data_addr, len );
    if( ret != 0 )
    {
        SENSOR_ERR_LOG("fail copy_from_user-->ret[%d]",
                       (int)ret);
        kfree( fw_data );
        return count;
    }

    sensor_drv_set_status_local( SENSOR_FW_UPDATE );
    ret = sns_update_fw(fw_data, len);
    sensor_drv_set_status_local( SENSOR_RESUME );
    kfree( fw_data );

    if(ret != 0){
        SENSOR_ERR_LOG("fail sns_update_fw_seq-->ret[%d]",
                       (int)ret);
        return count;
    }

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t sensor_com_test0_ctl_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long direction;
    int32_t ret;

    SENSOR_N_LOG("start");

    ret = strict_strtoul(buf, 10, &direction);
    if (ret < 0){
        SENSOR_ERR_LOG("strict_strtoul()-->ret[%d]",(int)ret);
        return count;
    }

    if(direction == 1){
        SENSOR_A_LOG("BRMP port --> set input");
        sns_BRMP_direction();
    }

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t sensor_com_fw_version_chk_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    uint8_t  fw_ver[4];
    uint32_t ret;
    uint32_t fw_version;
    uint32_t fw_chk = SNESOR_COM_FW_VER_CHK_NG;

    SENSOR_N_LOG("start");

    ret = sns_get_fw_version(fw_ver);
    fw_version = SNESOR_COM_GET_FW_VER(fw_ver);
    SENSOR_N_LOG("fw_version[%x]",(int)fw_version);
    if(ret != SNS_RC_OK){
        fw_version = SNESOR_COM_FW_VER_NONE;
        SENSOR_ERR_LOG("fail get fw ver-->set ver[%x]",(int)fw_version);
    }

    if(fw_version == SNESOR_COM_FW_VER_DATA){
        fw_chk = SNESOR_COM_FW_VER_CHK_OK;
        SENSOR_A_LOG("fw ver chk OK: fw_chk[%x]",(int)fw_chk);
    } else {
        fw_chk = SNESOR_COM_FW_VER_CHK_NG;
        SENSOR_A_LOG("fw ver chk NG: fw_chk[%x]",(int)fw_chk);
    }

    if( ret != 0 ) {
        sns_err_check();
    }

    SENSOR_N_LOG("end - return[%d]",(int)fw_chk);
    return sprintf(buf, "%x\n", fw_chk);
}

static ssize_t sensor_com_fw_version_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    uint8_t  fw_ver[4];
    uint32_t ret;
    uint32_t fw_version;

    SENSOR_N_LOG("start");

    ret = sns_get_fw_version(fw_ver);
    fw_version = SNESOR_COM_GET_FW_VER(fw_ver);
    SENSOR_N_LOG("fw_version[%x]",(int)fw_version);
    if(ret != SNS_RC_OK){
        fw_version = SNESOR_COM_FW_VER_NONE;
        SENSOR_ERR_LOG("fail get fw ver-->set ver[%x]",(int)fw_version);
    }

    if( ret != 0 ) {
        sns_err_check();
    }

    SENSOR_N_LOG("end - return[%d]",(int)fw_version);
    return sprintf(buf, "%x\n", fw_version);
}

static enum sensor_micon_polling_e_type sensor_get_poll_type(
    enum sensor_e_type type)
{
    enum sensor_micon_polling_e_type ret = SENSOR_POLLING_OFF;
    SENSOR_N_LOG("start");

    switch (type) {
    case SENSOR_ACC_LNR:
    case SENSOR_GRV:
    case SENSOR_GYRO:
    case SENSOR_GYRO_UNCAL:
    case SENSOR_ORTN:
    case SENSOR_ROT_VCTR:
    case SENSOR_GAME_ROT_VCTR:
        ret = SENSOR_9AXIS_POLLING;
        break;
    case SENSOR_MAG:
    case SENSOR_MAG_UNCAL:
    case SENSOR_MAG_ROT_VCTR:
        ret = SENSOR_6AXIS_POLLING;
        break;
    case SENSOR_ACC:
        ret = SENSOR_3AXIS_POLLING_75;
        break;
    case SENSOR_EXT_PEDO:
    case SENSOR_EXT_VEHI:
    case SENSOR_EXT_IWIFI:
    case SENSOR_KC_MOTION_TRAIN:
    case SENSOR_KC_MOTION_VEHICLE:
        ret = SENSOR_6AXIS_POLLING_A300M2400;
        break;
    case SENSOR_SGNFCNT_MTN:
    case SENSOR_STEP_CNT:
    case SENSOR_STEP_DTC:
    case SENSOR_EXT_VH:
    case SENSOR_KC_MOTION_WALK_START:
    case SENSOR_KC_MOTION_WALK_STOP:
    case SENSOR_KC_MOTION_BRINGUP:
        ret = SENSOR_3AXIS_POLLING_300;
        break;
    case SENSOR_LIGHT:
    case SENSOR_PROX:
    case SENSOR_RH:
    case SENSOR_TEMP:
    case SENSOR_TEMP_AMBIENT:
    case SENSOR_PRESSURE:
    default:
        break;
    }

    SENSOR_N_LOG("end - ret[%d]",ret);
    return ret;
}

static enum sensor_chk_status_e_type sensor_check_status(
    enum sensor_e_type type,
    bool enable )
{
    enum sensor_chk_status_e_type ret = SENSOR_CHK_NORMAL;

    if((SENSOR_ON<<type) & (SENSOR_NOT_SUPPORT_TYPE)){
        SENSOR_ERR_LOG("not supported!! type[%d]",(int)type);
        return SENSOR_CHK_SKIP_ALL;
    }

    if( true == enable ){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            ret = SENSOR_CHK_SKIP_ALL;
        } else if ( (SENSOR_ON<<type) & (~SENSOR_NOT_MICON_TYPE) ){
            ret = SENSOR_CHK_POLL_CTL;
        }
    } else {
        if( SENSOR_OFF == sensor_get_status_local(type) ){
            ret = SENSOR_CHK_SKIP_ALL;
        } else if ( (SENSOR_ON<<type) & (~SENSOR_NOT_MICON_TYPE) ){
            ret = SENSOR_CHK_POLL_CTL;
        }
    }

    return ret;
}

static int32_t sensor_poll_on_ctrl( enum sensor_e_type type )
{
    int32_t ret = 0;
    enum sensor_micon_polling_e_type sensor_poll_request = SENSOR_POLLING_OFF;

    SENSOR_N_LOG("start");

    sensor_poll_request = sensor_get_poll_type(type);
    SENSOR_N_LOG("req poll [%d]",(int)sensor_poll_request);

    if( sensor_poll_request > sensor_poll_status ) {
        sensor_before_poll_status = sensor_poll_status;
        sensor_poll_status = sensor_poll_request;
        ret = sensor_type_enable( type, true );
    }

    SENSOR_N_LOG("befor state [%d] poll state [%d]",(int)sensor_before_poll_status, (int)sensor_poll_status);
    SENSOR_N_LOG("end");

    return ret;
}

static int32_t sensor_poll_off_ctrl( enum sensor_e_type type )
{
    int32_t ret = 0;
    uint32_t micon_any_on = 0;
    enum sensor_e_type next_type = SENSOR_MAX;
    enum sensor_e_type type_counter;
    enum sensor_micon_polling_e_type poll_tmp = SENSOR_POLLING_OFF;
    enum sensor_micon_polling_e_type poll_highest = SENSOR_POLLING_OFF;

    SENSOR_N_LOG("start");

    micon_any_on = ( (sensor_type_status)
                     & (~SENSOR_NOT_MICON_TYPE)
                     & (~(SENSOR_ON<<type)) );

    if( micon_any_on ){
        SENSOR_N_LOG("micon_any_on [0x%X]",(int)micon_any_on);
        for(type_counter=SENSOR_ACC; type_counter<SENSOR_MAX; type_counter++) {
            if( ( SENSOR_ON == sensor_get_status_local(type_counter))
                && ( type_counter != type )) {
                poll_tmp = sensor_get_poll_type(type_counter);
                if( poll_tmp >= poll_highest ) {
                    poll_highest = poll_tmp;
                    next_type = type_counter;
                }
            }
        }

        sensor_before_poll_status = sensor_poll_status;
        sensor_poll_status = poll_highest;
        ret = sensor_type_enable( next_type, true );
    } else {
        SENSOR_N_LOG("micon_any NO on");
        sensor_before_poll_status = sensor_poll_status;
        sensor_poll_status = SENSOR_POLLING_OFF;
        ret = sensor_type_enable( type, false );
    }

    SENSOR_N_LOG("befor state [%d] poll state [%d]",(int)sensor_before_poll_status, (int)sensor_poll_status);
    SENSOR_N_LOG("end");
    return ret;
}

enum sensor_micon_polling_e_type sensor_get_poll_status(void)
{
    enum sensor_micon_polling_e_type ret = SENSOR_POLLING_OFF;

    SENSOR_N_LOG("start[%d]",ret);
    ret = sensor_before_poll_status;
    SENSOR_N_LOG("end[%d]",ret);

    return ret;
}

enum sensor_micon_polling_e_type sensor_get_poll_status_current(void)
{
    enum sensor_micon_polling_e_type ret = SENSOR_POLLING_OFF;

    SENSOR_N_LOG("start[%d]",ret);
    ret = sensor_poll_status;
    SENSOR_N_LOG("end[%d]",ret);

    return ret;
}

static void sensor_com_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev[%d]",(int)dev);
        return;
    }

    dev->name = "sensor_com";
    dev->id.bustype = BUS_SPI;

    SENSOR_N_LOG("end");
    return;
}

static int32_t sensor_type_enable(enum sensor_e_type type,bool enable)
{
    int32_t ret = 0;

    SENSOR_N_LOG("type [%d] enable[%d]",(int)type,(int)enable);

    switch (type) {
    case SENSOR_ACC:
        ret = sns_acc_activate( enable );
        break;
    case SENSOR_GYRO:
        ret = sns_gyro_activate( enable );
        break;
    case SENSOR_MAG:
        ret = sns_mag_activate( enable );
        break;
    case SENSOR_ACC_LNR:
        ret = sns_linacc_activate( enable );
        break;
    case SENSOR_GRV:
        ret = sns_gravity_activate( enable );
        break;
    case SENSOR_GYRO_UNCAL:
        ret = sns_gyrouncalib_activate( enable );
        break;
    case SENSOR_MAG_UNCAL:
        ret = sns_maguncalib_activate( enable );
        break;
    case SENSOR_ORTN:
        ret = sns_ori_activate( enable );
        break;
    case SENSOR_ROT_VCTR:
        ret = sns_rota_activate( enable );
        break;
    case SENSOR_GAME_ROT_VCTR:
        ret = sns_gamerota_activate( enable );
        break;
    case SENSOR_MAG_ROT_VCTR:
        ret = sns_magrota_activate( enable );
        break;
    case SENSOR_LIGHT:
        gp2ap_als_sensor_activate( enable );
        break;
    case SENSOR_PROX:
        gp2ap_ps_sensor_activate( enable );
        break;
    case SENSOR_SGNFCNT_MTN:
        ret = sns_motion_activate( enable );
        break;
    case SENSOR_STEP_CNT:
        ret = sns_scount_activate( enable );
        break;
    case SENSOR_STEP_DTC:
        ret = sns_sdetect_activate( enable );
        break;
    case SENSOR_EXT_PEDO:
        ret = sns_dailys_activate( enable );
        break;
    case SENSOR_EXT_VEHI:
        ret = sns_dailys_activate( enable );
        break;
    case SENSOR_EXT_IWIFI:
        ret = sns_iwifi_activate( enable );
        break;
    case SENSOR_EXT_VH:
        ret = sns_vhdetect_activate( enable );
        break;
    case SENSOR_KC_MOTION_WALK_START:
        ret = sns_kc_motion_walk_start_activate( enable );
        break;
    case SENSOR_KC_MOTION_WALK_STOP:
        ret = sns_kc_motion_walk_stop_activate( enable );
        break;
    case SENSOR_KC_MOTION_TRAIN:
        ret = sns_kc_motion_train_activate( enable );
        break;
    case SENSOR_KC_MOTION_VEHICLE:
        ret = sns_kc_motion_vehicle_activate( enable );
        break;
    case SENSOR_KC_MOTION_BRINGUP:
        ret = sns_kc_motion_bringup_activate( enable );
        break;
    case SENSOR_RH:
    case SENSOR_TEMP:
    case SENSOR_TEMP_AMBIENT:
    case SENSOR_PRESSURE:
    default:
        break;
    }

    SENSOR_N_LOG("ret [%d]",(int)ret );

    return ret;
}
static int32_t sensor_application_enable(enum sensor_e_type type,bool enable)
{
    int32_t ret = 0;

    SENSOR_N_LOG("type [%d] enable[%d]",(int)type,(int)enable);

    switch (type) {
    case SENSOR_SGNFCNT_MTN:
        ret = sns_motion_start( enable );
        break;
    case SENSOR_EXT_PEDO:
        ret = sns_dailys_start( enable );
        break;
    case SENSOR_EXT_VEHI:
        ret = sns_dailys_start( enable );
        break;
    case SENSOR_EXT_IWIFI:
        ret = sns_iwifi_start( enable );
        break;
    case SENSOR_EXT_VH:
        ret = sns_vhdetect_start( enable );
        break;
    case SENSOR_KC_MOTION_WALK_START:
        ret = sns_kc_motion_walk_start_start( enable );
        break;
    case SENSOR_KC_MOTION_WALK_STOP:
        ret = sns_kc_motion_walk_stop_start( enable );
        break;
    case SENSOR_KC_MOTION_TRAIN:
        ret = sns_kc_motion_train_start( enable );
        break;
    case SENSOR_KC_MOTION_VEHICLE:
        ret = sns_kc_motion_vehicle_start( enable );
        break;
    case SENSOR_KC_MOTION_BRINGUP:
        ret = sns_kc_motion_bringup_start( enable );
        break;
    default:
        break;
    }

    SENSOR_N_LOG("ret [%d]",(int)ret );

    return ret;
}

static uint32_t sensor_sort_batch_data(uint8_t* buffer_p)
{
    uint32_t total_size = MICON_FIFO_SIZE;
    uint32_t i;
    uint32_t x = 0;
    uint32_t tmp_status = 0;

    uint8_t* src_buffer_p;
    uint8_t* tmp_buff_p;
    uint8_t* tmp_buff_work_p;

    if( NULL == buffer_p ){
        SENSOR_ERR_LOG("bad parm buffer_p is NULL");
        return -1;
    }

    tmp_buff_p = kmalloc( MICON_FIFO_SIZE, GFP_KERNEL );
    if( NULL == tmp_buff_p ){
        SENSOR_ERR_LOG("fail alloc tmp buffer");
        return -1;
    }

    gyro_idx_num = 0;
    mag_idx_num  = 0;

    tmp_status = sensor_get_batch_status();
    if(tmp_status & (BATCH_ON << SENSOR_GYRO_UNCAL)){
        src_buffer_p = buffer_p;
        while(src_buffer_p < (buffer_p + total_size)){
            for(i = 0; i<sensor_micon_batch_info_tbl_size; i++){
                if( *src_buffer_p == sensor_micon_batch_info_tbl[i].id){
                    if( *src_buffer_p == SENSOR_BATCH_ID_GYRO){
                        gyro_idx[x] = 0x01;
                        x++;
                    }
                    if( *src_buffer_p == SENSOR_BATCH_ID_GYRO_UNCAL){
                        gyro_idx[x] = 0x02;
                        x++;
                    }
                    src_buffer_p = src_buffer_p + (sensor_micon_batch_info_tbl[i].size + 1);
                    break;
                }
            }
            if(i == sensor_micon_batch_info_tbl_size){
                break;
            }
        }
        gyro_idx_num = x;
        x = 0;
    }

    if(tmp_status & (BATCH_ON << SENSOR_MAG_UNCAL)){
        src_buffer_p = buffer_p;
        while(src_buffer_p < (buffer_p + total_size)){
            for(i = 0; i<sensor_micon_batch_info_tbl_size; i++){
                if( *src_buffer_p == sensor_micon_batch_info_tbl[i].id){
                    if( *src_buffer_p == SENSOR_BATCH_ID_MAG){
                        mag_idx[x] = 0x01;
                        x++;
                    }
                    if( *src_buffer_p == SENSOR_BATCH_ID_MAG_UNCAL){
                        mag_idx[x] = 0x02;
                        x++;
                    }
                    src_buffer_p = src_buffer_p + (sensor_micon_batch_info_tbl[i].size + 1);
                    break;
                }
            }
            if(i == sensor_micon_batch_info_tbl_size){
                break;
            }
        }
        mag_idx_num = x;
        x = 0;
    }

    src_buffer_p = buffer_p;
    while(src_buffer_p < (buffer_p + total_size)){
        for(i = 0; i<sensor_micon_batch_info_tbl_size; i++){
            if( *src_buffer_p == sensor_micon_batch_info_tbl[i].id){
                sensor_micon_batch_info_tbl[i].data_info.recode_num++;
                src_buffer_p = src_buffer_p + (sensor_micon_batch_info_tbl[i].size + 1);
                break;
            }
        }
        if(i == sensor_micon_batch_info_tbl_size){
            break;
        }
    }

    tmp_buff_work_p = tmp_buff_p;
    for(i = 0; i<sensor_micon_batch_info_tbl_size; i++){
        if(sensor_micon_batch_info_tbl[i].data_info.recode_num){
            sensor_micon_batch_info_tbl[i].tmp_buff_p = tmp_buff_work_p;
            tmp_buff_work_p += (sensor_micon_batch_info_tbl[i].data_info.recode_num
                                 * sensor_micon_batch_info_tbl[i].size) ;
        }
    }

    src_buffer_p = buffer_p;
    while(src_buffer_p < (buffer_p + total_size)){
        for(i = 0; i<sensor_micon_batch_info_tbl_size; i++){
            if( *src_buffer_p == sensor_micon_batch_info_tbl[i].id){
                memcpy( (sensor_micon_batch_info_tbl[i].tmp_buff_p
                          + sensor_micon_batch_info_tbl[i].data_info.payload_size),
                       src_buffer_p+1,
                       sensor_micon_batch_info_tbl[i].size);
                src_buffer_p = src_buffer_p + (sensor_micon_batch_info_tbl[i].size + 1);
                sensor_micon_batch_info_tbl[i].data_info.payload_size
                    += sensor_micon_batch_info_tbl[i].size;
                break;
            }
        }
        if(i == sensor_micon_batch_info_tbl_size){
            break;
        }
    }

    memset(buffer_p,0x00,MICON_FIFO_SIZE);
    src_buffer_p = buffer_p;
    for(i = 0; i<sensor_micon_batch_info_tbl_size; i++){
        if(sensor_micon_batch_info_tbl[i].data_info.recode_num){
            memcpy( src_buffer_p,
                    sensor_micon_batch_info_tbl[i].tmp_buff_p,
                    sensor_micon_batch_info_tbl[i].data_info.payload_size);

            sensor_micon_batch_info_tbl[i].data_info.payload_p = src_buffer_p;
            src_buffer_p += sensor_micon_batch_info_tbl[i].data_info.payload_size;
        }
    }

    kfree(tmp_buff_p);

    return 0;
}

static void sensor_broadcast_batch( enum sensor_batch_report_e_type repo_type )
{
    uint32_t i;
    uint32_t tmp_status = 0;
    SENSOR_N_LOG("start");

    for(i = 0; i<sensor_micon_batch_info_tbl_size; i++){

        SENSOR_N_LOG("sensor_micon_batch_info_tbl[i].data_info.recode_num:%d",sensor_micon_batch_info_tbl[i].data_info.recode_num);

        if(( sensor_micon_batch_info_tbl[i].data_info.recode_num) || (repo_type == SENSOR_COMP_FLUSH)){

            SENSOR_N_LOG("switch");

            switch(sensor_micon_batch_info_tbl[i].id){
            case SENSOR_BATCH_ID_ACC:
                SENSOR_N_LOG("report batch for ACC");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_ACC)){
                    acc_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            case SENSOR_BATCH_ID_MAG:
                SENSOR_N_LOG("report batch for MAG");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_MAG)){
                    SENSOR_N_LOG("report batch for MAG-MAG");
                    mag_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                if(tmp_status & (BATCH_ON << SENSOR_MAG_UNCAL)){
                    SENSOR_N_LOG("report batch for MAG-UNCAL");
                    mag_uncal_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            case SENSOR_BATCH_ID_GYRO:
                SENSOR_N_LOG("report batch for GYRO");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_GYRO)){
                    SENSOR_N_LOG("report batch for GYRO-GYRO");
                    gyro_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                if(tmp_status & (BATCH_ON << SENSOR_GYRO_UNCAL)){
                    SENSOR_N_LOG("report batch for GYRO-UNCAL");
                    gyro_uncal_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            case SENSOR_BATCH_ID_MAG_UNCAL:
                SENSOR_N_LOG("report batch for MAG_UNCAL");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_MAG_UNCAL)){
                    mag_uncal2_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            case SENSOR_BATCH_ID_GYRO_UNCAL:
                SENSOR_N_LOG("report batch for GYRO_UNCAL");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_GYRO_UNCAL)){
                    gyro_uncal2_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            case SENSOR_BATCH_ID_ORTN :
                SENSOR_N_LOG("report batch for ORTN");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_ORTN)){
                    ortn_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            case SENSOR_BATCH_ID_GRV:
                SENSOR_N_LOG("report batch for GRV");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_GRV)){
                    grv_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            case SENSOR_BATCH_ID_ACC_LNR:
                SENSOR_N_LOG("report batch for ACC_LNR");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_ACC_LNR)){
                     acc_lnr_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            case SENSOR_BATCH_ID_ROT_VCTR:
                SENSOR_N_LOG("report batch for ROT_VCTR");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_ROT_VCTR)){
                    rot_vctr_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            case SENSOR_BATCH_ID_MAG_ROT_VCTR:
                SENSOR_N_LOG("report batch for MAG_ROT_VCTR");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_MAG_ROT_VCTR)){
                    mag_rot_vctr_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            case SENSOR_BATCH_ID_GAME_ROT_VCTR:
                SENSOR_N_LOG("report batch for GAME_ROT_VCTR");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_GAME_ROT_VCTR)){
                    game_rot_vctr_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            case SENSOR_BATCH_ID_REL_STEP_1:
                SENSOR_N_LOG("report batch for REL_STEP_1");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_STEP_CNT)){
                    SENSOR_N_LOG("report batch for REL_STEP_1-STEP_CNT");
                    step_cnt_step1_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                if(tmp_status & (BATCH_ON << SENSOR_STEP_DTC)){
                    SENSOR_N_LOG("report batch for REL_STEP_1-STEP_DTC");
                    step_dtc_step1_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            case SENSOR_BATCH_ID_REL_STEP_2:
                SENSOR_N_LOG("report batch for REL_STEP_2");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_STEP_CNT)){
                    SENSOR_N_LOG("report batch for REL_STEP_2-STEP_CNT");
                    step_cnt_step2_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                if(tmp_status & (BATCH_ON << SENSOR_STEP_DTC)){
                    SENSOR_N_LOG("report batch for REL_STEP_2-STEP_DTC");
                    step_dtc_step2_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            case SENSOR_BATCH_ID_ABS_STEP:
                SENSOR_N_LOG("report batch for ABS_STEP");
                tmp_status = sensor_get_batch_status();
                if(tmp_status & (BATCH_ON << SENSOR_STEP_CNT)){
                    SENSOR_N_LOG("report batch for ABS_STEP-STEP_CNT");
                    step_cnt_step_abs_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                if(tmp_status & (BATCH_ON << SENSOR_STEP_DTC)){
                    SENSOR_N_LOG("report batch for ABS_STEP-STEP_DTC");
                    step_dtc_step_abs_report_batch( repo_type,
                                      sensor_micon_batch_info_tbl[i].data_info );
                }
                sensor_micon_batch_info_tbl[i].data_info.payload_size = 0;
                sensor_micon_batch_info_tbl[i].data_info.recode_num = 0;
                break;
            default:
                SENSOR_N_LOG("default");
                break;
            }
        }
    }

    SENSOR_N_LOG("end");
    return;
}

void sensor_set_batch_data(void)
{
    struct sensor_batch_str batch;
    int32_t ret = 0;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_batch_mutex);
    mutex_lock(&sensor_com_mutex);

    ret = sns_micon_get_batch_data(&batch);
    if(ret < 0){
        mutex_unlock(&sensor_com_mutex);
        mutex_unlock(&sensor_batch_mutex);
        SENSOR_ERR_LOG("fail sns_micon_get_batch_data ret[%d]",ret);
        return;
    }
    ret = sensor_sort_batch_data( batch.buffer_p );
    if(ret < 0){
        mutex_unlock(&sensor_com_mutex);
        mutex_unlock(&sensor_batch_mutex);
        SENSOR_ERR_LOG("fail sensor_sort_batch_data ret[%d]",ret);
        return;
    }
    sensor_broadcast_batch( batch.repo_type );

    mutex_unlock(&sensor_com_mutex);
    mutex_unlock(&sensor_batch_mutex);
    SENSOR_N_LOG("end");

    return;
}

int32_t sensor_timestamp_report(void)
{
    uint32_t batch_status = 0;
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    batch_status = sensor_get_batch_status();

    if(batch_status & (BATCH_ON << SENSOR_ACC)){
        acc_timestamp_report();
    }

    if(batch_status & (BATCH_ON << SENSOR_MAG)){
        mag_timestamp_report();
    }

    if(batch_status & (BATCH_ON << SENSOR_MAG_UNCAL)){
        mag_uncal_timestamp_report();
    }

    if(batch_status & (BATCH_ON << SENSOR_GYRO)){
        gyro_timestamp_report();
    }

    if(batch_status & (BATCH_ON << SENSOR_GYRO_UNCAL)){
        gyro_uncal_timestamp_report();
    }

    if(batch_status & (BATCH_ON << SENSOR_ORTN)){
        ortn_timestamp_report();
    }

    if(batch_status & (BATCH_ON << SENSOR_GRV)){
        grv_timestamp_report();
    }

    if(batch_status & (BATCH_ON << SENSOR_ACC_LNR)){
        acc_lnr_timestamp_report();
    }

    if(batch_status & (BATCH_ON << SENSOR_ROT_VCTR)){
        rot_vctr_timestamp_report();
    }

    if(batch_status & (BATCH_ON << SENSOR_MAG_ROT_VCTR)){
        mag_rot_vctr_timestamp_report();
    }

    if(batch_status & (BATCH_ON << SENSOR_GAME_ROT_VCTR)){
        game_rot_vctr_timestamp_report();
    }

    if(batch_status & (BATCH_ON << SENSOR_STEP_CNT)){
        step_cnt_timestamp_report();
    }

    if(batch_status & (BATCH_ON << SENSOR_STEP_DTC)){
        step_dtc_timestamp_report();
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;

}

uint32_t sensor_set_batch(
    enum sensor_e_type type,
    struct sensor_batch_info_str info )
{
    uint32_t ret = 0;
    struct sensor_poll_info_str* poll_info_p = NULL;
    int i;

    enum sensor_batch_status_e_type next_status;
    SENSOR_N_LOG("start");

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        return -1;
    }

    mutex_lock(&sensor_com_mutex);

    if( 0 == (info.timeout)){
        SENSOR_A_LOG("Batch OFF");
        if( 0 == (sensor_batch_status & ( BATCH_ON << type)) ) {
            SENSOR_ERR_LOG( "Sensor already OFF" );
            mutex_unlock(&sensor_com_mutex);
            return -1;
        }
        next_status = BATCH_OFF;

        SENSOR_A_LOG("do batch-off");
        sns_logging_state( type, info.timeout, info.period_ns );

        SENSOR_A_LOG("Batch ON - sensor off");
        mutex_unlock(&sensor_com_mutex);
        sensor_enable(type, NULL, false);
        mutex_lock(&sensor_com_mutex);

    } else {
        SENSOR_A_LOG("Batch ON");
        if( 0 != (sensor_batch_status & ( BATCH_ON << type)) ) {
            SENSOR_ERR_LOG( "Sensor already ON" );
            mutex_unlock(&sensor_com_mutex);
            return -1;
        }
        next_status = BATCH_ON;

        for(i=0; i<sensor_poll_ctl_tbl_size; i++){
            if( type == sensor_poll_ctl_tbl[i].type ){
                poll_info_p = sensor_poll_ctl_tbl[i].poll_info_p;
                break;
            }
        }

        if( SENSOR_ON == sensor_get_status_local(type) ){
            SENSOR_A_LOG("Batch ON - sensor off");
            mutex_unlock(&sensor_com_mutex);
            sensor_enable(type, poll_info_p, false);
            mutex_lock(&sensor_com_mutex);
        }
        SENSOR_A_LOG("Batch ON - sensor on");

        batch_app_task_flag = 1;

        mutex_unlock(&sensor_com_mutex);
        sns_set_period( type, info.period_ns, 1 );
        mutex_lock(&sensor_com_mutex);

        mutex_unlock(&sensor_com_mutex);
        sensor_enable(type, NULL, true);
        mutex_lock(&sensor_com_mutex);

        SENSOR_A_LOG("do batch-on");
        sns_logging_state( type, info.timeout, info.period_ns );

        sns_set_app_task_batch(type);

    }

    sensor_set_batch_status( type, next_status );

    if((0 == (info.timeout)) && (0 == sensor_batch_status)){
        batch_app_task_flag = 0;
    }

    mutex_unlock(&sensor_com_mutex);

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

uint32_t sensor_set_flush( enum sensor_e_type type , struct input_dev* dev)
{
    uint32_t ret = 0;
    uint32_t tmp_status = 0;

    SENSOR_N_LOG("start");

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        return -1;
    }

    mutex_lock(&sensor_com_mutex);

    tmp_status = sensor_get_batch_status();

    if(tmp_status & (BATCH_ON << type)){
        SENSOR_N_LOG("test flush");
        sns_flush_batch();
    }else{
        SENSOR_N_LOG("test flush abs");
        input_report_abs(dev, ABS_MISC, -1);
        input_report_abs(dev, ABS_MISC, SENSOR_COMP_FLUSH_BF);
        input_sync(dev);
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sensor_copy_batch_data(
   enum sensor_micon_batch_tbl_num_e_type type,
   struct sensor_batch_data_str batch_data,
   uint8_t* hal_addr)
{
    uint8_t *dst_addr = NULL;

    SENSOR_N_LOG("start");

    if( (NULL == hal_addr)||(NULL == batch_data.payload_p) ){
       SENSOR_ERR_LOG( "bat hal_addr[%X] batch_addr[%x]",
                       (int)hal_addr,
                       (int)batch_data.payload_p );
       return -1;
    }

    dst_addr = hal_addr;

    if( copy_to_user( dst_addr,
                      &(batch_data.payload_size),
                      sizeof(batch_data.payload_size) ) ){
       SENSOR_ERR_LOG("miss copy payload size");
       return -1;
    }

    dst_addr = dst_addr + sizeof(batch_data.payload_size);

    if( copy_to_user( dst_addr,
                      &(batch_data.recode_num),
                      sizeof(batch_data.recode_num) ) ){
       SENSOR_ERR_LOG("miss copy recode num");
       return -1;
    }

    dst_addr = dst_addr + sizeof(batch_data.recode_num);

    if( copy_to_user( dst_addr,
                     (batch_data.payload_p),
                     (batch_data.payload_size) )){
       SENSOR_ERR_LOG("miss copy payload");
       return -1;
    }

    SENSOR_N_LOG("end");

    return 0;
}

int32_t sensor_copy_batch_uncal_data(
   enum sensor_micon_batch_tbl_num_e_type type,
   struct sensor_batch_data_str batch_data,
   struct sensor_batch_data_str batch_data_cal,
   uint8_t* hal_addr)
{
    int i;
    uint8_t  *dst_addr = NULL;
    uint8_t  *uncal_data = NULL;
    uint8_t  *work_p = NULL;
    uint8_t  *batch_data_p =NULL;
    uint8_t  *batch_data_cal_p =NULL;

    SENSOR_N_LOG("start");

    uncal_data = (uint8_t *)kmalloc( batch_data.payload_size, GFP_KERNEL );
    if(uncal_data == NULL)
    {
        return -1;
    }

    work_p = uncal_data;
    batch_data_p     = batch_data.payload_p;
    batch_data_cal_p = batch_data_cal.payload_p;
    
    if( SENSOR_BATCH_TBL_MAG_UNCAL == type ){
        for(i = 0; i < mag_idx_num; i++){
            if(mag_idx[i] == 0x01){
                SENSOR_N_LOG("mag_idx[%d]:%d",i,mag_idx[i]);
                memcpy(work_p, batch_data_p, SENSOR_BATCH_SIZE_MAG);
                work_p += SENSOR_BATCH_SIZE_MAG;
                batch_data_p += SENSOR_BATCH_SIZE_MAG;

                *(work_p++) = (mag_uncal_offset[0] & 0xFF);
                *(work_p++) = ((mag_uncal_offset[0] >> 8) & 0xFF);
                *(work_p++) = mag_uncal_offset[1];
                *(work_p++) = ((mag_uncal_offset[1] >> 8) & 0xFF);
                *(work_p++) = mag_uncal_offset[2];
                *(work_p++) = ((mag_uncal_offset[2] >> 8) & 0xFF);
            } else if(mag_idx[i] == 0x02){
                SENSOR_N_LOG("mag_idx[%d]:%d",i,mag_idx[i]);
                mag_uncal_offset[0] = *(batch_data_cal_p++);
                mag_uncal_offset[0] = mag_uncal_offset[0] | ((*(batch_data_cal_p++) & 0xFF ) << 8);
                mag_uncal_offset[1] = *(batch_data_cal_p++);
                mag_uncal_offset[1] = mag_uncal_offset[1] | ((*(batch_data_cal_p++) & 0xFF ) << 8);
                mag_uncal_offset[2] = *(batch_data_cal_p++);
                mag_uncal_offset[2] = mag_uncal_offset[2] | ((*(batch_data_cal_p++) & 0xFF ) << 8);
            }
        }
    }
    if( SENSOR_BATCH_TBL_GYRO_UNCAL == type ){
        for(i = 0; i < gyro_idx_num; i++){
            if(gyro_idx[i] == 0x01){
                SENSOR_N_LOG("gyro_idx[%d]:%d",i,gyro_idx[i]);
                memcpy(work_p, batch_data_p, SENSOR_BATCH_SIZE_GYRO);
                work_p += SENSOR_BATCH_SIZE_GYRO;
                batch_data_p += SENSOR_BATCH_SIZE_GYRO;

                *(work_p++) = (gyro_uncal_offset[0] & 0xFF);
                *(work_p++) = ((gyro_uncal_offset[0] >> 8) & 0xFF);
                *(work_p++) = gyro_uncal_offset[1];
                *(work_p++) = ((gyro_uncal_offset[1] >> 8) & 0xFF);
                *(work_p++) = gyro_uncal_offset[2];
                *(work_p++) = ((gyro_uncal_offset[2] >> 8) & 0xFF);

            } else if(gyro_idx[i] == 0x02){
                SENSOR_N_LOG("gyro_idx[%d]:%d",i,gyro_idx[i]);
                gyro_uncal_offset[0] = *(batch_data_cal_p++);
                gyro_uncal_offset[0] = gyro_uncal_offset[0] | ((*(batch_data_cal_p++) & 0xFF ) << 8);
                gyro_uncal_offset[1] = *(batch_data_cal_p++);
                gyro_uncal_offset[1] = gyro_uncal_offset[1] | ((*(batch_data_cal_p++) & 0xFF ) << 8);
                gyro_uncal_offset[2] = *(batch_data_cal_p++);
                gyro_uncal_offset[2] = gyro_uncal_offset[2] | ((*(batch_data_cal_p++) & 0xFF ) << 8);
            }
        }
    }

    if( (NULL == hal_addr)||(NULL == batch_data.payload_p) ){
       SENSOR_ERR_LOG( "bat hal_addr[%X] batch_addr[%x]",
                       (int)hal_addr,
                       (int)batch_data.payload_p );
       kfree(uncal_data);
       return -1;
    }

    dst_addr = hal_addr;

    if( copy_to_user( dst_addr,
                      &(batch_data.payload_size),
                      sizeof(batch_data.payload_size) ) ){
       SENSOR_ERR_LOG("miss copy payload size");
       kfree(uncal_data);
       return -1;
    }

    dst_addr = dst_addr + sizeof(batch_data.payload_size);

    if( copy_to_user( dst_addr,
                      &(batch_data.recode_num),
                      sizeof(batch_data.recode_num) ) ){
       SENSOR_ERR_LOG("miss copy recode num");
       kfree(uncal_data);
       return -1;
    }

    dst_addr = dst_addr + sizeof(batch_data.recode_num);

    if( copy_to_user( dst_addr,
                      uncal_data,
                      batch_data.payload_size)){
       SENSOR_ERR_LOG("miss copy payload");
       kfree(uncal_data);
       return -1;
    }

    kfree(uncal_data);

    SENSOR_N_LOG("end");

    return 0;
}

void sensor_report_batch(
    struct input_dev* dev,
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data )
{

    SENSOR_N_LOG("start");

    if(SENSOR_COMP_TIME == repo_type){
        input_report_abs(dev, ABS_MISC, -1);
        input_report_abs(dev, ABS_MISC, repo_type);
    } else if(SENSOR_COMP_ABS == repo_type) {
        input_report_abs(dev, ABS_MISC, -1);
        input_report_abs(dev, ABS_MISC, repo_type);
        input_sync(dev);
    } else {
        input_report_abs(dev, ABS_MISC, -1);
        input_report_abs(dev, ABS_MISC, repo_type);
        input_sync(dev);
    }

    SENSOR_N_LOG("end");
    return;
}

uint32_t sensor_get_status(enum sensor_e_type type)
{
    int32_t status = 0;
    SENSOR_N_LOG("start");

    mutex_lock(&sensor_com_mutex);
    status = sensor_get_status_local(type);
    mutex_unlock(&sensor_com_mutex);

    SENSOR_N_LOG("end");
    return status;
}

static uint32_t sensor_get_status_local(enum sensor_e_type type)
{
    int32_t status = 0;
    SENSOR_N_LOG("start");
    status = ( (sensor_type_status & (SENSOR_ON << type)) >> type );
    SENSOR_N_LOG("end");
    return status;
}

enum sensor_drv_status_e_type sensor_drv_get_status( void )
{
   enum sensor_drv_status_e_type ret;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    ret = sensor_drv_get_status_local();

    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");
    return ret;
}

static enum sensor_drv_status_e_type sensor_drv_get_status_local(void)
{
    enum sensor_drv_status_e_type ret;

    SENSOR_N_LOG("start");
    ret = sensor_drv_status;
    SENSOR_N_LOG("end");

    return ret;
}

void sensor_drv_set_status( enum sensor_drv_status_e_type next_status )
{
    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    sensor_drv_set_status_local(next_status);

    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");
    return;
}

static void sensor_drv_set_status_local( enum sensor_drv_status_e_type next_status )
{
    SENSOR_N_LOG("start");
    sensor_drv_status = next_status;
    SENSOR_N_LOG("end");
    return;
}

void sensor_set_status(
    enum sensor_e_type type,
    enum sensor_type_status_e_type on
)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("parm: type[%d] on[%d] - status[%X]",
                 (int)type,(int)on,(int)sensor_type_status);

    if( on == SENSOR_ON) {
        sensor_type_status = (sensor_type_status | ( SENSOR_ON << type) );
    } else {
        sensor_type_status = (sensor_type_status & (~( SENSOR_ON << type)) );
    }

    SENSOR_N_LOG("status to [%X]",(int)sensor_type_status);
    SENSOR_N_LOG("end");
    return;
}

static void sensor_set_batch_status(
    enum sensor_e_type type,
    enum sensor_batch_status_e_type on )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("parm: type[%d] on[%d] - status[%X]",
                 (int)type,(int)on,(int)sensor_batch_status);

    if( on == BATCH_ON) {
        sensor_batch_status = (sensor_batch_status | ( BATCH_ON << type) );
    } else {
        sensor_batch_status = (sensor_batch_status & (~( BATCH_ON << type)) );
    }

    SENSOR_N_LOG("status to [%X]",(int)sensor_batch_status);
    SENSOR_N_LOG("end");
    return;
}

uint32_t sensor_get_batch_status(void)
{
    uint32_t ret;

    SENSOR_N_LOG("start");
    ret = sensor_batch_status;
    SENSOR_N_LOG("end");

    return ret;
}

void sensor_poll_init(
    enum sensor_e_type type,
    struct sensor_poll_info_str* poll_info )
{
   int i;

    SENSOR_N_LOG("start");

    if(!poll_info){
        return;
    }

    poll_info->poll_wq = create_singlethread_workqueue((poll_info->name));
    INIT_DELAYED_WORK(&(poll_info->poll_work), (poll_info->poll_func));

    for( i=0; i<sensor_poll_ctl_tbl_size; i++ ){
        if( sensor_poll_ctl_tbl[i].type == type ){
            sensor_poll_ctl_tbl[i].poll_info_p = poll_info;
            break;
        }
    }

    SENSOR_N_LOG("end");

    return;
}

int32_t sensor_type_get_data(
    enum sensor_e_type type,
    union sensor_read_data_u* read_data )
{
    int32_t ret = 0;

    mutex_lock(&sensor_com_mutex);

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        mutex_unlock(&sensor_com_mutex);
        return ret;
    }

    switch (type) {
    case SENSOR_ACC:
        ret = sns_acc_read_data( &(read_data->acc_data) );
        break;
    case SENSOR_GYRO:
        ret = sns_gyro_read_data( &(read_data->gyro_data) );
        break;
    case SENSOR_MAG:
        ret = sns_mag_read_data( &(read_data->mag_data) );
        break;
    case SENSOR_ACC_LNR:
        ret = sns_linacc_read_data( &(read_data->acc_lnr_data) );
        break;
    case SENSOR_GRV:
        ret = sns_gravity_read_data( &(read_data->grv_data) );
        break;
    case SENSOR_GYRO_UNCAL:
        ret = sns_gyrouncalib_read_data( &(read_data->gyro_uncal_data) );
        break;
    case SENSOR_MAG_UNCAL:
        ret = sns_maguncalib_read_data( &(read_data->mag_uncal_data) );
        break;
    case SENSOR_ORTN:
        ret = sns_ori_read_data( &(read_data->ortn_data) );
        break;
    case SENSOR_ROT_VCTR:
        ret = sns_rota_read_data( &(read_data->rot_vctr_data) );
        break;
    case SENSOR_GAME_ROT_VCTR:
        ret = sns_gamerota_read_data( &(read_data->game_rot_vctr_data) );
        break;
    case SENSOR_MAG_ROT_VCTR:
        ret = sns_magrota_read_data( &(read_data->mag_rot_vctr_data) );
        break;
    case SENSOR_LIGHT:
        break;
    case SENSOR_PROX:
        break;
    case SENSOR_SGNFCNT_MTN:
        break;
    case SENSOR_STEP_CNT:
        break;
    case SENSOR_STEP_DTC:
        break;
    case SENSOR_RH:
    case SENSOR_TEMP:
    case SENSOR_TEMP_AMBIENT:
    case SENSOR_PRESSURE:
    default:
        break;
    }

    if( ret != 0 ) {
        sns_err_check();
    }
    mutex_unlock(&sensor_com_mutex);

    return ret;
}

int32_t sensor_ext_get_data(
    enum sensor_e_type type,
    union sensor_ext_read_data_u* read_data )
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        mutex_unlock(&sensor_com_mutex);
        return ret;
    }

    switch (type) {
    case SENSOR_EXT_PEDO:
        ret = sns_get_pedo_data(&(read_data->pedm_data));
        break;
    case SENSOR_EXT_VEHI:
        ret = sns_get_vehi_data(&(read_data->vehi_data));
        break;
    case SENSOR_EXT_IWIFI:
        ret = sns_get_iwifi_data(&(read_data->iwifi_data));
        break;
    case SENSOR_EXT_VH:
        ret = sns_get_vhdetect_data(&(read_data->vh_data));
        break;
    default:
        break;
    }

    if( ret != 0 ) {
        sns_err_check();
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");

    return ret;
}

int32_t sensor_ext_set_param(
    enum sensor_e_type type,
    DailysSetIWifiParam* iwifi_param,
    struct sensor_ext_pedom_param_str* pedom_param )
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        mutex_unlock(&sensor_com_mutex);
        return ret;
    }

    switch (type) {
    case SENSOR_EXT_PEDO:
        if(pedom_param){
            sns_set_pedo_param( pedom_param->weight,
                                pedom_param->step_wide,
                                pedom_param->vehi_type );

            if( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_PEDO) ){
                sns_dailys_start(true);
            }
        }
        break;
    case SENSOR_EXT_IWIFI:
        if(iwifi_param){
            ret = sns_iwifi_set_info( 0, iwifi_param );
        }
        break;
    default:
        break;
    }

    if( ret != 0 ) {
        sns_err_check();
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");

    return ret;
}

int32_t sensor_ext_clear( uint32_t clear_req )
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        mutex_unlock(&sensor_com_mutex);
        return ret;
    }

    ret = sns_pedom_clear( clear_req );

    if( ret != 0 ) {
        sns_err_check();
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");

    return ret;
}

void sensor_report_abs (
    struct input_dev* dev,
    int32_t x, int32_t y, int32_t z,
    int32_t last_x, int32_t last_y, int32_t last_z,
    int* rudder_cnt
)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("report [x,y,z]:[%d,%d,%d]",(int)x,(int)y,(int)z );

    input_report_abs(dev, ABS_X, x);
    input_report_abs(dev, ABS_Y, y);
    input_report_abs(dev, ABS_Z, z);
    if( (last_x == x) && (last_y == y) && (last_z == z) ) {
        SENSOR_A_LOG("report grv - ABS_RUDDER: cnt[%d]",(int)(*rudder_cnt) );
        input_report_abs( dev,ABS_RUDDER,((*rudder_cnt)++) );
    }

    input_sync(dev);

    SENSOR_N_LOG("end");
    return;
}

void sensor_report_ac_abs (
    struct input_dev* dev,
    int32_t x, int32_t y, int32_t z,
    int32_t last_x, int32_t last_y, int32_t last_z,
    uint8_t accuracy,
    int* rudder_cnt
)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("report [x,y,z]:[%d,%d,%d]",(int)x,(int)y,(int)z );

    input_report_abs(dev, ABS_X, x);
    input_report_abs(dev, ABS_Y, y);
    input_report_abs(dev, ABS_Z, z);
    input_report_abs(dev, ABS_STATUS, accuracy);
    if( (last_x == x) && (last_y == y) && (last_z == z) ) {
        SENSOR_A_LOG("report grv - ABS_RUDDER: cnt[%d]",(int)(*rudder_cnt) );
        input_report_abs( dev,ABS_RUDDER,((*rudder_cnt)++) );
    }

    input_sync(dev);

    SENSOR_N_LOG("end");
    return;
}

void sensor_enable(
    enum sensor_e_type type,
    struct sensor_poll_info_str* poll_info,
    bool enable
)
{
    int polltime;
    int32_t ret = 0;
    int32_t ret_apl = 0;
    enum sensor_chk_status_e_type chk_state = SENSOR_CHK_NORMAL;
    enum sensor_micon_polling_e_type poll_status = 0;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_com_mutex);
    SENSOR_N_LOG("parm: type[%d], poll_info[%d] enable[%d]",
                 (int)type, (int)poll_info, (int)enable);

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        mutex_unlock(&sensor_com_mutex);
        return;
    }

    chk_state = sensor_check_status( type, enable );
    if( SENSOR_CHK_SKIP_ALL == chk_state ){
        SENSOR_A_LOG("same status-->type[%d] sensor_type_status[0x%x]",
                      (int)type,
                      (int)sensor_type_status);
        mutex_unlock(&sensor_com_mutex);
        return;
    }

    if( true == enable ){
        SENSOR_A_LOG("set ON");
        if( SENSOR_CHK_SKIP_CTL == chk_state ) {
            SENSOR_A_LOG("Skip ctrl ret[%d]",ret);
        } else if( SENSOR_CHK_POLL_CTL == chk_state ) {
            ret = sensor_poll_on_ctrl(type);
        } else {
            ret = sensor_type_enable( type, enable );
            SENSOR_N_LOG("sensor_type_enable() ret[%d]",ret);
        }

        ret_apl = sensor_application_enable( type, enable );

        if(poll_info){
            polltime = atomic_read(&(poll_info->poll_time));
            queue_delayed_work( (poll_info->poll_wq),
                                &(poll_info->poll_work),
                                msecs_to_jiffies(polltime) );
            SENSOR_A_LOG("start delay work :polltime[%d]",(polltime));
        }
        sensor_set_status( type, SENSOR_ON);

        mutex_unlock(&sensor_com_mutex);
        sns_set_period( type, 0, 0 );
        mutex_lock(&sensor_com_mutex);

        mutex_unlock(&sensor_com_mutex);
        sns_set_app_task( type, SENSOR_ON );
        mutex_lock(&sensor_com_mutex);


    } else {
        SENSOR_A_LOG("set OFF");

        ret_apl = sensor_application_enable( type, enable );

        if( SENSOR_CHK_SKIP_CTL == chk_state ) {
            SENSOR_A_LOG("Skip ctrl ret[%d]",ret);
        } else if( SENSOR_CHK_POLL_CTL == chk_state ) {
            ret = sensor_poll_off_ctrl(type);
        } else {
            ret = sensor_type_enable( type, enable );
            SENSOR_N_LOG("sensor_type_enable() ret[%d]",ret);
        }
        if(poll_info){
            mutex_unlock(&sensor_com_mutex);
            cancel_delayed_work_sync(&(poll_info->poll_work));
            mutex_lock(&sensor_com_mutex);
            SENSOR_A_LOG("cancel delay work [%s]",(poll_info->name));
        }
        sensor_set_status( type, SENSOR_OFF );

        poll_status = sensor_get_poll_status_current();

        SENSOR_N_LOG("sensor off poll status :%d",poll_status);

        if(poll_status != SENSOR_POLLING_OFF){
            mutex_unlock(&sensor_com_mutex);
            sns_set_app_task( type, SENSOR_OFF );
            mutex_lock(&sensor_com_mutex);
        }
    }

    if(( ret != 0 )||( ret_apl != 0 )) {
        sns_err_check();
    }

    mutex_unlock(&sensor_com_mutex);

    SENSOR_N_LOG("end");
    return;
}

int sensor_set_ps_threshold( void )
{
    int ret = 0;
    SENSOR_N_LOG("start");

    ret = gp2ap_set_ioctl_ps_threshold();

    SENSOR_N_LOG("end");
    return ret;
}

void sensor_interrupt( enum sensor_e_type type, uint32_t data)
{
    SENSOR_N_LOG("start");

    switch (type) {
    case SENSOR_EXT_IWIFI:
        sensor_ext_iwifi_interrupt();
        break;
    case SENSOR_EXT_VH:
        sensor_ext_vh_interrupt();
        break;
    case SENSOR_EXT_VEHI:
        sensor_ext_vehi_interrupt();
        break;
    case SENSOR_PROX:
        prox_interrupt(data);
        break;
    case SENSOR_STEP_DTC:
        step_dtc_interrupt();
        break;
    case SENSOR_SGNFCNT_MTN:
        sgnfcnt_mtn_interrupt();
        break;
    case SENSOR_KC_MOTION_WALK_START:
        kc_motion_sensor_walk_start_interrupt();
        break;
    case SENSOR_KC_MOTION_WALK_STOP:
        kc_motion_sensor_walk_stop_interrupt();
        break;
    case SENSOR_KC_MOTION_TRAIN:
        kc_motion_sensor_train_interrupt();
        break;
    case SENSOR_KC_MOTION_VEHICLE:
        kc_motion_sensor_vehicle_interrupt();
        break;
    case SENSOR_KC_MOTION_BRINGUP:
        kc_motion_sensor_bringup_interrupt();
        break;
    case SENSOR_ACC_AUTO_CAL:
        acc_auto_cal_interrupt();
        break;
    default:
        break;
    }

    SENSOR_N_LOG("end");
    return;
}

void sensor_report_data( enum sensor_e_type type,  uint32_t data )
{
    SENSOR_N_LOG("start");

    switch (type) {
    case SENSOR_LIGHT:
        light_input_report(data);
        break;
    default:
        break;
    }

    SENSOR_N_LOG("end");
    return;
}

void sensor_set_poll_time(
    enum sensor_e_type type,
    struct sensor_poll_info_str* poll_info,
    int32_t polltime
)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("parm: polltime[%d]",(int)polltime);

    if(!poll_info){
        SENSOR_ERR_LOG("bad parm --> poll_info[%d]",(int)poll_info);
        return;
    }

    sns_set_period(type,polltime,1);

    mutex_lock(&sensor_com_mutex);

    if ( SENSOR_ON == sensor_get_status_local(type) &&
       (0 == sensor_get_batch_status())){
        SENSOR_A_LOG("cancel work -> acc_poll_work");
        mutex_unlock(&sensor_com_mutex);
        cancel_delayed_work_sync(&(poll_info->poll_work));
        mutex_lock(&sensor_com_mutex);
        atomic_set(&(poll_info->poll_time), polltime);
        queue_delayed_work( (poll_info->poll_wq),
                            &(poll_info->poll_work),
                            msecs_to_jiffies(polltime) );
        SENSOR_A_LOG("restart delay work :polltime[%d]",
                     atomic_read(&(poll_info->poll_time)) );
    } else {
        atomic_set(&(poll_info->poll_time), polltime);
        SENSOR_A_LOG("set polltime[%d]",atomic_read(&(poll_info->poll_time)));
    }

    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");
    return;
};

int sensor_input_init( struct sensor_input_info_str* info )
{
    struct input_dev *dev;
    int err;

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("in parms info[%d]",(int)info );

    if(!info){
        SENSOR_ERR_LOG("fail bad parm --> info[%d]",(int)info);
        SENSOR_ERR_LOG("end return[-1]");
        return -1;
    }
    if( (!(info->param_func)) || (!(info->attr_grp)) ){
        SENSOR_ERR_LOG("fail bad parm --> info.func[%d] info.attr_grp[%d]",
                        (int)(info->param_func), (int)(info->attr_grp));
        SENSOR_ERR_LOG("end return[-1]");
        return -1;
    }

    dev = input_allocate_device();
    if (!(dev)) {
        SENSOR_ERR_LOG("fail input_allocate_device()-->ret[%d]",(int)(dev));
        SENSOR_ERR_LOG("end return[%d]",-ENOMEM);
        return -ENOMEM;
    }

    (*(info->param_func))(dev);

    err = input_register_device(dev);
    if (err < 0) {
        SENSOR_ERR_LOG("falut input_register_device()-->ret[%d]",err);
        input_free_device(dev);

        SENSOR_ERR_LOG("end return[%d]",err);
        return err;
    }

    err = sysfs_create_group(&dev->dev.kobj, (info->attr_grp));
    if (err < 0) {
        SENSOR_ERR_LOG("fail sysfs_create_group()-->ret[%d]",err);
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return err;
    }

    (info->dev) = dev;

    SENSOR_N_LOG("end -return[0] info_dev[%d]",(int)(info->dev));
    return 0;
}

static void sensor_com_init(void)
{
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_input_init( &sensor_com_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] sensor_com_dev[%d]",
                  ret, (int)sensor_com_input_info.dev );

    if( (0 != ret) || (NULL == (sensor_com_input_info.dev)) ) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    mutex_init(&sensor_com_mutex);
    mutex_init(&sensor_batch_mutex);

    SENSOR_N_LOG("end");
    return;
}

static void sensor_suspend_ctl( void )
{
    int i;
    uint32_t state_micon;
    uint32_t state_not_off;
    uint32_t type;
    uint32_t state_app;
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    for(i=0; i<sensor_poll_ctl_tbl_size; i++){
        if( SENSOR_ON == sensor_get_status_local(sensor_poll_ctl_tbl[i].type)){
            if( NULL != sensor_poll_ctl_tbl[i].poll_info_p){
               mutex_unlock(&sensor_com_mutex);
               cancel_delayed_work_sync(&(sensor_poll_ctl_tbl[i].poll_info_p->poll_work));
               mutex_lock(&sensor_com_mutex);
            }
        }
    }

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            state_app = (SENSOR_ON << type);
            SENSOR_N_LOG("SENSOR_ON state_app:%x",state_app);
            if((state_app & SENSOR_NOT_OFF_SUSPEND_TYPE) == 0){
                ret = sensor_application_enable( type, false );
                SENSOR_N_LOG("SUSPEND OFF state_app:%x",state_app);
            }
        }
    }

    state_micon = sensor_type_status & (~SENSOR_NOT_MICON_TYPE);
    state_not_off = state_micon &  (SENSOR_NOT_OFF_SUSPEND_TYPE);
    if( state_micon && (0 == state_not_off) ){
       ret = sensor_type_enable( SENSOR_ACC, false );
    }

    if( SENSOR_ON == sensor_get_status_local(SENSOR_LIGHT) ){
       ret = sensor_type_enable( SENSOR_LIGHT, false );
    }

    if(( SENSOR_ON == sensor_get_status_local(SENSOR_SGNFCNT_MTN) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_VEHI) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_IWIFI) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_STEP_CNT) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_STEP_DTC) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_WALK_START) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_WALK_STOP) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_TRAIN) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_VEHICLE) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_BRINGUP) ) ) {
        sns_enable_irq_wake_irq(true);
    }

    SENSOR_N_LOG("end");
    return;
}

static void sensor_resume_ctl( void )
{
    int i;
    uint32_t type;
    uint32_t state_micon;
    uint32_t state_app;
    int polltime;
    int32_t ret = 0;
    uint32_t state_batch;
    uint32_t batch_onoff;

    SENSOR_N_LOG("start");

    if(( SENSOR_ON == sensor_get_status_local(SENSOR_SGNFCNT_MTN) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_VEHI) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_IWIFI) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_STEP_CNT) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_STEP_DTC) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_WALK_START) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_WALK_STOP) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_TRAIN) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_VEHICLE) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_BRINGUP) ) ) {
        sns_enable_irq_wake_irq(false);
    }

    state_micon = sensor_type_status & (~SENSOR_NOT_MICON_TYPE);
    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
           if( 0 == ((SENSOR_ON << type) & SENSOR_NOT_OFF_SUSPEND_TYPE) ){
               ret = sensor_type_enable( type, true );
           }
        }
    }

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            state_app = (SENSOR_ON << type);
            SENSOR_N_LOG("SENSOR_ON state_app:%x",state_app);
            if((state_app & SENSOR_NOT_OFF_SUSPEND_TYPE) == 0){
                ret = sensor_application_enable( type, true );
                SENSOR_N_LOG("RESUME ON state_app:%x",state_app);
            }
        }
    }

    for(i=0; i<sensor_poll_ctl_tbl_size; i++){
        state_batch = sensor_get_batch_status();
        batch_onoff = ( (state_batch & (1 << sensor_poll_ctl_tbl[i].type)) >> sensor_poll_ctl_tbl[i].type );
        if( (SENSOR_ON == sensor_get_status_local(sensor_poll_ctl_tbl[i].type)) && 
            (BATCH_OFF == batch_onoff)){
            if( NULL != sensor_poll_ctl_tbl[i].poll_info_p){
                polltime = atomic_read(&(sensor_poll_ctl_tbl[i].poll_info_p->poll_time));
                queue_delayed_work( (sensor_poll_ctl_tbl[i].poll_info_p->poll_wq),
                                    &(sensor_poll_ctl_tbl[i].poll_info_p->poll_work),
                                    msecs_to_jiffies(polltime) );
            }
        }
    }

    SENSOR_N_LOG("end");
    return;
}

static void sensor_shutdown_ctl( void )
{
    int i;
    uint32_t type;
    int32_t ret = 0;
    struct sensor_batch_info_str batch_info = {0,0,0};

    SENSOR_N_LOG("start");

    for(i=0; i<sensor_poll_ctl_tbl_size; i++){
        if( SENSOR_ON == sensor_get_status_local(sensor_poll_ctl_tbl[i].type)){
            if( NULL != sensor_poll_ctl_tbl[i].poll_info_p){
               mutex_unlock(&sensor_com_mutex);
               cancel_delayed_work_sync(&(sensor_poll_ctl_tbl[i].poll_info_p->poll_work));
               mutex_lock(&sensor_com_mutex);
            }
        }
    }

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            ret = sensor_application_enable( type, false );
        }
    }

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            ret = sensor_type_enable( type, false );
        }
    }
    sensor_type_status = 0;

    for( type=0; type<SENSOR_MAX; type++){
        if( sensor_batch_status & (BATCH_ON << type) ) {
            sns_logging_state( type, batch_info.timeout, batch_info.period_ns);
        }
    }
    sensor_batch_status = 0;

    SENSOR_N_LOG("end");
    return;
}

static void sensor_reset_ctl( void )
{
    uint32_t type;
    int32_t ret = 0;

    SENSOR_ERR_LOG("start");

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            ret = sensor_application_enable( type, false );
        }
    }

    if( SENSOR_ON == sensor_get_status_local(SENSOR_LIGHT) ){
       ret = sensor_type_enable( SENSOR_LIGHT, false );
    }
    if( SENSOR_ON == sensor_get_status_local(SENSOR_PROX) ){
       ret = sensor_type_enable( SENSOR_PROX, false );
    }

    SENSOR_ERR_LOG("end");
    return;
}

static void sensor_reset_resume_ctl( void )
{
    uint32_t type;
    int32_t ret = 0;

    SENSOR_ERR_LOG("start");
    sensor_before_poll_status = sensor_poll_status;
    sensor_poll_status = SENSOR_POLLING_OFF;

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            if ( (SENSOR_ON<<type) & (~SENSOR_NOT_MICON_TYPE) ) {
                ret = sensor_poll_on_ctrl(type);
            } else {
                ret = sensor_type_enable( type, true );
            }
        }
    }

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            ret = sensor_application_enable( type, true );
        }
    }

    SENSOR_ERR_LOG("end");
    return;
}

void sensor_suspend( void )
{
    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    sensor_suspend_ctl();

    sensor_drv_set_status_local( SENSOR_SUSPEND );
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");
    return;
}

void sensor_resume( void )
{
    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    sensor_resume_ctl();

    sensor_drv_set_status_local( SENSOR_RESUME );
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");
    return;
}

void sensor_shutdown( void )
{
    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    sensor_shutdown_ctl();

    sensor_drv_set_status_local( SENSOR_SHUTDOWN );
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");
    return;
}

void sensor_reset( void )
{
    SENSOR_N_LOG("start");

    sensor_reset_ctl();

    sensor_drv_set_status_local( SENSOR_RESET );
    SENSOR_N_LOG("end");
    return;
}

void sensor_reset_resume( void )
{
    SENSOR_N_LOG("start");

    sensor_reset_resume_ctl();

    sensor_drv_set_status_local( SENSOR_RESUME );
    SENSOR_N_LOG("end");
    return;
}

static int32_t __init sensor_init(void)
{
    SENSOR_N_LOG("start");

    sensor_micon_init();
    gp2ap_init();

    sensor_com_init();

    acc_driver_init();
    acc_lnr_driver_init();
    gyro_driver_init();
    gyro_uncal_driver_init();
    grv_driver_init();
    mag_driver_init();
    mag_uncal_driver_init();
    relative_humidity_driver_init();
    rot_vctr_driver_init();
    game_rot_vctr_driver_init();
    mag_rot_vctr_driver_init();
    ortn_driver_init();
    pressure_driver_init();
    temperature_driver_init();
    temperature_ambient_driver_init();
    sgnfcnt_mtn_driver_init();
    step_cnt_driver_init();
    step_dtc_driver_init();
    sensor_ext_driver_init();
    light_driver_init();
    prox_driver_init();
    sensor_ext_vh_driver_init();
    kc_motion_sensor_driver_init();

    sensor_drv_set_status_local( SENSOR_RESUME );

    SENSOR_N_LOG("end");

    return 0;

}

static void __exit sensor_exit(void)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
}

module_init(sensor_init);
module_exit(sensor_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("Sensor Common");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sensor_driver.c");

