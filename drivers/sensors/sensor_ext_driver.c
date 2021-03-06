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
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include "sensor_driver.h"
#include "sensor_com.h"


#define SENSOR_EXT_FUNC_PEDO     0x01
#define SENSOR_EXT_FUNC_VEHI     0x02
#define SENSOR_EXT_FUNC_IWIFI     0x10

#define EXT_FUNC_VEHI_KIND      0x01
#define EXT_FUNC_IWIFI_KIND     0x10

static atomic_t sns_ext_vehi_interrupt_kind;
static atomic_t sns_ext_vehi_interrupt_flag;
static atomic_t sns_ext_iwifi_interrupt_kind;
static atomic_t sns_ext_iwifi_interrupt_flag;

static ssize_t sensor_ext_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t sensor_ext_pedom_param_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_pedom_param_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t sensor_ext_iwifi_param_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_iwifi_param_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t sensor_ext_pedom_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_vehi_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_iwifi_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_clear_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t sensor_ext_rt_state_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static int32_t sensor_ext_open( struct inode* inode, struct file* filp );
static int32_t sensor_ext_release( struct inode* inode, struct file* filp );
static unsigned int sensor_ext_pedom_poll(struct file *fp, poll_table *wait);
static unsigned int sensor_ext_iwifi_poll(struct file *fp, poll_table *wait);
static unsigned int sensor_ext_vehi_poll(struct file *fp, poll_table *wait);

static void sensor_ext_set_input_params( struct input_dev *dev );


static DEVICE_ATTR(dailys_enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_ext_enable_show,
    sensor_ext_enable_store
);
static DEVICE_ATTR(dailys_param,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_ext_pedom_param_show,
    sensor_ext_pedom_param_store
);
static DEVICE_ATTR(dailys_iwifi_param,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_ext_iwifi_param_show,
    sensor_ext_iwifi_param_store
);
static DEVICE_ATTR(dailys_pedo_data,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_ext_pedom_data_show,
    NULL
);
static DEVICE_ATTR(dailys_vehi_data,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_ext_vehi_data_show,
    NULL
);
static DEVICE_ATTR(dailys_iwifi_data,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_ext_iwifi_data_show,
    NULL
);
static DEVICE_ATTR(dailys_clear,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    NULL,
    sensor_ext_clear_store
);
static DEVICE_ATTR(dailys_rt_state,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_ext_rt_state_show,
    NULL
);

static struct attribute *sensor_ext_attributes[] = {
    &dev_attr_dailys_enable.attr,
    &dev_attr_dailys_param.attr,
    &dev_attr_dailys_iwifi_param.attr,
    &dev_attr_dailys_pedo_data.attr,
    &dev_attr_dailys_vehi_data.attr,
    &dev_attr_dailys_iwifi_data.attr,
    &dev_attr_dailys_clear.attr,
    &dev_attr_dailys_rt_state.attr,
    NULL
};

static struct attribute_group sensor_ext_attr_grp = {
    .attrs = sensor_ext_attributes
};

struct sensor_input_info_str sensor_ext_input_info =
{
    NULL,
    sensor_ext_set_input_params,
    &sensor_ext_attr_grp,
};

static struct file_operations sensor_ext_pedom_fops = {
  .owner   = THIS_MODULE,
  .open    = sensor_ext_open,
  .release = sensor_ext_release,
  .poll    = sensor_ext_pedom_poll,
};
static struct miscdevice sensor_ext_pedom_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "dailys_io_pedom",
  .fops  = &sensor_ext_pedom_fops,
};

static struct file_operations sensor_ext_vehi_fops = {
  .owner   = THIS_MODULE,
  .open    = sensor_ext_open,
  .release = sensor_ext_release,
  .poll    = sensor_ext_vehi_poll,
};
static struct miscdevice sensor_ext_vehi_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "dailys_io_vehicle",
  .fops  = &sensor_ext_vehi_fops,
};

static struct file_operations sensor_ext_iwifi_fops = {
  .owner   = THIS_MODULE,
  .open    = sensor_ext_open,
  .release = sensor_ext_release,
  .poll    = sensor_ext_iwifi_poll,
};
static struct miscdevice sensor_ext_iwifi_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "dailys_io_iwifi",
  .fops  = &sensor_ext_iwifi_fops,
};

static wait_queue_head_t sensor_ext_iwifi_p_queue;
static wait_queue_head_t sensor_ext_vehi_p_queue;
static DailysSetIWifiParam sensor_ext_iwifi_param;
static struct sensor_ext_pedom_param_str  sensor_ext_pedom_param;

static ssize_t sensor_ext_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int enable = 0;
    SENSOR_N_LOG("start");

    if( SENSOR_ON == sensor_get_status(SENSOR_EXT_PEDO)) {
        enable |= SENSOR_EXT_FUNC_PEDO;
    }
    if( SENSOR_ON == sensor_get_status(SENSOR_EXT_VEHI)) {
        enable |= SENSOR_EXT_FUNC_VEHI;
    }
    if( SENSOR_ON == sensor_get_status(SENSOR_EXT_IWIFI)) {
        enable |= SENSOR_EXT_FUNC_IWIFI;
    }

    SENSOR_N_LOG("end");
    return sprintf(buf, "%x\n",enable);
}

static ssize_t sensor_ext_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t enable = 0;
    int32_t type = 0;
    SENSOR_N_LOG("start");

    sscanf(buf, "%x %x", &type, &enable);
    SENSOR_N_LOG("type[%d] enable[%d]",(int)type, (int)enable);

    if( (type & SENSOR_EXT_FUNC_PEDO) == SENSOR_EXT_FUNC_PEDO ){
        sensor_enable( SENSOR_EXT_PEDO, NULL, (bool)enable );
    }

    if ((type & SENSOR_EXT_FUNC_VEHI) == SENSOR_EXT_FUNC_VEHI ){
        sensor_enable( SENSOR_EXT_VEHI, NULL, (bool)enable );
    }

    if ((type & SENSOR_EXT_FUNC_IWIFI) == SENSOR_EXT_FUNC_IWIFI ){
        sensor_enable( SENSOR_EXT_IWIFI, NULL, (bool)enable );
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t sensor_ext_pedom_param_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret = 0;

    SENSOR_N_LOG("start");

    ret = sprintf( buf, "%d %d %d\n",
                    sensor_ext_pedom_param.step_wide,
                    sensor_ext_pedom_param.weight,
                    sensor_ext_pedom_param.vehi_type );

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t sensor_ext_pedom_param_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    sscanf( buf, "%d %d %d",
            &sensor_ext_pedom_param.step_wide,
            &sensor_ext_pedom_param.weight,
            &sensor_ext_pedom_param.vehi_type );

    ret = sensor_ext_set_param(SENSOR_EXT_PEDO,NULL,&sensor_ext_pedom_param);
    if(ret < 0 ){
        SENSOR_ERR_LOG("sensor_ext_set_param --> ret[%d]",(int)ret);
    }


    SENSOR_N_LOG("end");
    return count;
}

static ssize_t sensor_ext_iwifi_param_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret = 0;

    SENSOR_N_LOG("start");
    ret = sprintf( buf, "%d %d %d %d\n",
                    sensor_ext_iwifi_param.m_nPedoStartStep,
                    sensor_ext_iwifi_param.m_nPedoEndTime,
                    sensor_ext_iwifi_param.m_nVehiStartTime,
                    sensor_ext_iwifi_param.m_nVehiEndTime );

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t sensor_ext_iwifi_param_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    sscanf( buf, "%d %d %d %d",
            &sensor_ext_iwifi_param.m_nPedoStartStep,
            &sensor_ext_iwifi_param.m_nPedoEndTime,
            &sensor_ext_iwifi_param.m_nVehiStartTime,
            &sensor_ext_iwifi_param.m_nVehiEndTime );

    ret = sensor_ext_set_param(SENSOR_EXT_IWIFI,&sensor_ext_iwifi_param,NULL);
    if(ret < 0 ){
        SENSOR_ERR_LOG("sensor_ext_set_param --> ret[%d]",(int)ret);
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t sensor_ext_pedom_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int get_ret = 0;
    ssize_t ret = 0;
    union sensor_ext_read_data_u read_data;

    SENSOR_N_LOG("start");
    memset(&read_data, 0, sizeof(read_data));

    if(sns_get_reset_status() == false){
        get_ret = sensor_ext_get_data( SENSOR_EXT_PEDO, &read_data );
    }
    if (  0 == get_ret ) {
        ret = sprintf( buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                       read_data.pedm_data.usStepCnt,
                       read_data.pedm_data.usWalkTime,
                       read_data.pedm_data.usCal,
                       read_data.pedm_data.usBodyFat,
                       read_data.pedm_data.usExercise,
                       read_data.pedm_data.usMets,
                       read_data.pedm_data.usSpeed,
                       read_data.pedm_data.usRunStatus,
                       read_data.pedm_data.usRunStepCnt,
                       read_data.pedm_data.usRunTime,
                       read_data.pedm_data.usStExercise,
                       read_data.pedm_data.usStCal,
                       read_data.pedm_data.usStBodyFat,
                       read_data.pedm_data.usSportExercise,
                       read_data.pedm_data.usSyncRunStepCnt,
                       read_data.pedm_data.usSyncRunTime,
                       read_data.pedm_data.usSyncRunCal,
                       read_data.pedm_data.usRunCal,
                       read_data.pedm_data.usRunExercise,
                       read_data.pedm_data.usPS_InitBaseHeight,
                       read_data.pedm_data.usPS_ActHeight,
                       read_data.pedm_data.usPS_ActHeightMin,
                       read_data.pedm_data.usPS_ActHeightAve,
                       read_data.pedm_data.usPS_ActHeightMax );
    }

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t sensor_ext_vehi_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int get_ret = 0;
    ssize_t ret = 0;
    union sensor_ext_read_data_u read_data;

    SENSOR_N_LOG("start");
    memset(&read_data, 0, sizeof(read_data));

    if(sns_get_reset_status() == false){
        get_ret = sensor_ext_get_data( SENSOR_EXT_VEHI, &read_data );
    }
    if (  0 == get_ret ) {
        ret = sprintf( buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                       read_data.vehi_data.usVehiStatus,
                       read_data.vehi_data.usVehiKind,
                       read_data.vehi_data.usVehiDetectTime,
                       read_data.vehi_data.usVehiRideTime,
                       read_data.vehi_data.usVehiRideCal,
                       read_data.vehi_data.usVehiBodyFat,
                       read_data.vehi_data.usVehiExercise,
                       read_data.vehi_data.usVehiMets,
                       read_data.vehi_data.usVehiStExercise,
                       read_data.vehi_data.usVehiStRideCal,
                       read_data.vehi_data.usVehiStBodyFat,
                       read_data.vehi_data.usVehiBiExercise,
                       read_data.vehi_data.usVehiBiRideCal,
                       read_data.vehi_data.usVehiBiBodyFat,
                       read_data.vehi_data.usVehiSportExercise );
    }

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t sensor_ext_iwifi_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int get_ret = 0;
    ssize_t ret = 0;
    union sensor_ext_read_data_u read_data;

    SENSOR_N_LOG("start");
    memset(&read_data, 0, sizeof(read_data));

    if(sns_get_reset_status() == false){
        get_ret = sensor_ext_get_data( SENSOR_EXT_IWIFI, &read_data );
    }
    if (  0 == get_ret ) {
        ret = sprintf( buf, "%d %d\n",
                       read_data.iwifi_data.usPedoStatus,
                       read_data.iwifi_data.usVehiStatus );
    }

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t sensor_ext_clear_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    uint32_t clear_req;
    int32_t ret;

    SENSOR_N_LOG("start");
    sscanf(buf, "%x", &clear_req);

    ret = sensor_ext_clear(clear_req);

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t sensor_ext_rt_state_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int get_ret = 0;
    ssize_t ret = 0;
    union sensor_ext_read_data_u read_data;

    SENSOR_N_LOG("start");
    memset(&read_data, 0, sizeof(read_data));

    get_ret = sensor_ext_get_data( SENSOR_EXT_PEDO, &read_data );
    if (  0 == get_ret ) {
        ret = sprintf( buf, "%d \n", read_data.pedm_data.usRTState );
    }

    SENSOR_N_LOG("end");
    return ret;
}

static int32_t sensor_ext_open( struct inode* inode, struct file* filp )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static int32_t sensor_ext_release( struct inode* inode, struct file* filp )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static unsigned int sensor_ext_pedom_poll(struct file *fp, poll_table *wait)
{
    unsigned int ret = 0;
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return ret;
}

static unsigned int sensor_ext_vehi_poll(struct file *fp, poll_table *wait)
{
    unsigned int ret = 0;
    unsigned int int_flag = 0;
    SENSOR_N_LOG("start");

    poll_wait(fp, &sensor_ext_vehi_p_queue, wait);
    int_flag = atomic_read(&sns_ext_vehi_interrupt_flag);
    if(int_flag)
    {
        ret = POLLIN | POLLPRI;
        atomic_set(&sns_ext_vehi_interrupt_kind, 0);
        atomic_set(&sns_ext_vehi_interrupt_flag, 0);
    }

    SENSOR_N_LOG("end");
    return ret;
}

static unsigned int sensor_ext_iwifi_poll(struct file *fp, poll_table *wait)
{
    unsigned int ret = 0;
    unsigned int int_flag = 0;
    SENSOR_N_LOG("start");

    poll_wait(fp, &sensor_ext_iwifi_p_queue, wait);
    int_flag = atomic_read(&sns_ext_iwifi_interrupt_flag);
    if(int_flag)
    {
        ret = POLLIN | POLLPRI;
        atomic_set(&sns_ext_iwifi_interrupt_kind, 0);
        atomic_set(&sns_ext_iwifi_interrupt_flag, 0);
    }

    SENSOR_N_LOG("end");
    return ret;
}

static void sensor_ext_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev[%d]",(int)dev);
        return;
    }

    dev->name = "sensor_ext";
    dev->id.bustype = BUS_SPI;

    SENSOR_N_LOG("end");
    return;
}

void sensor_ext_iwifi_interrupt( void )
{
    uint32_t tmp_kind = 0;

    SENSOR_N_LOG("start");
    if ( SENSOR_ON == sensor_get_status(SENSOR_EXT_IWIFI) ) {
        tmp_kind = atomic_read(&sns_ext_iwifi_interrupt_kind);
        tmp_kind |= EXT_FUNC_IWIFI_KIND;
        atomic_set(&sns_ext_iwifi_interrupt_kind,tmp_kind);
        atomic_set(&sns_ext_iwifi_interrupt_flag,1);
        SENSOR_A_LOG("wake up IWIFI");
        wake_up_interruptible(&sensor_ext_iwifi_p_queue);
    }
    SENSOR_N_LOG("end");
    return;
}

void sensor_ext_vehi_interrupt( void )
{
    uint32_t tmp_kind = 0;

    SENSOR_N_LOG("start");
    if ( SENSOR_ON == sensor_get_status(SENSOR_EXT_VEHI) ) {
        tmp_kind = atomic_read(&sns_ext_vehi_interrupt_kind);
        tmp_kind |= EXT_FUNC_VEHI_KIND;
        atomic_set(&sns_ext_vehi_interrupt_kind,tmp_kind);
        atomic_set(&sns_ext_vehi_interrupt_flag,1);
        SENSOR_A_LOG("wake up VEHI");
        wake_up_interruptible(&sensor_ext_vehi_p_queue);
    }
    SENSOR_N_LOG("end");
    return;
}

void sensor_ext_driver_init(void)
{
    int32_t ret;

    SENSOR_N_LOG("start");

    atomic_set(&sns_ext_vehi_interrupt_kind,0);
    atomic_set(&sns_ext_vehi_interrupt_flag,0);
    atomic_set(&sns_ext_iwifi_interrupt_kind,0);
    atomic_set(&sns_ext_iwifi_interrupt_flag,0);

    ret = sensor_input_init( &sensor_ext_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%d]",
                  ret, (int)(sensor_ext_input_info.dev) );

    if( (0 != ret) || (NULL == (sensor_ext_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        return;
    }

    ret = misc_register(&sensor_ext_pedom_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register():pedom-->ret[%d]",(int)ret);
        return;
    }

    ret = misc_register(&sensor_ext_vehi_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register():vehicle-->ret[%d]",(int)ret);
        return;
    }

    ret = misc_register(&sensor_ext_iwifi_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register():iwifi-->ret[%d]",(int)ret);
        return;
    }

    init_waitqueue_head(&sensor_ext_iwifi_p_queue);
    init_waitqueue_head(&sensor_ext_vehi_p_queue);

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(sensor_ext_driver_init);

