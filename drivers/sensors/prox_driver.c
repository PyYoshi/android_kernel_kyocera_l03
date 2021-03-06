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
#include <linux/ioctl.h>
#include "sensor_driver.h"
#include "sensor_micon_driver.h"
#include "sensor_com.h"

#define DEVICE_FILE_NAME    "ps_sensor"

#define D_NV_DATA_MAX                           (0x55)
#define GP2AP_DEV_STATUS_SUSPEND_INT            0x00000002

typedef struct _t_ps_ioctl_detection
{
    unsigned long ulps_detection;
}T_PS_IOCTL_DETECTION;

typedef struct _t_ps_ioctl_nv
{
    unsigned long ulLength;
    unsigned char ucData[D_NV_DATA_MAX];
    unsigned long ulItem;
}T_PS_IOCTL_NV;

typedef struct _t_ps_ioctl_report_thresh
{
    uint8_t set;
}T_PS_IOCTL_THRESHOLD;

#define PS_IO             'A'
#define IOCTL_PS_DETECTION_GET        _IOR(PS_IO, 0x01, T_PS_IOCTL_DETECTION)
#define IOCTL_PS_THRESHOLD_SET        _IOW(PS_IO, 0x02, T_PS_IOCTL_THRESHOLD)
#define IOCTL_PS_NV_DATA_SET          _IOW(PS_IO, 0x03, T_PS_IOCTL_NV)
#define IOCTL_PS_NV_DATA_GET          _IOR(PS_IO, 0x04, T_PS_IOCTL_NV)

static int prox_open(struct inode *inode_type, struct file *file);
static int prox_release(struct inode *inode_type, struct file *file);
static long prox_ioctl(struct file *file_type,
    unsigned int unCmd,
    unsigned long ulArg );

static ssize_t prox_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t prox_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static void prox_set_input_params( struct input_dev *dev );

static DEVICE_ATTR( enable_ps_sensor,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    prox_enable_show,
    prox_enable_store
);

static struct attribute *prox_attributes[] = {
    &dev_attr_enable_ps_sensor.attr,
    NULL
};

static struct attribute_group prox_attr_grp = {
    .attrs = prox_attributes
};

struct sensor_input_info_str prox_input_info =
{
    NULL,
    prox_set_input_params,
    &prox_attr_grp,
};

static struct file_operations prox_fops = {
    .owner = THIS_MODULE,
    .open = prox_open,
    .release = prox_release,
    .unlocked_ioctl = prox_ioctl,
};

static struct miscdevice prox_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_FILE_NAME,
    .fops = &prox_fops,
};

static uint32_t prox_last_read_data = 0;

static int prox_open(struct inode *inode_type, struct file *file)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static int prox_release(struct inode *inode_type, struct file *file)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}


static long prox_ioctl(struct file *file_type,
    unsigned int unCmd,
    unsigned long ulArg )
{
    s32 nRet = -EINVAL;
    T_PS_IOCTL_DETECTION ps_detection_type;
    T_PS_IOCTL_NV sensor_nv_type;

    SENSOR_N_LOG("start");

    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return -EFAULT;
    }

    memset((void*)&ps_detection_type, 0,
                        sizeof(T_PS_IOCTL_DETECTION));

   switch (unCmd)
   {
        case IOCTL_PS_DETECTION_GET:
            SENSOR_A_LOG("case IOCTL_PS_DETECTION_GET");
            nRet = copy_from_user(&ps_detection_type, 
                    (void __user *)ulArg, sizeof(T_PS_IOCTL_DETECTION));
            if (nRet)
            {
                return -EFAULT;
            }
            ps_detection_type.ulps_detection = prox_last_read_data;
            nRet = copy_to_user((void *)(ulArg),
                     &ps_detection_type, sizeof(T_PS_IOCTL_DETECTION));
            if (nRet)
            {
                return -EFAULT;
            }
            break;
        case IOCTL_PS_NV_DATA_SET:
            SENSOR_A_LOG("case IOCTL_PS_NV_DATA_SET");
            memset((void*)&sensor_nv_type, 0,sizeof(T_PS_IOCTL_NV));
            nRet = copy_from_user( &sensor_nv_type,
                                   (void __user *)ulArg,
                                   sizeof(T_PS_IOCTL_NV) );
            if (!nRet){
                gp2ap_set_ioctl_sensor_nv((unsigned long)&sensor_nv_type);
                nRet = 0;
            }
            break;
        case IOCTL_PS_THRESHOLD_SET:
            SENSOR_A_LOG("case IOCTL_PS_THRESHOLD_SET");
            sensor_set_ps_threshold();
            break;
        default:
            SENSOR_A_LOG("default");
            break;
    }

    SENSOR_N_LOG("end");
    return nRet;
}

static ssize_t prox_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");
    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    enable = sensor_get_status(SENSOR_PROX);
    SENSOR_N_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t prox_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = simple_strtoul(buf, NULL, 10);
    SENSOR_N_LOG("start");

    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

    sensor_enable(SENSOR_PROX, NULL ,(bool)enable);

    SENSOR_N_LOG("end");
    return count;
}

void prox_interrupt(uint32_t data)
{
    SENSOR_N_LOG("start");

    input_report_abs(prox_input_info.dev, ABS_DISTANCE, data);
    input_sync(prox_input_info.dev);

    prox_last_read_data = data;

    SENSOR_N_LOG("end");
    return;
}

static void prox_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev[%d]",(int)dev);
        return;
    }

    dev->name = "proximity";
    dev->id.bustype = BUS_I2C;

    set_bit(EV_ABS, dev->evbit);
    input_set_capability(dev, EV_ABS, ABS_DISTANCE);
    input_set_abs_params(dev, ABS_DISTANCE, 0, 1, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

void prox_driver_init(void)
{
    int ret = 0;
    SENSOR_N_LOG("start");

    ret = sensor_input_init( &prox_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%d]",
                  ret, (int)(prox_input_info.dev) );

    if( (0 != ret) || (NULL == (prox_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    ret = misc_register(&prox_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register()-->ret[%d]",(int)ret);
        return;
    }

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(prox_driver_init);
