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
#include "sensor_micon_driver.h"
#include "sensor_com.h"
#include <linux/spi/spi.h>

#include <linux/sensor_power.h>

#define SENSOR_MICON_DRIVER_NAME    "sensor_micon"
#define HC_MCU_GET_VERSION           (0x0001u)
#define HC_MCU_GET_EX_SENSOR         (0x0003u)
#define HC_MCU_SET_PCON              (0x0004u)
#define HC_MCU_GET_INT_DETAIL        (0x0006u)
#define HC_MCU_SELF_CHK_FW           (0x000Au)
#define HC_MCU_SENSOR_INIT           (0x000Bu)
#define HC_MCU_SET_PERI              (0x000Du)
#define HC_MCU_EXEC_TASK             (0x000Fu)
#define HC_MCU_SET_TASK              (0x0011u)
#define HC_MCU_SET_PDIR              (0x0013u)
#define HC_MCU_FUP_START             (0x0101u)
#define HC_MCU_FUP_ERASE             (0x0102u)
#define HC_MCU_FUP_END               (0x0104u)
#define HC_MCU_FUP_WRITE_FIFO        (0x0105u)
#define HC_ACC_SET_CALIB             (0x1009u)
#define HC_ACC_GET_CALIB             (0x100au)
#define HC_ACC_SET_CONV_AXIS         (0x105Eu)
#define HC_ACC_SET_PARAM             (0x1078u)
#define HC_DST_SET_DAILYS            (0x1080u)
#define HC_DST_CLR_DAILYS            (0x1082u)
#define HC_DST_CLR_INT_DETAIL        (0x1083u)
#define HC_DST_GET_INT_DETAIL        (0x1084u)
#define HC_DST_GET_PEDO1             (0x1085u)
#define HC_DST_GET_PEDO2             (0x1086u)
#define HC_DST_GET_RUN1              (0x1087u)
#define HC_DST_GET_TRANS1            (0x1088u)
#define HC_DST_GET_TRANS2            (0x1089u)
#define HC_DST_GET_INTELLI_WIFI      (0x108Au)
#define HC_DST_SET_IWIFI_INFO        (0x108Bu)
#define HC_DST_GET_RUN2              (0x1092u)
#define HC_DST_GET_PEDO5             (0x1093u)
#define HC_DST_EXEC_IWIFI            (0x1094u)
#define HC_MOT_EXEC_WALK_STOP        (0x1096u)
#define HC_MOT_EXEC_TRAIN            (0x1097u)
#define HC_MOT_GET_TRAIN_INFO        (0x1098u)
#define HC_DST_GET_TRANS6            (0x1099u)
#define HC_MOT_EXEC_WALK_START       (0x109Au)
#define HC_MOT_EXEC_VEHICLE          (0x109Bu)
#define HC_MOT_EXEC_BRINGUP          (0x10B3u)
#define HC_MOT_GET_BRINGUP_INFO      (0x10B4u)
#define HC_DST_EXEC_VH               (0x10B5u)
#define HC_DST_GET_VH                (0x10B6u)
#define HC_DST_SET_ANDROID_ENABLE    (0x10C0u)
#define HC_DST_SET_ANDROID_PARAM     (0x10C2u)
#define HC_DST_EXEC_MOTION           (0x10C3u)
#define HC_AUTO_CALIB_SET_DATA       (0x1e00u)
#define HC_AUTO_CALIB_GET_DATA       (0x1e01u)
#define HC_MAG_SET_DATA              (0x7005u)
#define HC_MAG_GET_DATA              (0x7006u)
#define HC_MAG_SET_CALIB             (0x7007u)
#define HC_EC_SET_CONV_AXIS          (0x7009u)
#define HC_MAG_SET_FILTER            (0x700Bu)
#define HC_MAG_SET_STATIC_MATRIX     (0x700Du)
#define HC_MAG_SET_DISPERTION_THRESH (0x700Fu)
#define HC_MAG_SET_OFFSET            (0x70F0u)
#define HC_MAG_GET_OFFSET            (0x70F1u)
#define HC_GYRO_SET_DATA             (0x8002u)
#define HC_GYRO_GET_DATA             (0x8003u)
#define HC_GYRO_SET_CALIB            (0x8004u)
#define HC_GYRO_SET_CONV_AXIS        (0x8009u)
#define HC_MUL_SET_LOG_PARAM_SENSOR  (0xF001u)
#define HC_MUL_GET_LOG_PARAM_SENSOR  (0xF002u)
#define HC_MUL_SET_LOG_DELAY_SENSOR  (0xF003u)
#define HC_MUL_SET_LOG_PARAM_FUSION  (0xF005u)
#define HC_MUL_SET_LOG_DELAY_FUSION  (0xF007u)
#define HC_MUL_GET_LOGGING_DATA      (0xF009u)
#define HC_MUL_GET_LOG_PUNCTUATION   (0xF00Au)
#define HC_MUL_SET_ANDROID           (0xF00Bu)
#define HC_MUL_GET_ANDROID           (0xF00Cu)
#define HC_MUL_SET_ANDROID_PERIOD    (0xF00Du)
#define HC_MUL_SET_FUSION            (0xF015u)
#define HC_MUL_SET_FUSION_STATUS     (0xF018u)
#define HC_MUL_GET_FUSION_STATUS     (0xF019u)

#define HC_MAG_SET_BUFF_INT          (0xF01Cu)

#define KC_LOG_READ                  (0x1F01u)
#define KC_LOG_SIZE                  (2 + 256)


#define DAILYS_CLEAR_PEDO_DATA     0x00001
#define DAILYS_CLEAR_PEDO_STATE    0x00002
#define DAILYS_CLEAR_VEHI_DATA     0x00004
#define DAILYS_CLEAR_VEHI_STATE    0x00008
#define DAILYS_CLEAR_WIFI_STATE    0x00010
#define DAILYS_CLEAR_HEIGHT_STATE  0x00020
#define DAILYS_CLEAR_HEIGHT_ALL    0x00040
#define DAILYS_CLEAR_RT_STATE      0x00080
#define DAILYS_CLEAR_SHOCK_STATE   0x00100
#define DAILYS_CLEAR_STOP_STATE    0x00200
#define DAILYS_CLEAR_OUT_STATE     0x00400
#define DAILYS_CLEAR_TRAIN_STATE   0x00800
#define DAILYS_CLEAR_MOTION_STATE  0x01000
#define DAILYS_CLEAR_STEP_COUNTER  0x02000
#define DAILYS_CLEAR_STEP_TIMESTMP 0x04000
#define DAILYS_CLEAR_BATCH_TIMER   0x08000
#define DAILYS_CLEAR_VH_STATE      0x10000
#define    DAILYS_CLEAR_ALL (DAILYS_CLEAR_PEDO_DATA | DAILYS_CLEAR_PEDO_STATE | DAILYS_CLEAR_VEHI_DATA | DAILYS_CLEAR_VEHI_STATE | DAILYS_CLEAR_WIFI_STATE | DAILYS_CLEAR_HEIGHT_ALL | DAILYS_CLEAR_RT_STATE)

#define OTHER_SENSOR                 (0x00u)
#define GEOMAGNETIC                  (0x01u)
#define GEOMAGNETIC_UNCALIB          (0x02u)
#define GYROSCOPE                    (0x04u)
#define GYROSCOPE_UNCALIB            (0x08u)
#define STEP_COUNTER                 (0x10u)
#define STEP_DETECTOR                (0x20u)

#define LOG_PARAM_SNS_NON            (0x00u)
#define LOG_PARAM_SNS_ACC            (0x01u)
#define LOG_PARAM_SNS_MAG            (0x40u)
#define LOG_PARAM_SNS_GYRO           (0x80u)

#define LOG_PARAM_FUS_NON            (0x00u)
#define LOG_PARAM_FUS_ORI            (0x01u)
#define LOG_PARAM_FUS_GRV            (0x02u)
#define LOG_PARAM_FUS_LACC           (0x04u)
#define LOG_PARAM_FUS_ROT            (0x08u)
#define LOG_PARAM_FUS_MROT           (0x20u)
#define LOG_PARAM_FUS_GROT           (0x10u)

#define ERROR_FUP_CERTIFICATION      (0x0013u)
#define FUP_MAX_RAMSIZE              (512)
#define MEASURE_DATA_SIZE            (18)
#define LOGGING_FIFO_SIZE            (3072)

#define SNS_BATCH_PRID_ACC_MAX       (479400)
#define SNS_BATCH_PRID_ACC_MIN       (7520)
#define SNS_BATCH_PRID_ACC_UNIT      (1800)

#define SNS_BATCH_PRID_GYRO_MAX      (479400)
#define SNS_BATCH_PRID_GYRO_MIN      (5640)
#define SNS_BATCH_PRID_GYRO_UNIT     (1800)

#define SNS_BATCH_PRID_MAG_MAX       (479400)
#define SNS_BATCH_PRID_MAG_MIN       (15040)
#define SNS_BATCH_PRID_MAG_UNIT      (1800)

#define SNS_BATCH_PRID_FUSION_MAX    (2550000)
#define SNS_BATCH_PRID_FUSION_MIN    (10000)
#define SNS_BATCH_PRID_FUSION_UNIT   (10000)

#define SNS_BATCH_PRID_STEP_MAX      (7650000)
#define SNS_BATCH_PRID_STEP_MIN      (30000)
#define SNS_BATCH_PRID_STEP_UNIT     (30000)

#define SNS_BATCH_TIMEOUT_MAX        (0xFFFFFFFF)
#define SNS_BATCH_TIMEOUT_MIN        (30)
#define SNS_BATCH_TIMEOUT_UNIT       (30)

#define DEFAULT_WEIGHT               (650)
#define DEFAULT_PEDOMETER            (76)
#define DEFAULT_VEHITYPE             (2)
#define STATUS_READ_RETRY_NUM  98
#define SNS_WORK_QUEUE_NUM   30
#define MICON_I2C_ENABLE_RETRY_NUM  5
#define GRAVITY_EARTH        9806550
#define ABSMAX_2G            (GRAVITY_EARTH * 2)
#define ABSMIN_2G            (-GRAVITY_EARTH * 2)
#define ABSMAX_GYRO          (32000)
#define ABSMIN_GYRO          (-32000)
#define ABSMAX_MAG          (32767)
#define ABSMIN_MAG          (-32767)
#define ACCDATA_SIGN_COMVERT_14_32BIT(data)    ((data) & 0x00002000 ? (((~(data) & 0x00003FFF) + 1) * (-1)): (data))
#define GYRODATA_SIGN_COMVERT_12_32BIT(data)    ((data) & 0x00008000 ? (((~(data) & 0x0000FFFF) + 1) * (-1)): (data))
#define MAGDATA_SIGN_COMVERT_12_32BIT(data)    ((data) & 0x00008000 ? (((~(data) & 0x0000FFFF) + 1) * (-1)): (data))
#define GRADATA_SIGN_COMVERT_14_32BIT(data)    ((data) & 0x00002000 ? (((~(data) & 0x00003FFF) + 1) * (-1)): (data))
#define LINACCDATA_SIGN_COMVERT_14_32BIT(data)    ((data) & 0x00002000 ? (((~(data) & 0x00003FFF) + 1) * (-1)): (data))
#define LOGGING_RESPONSE_HEADER 54
enum {
    INTERRUPT_NONE = 0,
    INTERRUPT_TIMEOUT,
    INTERRUPT_BUF512,
    INTERRUPT_BUFFULL,
    INTERRUPT_MAX
};

static DEFINE_SPINLOCK(acc_lock);

static int u2dh_position = CONFIG_INPUT_ML610Q793_ACCELEROMETER_POSITION;
static const int u2dh_position_map[][3][3] = {
    { { 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1} }, /* top/upper-left */
    { {-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1} }, /* top/upper-right */
    { { 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1} }, /* top/lower-right */
    { { 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1} }, /* top/lower-left */
    { { 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1} }, /* bottom/upper-left */
    { { 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1} }, /* bottom/upper-right */
    { { 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1} }, /* bottom/lower-right */
    { {-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1} }, /* bottom/lower-right */
};

static struct gpiomux_setting spi_sens_config = {
    .func = GPIOMUX_FUNC_1,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config msm_spi_sens_configs[] = {
    {
        .gpio      = 0,         /* BLSP1 QUP SPI_DATA_MOSI */
        .settings = {
            [GPIOMUX_ACTIVE] = &spi_sens_config,
            [GPIOMUX_SUSPENDED] = &spi_sens_config,
        },
    },
    {
        .gpio      = 1,         /* BLSP1 QUP SPI_DATA_MISO */
        .settings = {
            [GPIOMUX_ACTIVE] = &spi_sens_config,
            [GPIOMUX_SUSPENDED] = &spi_sens_config,
        },
    },
    {
        .gpio      = 2,         /* BLSP1 QUP SPI_SENS_CS_N */
        .settings = {
            [GPIOMUX_ACTIVE] = &spi_sens_config,
            [GPIOMUX_SUSPENDED] = &spi_sens_config,
        },
    },
    {
        .gpio      = 3,         /* BLSP1 QUP SPI_CLK */
        .settings = {
            [GPIOMUX_ACTIVE] = &spi_sens_config,
            [GPIOMUX_SUSPENDED] = &spi_sens_config,
        },
    },
};

static struct gpiomux_setting spi_camera_config = {
    .func = GPIOMUX_FUNC_4,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config msm_spi_camera_configs[] = {
    {
        .gpio      = 8,         /* BLSP3 QUP SPI_CAM_CS_N */
        .settings = {
            [GPIOMUX_ACTIVE] = &spi_camera_config,
            [GPIOMUX_SUSPENDED] = &spi_camera_config,
        },
    },
};

typedef struct t_SNS_WorkQueue {
    struct work_struct  work;
    bool                status;
} SNS_WorkQueue;

static struct sensor_power_callback sns_pre_power_cb;
static SNS_WorkQueue  s_tSnsWork[SNS_WORK_QUEUE_NUM];
static struct workqueue_struct *sns_wq_int;
static struct workqueue_struct *sns_wq;
static int32_t g_nSnsWorkCnt;
static uint8_t g_MagGyroState;
static int32_t g_nFWVersion;
static int32_t g_nIntIrqNo;
static int32_t g_FusionDelay;
static uint8_t g_dailys_status;
static uint8_t g_step_status;
static uint8_t g_reset_param_status;
static HostCmdRes diag_res;
static uint8_t *g_logging_data;
static uint8_t g_InterruptType;
static atomic_t g_FusionState;
static atomic_t g_SnsTaskState;
static atomic_t g_AppTaskState;
static atomic_t g_FusTaskState;
static atomic_t g_nCalX;
static atomic_t g_nCalY;
static atomic_t g_nCalZ;
static atomic_t g_IntreqAcc;
static atomic_t g_LogParamSns;
static atomic_t g_LogParamFus;
static atomic_t g_Step_start;
static atomic_t g_MiconDebug;
static atomic_t g_acc_auto_cal_offset;

atomic_t g_nStepWide;
atomic_t g_nWeight;
atomic_t g_nVehiType;
atomic_t g_FWUpdateStatus;

static struct geomagnetic s_MagData;
static int8_t g_MagOffset[3];
static struct gyroscope s_GyroData;
static uint8_t g_logging_param[2];
static int32_t g_workqueue_used = 0;

static uint8_t g_sensors_period[4] = { 0x80, 0x0c, 0x80 , 0x80};


static struct sensor_batch_set_param_str sns_set_param_tbl[SENSOR_BATCH_TYPE_MAX] =
{
    {1,0},
    {1,0},
    {1,0},
    {1,0},
    {1,0},
    {1,0},
    {1,0},
    {1,0},
    {1,0},
    {1,0},
    {1,0},
};
static uint32_t sns_nonstep_timeout_param = 0;
static uint32_t sns_step_timeout_param = 0;
static struct sensor_batch_enable_param_str sns_batch_enable_param;

static void sns_batch_buffer(
    enum sensor_e_type type,
    bool on,
    bool ctrl
);
static uint32_t sns_get_batch_type( enum sensor_e_type type );
static void sns_cal_batch_period (
    enum sensor_e_type type,
    uint32_t period
);
static void sns_cal_batch_timeout (
    enum sensor_e_type type,
    uint32_t timeout
);
static void sns_cal_batch_enable_param (
    enum sensor_e_type type,
    bool on
);
static int32_t sns_mag_gyro_onoff(uint8_t kind ,bool arg_iEnable);
static int32_t sns_9axis_polling(void);
static int32_t sns_6axis_polling(void);
static int32_t sns_3axis_polling_75(void);
static int32_t sns_3axis_polling_300(void);
static int32_t sns_6axis_polling_a300m2400(void);
static int32_t sns_polling_off(void);
static void sns_batch_interrupt(void);
static bool sns_devif_error_check(void);
static void sns_pre_power_on(void);
static void sns_pre_power_off(void);
static int32_t sns_micon_i2c_enable(bool arg_iEnable);
static int32_t sns_micon_initcmd(void);
static int32_t sns_reset_restore_param(void);
static int32_t sns_reset_save_param(void);

static irqreturn_t sns_irq_handler(int32_t irq, void *dev_id);
static void sns_int_work_func(struct work_struct *work);
static void sns_int_app_work_func(struct work_struct *work);
static int32_t sns_gpio_init(void);
static void sns_FW_BRMP_ctrl(void);
static int32_t sns_update_fw_exe(bool boot, uint8_t *arg_iData, uint32_t arg_iLen);
static void sns_workqueue_init(void);
static int32_t sns_workqueue_create( struct workqueue_struct *queue, void (*func)(struct work_struct *) );
static void sns_workqueue_delete(struct work_struct *work);
static void sns_set_buff_int(bool arg_iEnable);


static int32_t sensor_micon_resume( struct spi_device *client );
static int32_t sensor_micon_suspend( struct spi_device *client, pm_message_t mesg );
static void sensor_micon_shutdown( struct spi_device *client );
static int32_t sensor_micon_remove( struct spi_device *client );
static int32_t sensor_micon_probe( struct spi_device *client );

#define ENABLE_IRQ {                                                         \
    if((g_nIntIrqNo != -1) && (atomic_read(&g_bIsIntIrqEnable) == false)){   \
        atomic_set(&g_bIsIntIrqEnable,true);                                 \
        enable_irq(g_nIntIrqNo);                                             \
    }                                                                        \
}
#define DISABLE_IRQ {                                                        \
    if((g_nIntIrqNo != -1) && (atomic_read(&g_bIsIntIrqEnable) == true)){    \
        disable_irq_nosync(g_nIntIrqNo);                                     \
        atomic_set(&g_bIsIntIrqEnable,false);                                \
    }                                                                        \
}

static void sns_batch_buffer(
    enum sensor_e_type type,
    bool on,
    bool ctrl
)
{
    uint32_t now_status;
    uint32_t next_status;

    SENSOR_N_LOG("start");

    now_status = sensor_get_batch_status();
    if( on == true) {
        next_status = (now_status | ( 1 << type) );
    } else {
        next_status = (now_status & (~( 1 << type)) );
    }

    if( true == ctrl ){
        if( (0 == now_status)&&(next_status) ){
            g_logging_data = (uint8_t *)kmalloc( LOGGING_FIFO_SIZE, GFP_KERNEL );
         }
    } else {
        if( (now_status) && (0 == next_status)){
            if(g_logging_data != NULL){
                kfree( g_logging_data );
                g_logging_data = NULL;
            }
        }
    }

    SENSOR_N_LOG("end");

    return;
}

static uint32_t sns_get_batch_type( enum sensor_e_type type )
{
    uint32_t batch_type = 0;

    SENSOR_N_LOG("start - type[%d]",(int)type);

    switch (type) {
    case SENSOR_ACC:
        batch_type = SENSOR_BATCH_TYPE_ACC;
        break;
    case SENSOR_GYRO:
    case SENSOR_GYRO_UNCAL:
        batch_type = SENSOR_BATCH_TYPE_GYRO;
        break;
    case SENSOR_MAG:
    case SENSOR_MAG_UNCAL:
        batch_type = SENSOR_BATCH_TYPE_MAG;
        break;
    case SENSOR_ACC_LNR:
        batch_type = SENSOR_BATCH_TYPE_ACC_LNR;
        break;
    case SENSOR_GRV:
        batch_type = SENSOR_BATCH_TYPE_GRV;
        break;
    case SENSOR_ORTN:
        batch_type = SENSOR_BATCH_TYPE_ORTN;
        break;
    case SENSOR_ROT_VCTR:
        batch_type = SENSOR_BATCH_TYPE_ROT_VCTR;
        break;
    case SENSOR_GAME_ROT_VCTR:
        batch_type = SENSOR_BATCH_TYPE_GAME_ROT_VCTR;
        break;
    case SENSOR_MAG_ROT_VCTR:
        batch_type = SENSOR_BATCH_TYPE_MAG_ROT_VCTR;
        break;
    case SENSOR_STEP_CNT:
        batch_type = SENSOR_BATCH_TYPE_STEP_CNT;
        break;
    case SENSOR_STEP_DTC:
        batch_type = SENSOR_BATCH_TYPE_STEP_DET;
        break;
    default:
        break;
    }

    SENSOR_N_LOG("end - batch_type[%d]",(int)batch_type);

    return batch_type;
}

static void sns_cal_batch_period (
    enum sensor_e_type type,
    uint32_t period_ms
)
{
    uint32_t batch_type = 0;
    uint8_t  period_param = 0;
    uint32_t period_us = 0;

    SENSOR_N_LOG("start - type[%d] period_ms[%d]",(int)type, (int)period_ms);

    period_us = period_ms * 1000;

    batch_type = sns_get_batch_type(type);

    switch (batch_type) {
    case SENSOR_BATCH_TYPE_ACC:
        SENSOR_A_LOG("SENSOR_BATCH_TYPE_ACC");
        if( SNS_BATCH_PRID_ACC_MIN > period_us ){
            period_us = SNS_BATCH_PRID_ACC_MIN;
        }
        if(SNS_BATCH_PRID_ACC_MAX < period_us ){
            period_us = SNS_BATCH_PRID_ACC_MAX;
        }
        period_param = period_us / SNS_BATCH_PRID_ACC_UNIT;
        break;
    case SENSOR_BATCH_TYPE_GYRO:
        SENSOR_A_LOG("SENSOR_BATCH_TYPE_GYRO");
        if( SNS_BATCH_PRID_GYRO_MIN > period_us ){
            period_us = SNS_BATCH_PRID_GYRO_MIN;
        }
        if(SNS_BATCH_PRID_GYRO_MAX < period_us ){
            period_us = SNS_BATCH_PRID_GYRO_MAX;
        }
        period_param = period_us / SNS_BATCH_PRID_GYRO_UNIT;
        break;
    case SENSOR_BATCH_TYPE_MAG:
        SENSOR_A_LOG("SENSOR_BATCH_TYPE_MAG");
        if( SNS_BATCH_PRID_MAG_MIN > period_us ){
            period_us = SNS_BATCH_PRID_MAG_MIN;
        }
        if(SNS_BATCH_PRID_MAG_MAX < period_us ){
            period_us = SNS_BATCH_PRID_MAG_MAX;
        }
        period_param = period_us / SNS_BATCH_PRID_MAG_UNIT;
        break;
    case SENSOR_BATCH_TYPE_ACC_LNR:
    case SENSOR_BATCH_TYPE_GRV:
    case SENSOR_BATCH_TYPE_ORTN:
    case SENSOR_BATCH_TYPE_ROT_VCTR:
    case SENSOR_BATCH_TYPE_GAME_ROT_VCTR:
    case SENSOR_BATCH_TYPE_MAG_ROT_VCTR:
        SENSOR_A_LOG("SENSOR_BATCH_TYPE_FUSION");
        if( SNS_BATCH_PRID_FUSION_MIN > period_us ){
            period_us = SNS_BATCH_PRID_FUSION_MIN;
        }
        if(SNS_BATCH_PRID_FUSION_MAX < period_us ){
            period_us = SNS_BATCH_PRID_FUSION_MAX;
        }
        period_param = period_us / SNS_BATCH_PRID_FUSION_UNIT;
        break;
    case SENSOR_BATCH_TYPE_STEP_CNT:
    case SENSOR_BATCH_TYPE_STEP_DET:
        SENSOR_A_LOG("SENSOR_BATCH_TYPE_STEP");
        if( SNS_BATCH_PRID_STEP_MIN > period_us ){
            period_us = SNS_BATCH_PRID_STEP_MIN;
        }
        if(SNS_BATCH_PRID_STEP_MAX < period_us ){
            period_us = SNS_BATCH_PRID_STEP_MAX;
        }
        period_param = period_us / SNS_BATCH_PRID_STEP_UNIT;
        break;
    default:
        break;
    }

    sns_set_param_tbl[batch_type].period = period_param;
    SENSOR_N_LOG("save period[%x]",(int)sns_set_param_tbl[batch_type].period);

    SENSOR_N_LOG("end");

    return;
}

static void sns_cal_batch_timeout (
    enum sensor_e_type type,
    uint32_t timeout
)
{
    uint32_t batch_type = 0;
    uint32_t fast_timeout = SNS_BATCH_TIMEOUT_MAX;
    int i;

    SENSOR_N_LOG("start - type[%d] timeout[%d]",(int)type, (int)timeout);

    batch_type = sns_get_batch_type(type);

    if( 0 != timeout ){
        if( SNS_BATCH_TIMEOUT_MAX < timeout ){
            timeout = SNS_BATCH_TIMEOUT_MAX;
        }
        if( SNS_BATCH_TIMEOUT_MIN > timeout ){
            timeout = SNS_BATCH_TIMEOUT_MIN;
        }
    }
    sns_set_param_tbl[batch_type].timeout = timeout;
    SENSOR_N_LOG("save timeout[%d]",(int)sns_set_param_tbl[batch_type].timeout);

    for(i=0; i<SENSOR_BATCH_TYPE_STEP_CNT; i++){
        if( 0 != sns_set_param_tbl[i].timeout ){
            if(fast_timeout > sns_set_param_tbl[i].timeout){
                fast_timeout = sns_set_param_tbl[i].timeout;
            }
        }
    }

    sns_nonstep_timeout_param = fast_timeout / SNS_BATCH_TIMEOUT_UNIT;
    SENSOR_N_LOG("save fasttimeout[%d]",(int)sns_nonstep_timeout_param);

    if( 0 == sns_set_param_tbl[SENSOR_BATCH_TYPE_STEP_CNT].timeout){
        sns_step_timeout_param = sns_set_param_tbl[SENSOR_BATCH_TYPE_STEP_DET].timeout / SNS_BATCH_TIMEOUT_UNIT;
    } else if ( 0 == sns_set_param_tbl[SENSOR_BATCH_TYPE_STEP_DET].timeout){
        sns_step_timeout_param = sns_set_param_tbl[SENSOR_BATCH_TYPE_STEP_CNT].timeout / SNS_BATCH_TIMEOUT_UNIT;
    } else {
        if( sns_set_param_tbl[SENSOR_BATCH_TYPE_STEP_DET].timeout
            > sns_set_param_tbl[SENSOR_BATCH_TYPE_STEP_CNT].timeout){
            sns_step_timeout_param = sns_set_param_tbl[SENSOR_BATCH_TYPE_STEP_CNT].timeout / SNS_BATCH_TIMEOUT_UNIT;
        } else {
            sns_step_timeout_param = sns_set_param_tbl[SENSOR_BATCH_TYPE_STEP_DET].timeout / SNS_BATCH_TIMEOUT_UNIT;
        }
    }

    SENSOR_N_LOG("save fasttimeout(step)[%d]",(int)sns_step_timeout_param);

    SENSOR_N_LOG("end");

    return;
}

static void sns_cal_batch_enable_param (
    enum sensor_e_type type,
    bool on
)
{
    uint32_t now_status;
    uint32_t next_status;

    uint32_t status_tmp_1 = 0;
    uint32_t status_tmp_2 = 0;
    uint32_t status_sns_tmp = 0;
    uint32_t status_fusion_tmp = 0;

    SENSOR_N_LOG("start");

    now_status = sensor_get_batch_status();
    if( on == true) {
        next_status = (now_status | ( 1 << type) );
    } else {
        next_status = (now_status & (~( 1 << type)) );
    }

    status_tmp_1 = ( (next_status & (1 << SENSOR_STEP_CNT)) >> SENSOR_STEP_CNT );
    if(status_tmp_1){
        sns_batch_enable_param.step_cnt = 0x01;
    }else{
        sns_batch_enable_param.step_cnt = 0x00;
    }

    status_tmp_1 = ( (next_status & (1 << SENSOR_STEP_DTC)) >> SENSOR_STEP_DTC );
    if(status_tmp_1){
        sns_batch_enable_param.step_det = 0x01;
    }else{
        sns_batch_enable_param.step_det = 0x00;
    }

    if(next_status){
        sns_batch_enable_param.timer_enable = 0x01;
    }else{
        sns_batch_enable_param.timer_enable = 0x00;
    }

    status_tmp_1 = ( (next_status & (1 << SENSOR_STEP_CNT)) >> SENSOR_STEP_CNT );
    if((status_tmp_1 ==0 ) && (type == SENSOR_STEP_CNT) && (on == 1)){
        sns_batch_enable_param.new_step_cnt = 0x01;
    } else {
        sns_batch_enable_param.new_step_cnt = 0x00;
    }

    status_tmp_1 = ((next_status & (1 << SENSOR_ACC) ) >> SENSOR_ACC );
    if(status_tmp_1){
        status_sns_tmp = status_sns_tmp | 0x01;
    }
    status_tmp_1 = ((next_status & (1 << SENSOR_MAG) ) >> SENSOR_MAG );
    status_tmp_2 = ((next_status & (1 << SENSOR_MAG_UNCAL) ) >> SENSOR_MAG_UNCAL );
    if( status_tmp_1 || status_tmp_2 ){
        status_sns_tmp = status_sns_tmp | (0x01 << 6);
    }
    status_tmp_1 = ((next_status & (1 << SENSOR_GYRO) ) >> SENSOR_GYRO );
    status_tmp_2 = ((next_status & (1 << SENSOR_GYRO_UNCAL) ) >> SENSOR_GYRO_UNCAL );
    if( status_tmp_1 || status_tmp_2){
        status_sns_tmp = status_sns_tmp | (0x01 << 7);
    }
    sns_batch_enable_param.sns_enable = status_sns_tmp;


    status_tmp_1 = ((next_status & (1 << SENSOR_ORTN) ) >> SENSOR_ORTN );
    if( status_tmp_1 ){
        status_fusion_tmp = status_fusion_tmp | (0x01);
    }
    status_tmp_1 = ((next_status & (1 << SENSOR_GRV) ) >> SENSOR_GRV );
    if( status_tmp_1 ){
        status_fusion_tmp = status_fusion_tmp | (0x01 << 1 );
    }
    status_tmp_1 = ((next_status & (1 << SENSOR_ACC_LNR) ) >> SENSOR_ACC_LNR );
    if( status_tmp_1 ){
        status_fusion_tmp = status_fusion_tmp | (0x01 << 2 );
    }
    status_tmp_1 = ((next_status & (1 << SENSOR_ROT_VCTR) ) >> SENSOR_ROT_VCTR );
    if( status_tmp_1 ){
        status_fusion_tmp = status_fusion_tmp | (0x01 << 3 );
    }
    status_tmp_1 = ((next_status & (1 << SENSOR_GAME_ROT_VCTR) ) >> SENSOR_GAME_ROT_VCTR );
    if( status_tmp_1 ){
        status_fusion_tmp = status_fusion_tmp | (0x01 << 4 );
    }
    status_tmp_1 = ((next_status & (1 << SENSOR_MAG_ROT_VCTR) ) >> SENSOR_MAG_ROT_VCTR );
    if( status_tmp_1 ){
        status_fusion_tmp = status_fusion_tmp | (0x01 << 5 );
    }
    sns_batch_enable_param.fusion_enable = status_fusion_tmp;

    SENSOR_N_LOG("end");

    return;
}

int32_t sns_logging_state(
    uint32_t type,
    uint32_t arg_timeout_ms,
    uint32_t arg_period_ms
)
{
    int32_t ret = SNS_RC_ERR;
    HostCmd cmd;
    HostCmdRes res;
    bool batch_on = false;
    struct sensor_batch_str dummy_batch;
    uint8_t buff_int_flg = 0;
    uint32_t batch_status = 0;

    SENSOR_N_LOG("start - type[%d] timeout[%d], period[%d]",
                  (int)type, (int)arg_timeout_ms, (int)arg_period_ms );

    if(arg_timeout_ms != 0){
        batch_on = true;
        buff_int_flg = 1;
        SENSOR_N_LOG("BatchON-buff_int_flg[%d]",buff_int_flg);
        if(SENSOR_STEP_CNT == type){
            g_step_status = g_step_status | STEP_COUNTER;
            SENSOR_N_LOG("g_step_status[%x]",g_step_status);
        }else if(SENSOR_STEP_DTC == type){
            g_step_status = g_step_status | STEP_DETECTOR;
            SENSOR_N_LOG("g_step_status[%x]",g_step_status);
        }
    } else {
        batch_on = false;
        batch_status = sensor_get_batch_status();
        SENSOR_N_LOG("BatchOFF-batch_status[%d]",batch_status);
        if( 0 == (batch_status & (~( BATCH_ON << type)) )) {
            buff_int_flg = 0;
            SENSOR_N_LOG("BatchOFF-buff_int_flg[%d]",buff_int_flg);
        }
        if(SENSOR_STEP_CNT == type){
            g_step_status = g_step_status & ~STEP_COUNTER;
            SENSOR_N_LOG("g_step_status[%x]",g_step_status);
        }else if(SENSOR_STEP_DTC == type){
            g_step_status = g_step_status & ~STEP_DETECTOR;
            SENSOR_N_LOG("g_step_status[%x]",g_step_status);
        }
    }

    sns_cal_batch_period(type, arg_period_ms);
    sns_cal_batch_timeout(type, arg_timeout_ms);
    sns_cal_batch_enable_param(type, batch_on);
    sns_batch_buffer(type, batch_on, true);

    if(batch_on == true){
        cmd.cmd.udata16 = HC_GYRO_SET_CALIB;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_SET_CALIB err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }

        cmd.cmd.udata16 = HC_MAG_SET_CALIB;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_SET_CALIB err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    cmd.cmd.udata16 = HC_MUL_SET_LOG_DELAY_SENSOR;
    cmd.prm.ub_prm[0] = sns_set_param_tbl[SENSOR_BATCH_TYPE_ACC].period;
    cmd.prm.ub_prm[1] = 0x01;
    cmd.prm.ub_prm[2] = sns_set_param_tbl[SENSOR_BATCH_TYPE_MAG].period;
    cmd.prm.ub_prm[3] = sns_set_param_tbl[SENSOR_BATCH_TYPE_GYRO].period;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_LOG_DELAY_SENSOR err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_LOG_DELAY_FUSION;
    cmd.prm.ub_prm[0] = sns_set_param_tbl[SENSOR_BATCH_TYPE_ORTN].period;
    cmd.prm.ub_prm[1] = sns_set_param_tbl[SENSOR_BATCH_TYPE_GRV].period;
    cmd.prm.ub_prm[2] = sns_set_param_tbl[SENSOR_BATCH_TYPE_ACC_LNR].period;
    cmd.prm.ub_prm[3] = sns_set_param_tbl[SENSOR_BATCH_TYPE_ROT_VCTR].period;
    cmd.prm.ub_prm[4] = sns_set_param_tbl[SENSOR_BATCH_TYPE_GAME_ROT_VCTR].period;
    cmd.prm.ub_prm[5] = sns_set_param_tbl[SENSOR_BATCH_TYPE_MAG_ROT_VCTR].period;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_LOG_DELAY_FUSION err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_SET_ANDROID_PARAM;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = (uint8_t)(sns_nonstep_timeout_param & 0xFF);
    cmd.prm.ub_prm[2] = (uint8_t)((sns_nonstep_timeout_param >> 8) & 0xFF);
    cmd.prm.ub_prm[3] = (uint8_t)((sns_nonstep_timeout_param >> 16) & 0xFF);
    cmd.prm.ub_prm[4] = (uint8_t)((sns_nonstep_timeout_param >> 24) & 0xFF);
    cmd.prm.ub_prm[5] = (uint8_t)(sns_step_timeout_param & 0xFF);
    cmd.prm.ub_prm[6] = (uint8_t)((sns_step_timeout_param >> 8) & 0xFF);
    cmd.prm.ub_prm[7] = (uint8_t)((sns_step_timeout_param >> 16) & 0xFF);
    cmd.prm.ub_prm[8] = (uint8_t)((sns_step_timeout_param >> 24) & 0xFF);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_SET_ANDROID_PARAM err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MAG_SET_BUFF_INT;
    cmd.prm.ub_prm[0] = buff_int_flg;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_BUFF_INT err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_SET_ANDROID_ENABLE;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = sns_batch_enable_param.step_cnt;
    cmd.prm.ub_prm[2] = sns_batch_enable_param.step_det;
    cmd.prm.ub_prm[3] = sns_batch_enable_param.timer_enable;
    cmd.prm.ub_prm[4] = sns_batch_enable_param.new_step_cnt;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_SET_ANDROID_ENABLE err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_LOG_PARAM_SENSOR;
    cmd.prm.ub_prm[0] = (uint8_t)sns_batch_enable_param.sns_enable;
    cmd.prm.ub_prm[1] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_LOG_PARAM_SENSOR err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_LOG_PARAM_FUSION;
    cmd.prm.ub_prm[0] = (uint8_t)sns_batch_enable_param.fusion_enable;
    cmd.prm.ub_prm[1] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_LOG_PARAM_FUSION err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    sns_micon_get_batch_data(&dummy_batch);

    sns_batch_buffer(type, batch_on, false);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static int32_t sns_mag_gyro_onoff(uint8_t arg_Kind ,bool arg_iEnable)
{
    int32_t    ret;
    HostCmd    cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    if( GEOMAGNETIC_UNCALIB == arg_Kind ) {
        cmd.cmd.udata16 = HC_MAG_SET_CALIB;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x01;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_SET_CALIB err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }

        cmd.cmd.udata16 = HC_MAG_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_GET_DATA err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        else {
            s_MagData.x = res.res.sw_res[0];
            s_MagData.y = res.res.sw_res[1];
            s_MagData.z = res.res.sw_res[2];

            SENSOR_N_LOG("mag_data - x[%d] y[%d] z[%d]",s_MagData.x,s_MagData.y,s_MagData.z);
        }
    } else {
        cmd.cmd.udata16 = HC_MAG_SET_CALIB;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_SET_CALIB err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    if((arg_iEnable == false)&&
       (((arg_Kind == GEOMAGNETIC_UNCALIB)&&
         (g_MagGyroState == ( GEOMAGNETIC_UNCALIB & g_MagGyroState))) ||
        ((arg_Kind == GEOMAGNETIC)&&
         (g_MagGyroState == ( GEOMAGNETIC & g_MagGyroState))))){
        cmd.cmd.udata16 = HC_MAG_GET_OFFSET;
        ret = sns_hostcmd(&cmd, &res, 3, EXE_HOST_ALL, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end GEOMAGNETIC_UNCALIB err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        else {
            g_MagOffset[0] = res.res.sb_res[0];
            g_MagOffset[1] = res.res.sb_res[1];
            g_MagOffset[2] = res.res.sb_res[2];

            SENSOR_N_LOG("mag_offset - [%d] [%d] [%d]",g_MagOffset[0],g_MagOffset[1],g_MagOffset[2] );
        }

        cmd.cmd.udata16 = HC_MAG_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_GET_DATA err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        else {
            s_MagData.x = res.res.sw_res[0];
            s_MagData.y = res.res.sw_res[1];
            s_MagData.z = res.res.sw_res[2];

            SENSOR_N_LOG("mag_data - x[%d] y[%d] z[%d]",s_MagData.x,s_MagData.y,s_MagData.z);
        }
    }

    if( GYROSCOPE_UNCALIB == arg_Kind ) {
        cmd.cmd.udata16 = HC_GYRO_SET_CALIB;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x01;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_SET_CALIB err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }

        cmd.cmd.udata16 = HC_GYRO_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_GET_DATA err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        else {
            s_GyroData.x = res.res.sw_res[0];
            s_GyroData.y = res.res.sw_res[1];
            s_GyroData.z = res.res.sw_res[2];

            SENSOR_N_LOG("gyro_data - x[%d] y[%d] z[%d]",s_GyroData.x,s_GyroData.y,s_GyroData.z);
        }
    } else {
        cmd.cmd.udata16 = HC_GYRO_SET_CALIB;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_SET_CALIB err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    if((arg_iEnable == false)&&
       (((arg_Kind == GYROSCOPE_UNCALIB)&&
         (g_MagGyroState == ( GYROSCOPE_UNCALIB & g_MagGyroState))) ||
        ((arg_Kind == GYROSCOPE)&&
         (g_MagGyroState == ( GYROSCOPE & g_MagGyroState))))){
        cmd.cmd.udata16 = HC_GYRO_GET_DATA;
        ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_GET_DATA err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        else {
            s_GyroData.x = res.res.sw_res[0];
            s_GyroData.y = res.res.sw_res[1];
            s_GyroData.z = res.res.sw_res[2];

            SENSOR_N_LOG("gyro_data - x[%d] y[%d] z[%d]",s_GyroData.x,s_GyroData.y,s_GyroData.z);
        }
    }

    SENSOR_N_LOG("end - SNS_RC_OK");
    return SNS_RC_OK;
}

static int32_t sns_9axis_polling(void)
{
    int32_t    ret;
    HostCmd    cmd;
    HostCmdRes res;
    uint8_t    set_data1;
    uint8_t    set_data2;
    uint8_t    set_data3;
    uint8_t    set_data4;
    uint8_t    delay[2];

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MUL_SET_ANDROID;
    cmd.prm.ub_prm[0] = 0xC1;
    cmd.prm.ub_prm[1] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ANDROID err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    set_data1 = SNS_ON | atomic_read(&g_FusionState);
    cmd.cmd.udata16 = HC_MUL_SET_FUSION;
    cmd.prm.ub_prm[0] = set_data1;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    atomic_set(&g_FusionState,set_data1);

    cmd.cmd.udata16 = HC_MUL_SET_ANDROID_PERIOD;
    if(g_sensors_period[0] < 0x04)
        g_sensors_period[0] = 0x04;

    cmd.prm.ub_prm[0] = g_sensors_period[0];
    cmd.prm.ub_prm[1] = g_sensors_period[1];
    cmd.prm.ub_prm[2] = g_sensors_period[2];
    cmd.prm.ub_prm[3] = g_sensors_period[3];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ANDROID_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    if(g_FusionDelay == 0){
        delay[0] = 0xe8;
        delay[1] = 0x03;
    } else {
        delay[0] = (uint8_t)g_FusionDelay;
        delay[1] = (uint8_t)(g_FusionDelay << 8);
    }

    cmd.cmd.udata16 = HC_MCU_SET_TASK;
    cmd.prm.ub_prm[0] = 0xbc;
    cmd.prm.ub_prm[1] = 0x00;
    cmd.prm.ub_prm[2] = 0xb8;
    cmd.prm.ub_prm[3] = 0x0b;
    cmd.prm.ub_prm[4] = delay[0];
    cmd.prm.ub_prm[5] = delay[1];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_SET_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_ACC_SET_PARAM;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x04;
    cmd.prm.ub_prm[2] = 0x03;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_PARAM err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    set_data2 = SNS_ON | atomic_read(&g_SnsTaskState);
    set_data3 = SNS_OFF | atomic_read(&g_AppTaskState);
    set_data4 = SNS_ON | atomic_read(&g_FusTaskState);
    cmd.cmd.udata16 = HC_MCU_EXEC_TASK;
    cmd.prm.ub_prm[0] = set_data2;
    if(batch_app_task_flag){
        cmd.prm.ub_prm[1] = 0x01;
    }else{
        cmd.prm.ub_prm[1] = set_data3;
    }
    cmd.prm.ub_prm[2] = set_data4;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_EXEC_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    atomic_set(&g_SnsTaskState,set_data2);
    atomic_set(&g_AppTaskState,set_data3);
    atomic_set(&g_FusTaskState,set_data4);

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

static int32_t sns_6axis_polling(void)
{
    int32_t    ret;
    HostCmd    cmd;
    HostCmdRes res;
    uint8_t    set_data1;
    uint8_t    set_data2;
    uint8_t    set_data3;
    uint8_t    set_data4;
    uint8_t    delay[2];

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MUL_SET_ANDROID;
    cmd.prm.ub_prm[0] = 0x41;
    cmd.prm.ub_prm[1] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ANDROID err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    set_data1 = SNS_ON | atomic_read(&g_FusionState);
    cmd.cmd.udata16 = HC_MUL_SET_FUSION;
    cmd.prm.ub_prm[0] = set_data1;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    atomic_set(&g_FusionState,set_data1);

    cmd.cmd.udata16 = HC_MUL_SET_ANDROID_PERIOD;
    if(g_sensors_period[0] < 0x04)
        g_sensors_period[0] = 0x04;
    cmd.prm.ub_prm[0] = g_sensors_period[0];
    cmd.prm.ub_prm[1] = g_sensors_period[1];
    cmd.prm.ub_prm[2] = g_sensors_period[2];
    cmd.prm.ub_prm[3] = g_sensors_period[3];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ANDROID_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    if(g_FusionDelay == 0){
        delay[0] = 0xe8;
        delay[1] = 0x03;
    } else {
        delay[0] = (uint8_t)g_FusionDelay;
        delay[1] = (uint8_t)(g_FusionDelay << 8);
    }

    cmd.cmd.udata16 = HC_MCU_SET_TASK;
    cmd.prm.ub_prm[0] = 0xbc;
    cmd.prm.ub_prm[1] = 0x00;
    cmd.prm.ub_prm[2] = 0xb8;
    cmd.prm.ub_prm[3] = 0x0b;
    cmd.prm.ub_prm[4] = delay[0];
    cmd.prm.ub_prm[5] = delay[1];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_SET_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_ACC_SET_PARAM;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x04;
    cmd.prm.ub_prm[2] = 0x03;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_PARAM err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    set_data2 = SNS_ON | atomic_read(&g_SnsTaskState);
    set_data3 = SNS_OFF | atomic_read(&g_AppTaskState);
    set_data4 = SNS_ON | atomic_read(&g_FusTaskState);
    cmd.cmd.udata16 = HC_MCU_EXEC_TASK;
    cmd.prm.ub_prm[0] = set_data2;
    if(batch_app_task_flag){
        cmd.prm.ub_prm[1] = 0x01;
    }else{
        cmd.prm.ub_prm[1] = set_data3;
    }
    cmd.prm.ub_prm[2] = set_data4;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_EXEC_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    atomic_set(&g_SnsTaskState,set_data2);
    atomic_set(&g_AppTaskState,set_data3);
    atomic_set(&g_FusTaskState,set_data4);

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

static int32_t sns_3axis_polling_75(void)
{
    int32_t    ret;
    HostCmd    cmd;
    HostCmdRes res;
    uint8_t    set_data1;
    uint8_t    set_data2;
    uint8_t    set_data3;
    uint8_t    set_data4;
    uint8_t    set_android_param;
    enum sensor_micon_polling_e_type    polling = SENSOR_POLLING_OFF;

    SENSOR_N_LOG("start");
    polling = sensor_get_poll_status();
    SENSOR_N_LOG("sensor_get_poll_status-polling[%d]",polling);
    if(polling == SENSOR_6AXIS_POLLING_A300M2400) {
        set_android_param = 0x41;
    } else {
        set_android_param = 0x01;
    }

    cmd.cmd.udata16 = HC_MUL_SET_ANDROID;
    cmd.prm.ub_prm[0] = set_android_param;
    cmd.prm.ub_prm[1] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ANDROID err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    set_data1 = SNS_OFF & atomic_read(&g_FusionState);
    cmd.cmd.udata16 = HC_MUL_SET_FUSION;
    cmd.prm.ub_prm[0] = set_data1;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    atomic_set(&g_FusionState,set_data1);

    cmd.cmd.udata16 = HC_MUL_SET_ANDROID_PERIOD;
    if(g_sensors_period[0] < 0x04)
        g_sensors_period[0] = 0x04;

    cmd.prm.ub_prm[0] = g_sensors_period[0];
    cmd.prm.ub_prm[1] = g_sensors_period[1];
    cmd.prm.ub_prm[2] = 0x80;
    cmd.prm.ub_prm[3] = g_sensors_period[3];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ANDROID_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MCU_SET_TASK;
    cmd.prm.ub_prm[0] = 0xbc;
    cmd.prm.ub_prm[1] = 0x00;
    cmd.prm.ub_prm[2] = 0xb8;
    cmd.prm.ub_prm[3] = 0x0b;
    cmd.prm.ub_prm[4] = 0xe8;
    cmd.prm.ub_prm[5] = 0x03;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_SET_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_ACC_SET_PARAM;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x04;
    cmd.prm.ub_prm[2] = 0x03;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_PARAM err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    set_data2 = SNS_ON | atomic_read(&g_SnsTaskState);
    set_data3 = SNS_OFF | atomic_read(&g_AppTaskState);
    set_data4 = SNS_OFF & atomic_read(&g_FusTaskState);
    cmd.cmd.udata16 = HC_MCU_EXEC_TASK;
    cmd.prm.ub_prm[0] = set_data2;
    if(batch_app_task_flag){
        cmd.prm.ub_prm[1] = 0x01;
    }else{
        cmd.prm.ub_prm[1] = set_data3;
    }
    cmd.prm.ub_prm[2] = set_data4;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_EXEC_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    atomic_set(&g_SnsTaskState,set_data2);
    atomic_set(&g_AppTaskState,set_data3);
    atomic_set(&g_FusTaskState,set_data4);

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

static int32_t sns_3axis_polling_300(void)
{
    int32_t    ret;
    HostCmd    cmd;
    HostCmdRes res;
    uint8_t    set_data1;
    uint8_t    set_data2;
    uint8_t    set_data3;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MUL_SET_ANDROID;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ANDROID err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_FUSION;
    cmd.prm.ub_prm[0] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    atomic_set(&g_FusionState,SNS_OFF);

    cmd.cmd.udata16 = HC_MUL_SET_ANDROID_PERIOD;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = g_sensors_period[1];
    cmd.prm.ub_prm[2] = g_sensors_period[2];
    cmd.prm.ub_prm[3] = g_sensors_period[3];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ANDROID_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MCU_SET_TASK;
    cmd.prm.ub_prm[0] = 0xb8;
    cmd.prm.ub_prm[1] = 0x0b;
    cmd.prm.ub_prm[2] = 0xb8;
    cmd.prm.ub_prm[3] = 0x0b;
    cmd.prm.ub_prm[4] = 0xe8;
    cmd.prm.ub_prm[5] = 0x03;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_SET_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_ACC_SET_PARAM;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x04;
    cmd.prm.ub_prm[2] = 0x06;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_PARAM err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    set_data1 = SNS_ON | atomic_read(&g_SnsTaskState);
    set_data2 = SNS_ON | atomic_read(&g_AppTaskState);
    set_data3 = SNS_OFF & atomic_read(&g_FusTaskState);
    cmd.cmd.udata16 = HC_MCU_EXEC_TASK;
    cmd.prm.ub_prm[0] = set_data1;
    cmd.prm.ub_prm[1] = set_data2;
    cmd.prm.ub_prm[2] = set_data3;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_EXEC_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    atomic_set(&g_SnsTaskState,set_data1);
    atomic_set(&g_AppTaskState,set_data2);
    atomic_set(&g_FusTaskState,set_data3);

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

static int32_t sns_6axis_polling_a300m2400(void)
{
    int32_t    ret;
    HostCmd    cmd;
    HostCmdRes res;
    uint8_t    set_data1;
    uint8_t    set_data2;
    uint8_t    set_data3;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MUL_SET_ANDROID;
    cmd.prm.ub_prm[0] = 0x41;
    cmd.prm.ub_prm[1] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ANDROID err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_FUSION;
    cmd.prm.ub_prm[0] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    atomic_set(&g_FusionState,SNS_OFF);

    cmd.cmd.udata16 = HC_MUL_SET_ANDROID_PERIOD;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = g_sensors_period[1];
    cmd.prm.ub_prm[2] = g_sensors_period[2];
    cmd.prm.ub_prm[3] = g_sensors_period[3];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ANDROID_PERIOD err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MCU_SET_TASK;
    cmd.prm.ub_prm[0] = 0xb8;
    cmd.prm.ub_prm[1] = 0x0b;
    cmd.prm.ub_prm[2] = 0xb8;
    cmd.prm.ub_prm[3] = 0x0b;
    cmd.prm.ub_prm[4] = 0xe8;
    cmd.prm.ub_prm[5] = 0x03;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_SET_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_ACC_SET_PARAM;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x04;
    cmd.prm.ub_prm[2] = 0x06;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_PARAM err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    set_data1 = SNS_ON | atomic_read(&g_SnsTaskState);
    set_data2 = SNS_ON | atomic_read(&g_AppTaskState);
    set_data3 = SNS_OFF & atomic_read(&g_FusTaskState);
    cmd.cmd.udata16 = HC_MCU_EXEC_TASK;
    cmd.prm.ub_prm[0] = set_data1;
    cmd.prm.ub_prm[1] = set_data2;
    cmd.prm.ub_prm[2] = set_data3;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_EXEC_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    atomic_set(&g_SnsTaskState,set_data1);
    atomic_set(&g_AppTaskState,set_data2);
    atomic_set(&g_FusTaskState,set_data3);

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

static int32_t sns_polling_off(void)
{
    int32_t    ret;
    HostCmd    cmd;
    HostCmdRes res;
    uint8_t    set_data1;
    uint8_t    set_data2;
    uint8_t    set_data3;
    uint8_t    set_data4;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MUL_SET_ANDROID;
    cmd.prm.ub_prm[0] = 0x00;
    cmd.prm.ub_prm[1] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_ANDROID err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    set_data1 = SNS_OFF & atomic_read(&g_FusionState);
    cmd.cmd.udata16 = HC_MUL_SET_FUSION;
    cmd.prm.ub_prm[0] = set_data1;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    atomic_set(&g_FusionState,set_data1);

    set_data2 = SNS_OFF & atomic_read(&g_SnsTaskState);
    set_data3 = SNS_OFF & atomic_read(&g_AppTaskState);
    set_data4 = SNS_OFF & atomic_read(&g_FusTaskState);
    cmd.cmd.udata16 = HC_MCU_EXEC_TASK;
    cmd.prm.ub_prm[0] = set_data2;
    cmd.prm.ub_prm[1] = set_data3;
    cmd.prm.ub_prm[2] = set_data4;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_EXEC_TASK err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    atomic_set(&g_SnsTaskState,set_data2);
    atomic_set(&g_AppTaskState,set_data3);
    atomic_set(&g_FusTaskState,set_data4);

    SENSOR_N_LOG("period %d %d %d ", g_sensors_period[0] ,g_sensors_period[2] ,g_sensors_period[3] );

    g_sensors_period[0] = 0x80;
    g_sensors_period[2] = 0x80; 
    g_sensors_period[3] = 0x80; 

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

void sns_set_period( uint32_t type ,int32_t polltime ,uint8_t flag)
{
    int32_t    ret = 0;
    HostCmd    cmd;
    HostCmdRes res;
    uint8_t    prm_tmp = 0;
    enum sensor_micon_polling_e_type poll_status = 0;
    enum sensor_micon_polling_e_type poll_status_tmp = 0;

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("type:%d polltime:%d flag:%d", type, (int)polltime, flag);

    if(polltime <= 400){
        prm_tmp = 0x80;
        if(polltime < 240){
            prm_tmp = 0x6b;
            if(polltime <  200){
                prm_tmp = 0x35;
                if(polltime < 100){
                    prm_tmp = 0x23;
                    if(polltime < 66 ){
                        prm_tmp = 0x10;
                        if(polltime < 30 ){
                            prm_tmp = 0x0a;
                            if(polltime < 20 ){
                               prm_tmp = 0x08;
                               if(polltime < 15 ){
                                   prm_tmp = 0x05;
                                   if(polltime < 10 ){
                                       prm_tmp = 0x04;
                                       if(polltime < 8 ){
                                           prm_tmp = 0x02;
                                       }
                                   }
                               }
                            }
                        }
                    }
                }
            }
        }
    }else{
        SENSOR_ERR_LOG("unsupport polltime[%d]", (int)polltime);
        return;
    }

    SENSOR_N_LOG("changed prm_tmp:%d",prm_tmp);

    if(( SENSOR_ON == sensor_get_status(SENSOR_SGNFCNT_MTN) )||
       ( SENSOR_ON == sensor_get_status(SENSOR_STEP_CNT) )||
       ( SENSOR_ON == sensor_get_status(SENSOR_STEP_DTC) )||
       ( SENSOR_ON == sensor_get_status(SENSOR_EXT_PEDO) )||
       ( SENSOR_ON == sensor_get_status(SENSOR_EXT_VEHI) )||
       ( SENSOR_ON == sensor_get_status(SENSOR_EXT_IWIFI) )||
       ( SENSOR_ON == sensor_get_status(SENSOR_EXT_VH) )||
       ( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_WALK_START) )||
       ( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_WALK_STOP) )||
       ( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_TRAIN) )||
       ( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_VEHICLE) )||
       ( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_BRINGUP) ) ) {

        if(flag == 1){ 
            SENSOR_N_LOG("Call on polltime or batch");
        }else{
            SENSOR_N_LOG("Call on sensor_enable"); 

            poll_status = sensor_get_poll_status_current();

            if((poll_status == SENSOR_3AXIS_POLLING_300) || (poll_status == SENSOR_6AXIS_POLLING_A300M2400)){
                g_sensors_period[0] = 0x01; 
                g_sensors_period[2] = 0x08; 
                SENSOR_N_LOG("Guard 0x01 0x08");
            }

            if(g_sensors_period[0] > 0x10){
                g_sensors_period[0] = 0x10;
            }

            SENSOR_N_LOG("pollingStatus:%d g_sensors_period[0]:%d g_sensors_period[2]:%d",poll_status, g_sensors_period[0],g_sensors_period[2]); 

            cmd.cmd.udata16 = HC_MUL_SET_ANDROID_PERIOD;
            cmd.prm.ub_prm[0] = g_sensors_period[0];
            cmd.prm.ub_prm[1] = g_sensors_period[1];
            cmd.prm.ub_prm[2] = g_sensors_period[2];
            cmd.prm.ub_prm[3] = g_sensors_period[3];
            ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_MUL_SET_ANDROID_PERIOD err[%x]",res.err.udata16);
                return;
            }
            return;
        }
    }else{
        if(flag == 0){
            SENSOR_N_LOG("Call enable not 3axis300,6axis2400 Call on sensor_enable");
            return;
        }
    }

    SENSOR_N_LOG("start type select");

    switch (type) {
        case SENSOR_ACC:
             poll_status_tmp = SENSOR_3AXIS_POLLING_75;
            break;
        case SENSOR_MAG:
        case SENSOR_MAG_UNCAL:
        case SENSOR_MAG_ROT_VCTR:
            poll_status_tmp = SENSOR_6AXIS_POLLING;
            break;
        case SENSOR_GYRO:
        case SENSOR_GYRO_UNCAL:
        case SENSOR_ACC_LNR:
        case SENSOR_GRV:
        case SENSOR_ORTN:
        case SENSOR_ROT_VCTR:
        case SENSOR_GAME_ROT_VCTR:
            poll_status_tmp = SENSOR_9AXIS_POLLING;
            break;
        case SENSOR_STEP_CNT:
        case SENSOR_STEP_DTC:
            SENSOR_N_LOG("batch only prm_tmp:%d",prm_tmp);
            poll_status_tmp = SENSOR_3AXIS_POLLING_300;
            break;
        default:
            SENSOR_N_LOG("unsupport sensor[%d]", type);
            return;
    }

    switch (poll_status_tmp) {
        case SENSOR_3AXIS_POLLING_75:
            if(g_sensors_period[0] > prm_tmp){
                g_sensors_period[0] = prm_tmp; 
            }
            SENSOR_N_LOG("3polling g_sensors_period %d %d %d ", g_sensors_period[0] ,g_sensors_period[2] ,g_sensors_period[3] );
            break;

        case SENSOR_6AXIS_POLLING:
            if(g_sensors_period[0] > prm_tmp){
                g_sensors_period[0] = prm_tmp; 
            }

            if(g_sensors_period[2] > prm_tmp){
                g_sensors_period[2] = prm_tmp; 
            }
            SENSOR_N_LOG("6polling g_sensors_period %d %d %d ", g_sensors_period[0] ,g_sensors_period[2] ,g_sensors_period[3] );
            break;

        case SENSOR_9AXIS_POLLING:
            if(g_sensors_period[0] > prm_tmp){
                g_sensors_period[0] = prm_tmp; 
            }

            if(g_sensors_period[2] > prm_tmp){
                g_sensors_period[2] = prm_tmp; 
            }

            if(g_sensors_period[3] > prm_tmp){
                g_sensors_period[3] = prm_tmp; 
            }
            SENSOR_N_LOG("9polling g_sensors_period %d %d %d ", g_sensors_period[0] ,g_sensors_period[2] ,g_sensors_period[3] );
            break;

        case SENSOR_3AXIS_POLLING_300:
            if(g_sensors_period[0] > prm_tmp){
                g_sensors_period[0] = prm_tmp; 
            }
            if( prm_tmp > 0x10){
                g_sensors_period[0] = 0x10;
            }
            SENSOR_N_LOG("3polling g_sensors_period %d %d %d ", g_sensors_period[0] ,g_sensors_period[2] ,g_sensors_period[3] );
            break;

        default:
            SENSOR_N_LOG("unsupport sensor[%d]", type);
            return;
    }

    if(g_sensors_period[0] < 0x04){
        g_sensors_period[0] = 0x04;
        SENSOR_N_LOG("Guard 0x04");
    }

    if(g_sensors_period[2] < 0x08){
        g_sensors_period[2] = 0x08;
        SENSOR_N_LOG("Guard 0x08");
    }

    poll_status = sensor_get_poll_status_current();

    SENSOR_N_LOG("select poll_status:%d poll_status_tmp:%d",poll_status,poll_status_tmp);

    if((poll_status >= poll_status_tmp)){

        SENSOR_N_LOG("hostcmd ext");

        cmd.cmd.udata16 = HC_MUL_SET_ANDROID_PERIOD;
        cmd.prm.ub_prm[0] = g_sensors_period[0];
        cmd.prm.ub_prm[1] = g_sensors_period[1];
        cmd.prm.ub_prm[2] = g_sensors_period[2];
        cmd.prm.ub_prm[3] = g_sensors_period[3];
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MUL_SET_ANDROID_PERIOD err[%x]",res.err.udata16);
            return;
        }
    }

    SENSOR_N_LOG("g_sensors_period[0]:%d g_sensors_period[1]:%d g_sensors_period[2]:%d g_sensors_period[3]:%d",
                   g_sensors_period[0], g_sensors_period[1], g_sensors_period[2], g_sensors_period[3]);

    SENSOR_N_LOG("end");

    return;
}

void sns_set_app_task( uint32_t type ,uint8_t onoff)
{
    int32_t    ret = 0;
    HostCmd    cmd;
    HostCmdRes res;
    uint8_t    app_task_flag = 0;
    enum sensor_micon_polling_e_type poll_status = 0;
    uint32_t now_status = 0;
    uint32_t before_status = 0;

    SENSOR_N_LOG("start");

    if(onoff == SENSOR_ON){
        SENSOR_N_LOG("sensor on type:%d",type);
        switch (type) {
            case SENSOR_EXT_PEDO:
            case SENSOR_EXT_VEHI:
            case SENSOR_EXT_IWIFI:
            case SENSOR_KC_MOTION_TRAIN:
            case SENSOR_KC_MOTION_VEHICLE:
            case SENSOR_KC_MOTION_BRINGUP:
            case SENSOR_SGNFCNT_MTN:
            case SENSOR_STEP_CNT:
            case SENSOR_STEP_DTC:
            case SENSOR_EXT_VH:
            case SENSOR_KC_MOTION_WALK_START:
            case SENSOR_KC_MOTION_WALK_STOP:
                app_task_flag = 1;
                break;
            default:
                break;
        }

        if(app_task_flag){
            poll_status = sensor_get_poll_status_current();

            SENSOR_N_LOG("sensor on poll status :%d",poll_status);

            if(poll_status > SENSOR_6AXIS_POLLING_A300M2400){

                cmd.cmd.udata16 = HC_MCU_EXEC_TASK;
                cmd.prm.ub_prm[0] = 0x01;
                cmd.prm.ub_prm[1] = 0x01;
                if(poll_status == SENSOR_3AXIS_POLLING_75){
                    cmd.prm.ub_prm[2] = 0x00;
                }else{
                    cmd.prm.ub_prm[2] = 0x01;
                }

                ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
                if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                    SENSOR_ERR_LOG("end HC_MCU_EXEC_TASK err[%x]",res.err.udata16);
                    return ;
               }
            }
        }
    }else{
        SENSOR_N_LOG("sensor off type:%d",type);
        if(( SENSOR_ON != sensor_get_status(SENSOR_SGNFCNT_MTN) )&&
           ( SENSOR_ON != sensor_get_status(SENSOR_STEP_CNT) )&&
           ( SENSOR_ON != sensor_get_status(SENSOR_STEP_DTC) )&&
           ( SENSOR_ON != sensor_get_status(SENSOR_EXT_PEDO) )&&
           ( SENSOR_ON != sensor_get_status(SENSOR_EXT_VEHI) )&&
           ( SENSOR_ON != sensor_get_status(SENSOR_EXT_IWIFI) )&&
           ( SENSOR_ON != sensor_get_status(SENSOR_EXT_VH) )&&
           ( SENSOR_ON != sensor_get_status(SENSOR_KC_MOTION_WALK_START) )&&
           ( SENSOR_ON != sensor_get_status(SENSOR_KC_MOTION_WALK_STOP) )&&
           ( SENSOR_ON != sensor_get_status(SENSOR_KC_MOTION_TRAIN) )&&
           ( SENSOR_ON != sensor_get_status(SENSOR_KC_MOTION_VEHICLE) )&&
           ( SENSOR_ON != sensor_get_status(SENSOR_KC_MOTION_BRINGUP) ) ) {

            now_status = sensor_get_batch_status();

            before_status = ( now_status & (~( BATCH_ON << type)) );

            SENSOR_N_LOG(" now_status:%d before_status:%d",now_status,before_status);

            if( 0 == before_status ){

                poll_status = sensor_get_poll_status_current();

                SENSOR_N_LOG("sensor off poll status :%d",poll_status);

                cmd.cmd.udata16 = HC_MCU_EXEC_TASK;
                cmd.prm.ub_prm[0] = 0x01;
                cmd.prm.ub_prm[1] = 0x00;
                if(poll_status == SENSOR_3AXIS_POLLING_75){
                    cmd.prm.ub_prm[2] = 0x00;
                }else{
                    cmd.prm.ub_prm[2] = 0x01;
                }

                ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
                if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                    SENSOR_ERR_LOG("end HC_MCU_EXEC_TASK err[%x]",res.err.udata16);
                    return ;
                }
            }
        }
    }

    SENSOR_N_LOG("end");

    return;
}
void sns_set_app_task_batch( uint32_t type )
{
    int32_t    ret = 0;
    HostCmd    cmd;
    HostCmdRes res;
    enum sensor_micon_polling_e_type poll_status = 0;

    SENSOR_N_LOG("start");

    poll_status = sensor_get_poll_status_current();

    SENSOR_N_LOG("sensor on poll status :%d",poll_status);

    cmd.cmd.udata16 = HC_MCU_EXEC_TASK;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x01;
    if(poll_status == SENSOR_3AXIS_POLLING_75){
        cmd.prm.ub_prm[2] = 0x00;
    }else{
        cmd.prm.ub_prm[2] = 0x01;
    }

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_EXEC_TASK err[%x]",res.err.udata16);
        return ;
    }

    SENSOR_N_LOG("end");
}

int32_t sns_micon_get_batch_data(struct sensor_batch_str* batch_p)
{
    int32_t ret = SNS_RC_ERR;
    HostCmd cmd;
    HostCmdRes res;
    uint16_t rogging_data_size = 0;
    uint16_t read_data_size = 0;
    uint32_t stepcount_data = 0;
    uint8_t  fifo_size_res[2];
    uint16_t fifo_data_size = 0;
    uint32_t time_stamp_acc = 0;
    uint32_t time_stamp_mag = 0;
    uint32_t time_stamp_gyro = 0;
    uint32_t time_stamp_ori = 0;
    uint32_t time_stamp_gravity = 0;
    uint32_t time_stamp_linacc = 0;
    uint32_t time_stamp_rota = 0;
    uint32_t time_stamp_gamerota = 0;
    uint32_t time_stamp_magrota = 0;
    int32_t  time_stamp_step1 = 0;
    int32_t  time_stamp_step2 = 0;
    int32_t  time_stamp_stepcount = 0;
    int32_t i = 0;
    uint32_t batch_status = 0;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MUL_GET_LOG_PUNCTUATION;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("HC_MUL_GET_LOG_PUNCTUATION[%x]", res.err.udata16);
        return SNS_RC_ERR;
    }

    sensor_timestamp_report();

    ret = sns_device_read(RSLT3E, fifo_size_res, 2);
    SENSOR_N_LOG("sns_device_read-ret[%d] res1[%x] res2[%x]",ret,fifo_size_res[0],fifo_size_res[1]);
    if(SNS_RC_OK != ret){
        SENSOR_ERR_LOG("sns_device_read[%d]", ret);
        return ret;
    }
    fifo_data_size = ((fifo_size_res[1] << 8) | fifo_size_res[0]);
    SENSOR_N_LOG("sns_device_read-fifo_data_size[%x]",fifo_data_size);

    if((g_step_status == (g_step_status | STEP_COUNTER)) ||
       (g_step_status == (g_step_status | STEP_DETECTOR))){
        if((INTERRUPT_NONE == g_InterruptType) || (INTERRUPT_BUF512 == g_InterruptType))
        {
            cmd.cmd.udata16 = HC_DST_CLR_DAILYS;
            cmd.prm.ub_prm[0] = 0x01;
            cmd.prm.ub_prm[1] = 0x00;
            cmd.prm.ub_prm[2] = 0x00;
            cmd.prm.ub_prm[3] = 0x00;
            cmd.prm.ub_prm[4] = 0x00;
            cmd.prm.ub_prm[5] = 0x00;
            cmd.prm.ub_prm[6] = 0x00;
            cmd.prm.ub_prm[7] = 0x00;
            cmd.prm.ub_prm[8] = 0x00;
            cmd.prm.ub_prm[9] = 0xC0;
            cmd.prm.ub_prm[10] = 0x00;
            ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_DST_CLR_DAILYS err[%x]",res.err.udata16);
                return SNS_RC_ERR;
            }
        } else {
            cmd.cmd.udata16 = HC_DST_CLR_DAILYS;
            cmd.prm.ub_prm[0] = 0x01;
            cmd.prm.ub_prm[1] = 0x00;
            cmd.prm.ub_prm[2] = 0x00;
            cmd.prm.ub_prm[3] = 0x00;
            cmd.prm.ub_prm[4] = 0x00;
            cmd.prm.ub_prm[5] = 0x00;
            cmd.prm.ub_prm[6] = 0x00;
            cmd.prm.ub_prm[7] = 0x00;
            cmd.prm.ub_prm[8] = 0x00;
            cmd.prm.ub_prm[9] = 0x40;
            cmd.prm.ub_prm[10] = 0x00;
            ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_DST_CLR_DAILYS err[%x]",res.err.udata16);
                return SNS_RC_ERR;
            }
        }
    } else {
        if((INTERRUPT_NONE == g_InterruptType) || (INTERRUPT_BUF512 == g_InterruptType))
        {
            cmd.cmd.udata16 = HC_DST_CLR_DAILYS;
            cmd.prm.ub_prm[0] = 0x01;
            cmd.prm.ub_prm[1] = 0x00;
            cmd.prm.ub_prm[2] = 0x00;
            cmd.prm.ub_prm[3] = 0x00;
            cmd.prm.ub_prm[4] = 0x00;
            cmd.prm.ub_prm[5] = 0x00;
            cmd.prm.ub_prm[6] = 0x00;
            cmd.prm.ub_prm[7] = 0x00;
            cmd.prm.ub_prm[8] = 0x00;
            cmd.prm.ub_prm[9] = 0x80;
            cmd.prm.ub_prm[10] = 0x00;
            ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_DST_CLR_DAILYS err[%x]",res.err.udata16);
                return SNS_RC_ERR;
            }
        }
    }

    if(LOGGING_RESPONSE_HEADER > fifo_data_size) {
        SENSOR_ERR_LOG("fifo_data_size[%d]", fifo_data_size);
        return SNS_RC_ERR;
    }

    ret |= sns_device_read(FIFO, res.res.ub_res, fifo_data_size);
    if(SNS_RC_OK != ret){
        SENSOR_ERR_LOG("sns_device_read[%d]", ret);
        return ret;
    }
    rogging_data_size = ((res.res.ub_res[1] << 8) | res.res.ub_res[0]);
    SENSOR_N_LOG("rogging_data_size[%x]",rogging_data_size);

    if(rogging_data_size == 0) {
        if(g_logging_data != NULL){
            batch_p->buffer_p = g_logging_data;
        } else {
            ret = SNS_RC_ERR;
        }

        if(INTERRUPT_TIMEOUT == g_InterruptType) {
            batch_p->repo_type = SENSOR_COMP_BATCH;
        }else{
            batch_p->repo_type = SENSOR_COMP_FLUSH;
        }
        return ret;
    }

    batch_status = sensor_get_batch_status();

    time_stamp_acc = ((res.res.ub_res[5] << 24) | (res.res.ub_res[4] << 16) |
                      (res.res.ub_res[3] << 8) | res.res.ub_res[2]);
    SENSOR_N_LOG("time_stamp_acc[%x]",time_stamp_acc);
    if(0 != time_stamp_acc){
        acc_ring_buffer_timestamp(time_stamp_acc);
    }

    time_stamp_mag = ((res.res.ub_res[9] << 24) | (res.res.ub_res[8] << 16) |
                      (res.res.ub_res[7] << 8) | res.res.ub_res[6]);
    SENSOR_N_LOG("time_stamp_mag[%x]",time_stamp_mag);
    if(0 != time_stamp_mag){
        if(batch_status & (BATCH_ON << SENSOR_MAG)){
            mag_ring_buffer_timestamp(time_stamp_mag);
        }
        if(batch_status & (BATCH_ON << SENSOR_MAG_UNCAL)){
            mag_uncal_ring_buffer_timestamp(time_stamp_mag);
        }
    }

    time_stamp_gyro = ((res.res.ub_res[13] << 24) | (res.res.ub_res[12] << 16) |
                      (res.res.ub_res[11] << 8) | res.res.ub_res[10]);
    SENSOR_N_LOG("time_stamp_gyro[%x]",time_stamp_gyro);
    if(0 != time_stamp_gyro){
        if(batch_status & (BATCH_ON << SENSOR_GYRO)){
            gyro_ring_buffer_timestamp(time_stamp_gyro);
        }
        if(batch_status & (BATCH_ON << SENSOR_GYRO_UNCAL)){
            gyro_uncal_ring_buffer_timestamp(time_stamp_gyro);
        }
    }

    time_stamp_ori = ((res.res.ub_res[17] << 24) | (res.res.ub_res[16] << 16) |
                      (res.res.ub_res[15] << 8) | res.res.ub_res[14]);
    SENSOR_N_LOG("time_stamp_ori[%x]",time_stamp_ori);
    if(0 != time_stamp_ori){
        ortn_ring_buffer_timestamp(time_stamp_ori);
    }

    time_stamp_gravity = ((res.res.ub_res[21] << 24) | (res.res.ub_res[20] << 16) |
                      (res.res.ub_res[19] << 8) | res.res.ub_res[18]);
    SENSOR_N_LOG("time_stamp_gravity[%x]",time_stamp_gravity);
    if(0 != time_stamp_gravity){
        grv_ring_buffer_timestamp(time_stamp_gravity);
    }

    time_stamp_linacc = ((res.res.ub_res[25] << 24) | (res.res.ub_res[24] << 16) |
                      (res.res.ub_res[23] << 8) | res.res.ub_res[22]);
    SENSOR_N_LOG("time_stamp_linacc[%x]",time_stamp_linacc);
    if(0 != time_stamp_linacc){
        acc_lnr_ring_buffer_timestamp(time_stamp_linacc);
    }

    time_stamp_rota = ((res.res.ub_res[29] << 24) | (res.res.ub_res[28] << 16) |
                      (res.res.ub_res[27] << 8) | res.res.ub_res[26]);
    SENSOR_N_LOG("time_stamp_rota[%x]",time_stamp_rota);
    if(0 != time_stamp_rota){
        rot_vctr_ring_buffer_timestamp(time_stamp_rota);
    }

    time_stamp_gamerota = ((res.res.ub_res[33] << 24) | (res.res.ub_res[32] << 16) |
                      (res.res.ub_res[31] << 8) | res.res.ub_res[30]);
    SENSOR_N_LOG("time_stamp_gamerota[%x]",time_stamp_gamerota);
    if(0 != time_stamp_gamerota){
        game_rot_vctr_ring_buffer_timestamp(time_stamp_gamerota);
    }

    time_stamp_magrota = ((res.res.ub_res[37] << 24) | (res.res.ub_res[36] << 16) |
                      (res.res.ub_res[35] << 8) | res.res.ub_res[34]);
    SENSOR_N_LOG("time_stamp_magrota[%x]",time_stamp_magrota);
    if(0 != time_stamp_magrota){
        mag_rot_vctr_ring_buffer_timestamp(time_stamp_magrota);
    }

    time_stamp_step1 = ((res.res.ub_res[41] << 24) | (res.res.ub_res[40] << 16) |
                      (res.res.ub_res[39] << 8) | res.res.ub_res[38]);
    SENSOR_N_LOG("time_stamp_step1[%x]",time_stamp_step1);
    if(0 != time_stamp_step1){
        if(batch_status & (BATCH_ON << SENSOR_STEP_CNT)){
            step_cnt_step1_ring_buffer_timestamp(time_stamp_step1);
        }
        if(batch_status & (BATCH_ON << SENSOR_STEP_DTC)){
            step_dtc_step1_ring_buffer_timestamp(time_stamp_step1);
        }
    }

    time_stamp_step2 = ((res.res.ub_res[45] << 24) | (res.res.ub_res[44] << 16) |
                      (res.res.ub_res[43] << 8) | res.res.ub_res[42]);
    SENSOR_N_LOG("time_stamp_step2[%x]",time_stamp_step2);
    if(0 != time_stamp_step2){
        if(batch_status & (BATCH_ON << SENSOR_STEP_CNT)){
            step_cnt_step2_ring_buffer_timestamp(time_stamp_step2);
        }
        if(batch_status & (BATCH_ON << SENSOR_STEP_DTC)){
            step_dtc_step2_ring_buffer_timestamp(time_stamp_step2);
        }
    }

    time_stamp_stepcount = ((res.res.ub_res[49] << 24) | (res.res.ub_res[48] << 16) |
                      (res.res.ub_res[47] << 8) | res.res.ub_res[46]);
    SENSOR_N_LOG("time_stamp_stepcount[%x]",time_stamp_stepcount);

    stepcount_data = ((res.res.ub_res[53] << 24) | (res.res.ub_res[52] << 16) |
                      (res.res.ub_res[51] << 8) | res.res.ub_res[50]);
    SENSOR_N_LOG("stepcount_data[%x]",stepcount_data);
    if(0 != stepcount_data){
        if(batch_status & (BATCH_ON << SENSOR_STEP_CNT)){
            step_cnt_batch_ring_buffer(stepcount_data);
        }
        if(batch_status & (BATCH_ON << SENSOR_STEP_DTC)){
            step_dtc_batch_ring_buffer(stepcount_data);
        }
    }

    for(i = 0; i < (fifo_data_size - LOGGING_RESPONSE_HEADER); i++) {
        *(g_logging_data+i) = res.res.ub_res[i+LOGGING_RESPONSE_HEADER];
    }
    SENSOR_N_LOG("data[%s]",g_logging_data);

    read_data_size = (fifo_data_size - LOGGING_RESPONSE_HEADER);
    rogging_data_size -= (fifo_data_size - LOGGING_RESPONSE_HEADER);
    SENSOR_N_LOG("read_data_size[%d]-rogging_data_size[%d]",read_data_size,rogging_data_size);

    while(rogging_data_size > 0) {
        if((0 < rogging_data_size) && (rogging_data_size <= 512)) {
            cmd.cmd.udata16 = HC_MUL_GET_LOGGING_DATA;
            cmd.prm.ub_prm[0] = 0x00;
            cmd.prm.ub_prm[1] = (uint8_t)rogging_data_size;
            cmd.prm.ub_prm[2] = (uint8_t)(rogging_data_size>>8);
            ret = sns_hostcmd(&cmd, &res, rogging_data_size, EXE_HOST_ALL, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("HC_MUL_GET_LOGGING_DATA[%x]", res.err.udata16);
                return SNS_RC_ERR;
            }

            if(g_logging_data != NULL){
                memcpy((g_logging_data + read_data_size), &res.res.ub_res, rogging_data_size);
                read_data_size += rogging_data_size;
                rogging_data_size = 0;
            } else {
                SENSOR_ERR_LOG("g_logging_data:NULL!!!");
                break;
            }
        } else if(512 < rogging_data_size) {
            cmd.cmd.udata16 = HC_MUL_GET_LOGGING_DATA;
            cmd.prm.ub_prm[0] = 0x00;
            cmd.prm.ub_prm[1] = 0x00;
            cmd.prm.ub_prm[2] = 0x02;
            ret = sns_hostcmd(&cmd, &res, 512, EXE_HOST_ALL, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("HC_MUL_GET_LOGGING_DATA[%x]", res.err.udata16);
                return SNS_RC_ERR;
            }

            if(g_logging_data != NULL){
                memcpy((g_logging_data + read_data_size), &res.res.ub_res, 512);
                read_data_size += 512;
                rogging_data_size -= 512;
            } else {
                SENSOR_ERR_LOG("g_logging_data:NULL!!!");
                break;
            }
        }
    }

    if(g_logging_data != NULL){
        batch_p->buffer_p = g_logging_data;
    } else {
        ret = SNS_RC_ERR;
    }

    if(INTERRUPT_NONE == g_InterruptType) {
        batch_p->repo_type = SENSOR_COMP_FLUSH;
    } else {
        batch_p->repo_type = SENSOR_COMP_BATCH;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static void sns_batch_interrupt(void)
{
    SENSOR_N_LOG("start-Type[%d]",g_InterruptType);

    if((INTERRUPT_TIMEOUT == g_InterruptType)||(INTERRUPT_BUF512 == g_InterruptType)||
       (INTERRUPT_BUFFULL == g_InterruptType)){
        SENSOR_N_LOG("punctuation type[%d]", g_InterruptType);
        sensor_set_batch_data();
    }
    else{
        SENSOR_ERR_LOG("ERROR type[%d]", g_InterruptType);
    }

    SENSOR_N_LOG("end");
}

int32_t sns_acc_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_3axis_polling_75();
            SENSOR_N_LOG("sns_3axis_polling_75[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_acc_read_data(struct acceleration *arg_Acc)
{
    uint8_t ucBuff[MEASURE_DATA_SIZE];
    int32_t ret = SNS_RC_OK;
    int32_t raw[3];
    int32_t pos[3];
    int32_t xyz[3];
    int32_t temp;
    int i,j;

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT00, ucBuff, sizeof(ucBuff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

    if(SNS_RC_OK == ret){
        temp = (int32_t)ucBuff[0] | (((int32_t)ucBuff[1] & 0x3F) << 8);
        raw[0] = ACCDATA_SIGN_COMVERT_14_32BIT(temp);
        temp = (int32_t)ucBuff[2] | (((int32_t)ucBuff[3] & 0x3F) << 8);
        raw[1] = ACCDATA_SIGN_COMVERT_14_32BIT(temp);
        temp = (int32_t)ucBuff[4] | (((int32_t)ucBuff[5] & 0x3F) << 8);
        raw[2] = ACCDATA_SIGN_COMVERT_14_32BIT(temp);

        SENSOR_N_LOG("reg - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                     ucBuff[1],ucBuff[0],ucBuff[3],ucBuff[2],ucBuff[5],ucBuff[4]);
        SENSOR_N_LOG("raw - raw0[%04x] raw1[%04x] raw2[%04x]",
                     raw[0], raw[1], raw[2]);
    }

    for (i = 0; i < 3; i++) {
        xyz[i] = 0;
        for (j = 0; j < 3; j++){
            xyz[i] += raw[j] * u2dh_position_map[u2dh_position][i][j];
        }
        pos[i] = xyz[i];
        xyz[i] *= (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
    }

    arg_Acc->nX   = pos[0];
    arg_Acc->nY   = pos[1];
    arg_Acc->nZ   = pos[2];
    arg_Acc->outX = xyz[0];
    arg_Acc->outY = xyz[1];
    arg_Acc->outZ = xyz[2];

    SENSOR_N_LOG("arg_Acc - x[%04x] y[%04x] z[%04x]",
                     arg_Acc->nX, arg_Acc->nY, arg_Acc->nZ);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_acc_set_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;
    int32_t temp;

    SENSOR_N_LOG("start");

    temp = ACCDATA_SIGN_COMVERT_14_32BIT(offsets[0]);
    atomic_set(&g_nCalX, temp);
    SENSOR_N_LOG("set offset X[%d]",temp);
    
    temp = ACCDATA_SIGN_COMVERT_14_32BIT(offsets[1]);
    atomic_set(&g_nCalY, temp);
    SENSOR_N_LOG("set offset Y[%d]",temp);
    
    temp = ACCDATA_SIGN_COMVERT_14_32BIT(offsets[2]);
    atomic_set(&g_nCalZ, temp);
    SENSOR_N_LOG("set offset Z[%d]",temp);

    cmd.cmd.udata16 = HC_ACC_SET_CALIB;
    cmd.prm.ub_prm[0] = (uint8_t)(atomic_read(&g_nCalX) & 0x000000ff);
    cmd.prm.ub_prm[1] = (uint8_t)((atomic_read(&g_nCalX) >> 8) & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(atomic_read(&g_nCalY) & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((atomic_read(&g_nCalY) >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(atomic_read(&g_nCalZ) & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((atomic_read(&g_nCalZ) >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_CALIB err[%x]",res.err.udata16);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_acc_get_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_ACC_GET_CALIB;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_GET_CALIB err[%x]",res.err.udata16);
    }

    atomic_set(&g_nCalX, res.res.sw_res[0]);
    atomic_set(&g_nCalY, res.res.sw_res[1]);
    atomic_set(&g_nCalZ, res.res.sw_res[2]);

    offsets[0] = atomic_read(&g_nCalX);
    offsets[1] = atomic_read(&g_nCalY);
    offsets[2] = atomic_read(&g_nCalZ);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_acc_set_auto_cal_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;
    uint8_t fw_ver[4];

    SENSOR_N_LOG("start offset=%d",*offsets);

    atomic_set(&g_acc_auto_cal_offset, *offsets);

    ret = sns_get_fw_version(fw_ver);
    if( ret == SNS_RC_OK ) {
        if((fw_ver[0] >= 0x19) && (fw_ver[3] >= 0x01) ) {
            cmd.cmd.udata16 = HC_AUTO_CALIB_SET_DATA;
            cmd.prm.ub_prm[0] = 0x01;
            cmd.prm.ub_prm[1] = (uint8_t)(atomic_read(&g_acc_auto_cal_offset) & 0x000000ff);
            cmd.prm.ub_prm[2] = (uint8_t)((atomic_read(&g_acc_auto_cal_offset) >> 8) & 0x000000ff);
            ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_AUTO_CALIB_SET_DATA err[%x]",res.err.udata16);
            }
        }
    }
    else {
        SENSOR_ERR_LOG("sns_get_fw_version[%d]",ret);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_acc_get_auto_cal_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_AUTO_CALIB_GET_DATA;
    ret = sns_hostcmd(&cmd, &res, 2, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_GET_CALIB err[%x]",res.err.udata16);
    }

    *offsets = res.res.sw_res[0];
    atomic_set(&g_acc_auto_cal_offset, *offsets);

    SENSOR_N_LOG("end - return[%d] offset=%d",ret, *offsets);

    return ret;
}

int32_t sns_acc_set_host_cmd(int32_t* req_cmd, int32_t* req_param)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;
    uint32_t i;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = (uint16_t)((req_cmd[0] << 8) | req_cmd[1]);
    for(i=0;i<16;i++){
        cmd.prm.ub_prm[i] = (uint8_t)req_param[i];
    }

    ret = sns_hostcmd(&cmd, &res, 128, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("sns_acc_set_host_cmd[%x] err[%x]", cmd.cmd.udata16, res.err.udata16);
    }

    memcpy(&diag_res, &res, sizeof(HostCmdRes));

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

void sns_acc_get_host_cmd(uint32_t* res)
{
    uint32_t i;

    SENSOR_N_LOG("start");

    for(i=0;i<128;i++){
        res[i] = (uint32_t)diag_res.res.ub_res[i];
    }

    SENSOR_N_LOG("end");
}

int32_t sns_mag_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(GEOMAGNETIC, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_6axis_polling();
            SENSOR_N_LOG("sns_6axis_polling[%d]",ret);
            if(SNS_RC_OK == ret){
                g_MagGyroState = g_MagGyroState | GEOMAGNETIC;
            }
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
            if(SNS_RC_OK == ret){
                g_MagGyroState = g_MagGyroState & (~GEOMAGNETIC);
            }
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_mag_read_data(struct geomagnetic *arg_Mag)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[7];
    int32_t temp;

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT0C, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

    if(SNS_RC_OK == ret){
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0xFF) << 8);
        arg_Mag->x =MAGDATA_SIGN_COMVERT_12_32BIT(temp);

        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0xFF) << 8);
        arg_Mag->y = MAGDATA_SIGN_COMVERT_12_32BIT(temp);

        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0xFF) << 8);
        arg_Mag->z = MAGDATA_SIGN_COMVERT_12_32BIT(temp);

        arg_Mag->accuracy = buff[6];
        s_MagData.accuracy = arg_Mag->accuracy;

        SENSOR_N_LOG("mag - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x] a[%x]",
                     buff[1],buff[0],buff[3],buff[2],buff[5],buff[4],buff[6]);
        SENSOR_N_LOG("arg_Mag - x[%04x] y[%04x] z[%04x] a[%x]",
                     arg_Mag->x,arg_Mag->y,arg_Mag->z,arg_Mag->accuracy);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_mag_set_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    g_MagOffset[0] = offsets[0];
    g_MagOffset[1] = offsets[1];
    g_MagOffset[2] = offsets[2];

    cmd.cmd.udata16 = HC_MAG_SET_OFFSET;
    cmd.prm.ub_prm[0] = g_MagOffset[0];
    cmd.prm.ub_prm[1] = g_MagOffset[1];
    cmd.prm.ub_prm[2] = g_MagOffset[2];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_OFFSET err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    s_MagData.x = MAGDATA_SIGN_COMVERT_12_32BIT(offsets[3]);
    SENSOR_N_LOG("set offset X[%d]",s_MagData.x);
    
    s_MagData.y = MAGDATA_SIGN_COMVERT_12_32BIT(offsets[4]);
    SENSOR_N_LOG("set offset Y[%d]",s_MagData.y);
    
    s_MagData.z = MAGDATA_SIGN_COMVERT_12_32BIT(offsets[5]);
    SENSOR_N_LOG("set offset Z[%d]",s_MagData.z);

    cmd.cmd.udata16 = HC_MAG_SET_DATA;
    cmd.prm.ub_prm[0] = (uint8_t)(s_MagData.x & 0x000000ff);
    cmd.prm.ub_prm[1] = (uint8_t)((s_MagData.x >> 8) & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(s_MagData.y & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((s_MagData.y >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(s_MagData.z & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((s_MagData.z >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_mag_get_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MAG_GET_OFFSET;
    ret = sns_hostcmd(&cmd, &res, 3, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end GEOMAGNETIC_UNCALIB err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    g_MagOffset[0] = res.res.sb_res[0];
    g_MagOffset[1] = res.res.sb_res[1];
    g_MagOffset[2] = res.res.sb_res[2];

    offsets[0] = g_MagOffset[0];
    offsets[1] = g_MagOffset[1];
    offsets[2] = g_MagOffset[2];

    cmd.cmd.udata16 = HC_MAG_GET_DATA;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_GET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    s_MagData.x = res.res.sw_res[0];
    s_MagData.y = res.res.sw_res[1];
    s_MagData.z = res.res.sw_res[2];

    offsets[3] = s_MagData.x;
    offsets[4] = s_MagData.y;
    offsets[5] = s_MagData.z;

    SENSOR_N_LOG("offsets [%d] [%d] [%d] [%d] [%d] [%d]",
            offsets[0],offsets[1],offsets[2],offsets[3],offsets[4],offsets[5]);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_mag_set_accuracy(int8_t accuracy)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;
    struct geomagnetic mag;

    SENSOR_N_LOG("start");

    mag.x = s_MagData.x*100;
    mag.y = s_MagData.y*100;
    mag.z = s_MagData.z*100;
    mag.accuracy = accuracy;

    cmd.cmd.udata16 = HC_MUL_SET_FUSION_STATUS;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = mag.accuracy;
    cmd.prm.ub_prm[2] = (uint8_t)(mag.x & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((mag.x >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)((mag.x >> 16) & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((mag.x >> 24) & 0x000000ff);
    cmd.prm.ub_prm[6] = (uint8_t)(mag.y & 0x000000ff);
    cmd.prm.ub_prm[7] = (uint8_t)((mag.y >> 8) & 0x000000ff);
    cmd.prm.ub_prm[8] = (uint8_t)((mag.y >> 16) & 0x000000ff);
    cmd.prm.ub_prm[9] = (uint8_t)((mag.y >> 24) & 0x000000ff);
    cmd.prm.ub_prm[10] = (uint8_t)(mag.z & 0x000000ff);
    cmd.prm.ub_prm[11] = (uint8_t)((mag.z >> 8) & 0x000000ff);
    cmd.prm.ub_prm[12] = (uint8_t)((mag.z >> 16) & 0x000000ff);
    cmd.prm.ub_prm[13] = (uint8_t)((mag.z >> 24) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION_STATUS err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_mag_get_accuracy(int8_t* accuracy)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MUL_GET_FUSION_STATUS;
    ret = sns_hostcmd(&cmd, &res, 57, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_GET_FUSION_STATUS err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    *accuracy = res.res.sb_res[13];

    SENSOR_N_LOG("accuracy [%d]", *accuracy);
    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_gyro_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(GYROSCOPE, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_9axis_polling();
            SENSOR_N_LOG("sns_9axis_polling[%d]",ret);
            if(SNS_RC_OK == ret){
                g_MagGyroState = g_MagGyroState | GYROSCOPE;
            }
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
            if(SNS_RC_OK == ret){
                g_MagGyroState = g_MagGyroState & (~GYROSCOPE);
            }
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_gyro_read_data(struct gyroscope *arg_Gyro)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[6];
    int32_t temp;

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT06, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

    if(SNS_RC_OK == ret){
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0xFF) << 8);
        arg_Gyro->x =GYRODATA_SIGN_COMVERT_12_32BIT(temp);

        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0xFF) << 8);
        arg_Gyro->y = GYRODATA_SIGN_COMVERT_12_32BIT(temp);

        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0xFF) << 8);
        arg_Gyro->z = GYRODATA_SIGN_COMVERT_12_32BIT(temp);

        SENSOR_N_LOG("gyro - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                     buff[1],buff[0],buff[3],buff[2],buff[5],buff[4]);
        SENSOR_N_LOG("arg_Gyro - x[%04x] y[%04x] z[%04x]",
                     arg_Gyro->x,arg_Gyro->y,arg_Gyro->z);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_gyro_set_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;
    struct gyroscope gyro;

    SENSOR_N_LOG("start");

    s_GyroData.x = GYRODATA_SIGN_COMVERT_12_32BIT(offsets[0]);
    SENSOR_N_LOG("set offset X[%d]",s_GyroData.x);
    
    s_GyroData.y = GYRODATA_SIGN_COMVERT_12_32BIT(offsets[1]);
    SENSOR_N_LOG("set offset Y[%d]",s_GyroData.y);
    
    s_GyroData.z = GYRODATA_SIGN_COMVERT_12_32BIT(offsets[2]);
    SENSOR_N_LOG("set offset Z[%d]",s_GyroData.z);

    gyro.x = s_GyroData.x*100;
    gyro.y = s_GyroData.y*100;
    gyro.z = s_GyroData.z*100;

    cmd.cmd.udata16 = HC_MUL_SET_FUSION_STATUS;
    cmd.prm.ub_prm[0] = 0x02;
    cmd.prm.ub_prm[1] = 0x00;
    cmd.prm.ub_prm[2] = (uint8_t)(gyro.x & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((gyro.x >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)((gyro.x >> 16) & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((gyro.x >> 24) & 0x000000ff);
    cmd.prm.ub_prm[6] = (uint8_t)(gyro.y & 0x000000ff);
    cmd.prm.ub_prm[7] = (uint8_t)((gyro.y >> 8) & 0x000000ff);
    cmd.prm.ub_prm[8] = (uint8_t)((gyro.y >> 16) & 0x000000ff);
    cmd.prm.ub_prm[9] = (uint8_t)((gyro.y >> 24) & 0x000000ff);
    cmd.prm.ub_prm[10] = (uint8_t)(gyro.z & 0x000000ff);
    cmd.prm.ub_prm[11] = (uint8_t)((gyro.z >> 8) & 0x000000ff);
    cmd.prm.ub_prm[12] = (uint8_t)((gyro.z >> 16) & 0x000000ff);
    cmd.prm.ub_prm[13] = (uint8_t)((gyro.z >> 24) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION_STATUS err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_GYRO_SET_DATA;
    cmd.prm.ub_prm[0] = (uint8_t)(s_GyroData.x & 0x000000ff);
    cmd.prm.ub_prm[1] = (uint8_t)((s_GyroData.x >> 8) & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(s_GyroData.y & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((s_GyroData.y >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(s_GyroData.z & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((s_GyroData.z >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_GYRO_SET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_gyro_get_offset(int32_t* offsets)
{
    int32_t    ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_GYRO_GET_DATA;
    ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_GYRO_GET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    s_GyroData.x = res.res.sw_res[0];
    s_GyroData.y = res.res.sw_res[1];
    s_GyroData.z = res.res.sw_res[2];

    offsets[0] = s_GyroData.x;
    offsets[1] = s_GyroData.y;
    offsets[2] = s_GyroData.z;

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_maguncalib_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(GEOMAGNETIC_UNCALIB, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_6axis_polling();
            SENSOR_N_LOG("sns_6axis_polling[%d]",ret);
            if(SNS_RC_OK == ret){
                g_MagGyroState = g_MagGyroState | GEOMAGNETIC_UNCALIB;
            }
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
            if(SNS_RC_OK == ret){
                g_MagGyroState = g_MagGyroState & (~GEOMAGNETIC_UNCALIB);
            }
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_maguncalib_read_data(struct mag_uncalib *arg_Mag)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[6];
    int32_t temp;

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT0C, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

    if(SNS_RC_OK == ret){
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0xFF) << 8);
        arg_Mag->x =MAGDATA_SIGN_COMVERT_12_32BIT(temp) + s_MagData.x;

        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0xFF) << 8);
        arg_Mag->y = MAGDATA_SIGN_COMVERT_12_32BIT(temp) + s_MagData.y;

        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0xFF) << 8);
        arg_Mag->z = MAGDATA_SIGN_COMVERT_12_32BIT(temp) + s_MagData.z;

        arg_Mag->cal_x = s_MagData.x;
        arg_Mag->cal_y = s_MagData.y;
        arg_Mag->cal_z = s_MagData.z;

        SENSOR_N_LOG("mag - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                     buff[1],buff[0],buff[3],buff[2],buff[5],buff[4]);
        SENSOR_N_LOG("arg_Mag - x[%04x] y[%04x] z[%04x] calx[%04x] caly[%04x] calz[%04x]",
                     arg_Mag->x,arg_Mag->y,arg_Mag->z,arg_Mag->cal_x,arg_Mag->cal_y,arg_Mag->cal_z);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_gyrouncalib_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(GYROSCOPE_UNCALIB, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_9axis_polling();
            SENSOR_N_LOG("sns_9axis_polling[%d]",ret);
            if(SNS_RC_OK == ret){
                g_MagGyroState = g_MagGyroState | GYROSCOPE_UNCALIB;
            }
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
            if(SNS_RC_OK == ret){
                g_MagGyroState = g_MagGyroState & (~GYROSCOPE_UNCALIB);
            }
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_gyrouncalib_read_data(struct gyro_uncalib *arg_Gyro)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[6];
    int32_t temp;

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT06, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

    if(SNS_RC_OK == ret){
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0xFF) << 8);
        arg_Gyro->x =GYRODATA_SIGN_COMVERT_12_32BIT(temp) + s_GyroData.x;

        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0xFF) << 8);
        arg_Gyro->y = GYRODATA_SIGN_COMVERT_12_32BIT(temp) + s_GyroData.y;

        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0xFF) << 8);
        arg_Gyro->z = GYRODATA_SIGN_COMVERT_12_32BIT(temp) + s_GyroData.z;

        arg_Gyro->cal_x = s_GyroData.x;
        arg_Gyro->cal_y = s_GyroData.y;
        arg_Gyro->cal_z = s_GyroData.z;

        SENSOR_N_LOG("gyro - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                     buff[1],buff[0],buff[3],buff[2],buff[5],buff[4]);
        SENSOR_N_LOG("arg_Gyro - x[%04x] y[%04x] z[%04x] calx[%04x] caly[%04x] calz[%04x]",
                     arg_Gyro->x,arg_Gyro->y,arg_Gyro->z,arg_Gyro->cal_x,arg_Gyro->cal_y,arg_Gyro->cal_z);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

void sns_mag_offset_read_data(struct geomagnetic *arg_Mag)
{

    SENSOR_N_LOG("start");

    arg_Mag->x = s_MagData.x;
    arg_Mag->y = s_MagData.y;
    arg_Mag->z = s_MagData.z;

    SENSOR_N_LOG("end - x[%04x] y[%04x] z[%04x]",arg_Mag->x ,arg_Mag->y, arg_Mag->z);
}

void sns_gyro_offset_read_data(struct gyroscope *arg_Gyro)
{
    SENSOR_N_LOG("start");

    arg_Gyro->x = s_GyroData.x;
    arg_Gyro->y = s_GyroData.y;
    arg_Gyro->z = s_GyroData.z;

    SENSOR_N_LOG("end - x[%04x] y[%04x] z[%04x]",arg_Gyro->x ,arg_Gyro->y, arg_Gyro->z);
}

int32_t sns_gravity_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_9axis_polling();
            SENSOR_N_LOG("sns_9axis_polling[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_gravity_read_data(struct gravity *arg_Gravity)
{
    int32_t ret = SNS_RC_OK;
    int32_t temp;
    int32_t raw[3];
    int32_t xyz[3];
    int i,j;
    uint8_t buff[6];

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT1A, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

    if(SNS_RC_OK == ret){
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0xFF) << 8);
        raw[0] = GRADATA_SIGN_COMVERT_14_32BIT(temp);

        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0xFF) << 8);
        raw[1] = GRADATA_SIGN_COMVERT_14_32BIT(temp);

        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0xFF) << 8);
        raw[2] = GRADATA_SIGN_COMVERT_14_32BIT(temp);

        SENSOR_N_LOG("gravity - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                     buff[1],buff[0],buff[3],buff[2],buff[5],buff[4]);
        SENSOR_N_LOG("arg_Gravity raw - x[%04x] y[%04x] z[%04x]",
                     raw[0],raw[1],raw[2]);
    }

    for (i = 0; i < 3; i++) {
        xyz[i] = 0;
        for (j = 0; j < 3; j++){
            xyz[i] += raw[j] * u2dh_position_map[u2dh_position][i][j];
        }
        xyz[i] *= (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
    }

    arg_Gravity->x = xyz[0];
    arg_Gravity->y = xyz[1];
    arg_Gravity->z = xyz[2];

    SENSOR_N_LOG("arg_Gravity - x[%04x] y[%04x] z[%04x]",
                 xyz[0],xyz[1],xyz[2]);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_linacc_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_9axis_polling();
            SENSOR_N_LOG("sns_9axis_polling[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_linacc_read_data(struct linear_acceleration *arg_linacc)
{
    int32_t ret = SNS_RC_OK;
    int32_t raw[3];
    int32_t xyz[3];
    int32_t temp;
    int i,j;
    uint8_t buff[6];

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT20, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

    if(SNS_RC_OK == ret){
        temp = (int32_t)buff[0] | (((int32_t)buff[1] & 0x3F) << 8);
        raw[0] = LINACCDATA_SIGN_COMVERT_14_32BIT(temp);
        temp = (int32_t)buff[2] | (((int32_t)buff[3] & 0x3F) << 8);
        raw[1] = LINACCDATA_SIGN_COMVERT_14_32BIT(temp);
        temp = (int32_t)buff[4] | (((int32_t)buff[5] & 0x3F) << 8);
        raw[2] = LINACCDATA_SIGN_COMVERT_14_32BIT(temp);

        SENSOR_N_LOG("reg - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x]",
                     buff[1],buff[0],buff[3],buff[2],buff[5],buff[4]);
        SENSOR_N_LOG("raw - raw0[%04x] raw1[%04x] raw2[%04x]",
                     raw[0], raw[1], raw[2]);
    }

    for (i = 0; i < 3; i++) {
        xyz[i] = 0;
        for (j = 0; j < 3; j++){
            xyz[i] += raw[j] * u2dh_position_map[u2dh_position][i][j];
        }
        xyz[i] *= (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
    }

    arg_linacc->x = xyz[0];
    arg_linacc->y = xyz[1];
    arg_linacc->z = xyz[2];

    SENSOR_N_LOG("arg_linacc - x[%04x] y[%04x] z[%04x]",
                     arg_linacc->x, arg_linacc->y, arg_linacc->z);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_ori_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_9axis_polling();
            SENSOR_N_LOG("sns_9axis_polling[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_ori_read_data(struct orientation *arg_ori)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[7];

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT13, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

    if(SNS_RC_OK == ret){
        arg_ori->accuracy = buff[0];
        arg_ori->pitch = (int16_t)(buff[1] | (buff[2] << 8));
        arg_ori->roll =  (int16_t)(buff[3] | (buff[4] << 8));
        arg_ori->yaw =   (int16_t)(buff[5] | (buff[6] << 8));

        SENSOR_N_LOG("orientation - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x] a[%x]" ,
                     buff[2],buff[1],buff[4],buff[3],buff[6],buff[5],buff[0]);
        SENSOR_N_LOG("arg_ori - x[%04x] y[%04x] z[%04x] a[%x]",
                     arg_ori->pitch,arg_ori->roll,arg_ori->yaw,arg_ori->accuracy);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_rota_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_9axis_polling();
            SENSOR_N_LOG("sns_9axis_polling[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_rota_read_data(struct rotation_vector *arg_rota)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[7];

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT26, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

    if(SNS_RC_OK == ret){
        arg_rota->x = (int16_t)(buff[0] | (buff[1] << 8));
        arg_rota->y = (int16_t)(buff[2] | (buff[3] << 8));
        arg_rota->z = (int16_t)(buff[4] | (buff[5] << 8));
        arg_rota->accuracy = buff[6];

        SENSOR_N_LOG("rotation_vector - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x] a[%x]",
                     buff[1],buff[0],buff[3],buff[2],buff[5],buff[4],buff[6]);
        SENSOR_N_LOG("arg_rota - x[%04x] y[%04x] z[%04x] a[%x]",
                     arg_rota->x,arg_rota->y,arg_rota->z,arg_rota->accuracy);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_gamerota_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_9axis_polling();
            SENSOR_N_LOG("sns_9axis_polling[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_gamerota_read_data(struct game_rotation_vector *arg_gamerota)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[8];

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT36, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

    if(SNS_RC_OK == ret){
        arg_gamerota->x = (int16_t)(buff[0] | (buff[1] << 8));
        arg_gamerota->y = (int16_t)(buff[2] | (buff[3] << 8));
        arg_gamerota->z = (int16_t)(buff[4] | (buff[5] << 8));
        arg_gamerota->s = (int16_t)(buff[6] | (buff[7] << 8));

        SENSOR_N_LOG("game_rotation_vector - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x] s[%02x][%02x]",
                     buff[1],buff[0],buff[3],buff[2],buff[5],buff[4],buff[7],buff[6]);
        SENSOR_N_LOG("arg_gamerota - x[%04x] y[%04x] z[%04x] s[%04x]",
                     arg_gamerota->x,arg_gamerota->y,arg_gamerota->z,arg_gamerota->s);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_magrota_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_6axis_polling();
            SENSOR_N_LOG("sns_6axis_polling[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_magrota_read_data(struct geomagnetic_rotation_vector *arg_magrota)
{
    int32_t ret = SNS_RC_OK;
    uint8_t buff[9];

    SENSOR_N_LOG("start");

    ret = sns_device_read(RSLT2D, buff, sizeof(buff));
    SENSOR_N_LOG("sns_device_read[%d]",ret);

    if(SNS_RC_OK == ret){
        arg_magrota->accuracy = buff[0];
        arg_magrota->x = (int16_t)(buff[1] | (buff[2] << 8));
        arg_magrota->y = (int16_t)(buff[3] | (buff[4] << 8));
        arg_magrota->z = (int16_t)(buff[5] | (buff[6] << 8));
        arg_magrota->s = (int16_t)(buff[7] | (buff[8] << 8));

        SENSOR_N_LOG("mag_rotation_vector - x[%02x][%02x] y[%02x][%02x] z[%02x][%02x] s[%02x][%02x] a[%x]",
                     buff[2],buff[1],buff[4],buff[3],buff[6],buff[5],buff[8],buff[7],buff[0]);
        SENSOR_N_LOG("arg_magrota - x[%04x] y[%04x] z[%04x] s[%04x] a[%x]",
                     arg_magrota->x,arg_magrota->y,arg_magrota->z,arg_magrota->s,arg_magrota->accuracy);
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_motion_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_3axis_polling_300();
            SENSOR_N_LOG("sns_3axis_polling_300[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_motion_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_DST_EXEC_MOTION;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_EXEC_MOTION err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_scount_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(STEP_COUNTER, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_3axis_polling_300();
            SENSOR_N_LOG("sns_3axis_polling_300[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_sdetect_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(STEP_DETECTOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_3axis_polling_300();
            SENSOR_N_LOG("sns_3axis_polling_300[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_dailys_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_6axis_polling_a300m2400();
            SENSOR_N_LOG("sns_6axis_polling_a300m2400[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

void sns_set_pedo_param(uint32_t arg_iWeight, uint32_t arg_iStepWide, uint32_t arg_iVehiType)
{
    SENSOR_N_LOG("start");

    atomic_set(&g_nWeight,arg_iWeight);
    atomic_set(&g_nStepWide,arg_iStepWide);
    atomic_set(&g_nVehiType,arg_iVehiType);

    SENSOR_N_LOG("end - Weight[%d] StepWide[%d] VehiType[%d]",
                  atomic_read(&g_nStepWide), atomic_read(&g_nWeight), atomic_read(&g_nVehiType));
}

int32_t sns_dailys_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
        g_dailys_status = SNS_ON;
        param = HC_VALID;
    }else{
        g_dailys_status = SNS_OFF;
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_DST_SET_DAILYS;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = atomic_read(&g_nStepWide);
    cmd.prm.ub_prm[3] = (atomic_read(&g_nWeight) & 0xFF);
    cmd.prm.ub_prm[4] = ((atomic_read(&g_nWeight) >> 8) & 0xFF);
    cmd.prm.ub_prm[5] = atomic_read(&g_nVehiType);
    cmd.prm.ub_prm[6] = 0x00;
    cmd.prm.ub_prm[7] = param;
    cmd.prm.ub_prm[8] = 0x00;
    cmd.prm.ub_prm[9] = 0x00;
    cmd.prm.ub_prm[10] = 0x00;
    cmd.prm.ub_prm[11] = 0x00;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_SET_DAILYS err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_get_pedo_data(struct pedometer *arg_Pedo)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_DST_GET_PEDO1;
    ret = sns_hostcmd(&cmd, &res, 13, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usStepCnt  = res.res.ud_res[0];
        arg_Pedo->usWalkTime = res.res.ud_res[1];
        arg_Pedo->usCal      = res.res.ud_res[2];
        arg_Pedo->usRTState  = res.res.ub_res[12];

        SENSOR_N_LOG("Step Count         [%u]",arg_Pedo->usStepCnt);
        SENSOR_N_LOG("Walking Time(10ms) [%u]",arg_Pedo->usWalkTime);
        SENSOR_N_LOG("Calorie(kcal)      [%u]",arg_Pedo->usCal);
        SENSOR_N_LOG("RealTime State     [%u]",arg_Pedo->usRTState);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_PEDO1 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_GET_PEDO2;
    ret = sns_hostcmd(&cmd, &res, 11, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usBodyFat  = res.res.ud_res[0];
        arg_Pedo->usExercise = res.res.ud_res[1];
        arg_Pedo->usMets     = res.res.ub_res[8];
        arg_Pedo->usSpeed    = ((res.res.ub_res[10] << 8 ) | res.res.ub_res[9]);

        SENSOR_N_LOG("Body Fat(100mg)    [%u]",arg_Pedo->usBodyFat);
        SENSOR_N_LOG("Exercise(0.1Ex)    [%u]",arg_Pedo->usExercise);
        SENSOR_N_LOG("Mets               [%d]",arg_Pedo->usMets);
        SENSOR_N_LOG("Speedometer(cm/sec)[%u]",arg_Pedo->usSpeed);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_PEDO2 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_GET_PEDO5;
    ret = sns_hostcmd(&cmd, &res, 12, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usSyncRunStepCnt = res.res.ud_res[0];
        arg_Pedo->usSyncRunTime = res.res.ud_res[1];
        arg_Pedo->usSyncRunCal = res.res.ud_res[2];

        SENSOR_N_LOG("SyncRunStepCnt     [%u]",arg_Pedo->usSyncRunStepCnt);
        SENSOR_N_LOG("SyncRunTime(10ms)  [%u]",arg_Pedo->usSyncRunTime);
        SENSOR_N_LOG("SyncRunCal(kcal)   [%u]",arg_Pedo->usSyncRunCal);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_PEDO5 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_GET_RUN1;
    ret = sns_hostcmd(&cmd, &res, 9, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usRunStatus  = res.res.ub_res[0];
        arg_Pedo->usRunStepCnt = ((res.res.ub_res[4] << 24) | (res.res.uw_res[1] << 8) | res.res.ub_res[1]);
        arg_Pedo->usRunTime    = ((res.res.ub_res[8] << 24) | (res.res.uw_res[3] << 8) | res.res.ub_res[5]);

        SENSOR_N_LOG("Run Status         [%u]",arg_Pedo->usRunStatus);
        SENSOR_N_LOG("Run StepCnt        [%u]",arg_Pedo->usRunStepCnt);
        SENSOR_N_LOG("Run Time(10ms)     [%u]",arg_Pedo->usRunTime);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_RUN1 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_GET_RUN2;
    ret = sns_hostcmd(&cmd, &res, 8, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Pedo->usRunCal      = res.res.ud_res[0];
        arg_Pedo->usRunExercise = res.res.ud_res[1];

        SENSOR_N_LOG("Run Calorie(kcal)  [%u]",arg_Pedo->usRunCal);
        SENSOR_N_LOG("Run Exercise(0.1Ex)[%u]",arg_Pedo->usRunExercise);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_RUN2 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_get_vehi_data(struct vehicle *arg_Vehi)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_DST_GET_TRANS1;
    ret = sns_hostcmd(&cmd, &res, 10, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Vehi->usVehiStatus     = res.res.ub_res[0];
        arg_Vehi->usVehiKind       = res.res.ub_res[1];
        arg_Vehi->usVehiDetectTime = ((res.res.uw_res[2] << 16) | res.res.uw_res[1]);
        arg_Vehi->usVehiRideTime   = ((res.res.uw_res[4] << 16) | res.res.uw_res[3]);

        SENSOR_N_LOG("Vehi Status          [%u]",arg_Vehi->usVehiStatus);
        SENSOR_N_LOG("Vehi Kind            [%u]",arg_Vehi->usVehiKind);
        SENSOR_N_LOG("Vehi DetectTime(10ms)[%u]",arg_Vehi->usVehiDetectTime);
        SENSOR_N_LOG("Vehi RideTime(10ms)  [%u]",arg_Vehi->usVehiRideTime);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_TRANS1 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_GET_TRANS2;
    ret = sns_hostcmd(&cmd, &res, 13, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Vehi->usVehiRideCal  = res.res.ud_res[0];
        arg_Vehi->usVehiBodyFat  = res.res.ud_res[1];
        arg_Vehi->usVehiExercise = res.res.ud_res[2];
        arg_Vehi->usVehiMets     = res.res.ub_res[12];

        SENSOR_N_LOG("Vehi RideCal(kcal)   [%u]",arg_Vehi->usVehiRideCal);
        SENSOR_N_LOG("Vehi BodyFat(100mg)  [%d]",arg_Vehi->usVehiBodyFat);
        SENSOR_N_LOG("Vehi Exercise(Ex)    [%u]",arg_Vehi->usVehiExercise);
        SENSOR_N_LOG("Vehi Mets            [%u]",arg_Vehi->usVehiMets);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_TRANS2 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_DST_GET_TRANS6;
    ret = sns_hostcmd(&cmd, &res, 12, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Vehi->usVehiOtherRideTime = res.res.ud_res[0];
        arg_Vehi->usVehiBicycRideTime = res.res.ud_res[1];
        arg_Vehi->usVehiTrainRideTime = res.res.ud_res[2];

        SENSOR_N_LOG("Vehi OtherRideTime(10msec)[%u]",arg_Vehi->usVehiOtherRideTime);
        SENSOR_N_LOG("Vehi BicycRideTime(10msec)[%u]",arg_Vehi->usVehiBicycRideTime);
        SENSOR_N_LOG("Vehi TrainRideTime(10msec)[%u]",arg_Vehi->usVehiTrainRideTime);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_TRANS6 err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_iwifi_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_6axis_polling_a300m2400();
            SENSOR_N_LOG("sns_6axis_polling_a300m2400[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_iwifi_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_DST_EXEC_IWIFI;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = param;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_EXEC_IWIFI err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_get_iwifi_data(struct iwifi *arg_IWifi)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_DST_GET_INTELLI_WIFI;
    ret = sns_hostcmd(&cmd, &res, 2, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_IWifi->usPedoStatus = res.res.ub_res[0];
        arg_IWifi->usVehiStatus = res.res.ub_res[1];

        SENSOR_N_LOG("Pedo Status [%u]",arg_IWifi->usPedoStatus);
        SENSOR_N_LOG("Vehi Status [%u]",arg_IWifi->usVehiStatus);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_INTELLI_WIFI err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_iwifi_set_info(bool req, DailysSetIWifiParam *DailysIWifiParam)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_DST_SET_IWIFI_INFO;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = (DailysIWifiParam->m_nPedoStartStep & 0xFF);
    cmd.prm.ub_prm[2] = (DailysIWifiParam->m_nPedoEndTime & 0xFF);
    cmd.prm.ub_prm[3] = ((DailysIWifiParam->m_nPedoEndTime >> 8) & 0xFF);
    cmd.prm.ub_prm[4] = (DailysIWifiParam->m_nVehiStartTime & 0xFF);
    cmd.prm.ub_prm[5] = ((DailysIWifiParam->m_nVehiStartTime >> 8) & 0xFF);
    cmd.prm.ub_prm[6] = (DailysIWifiParam->m_nVehiEndTime & 0xFF);
    cmd.prm.ub_prm[7] = ((DailysIWifiParam->m_nVehiEndTime >> 8) & 0xFF);

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_SET_IWIFI_INFO err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return SNS_RC_OK;
}

int32_t sns_pedom_clear(int32_t clear_req)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;
    int32_t tmp = 0;

    SENSOR_N_LOG("start");

    tmp |= (clear_req & DAILYS_CLEAR_SHOCK_STATE)   ? 0x01:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_STOP_STATE)    ? 0x02:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_OUT_STATE)     ? 0x04:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_TRAIN_STATE)   ? 0x08:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_MOTION_STATE)  ? 0x10:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_STEP_COUNTER)  ? 0x20:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_STEP_TIMESTMP) ? 0x40:0x00;
    tmp |= (clear_req & DAILYS_CLEAR_BATCH_TIMER)   ? 0x80:0x00;

    cmd.cmd.udata16 = HC_DST_CLR_DAILYS;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = (clear_req & DAILYS_CLEAR_PEDO_DATA)  ? 0x01:0x00;
    cmd.prm.ub_prm[2] = (clear_req & DAILYS_CLEAR_PEDO_STATE) ? 0x01:0x00;
    cmd.prm.ub_prm[3] = (clear_req & DAILYS_CLEAR_VEHI_DATA)  ? 0x01:0x00;
    cmd.prm.ub_prm[4] = (clear_req & DAILYS_CLEAR_VEHI_STATE) ? 0x01:0x00;
    cmd.prm.ub_prm[5] = (clear_req & DAILYS_CLEAR_WIFI_STATE) ? 0x01:0x00;
    cmd.prm.ub_prm[6] = (clear_req & DAILYS_CLEAR_HEIGHT_STATE) ? 0x01:0x00;
    cmd.prm.ub_prm[7] = (clear_req & DAILYS_CLEAR_HEIGHT_ALL) ? 0x01:0x00;
    cmd.prm.ub_prm[8] = (clear_req & DAILYS_CLEAR_RT_STATE) ? 0x01:0x00;
    cmd.prm.ub_prm[9] = tmp;
    cmd.prm.ub_prm[10] = (clear_req & DAILYS_CLEAR_VH_STATE) ? 0x01:0x00;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end Pedometer Data Clear Error[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_vhdetect_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_3axis_polling_300();
            SENSOR_N_LOG("sns_3axis_polling_300[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_vhdetect_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_DST_EXEC_VH;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = 0x00;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_EXEC_VH err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_get_vhdetect_data(struct vhdetect *arg_VHdetect)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_DST_GET_VH;
    ret = sns_hostcmd(&cmd, &res, 10, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_VHdetect->usVhStatus = res.res.ub_res[0];

        SENSOR_N_LOG("VH Status [%d]",arg_VHdetect->usVhStatus);
    }else{
        SENSOR_ERR_LOG("end HC_DST_GET_VH err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_kc_motion_walk_start_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_3axis_polling_300();
            SENSOR_N_LOG("sns_3axis_polling_300[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_kc_motion_walk_start_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_MOT_EXEC_WALK_START;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MOT_EXEC_WALK_START err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_kc_motion_walk_stop_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_3axis_polling_300();
            SENSOR_N_LOG("sns_3axis_polling_300[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_kc_motion_walk_stop_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_MOT_EXEC_WALK_STOP;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MOT_EXEC_WALK_STOP err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_get_kc_motion_walk_data(struct kc_motion_walk_data *arg_Walk)
{
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    if( arg_Walk ) {
        arg_Walk->status = 0;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_kc_motion_train_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_6axis_polling_a300m2400();
            SENSOR_N_LOG("sns_6axis_polling_a300m2400[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_kc_motion_train_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_MOT_EXEC_TRAIN;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = 0x01;
    cmd.prm.ub_prm[3] = 0x00;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_DST_EXEC_TRAIN err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_get_kc_motion_train_data(struct kc_motion_train *arg_Train)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MOT_GET_TRAIN_INFO;
    ret = sns_hostcmd(&cmd, &res, 11, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Train->usTrFstDetect = res.res.ub_res[0];
        arg_Train->usTrOtherFstDetect = res.res.ub_res[1];
        arg_Train->usTrRes = res.res.ub_res[2];
        arg_Train->TrDtctTime = ((res.res.ub_res[3]      ) & 0x000000FF)
                              + ((res.res.ub_res[4] <<  8) & 0x0000FF00)
                              + ((res.res.ub_res[5] << 16) & 0x00FF0000)
                              + ((res.res.ub_res[6] << 24) & 0xFF000000);
        arg_Train->TrDtctTimeFix = ((res.res.ub_res[7]       ) & 0x000000FF) 
                                 + ((res.res.ub_res[8]  <<  8) & 0x0000FF00)
                                 + ((res.res.ub_res[9]  << 16) & 0x00FF0000)
                                 + ((res.res.ub_res[10] << 24) & 0xFF000000);

        SENSOR_N_LOG("First Detected       [%u]",arg_Train->usTrFstDetect);
        SENSOR_N_LOG("First Other Detected [%u]",arg_Train->usTrOtherFstDetect);
        SENSOR_N_LOG("Total Detected       [%u]",arg_Train->usTrRes);
        SENSOR_N_LOG("Detect Time          [%d]",arg_Train->TrDtctTime);
        SENSOR_N_LOG("Fix Detect Time      [%d]",arg_Train->TrDtctTimeFix);
    }else{
        SENSOR_ERR_LOG("end HC_MOT_GET_TRAIN_INFO err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_kc_motion_vehicle_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_6axis_polling_a300m2400();
            SENSOR_N_LOG("sns_6axis_polling_a300m2400[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_kc_motion_vehicle_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_MOT_EXEC_VEHICLE;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = 0x01;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MOT_EXEC_VEHICLE err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_kc_motion_bringup_activate(bool arg_iEnable)
{
    int32_t    ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    ret = sns_mag_gyro_onoff(OTHER_SENSOR, arg_iEnable);
    SENSOR_N_LOG("sns_mag_gyro_onoff[%d]",ret);

    if(SNS_RC_OK == ret){
        if(arg_iEnable == true){
            ret = sns_3axis_polling_300();
            SENSOR_N_LOG("sns_3axis_polling_300[%d]",ret);
        }else{
            ret = sns_polling_off();
            SENSOR_N_LOG("sns_polling_off[%d]",ret);
        }
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_kc_motion_bringup_start(bool arg_iStart)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t    ret = SNS_RC_OK;
    uint8_t    param;

    SENSOR_N_LOG("start");

    if(arg_iStart == true){
        param = HC_VALID;
    }else{
        param = HC_INVALID;
    }

    cmd.cmd.udata16 = HC_MOT_EXEC_BRINGUP;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = param;
    cmd.prm.ub_prm[2] = 0x05;
    cmd.prm.ub_prm[3] = 0x03;
    cmd.prm.ub_prm[4] = 0x0C;
    cmd.prm.ub_prm[5] = 0x06;
    cmd.prm.uw_prm[3] = 0x00B6;

    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MOT_EXEC_BRINGUP err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

int32_t sns_get_kc_motion_bringup_data(struct kc_motion_bringup_data *arg_Bringup)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MOT_GET_BRINGUP_INFO;
    ret = sns_hostcmd(&cmd, &res, 10, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK == ret) && (0 == res.err.udata16)) {
        arg_Bringup->status = res.res.ub_res[0];

        SENSOR_N_LOG("Bringup status [%u]",arg_Bringup->status);
    }else{
        SENSOR_ERR_LOG("end HC_MOT_GET_BRINGUP_INFO err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}



int32_t sns_set_dev_param(void)
{

    int32_t   ret;
    HostCmd cmd;
    HostCmdRes res;
    uint8_t fw_ver[4];
    oem_board_type board_type = OEM_get_board();
    oem_wireless_charge_type wc_board_type = OEM_get_wireless_charge();

    SENSOR_N_LOG("start");

    board_type &= OEM_BOARD3_TYPE;
    if( board_type == OEM_BOARD3_TYPE ) {
        cmd.cmd.udata16 = HC_ACC_SET_CONV_AXIS;
        cmd.prm.ub_prm[0] = 0x03;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_ACC_SET_CONV_AXIS err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        cmd.cmd.udata16 = HC_GYRO_SET_CONV_AXIS;
        cmd.prm.ub_prm[0] = 0x03;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_SET_CONV_AXIS err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        cmd.cmd.udata16 = HC_EC_SET_CONV_AXIS;
        cmd.prm.ub_prm[0] = 0x02;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_EC_SET_CONV_AXIS err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }
    else {
        cmd.cmd.udata16 = HC_ACC_SET_CONV_AXIS;
        cmd.prm.ub_prm[0] = 0x01;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_ACC_SET_CONV_AXIS err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        cmd.cmd.udata16 = HC_GYRO_SET_CONV_AXIS;
        cmd.prm.ub_prm[0] = 0x01;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_GYRO_SET_CONV_AXIS err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
        cmd.cmd.udata16 = HC_EC_SET_CONV_AXIS;
        cmd.prm.ub_prm[0] = 0x01;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_EC_SET_CONV_AXIS err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    cmd.cmd.udata16 = HC_ACC_SET_CALIB;
    cmd.prm.ub_prm[0] = (uint8_t)(atomic_read(&g_nCalX) & 0x000000ff);
    cmd.prm.ub_prm[1] = (uint8_t)((atomic_read(&g_nCalX) >> 8) & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(atomic_read(&g_nCalY) & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((atomic_read(&g_nCalY) >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(atomic_read(&g_nCalZ) & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((atomic_read(&g_nCalZ) >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_ACC_SET_CALIB err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    ret = sns_get_fw_version(fw_ver);
    if( ret == SNS_RC_OK ) {
        if((fw_ver[0] >= 0x19) && (fw_ver[3] >= 0x01) ) {
            cmd.cmd.udata16 = HC_AUTO_CALIB_SET_DATA;
            cmd.prm.ub_prm[0] = 0x01;
            cmd.prm.ub_prm[1] = (uint8_t)(atomic_read(&g_acc_auto_cal_offset) & 0x000000ff);
            cmd.prm.ub_prm[2] = (uint8_t)((atomic_read(&g_acc_auto_cal_offset) >> 8) & 0x000000ff);
            ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_AUTO_CALIB_SET_DATA err[%x]",res.err.udata16);
            }
        }
    }
    else {
        SENSOR_ERR_LOG("sns_get_fw_version[%d]",ret);
    }

    cmd.cmd.udata16 = HC_MAG_SET_DATA;
    cmd.prm.ub_prm[0] = (uint8_t)(s_MagData.x & 0x000000ff);
    cmd.prm.ub_prm[1] = (uint8_t)((s_MagData.x >> 8) & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(s_MagData.y & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((s_MagData.y >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(s_MagData.z & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((s_MagData.z >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_FUSION_STATUS;
    cmd.prm.ub_prm[0] = 0x00;
    cmd.prm.ub_prm[1] = 0x00;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x00;
    cmd.prm.ub_prm[5] = 0x00;
    cmd.prm.ub_prm[6] = 0x00;
    cmd.prm.ub_prm[7] = 0x00;
    cmd.prm.ub_prm[8] = 0x00;
    cmd.prm.ub_prm[9] = 0x00;
    cmd.prm.ub_prm[10] = 0x00;
    cmd.prm.ub_prm[11] = 0x00;
    cmd.prm.ub_prm[12] = 0x00;
    cmd.prm.ub_prm[13] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION_STATUS err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_FUSION_STATUS;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = s_MagData.accuracy;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x00;
    cmd.prm.ub_prm[5] = 0x00;
    cmd.prm.ub_prm[6] = 0x00;
    cmd.prm.ub_prm[7] = 0x00;
    cmd.prm.ub_prm[8] = 0x00;
    cmd.prm.ub_prm[9] = 0x00;
    cmd.prm.ub_prm[10] = 0x00;
    cmd.prm.ub_prm[11] = 0x00;
    cmd.prm.ub_prm[12] = 0x00;
    cmd.prm.ub_prm[13] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION_STATUS err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MUL_SET_FUSION_STATUS;
    cmd.prm.ub_prm[0] = 0x02;
    cmd.prm.ub_prm[1] = 0x00;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x00;
    cmd.prm.ub_prm[5] = 0x00;
    cmd.prm.ub_prm[6] = 0x00;
    cmd.prm.ub_prm[7] = 0x00;
    cmd.prm.ub_prm[8] = 0x00;
    cmd.prm.ub_prm[9] = 0x00;
    cmd.prm.ub_prm[10] = 0x00;
    cmd.prm.ub_prm[11] = 0x00;
    cmd.prm.ub_prm[12] = 0x00;
    cmd.prm.ub_prm[13] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_FUSION_STATUS err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MAG_SET_OFFSET;
    cmd.prm.ub_prm[0] = g_MagOffset[0];
    cmd.prm.ub_prm[1] = g_MagOffset[1];
    cmd.prm.ub_prm[2] = g_MagOffset[2];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_OFFSET err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_GYRO_SET_DATA;
    cmd.prm.ub_prm[0] = (uint8_t)(s_GyroData.x & 0x000000ff);
    cmd.prm.ub_prm[1] = (uint8_t)((s_GyroData.x >> 8) & 0x000000ff);
    cmd.prm.ub_prm[2] = (uint8_t)(s_GyroData.y & 0x000000ff);
    cmd.prm.ub_prm[3] = (uint8_t)((s_GyroData.y >> 8) & 0x000000ff);
    cmd.prm.ub_prm[4] = (uint8_t)(s_GyroData.z & 0x000000ff);
    cmd.prm.ub_prm[5] = (uint8_t)((s_GyroData.z >> 8) & 0x000000ff);
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_GYRO_SET_DATA err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MAG_SET_DISPERTION_THRESH;
    cmd.prm.ub_prm[0] = 0x4B;
    cmd.prm.ub_prm[1] = 0x00;
    cmd.prm.ub_prm[2] = 0x2D;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x0F;
    cmd.prm.ub_prm[5] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_DISPERTION_THRESH err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MAG_SET_FILTER;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x14;
    cmd.prm.ub_prm[2] = 0xb0;
    cmd.prm.ub_prm[3] = 0x04;
    cmd.prm.ub_prm[4] = 0xb0;
    cmd.prm.ub_prm[5] = 0x04;
    cmd.prm.ub_prm[6] = 0xb0;
    cmd.prm.ub_prm[7] = 0x04;
    cmd.prm.ub_prm[8] = 0x2c;
    cmd.prm.ub_prm[9] = 0x01;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_FILTER err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    if( wc_board_type == OEM_WLC_ON_BOARD_TYPE ) {
        cmd.cmd.udata16 = HC_MAG_SET_STATIC_MATRIX;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        cmd.prm.ub_prm[2] = 0xE2;
        cmd.prm.ub_prm[3] = 0x25;
        cmd.prm.ub_prm[4] = 0xA6;
        cmd.prm.ub_prm[5] = 0xFD;
        cmd.prm.ub_prm[6] = 0xCF;
        cmd.prm.ub_prm[7] = 0x00;
        cmd.prm.ub_prm[8] = 0x7E;
        cmd.prm.ub_prm[9] = 0xFD;
        cmd.prm.ub_prm[10] = 0xF7;
        cmd.prm.ub_prm[11] = 0x26;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_SET_STATIC_MATRIX err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }

        cmd.cmd.udata16 = HC_MAG_SET_STATIC_MATRIX;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x01;
        cmd.prm.ub_prm[2] = 0x28;
        cmd.prm.ub_prm[3] = 0x00;
        cmd.prm.ub_prm[4] = 0x6A;
        cmd.prm.ub_prm[5] = 0x03;
        cmd.prm.ub_prm[6] = 0x27;
        cmd.prm.ub_prm[7] = 0x02;
        cmd.prm.ub_prm[8] = 0x7B;
        cmd.prm.ub_prm[9] = 0x28;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_SET_STATIC_MATRIX err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }
    else {
        cmd.cmd.udata16 = HC_MAG_SET_STATIC_MATRIX;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        cmd.prm.ub_prm[2] = 0x04;
        cmd.prm.ub_prm[3] = 0x25;
        cmd.prm.ub_prm[4] = 0x5A;
        cmd.prm.ub_prm[5] = 0xFE;
        cmd.prm.ub_prm[6] = 0xC7;
        cmd.prm.ub_prm[7] = 0x00;
        cmd.prm.ub_prm[8] = 0x77;
        cmd.prm.ub_prm[9] = 0xFE;
        cmd.prm.ub_prm[10] = 0x0F;
        cmd.prm.ub_prm[11] = 0x28;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_SET_STATIC_MATRIX err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }

        cmd.cmd.udata16 = HC_MAG_SET_STATIC_MATRIX;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x01;
        cmd.prm.ub_prm[2] = 0xAF;
        cmd.prm.ub_prm[3] = 0xFF;
        cmd.prm.ub_prm[4] = 0x20;
        cmd.prm.ub_prm[5] = 0x03;
        cmd.prm.ub_prm[6] = 0x54;
        cmd.prm.ub_prm[7] = 0x00;
        cmd.prm.ub_prm[8] = 0x47;
        cmd.prm.ub_prm[9] = 0x28;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_MAG_SET_STATIC_MATRIX err[%x]",res.err.udata16);
            return SNS_RC_ERR;
        }
    }

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

int32_t sns_initialize( void )
{
    uint8_t fw_ver[4];
    uint8_t reg = 0xFF;
    int32_t cnt;
    int32_t ret = SNS_RC_OK;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    DISABLE_IRQ;

    sns_workqueue_init();

    atomic_set(&g_FusionState,SNS_OFF);
    atomic_set(&g_SnsTaskState,SNS_OFF);
    atomic_set(&g_AppTaskState,SNS_OFF);
    atomic_set(&g_FusTaskState,SNS_OFF);

    g_bDevIF_Error = false;

    g_InterruptType = INTERRUPT_NONE;

    gpio_set_value(SNS_GPIO_RST, 0);
    udelay(300);
    gpio_set_value(SNS_GPIO_RST, 1);

    msleep(20);

    msm_gpiomux_install(msm_spi_sens_configs, ARRAY_SIZE(msm_spi_sens_configs));
    msm_gpiomux_install(msm_spi_camera_configs, ARRAY_SIZE(msm_spi_camera_configs));

    cnt = 0;
    while(1) {
        sns_device_read(STATUS, &reg, sizeof(reg));
        if(reg == 0x00) {
            SENSOR_N_LOG("STATUS OK!");
            break;
        }

        msleep(10);
        ++cnt;
        if(cnt > STATUS_READ_RETRY_NUM) {
            SENSOR_ERR_LOG("STATUS read TimeOut[%x]",reg);
            return SNS_RC_ERR_TIMEOUT;
        }
    }

    reg = 0x04;
    sns_device_write(CFG, &reg, sizeof(reg));
    {
        uint8_t data = 0;
        sns_device_read(CFG, &data, 1);
        SENSOR_N_LOG("CFG read [%x]",data);
    }

    reg = 0x00;
    sns_device_write(INTMASK0, &reg, sizeof(reg));
    sns_device_write(INTMASK1, &reg, sizeof(reg));

    ENABLE_IRQ;

    cmd.cmd.udata16 = HC_MCU_SET_PDIR;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x01;
    cmd.prm.ub_prm[2] = 0x01;
    cmd.prm.ub_prm[3] = 0x01;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_SET_PDIR err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MCU_SET_PCON;
    cmd.prm.ub_prm[0] = 0x11;
    cmd.prm.ub_prm[1] = 0x11;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MCU_SET_PCON err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    ret = sns_check_sensor();
    if(ret != 0) {
        SENSOR_ERR_LOG("sns_check_sensor[%d]",ret);
    }

    ret = sns_get_fw_version(fw_ver);
    g_nFWVersion = SNESOR_COM_GET_FW_VER(fw_ver);
    if(ret != SNS_RC_OK ){
        g_nFWVersion = SNESOR_COM_FW_VER_NONE;
        SENSOR_ERR_LOG("sns_get_fw_version[%d]",ret);
    }

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

static irqreturn_t sns_irq_handler(int32_t irq, void *dev_id)
{

    SENSOR_N_LOG("start");

    if( irq != g_nIntIrqNo ){
        return IRQ_NONE;
    }

    DISABLE_IRQ;
    if( sns_workqueue_create(sns_wq_int, sns_int_work_func) != SNS_RC_OK){
        ENABLE_IRQ;
    }else{
        SENSOR_N_LOG("### --> s_tWork_Int");
    }

    SENSOR_N_LOG("end - IRQ_HANDLED");

    return IRQ_HANDLED;
}

static void sns_int_work_func(struct work_struct *work)
{
    Word sreg;
    Long lreg;

    u_int8_t sreg30, sreg31, sreg32, sreg33;
    u_int8_t sreg34, sreg35, sreg36, sreg37;

    SENSOR_N_LOG("start");

    memset( &sreg, 0x00, sizeof(lreg));

    sns_device_read(ERROR0, lreg.udata8, 4);
    sreg.udata16 = 0x0000;
    sns_device_write(CMD0, sreg.udata8, 2);

    if(lreg.udata16[1] == 0){
        sns_workqueue_delete(work) ;
        ENABLE_IRQ;
        return;
    }

    SENSOR_N_LOG("### INTREQ0/1[%x],ERROR0/1[%x]",
                                    lreg.udata16[0],lreg.udata16[1]);

    g_lastCmdError = lreg.udata16[0];

    if(lreg.udata16[1] == 0xFFFF){
        SENSOR_ERR_LOG("### sns_int_work_func MiconWDT Error[%x][%x]",
                                    lreg.udata16[0],lreg.udata16[1]);
        g_bDevIF_Error = true;
        sns_workqueue_delete(work);

        DISABLE_IRQ;
        return;
    }

    if(lreg.udata16[1] == INTREQ_ERROR){
        SENSOR_ERR_LOG("### sns_int_work_func Error[%x][%x]",
                                    lreg.udata16[0],lreg.udata16[1]);
        mutex_lock(&s_tDataMutex);
        g_nIntIrqFlg |= INTREQ_ERROR;
        mutex_unlock(&s_tDataMutex);

        wake_up_interruptible(&s_tWaitInt);

        sns_workqueue_delete(work);
        ENABLE_IRQ;
        return;
    }

    if(lreg.udata16[1] & INTREQ_HOST_CMD){
        if(!(g_nIntIrqFlg & INTREQ_HOST_CMD)){
            mutex_lock(&s_tDataMutex);
            g_nIntIrqFlg |= INTREQ_HOST_CMD;
            mutex_unlock(&s_tDataMutex);
            wake_up_interruptible(&s_tWaitInt);
        }
        SENSOR_N_LOG("### INTREQ_HOST_CMD[%x]",g_nIntIrqFlg);
    }

    if(lreg.udata16[1] & (INTREQ_ACC | INTREQ_MAG | INTREQ_GYRO | INTREQ_FUSION | INTREQ_KCLOG)){
        SENSOR_N_LOG("### INTREQ_ACC | INTREQ_MAG | INTREQ_GYRO | INTREQ_FUSION | INTREQ_KCLOG");
        atomic_set(&g_IntreqAcc,true);
        if(lreg.udata16[1] & INTREQ_KCLOG){
            atomic_set(&g_MiconDebug, true);
        }
        sns_workqueue_create(sns_wq, sns_int_app_work_func);
    }


    if(lreg.udata16[1]  & INTREQ_NMI){
        SENSOR_ERR_LOG("### INTREQ_NMI");

        memset( &sreg30, 0x00, sizeof(sreg30));
        memset( &sreg31, 0x00, sizeof(sreg31));
        memset( &sreg32, 0x00, sizeof(sreg32));
        memset( &sreg33, 0x00, sizeof(sreg33));
        memset( &sreg34, 0x00, sizeof(sreg34));
        memset( &sreg35, 0x00, sizeof(sreg35));
        memset( &sreg36, 0x00, sizeof(sreg36));
        memset( &sreg37, 0x00, sizeof(sreg37));

        sns_device_read(RSLT30, &sreg30, 1);
        sns_device_read(RSLT31, &sreg31, 1);
        sns_device_read(RSLT32, &sreg32, 1);
        sns_device_read(RSLT33, &sreg33, 1);
        sns_device_read(RSLT34, &sreg34, 1);
        sns_device_read(RSLT35, &sreg35, 1);
        sns_device_read(RSLT36, &sreg36, 1);
        sns_device_read(RSLT37, &sreg37, 1);



        SENSOR_ERR_LOG("### result reg 33-30(LR):[%02x][%02x][%02x][%02x]"
                                         ,sreg33, sreg32, sreg31, sreg30);
        SENSOR_ERR_LOG("### result reg 34:[%x], 35:[%x], 36:[%x], 37:[%x]"
                                         ,sreg34, sreg35, sreg36, sreg37);
    }

    if(lreg.udata16[1]  & INTREQ_EXCP){
        SENSOR_ERR_LOG("### INTREQ_EXCP");

        memset( &sreg30, 0x00, sizeof(sreg30));
        memset( &sreg31, 0x00, sizeof(sreg31));
        memset( &sreg32, 0x00, sizeof(sreg32));
        memset( &sreg33, 0x00, sizeof(sreg33));
        memset( &sreg34, 0x00, sizeof(sreg34));

        sns_device_read(RSLT30, &sreg30, 1);
        sns_device_read(RSLT31, &sreg31, 1);
        sns_device_read(RSLT32, &sreg32, 1);
        sns_device_read(RSLT33, &sreg33, 1);
        sns_device_read(RSLT34, &sreg34, 1);

        SENSOR_ERR_LOG("### result reg 33-30(LR):[%02x][%02x][%02x][%02x]"
                                         ,sreg33, sreg32, sreg31, sreg30);
        SENSOR_ERR_LOG("### result reg 34:[%02x]",sreg34);
    }


    if(lreg.udata16[1]  & INTREQ_KCLOG){
        SENSOR_ERR_LOG("### INTREQ_KCLOG");

        memset( &sreg30, 0x00, sizeof(sreg30));
        memset( &sreg31, 0x00, sizeof(sreg31));
        memset( &sreg32, 0x00, sizeof(sreg32));
        memset( &sreg33, 0x00, sizeof(sreg33));
        memset( &sreg34, 0x00, sizeof(sreg34));

        sns_device_read(RSLT30, &sreg30, 1);
        sns_device_read(RSLT31, &sreg31, 1);
        sns_device_read(RSLT32, &sreg32, 1);
        sns_device_read(RSLT33, &sreg33, 1);
        sns_device_read(RSLT34, &sreg34, 1);
 
        SENSOR_ERR_LOG("### result reg 33-30(LR):[%02x][%02x][%02x][%02x]"
                                         ,sreg33, sreg32, sreg31, sreg30);
        SENSOR_ERR_LOG("### result reg 34:[%02x]",sreg34);


    }
    sns_workqueue_delete(work) ;
    ENABLE_IRQ;

    SENSOR_N_LOG("end");

    return;
}

static void sns_int_app_work_func(struct work_struct *work)
{
    HostCmd            cmd;
    HostCmdRes         res;
    int32_t            ret = SNS_RC_OK;
    uint8_t            vehi_detect = 0;
    uint8_t            wifi_detect = 0;
    uint8_t            apl_detect = 0;
    uint8_t            apl_detect2 = 0;
    uint8_t            apl_detect3 = 0;
    Word               log_size;
    uint8_t            count_10, count, offset, i, j;
    char               strLog[50], temp[4];

    SENSOR_N_LOG("start");

    if(atomic_read(&g_IntreqAcc) == true) {
        cmd.cmd.udata16 = HC_DST_GET_INT_DETAIL;
        ret = sns_hostcmd(&cmd, &res, 10, EXE_HOST_ALL, READ_FIFO);
        if(ret != SNS_RC_OK) {
            SENSOR_ERR_LOG("###  Error HC_DST_GET_INT_DETAIL");
            sns_workqueue_delete(work) ;
            return;
        }

        SENSOR_N_LOG("HC_DST_GET_INT_DETAIL 0:[%x] 1:[%x] 2:[%x] 3:[%x] 4:[%x] 5:[%x] 6:[%x] 7:[%x] 8:[%x] 9:[%x]\n",
                   res.res.ub_res[0],res.res.ub_res[1],res.res.ub_res[2],res.res.ub_res[3],
                   res.res.ub_res[4],res.res.ub_res[5],res.res.ub_res[6],res.res.ub_res[7],res.res.ub_res[8],res.res.ub_res[9]);

        if(res.res.ub_res[0] == 0x01){
            SENSOR_N_LOG("### Step Detected !!");
        }

        if(res.res.ub_res[1] == 0x01){
            vehi_detect = 0x01;
            SENSOR_N_LOG("### Vehicle Detected !!");
            sensor_interrupt(SENSOR_EXT_VEHI,0);
        }

        if(res.res.ub_res[3] == 0x01){
            wifi_detect = 0x01;
            SENSOR_N_LOG("### Intelli Wifi Detected !!");
            sensor_interrupt(SENSOR_EXT_IWIFI,0);
        }

        if(res.res.ub_res[7] & 0x02){
            apl_detect |= 0x02;
            SENSOR_N_LOG("### Walk Stop Detected !!");
            sensor_interrupt(SENSOR_KC_MOTION_WALK_STOP,0);
        }

        if(res.res.ub_res[7] & 0x04){
            apl_detect |= 0x04;
            SENSOR_N_LOG("### Bringup Detected !!");
            sensor_interrupt(SENSOR_KC_MOTION_BRINGUP,0);
        }

        if(res.res.ub_res[7] & 0x08){
            apl_detect |= 0x08;
            SENSOR_N_LOG("### Train Detected !!");
            sensor_interrupt(SENSOR_KC_MOTION_TRAIN,0);
        }

        if(res.res.ub_res[7] & 0x10){
            apl_detect |= 0x10;
            SENSOR_N_LOG("### First Train Detected !!");
            sensor_interrupt(SENSOR_KC_MOTION_TRAIN,0);
        }

        if(res.res.ub_res[7] & 0x20){
            apl_detect |= 0x20;
            SENSOR_N_LOG("### Motion Detected !!");
            sensor_interrupt(SENSOR_SGNFCNT_MTN,0);
        }

        if(res.res.ub_res[7] & 0x40){
            apl_detect |= 0x40;
            SENSOR_N_LOG("### Batch Timer !!");
            g_InterruptType = INTERRUPT_TIMEOUT;
            sns_batch_interrupt();
        }

        if(res.res.ub_res[7] & 0x80){
            apl_detect |= 0x80;
            SENSOR_N_LOG("### VH Detected !!");
            sensor_interrupt(SENSOR_EXT_VH,0);
        }

        if(res.res.ub_res[8] & 0x01){
            apl_detect2 |= 0x01;
            SENSOR_N_LOG("### Walk Start Detected !!");
            sensor_interrupt(SENSOR_KC_MOTION_WALK_START,0);
        }

        if(res.res.ub_res[8] & 0x02){
            apl_detect2 |= 0x02;
            SENSOR_N_LOG("### Vehicle Start Detected !!");
            sensor_interrupt(SENSOR_KC_MOTION_VEHICLE,0);
        }

        if(res.res.ub_res[9] & 0x01){
            apl_detect3 |= 0x01;
            SENSOR_N_LOG("### Acc AutoCalibration Detected !!");
            sensor_interrupt(SENSOR_ACC_AUTO_CAL,0);
        }

        cmd.cmd.udata16 = HC_MCU_GET_INT_DETAIL;
        cmd.prm.ub_prm[0] = 0xc0;
        cmd.prm.ub_prm[1] = 0x40;
        ret = sns_hostcmd(&cmd, &res, 8, EXE_HOST_ALL, READ_FIFO);
        if(ret != SNS_RC_OK) {
            SENSOR_ERR_LOG("###  Error HC_MCU_GET_INT_DETAIL");
            sns_workqueue_delete(work) ;
            return;
        }

        if(res.res.ub_res[0] == 0x01){
            SENSOR_N_LOG("### Geomagnetic Detected !!");
            cmd.cmd.udata16 = HC_MAG_GET_DATA;
            ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_MAG_GET_DATA err[%x]",res.err.udata16);
                sns_workqueue_delete(work);
                return;
            } else {
                s_MagData.x = res.res.sw_res[0];
                s_MagData.y = res.res.sw_res[1];
                s_MagData.z = res.res.sw_res[2];

            SENSOR_N_LOG("mag_data - x[%d] y[%d] z[%d]",s_MagData.x,s_MagData.y,s_MagData.z);
            }
        }

        if(res.res.ub_res[2] == 0x01){
            SENSOR_N_LOG("### Gyroscope Detected !!");
            cmd.cmd.udata16 = HC_GYRO_GET_DATA;
            ret = sns_hostcmd(&cmd, &res, 6, EXE_HOST_ALL, READ_FIFO);
            if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
                SENSOR_ERR_LOG("end HC_GYRO_GET_DATA err[%x]",res.err.udata16);
                sns_workqueue_delete(work);
                return;
            } else {
                s_GyroData.x = res.res.sw_res[0];
                s_GyroData.y = res.res.sw_res[1];
                s_GyroData.z = res.res.sw_res[2];

                SENSOR_N_LOG("gyro_data - x[%d] y[%d] z[%d]",s_GyroData.x,s_GyroData.y,s_GyroData.z);
            }
        }

        if(res.res.ub_res[4] == 0x01){
            SENSOR_N_LOG("### Buffer 512Byte !!");
            g_InterruptType = INTERRUPT_BUF512;
            sns_batch_interrupt();
        }

        if(res.res.ub_res[4] == 0x02){
            SENSOR_N_LOG("### Buffer Full !!");
            g_InterruptType = INTERRUPT_BUFFULL;
            sns_batch_interrupt();
        }

        cmd.cmd.udata16 = HC_DST_CLR_INT_DETAIL;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        cmd.prm.ub_prm[2] = vehi_detect;
        cmd.prm.ub_prm[3] = 0x00;
        cmd.prm.ub_prm[4] = wifi_detect;
        cmd.prm.ub_prm[5] = 0x00;
        cmd.prm.ub_prm[6] = 0x00;
        cmd.prm.ub_prm[7] = apl_detect;
        cmd.prm.ub_prm[8] = apl_detect2;
        cmd.prm.ub_prm[9] = apl_detect3;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if(SNS_RC_OK != ret) {
            SENSOR_ERR_LOG("end HC_DST_CLR_INT_DETAIL err[%x]",res.err.udata16);
            sns_workqueue_delete(work) ;
            return;
        }


        if(atomic_read(&g_MiconDebug) == true){
            SENSOR_ERR_LOG("###Micon Debug ");

            cmd.cmd.udata16 = KC_LOG_READ;
            sns_hostcmd(&cmd, &res, KC_LOG_SIZE, EXE_HOST_ALL, READ_FIFO);
            log_size.udata16 = ( (res.res.ub_res[0]) | (res.res.ub_res[1] << 8));

            count_10 = log_size.udata16 / 10;
            count = log_size.udata16 % 10;
            offset = 0;

            SENSOR_ERR_LOG("###log_size:[%d] ", log_size.udata16);

            for(i = 0; i < count_10 ; i++)
            {
                SENSOR_ERR_LOG("###[%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x]",
                                   res.res.ub_res[0+offset], res.res.ub_res[1+offset], res.res.ub_res[2+offset],
                                   res.res.ub_res[3+offset], res.res.ub_res[4+offset], res.res.ub_res[5+offset],
                                   res.res.ub_res[6+offset], res.res.ub_res[7+offset], res.res.ub_res[8+offset],
                                   res.res.ub_res[9+offset] );
                offset += 10;
            }

            for(j = 0; j < count ; j++)
            {
                sprintf(temp, "[%02x]", res.res.ub_res[j + (10 * count_10)] );
                strcat(strLog, temp);
            }
            SENSOR_ERR_LOG("###%s ", strLog);

            sns_workqueue_delete(work);
            return;
        }

    } else {
        g_InterruptType = INTERRUPT_NONE;
        sensor_set_batch_data();

        cmd.cmd.udata16 = HC_DST_CLR_DAILYS;
        cmd.prm.ub_prm[0] = 0x01;
        cmd.prm.ub_prm[1] = 0x00;
        cmd.prm.ub_prm[2] = 0x00;
        cmd.prm.ub_prm[3] = 0x00;
        cmd.prm.ub_prm[4] = 0x00;
        cmd.prm.ub_prm[5] = 0x00;
        cmd.prm.ub_prm[6] = 0x00;
        cmd.prm.ub_prm[7] = 0x00;
        cmd.prm.ub_prm[8] = 0x00;
        cmd.prm.ub_prm[9] = 0x80;
        cmd.prm.ub_prm[10] = 0x00;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
            SENSOR_ERR_LOG("end HC_DST_CLR_DAILYS err[%x]",res.err.udata16);
            sns_workqueue_delete(work);
            return;
        }
    }

    sns_workqueue_delete(work) ;

    SENSOR_N_LOG("end");

    return;
}

void sns_flush_batch(void)
{
    SENSOR_N_LOG("start");

    atomic_set(&g_IntreqAcc,false);
    sns_workqueue_create(sns_wq, sns_int_app_work_func);

    SENSOR_N_LOG("end");
}

static int32_t sns_gpio_init(void)
{
    int32_t ret;

    SENSOR_N_LOG("start");

    ret = gpio_request(SNS_GPIO_RST, GPIO_RESET_NAME);
    if (ret < 0){
        SENSOR_ERR_LOG("failed to gpio_request SNS_GPIO_RST[%d]",ret);
        return ret;
    }

    ret = gpio_direction_output(SNS_GPIO_RST, 1);
    if (ret < 0){
        SENSOR_ERR_LOG("failed to gpio_direction_output SNS_GPIO_RST[%d]",ret);
        goto ERROR_RST;
    }

    ret = gpio_request(SNS_GPIO_BRMP, GPIO_BRMP_NAME);
    if (ret < 0){
        SENSOR_ERR_LOG("failed to gpio_request SNS_GPIO_BRMP[%d]",ret);
        goto ERROR_RST;
    }

    ret = gpio_direction_output(SNS_GPIO_BRMP, 0);
    if (ret < 0){
        SENSOR_ERR_LOG("failed to gpio_direction_output SNS_GPIO_BRMP[%d]",ret);
        goto ERROR_BRMP;
    }

    g_nIntIrqNo = gpio_to_irq(SNS_GPIO_INT);
    atomic_set(&g_bIsIntIrqEnable, true);
    ret = gpio_request(SNS_GPIO_INT, GPIO_INT_NAME);
    if (ret < 0){
        SENSOR_ERR_LOG("failed to gpio_request SNS_GPIO_INT[%d]",ret);
        goto ERROR_BRMP;
    }

    ret = gpio_direction_input(SNS_GPIO_INT);
    if (ret < 0){
        SENSOR_ERR_LOG("failed to gpio_direction_output SNS_GPIO_INT[%d]",ret);
        goto ERROR_INT;
    }

    ret = request_any_context_irq(g_nIntIrqNo, sns_irq_handler, IRQF_TRIGGER_LOW, GPIO_INT_NAME, NULL);
    if(ret < 0) {
        SENSOR_ERR_LOG("Failed request_any_context_irq[%d]",ret);
        goto ERROR_INT;
    }

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;

ERROR_INT:
    gpio_free(SNS_GPIO_INT);
ERROR_BRMP:
    gpio_free(SNS_GPIO_BRMP);
ERROR_RST:
    gpio_free(SNS_GPIO_RST);
    return SNS_RC_ERR;
}

int32_t sns_check_sensor(void)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MCU_GET_EX_SENSOR;
    ret = sns_hostcmd(&cmd, &res, 1, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("CMD Error <HC_MCU_GET_EX_SENSOR>[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    if( !(res.res.ub_res[0] & 0x01) ){
        SENSOR_ERR_LOG("Acc Sensor Not found[%x]",res.res.ub_res[0]);
        return SNS_RC_ERR;
    }
    if( !(res.res.ub_res[0] & 0x40) ){
        SENSOR_ERR_LOG("Mag Sensor Not found[%x]",res.res.ub_res[0]);
        return SNS_RC_ERR;
    }
    if( !(res.res.ub_res[0] & 0x80) ){
        SENSOR_ERR_LOG("Gyro Sensor Not found[%x]",res.res.ub_res[0]);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

int32_t sns_get_fw_version(uint8_t *arg_iData)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MCU_GET_VERSION;

    ret = sns_hostcmd(&cmd, &res, 8, (EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER), READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("FW Version Get Error[%d][%x]",cmd.prm.ub_prm[0], res.err.udata16);
        return SNS_RC_ERR;
    }

    arg_iData[0] = res.res.ub_res[0];
    arg_iData[1] = res.res.ub_res[1];
    arg_iData[2] = res.res.ub_res[2];
    arg_iData[3] = res.res.ub_res[3];

    SENSOR_N_LOG("FW Version[%02x][%02x][%02x][%02x]",arg_iData[0], arg_iData[1], arg_iData[2], arg_iData[3]);

    SENSOR_N_LOG("end - SNS_RC_OK");

    return SNS_RC_OK;
}

void sns_BRMP_direction(void)
{
    int32_t    ret;

    SENSOR_N_LOG("start");

    ret = gpio_direction_input(SNS_GPIO_BRMP);

    if(ret < 0 ){
        SENSOR_ERR_LOG("Error BRMP CTL");
    }

    SENSOR_N_LOG("end");
}

static void sns_FW_BRMP_ctrl(void)
{
    SENSOR_N_LOG("start");

    gpio_set_value(SNS_GPIO_BRMP, 1);

    gpio_set_value(SNS_GPIO_RST, 0);

    udelay(510);

    gpio_set_value(SNS_GPIO_RST, 1);

    msleep(20);

    gpio_set_value(SNS_GPIO_BRMP, 0);

    SENSOR_N_LOG("end");
}

int32_t sns_update_fw_seq(uint8_t *arg_iData, uint32_t arg_iLen)
{
    int32_t    ret = SNS_RC_OK;
    uint8_t    fw_ver[4];
    HostCmd    cmd;
    HostCmdRes res;
    int32_t    i;

    SENSOR_N_LOG("start");

    atomic_set(&g_FWUpdateStatus,true);
    
    ret = sns_get_fw_version(fw_ver);
    g_nFWVersion = SNESOR_COM_GET_FW_VER(fw_ver);
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Get Version Error!!");
        g_nFWVersion = SNESOR_COM_FW_VER_NONE;
    }

    SENSOR_N_LOG("Now[%x] Base[%x]",g_nFWVersion, SNESOR_COM_FW_VER_DATA);

    if(g_nFWVersion != SNESOR_COM_FW_VER_DATA){
        SENSOR_N_LOG("Need to update F/W Version");

        cmd.cmd.udata16 = HC_MCU_SELF_CHK_FW;
        ret = sns_hostcmd(&cmd, &res, 1, EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER, READ_FIFO);
        SENSOR_N_LOG("HC_MCU_SELF_CHK_FW[%d] err res[%x] err[%x]", ret, res.res.ub_res[0], res.err.udata16);
        if(1) {
            SENSOR_N_LOG("F/W Update Start...");
            DISABLE_IRQ;
            for(i = 0; i < SNS_SPI_RETRY_NUM; i++){
                sns_FW_BRMP_ctrl();
                ret = sns_update_fw_exe(true, arg_iData, arg_iLen);
                if(ret != SNS_RC_ERR_RAMWRITE)
                    break;
            }
        }else if(0x00 == res.res.ub_res[0]){
            SENSOR_N_LOG("HC_MCU_SELF_CHK_FW(-) OK!!");
        }
        SENSOR_N_LOG("F/W Initialize Start...");
        ret |= sns_initialize();
        ret |= sns_set_dev_param();
    }else{
        SENSOR_N_LOG("None update F/W Version");
    }
    SENSOR_N_LOG("F/W Update Check Completed...");

    ret |= gpio_direction_input(SNS_GPIO_BRMP);

    atomic_set(&g_FWUpdateStatus,false);

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_update_fw(uint8_t *arg_iData, uint32_t arg_iLen)
{
    int32_t    ret = SNS_RC_OK;
    uint32_t   i;

    SENSOR_N_LOG("start");

    atomic_set(&g_FWUpdateStatus,true);

    ret = sns_update_fw_exe(false,arg_iData, arg_iLen);
    if(ret == SNS_RC_ERR_RAMWRITE){
        ret = gpio_direction_output(SNS_GPIO_BRMP, 0);
        if (ret < 0){
            SENSOR_ERR_LOG("failed to gpio_request(output) ret[%d]",ret);
            atomic_set(&g_FWUpdateStatus,false);
            return SNS_RC_ERR;
        }
        DISABLE_IRQ;
        for(i = 0; i < SNS_SPI_RETRY_NUM; i++){
            sns_FW_BRMP_ctrl();
            ret = sns_update_fw_exe(true, arg_iData, arg_iLen);
            if(ret != SNS_RC_ERR_RAMWRITE) {
                break;
            }
        }
        ret |= gpio_direction_input(SNS_GPIO_BRMP);
    }

    atomic_set(&g_FWUpdateStatus,false);

    if(ret != 0){
        SENSOR_ERR_LOG("error(sns_update_fw_exe) : fw_update");
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_update_fw_exe(bool boot, uint8_t *arg_iData, uint32_t arg_iLen)
{
    uint8_t reg = 0xFF;
    int32_t i;
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;
    int32_t size;
    int32_t send_num = 0;
    uint32_t chksum = 0;
    uint8_t chksum_data[4] = {0x00, 0x00, 0x00, 0x00};
    uint8_t    fw_ver[4];
    int cnt = 170;

    SENSOR_N_LOG("start");

    memset(&res, 0x00, sizeof(HostCmdRes));

    SENSOR_N_LOG("boot[%d] Data[%x] Len[%d]", boot, (int)arg_iData, arg_iLen);

    if((arg_iData == NULL) || (arg_iLen == 0)){
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    ENABLE_IRQ;
    if(!boot){
        cmd.cmd.udata16 = HC_MCU_FUP_START;
        cmd.prm.ub_prm[0] = 0x55;
        cmd.prm.ub_prm[1] = 0xAA;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if(ret != SNS_RC_OK) {
            SENSOR_ERR_LOG("Communication Error!");
            DISABLE_IRQ;
            return SNS_RC_ERR;
        }
        if(res.err.udata16 == ERROR_FUP_CERTIFICATION) {
            SENSOR_ERR_LOG("Certification Error!");
            DISABLE_IRQ;
            return SNS_RC_ERR;
        }
    }

    msleep(30);

    reg = 0x04;
    sns_device_write(CFG, &reg, sizeof(reg));
    {
        uint8_t data = 0;
        sns_device_read(CFG, &data, 1);
        SENSOR_N_LOG("CFG read [%x]",data);
    }

    reg = 0x00;
    sns_device_write(INTMASK0, &reg, sizeof(reg));
    sns_device_write(INTMASK1, &reg, sizeof(reg));

    cmd.cmd.udata16 = HC_MCU_SET_PCON;
    cmd.prm.ub_prm[0] = 0x11;
    cmd.prm.ub_prm[1] = 0x11;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x00;
    cmd.prm.ub_prm[5] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_RSLT);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("PortSettint err[%x]", res.err.udata16);
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("Check Firmware Mode.");
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;
    ret = sns_hostcmd(&cmd, &res, 8, EXE_HOST_ALL, READ_RSLT);
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("Communication Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    if(res.res.ub_res[2] != 0x01){
        SENSOR_ERR_LOG("Version check Error!");
        SENSOR_N_LOG("FW Version[%02x] [%02x] [%02x] [%02x]",
                      res.res.ub_res[0], res.res.ub_res[1], res.res.ub_res[2], res.res.ub_res[3]);
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("FW Version[%02x] [%02x] [%02x] [%02x]",
                  res.res.ub_res[0], res.res.ub_res[1], res.res.ub_res[2], res.res.ub_res[3]);

    SENSOR_N_LOG("Flash Clear.");
    cmd.cmd.udata16 = HC_MCU_FUP_ERASE;
    cmd.prm.ub_prm[0] = 0xAA;
    cmd.prm.ub_prm[1] = 0x55;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_RSLT);
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("Communication Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    if(res.err.udata16 == ERROR_FUP_CERTIFICATION) {
        SENSOR_ERR_LOG("Certification Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    while(cnt > 0) {
        sns_device_read(STATUS, &reg, sizeof(reg));
        if(reg == 0x00) {
            SENSOR_N_LOG("STATUS OK!!");
            break;
        } else {
            msleep(10);
            cnt--;
        }
    }

    if(cnt <= 0){
        SENSOR_ERR_LOG("Flash Clear STATUS Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }

    send_num = arg_iLen / FUP_MAX_RAMSIZE;
    send_num = (arg_iLen % FUP_MAX_RAMSIZE) ? send_num+1 : send_num;

    for(i=0; i < arg_iLen; i++){
        chksum += (uint32_t)arg_iData[i];
    }
    for(i=0; i < send_num; i++){
        if((arg_iLen - (FUP_MAX_RAMSIZE * i)) >= FUP_MAX_RAMSIZE){
            size = FUP_MAX_RAMSIZE;
        } else {
            size = arg_iLen % FUP_MAX_RAMSIZE;
        }
        ret = sns_spi_ram_write_proc(FIFO, &arg_iData[i * FUP_MAX_RAMSIZE], size);
        if(ret != SNS_RC_OK) {
            SENSOR_ERR_LOG("RAM Write Error!");
            DISABLE_IRQ;
            return SNS_RC_ERR_RAMWRITE;
        }
        cmd.cmd.udata16 = HC_MCU_FUP_WRITE_FIFO;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_RSLT);
        if(ret != SNS_RC_OK) {
            SENSOR_ERR_LOG("Communication Error!");
            DISABLE_IRQ;
            return SNS_RC_ERR;
        }
    }

    SENSOR_N_LOG("Self Check.");
    chksum_data[3] = (uint8_t)((chksum >> 24) & 0x000000ff);
    chksum_data[2] = (uint8_t)((chksum >> 16) & 0x000000ff);
    chksum_data[1] = (uint8_t)((chksum >> 8) & 0x000000ff);
    chksum_data[0] = (uint8_t)(chksum & 0x000000ff);
    ret = sns_device_write(FIFO, chksum_data, sizeof(chksum_data));
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("chksum FIFO Write Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR_RAMWRITE;
    }
    cmd.cmd.udata16 = HC_MCU_FUP_WRITE_FIFO;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_RSLT);
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("Communication Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }
    cmd.cmd.udata16 = HC_MCU_SELF_CHK_FW;
    ret = sns_hostcmd(&cmd, &res, 1, EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER, READ_RSLT);
    SENSOR_N_LOG("HC_MCU_SELF_CHK_FW[%d] err res[%x] err[%x]",
                  ret, res.res.ub_res[0], res.err.udata16);
    SENSOR_N_LOG("End Firmware Update.");

    cmd.cmd.udata16 = HC_MCU_FUP_END;
    sns_hostcmd(&cmd, &res, 0, EXE_HOST_ERR, READ_RSLT);
    msleep(300);

    reg = 0x04;
    sns_device_write(CFG, &reg, sizeof(reg));
    {
        uint8_t data = 0;
        sns_device_read(CFG, &data, 1);
        SENSOR_N_LOG("CFG read [%x]", data);
    }

    reg = 0x00;
    sns_device_write(INTMASK0, &reg, sizeof(reg));
    sns_device_write(INTMASK1, &reg, sizeof(reg));

    SENSOR_N_LOG("Check User program mode.");
    ret = sns_get_fw_version(fw_ver);
    g_nFWVersion = SNESOR_COM_GET_FW_VER(fw_ver);
    if(ret != SNS_RC_OK ){
        g_nFWVersion = SNESOR_COM_FW_VER_NONE;
        SENSOR_ERR_LOG("Version not get.");
    }
    SENSOR_N_LOG("Sensor FW Version.[%08x]",g_nFWVersion);
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;
    ret = sns_hostcmd(&cmd, &res, 8, EXE_HOST_ALL, READ_FIFO);
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("Communication Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }
    if(res.res.ub_res[2] != 0x00){
        SENSOR_ERR_LOG("Version check Error!");
        DISABLE_IRQ;
        return SNS_RC_ERR;
    }
    DISABLE_IRQ;
    SENSOR_N_LOG("end");

    return SNS_RC_OK;
}

static void sns_workqueue_init(void)
{
    int32_t i;

    SENSOR_N_LOG("start");

    for(i=0; i<SNS_WORK_QUEUE_NUM; i++){
        cancel_work_sync(&s_tSnsWork[i].work);
        s_tSnsWork[i].status = false;
    }
    g_nSnsWorkCnt = 0;

    SENSOR_N_LOG("end");
}

static int32_t sns_workqueue_create( struct workqueue_struct *queue, void (*func)(struct work_struct *) )
{
    int32_t ret = SNS_RC_ERR;
    int32_t i;
    unsigned long flags;

    SENSOR_N_LOG("start");

    if((queue == NULL) || (func == NULL)){
        return SNS_RC_ERR;
    }

    spin_lock_irqsave(&acc_lock, flags);

    SENSOR_N_LOG("g_nSnsWorkCnt[%d]status[%x]",g_nSnsWorkCnt, s_tSnsWork[g_nSnsWorkCnt].status);

    if(s_tSnsWork[g_nSnsWorkCnt].status == false){

        INIT_WORK( &s_tSnsWork[g_nSnsWorkCnt].work, func );

        ret = queue_work( queue, &s_tSnsWork[g_nSnsWorkCnt].work );

        if (ret == 1) {
            s_tSnsWork[g_nSnsWorkCnt].status = true;

            if(++g_nSnsWorkCnt >= SNS_WORK_QUEUE_NUM){
                g_nSnsWorkCnt = 0;
            }
            ret = SNS_RC_OK;
        }else{
            SENSOR_ERR_LOG("queue_work Non Create[%d]",ret);
        }
        g_workqueue_used = 0;
    }else{
        if(g_workqueue_used < SNS_WORK_QUEUE_NUM){
            SENSOR_ERR_LOG("SNS queue_work[%d] used!!",g_nSnsWorkCnt);
            for(i=0; i<SNS_WORK_QUEUE_NUM; i++){
                SENSOR_N_LOG("g_nSnsWorkCnt[%d]status[%x]",i,s_tSnsWork[i].status);
            }
            if(++g_nSnsWorkCnt >= SNS_WORK_QUEUE_NUM){
                g_nSnsWorkCnt = 0;
            }
            g_workqueue_used++;
        }
    }

    spin_unlock_irqrestore(&acc_lock, flags);

    SENSOR_N_LOG("end - return[%d]",ret);

    return ret;
}

static void sns_workqueue_delete(struct work_struct *work)
{
    int32_t i;
    unsigned long flags;

    SENSOR_N_LOG("start");

    spin_lock_irqsave(&acc_lock, flags);

    for(i=0; i<SNS_WORK_QUEUE_NUM; i++){
        if(&s_tSnsWork[i].work == work){
            s_tSnsWork[i].status = false;
            SENSOR_N_LOG("hit delete queue[%d]! work:0x[%x] 0x[%x] ",
                        i, (int)&s_tSnsWork[i].work, (int)work);
            break;
        }
    }

    spin_unlock_irqrestore(&acc_lock, flags);

    SENSOR_N_LOG("end");

    return ;
}

static bool sns_devif_error_check(void)
{
    SENSOR_N_LOG("g_bDevIF_Error[%d]",g_bDevIF_Error);
    return g_bDevIF_Error;
}

static void sns_pre_power_on(void)
{
    int32_t ret = SNS_RC_OK;
    int32_t cnt = 0;

    SENSOR_N_LOG("start");

    if(sns_devif_error_check() == false){
        SENSOR_N_LOG("Other Sensor Error");
        while(1){
            ret = sns_micon_i2c_enable(true);
            cnt++;
            if(ret == SNS_RC_OK || cnt >= MICON_I2C_ENABLE_RETRY_NUM){
                SENSOR_N_LOG("i2c enable cnt[%d]",cnt);
                break;
            }
            msleep(10);
        }
        ret |= sns_micon_initcmd();
        ret |= sns_set_dev_param();
    } else {
        SENSOR_N_LOG("Micon SPI Error");
        ret |= sns_initialize();
        if(ret != SNS_RC_OK){
            SENSOR_N_LOG("sns_initialize ret[%d]",ret);
            atomic_set(&g_ResetStatus,false);
            return;
        }
        ret |= sns_set_dev_param();
    }

    if( true == g_reset_param_status) {
        ret = sns_reset_restore_param();
        sensor_reset_resume();
        g_reset_param_status = false;
    }

    atomic_set(&g_ResetStatus,false);

    SENSOR_N_LOG("end");
}

static void sns_pre_power_off(void)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_N_LOG("start");

    if(atomic_read(&g_ShutdownStatus) == false){
        atomic_set(&g_ResetStatus,true);
        g_reset_param_status = false;
        sensor_reset();
        ret = sns_reset_save_param();
        g_reset_param_status = true;
        if(sns_devif_error_check() == false){
            SENSOR_N_LOG("Other Sensor Error");
            sns_micon_i2c_enable(false);
        } else {
            SENSOR_N_LOG("Micon SPI Error");
        }
    }

    SENSOR_N_LOG("end");
}

static int32_t sns_micon_i2c_enable(bool arg_iEnable)
{
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    if(atomic_read(&g_FWUpdateStatus) || atomic_read(&g_ShutdownStatus) == true){
        SENSOR_ERR_LOG("FW Update or Recovery Now");
        return 0;
    }

    cmd.cmd.udata16 = HC_MCU_SET_PERI;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = (arg_iEnable == false) ? 0x01:0x00;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 4, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("HC_MCU_SET_PERI(-) err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    if((arg_iEnable == true) && res.res.ub_res[1] == 0x01)
    {
        SENSOR_ERR_LOG("I2C Valid Error RST01[%x]",res.res.ub_res[1]);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end");
    return ret;
}

static int32_t sns_micon_initcmd(void)
{
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MCU_SENSOR_INIT;
    cmd.prm.ub_prm[0] = 0xC1;
    ret = sns_hostcmd(&cmd, &res, 1, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("HC_MCU_SENSOR_INIT(-) err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    if((res.res.ub_res[0] & 0x01) != 0)
    {
        SENSOR_ERR_LOG("Acc Sensor Init Error[%x]",res.res.uw_res[0]);
        ret = SNS_RC_ERR;
    }

    if((res.res.ub_res[0] & 0x40) != 0)
    {
        SENSOR_ERR_LOG("Mag Sensor Init Error[%x]",res.res.uw_res[0]);
        ret = SNS_RC_ERR;
    }

    if((res.res.ub_res[0] & 0x80) != 0)
    {
        SENSOR_ERR_LOG("Gyro Sensor Init Error[%x]",res.res.uw_res[0]);
        ret = SNS_RC_ERR;
    }

    SENSOR_N_LOG("end");
    return ret;
}

static int32_t sns_reset_save_param(void)
{
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MUL_GET_LOG_PARAM_SENSOR;
    ret = sns_hostcmd(&cmd, &res, 2, EXE_HOST_ALL, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("HC_MUL_GET_LOG_PARAM_SENSOR(-) err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }
    g_logging_param[0] = res.res.ub_res[0];
    g_logging_param[1] = res.res.ub_res[1];

    cmd.cmd.udata16 = HC_MUL_SET_LOG_PARAM_SENSOR;
    cmd.prm.ub_prm[0] = g_logging_param[0] & 0x00;
    cmd.prm.ub_prm[1] = g_logging_param[1] & 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_LOG_PARAM_SENSOR err[%x]",res.err.udata16);
        kfree( g_logging_data );
        return SNS_RC_ERR;
    }


    SENSOR_N_LOG("end");
    return ret;
}

static int32_t sns_reset_restore_param(void)
{
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start");

    cmd.cmd.udata16 = HC_MUL_SET_LOG_PARAM_SENSOR;
    cmd.prm.ub_prm[0] = g_logging_param[0];
    cmd.prm.ub_prm[1] = g_logging_param[1];
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MUL_SET_LOG_PARAM_SENSOR err[%x]",res.err.udata16);
        return SNS_RC_ERR;
    }

    SENSOR_N_LOG("end");
    return ret;
}



int32_t sns_err_check(void)
{
    int ret_val = 0;

    if ((sns_devif_error_check() != false) && (atomic_read(&g_ResetStatus) == false)) {
        sensor_power_reset(SENSOR_INDEX_ACC);
        ret_val = -ECOMM;
    } else {
        ret_val = 0;
    }

    return ret_val;
}

int32_t sns_get_reset_status(void)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return atomic_read(&g_ResetStatus);
}

void sns_enable_irq_wake_irq(bool arg_iEnable)
{
    SENSOR_N_LOG("start");

    if(arg_iEnable == true){
        enable_irq_wake(g_nIntIrqNo);
        SENSOR_N_LOG("enable_irq_wake");
    } else {
        disable_irq_wake(g_nIntIrqNo);
        SENSOR_N_LOG("disable_irq_wake");
    }

    SENSOR_N_LOG("end");
}

static void sns_set_buff_int(bool arg_iEnable)
{
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;

    SENSOR_N_LOG("start-Enable[%d]",arg_iEnable);

    cmd.cmd.udata16 = HC_MAG_SET_BUFF_INT;
    cmd.prm.ub_prm[0] = arg_iEnable;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((SNS_RC_OK != ret) || (0 != res.err.udata16)) {
        SENSOR_ERR_LOG("end HC_MAG_SET_BUFF_INT err[%x]",res.err.udata16);
        return;
    }

    SENSOR_N_LOG("end");
}

static struct spi_driver sensor_micon_driver = {
    .probe       = sensor_micon_probe,
    .driver = {
        .name    = SENSOR_MICON_DRIVER_NAME,
        .bus     = &spi_bus_type,
        .owner   = THIS_MODULE,
    },
    .remove      = sensor_micon_remove,
    .suspend     = sensor_micon_suspend,
    .resume      = sensor_micon_resume,
    .shutdown    = sensor_micon_shutdown,
};

static int32_t sensor_micon_resume( struct spi_device *client )
{
    uint32_t batch_status = 0;

    SENSOR_N_LOG("start");

    atomic_set(&g_bIsResume,true);

    sensor_resume();

    batch_status = sensor_get_batch_status();
    if( 0 != batch_status) {
        sensor_set_batch_data();
        sns_set_buff_int(true);
    }

    SENSOR_N_LOG("end");
    return 0;
}

static int32_t sensor_micon_suspend( struct spi_device *client, pm_message_t mesg )
{
    uint32_t batch_status = 0;

    SENSOR_N_LOG("start");

    sensor_suspend();

    batch_status = sensor_get_batch_status();
    if( 0 != batch_status) {
        sns_set_buff_int(false);
    }

    atomic_set(&g_bIsResume,false);

    SENSOR_N_LOG("end");
    return 0;
}

static void sensor_micon_shutdown( struct spi_device *client )
{
    SENSOR_N_LOG("start");

    if(atomic_read(&g_ShutdownStatus) == false){

        sensor_shutdown();

        DISABLE_IRQ;
        sns_workqueue_init();

        gpio_free(SNS_GPIO_INT);
        gpio_free(SNS_GPIO_RST);

        if(sns_wq_int != NULL){
            SENSOR_N_LOG("sns_wq_int");
            flush_workqueue(sns_wq_int);
            destroy_workqueue(sns_wq_int);
            sns_wq_int = NULL;
        }

        if(sns_wq != NULL){
            SENSOR_N_LOG("sns_wq");
            flush_workqueue(sns_wq);
            destroy_workqueue(sns_wq);
            sns_wq = NULL;
        }
    }

    atomic_set(&g_ShutdownStatus,true);

    SENSOR_N_LOG("end");
    return;
}

static int32_t sensor_micon_remove( struct spi_device *client )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static int32_t sensor_micon_probe( struct spi_device *client )
{
    int32_t ret = SNS_RC_OK;

    SENSOR_N_LOG("start");

    client_sns = client;
    client_sns->bits_per_word = 8;

    g_nIntIrqNo = -1;
    g_nIntIrqFlg = 0;
    g_bDevIF_Error = false;
    atomic_set(&g_bIsResume,true);
    atomic_set(&g_ShutdownStatus,false);

    atomic_set(&g_nWeight,DEFAULT_WEIGHT);
    atomic_set(&g_nStepWide,DEFAULT_PEDOMETER);
    atomic_set(&g_nVehiType,DEFAULT_VEHITYPE);
    atomic_set(&g_FWUpdateStatus,false);
    atomic_set(&g_nCalX,0);
    atomic_set(&g_nCalY,0);
    atomic_set(&g_nCalZ,0);
    atomic_set(&g_IntreqAcc,false);
    atomic_set(&g_LogParamSns,0);
    atomic_set(&g_LogParamFus,0);
    atomic_set(&g_Step_start,0);
    atomic_set(&g_MiconDebug,0);
    atomic_set(&g_acc_auto_cal_offset,0);


    g_FusionDelay = 0;
    g_dailys_status = SNS_OFF;
    g_step_status = OTHER_SENSOR;

    init_waitqueue_head(&s_tWaitInt);

    sns_workqueue_init();
    mutex_init(&s_tDataMutex);

    mutex_lock(&s_tDataMutex);
    memset(&s_MagData, 0x00, sizeof(s_MagData));
    memset(&s_GyroData, 0x00, sizeof(s_GyroData));
    mutex_unlock(&s_tDataMutex);

    if(g_nIntIrqNo == -1){
        ret = sns_gpio_init();
        if(ret != SNS_RC_OK){
            SENSOR_ERR_LOG("Failed sns_gpio_init[%d]",ret);
            return -ENODEV;
        }
        DISABLE_IRQ;
    }

    ret = sns_initialize();
    if(ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("Failed sns_initialize[%d]",ret);
        return -ENODEV;
    }

    ret = sns_set_dev_param();
    if(ret != SNS_RC_OK){
        SENSOR_ERR_LOG("Failed sns_set_dev_param[%d]",ret);
        return -ENODEV;
    }

    sns_pre_power_cb.power_on  = sns_pre_power_on;
    sns_pre_power_cb.power_off = sns_pre_power_off;
    sensor_power_reg_cbfunc(&sns_pre_power_cb);

    SENSOR_N_LOG("end");
    return 0;
}

void sensor_micon_init(void)
{
    int ret = 0;

    SENSOR_N_LOG("start");

    sema_init(&s_tSnsrSema, 1);

    sns_wq_int = create_singlethread_workqueue("sns_wq_int");
    if(!sns_wq_int)
    {
        SENSOR_ERR_LOG("can't create interrupt queue-sns_wq_int");
        return;
    }

    sns_wq = create_singlethread_workqueue("sns_wq");
    if(!sns_wq)
    {
        SENSOR_ERR_LOG("can't create interrupt queue-sns_wq");
        goto REGIST_ERR;
    }

    ret = spi_register_driver(&sensor_micon_driver);
    if(ret != 0){
        SENSOR_ERR_LOG("fail:spi_register_driver()->ret[%d]",ret);
        goto REGIST_ERR;
    }

    SENSOR_N_LOG("end");
    return;

REGIST_ERR:
    if(sns_wq_int != NULL){
        flush_workqueue(sns_wq_int);
        destroy_workqueue(sns_wq_int);
        sns_wq_int = NULL;
    }

    if(sns_wq != NULL){
        flush_workqueue(sns_wq);
        destroy_workqueue(sns_wq);
        sns_wq = NULL;
    }

    return;
}

void sensor_micon_exit(void)
{
    SENSOR_N_LOG("start");

    SENSOR_N_LOG("end");
    return;
}

