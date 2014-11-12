/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
*/
/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

//#define DISABLE_DISP_DETECT 1
#define OVER_THERM_PROTECT 1
#define KPDBL_USE_REGULATOR

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/qpnp/pwm.h>
#include <linux/workqueue.h>
#include <linux/kc_led.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/regulator/consumer.h>
#ifdef OVER_THERM_PROTECT
#include <linux/power_supply.h>
#endif
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/delay.h>

#define LED_DEBUG 0

#if LED_DEBUG
#define DEBUG_PRINT(arg...)    printk(KERN_INFO "[LEDDRV]:" arg)
#define LED_DUMP_REGISTER(led, reg_array, array_size)    led_dump_register(led, reg_array, array_size)
#else
#define DEBUG_PRINT(arg...)
#define LED_DUMP_REGISTER(led, reg_array, array_size)
#endif

#define DEBUG_PRINT_INFO(arg...)    printk(KERN_INFO "[LEDDRV]:" arg)
#define DEBUG_PRINT_ERR(arg...)    printk(KERN_ERR "[LEDDRV]:" arg)

#define LED_RESET_ON    0x01
#define LED_RESET_OFF    0x00

#define ALL_OFF_STATE           0x00000000
#define BACK_LIGHT_STATE        0x00000001
#define LED_RED_STATE           0x00000004
#define LED_GREEN_STATE         0x00000008
#define LED_BLUE_STATE          0x00000010
#define MOBILE_LIGHT_STATE      0x00000020
#define RGB_LED_STATE           (LED_RED_STATE|LED_GREEN_STATE|LED_BLUE_STATE)
#define LED_CHARGEPUMP_STATE    (RGB_LED_STATE|MOBILE_LIGHT_STATE)

#define RGB_LED_INFO          "ledinfo"
#ifdef CONFIG_MOBILELIGHT
#define MOBILELIGHT_INFO      "mobilelightinfo"
#endif
#define BACKLIGHT_INFO        "backlightinfo"

#define KEYBACKLIGHT1_INFO    "kpdbl-pwm-1"
#define KEYBACKLIGHT2_INFO    "kpdbl-pwm-2"
#define KEYBACKLIGHT3_INFO    "kpdbl-pwm-3"

#define LABEL_RGB_LED         "rgb"
#ifdef CONFIG_MOBILELIGHT
#define LABEL_MOBILELIGHT     "flash"
#endif
#define LABEL_BACKLIGHT       "wled"

#define REG_MASK_VALUE    0xFF

#define RGB_LED_MAX_BRIGHT_VAL        0xFFFFFFFFu
#ifdef CONFIG_MOBILELIGHT
#define MOBILELIGHT_MAX_BRIGHT_VAL    0xFFFFFFFFu
#endif
#define BACKLIGHT_MAX_BRIGHT_VAL      0xFFFFFFFFu

#define LED_COL_BLACK    0x00000000

#define LSB_8_BIT_MASK    0xFF
#define MSB_4_BIT_MASK    0x0F
#define MSB_5_BIT_MASK    0x1F
#define MSB_8_BIT_SHFT    0x08

#define GET_MSB(val)         ((val >> MSB_8_BIT_SHFT) & MSB_4_BIT_MASK)
#define GET_MSB_5BIT(val)    ((val >> MSB_8_BIT_SHFT) & MSB_5_BIT_MASK)
#define GET_LSB(val)         (val & LSB_8_BIT_MASK)

#define VALUE_TRANSFORMATION(max_val, trans_val, base_val) ((max_val * trans_val) / base_val)

#define LEDLIGHT_BLINK_NUM        4

#define LEDLIGHT            'L'
#define LEDLIGHT_SET_BLINK            _IOW(LEDLIGHT, 0, T_LEDLIGHT_IOCTL)
#define LEDLIGHT_SET_TEMPERTURE_DM    _IOW(LEDLIGHT, 1, T_LEDLIGHT_IOCTL_DM)

#define SPMI_RETRIES_NUM    5
#define SPMI_NO_ERROR       0

#define BACKLIGHT_NUM        2

#define LPG_LUT_LSB_L(offset)    (0xB060 +(0x20 * offset))
#define LPG_LUT_MSB_L(offset)    (0xB061 +(0x20 * offset))
#define LPG_LUT_LSB_H(offset)    (0xB062 +(0x20 * offset))
#define LPG_LUT_MSB_H(offset)    (0xB063 +(0x20 * offset))

#define LPG_LUT_HI_INDEX_VAL(offset)        (0x11 + (0x10 * offset))
#define LPG_LUT_LO_INDEX_VAL(offset)        (0x10 + (0x10 * offset))

#define LPG_LUT_RAMP_CONTROL(offset)        (0x10 << offset)
#define LPG_LUT_RAMP_CONTROL_OFF            0x00

#define RGB_LED_GET_COL_B    0
#define RGB_LED_GET_COL_G    8
#define RGB_LED_GET_COL_R    16

#define RGB_LED_MIXED_COL       0x00FFFFFF
#define RGB_LED_SINGLE_COL_B    0x000000FF
#define RGB_LED_SINGLE_COL_G    0x0000FF00
#define RGB_LED_SINGLE_COL_R    0x00FF0000
#define RGB_LED_SINGLE_COL      0xFF

#define RGB_LED_MAX_LEVEL       0x1FF

#define RGB_LED_RAMP_DURATION    50
#define MAX_PAUSE_CODE           0x1FFF

#define RGB_LED_PATTERN_VAL_MAX_MSB      0x0F
#define RGB_LED_PATTERN_VAL_MAX_R_MSB     0x00
#define RGB_LED_PATTERN_VAL_MID_R_MSB     0x00
#define RGB_LED_PATTERN_VAL_MIN_R_MSB     0x00
#define RGB_LED_PATTERN_VAL_MAX_G_MSB     0x00
#define RGB_LED_PATTERN_VAL_MIN_G_MSB     0x00
#define RGB_LED_PATTERN_VAL_MAX_B_MSB     0x00
#define RGB_LED_PATTERN_VAL_MID1_B_MSB    0x00
#define RGB_LED_PATTERN_VAL_MID2_B_MSB    0x00
#define RGB_LED_PATTERN_VAL_MIN_B_MSB     0x00
#define RGB_LED_PATTERN_VAL_OFF_MSB      0x00

#define RGB_LED_PATTERN_VAL_MAX_LSB       0xFF
#define RGB_LED_PATTERN_VAL_MAX_R_LSB     0x2A
#define RGB_LED_PATTERN_VAL_MID_R_LSB     0x22
#define RGB_LED_PATTERN_VAL_MIN_R_LSB     0x19
#define RGB_LED_PATTERN_VAL_MAX_G_LSB     0x17
#define RGB_LED_PATTERN_VAL_MIN_G_LSB     0x15
#define RGB_LED_PATTERN_VAL_MAX_B_LSB     0x2A
#define RGB_LED_PATTERN_VAL_MID1_B_LSB    0x22
#define RGB_LED_PATTERN_VAL_MID2_B_LSB    0x19
#define RGB_LED_PATTERN_VAL_MIN_B_LSB     0x16
#define RGB_LED_PATTERN_VAL_OFF_LSB       0x00

#define RGB_LED_REG_B0C8    0xB0C8
#define RGB_LED_REG_D045    0xD045
#define RGB_LED_REG_D046    0xD046
#define RGB_LED_REG_D047    0xD047

#define RGB_LED_B_REG_B541    0xB541
#define RGB_LED_B_REG_B542    0xB542

#define RGB_LED_G_REG_B641    0xB641
#define RGB_LED_G_REG_B642    0xB642

#define RGB_LED_R_REG_B741    0xB741
#define RGB_LED_R_REG_B742    0xB742

#define LPG_CHAN_ENABLE_CONTROL(offset)             (0xB546 + (offset * 0x100))
#define LPG_CHAN_PWM_VALUE_MSB(offset)              (0xB545 + (offset * 0x100))
#define LPG_CHAN_PWM_VALUE_LSB(offset)              (0xB544 + (offset * 0x100))
#define LPG_CHAN_LPG_PATTERN_CONFIG(offset)         (0xB540 + (offset * 0x100))
#define LPG_CHAN_RAMP_STEP_DURATION_MSB(offset)     (0xB551 + (offset * 0x100))
#define LPG_CHAN_RAMP_STEP_DURATION_LSB(offset)     (0xB550 + (offset * 0x100))
#define LPG_CHAN_PAUSE_HI_MULTIPLIER_MSB(offset)    (0xB553 + (offset * 0x100))
#define LPG_CHAN_PAUSE_HI_MULTIPLIER_LSB(offset)    (0xB552 + (offset * 0x100))
#define LPG_CHAN_PAUSE_LO_MULTIPLIER_MSB(offset)    (0xB555 + (offset * 0x100))
#define LPG_CHAN_PAUSE_LO_MULTIPLIER_LSB(offset)    (0xB554 + (offset * 0x100))
#define LPG_CHAN_HI_INDEX(offset)                   (0xB556 + (offset * 0x100))
#define LPG_CHAN_LO_INDEX(offset)                   (0xB557 + (offset * 0x100))

#define RGB_LED_REG_D045_INIT    0x01
#define RGB_LED_REG_D047_INIT    0xC0

#define RGB_LED_B_REG_B541_INIT    0x33
#define RGB_LED_B_REG_B542_INIT    0x43

#define RGB_LED_G_REG_B641_INIT    0x33
#define RGB_LED_G_REG_B642_INIT    0x43

#define RGB_LED_R_REG_B741_INIT    0x33
#define RGB_LED_R_REG_B742_INIT    0x43

#define TRI_LED_EN_CTL_VAL(offset)    (0x20 << offset)
#define TRI_LED_EN_CTL_VAL_OFF        0x00

#define LPG_CHAN_ENABLE_CONTROL_ON       0xE4
#define LPG_CHAN_ENABLE_CONTROL_BLINK    0xE2
#define LPG_CHAN_ENABLE_CONTROL_OFF      0x04

#define LPG_CHAN_PWM_VALUE_MSB_OFF       0x00
#define LPG_CHAN_PWM_VALUE_LSB_OFF       0x00

#define LPG_CHAN_LPG_PATTERN_CONFIG_VAL    0x1F

#define MOBILELIGHT_REG_D341    0xD341
#define MOBILELIGHT_REG_D342    0xD342
#define MOBILELIGHT_REG_D343    0xD343
#define MOBILELIGHT_REG_D344    0xD344
#define MOBILELIGHT_REG_D346    0xD346
#define MOBILELIGHT_REG_D347    0xD347
#define MOBILELIGHT_REG_D348    0xD348
#define MOBILELIGHT_REG_D349    0xD349
#define MOBILELIGHT_REG_D34A    0xD34A
#define MOBILELIGHT_REG_D34B    0xD34B
#define MOBILELIGHT_REG_D34C    0xD34C
#define MOBILELIGHT_REG_D34D    0xD34D
#define MOBILELIGHT_REG_D34E    0xD34E
#define MOBILELIGHT_REG_D34F    0xD34F
#define MOBILELIGHT_REG_D351    0xD351
#define MOBILELIGHT_REG_D352    0xD352
#define MOBILELIGHT_REG_D356    0xD356

#define MOBILELIGHT_REG_D341_INIT    0x0F
#define MOBILELIGHT_REG_D342_INIT    0x01
#define MOBILELIGHT_REG_D343_INIT    0x00
#define MOBILELIGHT_REG_D344_INIT    0x0F
#define MOBILELIGHT_REG_D346_INIT    0x00
#define MOBILELIGHT_REG_D347_INIT    0x00
#define MOBILELIGHT_REG_D348_INIT    0x03
#define MOBILELIGHT_REG_D349_INIT    0x1F
#define MOBILELIGHT_REG_D34A_INIT    0x01
#define MOBILELIGHT_REG_D34B_INIT    0x01
#define MOBILELIGHT_REG_D34C_INIT    0x20
#define MOBILELIGHT_REG_D34D_INIT    0x00
#define MOBILELIGHT_REG_D34E_INIT    0x02
#define MOBILELIGHT_REG_D34F_INIT    0x80
#define MOBILELIGHT_REG_D351_INIT    0x80
#define MOBILELIGHT_REG_D352_INIT    0x00

#define MOBILELIGHT_REG_D342_ON_LOW    0x01
#define MOBILELIGHT_REG_D343_ON_LOW    0x00
#define MOBILELIGHT_REG_D34D_ON_LOW    0x00
#define MOBILELIGHT_REG_D34E_ON_LOW    0x02

#define MOBILELIGHT_REG_D342_ON_HIGH    0x00
#define MOBILELIGHT_REG_D343_ON_HIGH    0x00
#define MOBILELIGHT_REG_D34D_ON_HIGH    0x00
#define MOBILELIGHT_REG_D34E_ON_HIGH    0x00

#define MOBILELIGHT_REG_D346_ON    0x80
#define MOBILELIGHT_REG_D347_ON    0xC0

#define MOBILELIGHT_REG_D342_OFF    0x01
#define MOBILELIGHT_REG_D343_OFF    0x00
#define MOBILELIGHT_REG_D34D_OFF    0x00
#define MOBILELIGHT_REG_D34E_OFF    0x02
#define MOBILELIGHT_REG_D346_OFF    0x00
#define MOBILELIGHT_REG_D347_OFF    0x00

#define MOBILELIGHT_REG_D356_RESET_WDT    0x80
#define MOBILELIGHT_WDT_RESET_TIME        30
#define MOBILELIGHT_CAMERATERM_LOW_VAL    0x00
#define MOBILELIGHT_CAMERATERM_HIGH_VAL   0x01
#define MOBILELIGHT_THRESHOLD_TEMPVAL     0x41

#define MOBILELIGHT_ON    0x01
#define MOBILELIGHT_OFF   0x00

#define BACKLIGHT_BASE_LEVEL    0xFF
#define BACKLIGHT_MAX_LEVEL     0xFFF

#define BACKLIGHT_REG_D846    0xD846
#define BACKLIGHT_REG_D847    0xD847

#define BACKLIGHT_REG_D848    0xD848
#define BACKLIGHT_REG_D84C    0xD84C
#define BACKLIGHT_REG_D84D    0xD84D
#define BACKLIGHT_REG_D84E    0xD84E
#define BACKLIGHT_REG_D84F    0xD84F
#define BACKLIGHT_REG_D858    0xD858
#define BACKLIGHT_REG_D860    0xD860
#define BACKLIGHT_REG_D862    0xD862
#define BACKLIGHT_REG_D863    0xD863
#define BACKLIGHT_REG_D866    0xD866
#define BACKLIGHT_REG_D870    0xD870
#define BACKLIGHT_REG_D872    0xD872
#define BACKLIGHT_REG_D873    0xD873
#define BACKLIGHT_REG_D876    0xD876
#define BACKLIGHT_REG_D880    0xD880
#define BACKLIGHT_REG_D882    0xD882
#define BACKLIGHT_REG_D883    0xD883
#define BACKLIGHT_REG_D886    0xD886

#define BACKLIGHT_REG_D848_INIT    0x00
#define BACKLIGHT_REG_D84C_INIT    0x0B
#define BACKLIGHT_REG_D84D_INIT    0x03
#define BACKLIGHT_REG_D84E_INIT    0x03
#define BACKLIGHT_REG_D84F_INIT    0x60
#define BACKLIGHT_REG_D858_INIT    0x00
#define BACKLIGHT_REG_D860_INIT    0x80
#define BACKLIGHT_REG_D862_INIT    0x10
#define BACKLIGHT_REG_D863_INIT    0x00
#define BACKLIGHT_REG_D866_INIT    0x80
#define BACKLIGHT_REG_D870_INIT    0x80
#define BACKLIGHT_REG_D872_INIT    0x10
#define BACKLIGHT_REG_D873_INIT    0x00
#define BACKLIGHT_REG_D876_INIT    0x80
#define BACKLIGHT_REG_D880_INIT    0x00
#define BACKLIGHT_REG_D882_INIT    0x00
#define BACKLIGHT_REG_D883_INIT    0x00
#define BACKLIGHT_REG_D886_INIT    0x00

#define BACKLIGHT_REG_D846_ON      0x80
#define BACKLIGHT_REG_D846_OFF     0x00
#define BACKLIGHT_REG_D847_ON      0x07
#define BACKLIGHT_REG_D847_OFF     0x00

#define BACKLIGHT_BRIGHTNESS_SETTING_MSB(offset) (0xD841 + (0x02 * offset))
#define BACKLIGHT_BRIGHTNESS_SETTING_LSB(offset) (0xD840 + (0x02 * offset))

#define BACKLIGHT_ON                    0x01
#define BACKLIGHT_OFF                   0x00
#define BACKLIGHT_THRESHOLD_TEMPVAL     0x3C
#define BACKLIGHT_THRESHOLD_CURVAL      0xDFD

#define KPDBL_MAX_LEVEL			LED_FULL
#define KPDBL_ROW_SRC_SEL(base)		(base + 0x40)
#define KPDBL_ENABLE(base)		(base + 0x46)
#define KPDBL_ROW_SRC(base)		(base + 0xE5)

#define KPDBL_ROW_SRC_SEL_VAL_MASK	0x0F
#define KPDBL_ROW_SCAN_EN_MASK		0x80
#define KPDBL_ROW_SCAN_VAL_MASK		0x0F
#define KPDBL_ROW_SCAN_EN_SHIFT		7
#define KPDBL_MODULE_EN			0x80
#define KPDBL_MODULE_DIS		0x00
#define KPDBL_MODULE_EN_MASK		0x80

#define QPNP_LED_PWM_FLAGS	(PM_PWM_LUT_LOOP | PM_PWM_LUT_RAMP_UP)
#define QPNP_LUT_RAMP_STEP_DEFAULT	255
#define	PWM_LUT_MAX_SIZE		63
#define LED_TRIGGER_DEFAULT		"none"

#define KEYBACKLIGHT1_ON    0x01
#define KEYBACKLIGHT2_ON    0x02
#define KEYBACKLIGHT3_ON    0x04

/**
 * enum qpnp_leds - QPNP supported led ids
 * @QPNP_ID_WLED - White led backlight
 */
enum qpnp_leds {
    ID_RGB_LED=0,
    ID_MOBILELIGHT,
    ID_BACKLIGHT,
	ID_KPDBL,
    ID_LED_MAX,
};

enum led_index_enum{
    RGB_LED_B=0,
    RGB_LED_G,
    RGB_LED_R,
    RGB_LED_MAX,
};

enum lut_table_index_enum{
    LUT_MSB_L=0,
    LUT_LSB_L,
    LUT_MSB_H,
    LUT_LSB_H,
    LUT_TABLE_MAX,
};

enum color_pattern_enum{
    COLOR_PATTERN_1 = 0,
    COLOR_PATTERN_2,
    COLOR_PATTERN_3,
    COLOR_PATTERN_4,
    COLOR_PATTERN_5,
    COLOR_PATTERN_6,
    COLOR_PATTERN_7,
    COLOR_PATTERN_MAX
};

enum light_index_enum{
    MOBILELIGHT_INDEX = 0,
    LEDLIGHT_INDEX,
    BACKLIGHT_INDEX,
    LIGHT_INDEX_MAX
};

enum blink_control_enum{
    NO_BLINK_REQUEST = 0,
    BLINK_REQUEST,
};

enum led_mode {
	PWM_MODE = 0,
	LPG_MODE,
	MANUAL_MODE,
};

#if LED_DEBUG
static u16 backlight_debug_regs[] = {
    0xD846, 0xD847, 0xD848, 0xD84c, 0xD84d, 0xD84e, 0xD84f, 0xD858,
    0xD860, 0xD862, 0xD863, 0xD866,
    0xD870, 0xD872, 0xD873, 0xD876,
    0xD880, 0xD882, 0xD883, 0xD886,
};
#ifdef CONFIG_MOBILELIGHT
static u16 mobilelight_debug_regs[] = {
    0xA046, 
    0xD341, 0xD342, 0xD343, 0xD344, 0xD346, 0xD347, 0xD348, 0xD349, 0xD34a, 0xD34b, 0xD34c, 0xD34f,
    0xD351,
};
#endif
static u16 rgb_led_debug_regs[] = {
    0xB040, 0xB042, 0xB044, 0xB046, 0xB048, 0xB04A, 0xB04C, 0xB04E,
    0xB050, 0xB052, 0xB054, 0xB056, 0xB058, 0xB05A, 0xB05C, 0xB05E,
    0xB060,
    0xB0C8,
    0xD045, 0xD046, 0xD047,
    0xB540, 0xB541, 0xB542, 0xB544, 0xB545, 0xB546,
    0xB550, 0xB551, 0xB552, 0xB553, 0xB554, 0xB555, 0xB556, 0xB557,
    0xB640, 0xB641, 0xB642, 0xB644, 0xB645, 0xB646,
    0xB650, 0xB651, 0xB652, 0xB653, 0xB654, 0xB655, 0xB656, 0xB657,
    0xB740, 0xB741, 0xB742, 0xB744, 0xB745, 0xB746,
    0xB750, 0xB751, 0xB752, 0xB753, 0xB754, 0xB755, 0xB756, 0xB757,
};

static u16 kpdbl_debug_regs[] = {
	0xE204, 0xE205, 0xE208,
	0xE210, 0xE211, 0xE212, 0xE213, 0xE214, 0xE215, 0xE216, 0xE218, 0xE219, 0xE21A, 0xE21B, 
	0xE240, 0xE243, 0xE246,
	0xE2E5,
	0xE441, 0xE442, 0xE444, 0xE445,
	0xE541, 0xE542, 0xE544, 0xE545,
	0xE641, 0xE642, 0xE644, 0xE645,
};

#endif

static uint32_t const led_pattern_value[COLOR_PATTERN_MAX] = {
    0x000000FF,
    0x00FF0000,
    0x0000FF00,
    0x00FF00FF,
    0x0000FFFF,
    0x00FFFF00,
    0x00C0C0C0
};

static uint32_t const led_pattern_val_msb[COLOR_PATTERN_MAX][RGB_LED_MAX] = {
    {RGB_LED_PATTERN_VAL_MAX_B_MSB  ,RGB_LED_PATTERN_VAL_OFF_MSB   ,RGB_LED_PATTERN_VAL_OFF_MSB   },
    {RGB_LED_PATTERN_VAL_OFF_MSB    ,RGB_LED_PATTERN_VAL_OFF_MSB   ,RGB_LED_PATTERN_VAL_MAX_R_MSB },
    {RGB_LED_PATTERN_VAL_OFF_MSB    ,RGB_LED_PATTERN_VAL_MIN_G_MSB ,RGB_LED_PATTERN_VAL_OFF_MSB   },
    {RGB_LED_PATTERN_VAL_MID2_B_MSB ,RGB_LED_PATTERN_VAL_OFF_MSB   ,RGB_LED_PATTERN_VAL_MIN_R_MSB },
    {RGB_LED_PATTERN_VAL_MID1_B_MSB ,RGB_LED_PATTERN_VAL_MIN_G_MSB ,RGB_LED_PATTERN_VAL_OFF_MSB   },
    {RGB_LED_PATTERN_VAL_OFF_MSB    ,RGB_LED_PATTERN_VAL_MIN_G_MSB ,RGB_LED_PATTERN_VAL_MID_R_MSB },
    {RGB_LED_PATTERN_VAL_MIN_B_MSB  ,RGB_LED_PATTERN_VAL_MAX_G_MSB ,RGB_LED_PATTERN_VAL_MIN_R_MSB }
};

static uint32_t const led_pattern_val_lsb[COLOR_PATTERN_MAX][RGB_LED_MAX] = {
    {RGB_LED_PATTERN_VAL_MAX_B_LSB  ,RGB_LED_PATTERN_VAL_OFF_LSB   ,RGB_LED_PATTERN_VAL_OFF_LSB   },
    {RGB_LED_PATTERN_VAL_OFF_LSB    ,RGB_LED_PATTERN_VAL_OFF_LSB   ,RGB_LED_PATTERN_VAL_MAX_R_LSB },
    {RGB_LED_PATTERN_VAL_OFF_LSB    ,RGB_LED_PATTERN_VAL_MIN_G_LSB ,RGB_LED_PATTERN_VAL_OFF_LSB   },
    {RGB_LED_PATTERN_VAL_MID2_B_LSB ,RGB_LED_PATTERN_VAL_OFF_LSB   ,RGB_LED_PATTERN_VAL_MIN_R_LSB },
    {RGB_LED_PATTERN_VAL_MID1_B_LSB ,RGB_LED_PATTERN_VAL_MIN_G_LSB ,RGB_LED_PATTERN_VAL_OFF_LSB   },
    {RGB_LED_PATTERN_VAL_OFF_LSB    ,RGB_LED_PATTERN_VAL_MIN_G_LSB ,RGB_LED_PATTERN_VAL_MID_R_LSB },
    {RGB_LED_PATTERN_VAL_MIN_B_LSB  ,RGB_LED_PATTERN_VAL_MAX_G_LSB ,RGB_LED_PATTERN_VAL_MIN_R_LSB }
};

/**
 *  kpdbl_config_data - kpdbl configuration data
 *  @pwm_cfg - device pwm configuration
 *  @mode - running mode: pwm or lut
 *  @row_id - row id of the led
 *  @row_src_vbst - 0 for vph_pwr and 1 for vbst
 *  @row_src_en - enable row source
 *  @always_on - always on row
 *  @lut_params - lut parameters to be used by pwm driver
 *  @duty_cycles - duty cycles for lut
 */
struct kpdbl_config_data {
	struct pwm_config_data	*pwm_cfg;
	u32	row_id;
	bool	row_src_vbst;
	bool	row_src_en;
	bool	always_on;
	struct pwm_duty_cycles  *duty_cycles;
	struct lut_params	lut_params;
};
/**
 *  pwm_config_data - pwm configuration data
 *  @lut_params - lut parameters to be used by pwm driver
 *  @pwm_device - pwm device
 *  @pwm_channel - pwm channel to be configured for led
 *  @pwm_period_us - period for pwm, in us
 *  @mode - mode the led operates in
 *  @old_duty_pcts - storage for duty pcts that may need to be reused
 *  @default_mode - default mode of LED as set in device tree
 *  @use_blink - use blink sysfs entry
 *  @blinking - device is currently blinking w/LPG mode
 */
struct pwm_config_data {
	struct lut_params	lut_params;
	struct pwm_device	*pwm_dev;
	int			pwm_channel;
	u32			pwm_period_us;
	struct pwm_duty_cycles	*duty_cycles;
	int	*old_duty_pcts;
	u8	mode;
	u8	default_mode;
	bool use_blink;
	bool blinking;
};

struct light_led_data_type {
    struct led_classdev    cdev;
    struct spmi_device     *spmi_dev;
    struct work_struct     work;
    u8                     num_leds;
    struct mutex           lock;
    uint32_t               ul_value;
    uint32_t               blink_control; 
    uint32_t               blink_ramp_duration;
    uint32_t               blink_low_pause_time;
    uint32_t               blink_high_pause_time;
    uint32_t               blink_off_color;
    uint8_t                rgb_led_val[RGB_LED_MAX];
	struct delayed_work	dwork;
	int			id;
	u16		                    base;
	struct kpdbl_config_data	*kpdbl_cfg;
	int			max_current;
	bool			default_on;
	int			turn_off_delay_ms;
};

typedef struct write_reg_data_init {
    u16    addr;
    u16    value;
} write_reg_led_data_init_type;

typedef struct _t_ledlight_ioctl {
    uint32_t data[LEDLIGHT_BLINK_NUM];
}T_LEDLIGHT_IOCTL;

typedef struct _t_ledlight_ioctl_dm {
    uint32_t dm_data;
}T_LEDLIGHT_IOCTL_DM;

static const write_reg_led_data_init_type backlight_init_reg_data[] = {
    { BACKLIGHT_REG_D848, BACKLIGHT_REG_D848_INIT },

    { BACKLIGHT_REG_D84C, BACKLIGHT_REG_D84C_INIT },
    { BACKLIGHT_REG_D84D, BACKLIGHT_REG_D84D_INIT },
    { BACKLIGHT_REG_D84E, BACKLIGHT_REG_D84E_INIT },
    { BACKLIGHT_REG_D84F, BACKLIGHT_REG_D84F_INIT },
    { BACKLIGHT_REG_D858, BACKLIGHT_REG_D858_INIT },

    { BACKLIGHT_REG_D860, BACKLIGHT_REG_D860_INIT },
    { BACKLIGHT_REG_D862, BACKLIGHT_REG_D862_INIT },
    { BACKLIGHT_REG_D863, BACKLIGHT_REG_D863_INIT },
    { BACKLIGHT_REG_D866, BACKLIGHT_REG_D866_INIT },
    { BACKLIGHT_REG_D870, BACKLIGHT_REG_D870_INIT },
    { BACKLIGHT_REG_D872, BACKLIGHT_REG_D872_INIT },
    { BACKLIGHT_REG_D873, BACKLIGHT_REG_D873_INIT },
    { BACKLIGHT_REG_D876, BACKLIGHT_REG_D876_INIT },
    { BACKLIGHT_REG_D880, BACKLIGHT_REG_D880_INIT },
    { BACKLIGHT_REG_D882, BACKLIGHT_REG_D882_INIT },
    { BACKLIGHT_REG_D883, BACKLIGHT_REG_D883_INIT },
    { BACKLIGHT_REG_D886, BACKLIGHT_REG_D886_INIT },
};

static const write_reg_led_data_init_type mobilelight_init_reg_data[] = {
    { MOBILELIGHT_REG_D341, MOBILELIGHT_REG_D341_INIT },
    { MOBILELIGHT_REG_D342, MOBILELIGHT_REG_D342_INIT },
    { MOBILELIGHT_REG_D343, MOBILELIGHT_REG_D343_INIT },
    { MOBILELIGHT_REG_D344, MOBILELIGHT_REG_D344_INIT },
    { MOBILELIGHT_REG_D346, MOBILELIGHT_REG_D346_INIT },
    { MOBILELIGHT_REG_D347, MOBILELIGHT_REG_D347_INIT },
    { MOBILELIGHT_REG_D348, MOBILELIGHT_REG_D348_INIT },
    { MOBILELIGHT_REG_D349, MOBILELIGHT_REG_D349_INIT },
    { MOBILELIGHT_REG_D34A, MOBILELIGHT_REG_D34A_INIT },
    { MOBILELIGHT_REG_D34B, MOBILELIGHT_REG_D34B_INIT },
    { MOBILELIGHT_REG_D34C, MOBILELIGHT_REG_D34C_INIT },
    { MOBILELIGHT_REG_D34D, MOBILELIGHT_REG_D34D_INIT },
    { MOBILELIGHT_REG_D34E, MOBILELIGHT_REG_D34E_INIT },
    { MOBILELIGHT_REG_D34F, MOBILELIGHT_REG_D34F_INIT },
    { MOBILELIGHT_REG_D351, MOBILELIGHT_REG_D351_INIT },
    { MOBILELIGHT_REG_D352, MOBILELIGHT_REG_D352_INIT },
};

static const write_reg_led_data_init_type rgb_led_init_reg_data[] = {
    { RGB_LED_B_REG_B541, RGB_LED_B_REG_B541_INIT },
    { RGB_LED_B_REG_B542, RGB_LED_B_REG_B542_INIT },
    { RGB_LED_G_REG_B641, RGB_LED_G_REG_B641_INIT },
    { RGB_LED_G_REG_B642, RGB_LED_G_REG_B642_INIT },
    { RGB_LED_R_REG_B741, RGB_LED_R_REG_B741_INIT },
    { RGB_LED_R_REG_B742, RGB_LED_R_REG_B742_INIT },
    { RGB_LED_REG_D045, RGB_LED_REG_D045_INIT },
    { RGB_LED_REG_D047, RGB_LED_REG_D047_INIT },
};

static uint32_t guc_light_dm = 0;

#ifndef DISABLE_DISP_DETECT
static int do_disp_lock = 0;
static atomic_t g_disp_status = ATOMIC_INIT(0);
static atomic_t g_display_detect = ATOMIC_INIT(0);
static struct mutex led_disp_lock;
#endif  /* DISABLE_DISP_DETECT */

static struct light_led_data_type *grgb_led = NULL;
static struct light_led_data_type *gbacklight = NULL;
static struct light_led_data_type *gmobilelight = NULL;

static struct light_led_data_type *gkpdbl1 = NULL;
static struct light_led_data_type *gkpdbl2 = NULL;
static struct light_led_data_type *gkpdbl3 = NULL;
#ifdef KPDBL_USE_REGULATOR
static struct regulator *kpdbl_boost_boost_enable = NULL;
static int boost_count = 0;
static int boost_stat = 0;
#endif

#ifdef CONFIG_MOBILELIGHT
static struct regulator *boost_boost_enable;

static struct hrtimer mobilelight_wdt;
#endif

static uint8_t kpbl_leds_status;

#ifdef OVER_THERM_PROTECT
static int get_substrate_therm(enum power_supply_property get_therm)
{
    struct power_supply *battery_psy = NULL;
    union power_supply_propval ret = {0,};

    battery_psy = power_supply_get_by_name("battery");
    
    if(battery_psy)
        battery_psy->get_property(battery_psy, get_therm, &ret);

    return ret.intval;
}
#endif

static int led_reg_write(struct light_led_data_type *led, u16 addr, u8 val)
{
    int rc=0;
    int retry=0;

    do
    {
        rc = spmi_ext_register_writel(led->spmi_dev->ctrl, led->spmi_dev->sid, addr, &val, 1);
        if (rc) {
            DEBUG_PRINT_ERR("%s(): Unable to write to dev=[0x%08x] addr=[%04x] val=[0x%02x] rc=[%d] retry=[%d] \n",
                __func__, (unsigned int)&led->spmi_dev->dev, addr, val, rc, retry);
        }
    }while ((rc != SPMI_NO_ERROR) && (++retry < SPMI_RETRIES_NUM));

    DEBUG_PRINT("%s(): Write addr=[0x%04x] val=[0x%02x] \n", __func__, addr, val);

    return rc;
}

static int led_reg_read(struct light_led_data_type *led, u16 addr, u8 *val)
{
    int rc=0;
    int retry=0;
    u8 reg=0;

    do
    {
        rc = spmi_ext_register_readl(led->spmi_dev->ctrl, led->spmi_dev->sid, addr, &reg, 1);
        if (rc) {
            DEBUG_PRINT_ERR("%s(): Unable to read from dev=[0x%08x] addr=[%04x] val=[0x%02x] rc=[%d] retry=[%d] \n",
                __func__, (unsigned int)&led->spmi_dev->dev, addr, reg, rc, retry);
        }
    }while ((rc != SPMI_NO_ERROR) && (++retry < SPMI_RETRIES_NUM));

    *val = reg;

    DEBUG_PRINT("%s():  Read  addr=[0x%04x] val=[0x%02x] \n", __func__, addr, *val);

    return rc;
}

static int led_masked_reg_write(struct light_led_data_type *led, u16 addr, u8 mask, u8 val)
{
    int rc=0;
    u8 reg=0;

    rc = led_reg_read(led, addr, &reg);
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_reg_read rc=[%d] \n", __func__, rc);
		return rc;
	}

    reg &= ~mask;
    reg |= val;

	if( (addr & 0xe000) == 0xe000 ){
		DEBUG_PRINT("%s() called.\n", __func__);
		DEBUG_PRINT("     addr = 0x%02x\n", addr);
		DEBUG_PRINT("     mask = 0x%02x\n", mask);
		DEBUG_PRINT("     val  = 0x%02x\n", val);
		DEBUG_PRINT("     reg  = 0x%02x\n", reg);
	}

    rc = led_reg_write(led, addr, reg);
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_reg_write rc=[%d] \n", __func__, rc);
		return rc;
	}

    return rc;

}

#if LED_DEBUG
static void led_dump_register(struct light_led_data_type *led, u16 regs[], u16 array_size)
{
    int i;
    u8 val;

    DEBUG_PRINT("%s(): start \n", __func__);

    DEBUG_PRINT("%s(): ===== [%s] register dump start ===== \n", __func__, led->cdev.name);
    for (i = 0; i < array_size; i++) {
        spmi_ext_register_readl(led->spmi_dev->ctrl,
                    led->spmi_dev->sid,
                    regs[i],
                    &val, sizeof(val));
        DEBUG_PRINT("%s(): addr=[0x%04x] val=[0x%02x] \n", __func__, regs[i], val);
    }
    DEBUG_PRINT("%s(): ===== [%s] register dump end ===== \n", __func__,led->cdev.name);

    DEBUG_PRINT("%s(): end \n", __func__);
}
#endif

static int32_t rgb_led_color_pattern_check(uint32_t ul_colorval)
{
    int32_t lret = COLOR_PATTERN_MAX;
    int32_t lmatch = 0;

    DEBUG_PRINT("%s(): start \n", __func__);

    for (lmatch = 0; lmatch < COLOR_PATTERN_MAX; lmatch++) {
        if (ul_colorval == led_pattern_value[lmatch]) {
            DEBUG_PRINT("%s(): color pattern match [%d] \n", __func__, lmatch);
            lret = lmatch;
            break;
        }
    }

    DEBUG_PRINT("%s(): end \n", __func__);
    return lret;
}

static int32_t rgb_led_lut_table_reg(struct light_led_data_type *led, uint32_t ul_pattern_val)
{
    int rc=0, i;
    uint32_t led_val[RGB_LED_MAX], led_off_val[RGB_LED_MAX];
    uint32_t ul_pattern_val_off;
    write_reg_led_data_init_type blink_col_table[LUT_TABLE_MAX * RGB_LED_MAX];
    
    DEBUG_PRINT("%s(): start \n", __func__);
    
    if (ul_pattern_val != COLOR_PATTERN_MAX) {
        for (i = 0; i < RGB_LED_MAX; i++) {
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_L].addr  = LPG_LUT_MSB_L(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_L].value = led_pattern_val_msb[ul_pattern_val][i];
            
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_L].addr  = LPG_LUT_LSB_L(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_L].value = led_pattern_val_lsb[ul_pattern_val][i];
        }
    } else {
        for (i = 0; i < RGB_LED_MAX; i++) {
            led_val[i]  = VALUE_TRANSFORMATION(RGB_LED_MAX_LEVEL, led->rgb_led_val[i],  RGB_LED_SINGLE_COL);
            
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_L].addr  = LPG_LUT_MSB_L(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_L].value = GET_MSB(led_val[i]);
            
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_L].addr  = LPG_LUT_LSB_L(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_L].value = GET_LSB(led_val[i]);
        }
    }
    ul_pattern_val_off = rgb_led_color_pattern_check(grgb_led->blink_off_color & RGB_LED_MIXED_COL);
    DEBUG_PRINT("%s(): Debug info ul_pattern_val_off=[%i] \n", __func__, ul_pattern_val_off);

    if (ul_pattern_val_off != COLOR_PATTERN_MAX) {
        for (i = 0; i < RGB_LED_MAX; i++) {
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_H].addr  = LPG_LUT_MSB_H(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_H].value = led_pattern_val_msb[ul_pattern_val_off][i];
            
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_H].addr  = LPG_LUT_LSB_H(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_H].value = led_pattern_val_lsb[ul_pattern_val_off][i];
        }

    } else {
        led_off_val[RGB_LED_R] = (uint8_t)((grgb_led->blink_off_color >> (RGB_LED_GET_COL_R)) & RGB_LED_SINGLE_COL);
        led_off_val[RGB_LED_G] = (uint8_t)((grgb_led->blink_off_color >> (RGB_LED_GET_COL_G)) & RGB_LED_SINGLE_COL);
        led_off_val[RGB_LED_B] = (uint8_t)((grgb_led->blink_off_color >> (RGB_LED_GET_COL_B)) & RGB_LED_SINGLE_COL);

        for (i = 0; i < RGB_LED_MAX; i++) {
            led_off_val[i]  = VALUE_TRANSFORMATION(RGB_LED_MAX_LEVEL, led_off_val[i],  RGB_LED_SINGLE_COL);
            
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_H].addr  = LPG_LUT_MSB_H(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_H].value = GET_MSB(led_off_val[i]);
            
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_H].addr  = LPG_LUT_LSB_H(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_H].value = GET_LSB(led_off_val[i]);
        }
    }

    for (i = 0; i < (LUT_TABLE_MAX * RGB_LED_MAX); i++) {
        rc = led_masked_reg_write(led, blink_col_table[i].addr, REG_MASK_VALUE, blink_col_table[i].value);
    	if (rc != SPMI_NO_ERROR){
			DEBUG_PRINT_ERR("%s(): Failed to led_reg_write rc=[%d] \n", __func__, rc);
            return rc;
    	}
    }

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

}
static int32_t rgb_led_blink_request(struct light_led_data_type *led, uint32_t hi_pause_code, uint32_t lo_pause_code, int col)
{
    int rc=0;

    DEBUG_PRINT("%s(): start \n", __func__);
    DEBUG_PRINT("%s(): Debug info Regster color [%d(R:2/G:1/B:0)] \n", __func__, col);

    rc = led_masked_reg_write(led, LPG_CHAN_LPG_PATTERN_CONFIG(col), REG_MASK_VALUE, LPG_CHAN_LPG_PATTERN_CONFIG_VAL);
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_reg_write(LPG_CHAN_LPG_PATTERN_CONFIG) rc=[%d] \n", __func__, rc);
		return rc;
	}

    rc = led_masked_reg_write(led, LPG_CHAN_RAMP_STEP_DURATION_MSB(col), REG_MASK_VALUE, GET_MSB(grgb_led->blink_ramp_duration));
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_reg_write(LPG_CHAN_RAMP_STEP_DURATION_MSB) rc=[%d] \n", __func__, rc);
		return rc;
	}

    rc = led_masked_reg_write(led, LPG_CHAN_RAMP_STEP_DURATION_LSB(col), REG_MASK_VALUE, GET_LSB(grgb_led->blink_ramp_duration));
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_reg_write(LPG_CHAN_RAMP_STEP_DURATION_LSB) rc=[%d] \n", __func__, rc);
		return rc;
	}

    rc = led_masked_reg_write(led, LPG_CHAN_PAUSE_HI_MULTIPLIER_MSB(col), REG_MASK_VALUE, GET_MSB_5BIT(hi_pause_code));
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_reg_write(LPG_CHAN_PAUSE_HI_MULTIPLIER_MSB) rc=[%d] \n", __func__, rc);
		return rc;
	}

    rc = led_masked_reg_write(led, LPG_CHAN_PAUSE_HI_MULTIPLIER_LSB(col), REG_MASK_VALUE, GET_LSB(hi_pause_code));
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_reg_write(LPG_CHAN_PAUSE_HI_MULTIPLIER_LSB) rc=[%d] \n", __func__, rc);
		return rc;
	}

    rc = led_masked_reg_write(led, LPG_CHAN_PAUSE_LO_MULTIPLIER_MSB(col), REG_MASK_VALUE, GET_MSB_5BIT(lo_pause_code));
    if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_reg_write(LPG_CHAN_PAUSE_LO_MULTIPLIER_MSB) rc=[%d] \n", __func__, rc);
		return rc;
    }

    rc = led_masked_reg_write(led, LPG_CHAN_PAUSE_LO_MULTIPLIER_LSB(col), REG_MASK_VALUE, GET_LSB(lo_pause_code));
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_reg_write(LPG_CHAN_PAUSE_LO_MULTIPLIER_LSB) rc=[%d] \n", __func__, rc);
		return rc;
	}

    rc = led_masked_reg_write(led, LPG_CHAN_HI_INDEX(col), REG_MASK_VALUE, LPG_LUT_HI_INDEX_VAL(col));
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_reg_write(LPG_CHAN_HI_INDEX) rc=[%d] \n", __func__, rc);
		return rc;
	}

    rc = led_masked_reg_write(led, LPG_CHAN_LO_INDEX(col), REG_MASK_VALUE, LPG_LUT_LO_INDEX_VAL(col));
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_reg_write(LPG_CHAN_LO_INDEX) rc=[%d] \n", __func__, rc);
		return rc;
	}

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

}

static int32_t rgb_led_light_on(int col, uint8_t msb, uint8_t lsb, struct light_led_data_type *led)
{
    int rc=0;

    DEBUG_PRINT("%s(): start \n", __func__);
    DEBUG_PRINT("%s(): Debug info Regster color [%d(R:2/G:1/B:0)] \n", __func__, col);

    rc = led_masked_reg_write(led, LPG_CHAN_ENABLE_CONTROL(col), REG_MASK_VALUE, LPG_CHAN_ENABLE_CONTROL_ON);
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(LPG_CHAN_ENABLE_CONTROL) rc=[%d] \n", __func__, rc);
		return rc;
	}

    rc = led_masked_reg_write(led, LPG_CHAN_PWM_VALUE_MSB(col), REG_MASK_VALUE, msb);
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(LPG_CHAN_PWM_VALUE_MSB) rc=[%d] \n", __func__, rc);
		return rc;
	}

    rc = led_masked_reg_write(led, LPG_CHAN_PWM_VALUE_LSB(col), REG_MASK_VALUE, lsb);
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(LPG_CHAN_PWM_VALUE_LSB) rc=[%d] \n", __func__, rc);
		return rc;
	}

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

}

static int32_t rgb_led_light_off(struct light_led_data_type *led)
{
    int rc=0, i;

    DEBUG_PRINT("%s(): start \n", __func__);

    for (i = 0; i < RGB_LED_MAX; i++) {

        rc = led_masked_reg_write(led, LPG_CHAN_ENABLE_CONTROL(i), REG_MASK_VALUE, LPG_CHAN_ENABLE_CONTROL_OFF);
    	if (rc != SPMI_NO_ERROR){
			DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(LPG_CHAN_ENABLE_CONTROL) rc=[%d] \n", __func__, rc);
			return rc;
    	}

        rc = led_masked_reg_write(led, LPG_CHAN_PWM_VALUE_MSB(i), REG_MASK_VALUE, LPG_CHAN_PWM_VALUE_MSB_OFF);
    	if (rc != SPMI_NO_ERROR){
			DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(LPG_CHAN_PWM_VALUE_MSB) rc=[%d] \n", __func__, rc);
			return rc;
    	}

        rc = led_masked_reg_write(led, LPG_CHAN_PWM_VALUE_LSB(i), REG_MASK_VALUE, LPG_CHAN_PWM_VALUE_LSB_OFF);
    	if (rc != SPMI_NO_ERROR){
			DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(LPG_CHAN_PWM_VALUE_LSB) rc=[%d] \n", __func__, rc);
			return rc;
    	}

    }

    rc = led_reg_write(led, RGB_LED_REG_B0C8, LPG_LUT_RAMP_CONTROL_OFF);
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_reg_write(RGB_LED_REG_B0C8) rc=[%d] \n", __func__, rc);
		return rc;
	}

    rc = led_masked_reg_write(led, RGB_LED_REG_D046, REG_MASK_VALUE, TRI_LED_EN_CTL_VAL_OFF);
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_reg_write(RGB_LED_REG_D046) rc=[%d] \n", __func__, rc);
		return rc;
	}

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

}

static int32_t rgb_led_light_set(struct light_led_data_type *led)
{
    int rc=0, i;
    uint8_t ctl_val=0, lut_val = 0;
    uint32_t led_val[RGB_LED_MAX];
    uint32_t hi_pause_code, lo_pause_code;
    uint32_t ul_pattern_val;

    DEBUG_PRINT("%s(): start \n", __func__);

    ul_pattern_val = rgb_led_color_pattern_check(led->ul_value & RGB_LED_MIXED_COL);
    DEBUG_PRINT("%s(): Debug info ul_pattern_val=[%i] \n", __func__, ul_pattern_val);

    if (led->blink_control == BLINK_REQUEST) {
        rc = rgb_led_lut_table_reg(led, ul_pattern_val);
    	if (rc != SPMI_NO_ERROR){
			DEBUG_PRINT_ERR("%s(): Failed to rgb_led_lut_table_reg rc=[%d] \n", __func__, rc);
			return rc;
    	}

        hi_pause_code = (grgb_led->blink_high_pause_time / grgb_led->blink_ramp_duration);
        lo_pause_code = (grgb_led->blink_low_pause_time  / grgb_led->blink_ramp_duration);

        if (hi_pause_code != 0)
            hi_pause_code = hi_pause_code - 1;

        if (lo_pause_code != 0)
            lo_pause_code = lo_pause_code - 1;

        if (hi_pause_code > MAX_PAUSE_CODE)
            hi_pause_code = MAX_PAUSE_CODE;

        if (lo_pause_code > MAX_PAUSE_CODE)
            lo_pause_code = MAX_PAUSE_CODE;

        DEBUG_PRINT("%s(): Debug info hi_pause_code=[%i] lo_pause_code=[%i]", __func__, hi_pause_code, lo_pause_code);

        if ((led->ul_value & RGB_LED_SINGLE_COL_B) || (grgb_led->blink_off_color & RGB_LED_SINGLE_COL_B)) {

            rc = rgb_led_blink_request(led, hi_pause_code, lo_pause_code, RGB_LED_B);
            if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to rgb_led_blink_request(RGB_LED_B) rc=[%d] \n", __func__, rc);
				return rc;
            }

            rc = led_masked_reg_write(led, LPG_CHAN_ENABLE_CONTROL(RGB_LED_B), REG_MASK_VALUE, LPG_CHAN_ENABLE_CONTROL_BLINK);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(RGB_LED_B) rc=[%d] \n", __func__, rc);
				return rc;
        	}

            ctl_val |= TRI_LED_EN_CTL_VAL(RGB_LED_B);
            lut_val |= LPG_LUT_RAMP_CONTROL(RGB_LED_B);
        }

        if ((led->ul_value & RGB_LED_SINGLE_COL_G) || (grgb_led->blink_off_color & RGB_LED_SINGLE_COL_G)) {

            rc = rgb_led_blink_request(led, hi_pause_code, lo_pause_code, RGB_LED_G);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to rgb_led_blink_request(RGB_LED_G) rc=[%d] \n", __func__, rc);
				return rc;
        	}

            rc = led_masked_reg_write(led, LPG_CHAN_ENABLE_CONTROL(RGB_LED_G), REG_MASK_VALUE, LPG_CHAN_ENABLE_CONTROL_BLINK);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(RGB_LED_G) rc=[%d] \n", __func__, rc);
				return rc;
        	}

            ctl_val |= TRI_LED_EN_CTL_VAL(RGB_LED_G);
            lut_val |= LPG_LUT_RAMP_CONTROL(RGB_LED_G);
        }

        if ((led->ul_value & RGB_LED_SINGLE_COL_R) || (grgb_led->blink_off_color & RGB_LED_SINGLE_COL_R)) {

            rc = rgb_led_blink_request(led, hi_pause_code, lo_pause_code, RGB_LED_R);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to rgb_led_blink_request(RGB_LED_R) rc=[%d] \n", __func__, rc);
				return rc;
        	}

            rc = led_masked_reg_write(led, LPG_CHAN_ENABLE_CONTROL(RGB_LED_R), REG_MASK_VALUE, LPG_CHAN_ENABLE_CONTROL_BLINK);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(RGB_LED_R) rc=[%d] \n", __func__, rc);
				return rc;
        	}

            ctl_val |= TRI_LED_EN_CTL_VAL(RGB_LED_R);
            lut_val |= LPG_LUT_RAMP_CONTROL(RGB_LED_R);
        }

        if (ctl_val == 0 ) {
            DEBUG_PRINT_ERR("%s(): TRI_LED_EN_CTL(0x1D046) Must be set a value \n", __func__);
            rc = -1;
			DEBUG_PRINT_ERR("%s(): Error end rc=[%d],ctl_val=[0x%02x] \n", __func__, rc, ctl_val);
			return rc;
        } else {
            rc = led_masked_reg_write(led, RGB_LED_REG_D046, REG_MASK_VALUE, ctl_val);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(RGB_LED_REG_D046) rc=[%d] \n", __func__, rc);
				return rc;
        	}
        }

        if (lut_val == 0) {
            DEBUG_PRINT_ERR("%s(): LPG_LUT_RAMP_CONTROL(0x1B0C8) Must be set a value \n", __func__);
            rc = -1;
			DEBUG_PRINT_ERR("%s(): Error end rc=[%d],lut_val=[0x%02x] \n", __func__, rc, lut_val);
			return rc;
        } else {
            rc = led_reg_write(led, RGB_LED_REG_B0C8, lut_val);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_reg_write(RGB_LED_REG_B0C8) rc=[%d] \n", __func__, rc);
				return rc;
        	}
        }
    } 
    else {
        if (ul_pattern_val != COLOR_PATTERN_MAX) {
            for(i =0; i < RGB_LED_MAX; i++) {
                rc = rgb_led_light_on(i, led_pattern_val_msb[ul_pattern_val][i], led_pattern_val_lsb[ul_pattern_val][i], led);
            	if (rc != SPMI_NO_ERROR){
					DEBUG_PRINT_ERR("%s(): Failed to rgb_led_light_on() rc=[%d] \n", __func__, rc);
					DEBUG_PRINT_ERR("%s(): ul_pattern_val=[%d],i=[%d] \n", __func__, ul_pattern_val, i);
					return rc;
            	}

                if ((led_pattern_val_msb[ul_pattern_val][i] != RGB_LED_PATTERN_VAL_OFF_MSB) ||
                    (led_pattern_val_lsb[ul_pattern_val][i] != RGB_LED_PATTERN_VAL_OFF_LSB))
                        ctl_val |= TRI_LED_EN_CTL_VAL(i);
            }
        } else {
            for (i = 0; i < RGB_LED_MAX; i++) {

                led_val[i]  = VALUE_TRANSFORMATION(RGB_LED_MAX_LEVEL, led->rgb_led_val[i],  RGB_LED_SINGLE_COL);
                DEBUG_PRINT("%s(): Debug info RGB_LED=[%d(R:2/G:1/B:0)] Value=[0x%04X] \n", __func__, i, led_val[i]);

                rc = rgb_led_light_on(i, GET_MSB(led_val[i]), GET_LSB(led_val[i]), led);
            	if (rc != SPMI_NO_ERROR){
					DEBUG_PRINT_ERR("%s(): Failed to rgb_led_light_on() rc=[%d] \n", __func__, rc);
					DEBUG_PRINT_ERR("%s(): ul_pattern_val=[%d],i=[%d] \n", __func__, ul_pattern_val, i);
					return rc;
            	}

                if (led_val[i] != LED_COL_BLACK)
                    ctl_val |= TRI_LED_EN_CTL_VAL(i);
            }
        }

        if (ctl_val == 0) {
            DEBUG_PRINT_ERR("%s(): TRI_LED_EN_CTL(0x1D046) Must be set a value \n", __func__);
            rc = -1;
			DEBUG_PRINT_ERR("%s(): Error end rc=[%d],ctl_val=[0x%02x] \n", __func__, rc, ctl_val);
			return rc;
        } else {
            rc = led_masked_reg_write(led, RGB_LED_REG_D046, REG_MASK_VALUE, ctl_val);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(RGB_LED_REG_D046) rc=[%d] \n", __func__, rc);
				return rc;
        	}
        }
    }

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

}

static int rgb_led_set(struct light_led_data_type *led)
{
    int rc=0;

    mutex_lock(&grgb_led->lock);
    DEBUG_PRINT("%s(): mutex_lock \n", __func__);

    DEBUG_PRINT("%s(): start Value=[0x%08x] \n", __func__, led->ul_value);

    rc = rgb_led_light_off(led);
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to rgb_led_light_off rc=[%d] \n", __func__, rc);
        goto FAIL_SPMI_CONTROL;
	}

    if (LED_COL_BLACK != (led->ul_value & RGB_LED_MIXED_COL)){
        led->rgb_led_val[RGB_LED_R] = (u8)((led->ul_value >> RGB_LED_GET_COL_R) & RGB_LED_SINGLE_COL);
        led->rgb_led_val[RGB_LED_G] = (u8)((led->ul_value >> RGB_LED_GET_COL_G) & RGB_LED_SINGLE_COL);
        led->rgb_led_val[RGB_LED_B] = (u8)((led->ul_value >> RGB_LED_GET_COL_B) & RGB_LED_SINGLE_COL);

        DEBUG_PRINT("%s(): RGB color Shift and Masked R:[0x%04x] G:[0x%04x] B:[0x%04x] \n", 
            __func__, led->rgb_led_val[RGB_LED_R], led->rgb_led_val[RGB_LED_G], led->rgb_led_val[RGB_LED_B]);

        rc = rgb_led_light_set(led);
    	if (rc != SPMI_NO_ERROR){
			DEBUG_PRINT_ERR("%s(): Failed to rgb_led_light_set rc=[%d] \n", __func__, rc);
            goto FAIL_SPMI_CONTROL;
    	}
    }

    mutex_unlock(&grgb_led->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    mutex_unlock(&grgb_led->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
    return rc;
}
#ifdef CONFIG_MOBILELIGHT
static enum hrtimer_restart mobilelight_wdt_reset(struct hrtimer *timer)
{
    int rc=0;

    DEBUG_PRINT("%s(): start \n", __func__);

    rc = led_reg_write(gmobilelight, MOBILELIGHT_REG_D356, MOBILELIGHT_REG_D356_RESET_WDT);
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
		DEBUG_PRINT_ERR("%s(): Mobilelight WatchDogTimer can't reset and timer not restart \n", __func__);
		return HRTIMER_NORESTART;
	}

    hrtimer_start(&mobilelight_wdt, ktime_set(MOBILELIGHT_WDT_RESET_TIME, 0), HRTIMER_MODE_REL);

    DEBUG_PRINT("%s(): Mobilelight WatchDogTimer Reset and Timer Restart \n", __func__);

    DEBUG_PRINT("%s(): end \n", __func__);

    return HRTIMER_NORESTART;

}

static int mobilelight_set(struct light_led_data_type *led)
{
    int rc=0;
    uint8_t uc_mobilelight_val;
    int32_t camera_temp = 0;
    char *err_msg="SPMI";
    static int32_t mobilelight_status = MOBILELIGHT_OFF;

    DEBUG_PRINT("%s(): start mobilelight_status[%d]\n", __func__, mobilelight_status);

    mutex_lock(&gmobilelight->lock);
    DEBUG_PRINT("%s(): mutex_lock \n", __func__);

    if (led->ul_value > LED_COL_BLACK) {
        if (mobilelight_status == MOBILELIGHT_OFF) {

            DEBUG_PRINT("%s(): dm_val=[%d] \n", __func__, guc_light_dm);
            if (guc_light_dm == 1) {
                uc_mobilelight_val = MOBILELIGHT_CAMERATERM_HIGH_VAL;
                DEBUG_PRINT("%s(): mobilelight_val=[%d] \n", __func__, uc_mobilelight_val);
            }
            else if (guc_light_dm == 2) {
                uc_mobilelight_val = MOBILELIGHT_CAMERATERM_LOW_VAL;
                DEBUG_PRINT("%s(): mobilelight_val=[%d] \n", __func__, uc_mobilelight_val);
            }
            else {
#ifdef OVER_THERM_PROTECT
                camera_temp = get_substrate_therm(POWER_SUPPLY_PROP_OEM_CAMERA_THERM);
#else
                camera_temp = 0;
#endif
                DEBUG_PRINT("%s(): Get CameraTemp=[%d] \n", __func__, camera_temp);

                if (camera_temp >= MOBILELIGHT_THRESHOLD_TEMPVAL) {
                    DEBUG_PRINT_ERR("%s(): CameraTemp is over threshold. CameraTemp=[%d] \n", __func__, camera_temp);
                    uc_mobilelight_val = MOBILELIGHT_CAMERATERM_HIGH_VAL;
                }
                else {
                    uc_mobilelight_val = MOBILELIGHT_CAMERATERM_LOW_VAL;
                }

                DEBUG_PRINT("%s(): uc_mobilelight_val=[%d] \n",  __func__, uc_mobilelight_val);
            }

            if (uc_mobilelight_val == MOBILELIGHT_CAMERATERM_LOW_VAL) {
                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D342, REG_MASK_VALUE, MOBILELIGHT_REG_D342_ON_LOW);
            	if (rc != SPMI_NO_ERROR){
					DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D342) rc=[%d] \n", __func__, rc);
                    goto FAIL_SPMI_CONTROL;
            	}

                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D343, REG_MASK_VALUE, MOBILELIGHT_REG_D343_ON_LOW);
            	if (rc != SPMI_NO_ERROR){
					DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D343) rc=[%d] \n", __func__, rc);
                    goto FAIL_SPMI_CONTROL;
            	}

                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D34D, REG_MASK_VALUE, MOBILELIGHT_REG_D34D_ON_LOW);
            	if (rc != SPMI_NO_ERROR){
					DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D34D) rc=[%d] \n", __func__, rc);
                    goto FAIL_SPMI_CONTROL;
            	}

                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D34E, REG_MASK_VALUE, MOBILELIGHT_REG_D34E_ON_LOW);
            	if (rc != SPMI_NO_ERROR){
					DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D34E) rc=[%d] \n", __func__, rc);
                    goto FAIL_SPMI_CONTROL;
            	}
            } else {
                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D342, REG_MASK_VALUE, MOBILELIGHT_REG_D342_ON_HIGH);
            	if (rc != SPMI_NO_ERROR){
					DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D342) rc=[%d] \n", __func__, rc);
                    goto FAIL_SPMI_CONTROL;
            	}

                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D343, REG_MASK_VALUE, MOBILELIGHT_REG_D343_ON_HIGH);
            	if (rc != SPMI_NO_ERROR){
					DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D343) rc=[%d] \n", __func__, rc);
                    goto FAIL_SPMI_CONTROL;
            	}

                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D34D, REG_MASK_VALUE, MOBILELIGHT_REG_D34D_ON_HIGH);
            	if (rc != SPMI_NO_ERROR){
					DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D34D) rc=[%d] \n", __func__, rc);
                    goto FAIL_SPMI_CONTROL;
            	}

                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D34E, REG_MASK_VALUE, MOBILELIGHT_REG_D34E_ON_HIGH);
            	if (rc != SPMI_NO_ERROR){
					DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D34E) rc=[%d] \n", __func__, rc);
                    goto FAIL_SPMI_CONTROL;
            	}
            }

            rc = regulator_enable(boost_boost_enable);
            if (rc != 0) {
                err_msg = "Regulator";
				DEBUG_PRINT_ERR("%s(): Failed to regulator_enable rc=[%d] \n", __func__, rc);
                goto FAIL_SPMI_CONTROL;
            }

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D346, REG_MASK_VALUE, MOBILELIGHT_REG_D346_ON);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D346) rc=[%d] \n", __func__, rc);
                goto FAIL_SPMI_CONTROL;
        	}

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D347, REG_MASK_VALUE, MOBILELIGHT_REG_D347_ON);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D347) rc=[%d] \n", __func__, rc);
                goto FAIL_SPMI_CONTROL;
        	}

            hrtimer_start(&mobilelight_wdt, ktime_set(MOBILELIGHT_WDT_RESET_TIME, 0), HRTIMER_MODE_REL);

            DEBUG_PRINT("%s(): Mobilelight WatchDogTimer start \n", __func__);

            mobilelight_status = MOBILELIGHT_ON;
        } else {
            DEBUG_PRINT("%s(): Mobilelight ON skip \n", __func__);
        }
    } else {
        if (mobilelight_status == MOBILELIGHT_ON) {

            hrtimer_cancel(&mobilelight_wdt);
            DEBUG_PRINT("%s(): Mobilelight WatchDogTimer cancel \n", __func__);

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D342, REG_MASK_VALUE, MOBILELIGHT_REG_D342_OFF);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D342) rc=[%d] \n", __func__, rc);
                goto FAIL_SPMI_CONTROL;
        	}

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D343, REG_MASK_VALUE, MOBILELIGHT_REG_D343_OFF);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D343) rc=[%d] \n", __func__, rc);
                goto FAIL_SPMI_CONTROL;
        	}

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D34D, REG_MASK_VALUE, MOBILELIGHT_REG_D34D_OFF);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D34D) rc=[%d] \n", __func__, rc);
                goto FAIL_SPMI_CONTROL;
        	}

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D34E, REG_MASK_VALUE, MOBILELIGHT_REG_D34E_OFF);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D34E) rc=[%d] \n", __func__, rc);
                goto FAIL_SPMI_CONTROL;
        	}

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D347, REG_MASK_VALUE, MOBILELIGHT_REG_D347_OFF);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D347) rc=[%d] \n", __func__, rc);
                goto FAIL_SPMI_CONTROL;
        	}

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D346, REG_MASK_VALUE, MOBILELIGHT_REG_D346_OFF);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(MOBILELIGHT_REG_D346) rc=[%d] \n", __func__, rc);
                goto FAIL_SPMI_CONTROL;
        	}

            rc = regulator_disable(boost_boost_enable);
            if (rc != 0) {
                err_msg = "Regulator";
				DEBUG_PRINT_ERR("%s(): Failed to regulator_disable rc=[%d] \n", __func__, rc);
                goto FAIL_SPMI_CONTROL;
            }

            mobilelight_status = MOBILELIGHT_OFF;
        } else {
            DEBUG_PRINT("%s(): Mobilelight OFF skip \n", __func__);
        }
    }

    mutex_unlock(&gmobilelight->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    mutex_unlock(&gmobilelight->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
    return rc;
}
#endif
static int backlight_switch(struct light_led_data_type *led, uint8_t val)
{
    int rc=0;

    DEBUG_PRINT("%s(): start switch to [0x%02x(ON:0x80/OFF:0x00)] \n", __func__, val);

    rc = led_masked_reg_write(led, BACKLIGHT_REG_D847, REG_MASK_VALUE, BACKLIGHT_REG_D847_ON);
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(BACKLIGHT_REG_D847_ON) rc=[%d] \n", __func__, rc);
		return rc;
	}

    rc = led_masked_reg_write(led, BACKLIGHT_REG_D847, REG_MASK_VALUE, BACKLIGHT_REG_D847_OFF);
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(BACKLIGHT_REG_D847_OFF) rc=[%d] \n", __func__, rc);
		return rc;
	}

    udelay(10);
    rc = led_masked_reg_write(led, BACKLIGHT_REG_D846, REG_MASK_VALUE, val);
	if (rc != SPMI_NO_ERROR){
		DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(BACKLIGHT_REG_D846) rc=[%d] \n", __func__, rc);
		return rc;
	}

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

}

static int backlight_set(struct light_led_data_type *led)
{
    int32_t rc=0, level;
    uint8_t i;
    uint8_t uc_backlightval;
    uint32_t ul_val;
    static int32_t backlight_status = BACKLIGHT_OFF;
    int32_t batt_temp = 0;
#ifndef DISABLE_DISP_DETECT
    int32_t display_detect;
#endif  /* DISABLE_DISP_DETECT */

    DEBUG_PRINT("%s(): start backlight_status=[%d] \n", __func__, backlight_status);

    mutex_lock(&gbacklight->lock);
    DEBUG_PRINT("%s(): mutex_lock \n", __func__);

    ul_val = (led->ul_value & 0xffffffff);
    uc_backlightval  = (u8)(ul_val & BACKLIGHT_BASE_LEVEL);
    DEBUG_PRINT("%s(): backlight val=[%d] \n", __func__, uc_backlightval);

#ifndef DISABLE_DISP_DETECT
    display_detect = atomic_read(&g_display_detect);
#endif  /* DISABLE_DISP_DETECT */

    level = VALUE_TRANSFORMATION(BACKLIGHT_MAX_LEVEL, uc_backlightval, BACKLIGHT_BASE_LEVEL);

    if (backlight_status == BACKLIGHT_OFF) {
        if (level > BACKLIGHT_THRESHOLD_CURVAL) {
#ifdef OVER_THERM_PROTECT
            batt_temp = get_substrate_therm(POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM);
#else
            batt_temp = 0;
#endif
            DEBUG_PRINT("%s(): Get BatteryTemp=[%d] \n", __func__, batt_temp);

            if (batt_temp >= BACKLIGHT_THRESHOLD_TEMPVAL) {
                level = BACKLIGHT_THRESHOLD_CURVAL;
                DEBUG_PRINT_ERR("%s(): BatteryTemp is over threshold. BatteryTemp=[%d] level=[%d] \n", __func__, batt_temp, level);
            }

            if (guc_light_dm == 1) {
                level = BACKLIGHT_THRESHOLD_CURVAL;
                DEBUG_PRINT("%s(): backlight val=[%d] dm=[%d] \n", __func__, level, guc_light_dm);
            }
            else if(guc_light_dm == 2) {
                level = batt_temp;
                DEBUG_PRINT("%s(): backlight val=[%d] dm=[%d] \n", __func__, level, guc_light_dm);
            }
        }
    }

    /* program brightness control registers */
    for (i = 0; i < BACKLIGHT_NUM; i++) {
        rc = led_masked_reg_write(led, BACKLIGHT_BRIGHTNESS_SETTING_MSB(i), REG_MASK_VALUE, GET_MSB(level));
    	if (rc != SPMI_NO_ERROR){
			DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(BACKLIGHT_BRIGHTNESS_SETTING_MSB) i=[%d],rc=[%d] \n", __func__, i, rc);
            goto FAIL_SPMI_CONTROL;
    	}

        rc = led_masked_reg_write(led, BACKLIGHT_BRIGHTNESS_SETTING_LSB(i), REG_MASK_VALUE, GET_LSB(level));
    	if (rc != SPMI_NO_ERROR){
			DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(BACKLIGHT_BRIGHTNESS_SETTING_LSB) i=[%d],rc=[%d] \n", __func__, i, rc);
            goto FAIL_SPMI_CONTROL;
    	}
    }

    if (uc_backlightval == LED_COL_BLACK) {
#ifndef DISABLE_DISP_DETECT
        if (display_detect == 0) {
            rc = light_led_disp_set(LIGHT_MAIN_WLED_LED_DIS);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to light_led_disp_set(LIGHT_MAIN_WLED_LED_DIS) rc=[%d] \n", __func__, rc);
            	goto FAIL_SPMI_CONTROL;
        	}
        }
        else if (display_detect == 1) {
#endif  /* DISABLE_DISP_DETECT */
			rc = led_masked_reg_write(led, BACKLIGHT_REG_D862, REG_MASK_VALUE, 0);
			if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to BACKLIGHT_REG_D862 rc=[%d] \n", __func__, rc);
				goto FAIL_SPMI_CONTROL;
			}
			rc = led_masked_reg_write(led, BACKLIGHT_REG_D872, REG_MASK_VALUE, 0);
			if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to BACKLIGHT_REG_D872 rc=[%d] \n", __func__, rc);
				goto FAIL_SPMI_CONTROL;
			}
			rc = led_masked_reg_write(led, BACKLIGHT_REG_D882, REG_MASK_VALUE, 0);
			if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to BACKLIGHT_REG_D882 rc=[%d] \n", __func__, rc);
				goto FAIL_SPMI_CONTROL;
			}
			rc = led_masked_reg_write(led, BACKLIGHT_REG_D84F, REG_MASK_VALUE, 0x00);
			if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to BACKLIGHT_REG_D84F rc=[%d] \n", __func__, rc);
				goto FAIL_SPMI_CONTROL;
			}
			rc = led_masked_reg_write(led, BACKLIGHT_REG_D84E, REG_MASK_VALUE, 0x01);
			if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to BACKLIGHT_REG_D84E rc=[%d] \n", __func__, rc);
				goto FAIL_SPMI_CONTROL;
			}

            rc = backlight_switch(led, BACKLIGHT_REG_D846_OFF);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to backlight_switch(BACKLIGHT_REG_D846_OFF) rc=[%d] \n", __func__, rc);
            	goto FAIL_SPMI_CONTROL;
        	}

			rc = led_masked_reg_write(led, BACKLIGHT_REG_D862, REG_MASK_VALUE, BACKLIGHT_REG_D862_INIT);
			if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to BACKLIGHT_REG_D862_INIT rc=[%d] \n", __func__, rc);
				goto FAIL_SPMI_CONTROL;
			}
			rc = led_masked_reg_write(led, BACKLIGHT_REG_D872, REG_MASK_VALUE, BACKLIGHT_REG_D872_INIT);
			if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to BACKLIGHT_REG_D872_INIT2_INIT rc=[%d] \n", __func__, rc);
				goto FAIL_SPMI_CONTROL;
			}
			rc = led_masked_reg_write(led, BACKLIGHT_REG_D882, REG_MASK_VALUE, BACKLIGHT_REG_D882_INIT);
			if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to BACKLIGHT_REG_D882_INIT rc=[%d] \n", __func__, rc);
				goto FAIL_SPMI_CONTROL;
			}
			rc = led_masked_reg_write(led, BACKLIGHT_REG_D84E, REG_MASK_VALUE, BACKLIGHT_REG_D84E_INIT);
			if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to BACKLIGHT_REG_D84E rc=[%d] \n", __func__, rc);
				goto FAIL_SPMI_CONTROL;
			}
			rc = led_masked_reg_write(led, BACKLIGHT_REG_D84F, REG_MASK_VALUE, BACKLIGHT_REG_D84F_INIT);
			if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to BACKLIGHT_REG_D84F rc=[%d] \n", __func__, rc);
				goto FAIL_SPMI_CONTROL;
			}
#ifndef DISABLE_DISP_DETECT
        }
#endif  /* DISABLE_DISP_DETECT */

        backlight_status = BACKLIGHT_OFF;
    } else {
#ifndef DISABLE_DISP_DETECT
        if (display_detect == 0) {
            rc = light_led_disp_set(LIGHT_MAIN_WLED_LED_EN);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to light_led_disp_set(LIGHT_MAIN_WLED_LED_EN) rc=[%d] \n", __func__, rc);
                goto FAIL_SPMI_CONTROL;
        	}
        }
        else if(display_detect == 1) {
#endif  /* DISABLE_DISP_DETECT */
            rc = backlight_switch(led, BACKLIGHT_REG_D846_ON);
        	if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to backlight_switch(BACKLIGHT_REG_D846_ON) rc=[%d] \n", __func__, rc);
                goto FAIL_SPMI_CONTROL;
        	}
#ifndef DISABLE_DISP_DETECT
        }
        else {
            DEBUG_PRINT_ERR("%s(): No set display display_detect=[%x] \n", __func__, (int32_t)display_detect);
        }
#endif  /* DISABLE_DISP_DETECT */
        backlight_status = BACKLIGHT_ON;
    }

    mutex_unlock(&gbacklight->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    mutex_unlock(&gbacklight->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
    return rc;
}

static int qpnp_kpdbl_set(struct light_led_data_type *led)
{
	int duty_us;
	int rc;
	int i;

    DEBUG_PRINT("%s(): start brightness=0x%02x on=0x%02x\n", __func__, led->cdev.brightness,kpbl_leds_status);

    mutex_lock(&led->lock);
    DEBUG_PRINT("%s(): mutex_lock \n", __func__);

	if (led->cdev.brightness) {
		if (!led->kpdbl_cfg->pwm_cfg->blinking)
			led->kpdbl_cfg->pwm_cfg->mode =
				led->kpdbl_cfg->pwm_cfg->default_mode;
		if (!kpbl_leds_status) {
#ifdef KPDBL_USE_REGULATOR
			if( kpdbl_boost_boost_enable && 0 == boost_stat){
				boost_stat = 1;
				boost_count++;
	            rc = regulator_enable(kpdbl_boost_boost_enable);
	            if (rc != 0) {
					boost_stat = 0;
					DEBUG_PRINT_ERR("%s(): Failed to regulator_enable rc=[%d] <%d %d> [0x%p]\n", __func__, rc, boost_stat, boost_count, kpdbl_boost_boost_enable);
	                goto FAIL_SPMI_CONTROL;
				} else {
					boost_stat = 1;
					DEBUG_PRINT_INFO("%s(): Success regulator_enable <%d %d> [0x%p]\n", __func__, boost_stat, boost_count, kpdbl_boost_boost_enable);
				}
			}
#endif

			DEBUG_PRINT("%s(): KPDBL_MODULE_EN\n", __func__);
			rc = led_masked_reg_write(led, KPDBL_ENABLE(led->base),
					KPDBL_MODULE_EN_MASK, KPDBL_MODULE_EN);
			if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(KPDBL_MODULE_EN) rc=[%d] \n", __func__, rc);
				goto FAIL_SPMI_CONTROL;
			}
		}

		if (led->kpdbl_cfg->pwm_cfg->mode == PWM_MODE) {
			duty_us = (led->kpdbl_cfg->pwm_cfg->pwm_period_us *
				led->cdev.brightness) / KPDBL_MAX_LEVEL;
			DEBUG_PRINT("%s(): PWM_MOD\n", __func__);
			DEBUG_PRINT("     dus=0x%x pus=0x%x max=0x%x\n", duty_us, led->kpdbl_cfg->pwm_cfg->pwm_period_us, KPDBL_MAX_LEVEL);
			DEBUG_PRINT("%s(): pwm_config()\n", __func__);
			rc = pwm_config_us(led->kpdbl_cfg->pwm_cfg->pwm_dev,
					duty_us,
					led->kpdbl_cfg->pwm_cfg->pwm_period_us);
			if (rc < 0) {
				DEBUG_PRINT_ERR("%s(): Failed to pwm_config_us rc=[%d] \n", __func__, rc);
				goto FAIL_PWM_CONTROL;
			}
		}

		DEBUG_PRINT("%s(): pwm_enable()\n", __func__);
		rc = pwm_enable(led->kpdbl_cfg->pwm_cfg->pwm_dev);
		if (rc < 0) {
			DEBUG_PRINT_ERR("%s(): Failed to pwm_enable rc=[%d] \n", __func__, rc);
			goto FAIL_PWM_CONTROL;
		}
        if (!strcmp(led->cdev.name, KEYBACKLIGHT1_INFO)) {
        	kpbl_leds_status |= KEYBACKLIGHT1_ON;
        } else if (!strcmp(led->cdev.name, KEYBACKLIGHT2_INFO)) {
        	kpbl_leds_status |= KEYBACKLIGHT2_ON;
        } else {
        	kpbl_leds_status |= KEYBACKLIGHT3_ON;
        }
	} else {
		led->kpdbl_cfg->pwm_cfg->mode =
			led->kpdbl_cfg->pwm_cfg->default_mode;

		if (led->kpdbl_cfg->always_on) {
			DEBUG_PRINT("%s(): led->kpdbl_cfg->always_on\n", __func__);
			DEBUG_PRINT("%s(): pwm_config()\n", __func__);
			rc = pwm_config_us(led->kpdbl_cfg->pwm_cfg->pwm_dev, 0,
					led->kpdbl_cfg->pwm_cfg->pwm_period_us);
			if (rc < 0) {
				DEBUG_PRINT_ERR("%s(): Failed to pwm_config_us rc=[%d] \n", __func__, rc);
				goto FAIL_PWM_CONTROL;
			}

			DEBUG_PRINT("%s(): pwm_enable()\n", __func__);
			rc = pwm_enable(led->kpdbl_cfg->pwm_cfg->pwm_dev);
			if (rc < 0) {
				DEBUG_PRINT_ERR("%s(): Failed to pwm_enable rc=[%d] \n", __func__, rc);
				goto FAIL_PWM_CONTROL;
			}
		} else{
			DEBUG_PRINT("%s(): pwm_disable()\n", __func__);
			pwm_disable(led->kpdbl_cfg->pwm_cfg->pwm_dev);
		}
		if (kpbl_leds_status > 0){
            if (!strcmp(led->cdev.name, KEYBACKLIGHT1_INFO)) {
            	kpbl_leds_status &= ~KEYBACKLIGHT1_ON;
            } else if (!strcmp(led->cdev.name, KEYBACKLIGHT2_INFO)) {
            	kpbl_leds_status &= ~KEYBACKLIGHT2_ON;
            } else {
            	kpbl_leds_status &= ~KEYBACKLIGHT3_ON;
            }
		}
		if (!kpbl_leds_status) {
			DEBUG_PRINT("%s(): KPDBL_MODULE_DIS\n", __func__);
			rc = led_masked_reg_write(led, KPDBL_ENABLE(led->base),
					KPDBL_MODULE_EN_MASK, KPDBL_MODULE_DIS);
			if (rc != SPMI_NO_ERROR){
				DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(KPDBL_MODULE_DIS) rc=[%d] \n", __func__, rc);
			}

#ifdef KPDBL_USE_REGULATOR
			if( kpdbl_boost_boost_enable ){
	            rc = regulator_disable(kpdbl_boost_boost_enable);
				boost_count--;
	            if (rc != 0) {
					DEBUG_PRINT_ERR("%s(): Failed to regulator_disable rc=[%d] <%d %d> [0x%p]\n", __func__, rc, boost_stat, boost_count, kpdbl_boost_boost_enable);
				} else {
					boost_stat = 0;
					DEBUG_PRINT_INFO("%s(): Success regulator_disable <%d %d> [0x%p]\n", __func__, boost_stat, boost_count, kpdbl_boost_boost_enable);
				}
				if( boost_count ){
		            rc = regulator_disable(kpdbl_boost_boost_enable);
					boost_count--;
		            if (rc != 0) {
						DEBUG_PRINT_ERR("%s(): Failed to regulator_disable* rc=[%d] <%d %d>\n", __func__, rc, boost_stat, boost_count);
					} else {
						boost_stat = 0;
						DEBUG_PRINT_INFO("%s(): Success regulator_disable* <%d %d>\n", __func__, boost_stat, boost_count);
					}
				}

				for( i = 0; i < 5; i++ ){
		            rc = regulator_is_enabled(kpdbl_boost_boost_enable);
		            if (rc) {
						DEBUG_PRINT("%s(): regulator is enable* rc=[%d] %d\n", __func__, rc, i);
			            rc = regulator_disable(kpdbl_boost_boost_enable);
						boost_count--;
			            if (rc != 0) {
							DEBUG_PRINT_ERR("%s(): Failed to regulator_disable* rc=[%d] <%d %d>\n", __func__, rc, boost_stat, boost_count);
						} else {
							boost_stat = 0;
							DEBUG_PRINT_INFO("%s(): Success regulator_disable* <%d %d>\n", __func__, boost_stat, boost_count);
						}
					} else {
						boost_count = 0;
						boost_stat = 0;
						DEBUG_PRINT_INFO("%s(): regulator is disable* rc=[%d] %d\n", __func__, rc, i);
						break;
					}
				}
			}
#endif
		}
	}

	led->kpdbl_cfg->pwm_cfg->blinking = false;

    mutex_unlock(&led->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);

#if LED_DEBUG
	LED_DUMP_REGISTER(led, kpdbl_debug_regs, ARRAY_SIZE(kpdbl_debug_regs));
#endif

    DEBUG_PRINT("%s(): end on=0x%02x\n", __func__, kpbl_leds_status);
	return 0;

FAIL_SPMI_CONTROL:
    mutex_unlock(&led->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
    return rc;

FAIL_PWM_CONTROL:
    mutex_unlock(&led->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
    return rc;
}

static long leds_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    T_LEDLIGHT_IOCTL st_ioctl;
    T_LEDLIGHT_IOCTL_DM st_ioctl_dm;

    DEBUG_PRINT("%s(): start \n",__func__);
    switch (cmd) {
    case LEDLIGHT_SET_BLINK:
        DEBUG_PRINT("%s(): LEDLIGHT_SET_BLINK \n",__func__);
        ret = copy_from_user(&st_ioctl,
                    argp,
                    sizeof(T_LEDLIGHT_IOCTL));
        if (ret) {
            DEBUG_PRINT_ERR("%s(): Error leds_ioctl(cmd = LEDLIGHT_SET_BLINK) \n", __func__);
            return -EFAULT;
        }
        DEBUG_PRINT("%s(): st_ioctl data[0]=[%d] data[1]=[%d] data[2]=[%d] data[3]=[0x%08x] \n",
            __func__, st_ioctl.data[0], st_ioctl.data[1], st_ioctl.data[2], st_ioctl.data[3]);

        mutex_lock(&grgb_led->lock);
        DEBUG_PRINT("%s(): mutex_lock \n", __func__);

        grgb_led->blink_control         = st_ioctl.data[0];
        grgb_led->blink_ramp_duration   = RGB_LED_RAMP_DURATION;
        grgb_led->blink_low_pause_time  = st_ioctl.data[1];
        grgb_led->blink_high_pause_time = st_ioctl.data[2];
        grgb_led->blink_off_color       = st_ioctl.data[3];

        DEBUG_PRINT("%s(): grgb_led blink_control=[%d] blink_ramp_duration=[%d] blink_low_pause_time=[%d] blink_high_pause_time=[%d] blink_off_color=[0x%08x] \n",
            __func__, grgb_led->blink_control, grgb_led->blink_ramp_duration, grgb_led->blink_low_pause_time, grgb_led->blink_high_pause_time, grgb_led->blink_off_color);

        mutex_unlock(&grgb_led->lock);
        DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
        break;
    case LEDLIGHT_SET_TEMPERTURE_DM:
        DEBUG_PRINT("%s(): LEDLIGHT_SET_TEMPERTURE_DM \n",__func__);
        ret = copy_from_user(&st_ioctl_dm,
                    argp,
                    sizeof(T_LEDLIGHT_IOCTL_DM));
        if (ret) {
            DEBUG_PRINT_ERR("%s(): Error  st_ioctl_dm(cmd = LEDLIGHT_SET_TEMPERTURE_DM) \n", __func__);
            return -EFAULT;
        }
        mutex_lock(&grgb_led->lock);
        DEBUG_PRINT("%s(): mutex_lock \n", __func__);
        guc_light_dm = st_ioctl_dm.dm_data;
        DEBUG_PRINT("%s(): guc_light_dm=[%d] \n", __func__, guc_light_dm);
        mutex_unlock(&grgb_led->lock);
        DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
        break;
    default:
        DEBUG_PRINT("%s(): default \n", __func__);
        return -ENOTTY;
    }

    DEBUG_PRINT("%s(): end \n", __func__);

    return 0;
}

static void led_work(struct work_struct *work)
{
    int rc;
    struct light_led_data_type *led;

    DEBUG_PRINT("%s(): start \n", __func__);

    if (work == NULL) {
        DEBUG_PRINT_ERR("%s(): Error work is NULL \n", __func__);
        return;
    }

    led = container_of(work, struct light_led_data_type, work);

    if ((led != grgb_led) && (led != gbacklight) && (led != gmobilelight) &&
        (led != gkpdbl1) && (led != gkpdbl2) && (led != gkpdbl3)) {

        DEBUG_PRINT_ERR("%s(): Error in container_of led=[0x%08x] \n", __func__, (unsigned int)led);
        return;
    }

    DEBUG_PRINT("%s(): LED set info Dev=[0x%08x] DevName=[%s] Value=[0x%08x] \n", 
        __func__, (unsigned int)&led->spmi_dev->dev, led->cdev.name, led->ul_value);

    if (!strcmp(led->cdev.name, RGB_LED_INFO)) {
        rc = rgb_led_set(led);
        if (rc != SPMI_NO_ERROR)
            DEBUG_PRINT_ERR("%s(): RGB_LED set brightness failed rc=[%d] Dev=[0x%08x] \n", __func__, rc, (unsigned int)&led->spmi_dev->dev);
#ifdef CONFIG_MOBILELIGHT
    } else if (!strcmp(led->cdev.name, MOBILELIGHT_INFO)) {
        rc = mobilelight_set(led);
        if (rc != SPMI_NO_ERROR)
            DEBUG_PRINT_ERR("%s(): MOBILELIGHT set brightness failed rc=[%d] Dev=[0x%08x] \n", __func__, rc, (unsigned int)&led->spmi_dev->dev);
#endif
	} else if (!strcmp(led->cdev.name, BACKLIGHT_INFO)) {
        rc = backlight_set(led);
        if (rc != SPMI_NO_ERROR)
            DEBUG_PRINT_ERR("%s(): BACKLIGHT set brightness failed rc=[%d] Dev=[0x%08x] \n", __func__, rc, (unsigned int)&led->spmi_dev->dev);
	} else if (!strcmp(led->cdev.name, KEYBACKLIGHT1_INFO) || 
               !strcmp(led->cdev.name, KEYBACKLIGHT2_INFO) ||
               !strcmp(led->cdev.name, KEYBACKLIGHT3_INFO)) {
        rc = qpnp_kpdbl_set(led);
        if (rc != SPMI_NO_ERROR)
            DEBUG_PRINT_ERR("%s(): KEYBACKLIGHT set brightness failed rc=[%d] Dev=[0x%08x] \n", __func__, rc, (unsigned int)&led->spmi_dev->dev);
    } else {
        DEBUG_PRINT_ERR("%s(): No LED matched DevName=[%s] \n", __func__, led->cdev.name);
    }

    DEBUG_PRINT("%s(): end \n", __func__);
}

static void led_set(struct led_classdev *led_cdev,
                enum led_brightness value)
{
    struct light_led_data_type *led;

    DEBUG_PRINT("%s(): start value=[0x%08x] \n", __func__, value);

    if (led_cdev == NULL) {
        DEBUG_PRINT_ERR("%s(): Error led_cdev is NULL \n", __func__);
        return;
    }

    led = container_of(led_cdev, struct light_led_data_type, cdev);
    if ((led != grgb_led) && (led != gbacklight) && (led != gmobilelight)) {
        DEBUG_PRINT_ERR("%s(): Error in container_of led=[0x%08x] \n", __func__, (unsigned int)led);
        return;
    }

    if ((value < LED_OFF) || (value > led->cdev.max_brightness)) {
        DEBUG_PRINT_ERR("%s(): Invalid brightness value=[%d] Dev=[0x%08x] \n", __func__, value, (unsigned int)&led->spmi_dev->dev);
        return;
    }

    mutex_lock(&grgb_led->lock);
    DEBUG_PRINT("%s(): mutex_lock \n", __func__);

    led->ul_value = value;

    DEBUG_PRINT("%s(): LED set info Dev=[0x%08x] DevName=[%s] Value=[0x%08x] \n", 
        __func__, (unsigned int)&led->spmi_dev->dev, led->cdev.name, led->ul_value);

    mutex_unlock(&grgb_led->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);

    schedule_work(&led->work);

    DEBUG_PRINT("%s(): end \n", __func__);
}

static void qpnp_led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct light_led_data_type *led;

    DEBUG_PRINT("%s(): start value=[0x%08x] \n", __func__, value);

	led = container_of(led_cdev, struct light_led_data_type, cdev);
	if (value < LED_OFF) {
        DEBUG_PRINT("%s(): error\n", __func__);
		return;
	}

	if (value > led->cdev.max_brightness)
		value = led->cdev.max_brightness;

    DEBUG_PRINT("%s(): set value=[0x%08x] \n", __func__, value);
    mutex_lock(&led->lock);
	led->cdev.brightness = value;
    mutex_unlock(&led->lock);
	schedule_work(&led->work);

    DEBUG_PRINT("%s(): end \n", __func__);
}

static enum led_brightness qpnp_led_get(struct led_classdev *led_cdev)
{
	struct light_led_data_type *led;

	DEBUG_PRINT("%s(): start\n", __func__);
	led = container_of(led_cdev, struct light_led_data_type, cdev);
    DEBUG_PRINT("%s(): end brightness=0x%02x\n", __func__, led->cdev.brightness);

	return led->cdev.brightness;
}

static void __qpnp_led_work(struct light_led_data_type *led,
				enum led_brightness value)
{
	int rc;

    DEBUG_PRINT("%s(): start value=[0x%08x]\n", __func__,value);

	rc = qpnp_kpdbl_set(led);

	if (rc < 0)
		DEBUG_PRINT_ERR("%s(): Failed to qpnp_kpdbl_set rc=[%d] \n", __func__, rc);

    DEBUG_PRINT("%s(): end \n", __func__);
}

static void qpnp_led_turn_off_delayed(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct light_led_data_type *led
		= container_of(dwork, struct light_led_data_type, dwork);

	led->cdev.brightness = LED_OFF;
	qpnp_led_set(&led->cdev, led->cdev.brightness);
}

static void qpnp_led_turn_off(struct light_led_data_type *led)
{
	INIT_DELAYED_WORK(&led->dwork, qpnp_led_turn_off_delayed);
	schedule_delayed_work(&led->dwork,
		msecs_to_jiffies(led->turn_off_delay_ms));
}


static enum led_brightness led_get(struct led_classdev *led_cdev)
{
    struct light_led_data_type *led;

    DEBUG_PRINT("%s(): start \n", __func__);

    if (led_cdev == NULL) {
        DEBUG_PRINT_ERR("%s(): led_cdev is NULL \n", __func__);
        return 0;
    }

    led = container_of(led_cdev, struct light_led_data_type, cdev);
    if ((led != grgb_led) && (led != gbacklight) && (led != gmobilelight)) {
        DEBUG_PRINT_ERR("%s(): Error in container_of led=[0x%08x] \n", __func__, (unsigned int)led);
        return 0;
    }

    DEBUG_PRINT("%s(): end value=[%i] \n", __func__, led->ul_value);

    return led->ul_value;
}

static int __devinit rgb_led_init(struct light_led_data_type *led)
{
    int rc=0, i;

    DEBUG_PRINT("%s(): start \n", __func__);

    for (i = 0; i < (sizeof(rgb_led_init_reg_data)/sizeof(write_reg_led_data_init_type)); i++) {
        rc = led_masked_reg_write(led, rgb_led_init_reg_data[i].addr, REG_MASK_VALUE, rgb_led_init_reg_data[i].value);
    	if (rc != SPMI_NO_ERROR){
			DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write rc=[%d] \n", __func__, rc);
			return rc;
    	}
    }

    LED_DUMP_REGISTER(led, rgb_led_debug_regs, ARRAY_SIZE(rgb_led_debug_regs));

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;
}
#ifdef CONFIG_MOBILELIGHT
static int __devinit mobilelight_init(struct light_led_data_type *led)
{
    int rc=0, i;

    DEBUG_PRINT("%s(): start \n", __func__);

    for (i = 0; i < (sizeof(mobilelight_init_reg_data)/sizeof(write_reg_led_data_init_type)); i++) {
        rc = led_masked_reg_write(led, mobilelight_init_reg_data[i].addr, REG_MASK_VALUE, mobilelight_init_reg_data[i].value);
    	if (rc != SPMI_NO_ERROR){
			DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write rc=[%d] \n", __func__, rc);
			return rc;
    	}
    }

    LED_DUMP_REGISTER(led, mobilelight_debug_regs, ARRAY_SIZE(mobilelight_debug_regs));

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;
}
#endif
static int __devinit backlight_init(struct light_led_data_type *led)
{
    int rc=0, i;

    DEBUG_PRINT("%s(): start\n", __func__);

    for (i = 0; i < (sizeof(backlight_init_reg_data)/sizeof(write_reg_led_data_init_type)); i++) {
        rc = led_masked_reg_write(led, backlight_init_reg_data[i].addr, REG_MASK_VALUE, backlight_init_reg_data[i].value);
    	if (rc != SPMI_NO_ERROR){
			DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write rc=[%d] \n", __func__, rc);
			return rc;
    	}
    }

    LED_DUMP_REGISTER(led, backlight_debug_regs, ARRAY_SIZE(backlight_debug_regs));

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;
}


int32_t light_led_disp_set(e_light_main_wled_disp disp_status)
{
    int32_t ret=0;
#ifndef DISABLE_DISP_DETECT
    e_light_main_wled_disp status;
    
	if( !do_disp_lock ){
		DEBUG_PRINT("mutex_init() called\n");
	    mutex_init(&led_disp_lock);
		do_disp_lock = 1;
	}

    mutex_lock(&led_disp_lock);
    DEBUG_PRINT("%s(): mutex_lock \n", __func__);
    status = (e_light_main_wled_disp)atomic_read(&g_disp_status);
    DEBUG_PRINT("%s(): start status=[0x%x] disp_status=[0x%x] \n", __func__, (uint32_t)status, (uint32_t)disp_status);

    if((atomic_read(&g_display_detect)) != 0){
        mutex_unlock(&led_disp_lock);
        DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
        DEBUG_PRINT("%s(): [end]Already set g_display_detect=[%d] \n", __func__, (int32_t)atomic_read(&g_display_detect));
        return ret;
    }

    switch(disp_status) {
        case LIGHT_MAIN_WLED_LCD_EN:
        case LIGHT_MAIN_WLED_LED_EN:
            status |= disp_status;
            if(LIGHT_MAIN_WLED_EN == status) {
                ret = backlight_switch(gbacklight, BACKLIGHT_REG_D846_ON);
                atomic_set(&g_display_detect,1);
                DEBUG_PRINT("%s(): Set display detect status=[0x%x] \n", __func__, (uint32_t)status);
            }
            break;
        case LIGHT_MAIN_WLED_LCD_DIS:
            atomic_set(&g_display_detect,-1);
            DEBUG_PRINT_ERR("%s(): No set display disp_status=[0x%x] \n", __func__, (uint32_t)disp_status);
        case LIGHT_MAIN_WLED_LED_DIS:
            status &= ~(disp_status>>4);
            DEBUG_PRINT("%s(): status=[0x%x] \n", __func__, (uint32_t)status);
            break;
        default:
            break;
    }
    DEBUG_PRINT("%s(): status=[0x%x] g_display_detect=[%d] \n", __func__, (uint32_t)status, (int32_t)atomic_read(&g_display_detect));
    atomic_set(&g_disp_status,(uint32_t)status);
    mutex_unlock(&led_disp_lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
    DEBUG_PRINT("%s(): end ret=[%d] \n", __func__, ret);
#endif  /* DISABLE_DISP_DETECT */
    return ret;
}
EXPORT_SYMBOL(light_led_disp_set);


static int32_t leds_open(struct inode* inode, struct file* filp)
{
    DEBUG_PRINT("%s(): start \n", __func__);
    DEBUG_PRINT("%s(): end \n", __func__);
    return 0;
}

static int32_t leds_release(struct inode* inode, struct file* filp)
{
    DEBUG_PRINT("%s(): start \n", __func__);
    DEBUG_PRINT("%s(): end \n", __func__);
    return 0;
}

static struct file_operations leds_fops = {
    .owner        = THIS_MODULE,
    .open        = leds_open,
    .release    = leds_release,
    .unlocked_ioctl = leds_ioctl,
};

static struct miscdevice leds_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "leds-ledlight",
    .fops  = &leds_fops,
};

static int led_get_mode(const char *mode)
{
	if (strncmp(mode, "manual", strlen(mode)) == 0)
		return MANUAL_MODE;
	else if (strncmp(mode, "pwm", strlen(mode)) == 0)
		return PWM_MODE;
	else if (strncmp(mode, "lpg", strlen(mode)) == 0)
		return LPG_MODE;
	else
		return -EINVAL;
};

static int __devinit get_config_pwm(struct pwm_config_data *pwm_cfg,
				struct spmi_device *spmi_dev,
				struct device_node *node)
{
	struct property *prop;
	int rc, i;
	u32 val;
	u8 *temp_cfg;

    DEBUG_PRINT("%s(): start\n", __func__);

	rc = of_property_read_u32(node, "qcom,pwm-channel", &val);
	if (!rc)
		pwm_cfg->pwm_channel = val;
	else{
		DEBUG_PRINT_ERR("%s(): Failure reading pwm-channel, rc = %d\n", __func__, rc);
		return rc;
	}

	if (pwm_cfg->mode == PWM_MODE) {
		rc = of_property_read_u32(node, "qcom,pwm-us", &val);
		if (!rc)
			pwm_cfg->pwm_period_us = val;
		else{
			DEBUG_PRINT_ERR("%s(): Failure reading pwm-us, rc = %d\n", __func__, rc);
			return rc;
		}
	}

	pwm_cfg->use_blink =
		of_property_read_bool(node, "qcom,use-blink");

	if (pwm_cfg->mode == LPG_MODE || pwm_cfg->use_blink) {
		pwm_cfg->duty_cycles =
			devm_kzalloc(&spmi_dev->dev,
			sizeof(struct pwm_duty_cycles), GFP_KERNEL);
		if (!pwm_cfg->duty_cycles) {
			DEBUG_PRINT_ERR("%s(): Failed to devm_kzalloc for duty_cycles\n", __func__);
			rc = -ENOMEM;
			goto bad_lpg_params;
		}

		prop = of_find_property(node, "qcom,duty-pcts",
			&pwm_cfg->duty_cycles->num_duty_pcts);
		if (!prop) {
			DEBUG_PRINT_ERR("%s(): Looking up property node qcom,duty-pcts failed\n", __func__);
			rc =  -ENODEV;
			goto bad_lpg_params;
		} else if (!pwm_cfg->duty_cycles->num_duty_pcts) {
			DEBUG_PRINT_ERR("%s(): Invalid length of duty pcts\n", __func__);
			rc =  -EINVAL;
			goto bad_lpg_params;
		}

		pwm_cfg->duty_cycles->duty_pcts =
			devm_kzalloc(&spmi_dev->dev,
			sizeof(int) * PWM_LUT_MAX_SIZE,
			GFP_KERNEL);
		if (!pwm_cfg->duty_cycles->duty_pcts) {
			DEBUG_PRINT_ERR("%s(): Failed to devm_kzalloc for duty_pcts\n", __func__);
			rc = -ENOMEM;
			goto bad_lpg_params;
		}

		pwm_cfg->old_duty_pcts =
			devm_kzalloc(&spmi_dev->dev,
			sizeof(int) * PWM_LUT_MAX_SIZE,
			GFP_KERNEL);
		if (!pwm_cfg->old_duty_pcts) {
			DEBUG_PRINT_ERR("%s(): Failed to devm_kzalloc for old_duty_pcts\n", __func__);
			rc = -ENOMEM;
			goto bad_lpg_params;
		}

		temp_cfg = devm_kzalloc(&spmi_dev->dev,
				pwm_cfg->duty_cycles->num_duty_pcts *
				sizeof(u8), GFP_KERNEL);
		if (!temp_cfg) {
			DEBUG_PRINT_ERR("%s(): Failed to devm_kzalloc for temp_cfg\n", __func__);
			rc = -ENOMEM;
			goto bad_lpg_params;
		}

		memcpy(temp_cfg, prop->value,
			pwm_cfg->duty_cycles->num_duty_pcts);

		for (i = 0; i < pwm_cfg->duty_cycles->num_duty_pcts; i++)
			pwm_cfg->duty_cycles->duty_pcts[i] =
				(int) temp_cfg[i];

		rc = of_property_read_u32(node, "qcom,start-idx", &val);
		if (!rc) {
			pwm_cfg->lut_params.start_idx = val;
			pwm_cfg->duty_cycles->start_idx = val;
		} else{
			DEBUG_PRINT_ERR("%s(): Failure reading start-idx, rc = %d\n", __func__, rc);
			goto bad_lpg_params;
		}

		pwm_cfg->lut_params.lut_pause_hi = 0;
		rc = of_property_read_u32(node, "qcom,pause-hi", &val);
		if (!rc)
			pwm_cfg->lut_params.lut_pause_hi = val;
		else if (rc != -EINVAL){
			DEBUG_PRINT_ERR("%s(): Failure reading pause-hi, rc = %d\n", __func__, rc);
			goto bad_lpg_params;
		}

		pwm_cfg->lut_params.lut_pause_lo = 0;
		rc = of_property_read_u32(node, "qcom,pause-lo", &val);
		if (!rc)
			pwm_cfg->lut_params.lut_pause_lo = val;
		else if (rc != -EINVAL){
			DEBUG_PRINT_ERR("%s(): Failure reading pause-lo, rc = %d\n", __func__, rc);
			goto bad_lpg_params;
		}

		pwm_cfg->lut_params.ramp_step_ms =
				QPNP_LUT_RAMP_STEP_DEFAULT;
		rc = of_property_read_u32(node, "qcom,ramp-step-ms", &val);
		if (!rc)
			pwm_cfg->lut_params.ramp_step_ms = val;
		else if (rc != -EINVAL){
			DEBUG_PRINT_ERR("%s(): Failure reading ramp-step-ms, rc = %d\n", __func__, rc);
			goto bad_lpg_params;
		}

		pwm_cfg->lut_params.flags = QPNP_LED_PWM_FLAGS;
		rc = of_property_read_u32(node, "qcom,lut-flags", &val);
		if (!rc)
			pwm_cfg->lut_params.flags = (u8) val;
		else if (rc != -EINVAL){
			DEBUG_PRINT_ERR("%s(): Failure reading lut-flags, rc = %d\n", __func__, rc);
			goto bad_lpg_params;
		}

		pwm_cfg->lut_params.idx_len =
			pwm_cfg->duty_cycles->num_duty_pcts;
	}
    DEBUG_PRINT("%s(): end\n", __func__);

	return 0;

bad_lpg_params:
	pwm_cfg->use_blink = false;
	if (pwm_cfg->mode == PWM_MODE) {
		DEBUG_PRINT_ERR("%s():LPG parameters not set for blink mode, defaulting to PWM mode\n", __func__);
		return 0;
	}
	return rc;
};

static int __devinit get_config_kpdbl(struct light_led_data_type *led,
				struct device_node *node)
{
	int rc;
	u32 val;
	u8 led_mode;
	const char *mode;

    DEBUG_PRINT("%s(): start\n", __func__);

	led->kpdbl_cfg = devm_kzalloc(&led->spmi_dev->dev,
				sizeof(struct kpdbl_config_data), GFP_KERNEL);
	if (!led->kpdbl_cfg) {
		DEBUG_PRINT_ERR("%s(): Failed to devm_kzalloc\n", __func__);
		return -ENOMEM;
	}

	rc = of_property_read_string(node, "qcom,mode", &mode);
	if (!rc) {
		led_mode = led_get_mode(mode);
		if ((led_mode == MANUAL_MODE) || (led_mode == -EINVAL)) {
			DEBUG_PRINT_ERR("%s(): not supported mode for kpdbl\n", __func__);
			return -EINVAL;
		}
		led->kpdbl_cfg->pwm_cfg = devm_kzalloc(&led->spmi_dev->dev,
					sizeof(struct pwm_config_data),
					GFP_KERNEL);
		if (!led->kpdbl_cfg->pwm_cfg) {
			DEBUG_PRINT_ERR("%s(): Failed to devm_kzalloc\n", __func__);
			return -ENOMEM;
		}
		led->kpdbl_cfg->pwm_cfg->mode = led_mode;
		led->kpdbl_cfg->pwm_cfg->default_mode = led_mode;
	} else{
		DEBUG_PRINT_ERR("%s(): Failure reading mode, rc = %d\n", __func__, rc);
		return rc;
	}

	rc = get_config_pwm(led->kpdbl_cfg->pwm_cfg, led->spmi_dev,  node);
	if (rc < 0){
		DEBUG_PRINT_ERR("%s(): Failed to get_config_pwm rc=[%d] \n", __func__, rc);
		return rc;
	}

	rc = of_property_read_u32(node, "qcom,row-id", &val);
	if (!rc)
		led->kpdbl_cfg->row_id = val;
	else{
		DEBUG_PRINT_ERR("%s(): Failure reading row-id, rc = %d\n", __func__, rc);
		return rc;
	}

	led->kpdbl_cfg->row_src_vbst =
			of_property_read_bool(node, "qcom,row-src-vbst");

	led->kpdbl_cfg->row_src_en =
			of_property_read_bool(node, "qcom,row-src-en");

	led->kpdbl_cfg->always_on =
			of_property_read_bool(node, "qcom,always-on");

    DEBUG_PRINT("%s(): end\n", __func__);
	return 0;
}

static int pwm_init(struct pwm_config_data *pwm_cfg,
					struct spmi_device *spmi_dev,
					const char *name)
{
	int rc, start_idx, idx_len;

	DEBUG_PRINT("%s() START\n", __func__);

	if (pwm_cfg->pwm_channel != -1) {
		pwm_cfg->pwm_dev =
			pwm_request(pwm_cfg->pwm_channel, name);

		if (IS_ERR_OR_NULL(pwm_cfg->pwm_dev)) {
			DEBUG_PRINT_ERR("%s(): Failed to IS_ERR_OR_NULL Channel %d,error %ld\n", __func__, pwm_cfg->pwm_channel,PTR_ERR(pwm_cfg->pwm_dev));
			pwm_cfg->pwm_dev = NULL;
			return -ENODEV;
		}

		if (pwm_cfg->mode == LPG_MODE) {
			start_idx =
			pwm_cfg->duty_cycles->start_idx;
			idx_len =
			pwm_cfg->duty_cycles->num_duty_pcts;

			if (idx_len >= PWM_LUT_MAX_SIZE &&
					start_idx) {
				DEBUG_PRINT_ERR("%s(): Wrong LUT size or index\n", __func__);
				return -EINVAL;
			}
			if ((start_idx + idx_len) >
					PWM_LUT_MAX_SIZE) {
				DEBUG_PRINT_ERR("%s(): Exceed LUT limit\n", __func__);
				return -EINVAL;
			}
			rc = pwm_lut_config(pwm_cfg->pwm_dev,
				PM_PWM_PERIOD_MIN, /* ignored by hardware */
				pwm_cfg->duty_cycles->duty_pcts,
				pwm_cfg->lut_params);
			if (rc < 0) {
				DEBUG_PRINT_ERR("%s(): Failed to configure pwm LUT\n", __func__);
				return rc;
			}
		}
	} else {
		DEBUG_PRINT_ERR("%s(): Invalid PWM channel\n", __func__);
		return -EINVAL;
	}

	DEBUG_PRINT("%s() END\n", __func__);

	return 0;
}

static int __devinit kpdbl_init(struct light_led_data_type *led)
{
	int rc;
	u8 val;

	DEBUG_PRINT("%s() START\n", __func__);

	/* select row source - vbst or vph */
	rc = spmi_ext_register_readl(led->spmi_dev->ctrl, led->spmi_dev->sid,
				KPDBL_ROW_SRC_SEL(led->base), &val, 1);
	if (rc) {
		DEBUG_PRINT_ERR("%s(): Failed to spmi_ext_register_readl(KPDBL_ROW_SRC_SEL) rc=[%d] \n", __func__, rc);
		return rc;
	}

	if (led->kpdbl_cfg->row_src_vbst)
		val |= 1 << led->kpdbl_cfg->row_id;
	else
		val &= ~(1 << led->kpdbl_cfg->row_id);

	DEBUG_PRINT("%s() called.\n", __func__);
	DEBUG_PRINT("     addr = 0x%02x\n", KPDBL_ROW_SRC_SEL(led->base));
	DEBUG_PRINT("     val  = 0x%02x\n", val);
	rc = spmi_ext_register_writel(led->spmi_dev->ctrl, led->spmi_dev->sid,
				KPDBL_ROW_SRC_SEL(led->base), &val, 1);
	if (rc){
		DEBUG_PRINT_ERR("%s(): Failed to spmi_ext_register_writel(KPDBL_ROW_SRC_SEL) rc=[%d] \n", __func__, rc);
		return rc;
	}

	/* row source enable */
	rc = spmi_ext_register_readl(led->spmi_dev->ctrl, led->spmi_dev->sid,
				KPDBL_ROW_SRC(led->base), &val, 1);
	if (rc){
		DEBUG_PRINT_ERR("%s(): Failed to spmi_ext_register_readl(KPDBL_ROW_SRC) rc=[%d] \n", __func__, rc);
		return rc;
	}

	if (led->kpdbl_cfg->row_src_en)
		val |= KPDBL_ROW_SCAN_EN_MASK | (1 << led->kpdbl_cfg->row_id);
	else
		val &= ~(1 << led->kpdbl_cfg->row_id);

	DEBUG_PRINT("%s() called.\n", __func__);
	DEBUG_PRINT("     addr = 0x%02x\n", KPDBL_ROW_SRC(led->base));
	DEBUG_PRINT("     val  = 0x%02x\n", val);
	DEBUG_PRINT("     ROW  = 0x%02x row_id=0x%02x\n", KPDBL_ROW_SCAN_EN_MASK, led->kpdbl_cfg->row_id);
	rc = spmi_ext_register_writel(led->spmi_dev->ctrl, led->spmi_dev->sid,
		KPDBL_ROW_SRC(led->base), &val, 1);
	if (rc){
		DEBUG_PRINT_ERR("%s(): Failed to spmi_ext_register_writel(KPDBL_ROW_SRC) rc=[%d] \n", __func__, rc);
		return rc;
	}

	/* enable module */
	rc = led_masked_reg_write(led, KPDBL_ENABLE(led->base),
		KPDBL_MODULE_EN_MASK, KPDBL_MODULE_EN);
	if (rc){
		DEBUG_PRINT_ERR("%s(): Failed to led_masked_reg_write(KPDBL_MODULE_EN) rc=[%d] \n", __func__, rc);
		return rc;
	}

	DEBUG_PRINT("%s() qpnp_pwm_init()\n", __func__);
	rc = pwm_init(led->kpdbl_cfg->pwm_cfg, led->spmi_dev,
				led->cdev.name);
	if (rc) {
		DEBUG_PRINT("%s() Failed to pwm_init\n", __func__);
		return rc;
	}

#if LED_DEBUG
	/* dump kpdbl registers */
	LED_DUMP_REGISTER(led, kpdbl_debug_regs, ARRAY_SIZE(kpdbl_debug_regs));
#endif

	DEBUG_PRINT("%s() END\n", __func__);
	return 0;
}

static int __devinit led_initialize(struct light_led_data_type *led)
{
	int rc = 0;

	DEBUG_PRINT("%s(): start\n", __func__);

	rc = kpdbl_init(led);

	if (rc)
		DEBUG_PRINT("%s() KPDBL initialize failed(%d)\n", __func__, rc);

	DEBUG_PRINT("%s(): end\n", __func__);

	return rc;
}

static int __devinit qpnp_get_common_configs(struct light_led_data_type *led,
				struct device_node *node)
{
	int rc;
	u32 val;
	const char *temp_string;

	DEBUG_PRINT("%s(): start\n", __func__);

	led->cdev.default_trigger = LED_TRIGGER_DEFAULT;
	rc = of_property_read_string(node, "linux,default-trigger",
		&temp_string);
	if (!rc)
		led->cdev.default_trigger = temp_string;
	else if (rc != -EINVAL){
		DEBUG_PRINT_ERR("%s(): Failure reading default-trigger, rc = %d\n", __func__, rc);
		return rc;
	}

	led->default_on = false;
	rc = of_property_read_string(node, "qcom,default-state",
		&temp_string);
	if (!rc) {
		if (strncmp(temp_string, "on", sizeof("on")) == 0)
			led->default_on = true;
	} else if (rc != -EINVAL){
		DEBUG_PRINT_ERR("%s(): Failure reading default-state, rc = %d\n", __func__, rc);
		return rc;
	}

	led->turn_off_delay_ms = 0;
	rc = of_property_read_u32(node, "qcom,turn-off-delay-ms", &val);
	if (!rc)
		led->turn_off_delay_ms = val;
	else if (rc != -EINVAL){
		DEBUG_PRINT_ERR("%s(): Failure reading turn-off-delay-ms, rc = %d\n", __func__, rc);
		return rc;
	}

	DEBUG_PRINT("%s(): end\n", __func__);

	return 0;
}

static int __devinit led_set_max_brightness(struct light_led_data_type *led)
{

	DEBUG_PRINT("%s(): start\n", __func__);

	led->cdev.max_brightness = KPDBL_MAX_LEVEL;

	DEBUG_PRINT("%s(): end\n", __func__);

	return 0;
}

static int __devinit leds_probe_kpdbl(struct spmi_device *spmi)
{
	struct light_led_data_type *led, *led_array;
	struct resource *led_resource;
	struct device_node *node, *temp;
	int rc, i, num_leds = 0, parsed_leds = 0;
	const char *led_label;

	DEBUG_PRINT("%s(): start\n", __func__);

	node = spmi->dev.of_node;
	if (node == NULL){
		DEBUG_PRINT_ERR("%s(): spmi->dev.of_node = null \n", __func__);
		return -ENODEV;
	}

#ifdef KPDBL_USE_REGULATOR
	if( !kpdbl_boost_boost_enable ){
	kpdbl_boost_boost_enable = regulator_get(&spmi->dev, "parent");
	if (IS_ERR(kpdbl_boost_boost_enable)) {
		rc = -1;
		DEBUG_PRINT_ERR("%s(): Cannot get regulator Dev=[0x%08x] \n", __func__, (unsigned int)&spmi->dev);
		return rc;
	} else {
			DEBUG_PRINT("%s(): Success in getting regulator Dev=[0x%08x] [0x%p]\n", __func__, (unsigned int)&spmi->dev, kpdbl_boost_boost_enable);
		}
	}
#endif

	temp = NULL;
	while ((temp = of_get_next_child(node, temp)))
		num_leds++;

	if (!num_leds){
		DEBUG_PRINT_ERR("%s(): num_leds = %d \n", __func__, num_leds);
		return -ECHILD;
	}

	led_array = devm_kzalloc(&spmi->dev,
		(sizeof(struct light_led_data_type) * num_leds), GFP_KERNEL);
	if (!led_array) {
		DEBUG_PRINT_ERR("%s(): devm_kzalloc led_array = null \n", __func__);
		return -ENOMEM;
	}

	for_each_child_of_node(node, temp) {
		led = &led_array[parsed_leds];
		led->num_leds = num_leds;
		led->spmi_dev = spmi;

		led_resource = spmi_get_resource(spmi, NULL, IORESOURCE_MEM, 0);
		if (!led_resource) {
			DEBUG_PRINT_ERR("%s(): Unable to get LED base address\n", __func__);
			rc = -ENXIO;
			goto fail_id_check;
		}
		led->base = led_resource->start;
		DEBUG_PRINT("%s(): led->base = 0x%x\n", __func__, led->base);

		rc = of_property_read_string(temp, "label", &led_label);
		if (rc < 0) {
			DEBUG_PRINT_ERR("%s(): Failure reading label, rc = %d\n", __func__, rc);
			goto fail_id_check;
		}

		rc = of_property_read_string(temp, "linux,name",
			&led->cdev.name);
		if (rc < 0) {
			DEBUG_PRINT_ERR("%s(): Failure reading led name, rc = %d\n", __func__, rc);
			goto fail_id_check;
		}

		rc = of_property_read_u32(temp, "qcom,max-current",
			&led->max_current);
		if (rc < 0) {
			DEBUG_PRINT_ERR("%s(): Failure reading max_current, rc = %d\n", __func__, rc);
			goto fail_id_check;
		}

		rc = of_property_read_u32(temp, "qcom,id", &led->id);
		if (rc < 0) {
			DEBUG_PRINT_ERR("%s(): Failure reading led id, rc = %d\n", __func__, rc);
			goto fail_id_check;
		}

		rc = qpnp_get_common_configs(led, temp);
		if (rc) {
			DEBUG_PRINT_ERR("%s(): Failure reading common led configuration, rc = %d\n", __func__, rc);
			goto fail_id_check;
		}

		led->cdev.brightness_set    = qpnp_led_set;
		led->cdev.brightness_get    = qpnp_led_get;

		if (strncmp(led_label, "kpdbl", sizeof("kpdbl")) == 0) {
			kpbl_leds_status = 0;
			rc = get_config_kpdbl(led, temp);
			if (rc < 0) {
				DEBUG_PRINT_ERR("%s(): Unable to read kpdbl config data\n", __func__);
				goto fail_id_check;
			}
		} else {
			DEBUG_PRINT_ERR("%s(): No LED matching label\n", __func__);
			rc = -EINVAL;
			goto fail_id_check;
		}

		mutex_init(&led->lock);
		INIT_WORK(&led->work, led_work);

		rc =  led_initialize(led);
		if (rc < 0){
			DEBUG_PRINT_ERR("%s(): Failed to led_initialize\n", __func__);
			goto fail_id_check;
		}

		if (!strcmp(led->cdev.name, KEYBACKLIGHT1_INFO))
		{
			DEBUG_PRINT("leds_probe_kpdbl() gkpdbl1=0x%08x\n",(unsigned int)led);
			gkpdbl1 = led;
		}
		else if (!strcmp(led->cdev.name, KEYBACKLIGHT2_INFO))
		{
			DEBUG_PRINT("leds_probe_kpdbl() gkpdbl2=0x%08x\n",(unsigned int)led);
			gkpdbl2 = led;
		}
		else if (!strcmp(led->cdev.name, KEYBACKLIGHT3_INFO))
		{
			DEBUG_PRINT("leds_probe_kpdbl() gkpdbl3=0x%08x\n",(unsigned int)led);
			gkpdbl3 = led;
		}
		else
		{
			DEBUG_PRINT("leds_probe_kpdbl() gkpdbl4=0x%08x\n",(unsigned int)led);
		}

		rc = led_set_max_brightness(led);
		if (rc < 0){
			DEBUG_PRINT_ERR("%s(): Failed to led_set_max_brightness rc=[%d]\n", __func__,rc);
			goto fail_id_check;
		}

		rc = led_classdev_register(&spmi->dev, &led->cdev);
		if (rc) {
			DEBUG_PRINT_ERR("%s(): Failed to led_classdev_register led->id=[%d],rc=[%d]\n", __func__,led->id,rc);
			goto fail_id_check;
		}

		/* configure default state */
		if (led->default_on) {
			led->cdev.brightness = led->cdev.max_brightness;
			__qpnp_led_work(led, led->cdev.brightness);
			if (led->turn_off_delay_ms > 0)
				qpnp_led_turn_off(led);
		} else
			led->cdev.brightness = LED_OFF;

		parsed_leds++;
	}
	dev_set_drvdata(&spmi->dev, led_array);
	DEBUG_PRINT("%s(): end\n", __func__);
	return 0;

fail_id_check:
	for (i = 0; i < parsed_leds; i++) {
		mutex_destroy(&led_array[i].lock);
		led_classdev_unregister(&led_array[i].cdev);
	}
	return rc;
}


static int __devinit leds_probe(struct spmi_device *spmi)
{
    struct light_led_data_type *led, *led_array;
    struct resource *led_resource;
    struct device_node *node, *temp;
    int rc, num_leds = 0, parsed_leds;
    const char *led_label;

    if (spmi == NULL){
        DEBUG_PRINT_ERR("%s(): Received spmi is NULL \n", __func__);
        return -ENODEV;
    }

    DEBUG_PRINT("%s(): start spmi=[0x%08x] \n", __func__, (unsigned int)spmi);

    node = spmi->dev.of_node;
    if (node == NULL)
        return -ENODEV;

    temp = NULL;
	
    temp = of_get_next_child(node, temp);
    rc = of_property_read_string(temp, "label", &led_label);
    if (rc < 0) {
            DEBUG_PRINT_ERR("%s(): Failure reading label, rc=[%d] \n", __func__,  rc);
			return rc;
    }
    if (strncmp(led_label, "kpdbl", sizeof("kpdbl")) == 0) {
        rc = leds_probe_kpdbl(spmi);
        return rc;
    }

    while ((temp = of_get_next_child(node, temp)))
        num_leds++;
    num_leds = 1;

    led_array = devm_kzalloc(&spmi->dev, (sizeof(struct light_led_data_type) * num_leds), GFP_KERNEL);
    if (!led_array) {
        DEBUG_PRINT_ERR("%s(): Unable to allocate memory Dev=[0x%08x] \n", __func__, (unsigned int)&spmi->dev);
        return -ENOMEM;
    }

#ifndef DISABLE_DISP_DETECT
    DEBUG_PRINT("%s(): mutex_init() %d \n", __func__, do_disp_lock);
	if( !do_disp_lock ){
		DEBUG_PRINT("mutex_init() called\n");
	    mutex_init(&led_disp_lock);
		do_disp_lock = 1;
	}
#endif  /* DISABLE_DISP_DETECT */

    for (parsed_leds=0; parsed_leds < num_leds; parsed_leds++) {

        led = &led_array[parsed_leds];
        led->num_leds = num_leds;
        led->spmi_dev = spmi;

        temp = of_get_next_child(node, temp);

        led_resource = spmi_get_resource(spmi, NULL, IORESOURCE_MEM, 0);
        if (!led_resource) {
            DEBUG_PRINT_ERR("%s(): Unable to get LED base address Dev=[0x%08x] \n", __func__, (unsigned int)&spmi->dev);
            rc = -ENXIO;
			return rc;
        }

        rc = of_property_read_string(temp, "label", &led_label);
        if (rc < 0) {
            DEBUG_PRINT_ERR("%s(): Failure reading label, Dev=[0x%08x] rc=[%d] \n", __func__, (unsigned int)&led->spmi_dev->dev, rc);
			return rc;
        }

        mutex_init(&led->lock);
        INIT_WORK(&led->work, led_work);

        if (strcmp(led_label, LABEL_RGB_LED) == 0) {
            DEBUG_PRINT("%s(): Now start RGB_LED probe \n", __func__);

            led->cdev.name   = RGB_LED_INFO;
            led->cdev.max_brightness = (unsigned int)RGB_LED_MAX_BRIGHT_VAL;

            rc = rgb_led_init(led);
            if (rc != SPMI_NO_ERROR) {
                DEBUG_PRINT_ERR("%s(): RGB_LED initialize failed Dev=[0x%08x] rc=[%d] \n", __func__, (unsigned int)&led->spmi_dev->dev, rc);
				return rc;
            }

            grgb_led = led;
            misc_register(&leds_device);
#ifdef CONFIG_MOBILELIGHT
        } else if (strcmp(led_label, LABEL_MOBILELIGHT) == 0) {

        	DEBUG_PRINT("%s(): Now start MOBILELIGHT probe \n", __func__);

            led->cdev.name   = MOBILELIGHT_INFO;
            led->cdev.max_brightness = (unsigned int)MOBILELIGHT_MAX_BRIGHT_VAL;

            rc = mobilelight_init(led);
            if (rc != SPMI_NO_ERROR) {
                DEBUG_PRINT_ERR("%s(): MOBILELIGHT initialize failed Dev=[0x%08x] rc=[%d] \n", __func__, (unsigned int)&led->spmi_dev->dev, rc);
				return rc;
            }

            gmobilelight = led;

            boost_boost_enable = regulator_get(&spmi->dev, "parent");
            if (IS_ERR(boost_boost_enable)) {
                rc = -1;
                DEBUG_PRINT_ERR("%s(): Cannot get regulator Dev=[0x%08x] \n", __func__, (unsigned int)&spmi->dev);
				return rc;
            } else {
                DEBUG_PRINT("%s(): Success in getting regulator Dev=[0x%08x] \n", __func__, (unsigned int)&spmi->dev);
            }

            hrtimer_init(&mobilelight_wdt, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
            mobilelight_wdt.function = mobilelight_wdt_reset;
#endif            
        } else if (strcmp(led_label, LABEL_BACKLIGHT) == 0) {
            DEBUG_PRINT("%s(): Now start BACKLIGHT probe \n", __func__);

            led->cdev.name   = BACKLIGHT_INFO;
            led->cdev.max_brightness = (unsigned int)BACKLIGHT_MAX_BRIGHT_VAL;

            rc = backlight_init(led);
            if (rc != SPMI_NO_ERROR) {
                DEBUG_PRINT_ERR("%s(): BACKLIGHT initialize failed Dev=[0x%08x] rc=[%d] \n", __func__, (unsigned int)&led->spmi_dev->dev, rc);
				return rc;
            }

            gbacklight = led;

        } else {
            DEBUG_PRINT_ERR("%s(): No LED matched label=[%s] \n", __func__, led_label);
            rc = -EINVAL;
			return rc;
        }

        led->cdev.brightness_set    = led_set;
        led->cdev.brightness_get    = led_get;
        led->cdev.brightness        = LED_COL_BLACK;
        led->cdev.flags             = 0;

        rc = led_classdev_register(&led->spmi_dev->dev, &led->cdev);
        if (rc) {
            DEBUG_PRINT_ERR("%s(): Unable to register led name=[%s] Dev=[0x%08x] rc=[%d] \n", __func__, led_label, (unsigned int)&spmi->dev, rc);
			return rc;
        }

        led->blink_control         = NO_BLINK_REQUEST; 
        led->blink_ramp_duration   = 0;
        led->blink_low_pause_time  = 0;
        led->blink_high_pause_time = 0;
        led->blink_off_color       = LED_COL_BLACK;
    }
    dev_set_drvdata(&spmi->dev, led_array);

    DEBUG_PRINT("%s(): end \n",__func__);

    return 0;
}

static int __devexit qpnp_leds_remove(struct spmi_device *spmi)
{
    DEBUG_PRINT("%s(): start \n", __func__);
    DEBUG_PRINT("%s(): end \n", __func__);
    
    return 0;
}
#ifdef CONFIG_OF
static struct of_device_id spmi_match_table[] = {
	{ .compatible = "qcom,leds-qpnp",},
	{ },
};
#else
#define spmi_match_table NULL
#endif

static struct spmi_driver qpnp_leds_driver = {
    .driver        = {
        .name    = "qcom,leds-qpnp",
        .of_match_table = spmi_match_table,
    },
    .probe        = leds_probe,
    .remove        = __devexit_p(qpnp_leds_remove),
};

static int __init qpnp_led_init(void)
{
    return spmi_driver_register(&qpnp_leds_driver);
}
module_init(qpnp_led_init);

static void __exit qpnp_led_exit(void)
{
    spmi_driver_unregister(&qpnp_leds_driver);
}
module_exit(qpnp_led_exit);

MODULE_DESCRIPTION("QPNP LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds-qpnp");

