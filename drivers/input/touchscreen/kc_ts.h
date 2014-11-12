/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 *
 * drivers/input/touchscreen/kc_ts.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __LINUX_KC_TS_H
#define __LINUX_KC_TS_H

#include <linux/types.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/gpio.h>
#ifdef CONFIG_KC_TOUCH_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#else
#include <linux/i2c.h>
#endif /* CONFIG_KC_TOUCH_FB */

#ifdef CONFIG_KC_TOUCH_EARLY_SUSPEND
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define MXT_SUSPEND_LEVEL 1
#endif
#include <linux/cdev.h>
#include <linux/pm.h>
#ifdef CONFIG_KC_TOUCH_REGULATOR
#include <linux/regulator/consumer.h>
#endif

#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT_KC)
#include <linux/i2c/atmel_mxt_kc.h>
#elif defined(CONFIG_TOUCHSCREEN_PIXART_KC)
#include <linux/i2c/pixart.h>
#endif

#define KC_TS_INIT_RETRY_NUM	2
#define KC_TS_NAME				"kc touchscreen"

/* Touchscreen absolute values */
#define KC_TS_AREA_MAX		2000
#define KC_TS_FORCE_MAX		5000
#define KC_TS_MAX_FINGER	10
#define ESD_POLLING_TIME	5000	/* 5 seconds */

#define KC_TS_ACTIVE_LOAD_UA	30000
#define KC_TS_LPM_LOAD_UA		100

#define KC_TS_DEV_DBG(fmt, arg...)		if(ts_log_level & 0x01) pr_notice(fmt, ## arg)
#define KC_TS_DEV_INFO(fmt, arg...)		if(ts_log_level & 0x02) pr_notice(fmt, ## arg)
#define KC_TS_DEV_TOUCH(fmt, arg...)	if(ts_log_level & 0x04) pr_notice(fmt, ## arg)
#define KC_TS_DEV_I2C(fmt, arg...)		if(ts_log_level & 0x08) pr_notice(fmt, ## arg)

#define TS_EVENT_ENABLE		0
#define TS_NV_MAX_SIZE		1024
#define TS_NUM_OF_NV		3

#define TS_ERR_CHARGE			0x10
#define TS_ERR_DISCONNECTION	0x20
#define TS_ERR_FW_DL			0x40

#if defined(CONFIG_TOUCHSCREEN_PIXART_KC)
#define TS_VER_SIZE			2
#define TS_END_CODE			3
#elif defined(CONFIG_TOUCHSCREEN_ATMEL_MXT_KC)
#define TS_VER_SIZE			2
#define TS_END_CODE			0
#endif

enum ts_status_type {
	TS_STATUS_DEEP_SLEEP = 0,
	TS_STATUS_SLEEP,
	TS_STATUS_PRE_ACTIVE,
	TS_STATUS_ACTIVE,
};

enum ts_sequence_type {
	TS_SEQ_PROBE_START = 0,
#ifdef CONFIG_KC_TOUCH_REGULATOR
	TS_SEQ_PROBE_REGULATOR,
#endif
	TS_SEQ_PROBE_SETTING,
	TS_SEQ_PROBE_IC_NONE,
	TS_SEQ_PROBE_INITIALIZATION,
	TS_SEQ_PROBE_BOOTLOADER_MODE,
	TS_SEQ_PROBE_INTERRUPT_REGISTER,
	TS_SEQ_PROBE_CREATE_QUEUE,
	TS_SEQ_PROBE_START_QUEUE,
	TS_SEQ_PROBE_CREATE_SYSFS,
	TS_SEQ_PROBE_CREATE_SUSPEND,
	TS_SEQ_PROBE_END,
};

enum ts_nv_type {
	TS_CHARGE_C_NV = 0,
	TS_CHARGE_A_S1_NV,
	TS_CHARGE_A_S2_NV,
	TS_DISCHARGE_NV,
	TS_WIRELESS_NV,
	TS_INITIAL_VALUE_NV,
	TS_EXTENDED_NV,
	TS_NV_MAX,
};

enum ts_config_type {
	TS_CHARGE_CABLE = 0,
	TS_CHARGE_A_S1,
	TS_CHARGE_A_S2,
	TS_DISCHARGE,
	TS_WIRELESS,
	TS_INITIAL,
	TS_EXTENDED,
	TS_CONFIG_MAX,
};

enum kc_ts_log_type {
	KC_TS_LOG_MSG,
	KC_TS_LOG_CONF_STAT,
	KC_TS_LOG_RESTART,
	KC_TS_LOG_RESET_STATUS,
	KC_TS_LOG_ESD,
	KC_TS_LOG_DAEMON,
	KC_TS_LOG_SUSPEND,
	KC_TS_LOG_RESUME,
	KC_TS_LOG_CFGERR,
	KC_TS_LOG_RPTEN,
	KC_TS_LOG_IC_RESET,
	KC_TS_LOG_REPORT,
	KC_TS_LOG_INTERRUPT_START,
	KC_TS_LOG_INTERRUPT_END,
	KC_TS_LOG_READ_MESSAGE,
	KC_TS_LOG_MAX,
};

struct ts_diag_event {
	int x;
	int y;
	int width;
};

struct ts_diag_type {
	struct ts_diag_event ts[KC_TS_MAX_FINGER];
	int diag_count;
};

struct ts_nv_data {
	size_t size;
	char *data;
	enum ts_nv_type nv_type;
};

struct ts_config_nv {
	size_t size;
	u16 ver;
	u8 *data;
};
struct ts_log_data {
	int flag;
	int data;
};

#pragma pack(1)
struct kc_ts_finger {
	uint8_t id;
	uint16_t x;
	uint16_t y;
	uint16_t pressure;
	uint16_t area;
};

#ifdef CONFIG_TOUCHSCREEN_PIXART_KC
struct pix_slot {
	uint8_t		id;
	bool		tool_finger;
};

struct kc_ts_pix_data {
	uint8_t					status;
	uint8_t					total_touch;
	struct kc_ts_finger		point_data[KC_TS_MAX_FINGER];
	struct pix_slot			slot[KC_TS_MAX_FINGER];
};
#endif
#pragma pack()

struct ts_last_coordinate {
	uint16_t coor_x;
	uint16_t coor_y;
	int coor_num;
};

struct kc_ts_data {
	const struct kc_ts_operations		*tops;
	const struct kc_ts_platform_data	*pdata;
	struct kc_ts_vendor_data			*vdata;
	struct device						*dev;
	struct input_dev					*input_dev;
	struct workqueue_struct				*wq;
	struct workqueue_struct				*resume_wq;
#if defined(CONFIG_TOUCHSCREEN_PIXART_KC)
	struct kc_ts_pix_data				*pix_data;
#elif defined(CONFIG_TOUCHSCREEN_ATMEL_MXT_KC)
	struct kc_ts_finger					finger[KC_TS_MAX_FINGER];
#endif
	int									irq;
	unsigned int						max_x;
	unsigned int						max_y;
#if defined(CONFIG_KC_TOUCH_FB)
	struct notifier_block				fb_notif;
#elif defined(CONFIG_KC_TOUCH_EARLY_SUSPEND)
	struct early_suspend				early_suspend;
#endif
	struct mutex						lock;
	struct delayed_work					esdwork;
	struct work_struct					resume_work;
	struct cdev							device_cdev;
	int									device_major;
	struct class						*device_class;
	struct completion					init_done;
	bool								is_suspended;
	enum ts_sequence_type				ts_sequence;
	bool								is_enable;
	bool								is_set;
	enum ts_config_type					config_status;
	enum ts_config_type					config_status_last;
	struct ts_config_nv					config_nv[TS_CONFIG_MAX];
#ifdef CONFIG_KC_TOUCH_REGULATOR
	struct regulator					*reg_touch;
#endif
};

struct kc_ts_operations {
	u16 bustype;
#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT_KC)
	int (*multi_read)(struct kc_ts_data *ts, u16 size, u16 addr, u8 *dp);
#elif defined(CONFIG_TOUCHSCREEN_PIXART_KC)
	int (*read)(struct i2c_client *client, uint8_t readAddress, uint8_t *val);
	int (*multi_read)(struct i2c_client *client, uint8_t readAddress, uint8_t *pdata, uint8_t len);
#endif
	int (*write)(struct i2c_client *client, uint8_t reg, uint8_t val);
#ifdef FEATURE_TOUCH_TEST
	void (*write_log)(struct kc_ts_data *ts, enum kc_ts_log_type type, void *arg);
#endif
	int (*interrupt)(struct kc_ts_data *ts);
	int (*power_off)(struct kc_ts_data *ts);
	int (*power_on)(struct kc_ts_data *ts);
	void (*clear)(struct kc_ts_data *ts);
	int (*input_open)(struct kc_ts_data *ts);
	void (*input_close)(struct kc_ts_data *ts);
	int (*esd_proc)(struct kc_ts_data *ts);
	long (*ioctl)(struct kc_ts_data *ts, unsigned int cmd, unsigned long arg);
	long (*reference)(struct kc_ts_data *ts, u8 *val);
	int (*reset)(struct kc_ts_data *ts);
	int (*initialize)(struct kc_ts_data *ts, int force_dl);
	void (*resolution)(struct kc_ts_data *ts);
	int (*check_reg_nv)(struct kc_ts_data *ts);
	int (*backup_nv)(struct kc_ts_data *ts);
	void (*enable)(struct kc_ts_data *ts);
	void (*disable)(struct kc_ts_data *ts);
};

struct kc_ts_platform_data {
	const u8		*config;
	size_t			config_length;
	unsigned int	x_size;
	unsigned int	y_size;
	int				irq_gpio;
	int				reset_gpio;
	unsigned char	orient;
	unsigned long	irqflags;

	int (*init_hw) (void);
	int (*reset_hw) (void);
	int (*shutdown) (void);
};

extern struct ts_diag_type *diag_data;
extern struct mutex diag_lock;
extern struct mutex file_lock;
extern unsigned int ts_event_control;
extern unsigned int ts_log_level;
extern unsigned int ts_log_file_enable;
extern unsigned int ts_esd_recovery;
extern unsigned int ts_config_switching;
extern unsigned int ts_error_status;

int kc_ts_probe(struct kc_ts_data *ts);
void kc_ts_remove(struct kc_ts_data *ts);
void kc_ts_shutdown(struct kc_ts_data *ts);
int ts_ctrl_init(struct kc_ts_data *ts, const struct file_operations *fops);
int ts_ctrl_exit(struct kc_ts_data *ts);
void kc_ts_esd_work(struct work_struct *work);
#if defined(CONFIG_KC_TOUCH_FB)
int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#elif defined(CONFIG_KC_TOUCH_EARLY_SUSPEND)
void kc_ts_early_suspend(struct early_suspend *h);
void kc_ts_late_resume(struct early_suspend *h);
#endif
irqreturn_t kc_ts_interrupt(int irq, void *dev_id);

#define IOC_MAGIC 't'
#define IOCTL_SET_CONF_STAT _IOW(IOC_MAGIC, 1, enum ts_config_type)
#define IOCTL_SET_LOG _IOW(IOC_MAGIC, 3, struct ts_log_data)
#define IOCTL_SET_NV _IO(IOC_MAGIC, 8)
#define IOCTL_CHECK_FW _IOW(IOC_MAGIC, 11, struct ts_nv_data)
#define IOCTL_GET_INFO _IOW(IOC_MAGIC, 12, unsigned char)
#define IOCTL_LOAD_NV _IOW(IOC_MAGIC, 17, struct ts_nv_data)

#define IOCTL_DIAG_START _IO(IOC_MAGIC, 0xA1)
#define IOCTL_MULTI_GET _IOR(IOC_MAGIC, 0xA2, struct ts_diag_type)
#define IOCTL_COODINATE_GET _IOR(IOC_MAGIC, 0xA3, struct ts_diag_type)
#define IOCTL_DIAG_END _IO(IOC_MAGIC, 0xA4)
#define IOCTL_DIAG_LOG_LEVEL _IOW(IOC_MAGIC, 0xA5, unsigned char)
#define IOCTL_DIAG_EVENT_CTRL _IOW(IOC_MAGIC, 0xA6, unsigned char)
#define IOCTL_DIAG_RESET_HW _IO(IOC_MAGIC, 0xA7)
#define IOCTL_GET_GOLDEN_REFERENCE _IOWR(IOC_MAGIC, 0xA8, unsigned char)
#define IOCTL_DIAG_GET_C_REFERENCE _IOWR(IOC_MAGIC, 0xA9, unsigned char)
#define IOCTL_DIAG_GET_DELTA _IOR(IOC_MAGIC, 0xA0, unsigned char)
#define IOCTL_DIAG_SUSPEND_RESUME _IOW(IOC_MAGIC, 0xAA, unsigned char)

#endif /* __LINUX_KC_TS_H */
